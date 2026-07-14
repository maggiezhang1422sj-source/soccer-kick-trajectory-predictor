#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <math.h>
#include <stdarg.h>
#include "ICM_20948.h"  // Install "SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library"

// ---------- User configuration ----------
const char* WIFI_SSID = "ATTz42RXr5";
const char* WIFI_PASSWORD = "t8z%#dhu+2v8";
const char* MAC_IP = "192.168.1.237";
const uint16_t MAC_UDP_PORT = 4210;
const uint16_t MAC_TCP_PORT = 4210;
const bool SEND_TCP_DIRECT = true;  // TCP is reliable but requires MAC_IP to be current.
const bool SEND_UDP_CHUNKS_FALLBACK = true;  // If TCP connect fails, fall back to UDP broadcast chunks.
const bool SEND_UDP_DIRECT = false;  // Set true only when MAC_IP is confirmed current.
const bool SEND_UDP_BROADCAST = false;  // 255.255.255.255 can duplicate packets; subnet broadcast is preferred.
const bool SEND_UDP_SUBNET_BROADCAST = true;  // Also send to your LAN broadcast address, e.g. 192.168.1.255.
const bool SEND_UDP_HEARTBEAT = true;  // Sends a tiny packet so the Mac can prove UDP is arriving.

// Left-foot FSR layout, sent to Mac as fsr0..fsr3:
// GPIO32/fsr0 = outer upper FSR, farther from big toe and closer to ankle.
// GPIO33/fsr1 = inner lower FSR.
// GPIO34/fsr2 = outer lower FSR.
// GPIO35/fsr3 = inner upper FSR.
const int FSR_PINS[4] = {32, 33, 34, 35};

// Tune this after calibration. It is the raw ADC value measured when an FSR has
// about 1 kg of pressure in your voltage-divider circuit.
const int FSR_1KG_ADC_THRESHOLD = 1800;
const int FSR_TRIGGER_DELTA_THRESHOLD = 850;       // Per-channel ADC delta above idle baseline.
const int FSR_TRIGGER_SUM_DELTA_THRESHOLD = 1400;  // Sum of all 4 channel deltas before a kick is accepted.
const int FSR_TRIGGER_CONFIRM_READS = 5;           // Consecutive loop reads required before sending.
const int FSR_BASELINE_SAMPLES = 80;              // Keep FSRs untouched during boot calibration.

// ICM-20948 SPI. ESP32 default VSPI pins are SCK=18, MISO=19, MOSI=23.
const int ICM_SPI_SCK_PIN = 18;
const int ICM_SPI_MISO_PIN = 19;
const int ICM_SPI_MOSI_PIN = 23;
const int ICM_SPI_CS_PIN = 5;

const float MOTION_SPEED_THRESHOLD_CM_S = 10.0f;
const float MOTION_ACCEL_THRESHOLD_M_S2 = 1.20f;
const unsigned long COOLDOWN_MS = 3500;
const unsigned long IMPACT_COINCIDENCE_MS = 100;   // Kept for future FSR+ICM mode; current trigger is FSR-only.
const bool REQUIRE_ICM_FOR_KICK_TRIGGER = false;   // Debug mode: false means FSR alone sends UDP.
const uint32_t SENSOR_SAMPLE_PERIOD_US = 200;      // 5 kHz history buffer target.
const int SENSOR_SAMPLE_HZ = 5000;
const int SENSOR_BUFFER_SIZE = 1000;               // About 200 ms of pre-impact data.
const int SENSOR_CHUNK_SAMPLES = 40;               // Keep each UDP packet below Wi-Fi MTU.
const int UDP_CHUNK_REPEAT_COUNT = 2;              // Repeat chunks so one dropped UDP packet does not block the shot.
const int UDP_PAYLOAD_BYTES = 16000;
const int ACCEL_QUANT_SCALE = 100;                 // m/s^2 stored as centi m/s^2.
const int GYRO_QUANT_SCALE = 10;                   // deg/s stored as deci deg/s.
// ---------------------------------------

WiFiUDP udp;
WiFiClient tcpClient;
ICM_20948_SPI imu;

bool imuReady = false;
unsigned long lastKickMs = 0;
unsigned long lastImuMicros = 0;
unsigned long lastHeartbeatMs = 0;

float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 9.81f;
float velocityX = 0.0f;
float velocityY = 0.0f;
float velocityZ = 0.0f;
float lastMotionSpeedCmS = 0.0f;
bool gravityInitialized = false;
int lastFsrRaw[4] = {0, 0, 0, 0};
int fsrBaselineRaw[4] = {0, 0, 0, 0};
int lastFsrDeltaRaw[4] = {0, 0, 0, 0};
int lastFsrMaxRaw = 0;
int lastFsrMaxDeltaRaw = 0;
int lastFsrSumDeltaRaw = 0;
int fsrTriggerConfirmCount = 0;
float lastAccelMps2[3] = {0.0f, 0.0f, 9.81f};
float lastGyroDps[3] = {0.0f, 0.0f, 0.0f};
float lastLinearAccelMps2 = 0.0f;
uint32_t lastSensorSampleUs = 0;
unsigned long lastPressureTriggerMs = 0;
unsigned long lastMotionTriggerMs = 0;

struct SensorSample {
  uint32_t tUs;
  uint16_t fsr[4];
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

SensorSample sensorBuffer[SENSOR_BUFFER_SIZE];
int sensorWriteIndex = 0;
bool sensorBufferFilled = false;
uint16_t kickSequenceId = 1;

void sendUdpPayload(const uint8_t* payload, size_t length) {
  if (SEND_UDP_DIRECT) {
    udp.beginPacket(MAC_IP, MAC_UDP_PORT);
    udp.write(payload, length);
    udp.endPacket();
  }

  if (SEND_UDP_SUBNET_BROADCAST && WiFi.status() == WL_CONNECTED) {
    IPAddress localIp = WiFi.localIP();
    IPAddress subnet = WiFi.subnetMask();
    IPAddress broadcastIp(
      localIp[0] | (uint8_t)~subnet[0],
      localIp[1] | (uint8_t)~subnet[1],
      localIp[2] | (uint8_t)~subnet[2],
      localIp[3] | (uint8_t)~subnet[3]
    );
    udp.beginPacket(broadcastIp, MAC_UDP_PORT);
    udp.write(payload, length);
    udp.endPacket();
  }

  if (SEND_UDP_BROADCAST) {
    IPAddress broadcastIp(255, 255, 255, 255);
    udp.beginPacket(broadcastIp, MAC_UDP_PORT);
    udp.write(payload, length);
    udp.endPacket();
  }
}

bool ensureTcpConnected() {
  if (!SEND_TCP_DIRECT) {
    return false;
  }
  if (tcpClient.connected()) {
    return true;
  }

  tcpClient.stop();
  Serial.print("Connecting TCP to ");
  Serial.print(MAC_IP);
  Serial.print(":");
  Serial.print(MAC_TCP_PORT);
  Serial.print(" ... ");
  if (!tcpClient.connect(MAC_IP, MAC_TCP_PORT)) {
    Serial.println("failed");
    return false;
  }
  tcpClient.setNoDelay(true);
  Serial.println("connected");
  return true;
}

bool sendTcpPayload(const uint8_t* payload, size_t length) {
  if (length == 0 || length > 65535 || !ensureTcpConnected()) {
    return false;
  }

  uint8_t lengthPrefix[2] = {
    (uint8_t)(length & 0xFF),
    (uint8_t)((length >> 8) & 0xFF)
  };
  size_t headerWritten = tcpClient.write(lengthPrefix, sizeof(lengthPrefix));
  size_t payloadWritten = tcpClient.write(payload, length);
  if (headerWritten != sizeof(lengthPrefix) || payloadWritten != length) {
    Serial.println("TCP write failed; closing connection.");
    tcpClient.stop();
    return false;
  }
  return true;
}

void sendUdpHeartbeat() {
  if (!SEND_UDP_HEARTBEAT || millis() - lastHeartbeatMs < 2000) {
    return;
  }

  const char heartbeat[] = "KICK_HEARTBEAT";
  sendUdpPayload((const uint8_t*)heartbeat, sizeof(heartbeat) - 1);
  Serial.println("UDP heartbeat sent");
  lastHeartbeatMs = millis();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(350);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP32 subnet: ");
  Serial.println(WiFi.subnetMask());
}

void setupImu() {
  pinMode(ICM_SPI_CS_PIN, OUTPUT);
  digitalWrite(ICM_SPI_CS_PIN, HIGH);
  SPI.begin(ICM_SPI_SCK_PIN, ICM_SPI_MISO_PIN, ICM_SPI_MOSI_PIN, ICM_SPI_CS_PIN);
  delay(100);

  for (int attempt = 0; attempt < 3; attempt++) {
    Serial.print("Trying ICM-20948 SPI on CS GPIO");
    Serial.print(ICM_SPI_CS_PIN);
    Serial.print(" attempt ");
    Serial.println(attempt + 1);

    imu.begin(ICM_SPI_CS_PIN, SPI);
    if (imu.status == ICM_20948_Stat_Ok) {
      imuReady = true;
      lastImuMicros = micros();
      Serial.println("ICM-20948 SPI ready.");
      return;
    }

    Serial.print("ICM-20948 SPI init failed: ");
    Serial.println(imu.statusString());
    delay(250);
  }

  imuReady = false;
  Serial.println("ICM-20948 not detected over SPI. FSR triggers will still work.");
}

void calibrateFsrBaseline() {
  long sums[4] = {0, 0, 0, 0};

  Serial.println("Calibrating FSR idle baseline. Do not press FSRs...");
  delay(500);
  for (int sample = 0; sample < FSR_BASELINE_SAMPLES; sample++) {
    for (int i = 0; i < 4; i++) {
      sums[i] += analogRead(FSR_PINS[i]);
    }
    delay(5);
  }

  Serial.print("FSR baseline raw=[");
  for (int i = 0; i < 4; i++) {
    fsrBaselineRaw[i] = (int)(sums[i] / FSR_BASELINE_SAMPLES);
    Serial.print(fsrBaselineRaw[i]);
    if (i < 3) {
      Serial.print(",");
    }
  }
  Serial.println("]");
  Serial.print("FSR trigger delta threshold=");
  Serial.println(FSR_TRIGGER_DELTA_THRESHOLD);
  Serial.print("FSR trigger sum threshold=");
  Serial.println(FSR_TRIGGER_SUM_DELTA_THRESHOLD);
  Serial.print("FSR trigger confirm reads=");
  Serial.println(FSR_TRIGGER_CONFIRM_READS);
}

bool anyFsrPressureTriggered() {
  lastFsrMaxRaw = 0;
  lastFsrMaxDeltaRaw = 0;
  lastFsrSumDeltaRaw = 0;
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(FSR_PINS[i]);
    int delta = abs(raw - fsrBaselineRaw[i]);
    lastFsrRaw[i] = raw;
    lastFsrDeltaRaw[i] = delta;
    lastFsrSumDeltaRaw += delta;
    if (raw > lastFsrMaxRaw) {
      lastFsrMaxRaw = raw;
    }
    if (delta > lastFsrMaxDeltaRaw) {
      lastFsrMaxDeltaRaw = delta;
    }
  }

  bool overThreshold =
    lastFsrMaxDeltaRaw >= FSR_TRIGGER_DELTA_THRESHOLD &&
    lastFsrSumDeltaRaw >= FSR_TRIGGER_SUM_DELTA_THRESHOLD;

  if (overThreshold) {
    fsrTriggerConfirmCount++;
  } else {
    fsrTriggerConfirmCount = 0;
  }

  return fsrTriggerConfirmCount >= FSR_TRIGGER_CONFIRM_READS;
}

bool motionTriggeredByIcm() {
  if (!imuReady || !imu.dataReady()) {
    return false;
  }

  imu.getAGMT();
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastImuMicros) / 1000000.0f;
  lastImuMicros = nowMicros;
  if (dt <= 0.0f || dt > 0.20f) {
    return false;
  }

  // SparkFun library returns accelerometer readings in milli-g.
  float ax = imu.accX() * 0.00980665f;
  float ay = imu.accY() * 0.00980665f;
  float az = imu.accZ() * 0.00980665f;
  lastAccelMps2[0] = ax;
  lastAccelMps2[1] = ay;
  lastAccelMps2[2] = az;
  lastGyroDps[0] = imu.gyrX();
  lastGyroDps[1] = imu.gyrY();
  lastGyroDps[2] = imu.gyrZ();

  // Estimate gravity with a low-pass filter, then integrate linear acceleration.
  if (!gravityInitialized) {
    gravityX = ax;
    gravityY = ay;
    gravityZ = az;
    gravityInitialized = true;
    return false;
  }

  const float gravityAlpha = 0.035f;
  gravityX = gravityX * (1.0f - gravityAlpha) + ax * gravityAlpha;
  gravityY = gravityY * (1.0f - gravityAlpha) + ay * gravityAlpha;
  gravityZ = gravityZ * (1.0f - gravityAlpha) + az * gravityAlpha;

  float linX = ax - gravityX;
  float linY = ay - gravityY;
  float linZ = az - gravityZ;
  float linearAccel = sqrt(linX * linX + linY * linY + linZ * linZ);
  lastLinearAccelMps2 = linearAccel;

  if (linearAccel < 0.15f) {
    velocityX *= 0.82f;
    velocityY *= 0.82f;
    velocityZ *= 0.82f;
  } else {
    velocityX += linX * dt;
    velocityY += linY * dt;
    velocityZ += linZ * dt;
    velocityX *= 0.94f;
    velocityY *= 0.94f;
    velocityZ *= 0.94f;
  }

  float speedMps = sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);
  lastMotionSpeedCmS = speedMps * 100.0f;

  if (lastMotionSpeedCmS >= MOTION_SPEED_THRESHOLD_CM_S || linearAccel >= MOTION_ACCEL_THRESHOLD_M_S2) {
    return true;
  }
  return false;
}

void storeSensorSample(bool forceSample) {
  uint32_t nowUs = micros();
  if (!forceSample && (uint32_t)(nowUs - lastSensorSampleUs) < SENSOR_SAMPLE_PERIOD_US) {
    return;
  }

  lastSensorSampleUs = nowUs;
  SensorSample& sample = sensorBuffer[sensorWriteIndex];
  sample.tUs = nowUs;
  for (int i = 0; i < 4; i++) {
    // Send baseline-corrected FSR deltas to the Mac so stuck-high idle
    // channels, such as GPIO34/GPIO35 reading 4095, do not look pressed.
    sample.fsr[i] = (uint16_t)lastFsrDeltaRaw[i];
  }
  sample.ax = lastAccelMps2[0];
  sample.ay = lastAccelMps2[1];
  sample.az = lastAccelMps2[2];
  sample.gx = lastGyroDps[0];
  sample.gy = lastGyroDps[1];
  sample.gz = lastGyroDps[2];

  sensorWriteIndex = (sensorWriteIndex + 1) % SENSOR_BUFFER_SIZE;
  if (sensorWriteIndex == 0) {
    sensorBufferFilled = true;
  }
}

size_t appendPayload(char* payload, size_t used, size_t capacity, const char* fmt, ...) {
  if (used >= capacity) {
    return used;
  }

  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(payload + used, capacity - used, fmt, args);
  va_end(args);

  if (written < 0) {
    return used;
  }
  if ((size_t)written >= capacity - used) {
    return capacity - 1;
  }
  return used + (size_t)written;
}

void quantizeSample(const SensorSample& sample, uint32_t firstUs, int32_t values[11]) {
  values[0] = (int32_t)(sample.tUs - firstUs);
  values[1] = sample.fsr[0];
  values[2] = sample.fsr[1];
  values[3] = sample.fsr[2];
  values[4] = sample.fsr[3];
  values[5] = (int32_t)lroundf(sample.ax * ACCEL_QUANT_SCALE);
  values[6] = (int32_t)lroundf(sample.ay * ACCEL_QUANT_SCALE);
  values[7] = (int32_t)lroundf(sample.az * ACCEL_QUANT_SCALE);
  values[8] = (int32_t)lroundf(sample.gx * GYRO_QUANT_SCALE);
  values[9] = (int32_t)lroundf(sample.gy * GYRO_QUANT_SCALE);
  values[10] = (int32_t)lroundf(sample.gz * GYRO_QUANT_SCALE);
}

size_t appendIntRow(char* payload, size_t used, size_t capacity, const int32_t values[11]) {
  return appendPayload(
    payload,
    used,
    capacity,
    "[%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld]",
    (long)values[0],
    (long)values[1],
    (long)values[2],
    (long)values[3],
    (long)values[4],
    (long)values[5],
    (long)values[6],
    (long)values[7],
    (long)values[8],
    (long)values[9],
    (long)values[10]
  );
}

size_t putU16(uint8_t* payload, size_t offset, uint16_t value) {
  payload[offset++] = value & 0xFF;
  payload[offset++] = (value >> 8) & 0xFF;
  return offset;
}

size_t putI16(uint8_t* payload, size_t offset, int16_t value) {
  return putU16(payload, offset, (uint16_t)value);
}

size_t putI32(uint8_t* payload, size_t offset, int32_t value) {
  uint32_t raw = (uint32_t)value;
  payload[offset++] = raw & 0xFF;
  payload[offset++] = (raw >> 8) & 0xFF;
  payload[offset++] = (raw >> 16) & 0xFF;
  payload[offset++] = (raw >> 24) & 0xFF;
  return offset;
}

int16_t clampToI16(int32_t value) {
  if (value < -32768) return -32768;
  if (value > 32767) return 32767;
  return (int16_t)value;
}

void sendSensorBufferPacket() {
  static uint8_t payload[UDP_PAYLOAD_BYTES];
  int sampleCount = sensorBufferFilled ? SENSOR_BUFFER_SIZE : sensorWriteIndex;
  if (sampleCount <= 0) {
    storeSensorSample(true);
    sampleCount = sensorBufferFilled ? SENSOR_BUFFER_SIZE : sensorWriteIndex;
  }

  int startIndex = sensorBufferFilled ? sensorWriteIndex : 0;
  uint32_t firstUs = sensorBuffer[startIndex].tUs;
  uint16_t sequenceId = kickSequenceId++;
  uint8_t chunkCount = (sampleCount + SENSOR_CHUNK_SAMPLES - 1) / SENSOR_CHUNK_SAMPLES;
  int32_t previous[11] = {0};
  int32_t current[11] = {0};

  for (uint8_t chunkIndex = 0; chunkIndex < chunkCount; chunkIndex++) {
    int chunkStartSample = chunkIndex * SENSOR_CHUNK_SAMPLES;
    int rowCount = min(SENSOR_CHUNK_SAMPLES, sampleCount - chunkStartSample);
    size_t used = 0;

    payload[used++] = 'S';
    payload[used++] = 'K';
    payload[used++] = 'C';
    payload[used++] = '1';
    used = putU16(payload, used, sequenceId);
    payload[used++] = chunkIndex;
    payload[used++] = chunkCount;
    used = putU16(payload, used, (uint16_t)sampleCount);
    used = putU16(payload, used, (uint16_t)chunkStartSample);
    used = putU16(payload, used, (uint16_t)rowCount);
    used = putU16(payload, used, (uint16_t)SENSOR_SAMPLE_HZ);
    used = putU16(payload, used, (uint16_t)ACCEL_QUANT_SCALE);
    used = putU16(payload, used, (uint16_t)GYRO_QUANT_SCALE);
    used = putU16(payload, used, (uint16_t)FSR_1KG_ADC_THRESHOLD);
    used = putI16(payload, used, clampToI16((int32_t)lroundf(lastMotionSpeedCmS * 10.0f)));
    used = putI16(payload, used, clampToI16((int32_t)lroundf(lastLinearAccelMps2 * 100.0f)));

    for (int row = 0; row < rowCount; row++) {
      int sampleOffset = chunkStartSample + row;
      int index = (startIndex + sampleOffset) % SENSOR_BUFFER_SIZE;
      SensorSample& sample = sensorBuffer[index];
      quantizeSample(sample, firstUs, current);
      if (row == 0) {
        for (int j = 0; j < 11; j++) {
          used = putI32(payload, used, current[j]);
        }
      } else {
        for (int j = 0; j < 11; j++) {
          used = putI16(payload, used, clampToI16(current[j] - previous[j]));
        }
      }
      for (int j = 0; j < 11; j++) {
        previous[j] = current[j];
      }
    }

    if (used > sizeof(payload)) {
      Serial.println("Warning: chunked UDP sensor buffer payload overflow.");
      return;
    }

    bool sentByTcp = sendTcpPayload(payload, used);
    if (!sentByTcp && SEND_TCP_DIRECT) {
      Serial.println("TCP chunk send failed.");
    }
    if (SEND_UDP_CHUNKS_FALLBACK && !sentByTcp) {
      for (int repeat = 0; repeat < UDP_CHUNK_REPEAT_COUNT; repeat++) {
        sendUdpPayload(payload, used);
        delay(2);
      }
    }
    Serial.print("  chunk ");
    Serial.print(chunkIndex + 1);
    Serial.print("/");
    Serial.print(chunkCount);
    Serial.print(" bytes=");
    Serial.println(used);
    delay(2);
  }

  Serial.print("TCP target ");
  if (SEND_TCP_DIRECT) {
    Serial.print(MAC_IP);
    Serial.print(":");
    Serial.print(MAC_TCP_PORT);
  } else {
    Serial.print("direct off");
  }
  Serial.print(" UDP fallback ");
  if (SEND_UDP_DIRECT) {
    Serial.print(MAC_IP);
  } else {
    Serial.print("direct off");
  }
  if (SEND_UDP_SUBNET_BROADCAST) {
    Serial.print(" + subnet broadcast");
  }
  if (SEND_UDP_BROADCAST) {
    Serial.print(" + 255.255.255.255");
  }
  Serial.print(":");
  Serial.println(MAC_UDP_PORT);
  Serial.print("Sent chunked sensor buffer samples=");
  Serial.print(sampleCount);
  Serial.print(" chunks=");
  Serial.println(chunkCount);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < 4; i++) {
    pinMode(FSR_PINS[i], INPUT);
  }
  calibrateFsrBaseline();

  setupImu();
  connectWiFi();
  udp.begin(0);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  sendUdpHeartbeat();

  bool pressureTriggered = anyFsrPressureTriggered();
  bool motionTriggered = motionTriggeredByIcm();
  unsigned long nowMs = millis();
  if (pressureTriggered) {
    lastPressureTriggerMs = nowMs;
  }
  if (motionTriggered) {
    lastMotionTriggerMs = nowMs;
  }
  storeSensorSample(pressureTriggered || motionTriggered);

  bool pressureRecent = lastPressureTriggerMs > 0 && nowMs - lastPressureTriggerMs <= IMPACT_COINCIDENCE_MS;
  bool motionRecent = lastMotionTriggerMs > 0 && nowMs - lastMotionTriggerMs <= IMPACT_COINCIDENCE_MS;
  bool impactTriggered = REQUIRE_ICM_FOR_KICK_TRIGGER ? (pressureRecent && motionRecent) : pressureRecent;
  if (impactTriggered && nowMs - lastKickMs > COOLDOWN_MS) {
    if (REQUIRE_ICM_FOR_KICK_TRIGGER) {
      Serial.println("Kick impact detected by FSR+ICM - sending UDP chunks");
    } else {
      Serial.print("FSR-only trigger detected max_delta=");
      Serial.print(lastFsrMaxDeltaRaw);
      Serial.print(" sum_delta=");
      Serial.print(lastFsrSumDeltaRaw);
      Serial.print(" confirm=");
      Serial.print(fsrTriggerConfirmCount);
      Serial.print(" raw=[");
      for (int i = 0; i < 4; i++) {
        Serial.print(lastFsrRaw[i]);
        if (i < 3) {
          Serial.print(",");
        }
      }
      Serial.print("] baseline=[");
      for (int i = 0; i < 4; i++) {
        Serial.print(fsrBaselineRaw[i]);
        if (i < 3) {
          Serial.print(",");
        }
      }
      Serial.print("] delta=[");
      for (int i = 0; i < 4; i++) {
        Serial.print(lastFsrDeltaRaw[i]);
        if (i < 3) {
          Serial.print(",");
        }
      }
      Serial.print("] max_raw=");
      Serial.print(lastFsrMaxRaw);
      Serial.println(" - sending UDP chunks");
    }
    sendSensorBufferPacket();
    lastKickMs = nowMs;

    velocityX = 0.0f;
    velocityY = 0.0f;
    velocityZ = 0.0f;
    lastPressureTriggerMs = 0;
    lastMotionTriggerMs = 0;
    fsrTriggerConfirmCount = 0;
  }

  yield();
}
