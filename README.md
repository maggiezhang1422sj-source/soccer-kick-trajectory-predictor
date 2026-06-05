# soccer-kick-trajectory-predictor


This project connects a physical soccer-kick sensor to a Mac-based 3D soccer
ball simulator.

An ESP32 samples four force-sensitive resistors (FSRs) mounted on the cleat lace
area, plus an ICM-20948 inertial sensor. When the ESP32 detects a kick impact, it
sends the recent sensor history to a Mac. The Mac estimates initial ball speed,
launch angle, direction, and spin, then animates the shot in a Panda3D first
person soccer-field simulator.

The current development mode uses the FSRs for launch estimation and ignores ICM
data in the Mac estimator while the FSR orientation is being calibrated.

## Project Files

- `mac_simulator.py`: Panda3D 3D simulator, TCP/UDP packet receiver, sensor
  decoder, launch estimator, ball physics, replay view, and GUI.
- `esp32_kick_sensor_udp/esp32_kick_sensor_udp.ino`: ESP32 firmware for FSR and
  ICM-20948 sampling, circular buffering, trigger detection, TCP transfer, and
  UDP fallback.
- `udp_debug_listener.py`: small packet listener for testing network delivery
  without opening the Panda3D simulator.
- `requirements.txt`: Python dependencies for the Mac simulator.
- `kick_ai_model.json`: small persistent calibration file used by the adaptive
  launch model.

## System Overview

```text
Cleat FSRs + ICM-20948
          |
          v
       ESP32
   5 kHz circular buffer
          |
          v
  TCP packets on port 4210
  UDP fallback / heartbeat
          |
          v
      MacBook
 sensor decode + launch estimate
          |
          v
 Panda3D soccer ball simulator
 FPV shot + 3D replay
```

## Hardware

### ESP32

The sketch is written for a standard ESP32 using ADC1 pins for the FSRs and VSPI
for the ICM-20948.

### FSR Layout

The current code assumes a left-foot cleat layout:

```text
              Closer to ankle / upper

                  [fsr0] outer upper      [fsr3] inner upper
                  GPIO32                  GPIO35

                  [fsr2] outer lower      [fsr1] inner lower
                  GPIO34                  GPIO33

              Closer to toes / lower
```

In code:

- `fsr0` / GPIO `32`: outer upper FSR, farther from big toe and closer to ankle
- `fsr1` / GPIO `33`: inner lower FSR
- `fsr2` / GPIO `34`: outer lower FSR
- `fsr3` / GPIO `35`: inner upper FSR

### FSR Voltage Divider

Each FSR should be wired as a voltage divider into one ESP32 ADC pin.

One common wiring is:

```text
3.3V --- FSR --- ADC pin --- fixed resistor --- GND
```

When pressure changes the FSR resistance, the ADC voltage changes. The exact
direction depends on which side of the divider the FSR is placed. The firmware
therefore uses a baseline-corrected absolute delta:

```cpp
delta = abs(raw_adc - idle_baseline_adc)
```

This makes the trigger work whether the ADC value rises or falls when the FSR is
pressed.

Use a fixed resistor value that gives a useful ADC range for your FSR and foot
pressure. Values around `10k` are commonly practical for early testing, but the
best value depends on the FSR and the mechanical mounting.

Important ESP32 ADC notes:

- GPIO `32`, `33`, `34`, and `35` are ADC1 pins.
- ADC1 is preferred because ADC2 conflicts with Wi-Fi on ESP32.
- GPIO `34` and `35` are input-only, which is fine for FSR ADC input.
- Keep wires short and share a solid ground between sensors and ESP32.

### ICM-20948 SPI Wiring

The sketch uses ESP32 VSPI pins:

- ICM `SCK` -> ESP32 GPIO `18`
- ICM `MISO` -> ESP32 GPIO `19`
- ICM `MOSI` -> ESP32 GPIO `23`
- ICM `CS` -> ESP32 GPIO `5`
- ICM `VCC` -> `3.3V`
- ICM `GND` -> `GND`

Install the SparkFun ICM-20948 Arduino library before compiling.

## Physics Background

### Impulse

The key physical relationship is impulse:

```text
F * dt integrated over time = m * delta_v
```

For a kick, a single peak force sample is not enough. A short hard hit and a
longer softer hit may have similar impulse. The Mac estimator therefore
integrates each FSR channel over the kick window:

```python
impulse_i += 0.5 * (previous_delta_i + current_delta_i) * dt
```

This is trapezoidal integration. The result is not yet true Newton-seconds,
because the FSRs are not calibrated to Newtons. It is currently
`ADC_delta * seconds`, which is still useful for relative direction, launch, and
spin estimation. With calibration, each ADC value can later be converted to
force in Newtons.

### Direction and Launch From FSR Balance

The four integrated FSR channels are combined into physical regions:

```text
outer = fsr0 + fsr2
inner = fsr1 + fsr3
upper = fsr0 + fsr3
lower = fsr1 + fsr2
total = outer + inner
```

The current estimator uses:

```text
side_bias = (outer - inner) / total
vertical_bias = (lower - upper) / total
```

Interpretation:

- More lower-FSR impulse means a higher launch angle.
- More upper-FSR impulse means a lower launch angle or rolling shot.
- More outer/inner imbalance changes initial direction and side spin.

If your physical left/right direction is mirrored, flip these constants near the
top of `mac_simulator.py`:

```python
FSR_DIRECTION_SIGN = 1.0
FSR_SIDE_SPIN_SIGN = 1.0
```

### Drag and Magnus Effect

The simulator applies a simplified ball-flight model:

- Gravity pulls the ball downward.
- Drag slows the ball opposite the direction of travel.
- Backspin adds lift.
- Side spin creates curl through a simplified Magnus-force term.

The Magnus effect is modeled as acceleration perpendicular to the ball's forward
motion. It is not a full computational fluid dynamics model, but it is good
enough to visualize spin, curl, and lift during testing.

## ESP32 Firmware Algorithm

### Sampling

The ESP32 continuously writes sensor samples into a circular buffer:

```text
sample = [t_us, fsr0, fsr1, fsr2, fsr3, ax, ay, az, gx, gy, gz]
```

Current defaults:

- `SENSOR_SAMPLE_PERIOD_US = 200`
- `SENSOR_SAMPLE_HZ = 5000`
- `SENSOR_BUFFER_SIZE = 1000`

That stores about `200 ms` of recent history at a target `5 kHz` sample rate.

The outgoing FSR values are baseline-corrected deltas, not raw ADC values. This
prevents idle-high channels from looking pressed on the Mac side.

### Baseline Calibration

At boot, the ESP32 averages idle FSR readings:

```cpp
FSR_BASELINE_SAMPLES = 80
```

Do not touch the FSRs during boot. The serial monitor prints the baseline so you
can see whether any channel is floating or stuck.

### Trigger Detection

The current firmware is FSR-triggered for debugging. ICM data is still sampled
and sent, but the Mac ignores it for launch estimation.

A kick trigger currently requires:

```cpp
FSR_TRIGGER_DELTA_THRESHOLD = 850
FSR_TRIGGER_SUM_DELTA_THRESHOLD = 1400
FSR_TRIGGER_CONFIRM_READS = 5
```

That means:

- at least one FSR must move by `850+` ADC counts from baseline
- all four FSR deltas combined must reach `1400+`
- the condition must hold for `5` consecutive loop reads

This reduces false triggers from ADC noise or floating inputs.

After sending one kick buffer, the firmware waits:

```cpp
COOLDOWN_MS = 3500
```

### Packetization

The ESP32 sends the circular buffer as compact binary chunks. Each chunk starts
with magic bytes:

```text
SKC1
```

Each packet contains:

- sequence ID
- chunk index and chunk count
- total sample count
- starting sample index
- row count
- sample rate
- acceleration and gyro quantization scales
- FSR threshold metadata
- one full base row
- int16 delta rows for the rest of the chunk

This keeps packets smaller and avoids sending a huge UDP datagram.

### TCP and UDP

The current ESP32 sketch sends sensor chunks by TCP first:

```text
Mac TCP port: 4210
```

Each TCP payload is length-prefixed with a 2-byte little-endian payload length.

UDP support remains as fallback:

- UDP chunk fallback can send the same `SKC1` payloads.
- UDP heartbeat can send `KICK_HEARTBEAT` packets so you can confirm network
  connectivity.

TCP is more reliable than UDP because it handles ordering and retransmission.
The tradeoff is that TCP direct mode needs the Mac IP address to be correct.

## Mac Simulator Algorithm

### Packet Receive

The Mac listens on port `4210`. It can parse:

- TCP length-prefixed `SKC1` binary chunks
- UDP `SKC1` binary chunks
- older `SKB1` single binary packets
- older JSON and CSV debug packets

When all chunks in a sequence arrive, the Mac reconstructs the sensor buffer.

### Feature Extraction

The Mac extracts:

- per-channel FSR peak values for debug display
- per-channel integrated FSR impulse
- inner/outer balance
- upper/lower balance
- impact duration

Current launch estimation ignores ICM values:

```python
IGNORE_ICM_FOR_LAUNCH = True
```

This is intentional while debugging FSR orientation. Later, ICM acceleration and
gyro can be blended back into the model for better speed, swing direction, and
spin estimation.

### Launch Estimate

The simulator estimates:

- initial ball speed
- launch angle
- horizontal direction
- backspin RPM
- side-spin RPM
- drag scale
- Magnus lift/curl scale

The selected GUI distance/angle places the ball relative to the goal. The FSR
balance then adjusts direction, launch, and spin.

### Ball Flight

The ball state is integrated over time:

```text
velocity += acceleration * dt
position += velocity * dt
```

Acceleration includes gravity, drag, lift, and curl. The camera follows just
behind the ball during flight. After the shot ends, the simulator switches to a
3D bird-view replay and redraws the trajectory.

### Goal and Net

The simulator checks whether the ball crosses the goal plane inside:

```text
goal width  = 7.32 m
goal height = 2.44 m
```

If the shot enters the goal, the GUI shows:

```text
Goal!
```

The simulated net slows and contains the ball when it reaches the back of the
goal.

## Adaptive Calibration

`kick_ai_model.json` stores small correction parameters:

- speed scale and bias
- launch bias
- direction bias
- spin scale
- drag scale
- Magnus lift/curl scale

After each simulated shot, the Mac updates these parameters gently based on the
observed result. This is an online calibration layer around the physics model,
not a trained deep neural network.

For a real AI model later, collect labeled data:

```text
input:  FSR impulse curves + ICM acceleration/gyro history
label:  measured speed, launch angle, direction, spin, landing point
```

Then train the model on the Mac, not the ESP32. The ESP32 should stay focused on
fast sampling and reliable data transfer.

## Mac Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python mac_simulator.py
```

Controls:

- `D`: fire a demo shot without ESP32 hardware
- `A`: show listen status
- `R`: reset ball and camera
- `[` / `]`: decrease/increase test shot distance
- `,` / `.`: rotate test shot position angle
- `Esc`: quit

GUI controls:

- `Shot`: place the ball `10 m` to `100 m` from goal
- `Aim`: choose the ball position angle from `0 deg` to `180 deg`

The camera stays behind the ball before launch and follows the ball closely
during flight.

## ESP32 Setup

1. Open `esp32_kick_sensor_udp/esp32_kick_sensor_udp.ino` in Arduino IDE.
2. Install the SparkFun ICM-20948 Arduino library.
3. Set your Wi-Fi SSID, password, and Mac IP.
4. Select the correct ESP32 board and serial port.
5. Upload the sketch.
6. Open Serial Monitor at `115200` baud.
7. Keep FSRs untouched during boot baseline calibration.
8. Start `mac_simulator.py` on the Mac.

Find your Mac Wi-Fi IP address:

```bash
ipconfig getifaddr en0
```

If you use Ethernet, try:

```bash
ipconfig getifaddr en1
```

## Debugging

### Test Network Packets Without Panda3D

Stop the simulator first, then run:

```bash
python3 udp_debug_listener.py
```

Only one program can listen on port `4210` at a time.

### False FSR Triggers

If the ESP32 sends packets without FSR pressure:

1. Do not touch FSRs during boot calibration.
2. Check the serial baseline values.
3. Check for floating ADC wiring or loose ground.
4. Increase:

```cpp
FSR_TRIGGER_DELTA_THRESHOLD
FSR_TRIGGER_SUM_DELTA_THRESHOLD
FSR_TRIGGER_CONFIRM_READS
```

### Weak or Missing Trigger

If a real press does not trigger:

1. Watch `raw`, `baseline`, `delta`, and `sum_delta` in Serial Monitor.
2. Lower `FSR_TRIGGER_DELTA_THRESHOLD`.
3. Lower `FSR_TRIGGER_SUM_DELTA_THRESHOLD`.
4. Confirm the FSR voltage divider actually changes ADC voltage.

### Direction or Launch Reversed

If left/right is mirrored, flip:

```python
FSR_DIRECTION_SIGN
FSR_SIDE_SPIN_SIGN
```

If upper/lower launch feels backward, check the physical FSR mapping first:

```text
upper = fsr0 + fsr3
lower = fsr1 + fsr2
```

The current intended behavior is:

- lower FSRs -> higher ball launch
- upper FSRs -> lower or rolling launch

## Publishing Safety

Before uploading to a public GitHub repository, remove private values from the
ESP32 sketch:

```cpp
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* MAC_IP = "YOUR_MAC_IP";
```

Do not commit real Wi-Fi credentials.

## Limitations

- FSR values are not calibrated to Newtons yet.
- The current estimator uses FSR data only for launch while orientation is being
  debugged.
- ICM data is sampled and transmitted but not currently used by the launch model.
- The simulator physics are simplified and meant for visualization, not
  professional ball-flight measurement.
- TCP direct mode requires the Mac IP address to be correct.

## Future Improvements

- Calibrate each FSR from ADC value to force in Newtons.
- Use ICM acceleration and gyro to estimate leg speed and foot orientation.
- Collect labeled kicks with video or radar measurement.
- Train a Mac-side model to predict speed, direction, launch angle, and spin.
- Add automatic Mac IP discovery or ESP32 captive setup.
- Add saved sessions and shot history charts.
