# soccer-kick-trajectory-predictor
This project connects a physical soccer-kick sensor to a Mac-based 3D soccer ball simulator.  An ESP32 samples four force-sensitive resistors (FSRs), plus an ICM-20948 inertial sensor. When the ESP32 detects a kick impact, it sends the recent sensor history to a Mac. The Mac hen animates the shot in a Panda3D first person soccer-field simulator.
