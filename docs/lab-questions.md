# Experiment 3 — Answers to the lab questions

**1. What is the function of the ESC?**
The Electronic Speed Controller takes power from the battery (here a 12 V adapter) and switches it through
its three outputs in sequence to commutate the brushless motor. It reads a 50 Hz servo-style pulse
(1000–2000 µs) from the receiver and sets the motor speed from the pulse width. Its BEC also provides
a regulated 5 V, used in this build to power the two servos.

**2. Why is a shared signal ground required?**
The control pulse is a voltage measured against a reference. The ESC can only read the 0/3.3 V pulse from the
ESP32-S3 correctly if both share the same 0 V. Without a common ground the signal floats and the ESC
sees noise or nothing. In this build the ESC ground and both servo grounds go to GND pins of the RX board.

**3. Why is the motor not connected directly to an ESP?**
A brushless motor needs three-phase switching at several amps; an ESP GPIO gives a few milliamps at 3.3 V
and has no power stage. Connecting the motor (or the 12 V supply) to the board would destroy it.
The ESP only produces the low-current command pulse; the ESC does the power work.

**4. What does the failsafe do when Wi-Fi commands stop?**
If the RX gets no valid packet for 500 ms it forces the throttle to 1000 µs (motor stop) and returns both
servos to centre (neutral control surfaces). The RX LED turns red and the TX pad rumbles, the OLED and the
dashboard show `LINK LOST`. Demonstrated by unplugging the TX while the motor runs.

**5. Why is this setup not sufficient for real aircraft flight control?**
- Wi-Fi/UDP has no delivery guarantee, suffers interference, delay and short range (2.4 GHz shared band).
- No flight controller: no IMU, no stabilisation, no GPS, no return-to-home.
- No redundancy (single link, single power path) and no certified radio protocol.
- Bench prototype: limited throttle (1000–1200 µs), no propeller, no current/temperature protection.
Real systems use dedicated RC links (e.g. ELRS/Crossfire) or telemetry radios with a flight controller
running proven firmware (ArduPilot/PX4).

## Measured results (fill in during the demo)

| Item | Value |
|---|---|
| Command rate | 50 Hz |
| Telemetry rate | 10 Hz |
| Round-trip latency (bench) | ___ ms |
| RSSI at 0.5 m | about −40 dBm |
| Failsafe reaction | ≤ 500 ms |
| Bad packets | 0 |
