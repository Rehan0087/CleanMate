# CleanMate

Two ESP32 robots that drive side by side as one wide floor cleaner. The leader sets the path; the follower matches its speed over ESP-NOW and holds the side gap with a VL53L0X laser sensor.

CSE 4326 – Microprocessors and Microcontrollers Laboratory, United International University.

## Hardware (per car)

- ESP32 DevKit, MPU6050 gyro, 2× HC-SR04 ultrasonic, TB6612FNG driver, 2× N20 encoder motors
- Follower also has a VL53L0X ToF sensor facing the leader
- 3S 18650 pack with BMS, MP1584 buck converter
- A third ESP32 works as a USB gateway to the laptop

## Folders

| Path | What it is |
| --- | --- |
| `firmware/robot1_leader` | Leader car |
| `firmware/robot2_follower` | Follower car |
| `firmware/cleanmate_gateway` | ESP-NOW ⇄ USB serial gateway |
| `firmware/wheel_calibration` | Encoder counts-per-rev test |
| `dashboard/cleanmate_dashboard.html` | Web Serial dashboard (Chrome / Edge) |
| `pi/cleanmate_pi.py` | Terminal console and CSV logger for a Raspberry Pi |

## Build

Arduino IDE, board "ESP32 Dev Module" (core 2.x or 3.x). Library: **VL53L0X by Pololu**. Each car sketch checks its board MAC at boot, so flash the leader and follower sketches on the right boards.

## Serial commands (115200 baud)

| Command | Action |
| --- | --- |
| `d <v>` / `s` | drive at v m/s / stop |
| `g <mm>` | target side gap |
| `fwd <m>`, `turn <deg>`, `uturn <p>` | formation moves |
| `mow <lanes> <len> <pitch>` | back-and-forth cleaning pattern |
| `hold` / `hold off` | parked heading hold |
| `rejoin on/off` | parked re-join after a car is moved |
| `x` | emergency stop |
| `cal`, `cs`, `ffcal`, `cfg`, `trim`, `turns`, `dia`, `bw` | calibration |
| `k`, `p`, `sonar`, `i2c` | status and diagnostics |

Prefix a command with `1 ` or `2 ` on the gateway to send it to one car only.

## Team

| ID | Name |
| --- | --- |
| 0112330222 | Diba Jabin Fariha Tithy |
| 0112330205 | Radawona Islam |
| 0112330996 | MD Rayhan Hossain |
| 0112330997 | MD. Atif Rehan Kaif |
| 0112330984 | Md Toufiq Imroz Khealid Khan |
