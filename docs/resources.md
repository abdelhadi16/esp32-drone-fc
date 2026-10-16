# Resources

Real books, papers, courses and documentation by real people, chosen for this project.
⭐ = start here.

## Books

| Book | Why it helps |
|------|--------------|
| ⭐ Quan Quan, *Introduction to Multicopter Design and Control*, Springer, 2017. [doi:10.1007/978-981-10-3382-7](https://doi.org/10.1007/978-981-10-3382-7) | The textbook closest to this project: frame and propulsion design, sensor calibration, state estimation, attitude and position control, failsafe and health monitoring of multicopters. |
| Randal W. Beard and Timothy W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton University Press, 2012. [MathWorks book page](https://www.mathworks.com/academia/books/small-unmanned-aircraft-beard.html) | Coordinate frames, sensor models, state estimation and autopilot design explained step by step (fixed-wing focus, the same tools apply). |
| Karl J. Åström and Richard M. Murray, *Feedback Systems: An Introduction for Scientists and Engineers*, 2nd ed., Princeton University Press, 2021. [free PDF](https://www.cds.caltech.edu/~murray/amwiki/Second_Edition.html) | Feedback, PID, stability and loop shaping, free to read. |
| Terry Kilby and Belinda Kilby, *Make: Getting Started with Drones*, Maker Media, 2015. [Maker Shed](https://makershed.com/products/make-getting-started-with-drones) | Beginner-friendly build book: parts, assembly, first flights, safety. |

## Papers and theses

| Paper | Why it helps |
|-------|--------------|
| ⭐ R. Mahony, V. Kumar and P. Corke, "Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor", *IEEE Robotics & Automation Magazine*, 2012. [ANU page](https://researchportalplus.anu.edu.au/en/publications/multirotor-aerial-vehicles-modeling-estimation-and-control-of-qua/) | The standard tutorial on quadrotor dynamics, attitude estimation and control. |
| R. Mahony, T. Hamel and J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group", *IEEE Transactions on Automatic Control*, 2008. [HAL](https://hal.archives-ouvertes.fr/hal-00488376) | The "Mahony filter": the next step after our simple complementary filter. |
| S. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays", report, 2010. [PDF](https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf) | The "Madgwick filter", popular on microcontrollers. |
| S. Bouabdallah, *Design and Control of Quadrotors with Application to Autonomous Flying*, PhD thesis, EPFL, 2007. [Infoscience](https://infoscience.epfl.ch/record/95939) | A complete quadrotor built and modelled from scratch, including altitude control. |
| S. Colton, "The Balance Filter", MIT, 2007 (search the title) | Two pages that explain the complementary filter we use for roll and pitch. |

## Courses, videos, blogs

| Resource | Why it helps |
|----------|--------------|
| ⭐ Vijay Kumar, *Robotics: Aerial Robotics*, University of Pennsylvania, [Coursera](https://www.coursera.org/learn/robotics-flight) | Free-to-audit course on quadrotor dynamics, control and planning. |
| ⭐ Brian Douglas, MATLAB Tech Talks ("Understanding PID Control", "Drone Simulation and Control"), [MathWorks](https://in.mathworks.com/videos/learn-engineering-concepts-with-matlab-tech-talks-1694512002750.html) | Short, very clear videos on PID, sensor fusion and drone control. |
| Brett Beauregard, "Improving the Beginner's PID" blog series, 2011 (brettbeauregard.com) | By the author of the Arduino PID library: integral windup, derivative kick, sample time. |
| Joop Brokking, YMFC-AL self-levelling Arduino quadcopter (brokking.net, YouTube) | Our loop structure, per-loop PID gains and yaw coupling come from this project. |
| Oscar Liang, [oscarliang.com](https://oscarliang.com) | Practical guides on ESCs, receivers, PID tuning, LiPo care. |

## Open-source autopilots (documentation we learned from)

| Documentation | Topic |
|---------------|-------|
| ArduPilot: [Altitude Hold mode](https://ardupilot.org/copter/docs/altholdmode.html), [Land mode](https://ardupilot.org/copter/docs/land-mode.html), [Battery failsafe](https://ardupilot.org/copter/docs/failsafe-battery.html), [Radio failsafe](https://ardupilot.org/copter/docs/radio-failsafe.html) | how a mature autopilot behaves; our height filter follows ArduPilot's original inertial navigation idea |
| PX4: [Safety (failsafe) configuration](https://docs.px4.io/main/en/config/safety.html), [Land detector](https://docs.px4.io/v1.12/en/advanced_config/land_detector.html) | failsafe actions and how land detection is done |
| INAV: [Navigation](https://github.com/iNavFlight/inav/blob/master/docs/Navigation.md), landing detector in `src/main/navigation/navigation_multicopter.c` | the "throttle below the descent average + no movement" landing detector |

## Open-source drones on ESP32 and small boards

| Project | Why look at it |
|---------|----------------|
| [ESP-FC](https://github.com/rtlopez/esp-fc) | Betaflight-compatible flight firmware for ESP32 / ESP32-S3: compare with ours, borrow ideas (DSHOT, gyro filters). |
| [Espressif ESP-Drone](https://docs.espressif.com/projects/espressif-esp-drone/en/latest/index.html) | Espressif's own small ESP32-S2/S3 drone, based on Crazyflie code. |
| Nicholas Rehm, [dRehmFlight](https://github.com/nickrehm/dRehmFlight) | A readable hobby flight controller (Teensy) made for learning. |
| Bart Mellink, [IBusBM library](https://www.arduino.cc/reference/en/libraries/ibusbm) | Another FlySky i-BUS decoder for Arduino. |

## Datasheets and chip documentation

| Document | Used for |
|----------|----------|
| Bosch Sensortec, *BMP280 Digital Pressure Sensor* datasheet (BST-BMP280-DS001), [copy hosted by Blue Robotics](https://bluerobotics.com/wp-content/uploads/2022/05/BMP280-DATASHEET.pdf) | registers, oversampling, IIR filter, compensation formulas (`baro_bmp280.h`) |
| InvenSense, *MPU-6000 / MPU-6050 Product Specification* rev 3.4, [PDF](https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/PS-MPU-6000A-00v3.4.pdf), and *Register Map and Descriptions* (RM-MPU-6000A-00) | ranges, low-pass filter, registers (`imu_mpu6050.h`) |
| Espressif, *ESP32-S3 Technical Reference Manual*, [PDF](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf) | LEDC, UART, I²C, ADC peripherals |
| Arduino-ESP32, [LEDC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html) | `ledcAttach` / `ledcWrite` used for the ESC signals |

## Batteries

| Resource | Why |
|----------|-----|
| Battery University, [BU-808: How to Prolong Lithium-based Batteries](https://batteryuniversity.com/article/how-to-prolong-lithium-based-batteries) | charging, storage voltage, what ages a lithium battery |

## Regulations (Algeria)

| Source | What it says |
|--------|--------------|
| Echorouk Online, ["Ministry of National Defence calls on drone owners to register their UAVs"](https://www.echoroukonline.com/ministry-of-national-defence-calls-on-drone-owners-to-register-their-uavs), 24 March 2026 | all drone owners must register with the National Centre for Unmanned Aircraft Systems before 30 April 2026; legal basis Presidential Decree 21-285 and an interministerial decree of 8 September 2025 |
| Algeria Invest, ["Fabrication et commercialisation de drones : l'État fixe les conditions"](https://algeriainvest.com/premium-news/fabrication-et-commercialisation-de-drones-letat-fixe-les-conditions), November 2025 | the September 2025 decree on acquisition, import, manufacture and commercial use; weight categories |
| DzairTube, ["Algeria enforces mandatory drone registration by April 30, 2026"](https://www.dzair-tube.dz/en/algeria-enforces-mandatory-drone-registration-by-april-30-2026-under-national-security-and-regulatory-framework/) | registration in person, by appointment |

These are news summaries; the official texts are in the *Journal Officiel*. Ask the
competition organisers how competitors' drones are authorised.
