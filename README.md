# Self balacing robot in Rust 🦀

Simple to use and quick to print two wheel robot.

![IMG_9873 – média](https://github.com/user-attachments/assets/073c2572-50e8-4f28-95f0-d40f6ffb6d6e)

Requires:

* ESP32C3 (or equivalent)
* DRV8833 driver (or equivalent)
* MPU6050
* x2 n20 motor
* A battery (used a 7.4V one)

The code is written in Rust and runs in a single loop. Due the single loop operation, the timings may not be the best, but the goal was achieved. I tried to wrap all the possible errors that could happen inside the main loop in such a way that it would not cause any strange _panic_ in case of error.

This is essentially an exploration of what can be achieved when Rust and the robotics field is mixed.

I'd like to point out a few things:

* Calibration.
* PID control.
* Slow/Fast Decay.
* PWM frequency.
* Wheels size.
* Gyro and accel direction.
* Filters.
* I2C packet loss/ FIFO packet loss.
* Starting angle.

All the above had some kind of impact in the development of the project. 

Since the motors do not include a encoder, the following project scope is only within the angle error mitigation, the so called "balance". Therefore, it does not garantee any cascade control.
