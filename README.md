# Cruise Control System Using PID Controller

A Design Lab II course project focused on the **hardware implementation of a closed-loop cruise control system for a 4-wheel vehicle**, designed to maintain a specified wheel speed despite disturbances caused by inclined surfaces.

The project integrates **PID control, Arduino-based embedded programming, rotary encoder feedback, PWM motor actuation, and a mechanical bevel-gear drivetrain** to achieve real-time vehicle speed regulation.

---

## Project Information

| Parameter | Details |
|---|---|
| **Project** | Cruise Control System Using PID Controller |
| **Course** | Design Lab II |
| **Supervisor** | Dr. Dhiraj K. Mahajan |
| **Duration** | Jan 2026 – May 2026 |
| **Author** | Jahnavi Sharma |
| **Discipline** | Mechanical Engineering |
| **Platform** | Arduino UNO |
| **Control Method** | PID + PWM |

---

## Project Overview

The objective of this project was to design and implement a **hardware-based cruise control system** for a miniature 4-wheel vehicle.

Cruise control maintains a user-defined vehicle speed without continuous manual throttle adjustment. On inclined surfaces, gravitational forces act as disturbances and cause the vehicle speed to deviate from its desired value.

To address this problem, a **closed-loop PID controller** was implemented. A rotary encoder continuously measures the actual wheel speed, and the Arduino compares it with the desired speed. The resulting error is processed by the PID controller to generate an appropriate PWM command for the motor drivers.

Unlike a purely simulation-based approach, this project was implemented on a **physical vehicle prototype**, allowing the controller to be evaluated under real operating conditions.

---

## Project Objectives

- Design and develop a 4-wheel vehicle with a mechanically coupled drivetrain.
- Implement a closed-loop PID speed controller using Arduino UNO.
- Measure actual wheel speed using rotary encoder feedback.
- Maintain a constant speed under flat and inclined operating conditions.
- Reject disturbances caused by changes in road inclination.
- Tune the PID gains to improve speed tracking and transient response.
- Validate the controller through hardware experimentation.

---

## My Contribution

**Jahnavi Sharma — PID Tuning & Arduino Programming**

Primary responsibilities included:

- Implementing the PID control logic on Arduino UNO.
- Programming the real-time speed-control loop.
- Processing rotary encoder feedback.
- Calculating speed error between setpoint and measured speed.
- Implementing PWM-based motor actuation.
- Tuning the PID gains using the Ziegler–Nichols method as a starting point followed by manual refinement.
- Working toward minimizing overshoot, steady-state error, and settling time.
- Integrating the control algorithm with the motor-driver and encoder hardware.

---

# System Architecture

The closed-loop control architecture is:

```text
       Desired Speed
       (Set Point)
            │
            ▼
      Error Calculation
            │
            ▼
      PID Controller
            │
            ▼
        PWM Signal
            │
            ▼
     IBT-2 Motor Driver
            │
            ▼
        DC Motor
            │
            ▼
       Vehicle / Axle
            │
            ▼
       Rotary Encoder
            │
            ▼
      Speed Measurement
            │
            └──────────────► Feedback
