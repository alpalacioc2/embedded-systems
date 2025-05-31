MSP432 Line-Following Robot Final Project

**Authors:** Alejandro Caro & Joshua Romeo  
**Course:** EEL4742 - Embedded Systems  
**Semester:** Spring 2025  
**Lab Section:** Section 1  
**Lab Date:** 04/05/2025

## 📌 Project Overview

This project implements a fully autonomous line-following robot using the MSP432 microcontroller and RSLK MAX. The robot navigates a track by interpreting signals from infrared (IR) sensors, while also reacting to bumper switch inputs. It uses pulse-width modulation (PWM) to control motors and provides visual feedback through onboard LEDs and RGB LED indicators.

---

## 🛠 Features

- **FSM-based line-following logic** using an 8-element IR sensor array
- **PWM motor control** for differential drive and maneuverability
- **Bumper switch interrupts** for real-time obstacle interaction
- **Visual LED feedback** for system status and directionality
- **Startup LED animation** to confirm successful initialization
- **Dynamic handling of LOST condition** with correction maneuvers
- Modular design separating motor control, sensor interpretation, and LED signaling

---

## 🧩 Hardware Components

- MSP432P401R LaunchPad
- TI RSLK MAX 
- 2 DC motors (with H-Bridge motor driver)
- IR reflective sensor array (8-element)
- 6 bumper switches
- RGB LED and single-color LEDs (for feedback)
- External power supply for motors

---

## ⚙️ System Behavior

- **Standby Mode:**  
  - LED1 blinks while the system awaits bumper input.
  - RGB LED shows color based on sensor readings.

- **Tracking Mode:**  
  - Triggered when `BumperZeroState` is pressed.
  - Robot follows a black line based on IR sensor readings.
  - Color-coded RGB LED gives visual feedback on direction:
    - RED: Hard left  
    - YELLOW: Slight left  
    - GREEN: Forward  
    - CYAN: Slight right  
    - BLUE: Hard right  
    - WHITE: LOST

- **Obstacle Response:**  
  - On detecting a bumper press during tracking, the robot:
    - Stops
    - Reverses briefly
    - Then returns to standby mode

---

## 🔁 Main Functions

- `main()` — Controls overall behavior, mode switching, and loop execution  
- `LineFollowing()` — FSM implementation to follow the line based on IR sensor conditions  
- `readLine()` / `IRProcessData()` — Gathers and processes IR sensor input  
- `MotorDriver()` — Sets motor direction and PWM duty cycle  
- `LEDsDriver()` / `RGBDriver()` / `UpdateLineLED()` — Visual feedback  
- `bumperSwitchesHandler()` — Interrupt Service Routine for bumper switches  
- `LEDStartUp()` — Startup light animation

---

## 🧠 Sensor States (Defined as `sensorCondition` enum)

- `allLeft`, `moreLeft`, `equal`, `moreRight`, `allRight`, `LOST`, `STANDBY`

These values are used to determine robot steering and RGB LED output.

---

## 🔧 Setup and Compilation

This project uses **TI's DriverLib** and must be compiled using **Code Composer Studio** with MSP432P4xx support.

1. Import the project into Code Composer Studio.
2. Ensure the correct target is selected: MSP432P401R.
3. Connect your robot to the computer and flash the program.
4. Use the bumper switch BS0 to begin tracking.

---

## 📈 Performance Notes

- Optimal delays were determined through empirical testing for stability and sensor responsiveness.
- Sensor "LOST" handling includes a counter to initiate reversal and 180-degree corrections.
- The PWM `DUTY` variable is dynamically modified during line tracking for smooth motor response.

---

## 📃 License

This project is for educational purposes under EEL4742 at Florida State University.  
All rights reserved © 2025 Alejandro Caro & Joshua Romeo.
