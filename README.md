# Ánimo Platte — Mood-Responsive Environment Controller

A multi-sensor embedded environment controller that senses indoor environmental conditions and dynamically controls lighting, cooling, ventilation, and air purification using fuzzy-logic control.

**Developed for:** MTS-311 — Microcontrollers & Embedded Systems

**Program:** Bachelor of Mechatronics Engineering, NUST College of Electrical & Mechanical Engineering

**Role:** Firmware & Coding Lead

---

## Overview

Ánimo Platte is a physical embedded control system designed to create an adaptive indoor environment based on real-time environmental conditions.

The system continuously monitors:

* Temperature
* Humidity
* Ambient light
* Sound level
* Air quality
* Human occupancy

These measurements are processed by multiple fuzzy-logic controllers, which generate actuator commands for:

* RGB ambient lighting
* Cooling fan
* Ventilation servo
* Air-purifier servo
* Buzzer

The system also supports manual control through a Blynk interface. An ESP8266 handles wireless communication while an ATmega328P performs sensor processing, control logic, and actuator control.

---

## System Architecture

```text
                    ┌─────────────────────┐
                    │     Environment     │
                    └──────────┬──────────┘
                               │
              ┌────────────────┼────────────────┐
              │                │                │
           Sensors          Occupancy        Environment
              │              (PIR)            Feedback
              │                │                │
              └────────────────┼────────────────┘
                               │
                    ┌──────────▼──────────┐
                    │     ATmega328P      │
                    │                     │
                    │ Sensor Processing   │
                    │ Fuzzy Controllers   │
                    │ Actuator Control    │
                    │ Manual/Auto Modes   │
                    └──────────┬──────────┘
                               │
                 ┌─────────────┼─────────────┐
                 │             │             │
              PWM/Servo     MOSFET       Buzzer
                 │             │
          ┌──────▼─────┐   ┌───▼────┐
          │ Actuators  │   │  Fan   │
          └────────────┘   └────────┘
                              
                    Serial Communication
                               │
                    ┌──────────▼──────────┐
                    │      ESP8266        │
                    │   Wi-Fi / Blynk     │
                    └─────────────────────┘
```

---

## Hardware

### Controller

* Arduino UNO WiFi R3
* ATmega328P
* ESP8266 Wi-Fi module

### Sensors

| Sensor       | Measurement            | Interface         |
| ------------ | ---------------------- | ----------------- |
| BH1750       | Ambient light          | I²C               |
| BME280       | Temperature / humidity | I²C               |
| MAX9814      | Sound level            | Analog            |
| MQ-135       | Air quality            | Analog            |
| PIR HW-416-B | Occupancy              | Digital interrupt |

### Actuators

| Actuator               | Control      |
| ---------------------- | ------------ |
| RGB LED                | PWM          |
| DC cooling fan         | PWM + MOSFET |
| SG90 ventilation servo | Servo PWM    |
| SG90 purifier servo    | Servo PWM    |
| Buzzer                 | PWM          |

---

## Control System

The controller contains four independent fuzzy-logic controllers:

1. **Light Controller** — determines RGB lighting response from ambient lux.
2. **Noise Controller** — determines the response to measured sound level.
3. **Climate Controller** — processes temperature and humidity.
4. **Air Quality Controller** — determines the required ventilation/purification response.

Each controller follows the general sequence:

```text
Sensor Input
     ↓
Fuzzification
     ↓
Rule Inference
     ↓
Aggregation
     ↓
Defuzzification
     ↓
Actuator Command
```

The system also implements an interrupt-driven occupancy override using the PIR sensor.

When the environment is unoccupied, unnecessary actuators can be suppressed. Climate and air-quality control outputs are combined so that the system applies the higher required cooling/ventilation effort.

---

## Firmware

The ATmega328P firmware is responsible for:

* Sensor acquisition
* Fuzzy-logic calculations
* Membership functions
* Centroid defuzzification
* PWM generation
* Servo control
* Occupancy interrupt handling
* Automatic/manual control modes
* Serial command processing

The ESP8266 handles:

* Wi-Fi connectivity
* Blynk communication
* Manual user commands
* Serial communication with the ATmega328P

A defined serial command protocol allows individual subsystems to be switched between automatic and manual operation.

---

## Key Engineering Features

### Multi-protocol sensor integration

The project combines I²C, analog, digital interrupt, PWM, servo, and serial interfaces in one embedded system.

### Fuzzy-logic control

Instead of relying exclusively on fixed threshold values, environmental measurements are converted into fuzzy membership values and processed through rule-based controllers.

### Hybrid automatic/manual operation

Individual actuators can operate automatically under fuzzy control while others are manually controlled through the wireless interface.

### Interrupt-driven occupancy detection

The PIR sensor uses an external interrupt to respond to occupancy changes without relying solely on polling.

### Hardware-level actuator control

The DC fan is driven through an IRF530 MOSFET, while servos and RGB LEDs are controlled directly through appropriate PWM/control interfaces.

---

## Technical Challenge

One of the key integration challenges was preventing periodic automatic control updates from overwriting manual commands received through the ESP8266.

The solution was to implement independent control-mode states and stored manual actuator values. A manual command switches only the relevant subsystem into `MANUAL` mode, allowing the remaining actuators to continue operating under the fuzzy controller.

This created a hybrid control architecture where automatic and manual subsystems can operate simultaneously.

---

## Technologies

**Programming:** C++ / Arduino

**Embedded:** ATmega328P, ESP8266, Arduino UNO WiFi R3

**Sensors:** BH1750, BME280, MAX9814, MQ-135, PIR

**Control:** Fuzzy Logic, Membership Functions, Centroid Defuzzification

**Communication:** I²C, UART/Serial, Wi-Fi

**Actuation:** PWM, Servo Control, MOSFET Switching

**IoT:** Blynk

---

## Demonstration

### Demo 1

https://youtube.com/shorts/jURIiFCFpSs

### Demo 2

https://youtube.com/shorts/masb5z87Das

---

## Project Documentation

The `docs/` directory contains the project report, system diagrams, and supporting documentation.

The `hardware/` directory contains the bill of materials and wiring information.

---

## Future Improvements

Potential improvements identified during development include:

* ESP32-based controller architecture
* Dedicated CO₂ sensing
* OLED/local display
* Additional environmental presets
* Expanded fuzzy-rule sets
* Tunable controller parameters
* Improved wireless/embedded integration

---

## Authors

**Mahd Hassan** — Firmware & Coding Lead

NUST College of Electrical & Mechanical Engineering

Microcontrollers & Embedded Systems — MTS-311
