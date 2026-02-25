# FreeRTOS Multi-Sensor Monitoring System

## 📋 Overview

A real-time multi-sensor monitoring system built with ESP32 and FreeRTOS, demonstrating concurrent task execution, inter-task communication, and hardware control. The system simulates temperature and humidity monitoring with alert thresholds, moving average calculations, and LCD display output.

## 🎯 Objectives

- Implement a multi-task embedded system using FreeRTOS
- Demonstrate inter-task communication using queues and semaphores
- Apply task prioritization and dual-core processing
- Implement real-time data processing with moving averages
- Control hardware peripherals (LCD, LEDs) in a concurrent environment

## 🔧 Hardware/Platform

- **Microcontroller:** ESP32 DevKit (dual-core)
- **Display:** LCD 20x4 I2C (address 0x27)
- **Indicators:** 2 LEDs (Red - Temperature alert, Yellow - Humidity alert)
- **Simulation:** Wokwi Online Simulator
- **Protocols:** I2C (LCD communication)

## ✨ Features

### Core Functionality
- ✅ Real-time sensor data acquisition (simulated)
- ✅ Moving average calculation (5-sample window)
- ✅ Threshold-based alerting system
- ✅ LCD display with live updates
- ✅ Visual LED alerts
- ✅ Dual-core task distribution

### FreeRTOS Concepts Implemented
- **Tasks:** 4 concurrent tasks with different priorities
- **Queues:** 2 queues for sensor data transfer
- **Semaphores:** Binary semaphore for task synchronization
- **Priorities:** Priority-based task scheduling (1-3)
- **Dual-core:** Task distribution across ESP32 cores

## 🏗️ Architecture

```
Core 0:                           Core 1:
┌─────────────────────┐          ┌─────────────────────┐
│ SensorTask1 (Temp)  │──Queue──>│                     │
│ Priority: 2         │          │  ProcessTask        │
│                     │          │  Priority: 3 (HIGH) │
│ SensorTask2 (Hum)   │──Queue──>│                     │
│ Priority: 2         │          └──────────┬──────────┘
└─────────────────────┘                     │
                                      Semaphore
                                            │
                                  ┌─────────▼──────────┐
                                  │  DisplayTask       │
                                  │  Priority: 1 (LOW) │
                                  └────────────────────┘
```

### Task Details

**SensorTask1 (Temperature):**
- Reads simulated temperature (15-35°C)
- Sends data via `queueTemp` every 2 seconds
- Activates red LED if temperature > 30°C
- Runs on Core 0, Priority 2

**SensorTask2 (Humidity):**
- Reads simulated humidity (40-80%)
- Sends data via `queueHum` every 2 seconds
- Activates yellow LED if humidity > 70%
- Runs on Core 0, Priority 2

**ProcessTask:**
- Receives data from both sensor queues
- Calculates moving average (last 5 values)
- Evaluates comfort conditions
- Signals DisplayTask via semaphore
- Runs on Core 1, Priority 3 (highest)

**DisplayTask:**
- Waits for semaphore signal
- Updates LCD with current values and averages
- Shows alerts when thresholds exceeded
- Displays system uptime
- Runs on Core 1, Priority 1 (lowest)

## 📁 Project Structure

```
01-freertos-multitask/
├── src/
│   └── main.cpp
├── diagram.json
├── platformio.ini
├── wokwi.toml
├── images/
│   └── lcd_display.png
├── README.md
```

## 🚀 How to Run

### On Wokwi Simulator
1. Go to [Wokwi ESP32 Simulator](https://wokwi.com)
2. Create new ESP32 project
3. Copy `main.cpp` content
4. Add components:
   - ESP32 DevKit
   - LCD 20x4 I2C (address 0x27)
   - 2 LEDs (pins 2 and 4)
5. Connect I2C: SDA→GPIO21, SCL→GPIO22
6. Click "Start Simulation"

### On Real Hardware
```bash
# Clone repository
git clone https://github.com/DavidMurillo32/embedded-portfolio.git
cd embedded-portfolio/01-freertos-multitask

# Build and flash with ESP-IDF
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# Or use PlatformIO
pio run --target upload
pio device monitor
```

## 📊 LCD Display Format

```
┌────────────────────┐
│T:23.4C Pm:24.1C    │ ← Temperature: current & average
│H:56.7% Pm:58.3%    │ ← Humidity: current & average
│Estado: CONFORT     │ ← Status or alert message
│Tiempo: 12 seg      │ ← System uptime
└────────────────────┘
```

### Alert States
- **CONFORT**: 20°C ≤ Temp ≤ 25°C AND 40% ≤ Hum ≤ 60%
- **NORMAL**: Values within limits but outside comfort zone
- **ALERTA: Temp alta!**: Average temperature > 30°C
- **ALERTA: Hum alta!**: Average humidity > 70%
- **ALERTA: T y H alta!**: Both thresholds exceeded

## 🧪 Testing

### Functional Tests
- [x] All tasks start correctly and run concurrently
- [x] Sensor data flows through queues without loss
- [x] Moving average calculation is accurate
- [x] LCD updates every 2 seconds with correct values
- [x] LEDs activate at correct thresholds
- [x] Semaphore synchronization works properly
- [x] System runs stably for extended periods

### Performance Tests
- [x] Queue never fills up (0 messages waiting consistently)
- [x] No task starvation occurs
- [x] Priority scheduling works as expected
- [x] Dual-core distribution balances workload

## 🔍 Key Learnings

### Technical Concepts Applied
1. **FreeRTOS Task Management**: Created and managed multiple concurrent tasks with different priorities
2. **Inter-Task Communication**: Used queues to safely pass data between tasks
3. **Synchronization**: Implemented semaphores to coordinate task execution
4. **Circular Buffers**: Implemented moving average using circular array
5. **Dual-Core Processing**: Distributed workload across ESP32's two cores
6. **I2C Communication**: Interfaced with LCD display via I2C protocol
7. **Struct Data Types**: Created custom data structures for sensor information
8. **Volatile Variables**: Used volatile for variables accessed by multiple tasks

### Design Patterns
- **Producer-Consumer**: Sensor tasks produce data, ProcessTask consumes
- **Observer**: DisplayTask waits for signals from ProcessTask
- **State Machine**: Alert system based on threshold states

### Challenges Overcome
1. **Race Conditions**: Solved using FreeRTOS queues and semaphores
2. **Data Smoothing**: Implemented moving average to reduce noise
3. **Priority Inversion**: Avoided by proper priority assignment
4. **Initialization Sequencing**: Added semaphore to ensure all tasks ready before starting

## 📚 Dependencies

```cpp
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <LiquidCrystal_I2C.h>
```

## 🔗 References

- [FreeRTOS Official Documentation](https://www.freertos.org/a00106.html)
- [ESP32 FreeRTOS Tutorial](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html)
- [Wokwi ESP32 Simulator](https://docs.wokwi.com/parts/wokwi-esp32-devkit-v1)
- [LiquidCrystal I2C Library](https://github.com/johnrickman/LiquidCrystal_I2C)

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 👤 Author

**David Santiago Murillo Hernández**
- GitHub: [@DavidMurillo32](https://github.com/DavidMurillo32)
- Email: dasan46@outlook.com
- LinkedIn: www.linkedin.com/in/david-santiago-murillo-hernández-3902b7313

---

*Part of my [Embedded Systems Portfolio](https://github.com/DavidMurillo32/embedded-portfolio)*
