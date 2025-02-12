# GA_Shell_Detection
# 🚀 Arduino Servo & DC Motor Control with Relay and Sensor Integration

## 📌 Overview
This Arduino project controls a **servo motor, a DC motor, a relay, and a sensor system**. It integrates a **PCA9685 servo driver**, **push buttons**, and a **sensor** for detection. The system:
- Moves a **servo motor** between two positions (0° and 180°) via **button inputs**.
- **Toggles a relay** every 5 seconds.
- Controls a **DC motor** using PWM signals.
- **Detects an object** using a sensor and triggers a response.
- Outputs **debugging information** via Serial Monitor.

---

## ⚙️ Hardware Components
| Component                  | Description |
|----------------------------|-------------|
| **Arduino Uno/Mega/Nano**  | Main microcontroller |
| **PCA9685**                | 16-channel PWM driver for servos |
| **Servo Motor**            | Rotates between predefined angles |
| **DC Motor**               | Controlled via PWM signals |
| **Relay Module**           | Toggles ON/OFF every 5 seconds |
| **Keyence Fiber Optic Sensor** | Detects objects and sends TTL signals |
| **Push Buttons (2x)**      | Controls the servo position |
| **Wires & Power Supply**   | Connects components and provides power |

---

## 🔌 Wiring Diagram
| Arduino Pin  | Component |
|-------------|-----------|
| `SDA/SCL`   | **PCA9685 (I2C)** |
| `9`         | **Servo Motor PWM** |
| `10`        | **DC Motor PWM** (via H-Bridge) |
| `4`         | **Relay Control** |
| `2`         | **Sensor Digital Input** |
| `12, 13`    | **Push Buttons (Good Sump / Bad Sump)** |
| `5, 6, 7`   | **DC Motor Control (ENA, IN1, IN2)** |

---

## 🔧 Installation & Setup

### 1️⃣ Install Dependencies
1. Install **Arduino IDE**: [Download Here](https://www.arduino.cc/en/software)
2. Install **Adafruit PWM Servo Driver Library**:
   - Open **Arduino IDE** → Go to **Library Manager** (`Sketch` > `Include Library` > `Manage Libraries`).
   - Search for **Adafruit PWMServoDriver** and install it.

### 2️⃣ Upload Code to Arduino
1. Open `Arduino IDE`
2. Copy and paste the provided code into the editor.
3. Connect your Arduino board and select the correct **Board & Port**.
4. Click **Upload** to flash the program.

---

## ▶️ Usage Instructions
- The system **resets the servo** when powered on.
- **Press the buttons**:
  - `Good Sump (Pin 12)` → Moves servo to `0°`
  - `Bad Sump (Pin 13)` → Moves servo to `180°`
- The **relay toggles every 5 seconds**.
- The **DC motor runs continuously**.
- The **sensor detects objects**, triggering `"Shell detected! Take a picture!"` in the Serial Monitor.
