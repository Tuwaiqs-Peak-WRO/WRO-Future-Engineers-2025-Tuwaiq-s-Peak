# Tuwaiqy – Autonomous Robot  
### by Team Tuwaiq’s Peak

![WhatsApp Image 2025-10-19 at 17 24 52_02bfc6a8](https://github.com/user-attachments/assets/d0764b24-2cf6-438f-bb1b-285d80fe3eac)

---

## Team Members  
| Name | Role | Picture |
|------|------|------|
| Reda Mohammed Albin Ahmed| Coder, Strategic Designer | ![WhatsApp Image 2025-10-23 at 17 48 36_bcb1500e](https://github.com/user-attachments/assets/31816810-682c-4088-92c9-9af91d87e180) |
| Ibrahim Alangari | 3D Designer, Hardware Engineer | ![WhatsApp Image 2025-10-23 at 17 48 36_722a9d41](https://github.com/user-attachments/assets/52d58156-5b33-41bd-9506-ce900f828f3a) |

---

## Challenge Overview  
**Competition:** WRO Future Engineers  
**Goal:** Build a small autonomous car that can perceive, decide, and navigate without human input.  

### Objectives
- Lane keeping and path following  
- Traffic logic  
- Obstacle handling  
- Parking  

---

## System Overview  
Tuwaiqy uses a hybrid navigation and obstacle management system combining computer vision and Time-of-Flight (ToF) distance sensing.  
The camera provides visual awareness for lane and path detection, while the ToF sensor provides short-range precision distance data.  
Together, they allow the robot to follow the track, detect obstacles, and navigate autonomously in real time.  

---

## Hardware Components  

### 1. Raspberry Pi 5 
<img width="1420" height="798" alt="image" src="https://github.com/user-attachments/assets/4b8ca573-7fd4-4b4b-b9d7-c6f700e3056b" />

**Processor:** 64-bit Quad-core ARM Cortex-A76 @ 2.4 GHz  

**GPU:** VideoCore VII (dual 4K support)  

**Memory:** 8 GB LPDDR4X  

**Storage:** 32 GB microSD  

**Networking:** Gigabit Ethernet, Wi-Fi 5, Bluetooth 5.0  

**Ports:** 2 × USB 3.0, 2 × USB 2.0, 40-pin GPIO  

**Camera Interface:** 2 × CSI-2  

**Display:** 2 × micro-HDMI (4K 60 fps)  

**Power:** USB-C 5 V / 5 A  

**OS:** Raspberry Pi OS (64-bit)  

**Dimensions:** 85.6 × 56.5 × 17 mm (~50 g)  

**Advantages**
- High performance for AI and computer vision  
- Full Linux + Python environment  
- Compact and efficient  

**Disadvantages**
- Requires active cooling  
- Needs a stable 5 V / 5 A power source  

---
### 2. DC-DC Step Down Converter (Mini560)  
<img width="158" height="145" alt="Screenshot 1447-05-04 at 10 57 44 AM" src="https://github.com/user-attachments/assets/fae33096-4bd5-4f1b-a38e-c072c11e7097" />

**Output:** 3.3 / 5 / 9 / 12 V (adjustable)  

**Current:** Up to 8 A (>90 % efficiency)  

**Weight:** 7 g  

**Role:** Provides 5 V power for the Raspberry Pi and logic circuits.  

---

### 3. Raspberry Pi Active Cooler  
<img width="140" height="82" alt="Screenshot 1447-05-04 at 10 58 43 AM" src="https://github.com/user-attachments/assets/ee912d44-039d-47f1-8bdb-7358493ed084" />

Compact fan-based cooler mounted via GPIO to prevent thermal throttling during high CPU usage.  

---

### 4. Gyroscope (BNO085)  
<img width="219" height="149" alt="Screenshot 1447-05-04 at 10 59 45 AM" src="https://github.com/user-attachments/assets/47f14870-e7d3-4a53-8624-dbac9253f1f1" />

**Type:** 9-axis IMU (accelerometer + gyroscope + magnetometer)  

**Interface:** I²C / SPI  

**Voltage:** 3.3–5 V  

**Role:** Provides orientation and motion feedback for PID and steering control.  

---

### 5. OLED Display (0.96 in) 
<img width="219" height="200" alt="Screenshot 1447-05-04 at 11 00 40 AM" src="https://github.com/user-attachments/assets/754eff68-b34b-44ec-aa9d-ce8144c6c592" />

**Resolution:** 128 × 64 pixels  

**Interface:** I²C / SPI  

**Voltage:** 3.3–5 V  

**Role:** Displays system information and debugging data.  

---

### 6. Push Button  
<img width="209" height="161" alt="Screenshot 1447-05-04 at 11 03 05 AM" src="https://github.com/user-attachments/assets/f67bcd61-15c0-4a4c-89a9-35ea4ce64238" />

**Type:** Momentary (Normally Open)  

**Role:** Used for manual start or reset input.  

---

### 7. Buzzer  
<img width="209" height="195" alt="Screenshot 1447-05-04 at 11 04 08 AM" src="https://github.com/user-attachments/assets/c925b2a0-7310-487b-8a51-a365f5e2a5ab" />

**Voltage:** 3–5 V (active)  

**Role:** Provides audio feedback for system events such as start, stop, or error alerts.  

---

### 8. Raspberry Pi Camera Module 3 (Wide)  
<img width="205" height="158" alt="Screenshot 1447-05-04 at 11 04 52 AM" src="https://github.com/user-attachments/assets/7ee45f71-e1e4-4862-9177-f4cef0711623" />

**Resolution:** 12 MP  

**Video:** 1080p @ 60 fps  

**Interface:** CSI  

**Lens:** Fixed-focus wide-angle  

**Role:** Main vision system for lane detection and path recognition.  

---

### 9. Step-Up Converter (XL6019)  
<img width="273" height="242" alt="Screenshot 1447-05-04 at 11 10 43 AM" src="https://github.com/user-attachments/assets/52ffe352-581b-45dc-b381-d375202a45ed" />

**Input:** 5–32 V → **Output:** 5–35 V (adjustable up to 4 A)  

**Role:** Boosts voltage for servos or higher-load components.  

---

### 10. RGB LED  
<img width="190" height="185" alt="Screenshot 1447-05-04 at 11 13 34 AM" src="https://github.com/user-attachments/assets/a4e98489-56fe-418b-b040-95fb9fa7103a" />

**Type:** 3-channel (R, G, B)  

**Voltage:** ~3.2 V per channel  

**Control:** PWM  

**Role:** Visual indicator for operating states (ready, running, or error).  

---

### 11. Nihewo 6500 mAh LiPo Battery (90C)  
<img width="192" height="202" alt="image" src="https://github.com/user-attachments/assets/98bc96f4-0ffe-46a7-8396-3a00d67cd0c1" />

**Voltage:** 7.4 V  

**Discharge Rate:** 90 C  

**Role:** Main power source for all modules.  

---

### 12. Servo Motor (40 kg·cm)  
<img width="192" height="202" alt="Screenshot 1447-05-04 at 11 15 58 AM" src="https://github.com/user-attachments/assets/98aa4383-5e4b-49b4-8f08-e2068e6a015f" />

**Voltage:** 6–12 V  

**Range:** 180°  

**Role:** Controls the steering of the front wheels.  

---

### 13. Servo Motor Driver (PCA9685PW)  
<img width="204" height="102" alt="Screenshot 1447-05-04 at 11 17 08 AM" src="https://github.com/user-attachments/assets/7749815b-eab6-44ce-811a-20ae7c0f828a" />

**Channels:** 16  

**Interface:** I²C  

**Voltage:** 5 V  

**Role:** Controls the steering servo and other PWM-based actuators.  

---

### 14. DC Motor with Encoder (620 RPM)  
<img width="460" height="460" alt="image" src="https://github.com/user-attachments/assets/64784d68-058e-4be5-a217-edf7154162c9" />

**Voltage:** 12 V  

**Feedback:** Encoder output for speed monitoring  

**Role:** Rear-wheel drive and speed feedback for closed-loop control.  

---

### 15. IBT-4 DC Motor Driver 
<img width="221" height="176" alt="Screenshot 1447-05-04 at 11 21 07 AM" src="https://github.com/user-attachments/assets/e782f217-ae12-4de8-8917-34c535d7f0c0" />

**Voltage:** 5–36 V  

**Current:** Up to 43 A peak  

**Control:** PWM  

**Role:** Drives high-torque DC motors for movement.  

---

### 16. TOF200C-VL53L0X  
<img width="221" height="176" alt="Screenshot 1447-05-04 at 11 22 43 AM" src="https://github.com/user-attachments/assets/b8b67f98-9c6c-4fea-8307-4a565b7e8525" />

**Range:** Up to 2 m  

**Accuracy:** ±3 %  

**Interface:** I²C  

**Voltage:** 2.6–5 V  

**Role:** Provides short-range distance measurements for obstacle detection and avoidance.  

---

### 17. TCA9548A 1-To-8 I2C 8 -Way Multi-Channel Expansion Board IIC Module Development Board
<img width="221" height="176" alt="Screenshot 1447-05-04 at 11 22 43 AM" src="![cjmcu-tca9548a-i2c-8-multi-channel-expansion-board-2-550x550](https://github.com/user-attachments/assets/1d0d790a-9ad0-4e28-a954-7d4a0f626d3a)
" />


## System Integration  

**Power**
- Nihewo 7.4 V battery powers the system.  
- Mini560 converter outputs regulated 5 V for logic and control circuits.  
- XL6019 step-up converter provides higher voltage for servo operation.  

**Control**
- Raspberry Pi 5 handles computer vision and sensor fusion.  
- BNO085 IMU provides stabilization data.  
- TOF sensor measures distance for real-time obstacle management.  

**Motion**
- IBT-4 driver powers the rear DC motors.  
- PCA9685 controls the front steering servo.  
- Encoders provide feedback for accurate speed control.  

**Feedback**
- OLED display shows live data and status.  
- RGB LED and Buzzer provide visual and audio feedback.  
- Push Button allows manual start or reset.  

---

## System Flow  

```text
Camera + TOF Sensor
        ↓
    Raspberry Pi 5
        ↓
 PCA9685 → Servo (Steering)
 IBT-4 → DC Motors (Drive)
        ↓
 OLED + RGB LED + Buzzer (Feedback)
        ↓
 Power Distribution → Mini560 / XL6019 from 7.4 V Battery

