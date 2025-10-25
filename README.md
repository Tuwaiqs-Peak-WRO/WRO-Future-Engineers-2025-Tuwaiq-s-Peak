
---
# ⛰️ Team Logo
![WhatsApp Image 2025-10-19 at 17 24 52_59760c78](https://github.com/user-attachments/assets/85020a02-1447-4304-8e9d-ff8b65924a06)


---
# 🚗 Overview
Our robot Tuwaiqy(the robot's name) uses a hybrid obstacle management system that combines computer vision and ultrasonic distance sensing to navigate safely and intelligently. The camera provides visual awareness of the environment, while the ultrasonic sensors measure exact distances to nearby objects. Together, they allow the robot to follow a visible path and avoid collisions in real time, achieving accurate and autonomous movement.

---
# 👷‍♂️ Team Members
<img src="https://github.com/user-attachments/assets/df420f1a-1ffd-4679-99c3-c728850508a5" width="200" height="356">

- Reda Mohammed Albin Ahmed
- Coder, Strategic designer

<img src="https://github.com/user-attachments/assets/64c0a7ef-2c39-4f24-a726-1a84b37b1a8e" width="200" height="356">

- Ibrahim Alangari
- 3D Designer, Hardware 

---
#  The Challenge

Build a small autonomous car that can perceive the track, decide, and drive itself to complete a road‑style course without human input.

**Robot objectives:**

Lane keeping & path following: Detect lane lines/edges and stay centered while maintaining a target speed.

Traffic logic: Handle intersections, turns, and course markers/signs as specified by the rules.

Obstacle handling: Detect and avoid obstacles without leaving the course.

Parking: Enter and stop in the designated zone as instructed.

---
# 🪛 Hardware

## x4 Ultrasonic Sensors (HC-SR04)
Overview: Low‑cost, 5 V ultrasonic rangefinders (approx. 2 cm–4 m). Good for simple obstacle detection when used in a front‑left / front‑center / front‑right array.

<img width="292" height="173" alt="image" src="https://github.com/user-attachments/assets/505da7b6-f95d-45e4-9498-076d326b85ef" />

### Advantages:

- Inexpensive, widely available, easy to wire (Trig/Echo) and debug with an LED/oscilloscope.

- Robust to lighting conditions; works in dark/sunlight where vision may struggle.

- Narrow beam (~15°) helps with directional obstacle cues when angled.

**How we use it:**
It gives us a rough estimation of the close-proximity area surrounding the robot so that doesnt bump into walls or obstacles accidentally, And when we use four distance sensors in four directions, we can create a map that helps the robot choose the best route for the competition.

## MPU6500 Gyro + Accelerometer Sensor
Overview: InvenSense 6‑DOF IMU for heading estimation, stabilization, and odometry aiding. Combines a 3‑axis gyroscope (±250–2000 °/s) and 3‑axis accelerometer (±2–16 g) with configurable digital low‑pass filters and up to ~1 kHz sampling via I²C or SPI.

<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/ce65d78a-7271-49cc-9851-a843849031a7" />

### Advantages

- Low‑noise gyro (for its class) with DLPF → smoother heading estimates.

- SPI support enables high‑rate reads with low latency; broad library support.

- Compact and low‑power; easy to place near the chassis center of rotation.

**How we use it:**
Used to precisely detect the position of the gyrosocpe on the field to enable smooth PID control and increased object avoidance.

## 3-6V DC motor with yellow gearbox (TT Motor)

Overview: Common plastic 1‑stage/2‑stage geared DC motor used for lightweight drivetrains.

<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/6c7bc5fb-0636-4041-81f3-535f7d388686" />

### Advantages

- Very cheap and available with many gear ratios and wheel hubs.

- Simple to drive via H‑bridge; adequate torque for small chassis.

- Good for rapid prototyping and spares during competition.

**How we use it:**
Used for movement of the rear wheels.

## BTS7960 DC motor driver
Overview: High‑current MOSFET H‑bridge built from two BTS7960 half‑bridges. Suited for a single brushed DC drive motor with low losses compared to legacy bipolar drivers. Typical modules accept ~6–27 V, with large peak currents (marketing up to ~43 A) and realistic continuous currents in the 10–15 A range with proper heatsinking and airflow.

<img width="222" height="227" alt="image" src="https://github.com/user-attachments/assets/b8c60c7d-cd66-4a00-92ea-7d9fdbfabfa8" />=

Advantages

- High efficiency & torque: Low Rds(on) MOSFETs deliver more voltage at the motor vs. IBT‑2 (BTS7960).

- Current headroom: Handles surge/stall currents better (with thermal management).

- Built‑in protections: BTS7960 includes over‑current and over‑temperature safeguards.

Straightforward control: RPWM/LPWM + R_EN/L_EN enable simple direction and speed schemes.

**How we use it:**
Used to control the DC motors.

## DS3240 Servo Motor
Overview: Metal‑gear, high‑torque PWM hobby servo (often rated ≈ 40 kg·cm @ 7.4 V). Useful for steering mechanisms or active sensor mounts.

<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/0d9cd575-ed87-4c4e-a55b-4832fca8ad4b" />

### Advantages

- Excellent stall torque for size; metal gears and aluminum case (on many variants).

- Standard 3‑wire PWM control; drops into typical RC/robot stacks.

- Operates at 6–8.4 V; good match to a dedicated 7.4 V rail.

**How we use it:**
Used to steer the two front wheels.

## PCA9685 Servo Motor Driver
Overview: I²C 12‑bit PWM expander that generates up to 16 hardware PWM channels for servos/LEDs, freeing CPU timers.

<img width="297" height="170" alt="image" src="https://github.com/user-attachments/assets/17577f3f-feb5-4f1d-a2eb-ec3b479acf66" />

### Advantages

- Stable, jitter‑free PWM independent of host CPU load.

- Chainable via I²C addresses; simple libraries on Raspberry Pi.

- 3.3 V I²C‑friendly yet can drive 5–7.4 V servo rails (separate V+).

**How we use it:**
Controls the servo motor's output.

## 7.4V Lithium-ion batteries (x2)
Overview: Two independent 2‑cell packs.

<img width="194" height="259" alt="image" src="https://github.com/user-attachments/assets/b8866c4a-5bb7-4de0-b1c8-da94acc2b21c" />

### Advantages

- Electrical isolation reduces brown‑outs and noise coupling into the Raspberry Pi.

- Native 7.4 V matches high‑torque servos (e.g., DS3240) without heavy conversion losses.

- Modular swaps: replace a depleted pack without shutting down the entire robot.

- Fault containment: an issue on one rail is less likely to take down everything.

**How we use it:**
Power for the whole robot.

## XL4015 Step-down Converter
Overview: Adjustable buck module (typically up to 5 A) to derive stable 12 V/9 V/5 V rails from the traction battery.

<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/0819eb18-e5f9-4616-944b-c28641d61844" />

### Advantages

- High current capacity for price; efficiency far better than linear regulators.

- Screw‑terminal/trim‑pot modules are quick to integrate; common footprints.

- Often includes CC (constant‑current) mode useful for LED loads or battery precharge.

**How we use it:**
It lowers voltage coming from the batteries from 7.4V to 5V so the raspberry pi doesnt get overloaded.

## Raspberry Pi Camera Module 3

Overview: Autofocus 12 MP camera (Sony IMX708 family) with wide/standard FOV options and HDR variants. Primary sensor for vision and lane detection.

<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/910f6cb8-42ac-475b-a819-7c7838971d42" />

### Advantages

- Autofocus improves clarity for varying distances and vibration.

- Great color detection and masking, useful for obstacles.

- High resolution enables cropping/ROI; good low‑light vs older modules.

- Mature drivers and community support; works seamlessly with Raspberry Pi 5.

**How we use it:**
We are using AI vision on the camera to avoid obstacles with it's color detection capability and to calculate how far an object is by counting the amount of pixels it's color occupies on the camera's FOV.

## PCA9685 16‑Channel Servo Driver

Overview: I²C 12‑bit PWM expander that generates up to 16 hardware PWM channels for servos/LEDs, freeing CPU timers.

<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/bc5fc2f5-e839-4dc8-bb82-bed123f98905" />

Advantages

- Stable, jitter‑free PWM independent of host CPU load.

- Chainable via I²C addresses; simple libraries on Raspberry Pi.

- 3.3 V I²C‑friendly yet can drive 5–7.4 V servo rails (separate V+).

**How we use it:**
The MPU6500 gyro sensor, the BTS7960 DC motor driver, and the PCA9685 servo motor driver use I2C channels and there arent enough on the raspberry pi so this was the best solution.

## Raspberry Pi 5

Overview: Quad‑core Cortex‑A76 SBC with significant CPU/GPU uplift, dual 4K HDMI, PCIe, and improved I/O—well‑suited for real‑time perception/planning.

<img width="275" height="183" alt="image" src="https://github.com/user-attachments/assets/5fcf4a80-e49a-4dab-ad96-da1dc55fc80b" />

### Advantages

- Big performance headroom for OpenCV, TensorRT‑style inference, and Python control.

- Fast I/O (PCIe, higher USB throughput) for cameras, IMUs, and storage.

- Strong ecosystem: libraries, guides, and accessories.

**How we use it:**
The raspberry pi is the brain of our system, it recieves input from sensors to then act upon it by outputing signals to the actuators. It's great since it has a very high response time as well as being compatible with the camera's AI vision and the rest of the sensors and actuators.

--- 
# Camera-Based Path Detection

The PiCamera2 continuously captures frames at 640×480 resolution.
Each frame is processed in OpenCV to detect the open path or track using brightness and edge patterns.
The image is converted to grayscale, blurred, and filtered using Canny edge detection.
The largest contour or lane is identified as the path.
The robot calculates the center of the path (cx):
If cx is left of center → steer left.
If cx is right of center → steer right.
If centered → move straight forward.
This allows Tuwaiqy to visually follow the safest visible route.

---
# Ultrasonic Obstacle Detection

While the camera defines the path, the ultrasonic sensors confirm whether that direction is clear and safe:

front = front_ultra.distance_cm

left  = left_ultra.distance_cm

right = right_ultra.distance_cm

back  = back_ultra.distance_cm

Decision Logic:

if front < 15 cm:

    stop()
    
    if left > right:
    
        turn_left()
        
    else:
    
        turn_right()
        
elif left < 10 cm:

    steer_right()
    
elif right < 10 cm:

    steer_left()
    
else:

    move_forward()
    
The ultrasonic readings take priority for collision prevention, even if the camera sees a clear path.

---
# Combined Behavior

Camera detects path → provides general direction.

Ultrasonic sensors verify → confirm no nearby obstacle in that direction.

Raspberry Pi 5 fuses both data sources:

- If both camera and sensors agree → move smoothly forward.

- If camera sees open path but ultrasonic detects something close → slow down or stop.

- If sensors detect clear space but no visible path → rely on previous frame or turn slightly to search again.

- Commands are sent through PCA9685 servo driver (for steering) and BTS7960 motor driver (for movement).

## Advantages
 
- Reliable in all conditions: Camera detects overall layout, ultrasonic handles near obstacles.

- Improved accuracy: Fewer false positives or collisions.

- Real-time reaction: Sensors update multiple times per second.

- Light-independent: Works in different lighting or surface colors.

- Smooth navigation: Camera ensures continuous forward motion instead of random turns.

## Disadvantages
 
- More complex integration: Needs synchronization between camera and sensors.

- Processing load: Image processing consumes CPU power.

- Possible interference: Echoes from nearby surfaces can affect readings.

- Calibration required: Must align camera and sensor fields of view carefully.

## Result
 
With this dual-system setup, Tuwaiqy can both see and sense its environment. The camera provides long-range path guidance, while the ultrasonic sensors offer short-range precision safety. This combination makes the robot’s navigation smarter, safer, and more adaptive, aligning perfectly with the WRO Future Engineers challenge goal of developing advanced autonomous mobility systems.
