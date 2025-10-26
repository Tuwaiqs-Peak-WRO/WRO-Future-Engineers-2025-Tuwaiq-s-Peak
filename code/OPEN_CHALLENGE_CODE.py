import cv2
import numpy as np
from picamera2 import Picamera2
from process_frames_colors import *
from motor_controller import create_dc_motor
from servo_controller import create_servo
from BnoSensor_controller import creat_bno_sensor
import collections
import adafruit_vl53l0x
import board
from oledInfo import *
import time
import os

i2c = board.I2C()
tof_sensor = adafruit_vl53l0x.VL53L0X(i2c)

# ================== CONFIG ==================
DEFAULT_SPEED = 0.35
DEFAULT_REVERSE_SPEED = 0.3
MIN_FRONT_DIST = 20  # emergency stop when <= 20mm

motor = create_dc_motor()
servo = create_servo(channel=0, max_pulse=2500, min_pulse=500, reverse_angle=True, center_fix=6, max_angle=25)
bno = creat_bno_sensor()

# ================== FUNCTIONS ==================
def _get_raw_rc_angle(bno):
    sensor_data = bno.get_sensor_data()
    if not sensor_data:
        print("no gyro data")
        return None
    yaw_angle_raw = sensor_data['gyro_angles'][2]
    return yaw_angle_raw

def get_rc_angle(bno):
    yaw_angle = _get_raw_rc_angle(bno)
    yaw_angle = np.degrees(yaw_angle)
    yaw_angle = (yaw_angle + 180) % 360 - 180
    return yaw_angle

class stuck_checker:
    def _init_(self):
        self.similarity_start_time = 0
        self.previous_frame = None
        self.prev_gray = None
        self.prev_pts = None
        self.flow_start_time = 0
        self.similarity_threshold = 0.92
        self.similarity_duration = 2.5
        self.flow_mag_threshold = 0.6
        self.flow_duration = 1.5

    def frames_are_similar(self, frame1, frame2, threshold=None):
        if threshold is None:
            threshold = self.similarity_threshold
        if frame1 is None or frame2 is None:
            return False
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        difference = cv2.absdiff(gray1, gray2)
        _, diff_thresh = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY)
        non_zero_count = np.count_nonzero(diff_thresh)
        total_pixels = gray1.size
        similarity_ratio = (total_pixels - non_zero_count) / total_pixels
        return similarity_ratio >= threshold

    def compute_optical_flow_mag(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_pts = cv2.goodFeaturesToTrack(gray, None, 200, 0.01, 7)
            return None
        pts1 = self.prev_pts
        if pts1 is None:
            pts1 = cv2.goodFeaturesToTrack(self.prev_gray, None, 200, 0.01, 7)
            self.prev_pts = pts1
            if pts1 is None:
                self.prev_gray = gray
                return None
        pts2, st, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, pts1.astype(np.float32), None)
        if pts2 is None or st is None:
            self.prev_gray = gray
            self.prev_pts = None
            return None
        good = st.flatten() == 1
        if good.sum() == 0:
            self.prev_gray = gray
            self.prev_pts = None
            return None
        mags = np.linalg.norm(pts2[good] - pts1[good], axis=1)
        avg = float(np.mean(mags))
        self.prev_gray = gray
        self.prev_pts = cv2.goodFeaturesToTrack(gray, None, 200, 0.01, 7)
        return avg

    def check_if_stuck(self, frame, similarity_threshold=None, similarity_duration=None):
        now = time.time()
        if similarity_threshold is None:
            similarity_threshold = self.similarity_threshold
        if similarity_duration is None:
            similarity_duration = self.similarity_duration

        if self.previous_frame is None:
            self.previous_frame = frame.copy()
            return False

        if self.frames_are_similar(self.previous_frame, frame, similarity_threshold):
            if self.similarity_start_time == 0:
                self.similarity_start_time = now
            elif now - self.similarity_start_time >= similarity_duration:
                print("STUCK: similarity")
                self.similarity_start_time = 0
                return True
        else:
            self.similarity_start_time = 0
            self.previous_frame = frame.copy()

        avg_flow = self.compute_optical_flow_mag(frame)
        if avg_flow is not None:
            if avg_flow < self.flow_mag_threshold:
                if self.flow_start_time == 0:
                    self.flow_start_time = now
                elif now - self.flow_start_time >= self.flow_duration:
                    print(f"STUCK: optical flow avg={avg_flow:.2f}")
                    self.flow_start_time = 0
                    return True
            else:
                self.flow_start_time = 0

        return False

# ================== MAIN CONTROL ==================
def main():
    servo.center()
    stuck_checker_obj = stuck_checker()
    steer_angle_buffer = collections.deque(maxlen=3)
    last_actions = collections.deque(maxlen=200)
    cw = None

    def record_action(steer, speed):
        last_actions.append({'steer': float(steer), 'speed': float(speed), 'time': time.time()})

    def reverse_last_steps(n=3):
        if len(last_actions) == 0:
            return
        actions = list(last_actions)[-n:]
        actions.reverse()
        for a in actions:
            servo.set_angle(-a['steer'])
            motor.set_speed(-abs(a['speed']) if a['speed'] != 0 else -DEFAULT_REVERSE_SPEED)
            time.sleep(0.18)
        motor.set_speed(0)
        servo.center()

    while True:
        front_dist = tof_sensor.range
        frame = picam2.capture_array()
        rc_angle = get_rc_angle(bno)

        # EMERGENCY STOP
        if front_dist <= MIN_FRONT_DIST:
            print("EMERGENCY REVERSE: Too close!")
            motor.set_speed(0)
            reverse_last_steps()
            continue

        # STUCK CHECK
        if stuck_checker_obj.check_if_stuck(frame) or front_dist < 100:
            print("RECOVERY: stuck detected!")
            reverse_last_steps()
            motor.set_speed(0.12)
            time.sleep(0.25)
            motor.set_speed(0)
            continue

        offset_steer, wall_point, comp_img, flag = process_frame(frame.copy(), 60)

        # Steering
        max_offset = 40
        offset_steer = min(max_offset, max(-max_offset, offset_steer))
        steer_angle = int((offset_steer / max_offset) * servo.max_angle)
        steer_angle_buffer.append(steer_angle)
        steer_angle = int(sum(steer_angle_buffer) / len(steer_angle_buffer))
        servo.set_angle(steer_angle)
        record_action(steer_angle, DEFAULT_SPEED)

        if cw is None:
            if rc_angle > 80:
                cw = False
            elif rc_angle < -80:
                cw = True

        motor.set_speed(DEFAULT_SPEED)

        if os.environ.get('DISPLAY'):
            cv2.imshow("preview", comp_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# ================== ENTRY ==================
ip_address = get_local_ip()
oled.display_info([ip_address, "Obstacle"])

if _name_ == "_main_":
    picam2 = Picamera2()
    cam_res = (2304 // 1, 1296 // 1)
    video_config = picam2.create_video_configuration(raw={"size": cam_res}, main={"size": (2304//10, 1296//10)})
    picam2.configure(video_config)
    picam2.start()

    load_hsv_from_json()

    try:
        main()
    except Exception as e:
        print(e)
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        servo.center()
        servo.unlock()
        motor.stop()
