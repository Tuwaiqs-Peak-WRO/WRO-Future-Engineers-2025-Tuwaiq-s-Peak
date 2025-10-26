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

i2c = board.I2C()
tof_sensor = adafruit_vl53l0x.VL53L0X(i2c)

import time

DEFAULT_SPEED = 0.35
DEFAULT_REVERSE_SPEED = 0.3

motor = create_dc_motor()
servo = create_servo(channel=0, max_pulse=2500, min_pulse=500, reverse_angle=True,center_fix = 6, max_angle=30)
bno = creat_bno_sensor()

def _get_raw_rc_angle(bno):
        """Get current raw angle from sensor."""
        sensor_data = bno.get_sensor_data()
        if not sensor_data:
            print("no gyro data")
            return None
        yaw_angle_raw = sensor_data['gyro_angles'][2]
        yaw_angle_raw
        return yaw_angle_raw

def get_rc_angle(bno):
        yaw_angle = _get_raw_rc_angle(bno)
        yaw_angle = np.degrees(yaw_angle)
        yaw_angle = (yaw_angle + 180) % 360 - 180
        return yaw_angle

class stuck_checker:
    def __init__(self):
         pass
         self.currTime = time.time()
         self.similarity_start_time = 0
         self.previous_frame = None
    def frames_are_similar(self, frame1, frame2, threshold):
        if frame1 is None or frame2 is None:
            return False
        # Convert to grayscale for simpler comparison
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

        # Compute structural similarity
        difference = cv2.absdiff(gray1, gray2)
        _, diff_thresh = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY)

        # Calculate the ratio of non-difference pixels
        non_zero_count = np.count_nonzero(diff_thresh)
        total_pixels = frame1.shape[0] * frame1.shape[1]
        similarity_ratio = (total_pixels - non_zero_count) / total_pixels

        return similarity_ratio >= threshold
    
    def check_if_stuck( self, frame, similarity_threshold = 0.9, similarity_duration = 3):
        # Check for frame similarity over time (car is stuck)
        # print('in check')
        self.currTime = time.time()
        if self.previous_frame is None:
             self.previous_frame = frame.copy()
             return False
        
        # print("HHHHH")
        if self.frames_are_similar(self.previous_frame, frame, threshold=similarity_threshold):
            if self.similarity_start_time == 0:
                self.similarity_start_time = self.currTime
            elif self.currTime - self.similarity_start_time >= similarity_duration :
                print("stuck")
                
                self.similarity_start_time = 0  # Reset the timer
                return True
            else:
                pass
                # print("checking stuck..")
        else:
            # Reset similarity timer if the frames are not similar
            self.similarity_start_time = 0
            self.previous_frame = frame.copy()
            return False
        
    

def main():

    servo.center()
    initial_point = -1
    initial_angle = 0
    prev_wall_point = -1
    counter = 0
    prev_time_counter = time.time()
    prev_time_SteerLock = time.time()
    lab_count = 0
    start_counter = True
    num_points_avg = 60 # How many points from the trace to avg (magic number)
    steerLock_duration = 0
    stuck_checker_obj = stuck_checker()
    steer_angle = 0
    steer_angle_buffer = collections.deque(maxlen=3) #Increase this for smoother steering but too much will cause some steer delay
    run_speed = DEFAULT_SPEED
    def_offset_fix_factor =0
    offset_fix_factor = def_offset_fix_factor # to compensate quircky track , rais above 0 if bad ccw and lower for bad cw
    max_offset = 40
    cw = None
    last_obs_before_revere = ""
    try:
        while True:
            front_dist = tof_sensor.range
            rc_angle = get_rc_angle(bno)
            
            curr_time = time.time()
            frame = picam2.capture_array()
            
            offset_steer, wall_point, comp_img, flag = process_frame(frame.copy(), num_points_avg)
            
            has_display = True
            if os.environ.get('DISPLAY') is None:
                has_display = False

            if has_display:
                cv2.imshow("preview", comp_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
                 
            if steerLock_duration:
                 if curr_time - prev_time_SteerLock < steerLock_duration:
                    continue 
                 else:
                    motor.set_speed(0)
                    steerLock_duration = 0
            
            if stuck_checker_obj.check_if_stuck(frame) or front_dist < 100:
                print(f"backward at front dist {front_dist}")
                prev_time_counter = curr_time
                steerLock_duration = 1.5 #seconds to go backward
                prev_time_SteerLock = curr_time
                
                if cw is None:
                    smoothed_steering_angle  = sum(steer_angle_buffer)/len(steer_angle_buffer)
                    steer_angle = -smoothed_steering_angle * 0.8
                else:
                    steer_angle = 20 if cw else -20
                servo.set_angle(steer_angle)
                if run_speed:
                    motor.set_speed(-DEFAULT_REVERSE_SPEED)
                continue
            if len(flag):
                # print(flag)
                if "obs" in flag:
                    # max_offset = 20
                    run_speed = 0.3
                    offset_fix_factor = 0

                    if "obs clearance" in flag:

                        if -10 < rc_angle * (1 if cw else -1) < 100 and counter==1:
                            if "red" in flag:
                                last_obs_before_revere = "red"
                            elif "green" in flag:
                                last_obs_before_revere = "green"
                            # print("reverse nowwwwwww")
                        #flip cw
                        # cw = not cw
                        # # # hard steer to right
                        # steer_angle = servo.max_angle
                        # servo.set_angle(steer_angle)
                        # run_speed = 0.3
                        # motor.set_speed(run_speed)
                        # #reinitial angle
                        # initial_angle = 180
                        # steerLock_duration = 1.5
                        # continue

            else:
                if front_dist < 200:
                    # print('hard steer')
                    steerLock_duration = 1
                    run_speed = 0.3
                    steer_angle = -servo.max_angle if cw else servo.max_angle
                    servo.set_angle(steer_angle)
                    if run_speed:
                        motor.set_speed(run_speed)
                    continue
                else:
                    max_offset = 40
                    offset_fix_factor = def_offset_fix_factor
                    run_speed = DEFAULT_SPEED
           
            # if cw is not None:
            #     offset_steer += -offset_fix_factor if cw else offset_fix_factor
            offset_steer = min(max_offset, max(-max_offset, offset_steer))
            steer_ratio = offset_steer / max_offset
            steer_angle = int(steer_ratio*servo.max_angle)
            steer_angle_buffer.append(steer_angle)
            smoothed_steering_angle  = sum(steer_angle_buffer)/len(steer_angle_buffer)
            
            servo.set_angle(smoothed_steering_angle)
            # print("rc angle:",rc_angle)
            if cw is None:
                # if smoothed_steering_angle >= servo.max_angle*0.8:
                if rc_angle > 80:
                    print("round is ccw")
                    cw = False
                # elif smoothed_steering_angle <= -servo.max_angle*0.8:
                elif rc_angle < -80:
                    print("round is cw")
                    cw = True
            motor.set_speed(run_speed)
            
            #counting labs section
            if wall_point is not None:
                if initial_point < 0 and wall_point[1] < 35:
                        prev_time_counter = curr_time
                        initial_point = wall_point[1]
                        # initial_point = front_dist
                        initial_angle = 0
                        # prev_wall_point = 1000
                        print(initial_point)
                if curr_time - prev_time_counter > 10:
                    # print(rc_angle)
                    if abs(abs(rc_angle)-initial_angle) < 15 :
                        # print(wall_point)
                        # if wall_point[1] > 30:
                        print("angle at count section:",rc_angle)
                        print("midpoint white pixels:",wall_point[1])
                        # print("dist at count s8ection:", front_dist)
                        # if (abs(initial_point - front_dist) < 20):
                        if (wall_point[1] >= initial_point ):
                            if start_counter:
                                counter+=1
                                print("counter: ", counter)
                                if counter and last_obs_before_revere == "red":
                                    print("reverse nowwwwwww")
                                if counter == 3:
                                    motor.set_speed(-0.1)#brakes
                                    time.sleep(0.2)
                                    break
                                prev_time_counter = curr_time

                            start_counter = False
                        prev_wall_point = front_dist
                    else:
                         start_counter = True
                         prev_wall_point = 0
                    
        
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        servo.center()
        servo.unlock()
        motor.stop()
        


ip_address = get_local_ip()
oled.display_info([ip_address,"Obistcale"])

has_display = True
if os.environ.get('DISPLAY') is None:
    has_display = False


if __name__ == "__main__":
    if not has_display:
        while(not button.is_pressed):
            k = 1
    button.close()

    try:
        # Initialize Picamera2
        picam2 = Picamera2()
        RESIZE_FACTOR = 1
        # picam2.sensor_mode = 4
        #2304 x 1296-PC1B
        cam_res = (2304 // RESIZE_FACTOR, 1296 // RESIZE_FACTOR)

        video_config = picam2.create_video_configuration(raw={"size": cam_res, "format" : 'SGBRG10'}, main = {"size":(2304//10, 1296//10)})
        picam2.configure(video_config)
        picam2.start()

        load_hsv_from_json()
        main()
    except Exception as e:
        print(f"An error occurred: {e}")
        picam2.stop()
        cv2.destroyAllWindows()
        servo.center()
        time.sleep(2)
        servo.unlock()
        motor.stop()
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        servo.center()
        time.sleep(2)
        servo.unlock()
        motor.stop()