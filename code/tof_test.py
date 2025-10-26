import adafruit_vl53l0x
import board
import time

i2c = board.I2C()
tof_sensor = adafruit_vl53l0x.VL53L0X(i2c)
tof_sensor.measurement_timing_budget = 200000

while True:
    # Take 5 readings and filter out 8190 values
    readings = []
    for _ in range(5):
        distance = tof_sensor.range
        if distance < 6190:  # Only add valid readings
            readings.append(distance)
        time.sleep(0.01)
    
    if readings:  # Check if we have any valid readings
        average_dist = sum(readings) / len(readings)
        print(f"Individual readings: {readings}")
        print(f"Average distance: {average_dist:.1f}mm")
    else:
        print("No valid readings obtained")
    print("-" * 40)
    time.sleep(0.1)