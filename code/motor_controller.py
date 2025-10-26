from gpiozero import PWMOutputDevice, Buzzer,DigitalOutputDevice

buzzer = Buzzer(23)
class DCMotorController:
    def __init__(self, forward_pin=17, reverse_pin=27,servo_oe_pin = 10):
        self.motor_forward = PWMOutputDevice(forward_pin)
        self.motor_reverse = PWMOutputDevice(reverse_pin)
        self.oe_pin = DigitalOutputDevice(servo_oe_pin, active_high=True, initial_value=False)
        

    def set_speed(self, speed):
        """
        Set the motor speed and direction.
        :param speed: Float between -1 and 1. Negative for reverse, positive for forward.
        """
        if speed < -1 or speed > 1:
            raise ValueError("Speed must be between -1 and 1")

        if speed < 0:
            self.motor_forward.value = -speed
            self.motor_reverse.off()
        elif speed > 0:
            self.motor_reverse.value = speed
            self.motor_forward.off()
        else:
            self.motor_forward.off()
            self.motor_reverse.off()

    def stop(self):
        """Stop the motor."""
        self.motor_forward.off()
        self.motor_reverse.off()

    def disable_servo(self):
        """Disable servo outputs using OE pin"""
        self.oe_pin.on()  # Set HIGH to disable
        print("Servo outputs disabled")  # Debug message
        
    def enable_servo(self):
        """Enable servo outputs using OE pin"""
        self.oe_pin.off()  # Set LOW to enable
        print("Servo outputs enabled")  # Debug message
        
    def cleanup(self):
        """Clean up GPIO resources"""
        try:
            self.stop()
            self.disable_servo()
            self.motor_forward.close()
            self.motor_reverse.close()
            self.oe_pin.close()
        except:
            pass
        


    @staticmethod
    def buzzer_on():
        buzzer.on()
   
    @staticmethod
    def buzzer_off():
        buzzer.off()

def create_dc_motor(forward_pin=17, reverse_pin=27, servo_oe_pin=10):
    """Create and return a DCMotorController instance."""
    return DCMotorController(forward_pin, reverse_pin, servo_oe_pin)

