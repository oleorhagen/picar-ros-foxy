import RPi.GPIO as GPIO

# motor_EN_B: Pin11
#    |  motor_B: Pin13,Pin12

# motor_EN_A: Pin7  |
# motor_A:  Pin8,Pin10

# EN(able) motor B - Setting pin 17 high enables driver channel _
Motor_B_EN = 17
# Motor driver inputs
# Two control inputs for each motor
Motor_B_Pin1 = 27
Motor_B_Pin2 = 18

# States
stop = 1
forward = 2
backward = 3


class Motor:

    state = stop

    def __init__(self, enable_pin, pin1, pin2):
        self.enable_pin = enable_pin
        self.pin1 = pin1
        self.pin2 = pin2
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.enable_pin, GPIO.OUT)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        self.pwm = GPIO.PWM(self.enable_pin, 1000)

    def forward(self, speed):
        if self.state == forward:
            GPIO.output(self.pin1, GPIO.LOW)
            GPIO.output(self.pin2, GPIO.HIGH)
            self.pwm.ChangeDutyCycle(speed)
            return
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)
        self.pwm.start(0)
        self.pwm.ChangeDutyCycle(speed)
        self.state = forward

    def backward(self, speed):
        if self.state == backward:
            self.pwm.ChangeDutyCycle(speed)
            return
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW)
        self.pwm.start(0)
        self.pwm.ChangeDutyCycle(speed)
        self.state = backward

    def stop(self, speed):
        GPIO.output(self.self.pin1, GPIO.LOW)
        GPIO.output(self.self.pin2, GPIO.LOW)
        GPIO.output(self.enable_pin, GPIO.LOW)
        self.state = stop


class MotorController:
    """A simple wrapper class for controlling the PWM motor on the PiCar Pro

    goes from [-100, 100]
    """

    def __init__(self):  # Motor initialization
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.left = Motor(enable_pin=4, pin1=26, pin2=21)
        self.right = Motor(enable_pin=17, pin1=27, pin2=18)

    def forward(self, speed):
        if speed <= 0:
            raise Exception("Trying to set a negative speed on a forward command")
        self.left.forward(speed)
        self.right.forward(speed)

    def backward(self, speed):
        if speed >= 0:
            raise Exception("Trying to set a positive speed on a backward command")
        self.left.backward(speed)
        self.right.backward(speed)

    def stop(self):
        self.left.stop()
        self.right.stop()

class Steer:

    def __init__(self):
        # Import the library used to communicate with PCA9685
        import Adafruit_PCA9685

        import time

        # Instantiate the object used to control the PWM
        # Has a cycle of approximately:
        # 0 deg - 100 Hz
        # 180 deg - 560 Hz

        # TODO
        # Map this using:
        # f: [a,b] -> [c,d]
        # f(x) = c + d-c/b-a * (x-a)

        def f(x):
            return int(100 + ((560 - 100)/(180 - 0)) * (x - 0))
        pwm = Adafruit_PCA9685.PCA9685()

        self.pwm = pwm

        # Set the frequency of the PWM signal
        pwm.set_pwm_freq(50)


        # Make the servo connected to the No. 3 servo port on the RobotHAT
        # drive board reciprocate
        # while True:
        #     pwm.set_pwm(0, 0, 300)
        #     time.sleep(1)

        #     pwm.set_pwm(0, 0, 400)

        #     time.sleep(1)

        #     pwm.set_pwm(0, 0, f(60))

    def f(self, x):
        return int(100 + ((560 - 100)/(180 - 0)) * (x - 0))

    def set(self, angle):
        if not 20 <= angle + 90 <= 160:
            raise Exception(f"Angle not allowed: {angle}")
        self.pwm.set_pwm(0, 0, self.f(angle+90))




if __name__ == "__main__":
    # mc = MotorController()
    # mc.forward(100)
    import time
    # time.sleep(10)
    # mc.stop()
    steer = Steer()
    time.sleep(2)
    steer.set(120)
    pass
