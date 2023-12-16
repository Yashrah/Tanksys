import RPi.GPIO as GPIO
import time
import threading
import camera_processing as camera
import readDistance

# Motor Pin Definitions
motorA1 = 18
motorA2 = 16
motorB1 = 13
motorB2 = 11

# Optimal PWM Value
optimal_pwm = 60

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup([motorA1, motorA2, motorB1, motorB2], GPIO.OUT)

# Create PWM instances
pwmA = GPIO.PWM(motorA1, 1000)  # PWM frequency of 1000 Hz
pwmB = GPIO.PWM(motorB1, 1000)

# PID Controller Parameters
Kp = 0.4
Ki = 0.0
Kd = 0.0

# PID Controller State
previous_error = 0
integral = 0

# Safety Distance Threshold (in centimeters)
safety_distance = 30

# Emergency Stop Flag
emergency_stop = False

def pid_control(error):
    global previous_error, integral
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

def set_motor_speed(left_pwm, right_pwm):
    pwmA.ChangeDutyCycle(max(min(left_pwm, 100), 0))  # Constrain PWM to 0-100%
    pwmB.ChangeDutyCycle(max(min(right_pwm, 100), 0))

def startMotors():
    pwmA.start(0)
    pwmB.start(0)

def stop():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output([motorA2, motorB2], GPIO.LOW)

def forward(left_pwm, right_pwm):
    set_motor_speed(left_pwm, right_pwm)
    GPIO.output([motorA2, motorB2], GPIO.HIGH)

def emergency_stop_check():
    global emergency_stop
    while True:
        command = input("Enter 'STOP' to halt the platform: ")
        if command.lower() == 'stop':
            emergency_stop = True
            break

def control_loop():
    startMotors()
    readDistance.start_measuring()
    camera.start_camera_thread()

    stop_thread = threading.Thread(target=emergency_stop_check)
    stop_thread.start()

    try:
        while True:
            if emergency_stop:
                stop()
                break

            distance = readDistance.get_distance()
            com = camera.get_center_of_mass()

            if distance <= safety_distance:
                stop()
                continue

            if com is not None:
                center_x, center_y = com
                error = 320 - center_x
                correction = pid_control(error)
                left_pwm = optimal_pwm - correction
                right_pwm = optimal_pwm + correction
            else:
                left_pwm = right_pwm = optimal_pwm

            forward(left_pwm, right_pwm)

            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        stop()
        GPIO.cleanup()

if __name__ == "__main__":
    camera.start_camera_thread() 
    control_loop()
