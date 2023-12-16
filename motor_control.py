mport RPi.GPIO as GPIO
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
Kp = 1.0
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
    integral = integral + error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

def set_motor_speed(pwm_value):
    pwmA.ChangeDutyCycle(pwm_value)
    pwmB.ChangeDutyCycle(pwm_value)

def startMotors():
    pwmA.start(0)
    pwmB.start(0)

def stop():
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    GPIO.output(motorA2, GPIO.LOW)
    GPIO.output(motorB2, GPIO.LOW)

def forward(pwm_value):
    set_motor_speed(pwm_value)
    GPIO.output(motorA2, GPIO.HIGH)
    GPIO.output(motorB2, GPIO.HIGH)

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

    # Start emergency stop check in a separate thread
    stop_thread = threading.Thread(target=emergency_stop_check)
    stop_thread.start()

    try:
        while True:
            if emergency_stop:
                stop()
                break

            # Read sensor data
            distance = readDistance.get_distance()
            com = camera.get_center_of_mass()

            # Stop if too close to an object
            if distance <= safety_distance:
                stop()
                continue

            # Assuming center of image is (320, 240) for a 640x480 resolution
            if com is not None:
                center_x, center_y = com
                error = 320 - center_x  # Calculate deviation from the center
                correction = pid_control(error)

                # Adjust motor speed based on PID correction
                forward(optimal_pwm + correction)
            else:
                forward(optimal_pwm)

            time.sleep(0.1)  # Adjust as needed for your control loop timing

    except KeyboardInterrupt:
        pass
    finally:
        stop()
        GPIO.cleanup()

if __name__ == "__main__":
    camera.start_camera_thread() 
    control_loop()
