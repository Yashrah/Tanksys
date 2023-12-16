import RPi.GPIO as GPIO
import time
import threading

# Ultrasonic Sensor Pins
triggerPin = 37
echoPin = 36

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(triggerPin, GPIO.OUT)
GPIO.setup(echoPin, GPIO.IN)

# Shared variable for distance
distance = 0.0
lock = threading.Lock()

def measure_distance():
    global distance
    while True:
        # Set Trigger to HIGH
        GPIO.output(triggerPin, True)

        # Set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(triggerPin, False)

        start_time = time.time()
        stop_time = time.time()

        # Save StartTime
        while GPIO.input(echoPin) == 0:
            start_time = time.time()

        # Save time of arrival
        while GPIO.input(echoPin) == 1:
            stop_time = time.time()

        # Time difference between start and arrival
        time_elapsed = stop_time - start_time
        # Multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance_measured = (time_elapsed * 34300) / 2

        with lock:
            distance = distance_measured

def start_measuring():
    threading.Thread(target=measure_distance, daemon=True).start()

def get_distance():
    with lock:
        return distance

if __name__ == "__main__":
    try:
        start_measuring()
        while True:
            time.sleep(1)
            print(f"Distance: {get_distance():.1f} cm")
    except KeyboardInterrupt:
        GPIO.cleanup()
