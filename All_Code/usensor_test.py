import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIG_PIN = 19
ECHO_PIN = 26

GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    # Ensure the trigger pin is low initially
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.000002) # Short delay to ensure pulse is clean

    # Trigger the sensor
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001) # 10 us pulse
    GPIO.output(TRIG_PIN, GPIO.LOW)

    pulse_start = time.time()
    pulse_end = time.time()

    timeout_start = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start = time.time()
        if time.time() - timeout_start > 0.1:
            return -1

    # Wait for the ECHO pin to go LOW (pulse end)
    timeout_end = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end = time.time()
        if time.time() - timeout_end > 0.1: # 100ms timeout for waiting for low
            return -1 # Indicate an error or no object detected

    pulse_duration = pulse_end - pulse_start

    # Speed of sound is approx 343 meters/second = 34300 cm/second
    # Distance = (time * speed) / 2 (because sound travels there and back)
    distance = (pulse_duration * 34300) / 2
    distance = round(distance, 2)

    return distance

try:
    while True: # You might want to continuously read the distance
        dist = get_distance()
        if dist != -1:
            print(f"Object is at {dist} cm from the ultrasonic sensor")
        else:
            print("No object detected or sensor error (check wiring/obstacles).")
        time.sleep(0.5) # Read every half second
except KeyboardInterrupt:
    print("\nProgram interrupted by user. Cleaning up GPIO...")
finally:
    GPIO.cleanup()
    print("GPIO cleanup complete. Program terminated.")
