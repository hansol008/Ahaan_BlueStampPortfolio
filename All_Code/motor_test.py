import RPi.GPIO as GPIO
import cv2
import numpy as np

GPIO.setmode(GPIO.BCM)

MOTOR1B = 6
MOTOR1E = 5

MOTOR2B = 22
MOTOR2E = 23

GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(MOTOR1E, GPIO.OUT)

GPIO.setup(MOTOR2B, GPIO.OUT)
GPIO.setup(MOTOR2E, GPIO.OUT)

print("Motor Control Program Started.")
print("Commands: 'w' (forward), 'a' (left), 's' (backward), 'd' (right), 'x' (stop).")
print("Press Ctrl+C to exit.")

try:
    while True:
        userInput = input("Enter command: ").strip().lower()

        if userInput == 'w':
            print("Moving Forward")
            GPIO.output(MOTOR1B, GPIO.HIGH)
            GPIO.output(MOTOR1E, GPIO.LOW)
            GPIO.output(MOTOR2B, GPIO.HIGH)
            GPIO.output(MOTOR2E, GPIO.LOW)

        elif userInput == 'a':
            print("Turning Left")
            GPIO.output(MOTOR1B, GPIO.LOW)
            GPIO.output(MOTOR1E, GPIO.LOW)
            GPIO.output(MOTOR2B, GPIO.HIGH)
            GPIO.output(MOTOR2E, GPIO.LOW)

        elif userInput == 's':
            print("Moving Backward")
            GPIO.output(MOTOR1B, GPIO.LOW)
            GPIO.output(MOTOR1E, GPIO.HIGH)
            GPIO.output(MOTOR2B, GPIO.LOW)
            GPIO.output(MOTOR2E, GPIO.HIGH)

        elif userInput == 'd':
            print("Turning Right")
            GPIO.output(MOTOR1B, GPIO.HIGH)
            GPIO.output(MOTOR1E, GPIO.LOW)
            GPIO.output(MOTOR2B, GPIO.LOW)
            GPIO.output(MOTOR2E, GPIO.LOW)

        elif userInput == 'x':
            print("Stopping All Motors")
            GPIO.output(MOTOR1B, GPIO.LOW)
            GPIO.output(MOTOR1E, GPIO.LOW)
            GPIO.output(MOTOR2B, GPIO.LOW)
            GPIO.output(MOTOR2E, GPIO.LOW)
        else:
            print("Invalid command. Please use 'w', 'a', 's', 'd', or 'x'.")

except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected. Exiting program gracefully.")
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
finally:
    GPIO.cleanup()
    print("GPIO cleanup complete. Program terminated.")
