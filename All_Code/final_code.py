#v 1.6 with backing up and status led
import cv2
from picamera2 import Picamera2
import time
import numpy as np
import RPi.GPIO as GPIO

# Add GPIO.cleanup() here to ensure a clean slate on every run
GPIO.cleanup() # <--- ADD THIS LINE

# --- Configuration Constants ---
# Sensor Proximity (centimeters)
SENSOR_PROXIMITY = 2
REROUTING_PROXIMITY = 17.5
DISTANCE_THRESHOLD = 2    # cm: distance at which the robot should stop
BACKUP_DIST = 12

# Robot Behavior Constants
CENTER_TOLERANCE = 160    # Pixels: Defines a 40-pixel dead zone for straight movement
TARGET_CONTOUR_AREA_MIN = 2500    # Minimum pixel area for the ball
TARGET_CONTOUR_AREA_MAX = 110000  # Maximum pixel area for the ball
PARKED_AREA_THRESHOLD = 10000     # Area threshold for the 'parked' state

# Movement Delays (in seconds)
# Adjust these values to control how long each action is performed.
# Shorter delays: more responsive, potentially more erratic.
# Longer delays: smoother, less responsive.
FORWARD_DELAY = 0.1 # Reduced from 0.15
TURN_DELAY = 0.02   # Reduced from 0.1
REVERSE_DELAY = 0.1     # Reduced from 0.2
REROUTE_TURN_DELAY = 0.1  # Reduced from 0.25

# GPIO Pin Assignments
# Ultrasonic Sensor Pins
ULTRASONIC_PINS = {
    "front": {"trigger": 16, "echo": 26},
}

# Motor Pins
MOTOR_PINS = {
    "left_b": 6,    # LEFT Motor Backward
    "left_e": 5,    # LEFT Motor Enable (or Forward based on H-bridge setup)
    "right_b": 22, # RIGHT Motor Backward
    "right_e": 23, # RIGHT Motor Enable (or Forward based on H-bridge setup)
}

# RGB LED Pins (Common Cathode assumed: HIGH = ON, LOW = OFF)
PIN_RED = 13
PIN_GREEN = 17
PIN_BLUE = 27

# --- GPIO Setup ---
def setup_gpio():
    """Sets up all GPIO pins."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # Setup ultrasonic sensor pins
    for sensor in ULTRASONIC_PINS.values():
        GPIO.setup(sensor["trigger"], GPIO.OUT)
        GPIO.setup(sensor["echo"], GPIO.IN)
        GPIO.output(sensor["trigger"], False) # Ensure trigger is low initially

    # Setup motor pins
    for pin in MOTOR_PINS.values():
        GPIO.setup(pin, GPIO.OUT)

    # Setup RGB LED pins
    GPIO.setup(PIN_RED, GPIO.OUT)
    GPIO.setup(PIN_GREEN, GPIO.OUT)
    GPIO.setup(PIN_BLUE, GPIO.OUT)

    time.sleep(0.01) # Allow modules to settle

# --- RGB LED Control Function ---
def turn_on_color(red_state, green_state, blue_state):
    """
    Sets the state of the RGB LED pins.
    Assumes common cathode (LED connected to GND, and GPIO pulls high to turn on).
    """
    GPIO.output(PIN_RED, red_state)
    GPIO.output(PIN_GREEN, green_state)
    GPIO.output(PIN_BLUE, blue_state)

# --- Motor Control Functions ---
def set_motor_state(motor_b_pin, motor_e_pin, state_b, state_e):
    """Sets the state for a pair of motor pins."""
    GPIO.output(motor_b_pin, state_b)
    GPIO.output(motor_e_pin, state_e)

def stop_motors():
    """Stops both motors."""
    set_motor_state(MOTOR_PINS["left_b"], MOTOR_PINS["left_e"], GPIO.LOW, GPIO.LOW)
    set_motor_state(MOTOR_PINS["right_b"], MOTOR_PINS["right_e"], GPIO.LOW, GPIO.LOW)

def execute_movement(action_name, left_b_state, left_e_state, right_b_state, right_e_state, delay):
    """Executes a motor movement with a given delay and stops."""
    print(f"Moving: {action_name}")
    set_motor_state(MOTOR_PINS["left_b"], MOTOR_PINS["left_e"], left_b_state, left_e_state)
    set_motor_state(MOTOR_PINS["right_b"], MOTOR_PINS["right_e"], right_b_state, right_e_state)
    time.sleep(delay)
    stop_motors()
    time.sleep(0.0001)

def move_forward():
    execute_movement("Forward", GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW, FORWARD_DELAY)

def move_reverse():
    execute_movement("Backward", GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH, REVERSE_DELAY)

def turn_left():
    execute_movement("Turning Left", GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW, TURN_DELAY)

def turn_right():
    execute_movement("Turning Right", GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW, TURN_DELAY)

def sharp_left():
    execute_movement("Sharp Left", GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW, REROUTE_TURN_DELAY)

def sharp_right():
    execute_movement("Sharp Right", GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH, REROUTE_TURN_DELAY)

def back_left():
    execute_movement("Back Left", GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH, REVERSE_DELAY)

def back_right():
    execute_movement("Back Right", GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW, REVERSE_DELAY)

# --- Ultrasonic Sensor Function ---
def get_sonar_distance(trigger_pin, echo_pin, timeout=0.1):
    """Measures distance using an ultrasonic sensor in cm, with a timeout."""
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    pulse_start = time.time()
    pulse_end = time.time()

    # Wait for echo to go high
    start_time = time.time()
    while GPIO.input(echo_pin) == 0:
        pulse_start = time.time()
        if time.time() - start_time > timeout:
            return -1 # Timeout occurred

    # Wait for echo to go low
    end_time = time.time()
    while GPIO.input(echo_pin) == 1:
        pulse_end = time.time()
        if time.time() - end_time > timeout:
            return -1 # Timeout occurred

    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * 34300) / 2
    return round(distance, 2)

def all_clear(distances):
    """Checks if all given distances are greater than SENSOR_PROXIMITY."""
    return all(d > SENSOR_PROXIMITY for d in distances.values())

# --- Image Analysis Functions ---
def segment_colour(frame):
    """Returns a mask with only the red colors in the frame."""
    hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color range
    lower_red = np.array([150, 140, 1])
    upper_red = np.array([190, 255, 255])

    mask = cv2.inRange(hsv_roi, lower_red, upper_red)

    # Morphological operations for noise reduction and gap closing
    kern_dilate = np.ones((8, 8), np.uint8)
    kern_erode = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kern_erode)
    mask = cv2.dilate(mask, kern_dilate)

    cv2.imshow('Red Mask', mask)
    return mask

def find_blob(blob_mask):
    """Returns the bounding box and area of the largest red object."""
    contours, _ = cv2.findContours(blob_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    largest_contour_area = 0
    bounding_rect = (0, 0, 2, 2) # Default small bounding box

    if contours:
        # Find the contour with the largest area
        largest_contour = max(contours, key=cv2.contourArea)
        largest_contour_area = cv2.contourArea(largest_contour)
        bounding_rect = cv2.boundingRect(largest_contour)

    return bounding_rect, largest_contour_area

# --- Camera Setup ---
def setup_camera():
    """Initializes and configures the Picamera2."""
    print("Initializing Picamera2...")
    picam2 = Picamera2()
    picam2_config = picam2.create_preview_configuration(
        main={"format": 'XRGB8888', "size": (640, 480)},
        raw={"size": (1640, 1232)} # Optional: raw stream configuration
    )
    picam2.configure(picam2_config)
    picam2.start()
    print("Picamera2 started. Warming up...")
    time.sleep(2)
    return picam2

# --- Main Robot Control Logic ---
def main():
    setup_gpio()
    picam2 = setup_camera()

    flag = 0    # SEARCHING: 0: left turn for last location of ball, 1: right turn for last location of ball
    flag_reroute = -1 # REROUTE SEARCHING: -1: No reroute, 0: reroute left, 1: reroute right

    print("Starting robot navigation. Press 'q' to quit.")
    try:
        while True:
            frame = picam2.capture_array()
            height, width, _ = frame.shape

            mask_red = segment_colour(frame)
            loct, area = find_blob(mask_red)
            x, y, w, h = loct

            # Get distances from ultrasonic sensors
            distances = {
                "front": get_sonar_distance(ULTRASONIC_PINS["front"]["trigger"], ULTRASONIC_PINS["front"]["echo"]),
            }

            # Replace -1 (error/timeout) with a large value
            for key in distances:
                if distances[key] == -1:
                    distances[key] = 999

            print(f"dC: {distances['front']:.1f} cm")
            print(f"Flag: {flag}, Reroute Flag: {flag_reroute}")
            print(f"Detected Area: {area}")

            found_object = False
            center_x = 0
            # Only process if a significant red object is found within the target area range
            if TARGET_CONTOUR_AREA_MIN < area < TARGET_CONTOUR_AREA_MAX:
                found_object = True
                # Draw bounding box and centroid
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                center_x = x + (w // 2)
                cv2.circle(frame, (int(center_x), int(y + (h // 2))), 3, (0, 110, 255), -1)
                cv2.putText(frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if distances["front"] < BACKUP_DIST:
                while distances["front"] < BACKUP_DIST:
                    move_reverse()
                    # Update distance measurement after each reverse movement
                    distances["front"] = get_sonar_distance(
                        ULTRASONIC_PINS["front"]["trigger"],
                        ULTRASONIC_PINS["front"]["echo"]
                    )
                    if distances["front"] == -1:    # Handle timeout/error
                        distances["front"] = 999
                        break

            if found_object:
                print("Red object found.")
                turn_on_color(False, False, True) # Blue for searching/tracking

                # 1. Check if the ball is too close (parking condition)
                if distances["front"] < DISTANCE_THRESHOLD:
                    stop_motors()
                    turn_on_color(False, True, False) # Green for parked
                    cv2.putText(frame, "PARKED (TOO CLOSE)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # 2. Ball is not too close, now check for other obstacles or track
                    # With only a front sensor, "all_clear" now just checks the front
                    if distances["front"] > SENSOR_PROXIMITY:
                        # No general obstacles, track the ball by turning or moving forward
                        frame_center_x = width // 2
                        if center_x < frame_center_x - CENTER_TOLERANCE:
                            flag = 0 # Last seen on the left
                            turn_left()
                            cv2.putText(frame, "TURNING LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        elif center_x > frame_center_x + CENTER_TOLERANCE:
                            flag = 1 # Last seen on the right
                            turn_right()
                            cv2.putText(frame, "TURNING RIGHT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        else:
                            # Object is relatively centered within the dead zone, move forward
                            move_forward()
                            cv2.putText(frame, "MOVING FORWARD", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            # Update flag based on current center for future search if ball is lost
                            flag = 0 if center_x < width // 2 else 1
                    else:
                        # Obstacle detected (other than the ball being too close for parking)
                        stop_motors()
                        turn_on_color(False, False, False) # LEDs off for obstacle
                        cv2.putText(frame, "OBSTACLE DETECTED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                        if distances["front"] < SENSOR_PROXIMITY and area >= PARKED_AREA_THRESHOLD:
                            # If the red object is directly in front and large enough, consider it parked
                            stop_motors()
                            turn_on_color(False, True, False) # Green for parked
                            cv2.putText(frame, "PARKED (FRONT OBSTACLE)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        else:
                            # With only a front sensor, if an obstacle is detected and it's not the ball for parking,
                            # the robot can only reverse or turn to try and clear it.
                            move_reverse()
                            cv2.putText(frame, "REVERSING FROM OBSTACLE", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            else: # Red object not found or not within area criteria
                turn_on_color(True, False, False) # LEDs off when object is lost or out of range
                print("Red object not found or out of size range. Searching...")
                stop_motors()
                cv2.putText(frame, "SEARCHING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                # Implement a simple search pattern
                if flag == 0:
                    turn_left()
                else:
                    turn_right()
                time.sleep(0.05) # Small pause between search turns

            # Display the frames
            cv2.imshow("Robot View", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # --- Cleanup ---
        print("Releasing resources and cleaning up GPIO...")
        stop_motors()
        turn_on_color(False, False, False) # Ensure all LEDs are off on exit
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("Application closed.")

if __name__ == "__main__":
    main()
