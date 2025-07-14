#v1.5
import cv2
from picamera2 import Picamera2
import time
import numpy as np
import RPi.GPIO as GPIO

# --- Configuration Constants ---
# Sensor Proximity (centimeters)
SENSOR_PROXIMITY = 2
REROUTING_PROXIMITY = 17.5
DISTANCE_THRESHOLD = 2  # cm: distance at which the robot should stop

# Robot Behavior Constants
CENTER_TOLERANCE = 80  # Pixels: Defines a 40-pixel dead zone for straight movement
TARGET_CONTOUR_AREA_MIN = 5000  # Minimum pixel area for the ball
TARGET_CONTOUR_AREA_MAX = 110000  # Maximum pixel area for the ball
PARKED_AREA_THRESHOLD = 10000  # Area threshold for the 'parked' state

# Movement Delays (in seconds)
# Adjust these values to control how long each action is performed.
# Shorter delays: more responsive, potentially more erratic.
# Longer delays: smoother, less responsive.
FORWARD_DELAY = 0.15
TURN_DELAY = 0.1
REVERSE_DELAY = 0.2
REROUTE_TURN_DELAY = 0.25
PAUSE_AFTER_MOVEMENT = 0.01 # Small pause after stopping motors for stability

# GPIO Pin Assignments
# Ultrasonic Sensor Pins
ULTRASONIC_PINS = {
    "left": {"trigger": 19, "echo": 26},
    "front": {"trigger": 16, "echo": 20},
    "right": {"trigger": 11, "echo": 12},
}

# Motor Pins
MOTOR_PINS = {
    "left_b": 6,  # LEFT Motor Backward
    "left_e": 5,  # LEFT Motor Enable (or Forward based on H-bridge setup)
    "right_b": 22, # RIGHT Motor Backward
    "right_e": 23, # RIGHT Motor Enable (or Forward based on H-bridge setup)
}

# LED Pins
LED_SEARCH = 18
LED_PARKED = 5

# --- GPIO Setup ---
def setup_gpio():
    """Sets up all GPIO pins."""
    GPIO.cleanup()
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

    # Setup LED pins
    GPIO.setup(LED_SEARCH, GPIO.OUT)
    GPIO.setup(LED_PARKED, GPIO.OUT)

    time.sleep(0.01) # Allow modules to settle

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
    time.sleep(PAUSE_AFTER_MOVEMENT)

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

    flag = 0  # SEARCHING: 0: left turn for last location of ball, 1: right turn for last location of ball
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
                "left": get_sonar_distance(ULTRASONIC_PINS["left"]["trigger"], ULTRASONIC_PINS["left"]["echo"]),
                "front": get_sonar_distance(ULTRASONIC_PINS["front"]["trigger"], ULTRASONIC_PINS["front"]["echo"]),
                "right": get_sonar_distance(ULTRASONIC_PINS["right"]["trigger"], ULTRASONIC_PINS["right"]["echo"]),
            }

            # Replace -1 (error/timeout) with a large value
            for key in distances:
                if distances[key] == -1:
                    distances[key] = 999

            print(f"dL: {distances['left']:.1f} cm, dC: {distances['front']:.1f} cm, dR: {distances['right']:.1f} cm")
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
                # center_y = y + (h // 2) # Not used in logic, can be removed
                cv2.circle(frame, (int(center_x), int(y + (h // 2))), 3, (0, 110, 255), -1)
                cv2.putText(frame, f"Area: {area}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            if found_object:
                print("Red object found.")
                GPIO.output(LED_SEARCH, GPIO.HIGH)
                GPIO.output(LED_PARKED, GPIO.LOW)

                # 1. Check if the ball is too close (parking condition)
                if distances["front"] < DISTANCE_THRESHOLD:
                    stop_motors()
                    GPIO.output(LED_SEARCH, GPIO.LOW)
                    GPIO.output(LED_PARKED, GPIO.HIGH)
                    cv2.putText(frame, "PARKED (TOO CLOSE)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    # 2. Ball is not too close, now check for other obstacles or track
                    if all_clear(distances):
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
                        cv2.putText(frame, "OBSTACLE DETECTED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                        if distances["front"] < SENSOR_PROXIMITY and area >= PARKED_AREA_THRESHOLD:
                            # If the red object is directly in front and large enough, consider it parked
                            stop_motors()
                            GPIO.output(LED_SEARCH, GPIO.LOW)
                            GPIO.output(LED_PARKED, GPIO.HIGH)
                            cv2.putText(frame, "PARKED (FRONT OBSTACLE)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        elif distances["left"] < REROUTING_PROXIMITY:
                            print("Rerouting right (Left obstacle)")
                            back_right()
                            flag_reroute = 1
                            move_forward()
                        elif distances["right"] < REROUTING_PROXIMITY:
                            print("Rerouting left (Right obstacle)")
                            back_left()
                            flag_reroute = 0
                            move_forward()
                        else:
                            # General reverse if obstacle is not specifically left/right for rerouting
                            move_reverse()
                            cv2.putText(frame, "REVERSING", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            else: # Red object not found or not within area criteria
                GPIO.output(LED_SEARCH, GPIO.LOW)
                GPIO.output(LED_PARKED, GPIO.LOW)
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
        GPIO.output(LED_SEARCH, GPIO.LOW)
        GPIO.output(LED_PARKED, GPIO.LOW)
        picam2.stop()
        picam2.close()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        print("Application closed.")

if __name__ == "__main__":
    main()
