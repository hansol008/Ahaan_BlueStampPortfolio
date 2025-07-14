import cv2
from picamera2 import Picamera2
import time
import numpy as np # Import numpy for array operations and color definitions

# --- Camera Setup ---
print("Initializing Picamera2...")
picam2 = Picamera2()

# Configure the camera with XRGB8888 format
# XRGB8888 is a 32-bit format where X is an unused byte (alpha channel placeholder)
# OpenCV will typically interpret this as BGR, which is suitable for direct conversion to HSV
picam2_config = picam2.create_preview_configuration(
    main={"format": 'XRGB8888', "size": (640, 480)}, # Use XRGB8888 as per your code
    raw={"size": (1640, 1232)} # Optional: raw stream configuration
)
picam2.configure(picam2_config)

# Start the camera
picam2.start()
print("Picamera2 started. Warming up...")
time.sleep(2) # Give the camera a moment to warm up and set exposure

# --- Main Loop for Red Color Masking ---
print("Starting real-time red color masking. Press 'q' to quit.")
try:
    while True:
        # Capture a frame from the camera as a NumPy array
        img = picam2.capture_array()

        # The 'XRGB8888' format from picamera2 often gets interpreted as BGR by OpenCV.
        # So, we convert from BGR to HSV.
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # --- Define Red Color Range in HSV ---
        # Hue values in OpenCV range from 0-179.
        # Red typically wraps around 0 and 179.
        # So, we define two ranges for red:
        # 1. Lower red: H (0-10), S (50-255), V (50-255)
        # 2. Upper red: H (160-179), S (50-255), V (50-255)

        # Lower bound for red color
        lower_red1 = np.array([170, 100, 50])
        upper_red1 = np.array([180, 255, 255])

        # Upper bound for red color
             # Create masks for the two red ranges
        full_red_mask = cv2.inRange(hsv_frame, lower_red1, upper_red1)

        # Combine the two masks to get the full red mask
        # Bitwise OR operation combines the white pixels from both masks
        

        # Optional: Perform morphological operations on the mask
        # This can help remove small noise and close small gaps, making the mask cleaner
        kernel = np.ones((5, 5), np.uint8)
        full_red_mask = cv2.erode(full_red_mask, kernel, iterations=1)
        full_red_mask = cv2.dilate(full_red_mask, kernel, iterations=1)

        # Apply the combined mask to the original frame
        # This operation keeps only the pixels in 'img' where 'full_red_mask' is white (255)
        #red_filtered_output = cv2.bitwise_and(img, img, mask=full_red_mask)

        # Display the frames
        cv2.imshow("Original Output", img) # Original camera feed
        #cv2.imshow("Red Masked Output", red_filtered_output) # Only red objects visible
        cv2.imshow("Red Mask", full_red_mask) # The binary mask itself

        # Wait for a key press for 1ms. If 'q' is pressed, break the loop.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # --- Cleanup ---
    print("Releasing resources...")
    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows() # Close all OpenCV windows
    print("Application closed.")

