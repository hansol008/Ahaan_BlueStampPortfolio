# Ball Tracking Robot
This project is a robot that tracks balls using computer vision (like a camera seeing colors) with called OpenCV, a computer vision library. It's built with a Raspberry Pi , a camera to track colors, motors to move its wheels, and sensors to stop it from bumping into things. Using color tracking, the robot will track a ball of a predertimed color, and move towards it.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Ahaan P | Fremont High School | Mechanical Engineering | Incoming Junior


<img src="AhaanP2.png" width="350" height="400">


# Third Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/FoBqtvqutC4?si=Kx8xMMg17BJIcstW" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Summary
For my third milestone, I needed to mount all the components I showed in the previous milestone onto the drive base. I also made code that combined the functionality of each component to track the ball.
For the mounting, I first mounted my Pi camera and ultrasonic sensor to the front of the drive. I fed the wires of the ultrasonic sensor through a hole and taped them to the drivebase using 3M double-sided tape. Since the wires for the ultrasonic sensor are taped, the sensor was suspended and was stable. Then I stuck my pi camera right above the ultrasonic sensor, to make sure that the readings that the ultrasonic sensor were as close to the camera as possible. Then I mounted the Raspberry Pi behind the camera, to make sure that the camera's cable did not need to twist and bend. Then I mounted the power bank on top of the battery pack that powered the motors. The battery pack ensures that the Raspberry Pi has power, and will let the robot move freely without having to be hooked up to a wall outlet.

After mounting everything, I ran the same test code from the previous milestone to make sure all the components were still working. Then I got to writing code that would combine all the components. First, I had to detect the ball. I did this by scanning the color mask for the largest white space, which would represent a red object. Then I had to detect the area of the red object. I started this by looking at the past student's work and learning that to detect the area, I had to code a method that measures the pixels of the largest white space and calculate its area. The robot view panel also shows the square outline of the object. Other than that, I had to make the robot move towards the ball when it was detected. By assigning minimum and maximum area values for detection, the robot would only spot the ball if it was at least 5cm away from the distance sensor, and a maximum distance of about 20cm.  After that, I had to measure the distance between the ball and the robot. I did this by periodically sending pulses from the ultrasonic sensor. To detect the distance of the ball, the camera also had to see the ball, to make sure only the ball's distance would be detected. Then, I made sure that the robot moved incrementally until the distance between the ball and the robot was 5 cm. After setting everything up, all that was left was the calibration, to make the robot as accurate as I could make it.

## Challenges
One challenge I faced was that if the ball was not in the frame, the robot would just start moving randomly. To combat this, I created a "searching mode" where if there was no ball detected, then the robot would move in a circle slowly until the ball was spotted. To better see what the robot was doing, I made it so that the robot displayed its current action on the robot view screen. Another challenge I faced was that the Pi camera was not filtering color correctly. I fixed this by changing the RGB values of the filter until it seemed good. Another challenge I faced was mounting everything without wires getting caught by the wheels. After running out of space on the top, I had to move my h bridge to the underside of the robot and swap the wires for shorter ones so that they would not get caught by the motor.

## Next Steps
For my next steps, I want to have a map of the robots movements to be drawn as it moves. I plan on doing this using turtle, a drawing library in python, and using an IMU(inertial measurement unit) which will be able to give the real direction of the robot.


# Second Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/O2dyLsmRVgA?si=IcNODJyqyDdQhVPl" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Summary
For my second milestone, i had to connect all the different components together and run some test code for it. First, I coded picamera testing code, to make sure the camera could run a live feed through python code and not the terminal, which is what i did for the first milestone. I also assembled the drivebase, which consisted of two motors, a switch and a battery pack. I soldered wires to the motor, and connected the other end to the h bridge. An h bridge is a board that controls the direction of current for two terminal components. Two terminal components are components with an current in terminal and a current out terminal. Some two terminal devices that I am using are motors and the battery pack. I hooked up the h bridge to the raspberry pi by connecting it to the GPIO pins(General purpose in/out pins). This lets the raspberry pi give code to the motors and control them. I wrote code to test the motors, and ran them off the raspberry pi. My motors are going to be used to move the robot towards the ball.

Then, I connected a breadboard to the raspberry pi, so i could connect an ultrasonic sensor. An ultrasonic sensor detects distance by sending out pules of high pitch frequencies and recieving them, then calculating distance by timing how long the pulse takes to reach the sensor. I had to add a voltage divider, to reduce the voltage of the output of the ultrasonic sensor, so that the raspberry pi can recieve it. The ultrasonic sensor sends out data at 5V, and the raspberry pi only takes in 3.3V, so a voltage divider, which is a set up of 2 resistors(1k ohms and 2k ohms) to reduce the voltage to 3.3V. I wrote test code for the ultrasonic sensor, and I was able to get that working. The ultrasonic sensor will be used to find the distance from the robot to the ball, and tell the motors how far to move.

Finally, I needed to code basic color detection for my raspberry pi. I did this by creating a color mask, where i assigned lower and higher bound pixel values, so when the camera displays its feed, the code filters out the pixels that don't fall between the values. Pixels that fall in between the values appear white, while the rest appear black. The code bascially filters the color of each pixel and keeps whichever ones meet the values I put. I chose to filter for the color red, because thats the color of my ball. The camera will be used to see the ball, and detect it from its surroundings.

## Challenges
Since there were so many components that had to work, there were a few challenges I faced. First, the testing code provided by the previous student was outdated, so I had to change certain variables and methods to use newer functions. I also had a few problems with my motors. I had the motor wires in opposite terminals on my h bridge, so each wheel would spin in opposite directions. After that quick fix, I had to troubleshoot by ultrasonic sensor. My Raspberry Pi was not detecting the sensor, and after rewriting the code, I found that I called my pins twice, and the second time the pins were switched. After that, I had to tinker with the upper/lower bound values for my color mask. If I changed my values too much, then no colors were detected, or everything was detected as red. After playing around with it, I was able to get some values that worked well enough for my use.

## Next steps
My next steps will be to mount everything to the drive base, and to write code that can detect the red ball and have the robot move towards it.

## Test Codes

Ultrasonic Sensor test code : 
```python
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
```
Motor Testing Code : 
```python
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
```
Color Mask Code:
```python
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

```
Color Mask Result : 


<img src="colormask.png" width="646" height="288">


# First Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/4aHEpxQXCT8?si=oDebvNL3SYIfXqwK" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Summary
I've made some good progress on the technical side. I successfully installed the Raspberry Pi OS onto a 32 GB card. That was a bit of a hiccup at first because I accidentally installed the Mac version, so I had to reset the SD card and reinstall it. After that, I got the camera installed on the Raspberry Pi. This camera is important because it's what the robot will use to see its surroundings, find the ball, and then move toward it. I'm using the lib camera command to view the live feed, which is run in the Raspberry Pi terminal.

## Challanges
I definitely faced a couple of challenges. The incorrect OS installation was one, but I got that sorted. The other main one was with camera command compatibility. My Raspberry Pi is a Module 4B, and it turns out the camera commands are different from the regular Module 4, which uses different commands. I had to do some digging to find the correct commands to get everything working properly.

## Next Steps
Looking ahead, my next big milestone is to build the actual drive base of the robot, including getting the motors set up. After that, I'll need to mount the Raspberry Pi and the camera onto the robot's drive base 

## Code
```python
libcamera-hello -t 0
```



<!--
# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

# Code
Here's where you'll put your code. The syntax below places it into a block of code. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize it to your project needs. 

```c++
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
```

# Bill of Materials
Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. 

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |
| Item Name | What the item is used for | $Price | <a href="https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/"> Link </a> |

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.
-->
# Starter Project - Weevil Eye

<iframe width="560" height="315" src="https://www.youtube.com/embed/9kMi62yegL4?si=SjV1ctFakXZa0eug" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Summary
For my starter project, I decided to build the Weevil Eye. The purpose of the Weevil Eye is to light up when it's dark, using a light senestive photosensor, which only let current pass through when there was no light. I constructed the Weevil Eye by soldering the different comopnents to the PCB. The main understadning points for this project was for me to practice soldering, and learn how to put a circut together.

<img src="weevil1.png" width="200" height="200">
 <li class="masthead__menu-item">
          <a href="https://www.sparkfun.com/sparkfun-weevileye-beginner-soldering-kit.html">Weevil Eye </a>
        </li>


<img src="weevilschem.png" width="200" height="200">
 <li class="masthead__menu-item">
          <a href="https://cdn.sparkfun.com/datasheets/Kits/Weevil_Eye-v16.pdf">Weevil Eye Schematic</a>
        </li>




## Components Used
- WeevilEye PCB : the board that connects all the components together
- LEDs: A component that lights up when electricity passes through
- Resistors: a component that resists the flow of electricity
- Miniature Photocell: A compoment used to detect wether there is light
- 20mm Coin Cell Battery Holder : connection between the battery and PCB
- 20mm Coin Cell Battery : the powersource for the Weevil Eye
- 2N3904 Transistor : a device that regulates electricity flow and can act as a switch

The Weevil Eye's function relys on the photocell sensor, since that is the final bridge between the circut and the LED's. If there is light, then the photocell sensor will not bridge the circut, and the LED's will not turn on. The photocell sensor has a threshold for light, and will gradually open the circut, which means as it gets darker, the LED's light will become brighter and brighter.

## Challenges Faced
Though this project seems simple, there were some struggles. Mainly, the board was really small and soldering points had to be more precise. Another struggle I had was that I put the photocell sensor the opposite way and soldered it. I had to pull the sensor out, take out the solder, and insert it back into the correct way. Overall the project was very fun, and a great introduction to soldering.

