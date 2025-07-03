# Ball Tracking Robot
This project is a robot that tracks balls using computer vision (like a camera seeing colors) with called OpenCV, a computer vision library. It's built with a Raspberry Pi , a camera to track colors, motors to move its wheels, and sensors to stop it from bumping into things. Using color tracking, the robot will track a ball of a predertimed color, and move towards it.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Ahaan P | Fremont High School | Mechanical Engineering | Incoming Junior


<img src="AhaanP2.png" width="350" height="400">


# Third Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/FoBqtvqutC4?si=Kx8xMMg17BJIcstW" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE
## Summary
For my third milestone, I needed to mount all my components I showed in the previous milestone onto the drive base. I also made code that combined the functionality of each component to track the ball.
For the mounting, I first mounted my pi camera and ultrasonic sensor to the front of the drivebase. I fed the wires of the ultrasonicsensor through a hole and taped them to the drivebase usint 3M double sided tape. Since the wires for the ultrasonic sensor are taped, the sensor was suspended and was stable. Then I stuck my pi camera right above the ultrasonic sensor, to make sure that the readings that the ultrasonic sensor were as close to the camera as possible. Then I mounted the raspberry pi behind the camera, to make sure that the camera's cable did not need to twist and bend. Then I mounted the power bank on top of the battery pack that poweres the motors. The battery pack ensures that the raspberry pi has power, and will let the robot move freely without having to be hooked up to a wall outlet.

After mounting everything, I ran the same test code from the previous milestone to make sure all the components were still working. Then I got to writing code that would combine all the components. First, I had to detect the ball. I did this by scanning the color mask for the largest white space, which would represent a red object. Then I had to detect the area of the red object. I started this by looking at the past student's work, and learning that to detect the area, I had to code a 


# Second Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/O2dyLsmRVgA?si=IcNODJyqyDdQhVPl" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Summary
For my second milestone, i had to connect all the different components together and run some test code for it. First, I coded picamera testing code, to make sure the camera could run a live feed through python code and not the terminal, which is what i did for the first milestone. I also assembled the drivebase, which consisted of two motors, a switch and a battery pack. I soldered wires to the motor, and connected the other end to the h bridge. An h bridge is a board that controls the direction of current for two terminal components. Two terminal components are components with an current in terminal and a current out terminal. Some two terminal devices that I am using are motors and the battery pack. I hooked up the h bridge to the raspberry pi by connecting it to the GPIO pins(General purpose in/out pins). This lets the raspberry pi give code to the motors and control them. I wrote code to test the motors, and ran them off the raspberry pi. My motors are going to be used to move the robot towards the ball.

Then, I connected a breadboard to the raspberry pi, so i could connect an ultrasonic sensor. An ultrasonic sensor detects distance by sending out pules of high pitch frequencies and recieving them, then calculating distance by timing how long the pulse takes to reach the sensor. I had to add a voltage divider, to reduce the voltage of the output of the ultrasonic sensor, so that the raspberry pi can recieve it. The ultrasonic sensor sends out data at 5V, and the raspberry pi only takes in 3.3V, so a voltage divider, which is a set up of 2 resistors(1k ohms and 2k ohms) to reduce the voltage to 3.3V. I wrote test code for the ultrasonic sensor, and I was able to get that working. The ultrasonic sensor will be used to find the distance from the robot to the ball, and tell the motors how far to move.

Finally, I needed to code basic color detection for my raspberry pi. I did this by creating a color mask, where i assigned lower and higher bound pixel values, so when the camera displays its feed, the code filters out the pixels that don't fall between the values. Pixels that fall in between the values appear white, while the rest appear black. The code bascially filters the color of each pixel and keeps whichever ones meet the values I put. I chose to filter for the color red, because thats the color of my ball. The camera will be used to see the ball, and detect it from its surroundings.

## Challenges
Since there were so many components that had to work, there were a few challenges I faced. First, the testing code provided by the previous student was outdated, so I had to change certain variables and methods to use newer functions. I also had a few problems with my motors. I had the motor wires in opposite terminals on my h bridge, so each wheel would spin in opposite directions. After that quick fix, I had to troubleshoot by ultrasonic sensor. My raspberry pi was not detecting the sensor, and after rewriting the code, I found that I called my pins twice, and the second time the pins were switched. After that, I had to tinker with the upper/lower bound values for my color mask. If I changed my values too much, then no colors were detected, or everything was detected as red. After playing around with it, I was able to get some values that worked well enough for my use.

## Next steps
My next steps will be to mount everything to the drive base, and to write code that can detect the red ball and have the robot move towards it.


# First Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/4aHEpxQXCT8?si=oDebvNL3SYIfXqwK" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Summary
I've made some good progress on the technical side. I successfully installed the Raspberry Pi OS onto a 32 GB card. That was a bit of a hiccup at first because I accidentally installed the Mac version, so I had to reset the SD card and reinstall it. After that, I got the camera installed on the Raspberry Pi. This camera is important because it's what the robot will use to see its surroundings, find the ball, and then move towards it. I'm using the lib camera command to view the live feed, which is run in the raspberry pi terminal.

## Challanges
I definitely faced a couple of challenges. The incorrect OS installation was one, but I got that sorted. The other main one was with camera command compatibility. My Raspberry Pi is a Module 4B, and it turns out the camera commands are different from the regular Module 4, which uses different commands. I had to do some digging to find the correct commands to get everything working properly.

## Next Steps
Looking ahead, my next big milestone is to build the actual drive base of the robot, including getting the motors set up. After that, I'll need to mount the Raspberry Pi and the camera onto the robot's drive base 




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

