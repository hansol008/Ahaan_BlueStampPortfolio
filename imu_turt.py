import time
import board  # For MPU6050 I2C
import adafruit_mpu6050  # For MPU6050 sensor
import math  # For angle calculations
import turtle  # For graphical display

# MPU6050 Constants for filtering/stillness
# These thresholds are crucial for filtering out sensor noise.
# Tune them based on your robot's vibrations and the MPU's sensitivity.
MPU_ANGLE_CHANGE_THRESHOLD = 3.5  # Degrees: Ignore angular velocity below this to reduce drift.

# Global MPU6050 instance and last angle for odometry
mpu = None
current_robot_heading_degrees = 90  # Initial heading for mapping (0=East, 90=North)

# --- Turtle Graphics Setup ---
screen = turtle.Screen()
screen.setup(width=600, height=600)
screen.bgcolor("lightblue")
screen.title("MPU6050 Real-time Heading Display")
screen.tracer(0)  # Turn off screen updates for smoother animation

arrow_turtle = turtle.Turtle()
arrow_turtle.shape("arrow")
arrow_turtle.color("red")
arrow_turtle.shapesize(stretch_wid=2, stretch_len=2)
arrow_turtle.penup()  # Don't draw when moving
arrow_turtle.speed(0)  # Fastest animation speed
arrow_turtle.setheading(current_robot_heading_degrees)  # Set initial heading

# --- MPU6050 Functions ---
def setup_mpu6050():
    """Initializes the MPU6050 sensor."""
    global mpu
    try:
        i2c = board.I2C()  # Uses board.SCL and board.SDA pins for I2C communication
        mpu = adafruit_mpu6050.MPU6050(i2c)
        print("MPU6050 sensor initialized successfully.")
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")
        print("MPU6050 will not be used for heading. Turtle will not rotate based on MPU.")
        mpu = None  # Set mpu to None if initialization fails

def run_turtle_mpu_display():
    """
    Initializes the MPU6050 sensor and continuously updates the turtle's
    heading based on the MPU's Z-axis gyroscope data.
    Press 'Ctrl+C' to stop the script.
    """
    global current_robot_heading_degrees, mpu
    
    setup_mpu6050()

    last_loop_time = time.time()
    print("\n--- Starting MPU6050 Turtle Display ---")
    print("Rotate the MPU sensor to see the turtle arrow change direction.")
    print("Press Ctrl+C in the console to stop.")

    try:
        while True:
            current_loop_time = time.time()
            # Calculate the time difference (dt) since the last loop iteration.
            dt = current_loop_time - last_loop_time
            last_loop_time = current_loop_time 

            if mpu is not None:  # Only try to read MPU if it was initialized successfully
                try:
                    # Read raw gyroscope data (x, y, z angular velocity in radians/s)
                    gyro_raw = mpu.gyro
                    
                    # Use Z-axis gyroscope for yaw (rotation around vertical axis)
                    # Convert radians/s to degrees/s for easier understanding
                    gyro_z_degrees_per_sec = math.degrees(gyro_raw[2])

                    # Update heading based on gyroscope if significant movement
                    if abs(gyro_z_degrees_per_sec) > MPU_ANGLE_CHANGE_THRESHOLD:
                        # heading = heading + z-gyro * the change in time (dt)
                        current_robot_heading_degrees += gyro_z_degrees_per_sec * dt
                        # Normalize angle to 0-360 degrees
                        current_robot_heading_degrees = current_robot_heading_degrees % 360
                        if current_robot_heading_degrees < 0:
                            current_robot_heading_degrees += 360
                        
                        # Update the turtle's heading
                        arrow_turtle.setheading(current_robot_heading_degrees)
                        screen.update()  # Update the turtle screen

                except Exception as e:
                    print(f"Error reading MPU6050 data: {e}")
                    # If MPU reading fails, just continue the loop
                    pass
            else:
                # If MPU is not available, you might want to add a visual cue
                # or just let the turtle stay still. For now, it stays still.
                pass
            
            #time.sleep(0.02)  # Small delay to control update rate and CPU usage

    except KeyboardInterrupt:
        print("\n--- MPU6050 Turtle Display Stopped ---")
    except Exception as e:
        print(f"An unexpected error occurred during turtle display: {e}")
    finally:
        screen.bye()  # Close the turtle graphics window gracefully

if __name__ == "__main__":
    run_turtle_mpu_display()
    turtle.done()  # Keep the turtle window open until manually closed