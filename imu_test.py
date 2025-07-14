import time
import board # For MPU6050 I2C
import adafruit_mpu6050 # For MPU6050 sensor
import math # For angle calculations

# MPU6050 Constants for filtering/stillness
# These thresholds are crucial for filtering out sensor noise.
# Tune them based on your robot's vibrations and the MPU's sensitivity.
MPU_ANGLE_CHANGE_THRESHOLD = 3.5 # Degrees: Ignore angular velocity below this to reduce drift.

# Global MPU6050 instance and last angle for odometry
mpu = None
current_robot_heading_degrees = 0 # Initial heading for mapping (0=East, 90=North)

def setup_mpu6050():
    """Initializes the MPU6050 sensor."""
    global mpu
    try:
        i2c = board.I2C() # Uses board.SCL and board.SDA pins for I2C communication
        mpu = adafruit_mpu6050.MPU6050(i2c)
        print("MPU6050 sensor initialized successfully.")
    except Exception as e:
        print(f"Error initializing MPU6050: {e}")
        print("MPU6050 will not be used for heading.")
        mpu = None # Set mpu to None if initialization fails

def test_gyroscope_raw():
    """
    Tests the MPU6050 gyroscope and prints continuous raw gyroscope values
    (X, Y, Z angular velocity in rad/s) and the calculated angle.
    Press 'Ctrl+C' to stop the script.
    """
    global current_robot_heading_degrees, mpu
    
    setup_mpu6050()

    last_loop_time = time.time()
    print("\n--- Starting Raw Gyroscope and Angle Test ---")
    print("Move the sensor to see changes. Press Ctrl+C to stop.")

    try:
        while True:
            current_loop_time = time.time()
            # Calculate the time difference (dt) since the last loop iteration.
            # This 'dt' represents the 'change in time' for our heading calculation.
            dt = current_loop_time - last_loop_time
            last_loop_time = current_loop_time

            try:
                # Read raw gyroscope data (x, y, z angular velocity in radians/s)
                gyro_raw = mpu.gyro
                
                # Use Z-axis gyroscope for yaw (rotation around vertical axis)
                # Convert radians/s to degrees/s for easier understanding
                gyro_z_degrees_per_sec = math.degrees(gyro_raw[2]) 

                # Update heading based on gyroscope if significant movement
                if abs(gyro_z_degrees_per_sec) > MPU_ANGLE_CHANGE_THRESHOLD:
                    # This line implements the requested formula:
                    # heading = heading + z-gyro * the change in time (dt)
                    current_robot_heading_degrees += gyro_z_degrees_per_sec * dt 
                    # Normalize angle to 0-360 degrees
                    current_robot_heading_degrees = current_robot_heading_degrees % 360
                    if current_robot_heading_degrees < 0:
                        current_robot_heading_degrees += 360
                
                print(f"| Current Heading: {current_robot_heading_degrees:.2f} degrees")

            except Exception as e:
                print(f"Error reading MPU6050 data: {e}")
                # If MPU reading fails, just continue the loop
                pass 
            
            time.sleep(0.05) # Small delay to control print rate

    except KeyboardInterrupt:
        print("\n--- Raw Gyroscope and Angle Test Stopped ---")
    except Exception as e:
        print(f"An unexpected error occurred during gyroscope test: {e}")

if __name__ == "__main__":
    test_gyroscope_raw()
