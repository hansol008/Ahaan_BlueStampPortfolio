import RPi.GPIO as GPIO
import time

# Pin Definitions
# These are Broadcom (BCM) GPIO numbers, not physical pin numbers.
# Red LED connected to GPIO 4
# Green LED connected to GPIO 17
# Blue LED connected to GPIO 27
PIN_RED =13
PIN_GREEN =17
PIN_BLUE = 27

def setup_gpio():
    """
    Sets up the GPIO mode and initializes the LED pins as outputs.
    """
    GPIO.setmode(GPIO.BCM)  # Use Broadcom GPIO numbers
    GPIO.setwarnings(False) # Disable warnings for already set pins

    # Set up each LED pin as an output
    GPIO.setup(PIN_RED, GPIO.OUT)
    GPIO.setup(PIN_GREEN, GPIO.OUT)
    GPIO.setup(PIN_BLUE, GPIO.OUT)
    print("GPIO setup complete.")

def turn_on_color(red_state, green_state, blue_state):
    """
    Sets the state of the RGB LED pins.
    For common cathode LEDs:
    - High (True) means the LED is on.
    - Low (False) means the LED is off.
    This code now assumes common cathode (LED connected to GND, and GPIO pulls high to turn on).
    If your LED is common anode (LED connected to VCC, and GPIO pulls low to turn on),
    you'll need to re-add the 'not' operator to invert the states.
    """
    GPIO.output(PIN_RED, red_state)    # Removed 'not' for common cathode
    GPIO.output(PIN_GREEN, green_state) # Removed 'not' for common cathode
    GPIO.output(PIN_BLUE, blue_state)  # Removed 'not' for common cathode

def cleanup_gpio():
    """
    Cleans up all GPIO settings, turning off all LEDs.
    """
    print("Cleaning up GPIO...")
    GPIO.cleanup()
    print("GPIO cleanup complete. LEDs are off.")

def main():
    """
    Main function to run the RGB LED test sequence.
    """
    setup_gpio()

    try:
        print("Starting RGB LED test sequence...")
        # Cycle through colors
        colors = {
            "Red":    (True, False, False),  # R on, G off, B off
            "Green":  (False, True, False),  # R off, G on, B off
            "Blue":   (False, False, True),  # R off, G off, B on
            "White":  (True, True, True),    # All on (mixes to white)
        }
        while True:
            turn_on_color(True,False,False)


    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        cleanup_gpio()

if __name__ == "__main__":
    main()
