# External module imports
import Jetson.GPIO as GPIO
import time

# ALL GPIO PINS NEED TO BE CHANGED
# Define GPIO For Driver motors 
GPIO.setmode(GPIO.BOARD)
GPIO.setup(37, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)

# Define pins for all sensors (you need to fill these)
front_left_trigger_pin = 
front_left_echo_pin = 
front_right_trigger_pin = 
front_right_echo_pin = 
back_left_trigger_pin = 
back_left_echo_pin = 
back_right_trigger_pin = 
back_right_echo_pin = 
side_left_front_trigger_pin = 
side_left_front_echo_pin = 
side_left_back_trigger_pin = 
side_left_back_echo_pin = 
side_right_front_trigger_pin = 
side_right_front_echo_pin = 
side_right_back_trigger_pin = 
side_right_back_echo_pin = 

# Set up all trigger and echo pins
pins = [front_left_trigger_pin, front_right_trigger_pin, back_left_trigger_pin, back_right_trigger_pin,
        side_left_front_trigger_pin, side_left_back_trigger_pin, side_right_front_trigger_pin, side_right_back_trigger_pin]

echo_pins = [front_left_echo_pin, front_right_echo_pin, back_left_echo_pin, back_right_echo_pin,
             side_left_front_echo_pin, side_left_back_echo_pin, side_right_front_echo_pin, side_right_back_echo_pin]

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

for pin in echo_pins:
    GPIO.setup(pin, GPIO.IN)


# Get distance from a single sensor
def get_distance(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, False)
    time.sleep(0.2)
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start = time.time()
    stop = time.time()

    while GPIO.input(echo_pin) == 0:
        start = time.time()

    while GPIO.input(echo_pin) == 1:
        stop = time.time()

    elapsed = stop - start
    distance = (elapsed * 34300) / 2  # Speed of sound in cm/s, divide by 2 for round trip
    return distance

# Optimization: Get distances from all sensors at once
def get_all_distances():
    distances = {
        "front_left": get_distance(front_left_trigger_pin, front_left_echo_pin),
        "front_right": get_distance(front_right_trigger_pin, front_right_echo_pin),
        "right_front": get_distance(side_right_front_trigger_pin, side_right_front_echo_pin),
        "right_back": get_distance(side_right_back_trigger_pin, side_right_back_echo_pin),
        "left_front": get_distance(side_left_front_trigger_pin, side_left_front_echo_pin),
        "left_back": get_distance(side_left_back_trigger_pin, side_left_back_echo_pin),
        "back_left": get_distance(back_left_trigger_pin, back_left_echo_pin),
        "back_right": get_distance(back_right_trigger_pin, back_right_echo_pin)
    }
    return distances
