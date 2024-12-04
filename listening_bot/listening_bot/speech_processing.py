from datetime import datetime
import os
import json

import speech_recognition as sr
import requests


SAFETY_PREFIX = "sonic"  # words that have to be spoken before any action is taken (set to "", None or False if not desired)

TIME_TO_LISTEN = 4  # time to listen (will then stop listening to process); in seconds

MAX_STEERING_ANGLE = 45.0  # maximum angle that car can steer (in degrees [°])

DEFAULT_THROTTLE = 0.2
MAX_THROTTLE = 1.0
ZERO_THROTTLE = 0.0
MAX_LEFT_ANGLE = -1.0
MAX_RIGHT_ANGLE = 1.0
STRAIGHT_ANGLE = 0.0

FASTER_INCREMENT = 0.2
SLOWER_INCREMENT = 0.2

ANGLE_INCREMENT = 5/MAX_STEERING_ANGLE 

DEFAULT_TIMEOUT = 10  # when to stop after receiving command (in seconds)

COMMAND_VALUES = {
    "forward": (STRAIGHT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "backward": (STRAIGHT_ANGLE, -DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "left": (MAX_LEFT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "right": (MAX_RIGHT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "stop": (STRAIGHT_ANGLE, ZERO_THROTTLE, DEFAULT_TIMEOUT),
}

# TODO update to reflext current direction, angle, throttle and timeout (instruction + examples)
# Test that faster/slower commands are capped at 0, 1 or -1
LLM_SYSTEM_PROMPT = """
Instructions:
Extract the steering commands for the car from the given sentence. We need to extract these things:
- the steering direction: This can only be "left", "right" or "straight". Use "straight" if no direction/turn is mentioned.
- the steering angle: This can only be a number between 0.0 and 45.0 and is meant to be measured in degrees. If no direction/turn is mentioned, output 0.0. If a change in current angle is mentioned use "increment" or "decrement" depending on if angle gets larger or smaller
- the throttle mode: This can only be "forward" or "backward". By default, use "forward", unless going backward/ in reverse is indicated.
- the throttle value: This can only be a float number between 0.0 and 1.0. If no throttle is mentioned, output "default".
- the timeout: this encapsulates how long the car should execute the command before it stops. If no timeout is mentioned, output "default".
In your response, always carefully take into account the restrictions mentioned above for each point. If no throttle is given, output "default".


Examples:
- "please please take a left turn of 45 degrees here" should result in {"direction": "left", "angle": 45.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "go straight" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "be very quick, full throttle for 10 seconds" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": 1.0, "timeout": 10.0}
- "be kinda slow and take a right turn" results in {"direction": "right", "angle": 45.0, "throttle_mode": "forward", "throttle_value": 0.1, "timeout": "default"}
- "normal speed" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "I want you to take a stroll, turning right 32 degrees" results in {"direction": "right", "angle": 32.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "u-turn" results in {"direction": "left", "angle": 45.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "go in reverse" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "backward", "throttle_value": "default", "timeout": "default"}
-"I want you to go straight for 5 seconds" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": 5.0}
-"Go more towards the right" results in {"direction": "right", "angle": "increment", "throttle_mode": "unchanged", "throttle_value": "unchanged", "timeout": "default"}
- "Take a smoother left curve" results in {"direction": "left", "angle": "decrement", "throttle_mode": "unchanged", "throttle_value": "unchanged", "timeout": "default"}
-"Make a harder turn" results in {"direction": "unchanged", "angle": "increment", "throttle_mode": "unchanged", "throttle_value": "unchanged", "timeout": "default"}
-"Make a hard left turn" results in {"direction": "left", "angle": 45.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
Output Formatting:
Use proper JSON syntax, following this exact JSON schema:
Return: {"direction": str, "angle": float, "throttle_mode": str, "throttle_value": float|str, "timeout": float|str}
Do not include any prefix or whitespace in your response.
"""


def speech_to_text(verbose: bool = False) -> str:
    """Listen to microphone, use Google API for speech to text, return  recognized text."""
    recognizer = sr.Recognizer()

    try:
        # use the microphone as source for input.
        with sr.Microphone() as mic_source:
            # wait for a second to let the recognizer
            # adjust the energy threshold based on
            # the surrounding noise level
            recognizer.adjust_for_ambient_noise(mic_source, duration=0.2)

            # listens for the user's input
            if verbose:
                print("Start listening")
            audio = recognizer.listen(mic_source, phrase_time_limit=TIME_TO_LISTEN)
            if verbose:
                print("Stopped listening")

            # Using google to recognize audio
            start_time = datetime.now()
            text_recognized = recognizer.recognize_google(audio)
            request_time = (datetime.now() - start_time).microseconds / 1000  # in ms

            text_recognized = text_recognized.lower()

            if verbose:
                print(
                    f'Text recognized: "{text_recognized}" (time taken: {request_time} ms)'
                )

            return text_recognized

    except sr.RequestError as e:
        print(f"Could not request results: {e}")

    except sr.UnknownValueError:
        print("Unknown value error. Did you say anything?")


def make_gemini_request(
    text: str,
    current_direction: str,
    current_angle: float,
    current_throttle_mode: str,
    current_throttle_value: float,
    current_timeout: float
) -> str:
    """Make a request to the Google Gemini LLM API using the currently recognized command text and currently set steering parameters."""
    llm_prompt = LLM_SYSTEM_PROMPT + text
     # f"Current direction: {current_direction}, Current angle: {current_angle}, Current throttle_mode: {current_throttle_mode}, Current throttle_value: {current_throttle_value}, Current timeout: {current_timeout}, " + text  # TODO incorporate current direction, angle, throttle and timeout
# Give Current Values and Recognized Text

    # Send the request to the API and retrieve response
    before = datetime.now()
    url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent"
    headers = {"Content-Type": "application/json"}
    data = {"contents": [{"parts": [{"text": llm_prompt}]}]}
    response = requests.post(
        f"{url}?key={os.environ['GOOGLE_API_KEY']}", headers=headers, json=data
    )

    response_text = response.json()["candidates"][0]["content"]["parts"][0]["text"]
    latency = (datetime.now() - before).microseconds / 1000
    print(f'Response text: {response_text} (latency: {latency} ms)')
    parsed_response = json.loads(response_text)
    return parsed_response


def get_steering_values_from_text(
    text_recognized: str,
    current_angle: float,
    current_throttle: float,
    current_timeout: float,
) -> tuple:
    """Determine new steering parameters (angle, throttle, timeout) based on currently recognized command text and currently set steering parameters."""
    # Make sure safety prefix is uttered first
    text_recognized = text_recognized.lower()
    if SAFETY_PREFIX and (not text_recognized.startswith(SAFETY_PREFIX)):
        print(f'Did not start with "{SAFETY_PREFIX}".')
        return None, None, None

    # Preprocess text
    text_recognized = text_recognized.replace(SAFETY_PREFIX, "")  # remove safety prefix
    text_recognized = text_recognized.replace(",", "")  # remove all commas
    text_recognized = text_recognized.strip()  # remove leading and trailing whitespace

    # Keyword matching
    if text_recognized in ["forward"]:
        return COMMAND_VALUES["forward"]
    elif text_recognized in ["backwards", "backward"]:
        return COMMAND_VALUES["backward"]
    elif text_recognized in ["left"]:
        return COMMAND_VALUES["left"]
    elif text_recognized in ["right"]:
        return COMMAND_VALUES["right"]
    elif text_recognized in ["stop"]:
        return COMMAND_VALUES["stop"]

    # LLM-based More complicated strings
    # Convert current angle to a direction and a degree° value
    current_direction = "left" if current_angle < 0 else "right"
    current_angle = abs(current_angle) * MAX_STEERING_ANGLE  # convert to degrees [°]
    current_throttle_mode = "forward" if current_throttle >= 0 else "backward"
    current_throttle_value = abs(current_throttle)

    response = make_gemini_request(text_recognized, current_direction, current_angle, current_throttle_mode, current_throttle_value, current_timeout)
    direction = response["direction"]
    angle = response["angle"]
    throttle_mode = response["throttle_mode"]
    throttle_value = response["throttle_value"]
    timeout = response["timeout"]

# Increment or Decrement
    if direction == "unchanged":
        direction = current_direction
    if angle == "increment":
        if direction == "left":
            steering_angle = current_angle - ANGLE_INCREMENT
        if direction == "right":
            steering_angle = current_angle + ANGLE_INCREMENT
    if angle == "decrement":
        if direction == "left":
            steering_angle = current_angle + ANGLE_INCREMENT
        if direction == "right":
            steering_angle = current_angle - ANGLE_INCREMENT        

    # Convert angle
    if direction == "left":
        steering_angle = -1 * float(angle) / MAX_STEERING_ANGLE
    elif direction == "right":
        steering_angle = float(angle) / MAX_STEERING_ANGLE
    else:
        steering_angle = 0.0 

    # Clip the steering angle at -1 or 1
    if steering_angle > 0:
       steering_angle = min(1, steering_angle)
    elif steering_angle < 0:
        steering_angle = max(-1, steering_angle)

    # Throttle
    throttle = DEFAULT_THROTTLE if throttle_value == "default" else float(throttle_value)
    throttle = -1.0 * throttle if throttle_mode == "backward" else throttle

    # Timeout
    timeout = DEFAULT_TIMEOUT if timeout == "default" else float(timeout)

    if type(steering_angle is float) and (type(throttle) is float):
        return steering_angle, throttle, timeout

    # No match
    print(f'No suitable command found for recognized text "{text_recognized}"')
    return None, None, None
