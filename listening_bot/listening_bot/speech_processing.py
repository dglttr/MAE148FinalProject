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

ANGLE_INCREMENT = 5 / MAX_STEERING_ANGLE   # increase or decrease angle by this many degrees

DEFAULT_TIMEOUT = 10.0  # when to stop after receiving command (in seconds)

COMMAND_VALUES = {
    "forward": (STRAIGHT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "backward": (STRAIGHT_ANGLE, -DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "left": (MAX_LEFT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "right": (MAX_RIGHT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "stop": (STRAIGHT_ANGLE, ZERO_THROTTLE, DEFAULT_TIMEOUT),
}

GEMINI_URL = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent"

LLM_SYSTEM_PROMPT = """
Instructions:
Extract the steering commands for the car from the given sentence. We need to extract these things:
- the steering direction: This can only be "left", "right" or "straight". Use "unchanged" if no direction/turn is mentioned.
- the steering angle: This can only be a number between 0.0 and 45.0 and is meant to be measured in degrees. If no direction/turn is mentioned, output "unchanged". If a change in current angle is mentioned use "increment" or "decrement" depending on if angle gets larger or smaller
- the throttle mode: This can only be "forward" or "backward". By default, use "forward", unless going backward/ in reverse is indicated. If a change in current throttle is mentioned, use "increment" or "decrement" depending on if the speed gets faster or slower
- the throttle value: This can only be a float number between 0.0 and 1.0. If no throttle is mentioned, output "unchanged".
- the timeout: this encapsulates how long the car should execute the command before it stops. If no timeout is mentioned, output "default".
In your response, always carefully take into account the restrictions mentioned above for each point.

Examples:
- "please please take a left turn of 45 degrees here" should result in {"direction": "left", "angle": 45.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "go straight" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "be very quick, full throttle for 10 seconds" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": 1.0, "timeout": 10.0}
- "be kinda slow and take a right turn" results in {"direction": "right", "angle": 45.0, "throttle_mode": "forward", "throttle_value": 0.1, "timeout": "default"}
- "normal speed" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "I want you to take a stroll, turning right 32 degrees" results in {"direction": "right", "angle": 32.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "u-turn" results in {"direction": "left", "angle": 45.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "go in reverse" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "backward", "throttle_value": "default", "timeout": "default"}
- "I want you to go straight for 5 seconds" results in {"direction": "straight", "angle": 0.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": 5.0}
- "Go more towards the right" results in {"direction": "right", "angle": "increment", "throttle_mode": "unchanged", "throttle_value": "unchanged", "timeout": "default"}
- "Take a smoother left curve" results in {"direction": "left", "angle": "decrement", "throttle_mode": "unchanged", "throttle_value": "unchanged", "timeout": "default"}
- "Take a smooth left curve" results in {"direction": "left", "angle": 15.0, "throttle_mode": "forward", "throttle_value": "default, "timeout": "default"}
- "Make a harder turn" results in {"direction": "unchanged", "angle": "increment", "throttle_mode": "unchanged", "throttle_value": "unchanged", "timeout": "default"}
- "Make a hard left turn" results in {"direction": "left", "angle": 45.0, "throttle_mode": "forward", "throttle_value": "default", "timeout": "default"}
- "go faster" results in {"direction": "unchanged", "angle": "unchanged", "throttle_mode": "unchanged", "throttle_value": "increment", "timeout": "default"}
- "go slower" results in {"direction": "unchanged", "angle": "unchanged", "throttle_mode": "unchanged", "throttle_value": "decrement", "timeout": "default"}
- "oh no we're going to crash" results in {"direction": "unchanged", "angle": "unchanged", "throttle_mode": "unchanged", "throttle_value": 0.0, "timeout": "default"}

Output Formatting:
Use proper JSON syntax, following this exact JSON schema:
Return: {"direction": str, "angle": float, "throttle_mode": str, "throttle_value": float|str, "timeout": float|str}
Do not include any prefix or whitespace in your response.

Consider the following text: 
"""


class PrefixMissingError(ValueError):
    """Exception raised if safety prefix is missing."""

    def __init__(self):
        self.message = f'Did not start command with "{SAFETY_PREFIX}". Please try again.'
        super().__init__(self.message)


def voice_recording(recognizer: sr.Recognizer):
    # use the microphone as source for input.
    with sr.Microphone() as mic_source:
        # wait for a second to let the recognizer
        # adjust the energy threshold based on
        # the surrounding noise level
        recognizer.adjust_for_ambient_noise(mic_source, duration=0.2)

        # listens for the user's input
        audio = recognizer.listen(mic_source)#, phrase_time_limit=TIME_TO_LISTEN)
        return audio
    

def audio_to_text(audio, recognizer: sr.Recognizer) -> str:
    start_time = datetime.now()
    text_recognized = recognizer.recognize_google(audio)
    latency = (datetime.now() - start_time).microseconds / 1000  # in ms
    print(f'Text recognized: "{text_recognized}" (latency: {latency} ms)')

    text_recognized = text_recognized.lower()

    return text_recognized


def make_gemini_request(text: str) -> dict:
    """Make a request to the Google Gemini LLM API using the currently recognized command text and currently set steering parameters."""
    llm_prompt = LLM_SYSTEM_PROMPT + text

    # Send the request to the API and retrieve response
    before = datetime.now()
    headers = {"Content-Type": "application/json"}
    data = {"contents": [{"parts": [{"text": llm_prompt}]}]}
    response = requests.post(
        f"{GEMINI_URL}?key={os.environ['LLM_API_KEY']}", headers=headers, json=data
    )

    response_text: str = response.json()["candidates"][0]["content"]["parts"][0]["text"]
    latency = (datetime.now() - before).microseconds / 1000
    print(f'Response text: {response_text.strip()} (latency: {latency} ms)')
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
        raise PrefixMissingError()

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
    if current_angle < 0:
        current_direction = "left"
    elif current_angle > 0:
        current_direction = "right"
    else:
        current_direction = "straight"
    current_throttle_mode = "forward" if current_throttle >= 0 else "backward"
    current_throttle_value = abs(current_throttle)

    response = make_gemini_request(text_recognized)
    direction = response["direction"]
    angle = response["angle"]
    throttle_mode = response["throttle_mode"]
    throttle_value = response["throttle_value"]
    timeout = response["timeout"]

    # Set direction
    if direction == "unchanged":
        direction = current_direction
    
    # Set steering angle
    if angle == "increment":
        steering_angle = current_angle - ANGLE_INCREMENT if direction == "left" else current_angle + ANGLE_INCREMENT
    elif angle == "decrement":
        steering_angle = current_angle + ANGLE_INCREMENT if direction == "left" else current_angle - ANGLE_INCREMENT
    elif angle == "unchanged":
        steering_angle = current_angle
    else:
        # Not increment or decrement, but a number --> convert angle
        if direction == "left":
            steering_angle = -1 * float(angle) / MAX_STEERING_ANGLE
        elif direction == "right":
            steering_angle = float(angle) / MAX_STEERING_ANGLE
        else:
            steering_angle = 0.0 

    # Clip the steering angle at -1 or 1
    if steering_angle > 0:
       steering_angle = min(1.0, steering_angle)
    elif steering_angle < 0:
        steering_angle = max(-1.0, steering_angle)

    # Throttle
    if throttle_mode == "unchanged":
        throttle_mode == current_throttle_mode

    if throttle_value == "unchanged":
        throttle = current_throttle_value
    elif throttle_value == "default":
        throttle = DEFAULT_THROTTLE
    elif throttle_value == "increment":
        throttle = current_throttle_value + FASTER_INCREMENT
    elif throttle_value == "decrement":
        throttle = current_throttle_value - SLOWER_INCREMENT
        if throttle <= 0.0:     # to prevent that the car changes direction when it is supposed to go slower
            throttle = 0.0
    else:
        throttle = float(throttle_value)

    throttle = -1.0 * throttle if throttle_mode == "backward" else throttle
    if throttle < 0.0:
        throttle = max(-1.0, throttle)
    elif throttle > 0.0:
        throttle = min(1.0, throttle)

    # Timeout
    if timeout == "default":
        timeout = DEFAULT_TIMEOUT
    else:
        timeout = float(timeout)

    # Return values
    if type(steering_angle is float) and (type(throttle) is float):
        return steering_angle, throttle, timeout

    # No match
    print(f'No suitable command found for recognized text "{text_recognized}"')
    return None, None, None
