from datetime import datetime
import os

import speech_recognition as sr
import requests


SAFETY_PREFIX = "sonic"   # words that have to be spoken before any action is taken (set to "", None or False if not desired)

TIME_TO_LISTEN = 4      # time to listen (will then stop listening to process); in seconds

DEFAULT_THROTTLE = 0.2
MAX_THROTTLE = 1.0
ZERO_THROTTLE = 0.0
MAX_LEFT_ANGLE = -1.0
MAX_RIGHT_ANGLE = 1.0
STRAIGHT_ANGLE = 0.0

FASTER_INCREMENT = 0.2
SLOWER_INCREMENT = 0.2

DEFAULT_TIMEOUT = 10    # when to stop after receiving command (in seconds)

COMMAND_VALUES = {
    "forward": (STRAIGHT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "backward": (STRAIGHT_ANGLE, -DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "left": (MAX_LEFT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "right": (MAX_RIGHT_ANGLE, DEFAULT_THROTTLE, DEFAULT_TIMEOUT),
    "stop": (STRAIGHT_ANGLE, ZERO_THROTTLE, DEFAULT_TIMEOUT)
}

# TODO update to reflext current direction, angle, throttle and timeout (instruction + examples)
# Test that faster/slower commands are capped at 0, 1 or -1 
LLM_SYSTEM_PROMPT = """
Extract the command from this sentence. We need to extract three things: the direction (only left or right), the steering angle (as a number between 0 and 45) and the throttle value (from -1 to 1). Everything positive (0 to 1) is considered forward, all negative throttle value (below 0 to -1) are considered backward/in reverse. 
In your response, only respond with left or right, followed by a number indicating the angle of the turn, followed by the throttle value. If no throttle is given, output "default". If no turn is given, return "straight 0".
"direction angle throttle throttle_value timeout"
Examples:
- "please please take a left turn of 45 degrees here" should result in "left 45 throttle default default".
- "go straight" results in "straight 0 throttle default default"
- "be very quick, full throttle for 10 seconds" results in "straight 0 throttle 1 10"
- "be kinda slow and take a right turn" results in "right 45 throttle 0.1 default"
- "normal speed" results in "straight 0 throttle default default"
- "I want you to take a stroll, turning right 32 degrees" results in "right 32 throttle default default"
- "u-turn" results in "left 45 throttle default default"
- "reverse" results in "left 38 throttle 0.4 default"
-"I want you to go straight for 5 seconds" results in "straight 0 throttle 1 5"
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
            
            #listens for the user's input
            if verbose:
                print("Start listening")
            audio = recognizer.listen(mic_source, phrase_time_limit=TIME_TO_LISTEN)
            if verbose:
                print("Stopped listening")
            
            # Using google to recognize audio
            start_time = datetime.now()
            text_recognized = recognizer.recognize_google(audio)
            request_time = (datetime.now() - start_time).microseconds / 1000 # in ms

            text_recognized = text_recognized.lower()

            if verbose:
                print(f'Text recognized: "{text_recognized}" (time taken: {request_time} ms)')

            return text_recognized
            
    except sr.RequestError as e:
        print(f"Could not request results: {e}")
        
    except sr.UnknownValueError:
        print("Unknown value error. Did you say anything?")


def make_gemini_request(text: str, current_direction: str, current_angle: float, current_throttle: float, current_timeout: float) -> str:
    """Make a request to the Google Gemini LLM API using the currently recognized command text and currently set steering parameters."""
    llm_prompt = LLM_SYSTEM_PROMPT + text   # TODO incorporate current direction, angle, throttle and timeout

    # Send the request to the API and retrieve response
    url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent"
    headers = {"Content-Type": "application/json"}
    data = {"contents": [{"parts": [{"text": llm_prompt}]}]}
    response = requests.post(f"{url}?key={os.environ['GOOGLE_API_KEY']}", headers=headers, json=data)
    return response.json()['candidates'][0]['content']['parts'][0]['text']

def get_steering_values_from_text(text_recognized: str, current_angle: float, current_throttle: float, current_timeout: float) -> tuple:
    """Determine new steering parameters (angle, throttle, timeout) based on currently recognized command text and currently set steering parameters."""
    # Make sure safety prefix is uttered first
    text_recognized = text_recognized.lower()
    if SAFETY_PREFIX and (not text_recognized.startswith(SAFETY_PREFIX)):
        print(f'Did not start with "{SAFETY_PREFIX}".')
        return None, None, None
    
    # Preprocess text
    text_recognized = text_recognized.replace(SAFETY_PREFIX, "")    # remove safety prefix
    text_recognized = text_recognized.replace(",", "")  # remove all commas
    text_recognized = text_recognized.strip()       # remove leading and trailing whitespace

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
    # TODO Convert current angle to a direction and a degree° value
    current_direction = "TODO" # "left" or "right"
    current_angle = 45.0     # converted to degrees [°]

    time_before_llm = datetime.now()
    response = make_gemini_request(text_recognized, current_direction, current_angle, current_throttle, current_timeout)
    llm_latency = (datetime.now() - time_before_llm).microseconds / 1000
    print(f'Response: {response} (latency: {llm_latency} ms)')

    direction, angle, _, throttle_value, timeout_value = response.split()   # return value looks like: 'left 35 throttle 0.2 8'
    
    # Convert angle
    if direction == "left":
        steering_angle = -1 * float(angle) / 45.0
    elif direction == "right":
        steering_angle = float(angle) / 45.0
    else:
        steering_angle = 0.0

    # Throttle
    if throttle_value == "default":
        throttle = DEFAULT_THROTTLE
    else:
        throttle = float(throttle_value)

    # Timeout
    if timeout_value == "default":
        timeout = DEFAULT_TIMEOUT
    else:
        timeout = float(timeout_value)

    if (type(steering_angle is float) and (type(throttle) is float)):
        return steering_angle, throttle, timeout

    # No match
    print(f'No suitable command found for recognized text "{text_recognized}"')
    return None, None, None
