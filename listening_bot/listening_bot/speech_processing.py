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

COMMAND_VALUES = {
    "forward": (STRAIGHT_ANGLE, DEFAULT_THROTTLE),
    "backward": (STRAIGHT_ANGLE, -DEFAULT_THROTTLE),
    "left": (MAX_LEFT_ANGLE, DEFAULT_THROTTLE),
    "right": (MAX_RIGHT_ANGLE, DEFAULT_THROTTLE),
    "stop": (STRAIGHT_ANGLE, ZERO_THROTTLE)
}

LLM_SYSTEM_PROMPT = """
Extract the command from this sentence. We need to extract three things: the direction (only left or right), the steering angle (as a number between 0 and 45) and the throttle value (from 0 to 1).
In your response, only respond with left or right, followed by a number indicating the angle of the turn, followed by the throttle value. If no throttle is given, output "default". If no turn is given, return "straight 0".
Examples:
- "please please take a left turn of 45 degrees here" should result in "left 45 throttle default".
- "go straight" results in "straight 0 throttle default"
- "be very quick, full throttle" results in "straight 0 throttle 1"
- "be kinda slow and take a right turn" results in "right 45 throttle 0.1"
- "normal speed" results in "straight 0 throttle default"
- "I want you to take a stroll, turning right 32 degrees" results in "right 32 throttle default"
Respond to this sentence: 
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


def make_gemini_request(text: str) -> str:
    url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent"

    headers = {"Content-Type": "application/json"}

    data = {
        "contents": [
            {"parts": [
                    {"text": LLM_SYSTEM_PROMPT + text}
                ]
            }
        ]
    }

    # Send the POST request
    response = requests.post(f"{url}?key={os.environ['GOOGLE_API_KEY']}", headers=headers, json=data)

    # Print the response
    return response.json()['candidates'][0]['content']['parts'][0]['text']

def get_steering_values_from_text(text_recognized: str) -> tuple:
    """Matches recognized text to the supported commands. Returns command and flag indicating if the command is supported."""
    # Make sure safety prefix is uttered first
    text_recognized = text_recognized.lower()
    if SAFETY_PREFIX and (not text_recognized.startswith(SAFETY_PREFIX)):
        print(f'Did not start with "{SAFETY_PREFIX}".')
        return None, None
    
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
    time_before_llm = datetime.now()
    response = make_gemini_request(text_recognized) # looks like: 'left 35 throttle 0.2'
    llm_latency = (datetime.now() - time_before_llm).microseconds / 1000
    print(f'Response: {response} (latency: {llm_latency} ms)')

    direction, angle, _, throttle_value = response.split()
    
    # Convert angle
    if direction == "left":
        steering_angle = -1 * float(angle) / 45.0
    elif direction == "right":
        steering_angle = float(angle) / 45.0
    else:
        steering_angle = 0

    # Throttle
    if throttle_value == "default":
        throttle = DEFAULT_THROTTLE
    else:
        throttle = float(throttle_value)

    if (type(steering_angle is float) and (type(throttle) is float)):
        return steering_angle, throttle 

    # No match
    print(f'No suitable command found for recognized text "{text_recognized}"')
    return None, None