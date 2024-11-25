from datetime import datetime

import speech_recognition as sr


SAFETY_PREFIX = "hey car"   # words that have to be spoken before any action is taken (set to "", None or False if not desired)

DEFAULT_THROTTLE = 0.5
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
            audio = recognizer.listen(mic_source, phrase_time_limit=2)
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
    
    # TODO More complicated strings - degree angles, throttle numbers, ...

    # No match
    print(f'No suitable command found for recognized text "{text_recognized}"')
    return None, None