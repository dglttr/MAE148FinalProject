from datetime import datetime

import speech_recognition as sr


recognizer = sr.Recognizer()

def speech_to_text(verbose: bool = False) -> str:
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
            audio = recognizer.listen(mic_source)
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


def match_command(text_recognized: str) -> tuple:
    # forward, backward, left, right, stop
    if text_recognized in ["forward"]:
        return "forward", True
    elif text_recognized in ["backwards", "backward"]:
        return "backward", True
    elif text_recognized in ["left"]:
        return "left", True
    elif text_recognized in ["right"]:
        return "right", True
    else:
        print(f'No matching command found for recognized text "{text_recognized}"')
        return "", False