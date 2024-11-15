from datetime import datetime

import speech_recognition as sr


recognizer = sr.Recognizer()

keep_listening = True

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


def send_command_to_jetson(topic, command: str):
    # TODO publish to ROS2 topic?
    pass


if __name__ == "__main__":
    while(keep_listening):   # loop continuously
        text_recognized = speech_to_text(verbose=True)
        intended_command, valid = match_command(text_recognized)
        
        if valid:
            print(f"Matched command {intended_command}")
            # send_command_to_jetson(topic, intended_command)
        else:
            print("No matching command found")        