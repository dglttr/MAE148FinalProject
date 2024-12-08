from listening_bot.speech_processing import voice_recording, audio_to_text, get_steering_values_from_text, MAX_STEERING_ANGLE, SAFETY_PREFIX, PrefixMissingError, DEFAULT_TIMEOUT, ZERO_THROTTLE, STRAIGHT_ANGLE

from os import path
import tkinter as tk
import time

from PIL import Image, ImageTk  # For displaying icons
import speech_recognition as sr
from ament_index_python.packages import get_package_share_directory


def get_image_path():
    package_name = 'listening_bot'
    image_file = 'resource/mic_icon.png'
    
    package_share_directory = get_package_share_directory(package_name)
    image_path = path.join(package_share_directory, image_file)
    return image_path


GREEN_COLOR = '#228B22'
GREY_COLOR = '#808080'
BLACK_COLOR = '#FFFFFF'
BG_COLOR = "#f0f8ff"

MIC_ICON_FILE = get_image_path()

class VoiceRecorderUI:
    def __init__(self, root, publisher_node):
        """Graphical User Interface for voice recognition and command parsing."""
        self.publisher_node = publisher_node

        self.root = root
        self.root.title("Speech-to-Text Voice Recorder")
        self.root.geometry("450x600")
        self.root.resizable(False, False)
        self.root.configure(bg=BG_COLOR)

        # Icon
        try:
            mic_image = Image.open(MIC_ICON_FILE).resize((30, 30))
            self.mic_icon = ImageTk.PhotoImage(mic_image)
        except Exception:
            print(f"Mic file path not found: {MIC_ICON_FILE}")
            self.mic_icon = None  # Fallback in case icon isn't found

        # Recording button
        self.record_button = tk.Button(
            root,
            text="Start Recording",
            command=self.record_and_update_steering_parameters,
            bg=GREEN_COLOR,
            fg=BLACK_COLOR,
            font=("Segoe UI Emoji", 14),
            padx=10,
            pady=5,
            image=self.mic_icon,
            compound="left"
        )
        self.record_button.pack(pady=50)

        # Status label
        self.status_label = tk.Label(
            root,
            text=f'Press "Start Recording" to begin.\n\n• Make sure to start your sentence with "{SAFETY_PREFIX}"\n• Recording stops automatically when you stop speaking',
            bg=BG_COLOR,
            font=("Segoe UI Emoji", 12),
            justify="left"
        )
        self.status_label.pack(pady=20)

        # Timer label
        self.timer_label = tk.Label(
            root,
            text="",
            bg=BG_COLOR,
            font=("Segoe UI Emoji", 12)
        )
        self.timer_label.pack(pady=10)

        # Emergency stop label
        self.emergency_stop_button = tk.Button(
            root,
            text="Emergency Stop",
            command=self.emergency_stop,
            bg="#FF0000",
            fg=BLACK_COLOR,
            font=("Segoe UI Emoji", 14),
            padx=10,
            pady=15,
            compound="left"
        )
        self.emergency_stop_button.pack(pady=50)

    def record_and_update_steering_parameters(self):
        self.record_button.config(
            text="Start Recording",
            bg = GREY_COLOR,
            state = "disabled"
        )
        self.status_label.config(text=f'Recording... Speak now!\n\n• Make sure to start your sentence with "{SAFETY_PREFIX}"\n• Recording stops automatically when you stop speaking')
        self.status_label.update()  # calling update() to ensure text changes, even when another change happens quickly after

        # Record audio
        recognizer = sr.Recognizer()
        audio = voice_recording(recognizer)
        self.status_label.config(text="Stopped recording. Attempting to recognize text...")
        self.status_label.update()

        # Speech-to-Text and error handling
        try:
            text_recognized = audio_to_text(audio, recognizer)

            self.status_label.config(text=f'Text recognized: "{text_recognized}"\n\nCalling LLM to understand intent...')
            self.status_label.update()

            # Get steering values using LLM
            steering_angle, throttle, timeout = get_steering_values_from_text(text_recognized,
                                                                              current_angle=self.publisher_node.twist_cmd.angular.z,
                                                                              current_throttle=self.publisher_node.twist_cmd.linear.x,
                                                                              current_timeout=self.publisher_node.twist_cmd.linear.y)

            # Show result in textbox
            if steering_angle < 0:
                direction = "Left"
            elif steering_angle > 0:
                direction = "Right"
            else:
                direction = "Straight"
            converted_angle = abs(steering_angle) * MAX_STEERING_ANGLE
            self.status_label.config(text=f'Text recognized: "{text_recognized}"\n\nCommand identified:\n    ↔️ Direction: "{direction}"\n    📐 Angle: {converted_angle:.1f}°\n    ⚡ Throttle: {throttle*100:.1f}%\n    ⏳ Timeout: {timeout:.1f}s\n\nPublishing to Jetson...')

            # Publish to Jetson
            self.publisher_node.publish_new_steering_parameters(steering_angle, throttle, timeout)

            # Start countdown
            self.timeout = timeout
            self.start_time = time.time()
            self.update_timer()
        except sr.RequestError as e:
            self.status_label.config(text='Could not request results. Try recording again.')
            self.status_label.update()
            self.publisher_node.get_logger().info(f"Request Error: {e}")
        except sr.UnknownValueError as e:
            self.status_label.config(text='Unknown value error. Did you say anything? Try recording again.')
            self.status_label.update()
            self.publisher_node.get_logger().info(f"Unknown Value Error: {e}")
        except PrefixMissingError as e:
            self.status_label.config(text=e.message)
            self.status_label.update()
            self.publisher_node.get_logger().info(e.message)
        except KeyError as e:   # most likely error with LLM response
            self.status_label.config(text='There was likely an error with the LLM API. Please check that you set the right API key.')
            self.status_label.update()
            self.publisher_node.get_logger().info(f'There was likely an error with the LLM API. Please check that you set the right API key as the environment variable "LLM_API_KEY". Full error: {e}')

        # tkinter re-enable button
        self.record_button.config(
            text="Start Recording",
            bg = GREEN_COLOR,
            state = "normal"
        )

    def emergency_stop(self):
        steering_angle, throttle, timeout = STRAIGHT_ANGLE, ZERO_THROTTLE, DEFAULT_TIMEOUT
        self.publisher_node.publish_new_steering_parameters(steering_angle, throttle, timeout)

    def update_timer(self):
        elapsed_time = time.time() - self.start_time
        time_remaining = self.timeout - elapsed_time

        if time_remaining >= 0:
            self.timer_label.config(text=f"Time remaining: {round(time_remaining)} seconds")
            self.root.after(1000, self.update_timer)  # Update every second
        else:
            self.timer_label.config(text=f"Timeout of {self.timeout} seconds reached. Command stopped.")

if __name__ == "__main__":
    root = tk.Tk()
    app = VoiceRecorderUI(root)
    root.mainloop()
