import _deprecated.hardware as hardware

def connect_to_topic():
    # TODO
    pass


def actuate(command: str):
    if command == "forward":
        hardware.actuate_forward()
    elif command == "backward":
        hardware.actuate_backward()
    elif command == "left":
        hardware.actuate_left()
    elif command == "right":
        hardware.actuate_right()
    elif command == "stop":
        hardware.actuate_stop()
    else:
        print("Invalid command.")


def check_lidar() -> bool:
    pass


if __name__ == "__main__":
    topic = connect_to_topic()

    while(True):
        # TODO Poll topic
        # TODO Understand command
        actuate()
        # TODO Check LIDAR
        too_close = check_lidar()

        if too_close:
            actuate("stop")