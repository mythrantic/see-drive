#!/usr/bin/env python3

import pygame
import time


def main():
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()

    # Check for controllers
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No controllers found")
        return

    print(f"Found {joystick_count} controller(s)")

    # Initialize the first controller
    controller = pygame.joystick.Joystick(0)
    controller.init()

    print(f"Using controller: {controller.get_name()}")
    print(f"Axes: {controller.get_numaxes()}")
    print(f"Buttons: {controller.get_numbuttons()}")
    print(f"Hats: {controller.get_numhats()}")

    print("Press CTRL+C to exit")

    try:
        while True:
            # Get fresh events
            pygame.event.pump()

            # Print axes
            axes = [controller.get_axis(i)
                    for i in range(controller.get_numaxes())]
            print(f"Axes: {axes}")

            # Print buttons
            buttons = [controller.get_button(
                i) for i in range(controller.get_numbuttons())]
            print(f"Buttons: {buttons}")

            # Print hats (D-pad)
            hats = [controller.get_hat(i)
                    for i in range(controller.get_numhats())]
            print(f"Hats: {hats}")

            print("-" * 50)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
