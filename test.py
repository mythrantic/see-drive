import serial
import time
from pynput import keyboard

# Global variables
speed = 0
angle = 45
running = True

def on_press(key):
    global speed, angle, running
    try:
        if key.char == 'w':
            speed = 100
        elif key.char == 's':
            speed = -100
        elif key.char == 'a':
            angle = 0
        elif key.char == 'd':
            angle = 90
    except AttributeError:
        if key == keyboard.Key.esc:
            running = False
            return False  # Stop listener

def on_release(key):
    global speed, angle
    try:
        if key.char == 'w' or key.char == 's':
            speed = 0
        elif key.char == 'a' or key.char == 'd':
            angle = 45
    except AttributeError:
        pass

def main():
    try:
        # Open serial port
        ser = serial.Serial('/dev/ttyACM0', 115200)
        print("Connected to /dev/ttyACM0")
        print("Controls:")
        print("  W: Forward")
        print("  S: Backward")
        print("  A: Left")
        print("  D: Right")
        print("  ESC: Quit")
        
        # Start keyboard listener in a non-blocking way
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        
        # Control loop
        global running
        while running:
            # Send command
            command = f"{speed},{angle}\n"
            ser.write(command.encode())
            print(f"Sent: {command.strip()}")
            time.sleep(0.1)  # Short delay
            
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Make sure to stop the car and close the port
        if 'ser' in locals() and ser.is_open:
            ser.write("0,45\n".encode())  # Stop and center
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
