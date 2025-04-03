#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import sys
import termios
import tty
import select
import time

# Key mappings
key_map = {
    'w': 1,  # Forward
    'a': 2,  # Left
    's': 3,  # Backward
    'd': 4   # Right
}
DEFAULT_STOP = 5  # Stop command

def get_key():
    """Non-blocking keyboard input"""
    tty.setraw(sys.stdin.fileno())  # Set terminal to raw mode
    select_input, _, _ = select.select([sys.stdin], [], [], 1)  # 10ms polling
    key = sys.stdin.read(1) if select_input else None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore terminal settings
    return key

def main():
    global settings
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('/key_cmd', Int32, queue_size=10)
    settings = termios.tcgetattr(sys.stdin)  # Save terminal settings

    rospy.loginfo("Use W A S D keys to move. Press 'q' to quit.")

    current_cmd = DEFAULT_STOP  # Start with STOP
    last_publish_time = time.time()

    try:
        while not rospy.is_shutdown():
            key = get_key()

            if key == 'q':  # Quit command
                rospy.loginfo("Exiting...")
                break

            if key in key_map:
                command = key_map[key]
                if command != current_cmd:
                    current_cmd = command  # Change command
                    rospy.loginfo(f"Key Pressed: {key.upper()} | Publishing: {command}")
                    pub.publish(command)

            elif key is None and current_cmd != DEFAULT_STOP:
                # No key pressed, reset to default
                current_cmd = DEFAULT_STOP
                rospy.loginfo(f"No key pressed | Publishing: {DEFAULT_STOP}")
                pub.publish(DEFAULT_STOP)

            # Ensure continuous publishing (prevents ESP32 from missing messages)
            if time.time() - last_publish_time >= 0.1:  # Publish every 0.5 sec
                rospy.loginfo(f"Continuing to Publish: {current_cmd}")
                pub.publish(current_cmd)
                last_publish_time = time.time()

            rospy.sleep(0.01)  # Adjust for smooth control

    except Exception as e:
        rospy.logerr(f"Error: {e}")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore terminal settings

if __name__ == "__main__":
    main()
