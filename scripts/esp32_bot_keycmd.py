#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import websocket  # Make sure `websocket-client` is installed
import threading
import time

WS_SERVER = "ws://192.168.4.1/ws"
ws = None  # Global WebSocket object

def command_callback(msg):
    global ws
    try:
        if ws and ws.connected:
            ws.send(str(msg.data))
            rospy.loginfo(f"Sent: {msg.data}")
        else:
            rospy.logwarn("WebSocket not connected. Message not sent.")
    except Exception as e:
        rospy.logerr(f"WebSocket send error: {e}")

def websocket_thread():
    global ws
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Connecting to ESP32 WebSocket...")
            ws = websocket.create_connection(WS_SERVER)
            rospy.loginfo("Connected to ESP32 WebSocket")

            while ws.connected and not rospy.is_shutdown():
                time.sleep(1)  # Keep connection alive
        except Exception as e:
            rospy.logerr(f"WebSocket connection error: {e}")
            time.sleep(2)  # Retry after short delay

def main():
    rospy.init_node('esp32_websocket_persistent_client')

    # Start WebSocket connection thread
    threading.Thread(target=websocket_thread, daemon=True).start()

    # Subscribe to ROS topic
    rospy.Subscriber('/key_cmd', Int32, command_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
