#!/usr/bin/env python3
import rospy
import websockets
import asyncio
import json
from std_msgs.msg import String

# WebSocket server address (use the IP printed from the ESP32 serial monitor)
WS_SERVER = "ws://192.168.4.1/ws"  # Default Soft AP IP is usually 192.168.4.1

# Initialize ROS node
rospy.init_node('websocket_client')

async def websocket_communicator():
    try:
        async with websockets.connect(WS_SERVER) as websocket:
            while not rospy.is_shutdown():
                # Send a message
                message = "Hello from ROS"
                await websocket.send(message)
                rospy.loginfo(f"Sent: {message}")
                
                # Receive a response
                response = await websocket.recv()
                rospy.loginfo(f"Received: {response}")
                
                # Publish the response to a ROS topic
                pub = rospy.Publisher('websocket_topic', String, queue_size=10)
                pub.publish(response)
                rospy.sleep(1)  # Delay for 1 second before sending the next message
                
    except Exception as e:
        rospy.logerr(f"Error with WebSocket connection: {e}")

def run():
    loop = asyncio.get_event_loop()
    loop.run_until_complete(websocket_communicator())

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
