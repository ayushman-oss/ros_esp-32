#!/usr/bin/env python3
import rospy
import websockets
import asyncio
from geometry_msgs.msg import Twist

# WebSocket server address (ESP32 AP IP)
WS_SERVER = "ws://192.168.4.1/ws"

# Initialize ROS node
rospy.init_node('websocket_teleop_client')

async def send_command(command):
    try:
        async with websockets.connect(WS_SERVER) as websocket:
            while not rospy.is_shutdown():
                await websocket.send(command)
                rospy.loginfo(f"Sent: {command}")
                
                response = await websocket.recv()
                rospy.loginfo(f"Received: {response}")
    except Exception as e:
        rospy.logerr(f"WebSocket Error: {e}")

# Callback to process Twist messages
def cmd_vel_callback(msg):
    if msg.linear.x > 0:
        command = "forward"
        rospy.loginfo(command)

    elif msg.linear.x < 0:
        command = "backward"
        rospy.loginfo(command)

    elif msg.angular.z > 0:
        command = "left"
        rospy.loginfo(command)

    elif msg.angular.z < 0:
        command = "right"
        rospy.loginfo(command)

    else:
        command = "stop"
        rospy.loginfo(command)

    
    asyncio.run(send_command(command))

# Subscribe to /cmd_vel
def main():
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
