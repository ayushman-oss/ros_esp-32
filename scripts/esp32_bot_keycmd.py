#!/usr/bin/env python3
import rospy
import asyncio
import websockets
from std_msgs.msg import Int32

WS_SERVER = "ws://192.168.4.1/ws"

# Queue to store commands coming from ROS
command_queue = asyncio.Queue()

async def websocket_client():
    try:
        async with websockets.connect(WS_SERVER) as websocket:
            rospy.loginfo("Connected to ESP32 WebSocket server")

            while not rospy.is_shutdown():
                # Wait for a command from the queue
                command = await command_queue.get()
                await websocket.send(str(command))
                rospy.loginfo(f"Sent: {command}")

                response = await websocket.recv()
                rospy.loginfo(f"Received: {response}")
    except Exception as e:
        rospy.logerr(f"WebSocket error: {e}")

def ros_command_callback(msg):
    # Put the received command into the queue
    asyncio.run_coroutine_threadsafe(command_queue.put(msg.data), asyncio.get_event_loop())

def ros_spin():
    rospy.Subscriber('/key_cmd', Int32, ros_command_callback)
    rospy.spin()

def main():
    rospy.init_node('websocket_client')

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(asyncio.gather(
            websocket_client(),
            loop.run_in_executor(None, ros_spin)
        ))
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
