#!/usr/bin/env python3
import rospy
import asyncio
import websockets
from std_msgs.msg import String

WS_IMU_SERVER = "ws://192.168.4.1/ws_imu"

async def imu_listener(publisher):
    try:
        async with websockets.connect(WS_IMU_SERVER) as websocket:
            rospy.loginfo("Connected to ESP32 IMU WebSocket")

            while not rospy.is_shutdown():
                imu_data = await websocket.recv()
                rospy.loginfo(f"IMU Received: {imu_data}")
                publisher.publish(imu_data)
    except Exception as e:
        rospy.logerr(f"WebSocket IMU error: {e}")

def main():
    rospy.init_node('websocket_imu_listener')
    imu_pub = rospy.Publisher('/imu_data', String, queue_size=10)

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(imu_listener(imu_pub))
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
