#!/usr/bin/env python3
import rospy
import asyncio
import websockets
from std_msgs.msg import String

WS_RPM_SERVER = "ws://192.168.4.1/ws_rpm"

async def rpm_listener(publisher):
    try:
        async with websockets.connect(WS_RPM_SERVER) as websocket:
            rospy.loginfo("Connected to ESP32 RPM WebSocket")

            while not rospy.is_shutdown():
                rpm_data = await websocket.recv()
                rospy.loginfo(f"RPM Received: {rpm_data}")
                publisher.publish(rpm_data)
    except Exception as e:
        rospy.logerr(f"WebSocket RPM error: {e}")

def main():
    rospy.init_node('websocket_rpm_listener')
    rpm_pub = rospy.Publisher('/motor_rpm', String, queue_size=10)

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(rpm_listener(rpm_pub))
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
