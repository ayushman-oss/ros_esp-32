import rospy
import websockets
import asyncio
from std_msgs.msg import Int32

# WebSocket server address (ESP32 AP IP)
WS_SERVER = "ws://192.168.4.1/ws"

# Initialize ROS node
rospy.init_node('websocket_client')

# Function to send WebSocket commands
async def send_command(command):
    try:
        async with websockets.connect(WS_SERVER) as websocket:
            await websocket.send(str(command))  # Send integer as string
            rospy.loginfo(f"Sent: {command}")
            
            # Receive response from ESP32
            response = await websocket.recv()
            rospy.loginfo(f"Received: {response}")
    except Exception as e:
        rospy.logerr(f"WebSocket Error: {e}")

# Callback to process integer commands from the topic
def command_callback(msg):
    asyncio.run(send_command(msg.data))

# Subscribe to /key_cmd
def main():
    rospy.Subscriber('/key_cmd', Int32, command_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
