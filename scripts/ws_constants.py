#!/usr/bin/env python3
import rospy
import asyncio
import websockets
from tkinter import *
import threading

WS_CONSTANTS_URL = "ws://192.168.4.1/ws_constants"

# Declare global loop reference
websocket_loop = None
constants_queue = None

class PIDKalmanGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PID & Kalman Constants")

        self.kp = self.add_slider("Kp", 0, 1, 0.1)
        self.ki = self.add_slider("Ki", 0, 1, 0.01)
        self.kd = self.add_slider("Kd", 0, 1, 0.1)
        self.q = self.add_slider("Kalman Q", 0, 5, 0.1)
        self.r = self.add_slider("Kalman R", 0, 10, 0.1)
        self.speed = self.add_slider("Speed", 0, 4000, 10)

        Button(root, text="Update", command=self.enqueue_constants).pack(pady=10)

    def add_slider(self, label, min_val, max_val, resolution):
        Label(self.root, text=label).pack()
        slider = Scale(self.root, from_=min_val, to=max_val, resolution=resolution,
                       orient=HORIZONTAL, length=300)
        slider.set((min_val + max_val) / 2)
        slider.pack()
        return slider

    def enqueue_constants(self):
        constants = f"{self.kp.get()},{self.ki.get()},{self.kd.get()},{self.q.get()},{self.r.get()},{self.speed.get()}"
        global websocket_loop, constants_queue
        if websocket_loop and constants_queue:
            # Use correct loop and queue from the websocket thread
            asyncio.run_coroutine_threadsafe(constants_queue.put(constants), websocket_loop)
            rospy.loginfo(f"Enqueued: {constants}")
        else:
            rospy.logerr("WebSocket loop not initialized")

async def websocket_sender():
    global constants_queue
    constants_queue = asyncio.Queue()
    try:
        async with websockets.connect(WS_CONSTANTS_URL) as websocket:
            rospy.loginfo("Connected to ESP32 WebSocket for constants")
            while not rospy.is_shutdown():
                const_str = await constants_queue.get()
                await websocket.send(const_str)
                rospy.loginfo(f"Sent constants: {const_str}")
    except Exception as e:
        rospy.logerr(f"WebSocket error: {e}")

def start_async_loop():
    global websocket_loop
    websocket_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(websocket_loop)
    websocket_loop.run_until_complete(websocket_sender())

def main():
    rospy.init_node('pid_kalman_constants_node')

    # Start websocket thread
    thread = threading.Thread(target=start_async_loop, daemon=True)
    thread.start()

    # Tkinter GUI in main thread
    root = Tk()
    gui = PIDKalmanGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
