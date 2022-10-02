import websocket
import time
import json
import rospy
from std_msgs.msg import Float64

rospy.init_node("smart_glove")

message_pub = rospy.Publisher("Test", Float64, queue_size=10)
global a
a = 3
def on_open(ws):
    print("opened")


def on_message(ws, message):
    data_received = json.loads(message)
            #print(data_received)
    message_pub.publish(data_received["first"])
    print(data_received)

def on_close(ws):
    a= 4
    print("closed connection")


socket = "ws://192.168.43.108/"                 
ws = websocket.WebSocketApp(socket, 
                            on_open=on_open, 
                            on_message=on_message,
                            on_close=on_close)     
ws.run_forever()


print("r")

while True:
    print(a)
    time.sleep(1)