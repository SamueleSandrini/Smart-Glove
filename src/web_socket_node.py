from ast import If
import websocket
import time
import json
import rospy
from std_msgs.msg import Float64

rospy.init_node("smart_glove")

message_pub = rospy.Publisher("Test", Float64, queue_size=10)

ws = websocket.WebSocket()
websocket.enableTrace(False)
websocket.setdefaulttimeout(5)
while True: 
    retry = True
    while retry:
        try:
            ws.connect("ws://192.168.43.108/")
            retry = False
              
        except:
            time.sleep(0.1)
            print("Connecting")
    print("Connected Succesfully")

    k = 0
    
    server_connected = True
    while server_connected:
        print(k)
        ws.send("Starting connection" + str(k))
        try:
            #print(ws.recv())
            result = ws.recv()
            data_received = json.loads(result)
            #print(data_received)
            message_pub.publish(data_received["first"])

            k+=1
            #print(result)
            #print(json.loads(result))
            print(k)
        except:
            print("Server disconnected")
            server_connected = False
        #print("I am connected")
        time.sleep(0.1)
        
ws.close()