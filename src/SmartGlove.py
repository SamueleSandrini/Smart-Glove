import ipaddress
import json
from dataclasses import dataclass

import rospy
import websocket
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

#from sensor_msgs.msg import Imu


@dataclass
class SmartGlove:
    ip_address_ : str

    def __post_init__(self):
        """ Constructor of SmartGlove object, called after __init__

        Args:
            ip_address_ (str): Ip Address of Smart Glove ESP-Device

        Raises:
            ValueError: If Ip Address has illegal format
        """
        rospy.loginfo("Smart Glove Object created")
        self.finger_joint_state_pub_ = rospy.Publisher("smart_glove", JointState, queue_size=10)
        
        # Check ip address
        try:
            ipaddress.ip_address(self.ip_address_)
        except ValueError:
            raise ValueError("Illegal ip address.")
    
        socket = "ws://{}/".format(self.ip_address_)
        websocket.enableTrace(True)
        self.ws = websocket.WebSocketApp(socket,
                                     on_open = self.__on_open, 
                                     on_message = self.__on_message,
                                     on_close = self.__on_close)

        self.ws.run_forever()



    def __on_open(self):
        """Private function called by WebsocketApp when started"""
        rospy.loginfo("Connection started")


    def __on_message(self, ws, message):
        """ Private function called by WebsocketApp when a new message comes on websocket 

        Args:
            message (json): message received on websocket
        """
        data_received = json.loads(message)
        
        # Compose finger_joint_state message #
        finger_joint_state_ = JointState()
        finger_joint_state_.header.stamp = rospy.Time.now()
        finger_joint_state_.header.frame_id = "base"   
        finger_joint_state_.name = data_received.keys()
        finger_joint_state_.position = data_received.values()

        # Publish finger_joint_state
        self.finger_joint_state_pub_.publish(finger_joint_state_)

    def __on_close(self):
        """Private method called by WebsocketApp when is disconnected"""
        rospy.loginfo("Cljson_dataosed the connection")

    def run_glove_connection(self):
        """Public method that make start the connection with the glove"""

    def close_glove_connection(self):
        """Public method that make close the connection with the glove"""

