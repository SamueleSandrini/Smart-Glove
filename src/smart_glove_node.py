#! /usr/bin/env python3
import rospy
from SmartGlove import SmartGlove

PARAM_NOT_DEFINED_ERROR = "Param ({}) not defined"

def main():
    
    rospy.init_node("Smart_Glove",anonymous=True)
    
    #Required ROS parameters (agents)
    try:
        ip_address=rospy.get_param("~ip_address")
    except KeyError:
        rospy.logerr(PARAM_NOT_DEFINED_ERROR.format("ip_address"))
        return 0
    
    rospy.loginfo(ip_address)
    try:
        smart_glove = SmartGlove(ip_address, 0.7)
        print(smart_glove)
    except ValueError:
        print("Error")
    
if __name__ == "__main__":
    main()