#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def text_publisher():
    # Initialize the node
    rospy.init_node('text_publisher', anonymous=True)
    
    # Retrieve the launch argument
    text_to_print = rospy.get_param('~text_to_print', 'default text')
    
    # Create a publisher
    pub = rospy.Publisher('text_topic', String, queue_size=10)
    
    # Set the loop rate
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        # Publish the text
        rospy.loginfo(f"Publishing: {text_to_print}")
        pub.publish(text_to_print)
        
        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        text_publisher()
    except rospy.ROSInterruptException:
        pass
