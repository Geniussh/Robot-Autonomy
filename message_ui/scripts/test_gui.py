#!/usr/bin/env python

import rospy
from message_ui.msg import sent_recipe
from message_ui.msg import reply_msg
import time

class Test(object):
    def __init__(self):
        self.pub = rospy.Publisher("reply_msg", reply_msg, queue_size=1000)
        rospy.Subscriber('sent_recipe', sent_recipe, self.start_mixing_cb)
        self.status = reply_msg()
        rospy.on_shutdown(self.shutdownhook)

    def start_mixing_cb(self, msg):
        self.status.message = "Franka starts making your drink..."
        self.pub.publish(self.status)

        # Testing only. Should be replaced by calling franka
        for _ in range(msg.Red):
            time.sleep(1)
        for _ in range(msg.Blue):
            time.sleep(1)
        for _ in range(msg.Green):
            time.sleep(1)

        self.status.message = "Done. Enjoy!"
        self.pub.publish(self.status)

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        global ctrl_c
        self.status.message = "Oops, it's time to close. Franka looks forward to serving you next time!"
        self.pub.publish(self.status)
        ctrl_c = True


if __name__ == "__main__":
    ctrl_c = False

    rospy.init_node('test_gui', anonymous=True)
    Test()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown time! Stop the robot")