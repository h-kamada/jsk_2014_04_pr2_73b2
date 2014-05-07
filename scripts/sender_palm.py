__author__ = 'flier'
import rospy
import leap_interface_palm
from leap_control.msg import leap
from leap_control.msg import leapros

# Obviously, this method publishes the data defined in leapros.msg to /leapmotion/data
def sender():
    li = leap_interface_palm.Runner()
    li.setDaemon(True)
    li.start()
    # pub     = rospy.Publisher('leapmotion/raw',leap)
    pub_ros   = rospy.Publisher('leapmotion/data',leapros)
    rospy.init_node('leap_pub')

    while not rospy.is_shutdown():
        points = li.get_points()
        msg = leapros()
        #print points
        #print "\n"
        msg.points = points

        pub_ros.publish(msg)
        # Save some CPU time, circa 100Hz publishing.
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
