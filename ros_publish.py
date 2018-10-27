import rospy
from geometry_msgs.msg import Quaternion
#import Obstacle

topic = 'obstacles'
node_name = 'master'

def send_obstacle_data(obs):
    global topic
    global node_name
    try:
        pub = rospy.Publisher(topic, Quaternion)
	msg = Quaternion()
	msg.x = obs.x
	msg.y = obs.y
	msg.z = obs.z
	msg.w = obs.diameter
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo(msg)
        pub.publish(msg)
    except rospy.ROSInterruptException as e:
	print(e.getMessage())
        pass
