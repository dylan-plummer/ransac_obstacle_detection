import rospy
from std_msgs.msg import String
#import Obstacle

topic = 'obstacles'
node_name = 'obstacle detection'

def send_obstacle_data(obs):
    global topic
    global node_name
    try:
        pub = rospy.Publisher(topic, String)
	#msg = Obstacle()
	#msg.x = obs.x
	#msg.y = obs.y
	#msg.z = obs.z
	#msg.diameter = obs.diameter
        rospy.init_node(node_name, anonymous=True)
        rospy.loginfo(str(obs))
        pub.publish(str(obs))
    except rospy.ROSInterruptException:
        pass
