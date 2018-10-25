import rospy
from std_msgs.msg import String
from obstacle import Obstacle

topic = 'obstacles'
node_name = 'obstacle detection'

def send_obstacle_data(obs):
    global topic
    try:
        pub = rospy.Publisher(topic, Obstacle, queue_size=10)
        rospy.init_node(node_name. anonymous=True)
        rospy.loginfo(obs)
        pub.publish(obs)
    except rospy.ROSInterruptException:
        pass
