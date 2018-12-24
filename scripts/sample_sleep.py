import time
import rospy

if __name__ == '__main__':
	rospy.init_node('sample_sleep')
	count = 0
	while not rospy.is_shutdown():
       		time.sleep(5)
        	rospy.loginfo(count)
        	count = count + 1
