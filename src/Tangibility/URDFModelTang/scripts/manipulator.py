#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String,Float64
from sensor_msgs.msg import JointState
from t_motor_api.msg import *


class JointStateMessage():
    def __init__(self, name, position, velocity, effort):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort

class puppetmanipulatorclass():
	def __init__(self):
		self.joint_states_pub = rospy.Publisher('/joint_states', JointState)
		rospy.init_node('puppetmanipulator', anonymous=True)
		self.joint=[]
		jointnamen = ["J3", "J4","J5","J6","J7"]
		for i in range(4):
			self.joint.append(JointStateMessage(jointnamen[i],0,0,0))
		rospy.Subscriber("/tmotor/1", tmotor_data, self.callback0)
		rospy.Subscriber("/tmotor/2", tmotor_data, self.callback1)
		rospy.Subscriber("/tmotor/3", tmotor_data, self.callback2)
		rospy.Subscriber("/tmotor/4", tmotor_data, self.callback3)
		#rospy.Subscriber("/tmotor/4", tmotor_data, self.callback4)
		self.talker()
		rospy.spin()

	def callback0(self, data):
		# self.joint[0].position=(data.data-5253)/16384*2*3.1415926535
		# self.joint[0].position=-(data.data-13445)/16384*2*3.1415926535
		self.joint[0].position = data.position
		self.talker()
	def callback1(self, data):
		# self.joint[1].position=(data.data-7066)/16384*2*3.1415926535
		# self.joint[1].position=-(data.data-15258)/16384*2*3.1415926535
		self.joint[1].position = data.position
		self.talker()
	def callback2(self, data):
		# self.joint[2].position=(data.data-10562)/16384*2*3.1415926535
		self.joint[2].position = data.position
		self.talker()
	def callback3(self, data):
		# self.joint[3].position=(data.data-9000)/16384*2*3.1415926535
		self.joint[3].position = data.position
		self.talker()
	def callback4(self, data):
		self.joint[4].position = data.position
		# self.joint[4].position=(data.data-0)/16384*2*3.1415926535
		self.talker()

	def talker(self):
		msg = JointState()
		msg.name = []
		msg.position = []
		msg.velocity = []
		msg.effort = []
		for j in range(4):
		    msg.name.append(self.joint[j].name)
		    msg.position.append(self.joint[j].position)
		    #msg.velocity.append(joint.velocity)
		    #msg.effort.append(joint.effort)
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = ''
		self.joint_states_pub.publish(msg)


if __name__ == '__main__':
    try:
        pup=puppetmanipulatorclass()

    except rospy.ROSInterruptException:
        pass
