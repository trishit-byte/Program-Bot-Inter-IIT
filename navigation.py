#!/usr/bin/env python

from rospy.core import xmlrpcapi
from std_msgs.msg import Float32,Float64
from sensor_msgs.msg import LaserScan, Imu,NavSatFix,JointState
from geometry_msgs.msg import Quaternion,Twist
import rospy
import tf
import math
import numpy as np

class sbb():
	def __init__(self):

		rospy.init_node('navigation')
		self.a=0
		self.b=0
		self.sbb_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.sbb_orientation_euler = [0.0, 0.0, 0.0]
		self.sample_rate = 50.0  # in Hz
		self.i = 0.0
		self.curr_angle = 0.0
		self.prev_angle = 0.0
		self.reset = 0
		self.w=0.0
		self.data_cmd = Float32()
		self.data_cmd.data = 0.0
		self.handle_angle=0.0
		self.omega=0.0
		self.stand1_omega=0.0
		self.stand2_omega=0.0
		self.d=0.0
		self.data_pub = rospy.Publisher('/flywheel/command', Float32, queue_size=1)
		rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
		self.stand1=rospy.Publisher("/stand1/command", Float32, queue_size = 1)
		self.stand2=rospy.Publisher("/stand2/command", Float32, queue_size = 1)
		self.front_wheel= rospy.Publisher("/handle/command", Float32, queue_size = 1)
		rospy.Subscriber('/handle/joint_state',JointState, self.handle_callback)
		#rospy.Subscriber('/drive_wheel/joint_state',JointState, self.fly_wheel_callback)
		rospy.Subscriber('/stand1/joint_state',JointState,self.stand1_callback)
		rospy.Subscriber('/stand2/joint_state',JointState,self.stand2_callback)

	def imu_callback(self, msg):

	    self.sbb_orientation_quaternion[0] = msg.orientation.x
	    self.sbb_orientation_quaternion[1] = msg.orientation.y
	    self.sbb_orientation_quaternion[2] = msg.orientation.z
	    self.sbb_orientation_quaternion[3] = msg.orientation.w
	def handle_callback(self,msg):
		#print(msg.position[0]," handle angle is ")
		self.handle_angle=msg.position[0]
	def fly_wheel_callback(self,msg):
		print(msg.velocity,"fly wheel actual velocity")
		print(msg.effort,"fly wheel effort")
	def stand1_callback(self,msg):
		self.stand1_omega=msg.position[0]
	def stand2_callback(self,msg):
		self.stand2_omega=msg.position[0]

	
	
	def pid(self):
	    rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
	    self.handle_pid()
	    self.stand1_pid()
	    self.stand2_pid()
	    (self.sbb_orientation_euler[1], self.sbb_orientation_euler[0], self.sbb_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.sbb_orientation_quaternion[0], self.sbb_orientation_quaternion[1], self.sbb_orientation_quaternion[2], self.sbb_orientation_quaternion[3]])
	    print(math.degrees(self.sbb_orientation_euler[1]),"degree is")
	    self.curr_angle = self.sbb_orientation_euler[1]


	    print((self.sbb_orientation_euler[2]),"degree about z is")
	    #print((self.sbb_orientation_euler[0]),"degree about x is")


	    #rospy.loginfo("angle: " + str(self.curr_angle))
            kp=100
            kd=0
            dt=0.02
            ki=0
            self.i=(self.i + self.curr_angle*dt)
	    self.a=self.curr_angle-self.prev_angle
            #self.w=(kp*math.degrees(self.curr_angle)) + (0)*(self.curr_angle-self.prev_angle) +(ki*self.i) +(0)*(self.a-self.b)
	    #if(self.curr_angle>0):
		#self.w=3
	    #else:
		#self.w=-3
	    #if(math.degrees(self.sbb_orientation_euler[1])>-3 and math.degrees(self.sbb_orientation_euler[1])<3):
		#self.w=0.0
	    self.w= -(kp*self.curr_angle)-(kd*(self.curr_angle-self.prev_angle))+(ki*self.i)    
            
	    #self.data_cmd.data= 6
	    #if(self.w>=6):
		#self.w=6
	    #if(self.w<-6):
		#self.w=-6
	    self.d= self.w
            print("flywheel w: "+str(self.w))
	    #print("self d is ",self.d)
            self.data_pub.publish(self.d)
	    #print("flywheel velocity is :",-100*self.curr_angle)
	    #self.data_pub.publish((100)*(self.curr_angle))
	    #self.data_pub.publish((-50)*(self.curr_angle))
            #self.prev_angle=self.curr_angle
	    self.b=self.a
	def handle_pid(self):
		rospy.Subscriber('/handle/joint_state',JointState, self.handle_callback)
		self.omega=-5*self.handle_angle
		#print("handle omega is ",self.omega)
		self.front_wheel.publish(self.omega)

	def stand1_pid(self):
		rospy.Subscriber('/stand1/joint_state',JointState,self.stand1_callback)
		omega=-5*(self.stand1_omega)
		self.stand1.publish(omega)
		#self.stand1.publish(-5)
	def stand2_pid(self):
		rospy.Subscriber('/stand2/joint_state',JointState,self.stand2_callback)
		omega=-5*(self.stand2_omega)
		self.stand2.publish(omega)
		#self.stand2.publish(5)		    
	    



class nav():
	def __init__(self):
		rospy.init_node('navigation')
		####################
		self.initial_angle=0
		self.k=0
		self.lidar_switch=0
		self.rotation_switch=0
		self.moving_switch=0
		self.vector1=[math.sin(math.radians(68)),-math.cos(math.radians(68)),0]
		self.vector2=[1,0,0]
		self.vector3=[math.sin(math.radians(68)),math.cos(math.radians(68)),0]
		self.coordinates=[0,0,0]
		self.target_coordinates=[0,0,0]
		self.current_vector=[0,0,0]
		self.first_angle=0
		self.second_angle=0
		self.third_angle=0
		self.range=[0]*360
		self.max_radius=7.5
		self.region1=0
		self.region2=0
		self.region3=0
		self.theta=0
		self.a_pid=0
		self.phi=0
		self.handle_wheel= rospy.Publisher("/handle/command", Float32, queue_size = 1)
		rospy.Subscriber('/sbb/distance_sensor/front', LaserScan, self.lidar_callback)
		rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
		rospy.Subscriber('/sbb/gps', NavSatFix, self.gps_callback)
		self.back_wheel = rospy.Publisher("/drive_wheel/command", Float32, queue_size = 1)


#######################################################################
		self.a=0
		self.b=0
		self.sbb_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]
		self.sbb_orientation_euler = [0.0, 0.0, 0.0]
		self.sample_rate = 50.0  # in Hz
		self.i = 0.0
		self.curr_angle = 0.0
		self.prev_angle = 0.0
		self.reset = 0
		self.w=0.0
		self.data_cmd = Float32()
		self.data_cmd.data = 0.0
		self.handle_angle=0.0
		self.omega=0.0
		self.stand1_omega=0.0
		self.stand2_omega=0.0
		self.d=0.0
		self.data_pub = rospy.Publisher('/flywheel/command', Float32, queue_size=1)
		rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
		self.stand1=rospy.Publisher("/stand1/command", Float32, queue_size = 1)
		self.stand2=rospy.Publisher("/stand2/command", Float32, queue_size = 1)
		self.front_wheel= rospy.Publisher("/handle/command", Float32, queue_size = 1)
		rospy.Subscriber('/handle/joint_state',JointState, self.handle_callback)
		#rospy.Subscriber('/drive_wheel/joint_state',JointState, self.fly_wheel_callback)
		rospy.Subscriber('/stand1/joint_state',JointState,self.stand1_callback)
		rospy.Subscriber('/stand2/joint_state',JointState,self.stand2_callback)



	def switch(self):

		if(self.lidar_switch==0):
			print("-------------------------")
			self.identify()
			self.lidar_switch=1
			self.rotation_switch=1
			self.moving_switch=0
			self.k=0
			print("-------------------------")
		if(self.rotation_switch==1):
			print("-------------------------")
			self.deciding()
			print("-------------------------")
		# if(self.moving_switch==1):
		# 	rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
		# 	self.pid(self.theta+math.radians(0))
		# 	self.pidfull()
		# 	self.back_wheel.publish(3)
		# 	self.lidar_switch=0
		# 	self.rotation_switch=0
		# 	self.moving_switch=0


	
	#####################################
	def imu_callback(self, msg):
	    self.sbb_orientation_quaternion[0] = msg.orientation.x
	    self.sbb_orientation_quaternion[1] = msg.orientation.y
	    self.sbb_orientation_quaternion[2] = msg.orientation.z
	    self.sbb_orientation_quaternion[3] = msg.orientation.w
	
	def handle_callback(self,msg):
		#print(msg.position[0]," handle angle is ")
		self.handle_angle=msg.position[0]
	
	def fly_wheel_callback(self,msg):
		print(msg.velocity,"fly wheel actual velocity")
		print(msg.effort,"fly wheel effort")
	
	def stand1_callback(self,msg):
		self.stand1_omega=msg.position[0]
	
	def stand2_callback(self,msg):
		self.stand2_omega=msg.position[0]
	
	def pidfull(self):
	    rospy.Subscriber('/sbb/imu', Imu, self.imu_callback)
	    #self.handle_pid()
	    self.stand1_pid()
	    self.stand2_pid()
	    (self.sbb_orientation_euler[1], self.sbb_orientation_euler[0], self.sbb_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.sbb_orientation_quaternion[0], self.sbb_orientation_quaternion[1], self.sbb_orientation_quaternion[2], self.sbb_orientation_quaternion[3]])
	    print(math.degrees(self.sbb_orientation_euler[1]),"degree is")
	    self.curr_angle = self.sbb_orientation_euler[1]


	    print((self.sbb_orientation_euler[2]),"degree about z is")
	    #print((self.sbb_orientation_euler[0]),"degree about x is")


	    #rospy.loginfo("angle: " + str(self.curr_angle))
            kp=100
            kd=0
            dt=0.02
            ki=0
            self.i=(self.i + self.curr_angle*dt)
	    self.a=self.curr_angle-self.prev_angle
	    self.w= -(kp*self.curr_angle)-(kd*(self.curr_angle-self.prev_angle))+(ki*self.i)    
            
	    #self.data_cmd.data= 6
	    #if(self.w>=6):
		#self.w=6
	    #if(self.w<-6):
		#self.w=-6
	    self.d= self.w
            print("flywheel w: "+str(self.w))
	    #print("self d is ",self.d)
            self.data_pub.publish(self.d)
	    #print("flywheel velocity is :",-100*self.curr_angle)
	    #self.data_pub.publish((100)*(self.curr_angle))
	    #self.data_pub.publish((-50)*(self.curr_angle))
            #self.prev_angle=self.curr_angle
	    self.b=self.a
	
	def handle_pid(self):
		rospy.Subscriber('/handle/joint_state',JointState, self.handle_callback)
		self.omega=-5*self.handle_angle
		#print("handle omega is ",self.omega)
		self.front_wheel.publish(self.omega)

	
	def stand1_pid(self):
		rospy.Subscriber('/stand1/joint_state',JointState,self.stand1_callback)
		omega=-5*(self.stand1_omega)
		self.stand1.publish(omega)
		#self.stand1.publish(-5)
	
	def stand2_pid(self):
		rospy.Subscriber('/stand2/joint_state',JointState,self.stand2_callback)
		omega=-5*(self.stand2_omega)
		self.stand2.publish(omega)
		#self.stand2.publish(5)		    
	
	
	
	#####################################################
	
	


	def update_value(self):
		self.region1=1
		self.region2=1
		self.region3=1
	def handle_controller(self,msg):
		self.theta=msg.position[0]
		print(math.degrees(self.theta),"handle angle")
	def lidar_callback(self,msg):
		# for i in range(len(msg.ranges)):
		# 	self.range[i]=msg.ranges[i]
		self.range=list(msg.ranges)
		#print(msg.ranges,"real")
		#print(self.range,"actual")	
	def gps_callback(self,msg):
		phi = np.deg2rad(msg.latitude)
    		lamb = np.deg2rad(msg.longitude)
    		#constant(WGS-For 84 reference ellipsoids)
    		a = 6378137          #Semimajor axis[m]
    		e = 0.0818191908426  #Eccentricity[-]
    		h=msg.altitude
    		N = a / np.sqrt(1 - (e ** 2 * np.sin(phi) ** 2))
    		x = (N + h) * np.cos(phi) * np.cos(lamb)-6186437.331874527
    		y = (N + h) * np.cos(phi) * np.sin(lamb)-1090835.8159847995
    		z = (N * (1 - e ** 2) + h) * np.sin(phi)-1100248.6012548439
		#print(z,-y,x)
		self.coordinates[0]=z
		self.coordinates[1]=-y
		#self.coordinates[2]=x
		#print(self.coordinates)
	def targetcoordinates(self):
		phi = np.deg2rad(10.0001415852)
    		lamb = np.deg2rad(10.0003820894)
    		#constant(WGS-For 84 reference ellipsoids)
    		a = 6378137          #Semimajor axis[m]
    		e = 0.0818191908426  #Eccentricity[-]
    		h=0.27505924203415416
    		N = a / np.sqrt(1 - (e ** 2 * np.sin(phi) ** 2))
    		x = (N + h) * np.cos(phi) * np.cos(lamb)-6186437.331874527
    		y = (N + h) * np.cos(phi) * np.sin(lamb)-1090835.8159847995
    		z = (N * (1 - e ** 2) + h) * np.sin(phi)-1100248.6012548439
		#print(z,-y,x)
		self.target_coordinates[0]=z
		self.target_coordinates[1]=-y
		self.target_coordinates[2]=0
		self.vector1[2]=0
		self.vector2[2]=0
		self.vector3[2]=0
		#print(self.coordinates)
	def dotproduct(self,v1, v2):
  		return sum((a*b) for a, b in zip(v1, v2))
	def length(self,v):
 		return math.sqrt(self.dotproduct(v, v))
	def angle(self,v1, v2):
  		return math.acos(self.dotproduct(v1, v2) / (self.length(v1) * self.length(v2)))

	def closest_region(self):
		print("enter closest region")
		self.targetcoordinates()
		rospy.Subscriber('/sbb/gps', NavSatFix, self.gps_callback)
		arr=np.array(self.target_coordinates)-np.array(self.coordinates)
		self.current_vector=arr.tolist()
		print("current vector",self.current_vector)
		#print("vectors",self.vector1,self.vector2,self.vector3)
		self.first_angle=abs(self.angle(self.current_vector,self.vector1))
		self.second_angle=abs(self.angle(self.current_vector,self.vector2))
		self.third_angle=abs(self.angle(self.current_vector,self.vector3))

	def identify(self):
		print("enter identify")
		self.update_value()
		rospy.Subscriber('/sbb/distance_sensor/front', LaserScan, self.lidar_callback)
		#print(self.region1+self.region2+self.region3,"initial sum checker")
		#print(self.range)
		for i in range(136-5,136+5):
			print(self.range[i],i)
			if(self.range[i]<=self.max_radius):
				#print("obstacles in right")
				self.region1=0
		for i in range(180-5,180+5):
			print(self.range[i],i)
			if(self.range[i]<=self.max_radius):
				#print("obstacles in straight")
				self.region2=0	
		for i in range(224-5,224+5):
			print(self.range[i],i)
			if(self.range[i]<=self.max_radius):
				#print("obstacles in left")
				self.region3=0	
		print(self.region1+self.region2+self.region3,"final sum checker")
		#self.deciding()
		self.closest_region()

	def pid(self,angle):
		print("enter rotation pid")
		b=self.theta-angle
		
		c=-5*(self.theta-angle)-2*(b-self.a_pid)
		if(c>1):
			c=1
		if(c<-1):
			c=-1
		
		print(abs(math.degrees(self.theta-angle)),"angle difference")
		
		self.handle_wheel.publish(c)
		if(abs(math.degrees(b)<0.1)):
			print("thershold reached of rotation")
			self.rotation_switch=0
			self.lidar_switch=0
			#self.moving_switch=1

		#handle_wheel.publish(-5*(theta-angle)-2*b)
		#handle_wheel.publish(-4*(theta-angle))
		self.a_pid=b
	def deciding(self):
		print("enter deciding")
		print(self.region1+self.region2+self.region3,"deciding sum checker")
		# self.closest_region()
		
		if((self.region1+self.region2+self.region3)==1):
			# rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
			if(self.region1==1):
				rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
				if(self.k==0):
					self.initial_angle=self.theta
					self.k=1
				self.pid(self.initial_angle+math.radians(180))
				print("left with one option")
			elif(self.region2==1):
				rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
				if(self.k==0):
					self.initial_angle=self.theta
					self.k=1
				self.pid(self.initial_angle+math.radians(0))
				print("straight with one option")
			elif(self.region3==1):
				rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
				if(self.k==0):
					self.initial_angle=self.theta
					self.k=1
				self.pid(self.initial_angle+math.radians(-180))
				print("right one option")
		
		elif((self.region1+self.region2+self.region3)==2):
			self.closest_region()
			if(self.region1==1 and self.region2==1):
				if(self.first_angle<self.second_angle):
					rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
					if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
					self.pid(self.initial_angle+math.radians(180))
					print("left with two option")
				else:
					rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
					if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
					self.pid(self.initial_angle+math.radians(0))
					print("st with two option")
			elif(self.region1==1 and self.region3==1):
				if(self.first_angle<=self.third_angle):
					rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
					if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
					self.pid(self.initial_angle+math.radians(180))
					print("left with two option")
				else:
					rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
					if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
					self.pid(self.initial_angle+math.radians(-180))
					print("right with two option")
			elif(self.region3==1 and self.region2==1):
				if(self.third_angle<self.second_angle):
					rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
					if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
					self.pid(self.initial_angle+math.radians(-180))
					print("right with two option")
				else:
					rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
					if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
					self.pid(self.initial_angle+math.radians(0))
					print("st with two option")
		
		elif((self.region1+self.region2+self.region3)==3):
			self.closest_region()
			minimum=min(self.first_angle,self.second_angle,self.third_angle)
			if(minimum==self.second_angle):
				rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
				if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
				self.pid(self.initial_angle+math.radians(0))
				print("st with 3 option")
			elif(minimum==self.first_angle):
				rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
				if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
				self.pid(self.initial_angle+math.radians(180))
				print("left with 3 option")
			elif(minimum==self.third_angle):
				rospy.Subscriber('/handle/joint_state',JointState, self.handle_controller)
				if(self.k==0):
						self.initial_angle=self.theta
						self.k=1
				self.pid(self.initial_angle+math.radians(-180))
				print("right with 3 option")			
		else:
			print("all regions filled")
			self.lidar_switch=0
			self.rotation_switch=0
			self.moving_switch=0
		
		


def gps_callback(msg):
	print(msg.ranges)

def callback(msg):
	print(msg.latitude, " latitude is")
	print(msg.longitude, "longitude is ")
	#print(type(msg.latitude))
	#print(type(msg.latitude[0]))
	x = utm.from_latlon(msg.latitude, msg.longitude)
	#print(x,"  :")
	
def imu_callback(msg):
	print(msg)
theta=0
def handle_controller(msg):
	global theta
	theta=msg.position[0]

	print(math.degrees(theta)," handle angle in degree is") 
a=0
def pid(angle):
	global a,theta
	b=theta-angle
	handle_wheel= rospy.Publisher("/handle/command", Float32, queue_size = 1)
	c=-5*(theta-angle)-2*b
	if(c>1):
		c=1
	if(c<-1):
		c=-1
	#if(abs(math.degrees(theta-angle))<0.5):
		#c=0
	handle_wheel.publish(c)
	#handle_wheel.publish(-5*(theta-angle)-2*b)
	#handle_wheel.publish(-4*(theta-angle))
	a=b
		

if __name__ == '__main__':

    sbb = sbb()
    r = rospy.Rate(sbb.sample_rate)  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    #back_wheel = rospy.Publisher("/drive_wheel/command", Float32, queue_size = 1)
    #front_wheel= rospy.Publisher("/handle/command", Float32, queue_size = 1)
    #stand1=rospy.Publisher("/stand1/command", Float32, queue_size = 1)
    #stand2=rospy.Publisher("/stand2/command", Float32, queue_size = 1)
    #fly_wheel=rospy.Publisher('/flywheel/command', Float32, queue_size=1)
    nav=nav()
    
    #vel=Twist()
    while not rospy.is_shutdown():
        try: 
			#print("################################################")
			#nav=nav()
			sbb.stand1_pid()
			sbb.stand2_pid()
			nav.switch()
			#sbb.pid()
			#vel=3
			#sub=rospy.Subscriber('/sbb/distance_sensor/front', LaserScan, gps_callback)
			#subject=rospy.Subscriber('/sbb/gps', NavSatFix, callback)
			#rospy.Subscriber('/stand1/joint_state',JointState,stand1_callback)
			#rospy.Subscriber('/flywheel/joint_state',JointState, callback)
			#back_wheel.publish(vel)
			#front_wheel.publish(2)
			#rospy.Subscriber('/handle/joint_state',JointState, handle_controller)
			#pid(math.radians(180))
			#fly_wheel.publish(50)
			#stand1.publish(-5)
			#stand2.publish(5)	
			#print("helloji")
			#rospy.Subscriber('/sbb/imu', Imu, imu_callback,queue_size=1)
			#print("----------------------------------------------------------")
			r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException: pass
