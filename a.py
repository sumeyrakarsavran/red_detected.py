#!/usr/bin/env python
# ROS python API
import rospy, sys, cv2
import numpy as np

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist, Pose, Quaternion, TwistStamped
from sensor_msgs.msg import NavSatFix, Image
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String, Float64
from decimal import *
from math import radians, cos, sin, asin, sqrt
from cv_bridge import CvBridge, CvBridgeError

# Message publisher for haversine
velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
msg1 = Twist()

# Current Position
latitude = 47.397742
longitude = 8.5455936
altitude = 0.0

# Position before Move function execute
previous_latitude = 47.397742
previous_longitude = 8.5455936
previous_altitude = 0.0

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
# Flight modes class
# Flight modes are activated using ROS services

# cv2 bridge
bridge = CvBridge()
cv_image = ""

#callback for image
def image_callback(ros_image):
    global bridge, cv_image
    try:
	cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
	print(e)
    while True:
	kernel = np.ones((5,5),np.float32)/25
        color_boundaries = {
    	"red":    ([0,   0,   80], [0, 255,   255]),
    }
	frame=cv_image
        frame = cv2.GaussianBlur(frame, (5, 5), 0) # kirmizi noktanin ortasindaki puru
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        for color_name, (lower, upper) in color_boundaries.items():
        # create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = np.uint8)
            upper = np.array(upper, dtype = np.uint8)
            # find the colors within the specified boundaries and apply the mask
            mask = cv2.inRange(frame, lower, upper)
            res = cv2.bitwise_and(frame, frame, mask=mask)
            if mask.any():
                print("******RED COLOR DETECTED******")
	break



class fcuModes:
    def _init_(self):
        pass

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
  	try:
    		takeoffService = rospy.ServiceProxy(
        '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude=10, latitude=47.397742,
                   longitude=8.5455936, min_pitch=0, yaw=0)
  	except rospy.ServiceException, e:
   	  print "Service takeoff call failed: %s" % e

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for arming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print ("Service arming call failed: %s"%e)

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Waiting for disarming...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print ("Service disarming call failed: %s"%e)

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            print("It's stabilazed mode!")
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set."%e)

    def setOffboardMode(self):
	cnt = Controller()
        rate = rospy.Rate(20.0)
	sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        rospy.wait_for_service('/mavros/set_mode')
       
	try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e
            return False

    def setLoiterMode(self):
   	rospy.wait_for_service('/mavros/set_mode')
   	try:
       	    flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
       	    isModeChanged = flightModeService(custom_mode='AUTO.LOITER') #return true or false
   	except rospy.ServiceException as e:
       	    print ("service set_mode call failed: %s. AUTO.LOITER Mode could not be set. Check that GPS is enabled %s"%e)

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set."%e)

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print ("service set_mode call failed: %s. Position Mode could not be set."%e)

    def setLandMode(self):
	global pos_mode
	print("SET LAND")
	pos_mode = False
	rospy.wait_for_service('/mavros/cmd/land')
	try:
		landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
		isLanding = landService(altitude = 0)
	except rospy.ServiceException, e:
		print "service land call failed: %s. The vehicle cannot land "%e

class Controller:

    # initialization method
    def _init_(self):
        # Drone state
        self.state = State() #using that msg for send few setpoint messages, then activate OFFBOARD mode, to take effect
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

def haversine():

        global previous_longitude, previous_latitude, longitude, latitude

	#print("calculating distance")
  	lon1, lat1, lon2, lat2 = previous_longitude, previous_latitude, longitude, latitude

  	# lat, long in radians
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

   	# difference lat, long
	dlon = abs(lon2 - lon1) 
 	dlat = abs(lat2 - lat1)

        # haversine formula 
	a = sin(dlat/2)*2 + cos(lat1) * cos(lat2) * sin(dlon/2)*2
	c = 2 * asin(sqrt(a)) 
	r = 6371 # Radius of earth in kilometers

	#print("distance calculated:", c*r*1000)
	return Decimal(c * r*1000 )
 
def moveX(distance, speed):

        global previous_longitude, previous_latitude, longitude, latitude
	msg1.linear.x = speed
        rate = rospy.Rate(20.0)
	modes = fcuModes()

	while not rospy.is_shutdown():
		#print("going to position")
		haversine_distance = haversine()

		# break loop if desired distance covered 
		if haversine_distance > distance:
			break

		velocity_pub.publish(msg1)
		rate.sleep()

	print("Latitude: {:.7f} ,Longitude: {:.7f}\nDistance Covered: {:.7f}".format(latitude, longitude, haversine_distance))

	msg1.linear.x = 0.
	msg1.linear.y = 0.
	msg1.linear.z = 0.
	for i in range(100):
		velocity_pub.publish(msg1)
	print("Speed X: {} ,Speed Y: {} ,Speed Z: {} ".format(msg1.linear.x, msg1.linear.y, msg1.linear.z))

	previous_latitude = latitude
	previous_longitude = longitude

def moveY(distance, speed):
        global previous_longitude, previous_latitude, longitude, latitude
	msg1.linear.y = speed
        rate = rospy.Rate(20.0)
	modes = fcuModes()

	while not rospy.is_shutdown():
		#print("going to position")
		haversine_distance = haversine()

		# break loop if desired distance covered 
		if haversine_distance > distance:
			break
		velocity_pub.publish(msg1)
		rate.sleep()

	print("Latitude: {:.7f} ,Longitude: {:.7f}\nDistance Covered: {:.7f}".format(latitude, longitude, haversine_distance))

	msg1.linear.x = 0.
	msg1.linear.y = 0.
	msg1.linear.z = 0.
	for i in range(100):
		velocity_pub.publish(msg1)

	print("Speed X: {} ,Speed Y: {} ,Speed Z: {} ".format(msg1.linear.x, msg1.linear.y, msg1.linear.z))

	# after reaching desired position, set home_latitude, home_longitude to current position value
	previous_latitude = latitude
	previous_longitude = longitude

def testDrawSquare4M():
	moveX(5, 2)
	moveY(3, 2)
	moveX(10, -2)
	moveY(3, -2)
	moveX(4, 2)




velocity = TwistStamped()

def vel_callback(vel):
    global velocity
    print("vel_working")
    velocity = vel
    velocity_publisher.publish(velocity)




	
# Main function
def main():

    # initiate node
    rospy.init_node('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes()

    # controller object
    cnt = Controller()

    # ROS loop rate
    rate = rospy.Rate(20.0)

    # Subscribe to drone state
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)



    ###teknofest_mission_2 daki velocity lere bak ordan al yaz dene, msg1, msg2 leri print yap bak ne cikacak
    # Message publisher for local velocity
    velocityPub = rospy.Publisher ('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    msg1 = PositionTarget ()
    zVelocityPub = rospy.Publisher ('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    msg2 = Twist ()

    velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)
    #sub_vel = rospy.Subscriber("cont_pos_msg", TwistStamped , vel_callback )



    # Subscribe to drone image
    image_sub = rospy.Subscriber ("iris/camera/rgb/image_raw", Image, image_callback)


    # Setpoint publisher    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    

    #while not rospy.is_shutdown ():





    # Make sure the drone is armed
    """while not cnt.state.armed:
        modes.setArm()
        rate.sleep()
	
    modes.setTakeoff()
    rospy.sleep(8)


    # sending few setpoint messages for activate OFFBOARD mode, to take effect
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1 


    print("MAIN: SET OFFBOARD")
    # activate OFFBOARD mode"""
    """modes.setOffboardMode()
    testDrawSquare4M()
    modes.setLoiterMode()
    rospy.sleep(10)
    modes.setLandMode()"""

if _name_ == '_main_':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
