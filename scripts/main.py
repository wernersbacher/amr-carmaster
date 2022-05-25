import rospy
from ros_pi_pwm.msg import PWMArray, PWM
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

FREQ = 50

def saturate_number(x, lower, upper):
	return min(max(x, lower), upper)
    

class ServoConvert:
	# range is 5-10% (1-2ms)
	def __init__(self, id=1, center_value=7.5, range=5, direction=1):
		self.value = 0.0
		self.value_out = center_value
		self._center = center_value
		self._range = range
		self._half_range = 0.5 * range
		self._dir = direction
		self.id = id

		# --- Convert its range in [-1, 1]
		self._sf = 1.0 / self._half_range

	def get_value_out(self, value_in):
		# --- value is in [-1, 1]
		self.value = value_in
		self.value_out = self._dir * value_in * self._half_range + self._center
		rospy.logdebug(f"IN: {self.id}, OUT: {self.value_out}")
		return self.value_out


class DkLowLevelCtrl:
	def __init__(self):
		rospy.loginfo("Setting Up the Node...")

		rospy.init_node('carmaster')

		self.max_speed = rospy.get_param('~max_speed', 1.0)
		rospy.loginfo(f"Max Speed of {self.max_speed} allowed.")
		self.rate = rospy.get_param('~hz', 10)
		rospy.loginfo(f"Refreshing at {self.rate} HZ.")

		self.actuators = {
			'steering': ServoConvert(id=1,
									center_value=rospy.get_param('~steering_center', 7.5)),
			'throttle': ServoConvert(id=2, 
									center_value=rospy.get_param('~engine_center', 7.5)) 
			}
		rospy.loginfo("> Actuators corrrectly initialized")

		self._servo_msg = PWMArray()
		for i in range(2):
			self._servo_msg.pwms.append(PWM())

		# --- Create the servo array publisher
		self.ros_pub_servo_array = rospy.Publisher("/pwm_listener", PWMArray, queue_size=1)
		rospy.loginfo("> Publisher corrrectly initialized")

		# --- Create the Subscriber to Twist commands
		self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.set_actuators_from_cmdvel)
		rospy.loginfo("> Subscriber /cmd_vel corrrectly initialized")

		# --- Get the last time we got a command
		self._last_time_cmd_rcv = time.time()
		self._timeout_cmd = 10

		# --- Create the Subscriber to lifeping
		self.ros_sub_twist = rospy.Subscriber("/emergency_stop", Bool, self.check_emergency_state)
		self._last_time_emergency_rcv = time.time()
		self._last_emergency_msg = True   # no driving allowed until we have positive message from /emergency_stop
		self._timeout_emergency = 1  # after 1 second message timeout, bring the car to a stop
		self._last_emergency_state = True  # don't touch here, it's to save the last state of property is_emergency_stop
		rospy.loginfo("> Subscriber /emergency_stop corrrectly initialized")

		rospy.loginfo("Initialization complete")

	def check_emergency_state(self, stop_required):
		self._last_emergency_msg = stop_required.data
		self._last_time_emergency_rcv = time.time()

	@property
	def is_emergency_stop(self):
		rospy.logdebug(time.time() - self._last_time_emergency_rcv)
		# if emergency stop is set to true or the last message is too long ago, we set the state to TRUE
		state = self._last_emergency_msg or time.time() - self._last_time_emergency_rcv > self._timeout_emergency
		if state and not self._last_emergency_state:
			rospy.logwarn(" !!!! ------ Emergency STOP  ------ !!!! ")

		elif not state and self._last_emergency_state:
			rospy.logwarn(" >>>> ------ Driving is allowed ------ <<<< ")
		
		self._last_emergency_state = state
		return state

	def set_actuators_from_cmdvel(self, message):
		"""
		Get a message from cmd_vel, assuming a maximum input of 1
		"""

		# -- Save the time
		self._last_time_cmd_rcv = time.time()

		
		lin_x = saturate_number(message.linear.x, -1, 1) * self.max_speed
		ang_z = saturate_number(message.angular.z, -1, 1)
		
		if not (-1 <= message.linear.x <= 1):
			rospy.logwarn("Movement message is bigger than 1 (or smaller than -1)! Will be saturated!")
			rospy.logwarn(f"message.linear.x={message.linear.x}")
			

		if not (-1 <= message.angular.z <= 1):
			rospy.logwarn("Movement message is bigger than 1! (or smaller than -1)! Will be saturated!")
			rospy.logwarn(f"message.angular.z={message.angular.z}")
			

		# -- Convert vel into servo values
		self.actuators['throttle'].get_value_out(lin_x)
		self.actuators['steering'].get_value_out(ang_z)
		rospy.logdebug("Got a command v = %2.1f  s = %2.1f" % (lin_x, ang_z))
		self.send_servo_msg()

	def set_actuators_idle(self):
		# -- Convert vel into servo values
		self.actuators['throttle'].get_value_out(0)
		self.actuators['steering'].get_value_out(0)
		rospy.loginfo("Setting actuators to idle")
		self.send_servo_msg()

	def send_servo_msg(self):
		for actuator_name, servo_obj in self.actuators.items():
			self._servo_msg.pwms[servo_obj.id - 1].pwm_num = servo_obj.id - 1
			self._servo_msg.pwms[servo_obj.id - 1].freq = FREQ
			self._servo_msg.pwms[servo_obj.id - 1].duty = servo_obj.value_out
			rospy.logdebug("Sending to %s command %f" % (actuator_name, servo_obj.value_out))

		self.ros_pub_servo_array.publish(self._servo_msg)

	@property
	def is_controller_connected(self):
		rospy.logdebug(time.time() - self._last_time_cmd_rcv)
		return time.time() - self._last_time_cmd_rcv < self._timeout_cmd

	def run(self):

		# --- Set the control rate
		rate = rospy.Rate(self.rate)

		while not rospy.is_shutdown():
			rospy.loginfo(f"Emergency Stop = {self.is_emergency_stop}, Connected = {self.is_controller_connected}")
			if self.is_emergency_stop or not self.is_controller_connected:
				self.set_actuators_idle()

			rate.sleep()


if __name__ == "__main__":
	carcontroller = DkLowLevelCtrl()
	carcontroller.run()
