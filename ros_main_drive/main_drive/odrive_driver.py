import rclpy
from rclpy.node import Node

import odrive
from odrive.enums import *

from sensor_msgs.msg import Joy

class OdriveSubscriber(Node):

	def __init__(self, front, rear):
		super().__init__('odrive_driver')
		self.subscription = self.create_subscription(Joy, 'xbox_controller_val', self.listener_callback, 10)
		self.subscription  # prevent unused variable warning
		self.front = front
		self.rear = rear
		self.fullStop()
		self.reset()
		self.left = 0
		self.right = 0

	def listener_callback(self, msg):
		self.get_logger().info("Received values: " + str(msg.axes[1]) + ", " + str(msg.axes[4]))
		if msg.buttons[7] == 1:
			self.reset()
		self.left = msg.axes[1]
		self.right = msg.axes[4]
		self.updateMotorValues()

	def reset(self):
		self.front.axis0.error = 0
		self.front.axis1.error = 0
		self.rear.axis0.error = 0
		self.rear.axis1.error = 0
		self.front.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.front.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.rear.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.rear.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

	def fullStop(self):
		self.left = 0
		self.right = 0
		self.reset()
		self.front.axis0.controller.vel_setpoint = 0
		self.front.axis1.controller.vel_setpoint = 0
		self.rear.axis0.controller.vel_setpoint = 0
		self.rear.axis1.controller.vel_setpoint = 0

	def updateMotorValues(self):
		self.reset()
		self.get_logger().info("left = " + str(self.left))
		self.get_logger().info("right = " + str(self.right))
		self.front.axis0.controller.vel_setpoint = -self.left
		self.front.axis1.controller.vel_setpoint = self.right
		self.rear.axis0.controller.vel_setpoint = self.right
		self.rear.axis1.controller.vel_setpoint = -self.left

def main(args=None):
	print("Connecting to odrive front...")
	front = odrive.find_any(serial_number="20763593524B")
	print("Connecting to odrive rear...")
	rear = odrive.find_any(serial_number="205F3599524B")
	print("All drives connected")

	rclpy.init(args=args)

	odrive_subscriber = OdriveSubscriber(front, rear)
	rclpy.spin(odrive_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	odrive_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()