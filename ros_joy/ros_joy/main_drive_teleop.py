import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import array
import os
import struct
import sys
import time
from fcntl import ioctl

class MainDriveTeleop(Node):

	def __init__(self, jsdev, axis_map, axis_states, button_map, button_states, num_axes, num_buttons):
		super().__init__('main_drive_teleop')
		self.jsdev = jsdev
		self.axis_map = axis_map
		self.axis_states = axis_states
		self.button_map = button_map
		self.button_states = button_states
		self.num_axes = num_axes
		self.num_buttons = num_buttons

		self.speed_setting = 2

		self.joy_publisher_ = self.create_publisher(Joy, 'xbox_controller_val', 10)
		
		self.m = Joy()
		self.m.axes = [0.0] * self.num_axes
		self.m.buttons = [0] * self.num_buttons

		self.pubJoy()

	def map(self, x, in_min, in_max, out_min, out_max):
		return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

	# 8 axes found: x, y, z, rx, ry, rz, hat0x, hat0y
	# 11 buttons found: a, b, x, y, tl, tr, select, start, mode, thumbl, thumbr
	def pubJoy(self):
		update_timer = time.time()
		while True:
			evbuf = self.jsdev.read(8)
			if evbuf:
				mtime, value, type, number = struct.unpack('IhBB', evbuf)
				if time.time() > (update_timer + 0.01):
					if type & 0x01:
						button = self.button_map[number]
						if button:
							self.button_states[button] = value
							if value:
								self.get_logger().info("%s pressed" % (button))
								self.m.buttons[number] = value
								if number == 6:
									self.m.axes[1] = 0
									self.m.axes[4] = 0
							else:
								self.get_logger().info("%s released" % (button))
								self.m.buttons[number] = value

					if type & 0x02:
						axis = self.axis_map[number]
						if axis:
							self.axis_states[axis] = value
							if number == 0:
								pass
							if number == 1:
								self.m.axes[number] = self.map(value, -32767, 32767, -200 / self.speed_setting, 200 / self.speed_setting)
								self.get_logger().info("%.3f - %s: %.3f" % (mtime, axis, self.m.axes[number]))
							if number == 2:
								self.m.axes[number] = self.map(value, -32767, 32767, -200, 200)
								self.get_logger().info("%.3f - %s: %.3f" % (mtime, axis, self.m.axes[number]))
							if number == 3:
								pass
							if number == 4:
								self.m.axes[number] = self.map(value, -32767, 32767, -200 / self.speed_setting, 200 / self.speed_setting)
								self.get_logger().info("%.3f - %s: %.3f" % (mtime, axis, self.m.axes[number]))
							if number == 5:
								self.m.axes[number] = self.map(value, -32767, 32767, -200, 200)
								self.get_logger().info("%.3f - %s: %.3f" % (mtime, axis, self.m.axes[number]))
							if number == 6:
								self.speed_setting = 2
								self.m.axes[number] = self.speed_setting
								self.get_logger().info("%.3f - %s: %.3f" % (mtime, axis, self.m.axes[number]))
							if number == 7:
								if value > 0:
									self.speed_setting = 1
									self.m.axes[number] = self.speed_setting
								if value < 0:
									self.speed_setting = 3
									self.m.axes[number] = self.speed_setting
								self.get_logger().info("%.3f - %s: %.3f" % (mtime, axis, self.m.axes[number]))
					self.joy_publisher_.publish(self.m)
					update_timer = time.time()


def main(args=None):
	# Iterate over the joystick devices.
	print('Available devices:')
	i = 0
	for fn in os.listdir('/dev/input'):
		if fn.startswith('js'):
			print('  /dev/input/%s' % (fn))
			i += 1
	if i == 0:
		print("No joysticks found. Terminating")
		sys.exit(-1)

	# We'll store the states here.
	axis_states = {}
	button_states = {}

	# These constants were borrowed from linux/input.h
	axis_names = {
		0x00 : 'x',
		0x01 : 'y',
		0x02 : 'z',
		0x03 : 'rx',
		0x04 : 'ry',
		0x05 : 'rz',
		0x06 : 'trottle',
		0x07 : 'rudder',
		0x08 : 'wheel',
		0x09 : 'gas',
		0x0a : 'brake',
		0x10 : 'hat0x',
		0x11 : 'hat0y',
		0x12 : 'hat1x',
		0x13 : 'hat1y',
		0x14 : 'hat2x',
		0x15 : 'hat2y',
		0x16 : 'hat3x',
		0x17 : 'hat3y',
		0x18 : 'pressure',
		0x19 : 'distance',
		0x1a : 'tilt_x',
		0x1b : 'tilt_y',
		0x1c : 'tool_width',
		0x20 : 'volume',
		0x28 : 'misc',
	}

	button_names = {
		0x120 : 'trigger',
		0x121 : 'thumb',
		0x122 : 'thumb2',
		0x123 : 'top',
		0x124 : 'top2',
		0x125 : 'pinkie',
		0x126 : 'base',
		0x127 : 'base2',
		0x128 : 'base3',
		0x129 : 'base4',
		0x12a : 'base5',
		0x12b : 'base6',
		0x12f : 'dead',
		0x130 : 'a',
		0x131 : 'b',
		0x132 : 'c',
		0x133 : 'x',
		0x134 : 'y',
		0x135 : 'z',
		0x136 : 'tl',
		0x137 : 'tr',
		0x138 : 'tl2',
		0x139 : 'tr2',
		0x13a : 'select',
		0x13b : 'start',
		0x13c : 'mode',
		0x13d : 'thumbl',
		0x13e : 'thumbr',

		0x220 : 'dpad_up',
		0x221 : 'dpad_down',
		0x222 : 'dpad_left',
		0x223 : 'dpad_right',

		# XBox 360 controller uses these codes.
		0x2c0 : 'dpad_left',
		0x2c1 : 'dpad_right',
		0x2c2 : 'dpad_up',
		0x2c3 : 'dpad_down',
	}

	axis_map = []
	button_map = []

	print("\nFinding correct joystick")
	for fn in os.listdir('/dev/input'):
		if fn.startswith('js'):
			jsdev = open("/dev/input/" + fn, 'rb')
			buf = array.array('B', [0] * 64)
			ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
			js_name = buf.tobytes().rstrip(b'\x00').decode('utf-8')
		if js_name == "Microsoft X-Box One S pad":
			break
		jsdev.close()
	if js_name != "Microsoft X-Box One S pad":
		print("Joystick not found terminating")
		jsdev.close()
		sys.exit(-1)
	print("Correct joystick found\n")

	# Get number of axes and buttons.
	buf = array.array('B', [0])
	ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
	num_axes = buf[0]

	buf = array.array('B', [0])
	ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
	num_buttons = buf[0]

	# Get the axis map.
	buf = array.array('B', [0] * 0x40)
	ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

	for axis in buf[:num_axes]:
		axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
		axis_map.append(axis_name)
		axis_states[axis_name] = 0.0

	# Get the button map.
	buf = array.array('H', [0] * 200)
	ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

	for btn in buf[:num_buttons]:
		btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
		button_map.append(btn_name)
		button_states[btn_name] = 0

	print('%d axes found: %s' % (num_axes, ', '.join(axis_map)))
	print('%d buttons found: %s' % (num_buttons, ', '.join(button_map)))

	rclpy.init(args=args)

	controller = MainDriveTeleop(jsdev, axis_map, axis_states, button_map, button_states, num_axes, num_buttons)
	rclpy.spin(controller)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	controller.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()