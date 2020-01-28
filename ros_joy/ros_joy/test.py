#!/usr/bin/env python3

# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import array
import os
import struct
import sys
# import time
from fcntl import ioctl

def map(x, in_min, in_max, out_min, out_max):
		return int((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)
def main():
	# Iterate over the joystick devices.
	print('Available devices:')

	for fn in os.listdir('/dev/input'):
		if fn.startswith('js'):
			print('  /dev/input/%s' % (fn))

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
			# print('  /dev/input/%s' % (fn))
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

	maxx = 0
	minx = 0

	maxy = 0
	miny = 0

	maxrx = 0
	minrx = 0

	maxry = 0
	minry = 0

	maxrz = 0
	minrz = 0

	maxz = 0
	minz = 0
	try:	
		while True:
			evbuf = jsdev.read(8)
			if evbuf:
				time, value, type, number = struct.unpack('IhBB', evbuf)

				# if type & 0x80:
				# 	print("(initial)", end="")

				if type & 0x01:
					button = button_map[number]
					if button:
						button_states[button] = value
						if value:
							print("%s pressed" % (button))
						else:
							print("%s released" % (button))

				if type & 0x02:
					axis = axis_map[number]
					if axis:
						fvalue = value
						axis_states[axis] = map(fvalue, -32767, 32767, -1000, 1000)
						print(number)
						print("%.3f - %s: %.3f" % (time, axis, fvalue))
						if axis == "x":
							if fvalue > maxx:
								maxx = fvalue
							if fvalue < minx:
								minx = fvalue
						if axis == "y":
							if fvalue > maxy:
								maxy = fvalue
							if fvalue < miny:
								miny = fvalue
						if axis == "z":
							if fvalue > maxz:
								maxz = fvalue
							if fvalue < minz:
								minz = fvalue
						if axis == "rx":
							if fvalue > maxrx:
								maxrx = fvalue
							if fvalue < minrx:
								minrx = fvalue
						if axis == "ry":
							if fvalue > maxry:
								maxry = fvalue
							if fvalue < minry:
								minry = fvalue
						if axis == "rz":
							if fvalue > maxrz:
								maxrz = fvalue
							if fvalue < minrz:
								minrz = fvalue
	except Exception as e:
		print(e)
	finally:
		print("\nmax x: " + str(maxx))
		print("max y: " + str(maxy))
		print("max z: " + str(maxz))
		print("max rx: " + str(maxrx))
		print("max ry: " + str(maxry))
		print("max rz: " + str(maxrz))
		print("min x: " + str(minx))
		print("min y: " + str(miny))
		print("min z: " + str(minz))
		print("min rx: " + str(minrx))
		print("min ry: " + str(minry))
		print("min rz: " + str(minrz))


if __name__ == "__main__":
	main()