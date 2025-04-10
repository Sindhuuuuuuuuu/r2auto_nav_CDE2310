import time
import busio
import board
import adafruit_amg88xx
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import RPi.GPIO as GPIO

FRONT_HEAT_LAUNCH = 33
FRONT_HEAT_FORWARD = 29
LEFT_RIGHT_HEAT_THRESHOLD = 26

class AMG8833Node(Node):
	def __init__(self):
		super().__init__('amg8833_node')

		# Thermal sensor setup
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.amg = adafruit_amg88xx.AMG88XX(self.i2c)

		# ROS2 publisher
		# self.publisher = self.create_publisher(Float32MultiArray, '/temperature_map', 10)
		self.publisher = self.create_publisher(String, '/heat_location', 10)
		self.timer = self.create_timer(1.0, self.read_publish_temperature)

		# Motor GPIO setup
		self.motor_pins = [22, 23, 25, 24] # BCM pins
		#self.motor_pins = [15, 16, 22, 18] # board pins
		for pin in self.motor_pins:
			GPIO.setup(pin, GPIO.OUT)

		# Enable pins for two motors
		self.enable_pins = [13, 19]
		for pin in self.enable_pins:
				GPIO.setup(pin, GPIO.OUT)
				GPIO.output(pin, True)

		self.enable_pwms = [
				GPIO.PWM(13, 50),  # Motor A
				GPIO.PWM(19, 50)   # Motor B
			]

		# Servo GPIO setup
		# GPIO.setmode(GPIO.BCM) # no need, alrdy set in imported library
		self.servo_pin = 5
		GPIO.setup(self.servo_pin, GPIO.OUT)
		self.servo_pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz
		self.servo_pwm.start(2.5)  # Neutral position

	def spin_start(self):
		GPIO.output(22, True)
		GPIO.output(23, False)
		GPIO.output(25, True)
		GPIO.output(24, False)
		time.sleep(1)
		GPIO.output(22, True)
		GPIO.output(23, True)
		GPIO.output(25, False)
		GPIO.output(24, False)
		# for pin in self.motor_pins:
		# 	GPIO.output(pin, False)
		for pin in self.enable_pwms:
			pin.ChangeDutyCycle(30) # (30/50)% of max speed

	def spin_stop(self):
		for pin in self.motor_pins:
			GPIO.output(pin, False)

	def activate_servo(self):
		# Move to 0°, then back to 180°
		self.servo_pwm.ChangeDutyCycle(10)
		time.sleep(1)
		self.servo_pwm.ChangeDutyCycle(2.5)
		time.sleep(1)
		# self.servo_pwm.ChangeDutyCycle(7.5)  # back to center

	def read_publish_temperature(self):
		# Flatten and publish temperature data
		temperature_data = Float32MultiArray()
		temperature_data.data = [temp for row in self.amg.pixels for temp in row]
		# self.publisher.publish(temperature_data)

		# Debug print
		for row in self.amg.pixels:
			print(["{0:.1f}".format(temp) for temp in row])
		print("\n")

		# Analyze columns
		# transposed = list(zip(*self.amg.pixels))
		# col_avgs = [sum(col)/len(col) for col in transposed]
		# max_avg_col_index = col_avgs.index(max(col_avgs))

		# print(f"Column with Highest Average Temperature: {max_avg_col_index}")
		
		print("Max element per column")
		max_element_per_col = []
		for idxcol in range(8):
			maxcurcol = -1
			for idxrow in range(8):
				maxcurcol = max(maxcurcol, self.amg.pixels[idxrow][idxcol])
			print(maxcurcol, end=' ')
			max_element_per_col.append(maxcurcol)
		print()

		# max_element = max([temp for row in self.amg.pixels for temp in row])

		if max(max_element_per_col[2:6]) > FRONT_HEAT_LAUNCH:
			print("Heat is detected in front! Activating servo.")
			self.publisher.publish(String(data='ok'))
			self.spin_start()
			time.sleep(1)
			self.activate_servo()	# first ball
			time.sleep(1)
			self.activate_servo()	# second ball
			time.sleep(1)
			self.activate_servo()	# third ball
			time.sleep(1)
			self.spin_stop()
		elif max(max_element_per_col[2:6]) > FRONT_HEAT_FORWARD:
			print("Heat is detected in front. Move closer please!")
			self.publisher.publish(String(data='forward'))
		elif max(max_element_per_col[:2]) > LEFT_RIGHT_HEAT_THRESHOLD:
			print("Heat is on the Right")
			self.publisher.publish(String(data='right'))
		elif max(max_element_per_col[6:]) > LEFT_RIGHT_HEAT_THRESHOLD:
			print("Heat is on the Left")
			self.publisher.publish(String(data='left'))
		else:
			print("No heat ahead. Continue Exploration.")
			self.publisher.publish(String(data='null'))

	def destroy_node(self):
		super().destroy_node()
		self.servo_pwm.stop()
		GPIO.cleanup()

def main(args=None):
	rclpy.init(args=args)
	node = AMG8833Node()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()

