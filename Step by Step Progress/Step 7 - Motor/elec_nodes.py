import time
import busio
import board
import adafruit_amg88xx
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO

class AMG8833Node(Node):
	def __init__(self):
		super().__init__('amg8833_node')

		# Thermal sensor setup
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.amg = adafruit_amg88xx.AMG88XX(self.i2c)

		# ROS2 publisher
		self.publisher = self.create_publisher(Float32MultiArray, '/temperature_map', 10)
		self.timer = self.create_timer(1.0, self.read_publish_temperature)

		# Motor GPIO setup
		self.motor_pins = [22, 23, 25, 24] # BCM pins
		#self.motor_pins = [15, 16, 22, 18] # board pins
		for pin in self.motor_pins:
			GPIO.setup(pin, GPIO.OUT)

		# Servo GPIO setup
		# GPIO.setmode(GPIO.BCM) # no need, alrdy set in imported library
		self.servo_pin = 5
		GPIO.setup(self.servo_pin, GPIO.OUT)
		self.servo_pwm = GPIO.PWM(self.servo_pin, 50)  # 50Hz
		self.servo_pwm.start(2.5)  # Neutral position

	def spin_start(self):
		GPIO.output(22, True)
		GPIO.output(23, True)
		GPIO.output(25, False)
		GPIO.output(24, False)
		# self.spin_stop()

	def spin_stop(self):
		for pin in self.motor_pins:
			GPIO.output(pin, False)

	def activate_servo(self):
		# Move to 0°, then back to 180°
		self.servo_pwm.ChangeDutyCycle(12.5)
		time.sleep(1)
		self.servo_pwm.ChangeDutyCycle(2.5)
		time.sleep(1)
		# self.servo_pwm.ChangeDutyCycle(7.5)  # back to center

	def read_publish_temperature(self):
		# Flatten and publish temperature data
		temperature_data = Float32MultiArray()
		temperature_data.data = [temp for row in self.amg.pixels for temp in row]
		self.publisher.publish(temperature_data)

		# Debug print
		for row in self.amg.pixels:
			print(["{0:.1f}".format(temp) for temp in row])
		print("\n")

		# Analyze columns
		transposed = list(zip(*self.amg.pixels))
		col_avgs = [sum(col)/len(col) for col in transposed]
		max_avg_col_index = col_avgs.index(max(col_avgs))

		print(f"Column with Highest Average Temperature: {max_avg_col_index}")

		max_element = max([temp for row in self.amg.pixels for temp in row])

		if (max_element>30):
			if max_avg_col_index in [2, 3, 4, 5]:
				print(" Heat detected ahead! Moving forward and activating servo.")
				self.spin_start()
				time.sleep(1)
				self.activate_servo()	# first ball
				time.sleep(1)
				self.activate_servo()	# second ball
				time.sleep(1)
				self.activate_servo()	# third ball
				time.sleep(1)
				self.spin_stop()
			elif max_avg_col_index in [0, 1]:
				print("Heat is on the Right")
			elif max_avg_col_index in [6, 7]:
				print("Heat is on the Left")
		else:
				print("No heat ahead. Continue Exploration.")

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

