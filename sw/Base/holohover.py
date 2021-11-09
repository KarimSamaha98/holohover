from aioudp import * 
import asyncio
from datetime import datetime
import threading
import click
import csv
import time
import numpy as np
import hovercraft_pb2 as hovercraft
from math import pi

import pygame


class Periodic_Delay:
	def __init__(self, period, show_period=None, name=None):
		self.period = period
		self.prevtime = datetime.now()
		self.prevshow = datetime.now()
		self.show_period = show_period
		self.name = name
		self.count = 0

	async def sleep(self):
		now = datetime.now()
		delta = now - self.prevtime
		self.prevtime = now
		await asyncio.sleep((self.period - delta.total_seconds())/1000.0)

		self.count = self.count + 1
		if self.show_period is not None:
			delta = now - self.prevshow
			if delta.total_seconds() > self.show_period:
				frequency = (float)(self.count) / (float)(delta.total_seconds())
				print(f"Frequency of {self.name} is {frequency:.1f} (target {1/self.period*1000.0:.1f})\r")
				self.count = 0
				self.prevshow = now


async def setup():
	# Create a linkState UDP enpoint
	linkState = await open_local_endpoint('255.255.255.255', 3333)

	# The linkState endpoint receives the datagram, along with the address
	print("=> Waiting for first message from hover")
	data, address = await linkState.receive()

	# Create a UDP enpoint from the sending address
	print("=> Received message - beginning")
	linkCommand = await open_remote_endpoint(address[0], address[1])

	return linkState, linkCommand



class Controller(object):
	def __init__(self, linkCommand):
		self.linkCommand = linkCommand
		self.motors = hovercraft.Motors()

		self._armed = False

		self._acc_x_ref = 0 #m/s**2
		self._acc_y_ref = 0 #m/s**2
		self._yawrate_ref = 0 #rad/s**2

		self.Kp_x = 0
		self.Kd_x = 0
		self.Ki_x = 0

		self.Kp_y = 0
		self.Kd_y = 0
		self.Ki_y = 0

		self.Kp_yaw = 0
		self.Kd_yaw = 0
		self.Ki_yaw = 0

		


	def stop(self):
		self.armed = False

	@property
	def yawrate_ref(self):
		return self._yawrate_ref

	@yawrate_ref.setter
	def yawrate_ref(self, val):
		# Incoming signal is between min and max
		min_yawrate = -2*pi
		max_yawrate = +2*pi
		val = min(max(val,min_yawrate),max_yawrate)
		# Set the yawrate target
		self._yawrate_ref = val

	@property
	def acc_x_ref(self):
		return self._acc_x_ref

	@acc_x_ref.setter
	def acc_x_ref(self, val):
		# Incoming signal is between min and max
		min_acc_x = -0.2 #[m/s**2]
		max_acc_x = 0.2 #[m/s**2]
		val = min(max(val,min_acc_x),max_acc_x)
		# Set the yawrate target
		self._acc_x_ref = val

	@property
	def acc_y_ref(self):
		return self._acc_y_ref

	@acc_y_ref.setter
	def acc_y_ref(self, val):
		# Incoming signal is between min and max
		min_acc_y = -2*pi
		max_acc_y = +2*pi
		val = min(max(val,min_acc_y),max_acc_y)
		# Set the yawrate target
		self._acc_y_ref = val

	@property
	def armed(self):
		return self._armed
	
	@armed.setter
	def armed(self, val):
		if val == True:
			self._armed = True
		else:
			self._armed = False
			self._off()
			self._send_motors()

	async def closed_loop_run(self, estimator, period):
		periodic = Periodic_Delay(period)

		#Low level control loop
		while True:
			# Sets the motor values
			self.control_loop(estimator.state)

			if self.armed is False:
				self._off()
			self._send_motors()

			await periodic.sleep()

	async def no_loop_run(self, period):
		# Sends motor commands periodically

		periodic = Periodic_Delay(period)

		#Low level control loop
		while True:

			if self.armed is False:
				self._off()
			self._send_motors()

			await periodic.sleep()

	async def open_loop_run(self, period):
		# Sends motor commands based on acceleration references in open loop

		periodic = Periodic_Delay(period)

		# Low level control loop
		while True:
			# Sets the motor values
			self.open_loop()

			if self.armed is False:
				self._off()
			self._send_motors()

			await periodic.sleep()


	def control_loop(self, state):

		# Compute control action
		# Given the state that is estimated by the flight controller


		# Modify the motor values
		# self.motors.motor_A_1  = None
		# self.motors.motor_A_2  = None
		# self.motors.motor_B_1  = None
		# self.motors.motor_B_2  = None
		# self.motors.motor_C_1  = None
		# self.motors.motor_C_2  = None
		return

	def open_loop(self):

		# Compute control action without feedback
		
		
		# Modify the motor values
		# self.motors.motor_A_1  = None
		# self.motors.motor_A_2  = None
		# self.motors.motor_B_1  = None
		# self.motors.motor_B_2  = None
		# self.motors.motor_C_1  = None
		# self.motors.motor_C_2  = None
		return

	def _send_motors(self):
		self.linkCommand.send(self.motors.SerializeToString())

	def _off(self):
		# Give idle values to the motors
		self.motors.motor_A_1  = 1000
		self.motors.motor_A_2  = 1000
		self.motors.motor_B_1  = 1000
		self.motors.motor_B_2  = 1000
		self.motors.motor_C_1  = 1000
		self.motors.motor_C_2  = 1000

	def __str__(self):
		s = '[x] ' if self.armed else '[ ] '
		s = s + "Thrust %.1f Lift %.1f " % (self.thrust, self.lift)
		s = s + "YawRef %.1f " % self.yawrate_ref
		return s
		
	def set_motors(self, commands):
		# Sets motors to a given rpm.

		#Parameters:
		#    commands: (6,1) list of float numbers ranging from 1000 to 2000. Equivalent to 0 to full speed rotations

		self.motors.motor_A_1 = commands[0]
		self.motors.motor_A_2 = commands[1]
		self.motors.motor_B_1 = commands[2]
		self.motors.motor_B_2 = commands[3]
		self.motors.motor_C_1 = commands[4]
		self.motors.motor_C_2 = commands[5]

	def set_acc(self, commands):
		# Sets acceleration values to a given reference

		#Parameters:
		#    commands: (3,1) list of float numbers corresponding to acc_x [m/s**2], acc_y[m/s**2], yaw_rate[rad/s**2]

		self.acc_x_ref = commands[0]
		self.acc_y_ref = commands[1]
		self.yawrate_ref = commands[2]






class Estimator(object):
	def __init__(self):
		self.state = hovercraft.HoverCraftState()
		self._start_time = datetime.now()

	async def run(self, linkState):
		prevtime = datetime.now()
		count = 0
		while True:
			count = count + 1
			if count > 1000:
				delta = datetime.now() - prevtime
				frequency = 1.0 / delta.total_seconds() * 1000.0
				print ("Estimator loop frequency = %.2f\r" % frequency)
				count = 0
				prevtime = datetime.now()

			data, address = await linkState.receive() # Wait until you receive state from ESP
			self.state.ParseFromString(data)




class Logger(object):
	def __init__(self, filename='hoverlog.csv'):

		self._f = open('hoverlog.csv', 'w')
		self._logger = csv.writer(self._f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

		self._logger.writerow([
			'Time', 
			'roll', 'pitch', 'yaw', 
			'gyro_x', 'gyro_y', 'gyro_z', 
			'acc_x', 'acc_y', 'acc_z', 
			'mag_x', 'mag_y', 'mag_z',
			'armed', 'thrust', 'lift', 'yawrate_ref',
			'motor_A_1', 'motor_A_2', 'motor_B_1', 'motor_B_2', 'motor_C_1', 'motor_C_2'])

		self._start_time = datetime.now()

	async def run(self, period, estimator, controller):
		periodic = Periodic_Delay(period)
		while True:
			time = datetime.now() - self._start_time

			state = estimator.state
			motors = controller.motors

			if controller.armed:
				self._logger.writerow([
					time.total_seconds(), 
					state.roll,   state.pitch,  state.yaw, 
					state.gyro_x, state.gyro_y, state.gyro_z, 
					state.acc_x,  state.acc_y,  state.acc_z, 
					state.mag_x,  state.mag_y,  state.mag_z,
					controller.armed, controller.thrust, controller.lift, controller.yawrate_ref,
					motors.motor_A_1, motors.motor_A_2, motors.motor_B_1, motors.motor_B_2, motors.motor_C_1, motors.motor_C_2])

			await periodic.sleep()


	async def display(self, period, estimator, controller):
		periodic = Periodic_Delay(50)
		while True:
			state = estimator.state
			print(f"[{'x' if controller.armed else ' '}] gyro/ref {state.gyro_z/1000:6.1f}/{controller.yawrate_ref:6.1f}", end='')
			await periodic.sleep()

	def __del__(self):
		close(self._f)


class SystemThread (threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

		loop = asyncio.new_event_loop()
		asyncio.set_event_loop(loop)

		#Run the setup function until both sender and receiver are set up
		linkState, linkCommand = loop.run_until_complete(setup())

		controller = Controller(linkCommand)
		estimator = Estimator()
		logger = Logger()

		control_period = 10 # ms
		logging_period = 10 # ms
		display_period = 5000 # ms

		

		tasks = []
		tasks.append(loop.create_task(controller.no_loop_run(control_period)))
		tasks.append(loop.create_task(estimator.run(linkState)))
		#tasks.append(loop.create_task(logger.run(logging_period, estimator, controller)))
		#tasks.append(loop.create_task(logger.display(display_period, estimator, controller)))

		self.loop = loop
		self.controller = controller
		self.estimator = estimator
		self.logger = logger
		self.tasks = tasks


		# userInput = UserInput(controller)
		# userInput.start()




	def run(self):
		# self.loop = asyncio.get_event_loop()
		self.loop.run_until_complete(asyncio.gather(*self.tasks))


# Thread for managing the hovercraft
systemThread = SystemThread()
controller = systemThread.controller
systemThread.run()

systemThread.controller.armed = True
print('System Armed')
time.sleep(5)

systemThread.controller.set_motors([1500,1500,1500,1500,1500,1500])

#while True:
	#Get reference commands from keyboard
	# key = click.getchar(False)
	# if key == 'a':
	# 	controller.armed = False if controller.armed else True
	# if key == 'x':
	# 	controller._yawrate_ref = controller._yawrate_ref + 0.1
	# if key == 'z':
	# 	controller._yawrate_ref = controller._yawrate_ref - 0.1

	# if key == u'\x1b[A': # Up 
	# 	controller._acc_x_ref = controller._acc_x_ref + 0.1
	# if key == u'\x1b[B': # Down
	# 	controller._acc_x_ref = controller._acc_x_ref - 0.1

	# if key == u'\x1b[C': # Right			
	# 	controller.yawrate_ref = controller._acc_y_ref + 0.1
	# if key == u'\x1b[D': # Left
	# 	controller.yawrate_ref = controller._acc_y_ref - 0.1
	# if key == ' ': # Space
	# 	controller.yawrate_ref = 0
	#systemThread.controller.set_motors([1500,1500,1500,1500,1500,1500])
		



