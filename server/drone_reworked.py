#!/usr/bin/python

#########################################################################
#########################################################################
#  DRONE CODE - REWORKED
#########################################################################
#########################################################################

import os 													# clearia varten
import MPU6050												# External gyro-library
import thread
import time
import smbus
import math
import pigpio												# Externally downloaded PiGPIO library, IO control
import netifaces as ni										# Needed to get IP from host system
from socketIO_client import SocketIO, LoggingNamespace		# TCP socket library for communication
from subprocess import call									# For starting video feed
import curses


# VARIABLE CLASS DECLARATIONS ###########################################
zAccel_temp = 0.0
class var_timer:
	def __init__(self):
		# Thread timers
		self.time_thread_1 = 0.0
		self.time_thread_2 = 0.0
		self.time_thread_3 = 0.0
		# function timers
		self.thread1_last = 0.0
		self.throttle = 0
		self.lastThrottle = 0
		self.lastCalib = 0
		
class var_pid:
	def __init__(self):
		# Global
		self.SampleTime = 20.0
		self.outMin = -40
		self.outMax = 40
		self.outMin_z = -20
		self.outMax_z = 20
		self.outMin_thrust = 0
		self.outMax_thrust = 800
		self.Motor_FL = 0
		self.Motor_FR = 0
		self.Motor_RL = 0
		self.Motor_RR = 0
		self.Motor_FL_hover = 0
		self.Motor_FR_hover = 0
		self.Motor_RL_hover = 0
		self.Motor_RR_hover = 0
		# setup
		self.kp = 0
		self.ki = 0
		self.kd = 0
		# PID Thrust variables
		kp_thrust = 0
		ki_thrust = 0
		kd_thrust = 0
		# X
		self.ITerm_x = 0
		self.Output_x = 0
		self.lastInput_x = 0
		self.lastTime_x = 0
		self.xRot = 0
		self.error_x = 0
		# Y
		self.ITerm_y = 0
		self.Output_y = 0
		self.lastInput_y = 0
		self.lastTime_y = 0
		self.yRot = 0
		self.error_y = 0
		# Z
		self.ITerm_z = 0
		self.Output_z = 0
		self.lastInput_z = 0
		self.lastTime_z = 0
		self.zRot = 0
		self.error_z = 0
		
class var_demand:			# Drone angle demand
	def __init__(self):
		self.x_angle_demand = 0
		self.y_angle_demand = 0
		self.z_angle_demand = 0
		self.hover = 0
		
		#self.data = '00000000'
		self.left = '0'
		self.right = '0'
		self.forward = '0'
		self.backward = '0'
		self.cclockwise = '0'
		self.clockwise = '0'
		self.up = '0'
		self.down = '0'
		self.tune = '0'
		self.keymap = '000000000'

class var_gyro:
	def __init__(self):
		# setup
		self.mpu6050 = 0
		self.gyroscope = 0
		# calibration
		self.xRot_calib = 0.0
		self.yRot_calib = 0.0
		self.zAccel_calib = 0.0
		
		self.xRot = 0
		self.yRot = 0
		self.zAccel = 0
		self.accel_xout = [0] * 8
		self.accel_yout = [0] * 8
		self.accel_zout = [0] * 8
	
		self.accel_xout_medianlist = [0.0] * 3
		self.accel_yout_medianlist = [0.0] * 3
		self.accel_zout_medianlist = [0.0] * 3
		self.median_i = 0
		
		self.gyro_index = 0
		self.gyro_zout = 0

class var_motco:
	def __init__(self):
		# Motor assignment to GPIO ports
		self.MOTOR1 = 4								# Pin07 GPIO 04
		self.MOTOR2 = 17							# Pin11 GPIO 17
		self.MOTOR3 = 27							# Pin13	GPIO 27
		self.MOTOR4 = 22							# Pin15 GPIO 22
		# Main pulsewidth variables
		self.motorFL_pw = 1000
		self.motorFR_pw = 1000
		self.motorRR_pw = 1000
		self.motorRL_pw = 1000
		# Minimum and maximum pulsewidths
		self.MIN_PW = 1000
		self.MAX_PW = 1999
		
class var_response():								# Responses to server
	def __init__(self):
		self.gyro = ''
		self.throttle = 0
		

####// END OF CLASS DECLARATIONS //######################################
#########################################################################		


### PID FUNCTIONS #######################################################

def PID_X_COMPUTE():
	# How long since we last calculated
	now = time.clock() * 1000
	timeChange = now - pid.lastTime_x
	
	if timeChange >= pid.SampleTime:
		#Compute all the working error variables
		pid.error_x = demand.x_angle_demand - gyro.xRot		# error_x = haluttu kulma. xRot = gyron tieto.
			
		pid.ITerm_x += pid.ki * pid.error_x
		if pid.ITerm_x > pid.outMax:
			pid.ITerm_x = pid.outMax
		elif pid.ITerm_x < pid.outMin:
			pid.ITerm_x = pid.outMin		
		dInput = gyro.xRot - pid.lastInput_x
		#Compute PID output
		pid.Output_x = (pid.kp * pid.error_x) + pid.ITerm_x - (pid.kd * dInput)
		if pid.Output_x > pid.outMax:
			pid.Output_x = pid.outMax
		elif pid.Output_x < pid.outMin:
			pid.Output_x = pid.outMin
			
		# Output
		pid.Motor_FR = pid.Output_x
		pid.Motor_RL = -pid.Output_x

		# Remember some variables for next time
		pid.lastInput_x = gyro.xRot
		pid.lastTime_x = now
		
		# Give motorcontrol permission to execute
#		motorcontrol_flag_x = True

def PID_Y_COMPUTE():
	# How long since we last calculated
	now = time.clock() * 1000
	timeChange = now - pid.lastTime_y
	
	if timeChange >= pid.SampleTime:
		#Compute all the working error variables
		pid.error_y = demand.y_angle_demand - gyro.yRot
		pid.ITerm_y += pid.ki * pid.error_y
		if pid.ITerm_y > pid.outMax:
			pid.ITerm_y = pid.outMax
		elif pid.ITerm_y < pid.outMin:
			pid.ITerm_y = pid.outMin		
		dInput = gyro.yRot - pid.lastInput_y
		#Compute PID output
		pid.Output_y = pid.kp * pid.error_y + pid.ITerm_y - pid.kd * dInput
		if pid.Output_y > pid.outMax:
			pid.Output_y = pid.outMax
		elif pid.Output_y < pid.outMin:
			pid.Output_y = pid.outMin
		
		# output
		pid.Motor_FL = -pid.Output_y
		pid.Motor_RR = pid.Output_y

		# Remember some variables for next time
		pid.lastInput_y = gyro.yRot
		pid.lastTime_y = now

def PID_Z_COMPUTE():
	# How long since we last calculated
	now = time.clock() * 1000
	timeChange = now - pid.lastTime_z
	
	if timeChange >= pid.SampleTime:
		#Compute all the working error variables
		pid.error_z = demand.z_angle_demand - gyro.zAccel
			
		pid.ITerm_z += pid.ki_thrust * pid.error_z
		if pid.ITerm_z > pid.outMax_thrust:
			pid.ITerm_z = pid.outMax_thrust
		elif pid.ITerm_z < pid.outMin_thrust:
			pid.ITerm_z = pid.outMin_thrust		
		dInput = gyro.zAccel - pid.lastInput_z
		#Compute PID output
		pid.Output_z = pid.kp_thrust * pid.error_z + pid.ITerm_z - pid.kd_thrust * dInput
		if pid.Output_z > pid.outMax_z:
			pid.Output_z = pid.outMax_z
		elif pid.Output_z < pid.outMin_z:
			pid.Output_z = pid.outMin_z

		pid.Motor_FL_hover = -pid.Output_z
		pid.Motor_RR_hover = -pid.Output_z
		pid.Motor_FR_hover = pid.Output_z
		pid.Motor_RL_hover = pid.Output_z
	
		# Remember some variables for next time
		pid.lastInput_z = gyro.zAccel
		pid.lastTime_z = now

def SetTunings(Kp, Ki, Kd):									# Change PID-controller behaviour
	SampleTimeInSec = pid.SampleTime / 1000
	pid.kp = Kp
	pid.ki = Ki * SampleTimeInSec
	pid.kd = Kd / SampleTimeInSec
	

def SetTunings_thrust(Kp, Ki, Kd):							# Change PID behaviour when turning
	SampleTimeInSec = pid.SampleTime / 1000
	pid.kp_thrust = Kp
	pid.ki_thrust = Ki * SampleTimeInSec
	pid.kd_thrust = Kd / SampleTimeInSec

def SetSampleTime(NewSampleTime):
	if NewSampleTime > 0:
		self.ratio = NewSampleTime / pid.SampleTime
		pid.ki *= self.ratio
		pid.kd /= self.ratio
		pid.ki_thrust *= self.ratio
		pid.kd_thrust /= self.ratio
		pid.SampleTime = NewSampleTime

###// END OF PID FUNCTIONS //###########################################
########################################################################


# MPU-6050 GYROSCOPE ###################################################

def init_gyroscope(samplerate, gresolution):				# Wake the MPU-6050 up as it starts in sleep mode
	ready = False
	while ready == False:
		try:
			gyro.mpu6050 = MPU6050.MPU6050()				# var init. KEEP HERE, MIGHT FAIL OTHERWISE
			gyro.mpu6050.setup()
			gyro.mpu6050.setSampleRate(samplerate)			# 100 probably ok
			gyro.mpu6050.setGResolution(gresolution)		# 2g for lessening vibration
			ready = True									# Let us know if setup successful
		except:
			pass											# If setup fails, try again (DO NOTHING)
	time.sleep(0.01)

def accelerometer_calibrate():								# Calibrate MPU6050 at startup
	ready = False
	i = 0
	x = 5
	accel_xout = [0] * 20
	accel_yout = [0] * 20
	accel_zout = [0] * 20
	
	accel_xout_temp = 0
	accel_yout_temp = 0
	accel_zout_temp = 0
	
	gyro.xRot_calib = 0.0
	gyro.yRot_calib = 0.0
	gyro.zAccel_calib = 0.0
	
	while i < 20:
		try:
			while (gyro.mpu6050.readStatus() & 1)==0 :
				time.sleep(0.001)
			ready = True
		except:
			pass

		while ready == True:
			try:
				gyroscope_calibration = gyro.mpu6050.readData()
				accel_xout[i] = gyroscope_calibration.Gx * 100
				accel_yout[i] = gyroscope_calibration.Gy * 100
				accel_zout[i] = gyroscope_calibration.Gyroz / 10
				i += 1
				ready = False
				time.sleep(0.01)
			except:
				pass
		
	accel_xout.sort()
	accel_yout.sort()
	accel_zout.sort()

	while x < 15:
		gyro.xRot_calib += accel_xout[x] / 10
		gyro.yRot_calib += accel_yout[x] / 10
		gyro.zAccel_calib += accel_zout[x] / 10
		x += 1
	save_calibration()

def save_calibration():
	savefile = open('/home/terminal/calib_data', 'w')
	savefile.write(str(gyro.xRot_calib) + ":" + str(gyro.yRot_calib) + ":" + str(gyro.zAccel_calib))
	savefile.close()
	load_calibration()

def load_calibration():
	savefile = open('/home/terminal/calib_data', 'r')
	data = savefile.readlines()
	for line in data:
		calib_data = line.split(':')
	gyro.xRot_calib = float(calib_data[0])
	gyro.yRot_calib = float(calib_data[1])
	gyro.zAccel_calib = float(calib_data[2])

def convert_accelerometer_data():	
	gyro.accel_xout[gyro.gyro_index] = gyro.gyroscope.Gx * 100
	gyro.accel_yout[gyro.gyro_index] = gyro.gyroscope.Gy * 100
	gyro.accel_zout[gyro.gyro_index] = gyro.gyroscope.Gyroz / 10
	gyro.gyro_index = gyro.gyro_index + 1
	
	if gyro.gyro_index >= 8:
		gyro.gyro_index = 0
	
	accel_xout_sorted = gyro.accel_xout											# SORT DATA TO GET MIDDLE VALUE
	accel_xout_sorted.sort()
	xRot_temp = (accel_xout_sorted[2] + accel_xout_sorted[3] + accel_xout_sorted[4] + accel_xout_sorted[5]) / 4 # / 16384.0
	accel_yout_sorted = gyro.accel_yout
	accel_yout_sorted.sort()
	yRot_temp = (accel_yout_sorted[2] + accel_yout_sorted[3] + accel_yout_sorted[4] + accel_yout_sorted[5]) / 4 # / 16384.0
	accel_zout_sorted = gyro.accel_zout
	accel_zout_sorted.sort()
	global zAccel_temp
	zAccel_temp = (accel_zout_sorted[2] + accel_zout_sorted[3]) + accel_zout_sorted[4] + accel_zout_sorted[5] / 4 # / 16384.0
			
	if gyro.median_i >= 3:
		gyro.median_i = 0
				
	gyro.accel_xout_medianlist[gyro.median_i] = xRot_temp - float(gyro.xRot_calib)
	gyro.accel_yout_medianlist[gyro.median_i] = yRot_temp - float(gyro.yRot_calib)
	gyro.accel_zout_medianlist[gyro.median_i] = zAccel_temp - float(gyro.zAccel_calib)
		
	gyro.xRot =   round((gyro.accel_xout_medianlist[0] + gyro.accel_xout_medianlist[1] + gyro.accel_xout_medianlist[2]) / 3)
	gyro.yRot =   round((gyro.accel_yout_medianlist[0] + gyro.accel_yout_medianlist[1] + gyro.accel_yout_medianlist[2]) / 2)
	gyro.zAccel = round((gyro.accel_zout_medianlist[0] + gyro.accel_zout_medianlist[1] + gyro.accel_zout_medianlist[2]) / 3)
			
	gyro.median_i += 1
			
	# Gyro response - send to server
	response.gyro = gyro.xRot , gyro.yRot , gyro.zAccel

########################################################################
########################################################################


# CHECK ANGLE DEMAND ###################################################

def check_demand():
	x_angle_tmp = 0
	y_angle_tmp = 0
	desired = 15
	
	time_start = time.clock() * 1000
	timeChange = time_start - timer.lastThrottle
	if timeChange >= 20:
		if demand.up == '1' and demand.down != '1':							# Up
			demand.hover += 5
			timer.lastThrottle = time_start

	if demand.up != '1' and demand.down == '1':								# Down
		if demand.hover >= -200:
			demand.hover += -5
	
	if demand.tune == '1':
		tune_calibration()
	else:
		if demand.forward == '1':											# Forward
			y_angle_tmp += desired
			x_angle_tmp += -desired
		if demand.backward == '1':											# Backward
			y_angle_tmp += -desired
			x_angle_tmp += desired
		if demand.forward == '0' and demand.backward == '0':
			y_angle_tmp = 0
			x_angle_tmp = 0
		
		if demand.left == '1' and demand.right == '0':						# Left
			if x_angle_tmp == -desired or x_angle_tmp == desired:					# If Forward/Backward set
				y_angle_tmp += -desired
			else:															# If not set
				x_angle_tmp += -desired
				y_angle_tmp += -desired
		if demand.right == '1' and demand.left == '0':						# Right
			if y_angle_tmp == -desired or y_angle_tmp == desired:					# If Forward/Backward set
				x_angle_tmp += desired
			else:															# If not set
				x_angle_tmp += desired
				y_angle_tmp += desired
	
		if demand.cclockwise == '1' and demand.clockwise == '0' :
			demand.z_angle_demand = 8
		elif demand.clockwise == '1' and demand.cclockwise == '0' :
			demand.z_angle_demand = -8
		elif demand.cclockwise == '0' or demand.clockwise == '0' :
			demand.z_angle_demand = 0
			
		demand.x_angle_demand = x_angle_tmp
		demand.y_angle_demand = y_angle_tmp
		
def tune_calibration():
	time_start = time.clock() * 1000
	timeChange = time_start - timer.lastCalib
	
	if timeChange >= 20:
		if demand.forward == '1' and demand.backward == '0':
			gyro.xRot_calib += 0.1
			gyro.yRot_calib += -0.1
		if demand.backward == '1' and demand.forward == '0':
			gyro.xRot_calib += -0.1
			gyro.yRot_calib += 0.1
		if demand.left == '1' and demand.right == '0':
			gyro.xRot_calib += 0.1
			gyro.yRot_calib += 0.1
		if demand.right == '1' and demand.left == '0':
			gyro.xRot_calib += -0.1
			gyro.yRot_calib += -0.1
		timer.lastCalib = time_start
	
########################################################################
########################################################################

	
### MOTOR FUNCTIONS ####################################################

def calibrate_ESC():													# Calibrate ESC on boot. SHOULD ONLY BE MADE ONCE
	drone.set_servo_pulsewidth(motco.MOTOR1, 2000)
	drone.set_servo_pulsewidth(motco.MOTOR2, 2000)
	drone.set_servo_pulsewidth(motco.MOTOR3, 2000)
	drone.set_servo_pulsewidth(motco.MOTOR4, 2000)
	time.sleep(3)
	drone.set_servo_pulsewidth(motco.MOTOR1, 1000)
	drone.set_servo_pulsewidth(motco.MOTOR2, 1000)
	drone.set_servo_pulsewidth(motco.MOTOR3, 1000)
	drone.set_servo_pulsewidth(motco.MOTOR4, 1000)
	time.sleep(2)
	drone.set_servo_pulsewidth(motco.MOTOR1, 1045)
	drone.set_servo_pulsewidth(motco.MOTOR2, 1045)
	drone.set_servo_pulsewidth(motco.MOTOR3, 1045)
	drone.set_servo_pulsewidth(motco.MOTOR4, 1045)
	time.sleep(1)

def motorcontrol(FL, FR, RR, RL):	
	motco.motorFL_pw = 1000 + demand.hover + FL + pid.Motor_FL_hover
	motco.motorFR_pw = 1000 + demand.hover + FR + pid.Motor_FR_hover + 4
	motco.motorRR_pw = 1000 + demand.hover + RR + pid.Motor_RR_hover + 4
	motco.motorRL_pw = 1000 + demand.hover + RL + pid.Motor_RL_hover

# MOTOR 1 - FRONT lEFT	
	if motco.motorFL_pw < motco.MIN_PW:
		motco.motorFL_pw = motco.MIN_PW
	elif motco.motorFL_pw > motco.MAX_PW:
		motco.motorFL_pw = motco.MAX_PW
	drone.set_servo_pulsewidth(motco.MOTOR1, motco.motorFL_pw)

# MOTOR 2 - FRONT RIGHT
	if motco.motorFR_pw < motco.MIN_PW:
		motco.motorFR_pw = motco.MIN_PW
	elif motco.motorFR_pw > motco.MAX_PW:
		motco.motorFR_pw = motco.MAX_PW
	drone.set_servo_pulsewidth(motco.MOTOR2, motco.motorFR_pw)

# MOTOR 3 - REAR RIGHT
	if motco.motorRR_pw < motco.MIN_PW:
		motco.motorRR_pw = motco.MIN_PW
	elif motco.motorRR_pw > motco.MAX_PW:
		motco.motorRR_pw = motco.MAX_PW
	drone.set_servo_pulsewidth(motco.MOTOR3, motco.motorRR_pw)

#MOTOR 4 - REAR LEFT
	if motco.motorRL_pw < motco.MIN_PW:
		motco.motorRL_pw = motco.MIN_PW
	elif motco.motorRL_pw > motco.MAX_PW:
		motco.motorRL_pw = motco.MAX_PW
	drone.set_servo_pulsewidth(motco.MOTOR4, motco.motorRL_pw)

#########################################################################
#########################################################################


### TCP SOCKET CONNECTION ###############################################

def server_response(*args):
	data = args
	try:
		demand.keymap = data[0].encode('ascii')
	except AttributeError:
		demand.keymap = '000000000'
	server_keymapper(demand.keymap)							# Calls 'keymapper'-function to break data into variables
	
def server_keymapper(keymap):
	try:
		demand.left = keymap[0]
		demand.right = keymap[1]
		demand.forward = keymap[2]
		demand.backward = keymap[3]
		demand.cclockwise = keymap[4]
		demand.clockwise = keymap[5]
		demand.up = keymap[6]
		demand.down = keymap[7]
		demand.tune = keymap[8]
	except IndexError:
		demand.left = '0'
		demand.right = '0'
		demand.forward = '0'
		demand.backward = '0'
		demand.cclockwise = '0'
		demand.clockwise = '0'
		demand.up = '0'
		demand.down = '0'
		demand.tune = '0'

def server_adjustpid(*args):
	temp = '000'
	data = args

	try:
		temp = data[0].encode('ascii')
		#temp_i = data[1].encode('ascii')
		kp = float(temp[0])
		ki = float(temp[1])
		kd = float(temp[2])
		SetTunings(kp, ki, kd)
		#		socketIO.emit('DRONErequest', P + I + D)
	except:
		print "ASD"
		time.sleep(1)
		

def server_calibration(*args):
	data = str(args)
	accelerometer_calibrate()								# Calibrate gyroscope 
		
def server_calibration_tuning(*args):
	data = str(args)
	try:
		temp = data[0].encode('ascii')
		for index in temp:
			if index == 1:
				index = -0.1
			elif index == 2:
				index = 0.1
		gyro.xRot_calib += float(temp[0])
		gyro.yRot_calib += float(temp[1])
	except:
		pass
		
#########################################################################
#########################################################################


### THREADS #############################################################

### THREAD 1 - COMMUNICATION ###############################			
# // Sends gyro information, receives control commands, assigns commands to global variables	
def thread1():
	while True:
		time_start = time.clock() * 1000
		timeChange = time_start - timer.thread1_last

		socketIO.wait()										# Continuously listen for server messages
		#socketIO.on('IOanswer', server_response)
		#socketIO.on('server_pid', server_adjustpid)
		#socketIO.on('calibrate', server_calibration)
		
		if timeChange >= 100:
			socketIO.emit('DRONErequest', str(response.gyro)) #response.gyro
			timer.thread1_last = time.clock() * 1000
		
		time_end = time.clock() * 1000
		timer.time_thread_1 = time_end - time_start

### THREAD 2 - MOTOR CONTROL ###############################	
# // Motor control thread, Reads the global control variables, Changes the wanted "0" angle variable	
def thread2():
	while True:
		time_start = time.clock() * 1000
		check_demand()													# Check directions where user wants to go
		PID_X_COMPUTE()													# Compute x output
		PID_Y_COMPUTE()													# Compute y output
		PID_Z_COMPUTE()													# Compute z output
		motorcontrol(pid.Motor_FL, pid.Motor_FR, pid.Motor_RR, pid.Motor_RL)	# Commands to motors!
		time_end = time.clock() * 1000
		timer.time_thread_2 = time_end - time_start
		time.sleep(0.005)
		
### THREAD 3 - GYROSCOPE ###################################
# // Gyroscope thread, read gyroscope data
def thread3():
	ready = False
	while True:
		time_start = time.clock() * 1000
		try:
			while (gyro.mpu6050.readStatus() & 1)==0 :
				time.sleep(0.001)
			ready = True
		except:
			pass
		if ready == True:
			try:
				gyro.gyroscope = gyro.mpu6050.readData()
				convert_accelerometer_data()
				ready = False
			except:
				pass
		time_end = time.clock() * 1000
		timer.time_thread_3 = time_end - time_start
		time.sleep(0.0001)			# Sleep for a while to prevent thread crash..
	
#########################################################################
#########################################################################


# MAIN INITIALIZATION ###################################################
drone = pigpio.pi()
timer = var_timer()
response = var_response()
pid = var_pid()
demand = var_demand()
gyro = var_gyro()
motco = var_motco()
init_gyroscope(50, 2)										# Initialize gyroscope. Input: SampleRate, GResolution

ni.ifaddresses('eth0')
received = 0
while received == 0:
	try:
		drone_ip = ni.ifaddresses('ppp0')[2][0]['addr']		# Get ip from system ifaddresses
		received = 1
	except:
		pass

print drone_ip												# should print IP on boot

socketIO = SocketIO('http://avela.ddns.net', 3000)			# Initializes the connection to avela.ddns.net, port 3000
socketIO.emit('DRONEboot', drone_ip)						# Send IP to server
socketIO.on('server_pid', server_adjustpid)
socketIO.on('calibrate', server_calibration)
socketIO.on('IOanswer', server_response)

socketIO.wait(0.25)

accelerometer_calibrate()									# Calibrate gyroscope 
load_calibration()
#calibrate_ESC()
SetTunings(1.5, 2.5, 0.1)										# X Y PID tunings
SetTunings_thrust(4, 1, 0.01)								# Thrust PID tunings here!															

try:
	thread.start_new_thread( thread1, () )
	thread.start_new_thread( thread2, () )
	thread.start_new_thread( thread3, () )
except:
	print "Error: unable to start thread"

time.sleep(0.01)
terminal = True
while terminal == True:										# Below starts CLI for debugging purposes via terminal
	try:
		screen = curses.initscr()							# Init CURSES
		screen.border(0)								# Give cool borders
		screen.addstr(2, 5, "DRONE COMMAND CENTER")		# Print stuff..
		screen.addstr(8, 5, "PID variables #####")
		screen.addstr(12, 5, "Angular error #####")
		screen.addstr(17, 5, "Gyroscope data ####")
		while True:
			if pid.kp != 0:
				screen.addstr(5, 5, drone_ip)
				screen.addstr(6, 5, "READY")
			elif pid.kp != 1:
				screen.addstr(6, 5, "booting..")
			
			screen.addstr(9, 5, "X:   " + str(pid.Motor_FR))
			screen.addstr(10, 5, "Y:   " + str(pid.Motor_FL))
			screen.addstr(11, 5, "Z:   " + str(demand.hover))
			
			screen.addstr(13, 5, "X:   " + str(pid.error_x))
			screen.addstr(14, 5, "Y:   " + str(pid.error_y))
			screen.addstr(15, 5, "Z:   " + str(pid.error_z))
			
			screen.addstr(18, 5, "X:   " + str(gyro.xRot))
			screen.addstr(19, 5, "Y:   " + str(gyro.yRot))
			screen.addstr(20, 5, "Z:   " + str(gyro.zAccel))
			
			screen.addstr(21, 5, "PID: " + str(pid.kp))
			screen.addstr(22, 5, "PID: " + str(pid.ki))
			screen.addstr(23, 5, "PID: " + str(pid.kd))
			screen.refresh()
		#	screen.getch() # pause until keyhit
			
		curses.endwin() # shuts down curse


#			print ""
#			print "Temperature: ",gyroscope.Temperature
	except KeyboardInterrupt:
		# quit
		motorcontrol(1000,1000,1000,1000)
		print "Quitting.."
		time.sleep(1)

		exit()
while terminal == False:
	time.sleep(0.01)
#	os.system('clear')
#	print gyro.xRot_calib
#	print gyro.yRot_calib
#	print "Thread completion time (ms):"
#	print "Thread 1 - Communication: -", timer.time_thread_1
#	print "Thread 2 - Motor control: -", timer.time_thread_2
#	print "Thread 3 - Gyroscope:     -", timer.time_thread_3
#	print pid.Output_z
#	print demand.hover
