#!/usr/bin/python

#########################################################################
#########################################################################
#  DRONE CODE - REWORKED
#########################################################################
#########################################################################

import sys
import os 													# clearia varten
import thread
import time
import smbus
import math
import pigpio												# Externally downloaded PiGPIO library, IO control
import netifaces as ni										# To get IP from host system
from socketIO_client import SocketIO, LoggingNamespace		# TCP socket library for communication
from subprocess import call									# For starting video feed
from MPU6050_4 import MPU6050
import curses

# GLOBAL DECLARATIONS ###################################################
enable_debug_output = True
enable_logging = True
i2c_bus = 1
device_address = 0x68

# VARIABLE CLASS DECLARATIONS ###########################################
class var_timer:			# General thread timing and timers in functions
	def __init__(self):
		# Thread timers
		self.time_thread_1 = 0
		self.time_thread_2 = 0
		self.time_thread_3 = 0
		# function timers
		self.thread1_last_pidvalues = 0.0
		self.thread1_last = 0.0
		self.throttle = 0
		self.lastThrottle = 0
		self.lastCalib = 0
		self.acc_dt_start = 0.0
		# counters
		self.COMMcount_start = 0
		self.COMMcountpersec = 0
		self.samplecount_start = 0
		self.samplecountpersec = 0
		self.PIDcount_start = 0
		self.PIDcount = 0
		self.PIDcountpersec = 0
		self.dtC = 0
		# Failsafe
		self.failsafe = 0
		self.failsafe_triggered = False
		self.failsafe_throttle = 0
		
class var_pid:						# For PID controllers
	def __init__(self):
		# General PID adjustments
		self.SampleTime = 2
		self.ITerm_max = 100
		self.ITerm_min = -100
		# setup
		self.kp = 0
		self.ki = 0
		self.kd = 0
		self.kp_yaw = 0
		self.ki_yaw = 0
		self.kd_yaw = 0
		# X
		self.P_value_x = 0
		self.I_value_x = 0
		self.D_value_x = 0
		self.ITerm_x = 0
		self.Output_x = 0
		self.lastTime_x = 0
		self.error_x = 0
		self.error_x_last = 0
		# Y
		self.P_value_y = 0
		self.I_value_y = 0
		self.D_value_y = 0
		self.ITerm_y = 0
		self.Output_y = 0
		self.lastTime_y = 0
		self.error_y = 0
		self.error_y_last = 0
		# Z
		self.P_value_z = 0
		self.I_value_z = 0
		self.D_value_z = 0
		self.ITerm_z = 0
		self.Output_z = 0
		self.lastTime_z = 0
		self.zRot = 0
		self.error_z = 0
		self.error_z_last = 0
		# Final thrust outputs
		self.Motor_FL = 0
		self.Motor_FR = 0
		self.Motor_RL = 0
		self.Motor_RR = 0
		self.Motor_FL_yaw = 0
		self.Motor_FR_yaw = 0
		self.Motor_RL_yaw = 0
		self.Motor_RR_yaw = 0

class var_demand:			# For angle calculations and parsing demands
	def __init__(self):
		# Initial received keymap
		self.keymap = '000000000'
		# Keymap parsed to demands
		self.left = '0'
		self.right = '0'
		self.forward = '0'
		self.backward = '0'
		self.cclockwise = '0'
		self.clockwise = '0'
		self.up = '0'
		self.down = '0'
		self.tune = '0'
		# Final demand outputs		
		self.x_angle_demand = 0
		self.y_angle_demand = 0
		self.z_angle_demand = 0
		self.throttle = 0

class var_gyro:
	def __init__(self):		
		self.PI = 3.14159265
		# setup
		self.mpu6050 = 0
		self.gyroscope = 0
		self.__k_norm = 1.0
		# calibration
		self.acc_offset_x = 0.0
		self.acc_offset_y = 0.0
		self.acc_offset_z = 0.0
		self.gyro_offset_x = 0.0
		self.gyro_offset_y = 0.0
		self.gyro_offset_z = 0.0
		# Save IMU data to arrays
		self.array_index = 0		
		self.acc_array_x = [0.0] * 10
		self.acc_array_y = [0.0] * 10
		self.acc_array_z = [0.0] * 10
		self.gyro_array_x = [0.0] * 10	
		self.gyro_array_y = [0.0] * 10
		self.gyro_array_z = [0.0] * 10
		# Anomaly detection
		self.anomaly_pre = False
		# Final output
		self.xRot_temp = 0.0
		self.yRot_temp = 0.0
		self.xRot = 0
		self.yRot = 0
		self.zAccel = 0
		
		self.acc_xRot_median = [0] * 5
		self.acc_xRot_median = [0] * 5
		self.median_i = 0
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
def PID_update_x():
	now = time.clock()*1000
	timeChange = now - pid.lastTime_x
	if timeChange >= pid.SampleTime:			
		pid.error_x = demand.x_angle_demand - sensor.xRot
		pid.P_value_x = round((pid.kp * pid.error_x), 2)
		pid.D_value_x = round((pid.kd * (pid.error_x - pid.error_x_last)), 2)
		pid.error_x_last = pid.error_x
	
		pid.ITerm_x = pid.ITerm_x + pid.error_x
		if pid.ITerm_x > pid.ITerm_max:
			pid.ITerm_x = pid.ITerm_max
		elif pid.ITerm_x < pid.ITerm_min:
			pid.ITerm_x = pid.ITerm_min
		pid.I_value_x = round((pid.ITerm_x * pid.ki), 2)

		pid.Output_x = round(pid.P_value_x + pid.I_value_x + pid.D_value_x)
		if pid.Output_x >= 0:
			pid.Motor_RL = -pid.Output_x
			pid.Motor_FR = pid.Output_x
		else:
			pid.Motor_RL = -pid.Output_x
			pid.Motor_FR = pid.Output_x
		pid.lastTime_x = now
			
		# Count how many times function gets called per second
		timer.PIDcount += 1
		PIDtimeChange = time.clock()*1000 - timer.PIDcount_start
		if PIDtimeChange >= 1000:
			timer.PIDcountpersec = timer.PIDcount
			timer.PIDcount = 0
			timer.PIDcount_start = time.clock()*1000
	else:
		pass

def PID_update_y():
	now = time.clock()*1000
	timeChange = now - pid.lastTime_y
	if timeChange >= pid.SampleTime:
		pid.lastTime_y = now			
		pid.error_y = demand.y_angle_demand - sensor.yRot
		pid.P_value_y = round((pid.kp * pid.error_y), 2)
		pid.D_value_y = round((pid.kd * (pid.error_y - pid.error_y_last)), 2)
		pid.error_y_last = pid.error_y
	
		pid.ITerm_y = pid.ITerm_y + pid.error_y
		if pid.ITerm_y > pid.ITerm_max:
			pid.ITerm_y = pid.ITerm_max
		elif pid.ITerm_y < pid.ITerm_min:
			pid.ITerm_y = pid.ITerm_min
		pid.I_value_y = round((pid.ITerm_y * pid.ki), 2)

		pid.Output_y = round(pid.P_value_y + pid.I_value_y + pid.D_value_y)
		if pid.Output_y >= 0:
			pid.Motor_RR = -pid.Output_y
			pid.Motor_FL = pid.Output_y
		else:
			pid.Motor_RR = -pid.Output_y
			pid.Motor_FL = pid.Output_y
	else:
		pass

def PID_update_z():
	now = time.clock()*1000
	timeChange = now - pid.lastTime_z
	if timeChange >= pid.SampleTime:
		pid.lastTime_z = now			
		pid.error_z = demand.z_angle_demand - sensor.zAccel
		pid.P_value_z = round((pid.kp_yaw * pid.error_z), 2)
		pid.D_value_z = round((pid.kd_yaw * (pid.error_z - pid.error_z_last)), 2)
		pid.error_z_last = pid.error_z
	
		pid.ITerm_z = pid.ITerm_z + pid.error_z
		if pid.ITerm_z > 15:
			pid.ITerm_z = 15
		elif pid.ITerm_z < -15:
			pid.ITerm_z = -15
		pid.I_value_z = round((pid.ITerm_z * pid.ki_yaw), 2)

		pid.Output_z = round(pid.P_value_z + pid.I_value_z + pid.D_value_z)
		if pid.Output_z >= 0:
			pid.Motor_FL_yaw = pid.Output_z
			pid.Motor_RR_yaw = pid.Output_z
			pid.Motor_FR_yaw = -pid.Output_z
			pid.Motor_RL_yaw = -pid.Output_z
		else:
			pid.Motor_FL_yaw = pid.Output_z
			pid.Motor_RR_yaw = pid.Output_z
			pid.Motor_FR_yaw = -pid.Output_z
			pid.Motor_RL_yaw = -pid.Output_z
	else:
		pass

def SetTunings(tuner, data):									# Change PID-controller behaviour
	if tuner == 'kp':
		pid.kp = data
	elif tuner == 'ki':
		pid.ki = data
	elif tuner == 'kd':
		pid.kd = data
	else:
		pass

def SetTunings_yaw(Kp, Ki, Kd):							# Change PID behaviour when turning
	#SampleTimeInSec = pid.SampleTime / 1000
	pid.kp_yaw = Kp
	pid.ki_yaw = Ki# * SampleTimeInSec
	pid.kd_yaw = Kd# / SampleTimeInSec

def SetSampleTime(NewSampleTime):
	if NewSampleTime > 0:
		self.ratio = NewSampleTime / pid.SampleTime
		pid.ki *= self.ratio
		pid.kd /= self.ratio
		pid.ki_yaw *= self.ratio
		pid.kd_yaw /= self.ratio
		pid.SampleTime = NewSampleTime

###// END OF PID FUNCTIONS //###########################################
########################################################################


# MPU-6050 GYROSCOPE ###################################################

def IMU_init():#(samplerate, gresolution):					# Wake the MPU-6050 up as it starts in sleep mode
	ready = False
	while ready == False:
		try:
			sensor.mpu6050 = MPU6050()									# var init. KEEP HERE, MIGHT FAIL OTHERWISE
			sensor.mpu6050.setup()
			print "IMU OK: init complete"
			#mpu6050.setSampleRate(samplerate)							# 100 probably ok
			#mpu6050.setGResolution(gresolution)							# 2g for lessening vibration
			ready = True											# Let us know if setup successful
		except:
			print "IMU error: init failed"
			# If setup fails, try again
	time.sleep(0.01)

def getEulerAngles(fax, fay, faz):
		#---------------------------------------------------------------------------
		# What's the angle in the x and y plane from horizonal?
		#---------------------------------------------------------------------------

		roll= math.atan2(fax, math.pow(math.pow(fay, 2) + math.pow(faz, 2), 0.5))
		pitch = math.atan2(fay, math.pow(math.pow(fax, 2) + math.pow(faz, 2), 0.5))
		yaw = math.atan2(math.pow(math.pow(fax, 2) + math.pow(fay, 2), 0.5), faz)

		roll *=  (180 / math.pi)
		pitch *=  (180 / math.pi)
		yaw *=  (180 / math.pi)

		return roll,pitch,yaw
	
def IMU_calibrate():										# Update IMU offsets
	print "IMU OK: Calibration init.."
	time.sleep(1)
	g=9.80665 #m/s^2
	ax_offset = 0.0
	ay_offset = 0.0
	az_offset = 0.0
	gx_offset = 0.0
	gy_offset = 0.0
	gz_offset = 0.0

	for loop_count in range(0, 100):
		newData = False
		while newData != True:
			try:
				while (sensor.mpu6050.readStatus() & 1)==0 :
					pass
			except:
				time.sleep(0.001)								# Sleep 1ms before trying again
			try: 
				offset = sensor.mpu6050.readData()
				gx_offset += offset.Gx
				gy_offset += offset.Gy
				gz_offset += offset.Gz
				ax_offset += offset.Accx
				ay_offset += offset.Accy
				az_offset += offset.Accz
				time.sleep(0.01)
				newData = True
			except:
				print "IMU error: calibration loop"

	sensor.gyro_offset_x = gx_offset / 100
	sensor.gyro_offset_y = gy_offset / 100
	sensor.gyro_offset_z = gz_offset / 100

	sensor.acc_offset_x = ax_offset / 100
	sensor.acc_offset_y = ay_offset / 100
	sensor.acc_offset_z = az_offset / 100

	sensor.__k_norm = math.pow((sensor.acc_offset_x),2) + math.pow((sensor.acc_offset_y),2) + math.pow((sensor.acc_offset_z),2)
	sensor.__k_norm = sensor.__k_norm/math.pow((g),2)
	sensor.__k_norm = float(math.pow((sensor.__k_norm),0.5))

#	print 'k write:' + str(sensor.__k_norm)
	print "IMU OK: Calibration complete"
	IMU_save_calibration()										# Save offset data to file

def IMU_save_calibration():
	print "IMU OK: Offset save init.."
	save_ok = False
	while save_ok == False:
		try:
			savefile = open('/home/terminal/offset_data', 'w')
			savefile.write(str(sensor.gyro_offset_x) +":"+ str(sensor.gyro_offset_y) +":"+ str(sensor.gyro_offset_z) +":"+ str(sensor.acc_offset_x) +":"+ str(sensor.acc_offset_y) +":"+ str(sensor.acc_offset_z) +":"+ str(sensor.__k_norm))
			savefile.close()
			save_ok = True
		except:
			print "IMU error: Saving offset data failed"
			time.sleep(0.01)
	print "IMU OK: Offsets saved"
	#IMU_load_calibration()		# Load saved offsets instantly into IMU

def IMU_load_calibration():
	print "IMU OK: Loading offsets init.."									# Load offset data from file
	load_ok = False
	while load_ok == False:
		try:
			savefile = open('/home/terminal/offset_data', 'r')
			data = savefile.readlines()
			for line in data:
				offset_data = [0] * 7
				offset_data = line.split(':')
			sensor.gyro_offset_x = float(offset_data[0])
			sensor.gyro_offset_y = float(offset_data[1])
			sensor.gyro_offset_z = float(offset_data[2])
			sensor.acc_offset_x = float(offset_data[3])
			sensor.acc_offset_y = float(offset_data[4])
			sensor.acc_offset_z = float(offset_data[5])
			sensor.__k_norm = float(offset_data[6])
			load_ok = True
			#sensor.__k_norm = float(offset_data[6])
			#sensor.mpu6050.readOffsets(sensor.acc_offset_x, sensor.acc_offset_y, sensor.acc_offset_z, sensor.gyro_offset_x, sensor.gyro_offset_y, sensor.gyro_offset_z, sensor.__k_norm)
		except:
			print "IMU error: Loading offsets failed"
			time.sleep(0.01)		# Wait 10ms before trying again
	print "IMU OK: Offsets loaded"

def IMU_convert_data():											# Convert raw IMU data into actual angles
	now = time.clock() * 1000
	timer.dtC = (now - timer.acc_dt_start) / 1000
	timer.acc_dt_start = time.clock() * 1000
	newData = False
	while newData != True:
		try:
			while (sensor.mpu6050.readStatus() & 1)==0 :
				pass
		except:
			pass
		try: 
			data = sensor.mpu6050.readData()
			newData = True
		except:
			print "data not found"

	data.Gx = (data.Gx - sensor.gyro_offset_x) * 100
	data.Gy = (data.Gy - sensor.gyro_offset_y) * 100 
	data.Gz = (data.Gz - sensor.gyro_offset_z) * 100
	data.Accx = (data.Accx - sensor.acc_offset_x)
	data.Accy = (data.Accy - sensor.acc_offset_y)
	data.Accz = (data.Accz - sensor.acc_offset_z)
	data.Gx = -data.Gx
	data.Gy = -data.Gy

	sensor.zAccel = round(data.Accz,1)
	RollAcc = -data.Accx
	PitchAcc = data.Accy
	# sensor.xRot_temp = 0.9 * (sensor.xRot_temp + (RollAcc * timer.dtC)) + (0.1 * data.Gx)
	# Only acc data
	sensor.xRot_temp = sensor.xRot_temp + (RollAcc * timer.dtC)
	sensor.xRot = round(sensor.xRot_temp, 1)
	# sensor.yRot_temp = 0.9 * (sensor.yRot_temp + (PitchAcc * timer.dtC)) + (0.1 * data.Gy)
	# Only acc data
	sensor.yRot_temp = sensor.yRot_temp + (PitchAcc * timer.dtC)
	sensor.yRot = round(sensor.yRot_temp, 1)

	response.gyro = str(sensor.xRot) , str(sensor.yRot) ,  str(sensor.zAccel)	# Response to server

########################################################################
########################################################################


# CHECK ANGLE DEMAND ###################################################

def check_demand():
	x_angle_tmp = 0
	y_angle_tmp = 0
	desired = 15
	
	time_start = time.clock() * 1000
	timeChange = time_start - timer.lastThrottle
	if timeChange >= 10:
		if demand.up == '1' and demand.down != '1':						# Up
			demand.throttle += 5
			timer.lastThrottle = time_start

	if demand.up != '1' and demand.down == '1':							# Down
		if demand.throttle >= -50:
			demand.throttle += -5
	
	if demand.tune == '1':
		tune_calibration()
	else:
		if demand.forward == '1':										# Forward
			y_angle_tmp += desired
			x_angle_tmp += -desired
		if demand.backward == '1':										# Backward
			y_angle_tmp += -desired
			x_angle_tmp += desired
		if demand.forward == '0' and demand.backward == '0':
			y_angle_tmp = 0
			x_angle_tmp = 0
		
		if demand.left == '1' and demand.right == '0':					# Left
			if x_angle_tmp == -desired or x_angle_tmp == desired:				# If Forward/Backward set
				y_angle_tmp += -desired
			else:														# If not set
				x_angle_tmp += -desired
				y_angle_tmp += -desired
		if demand.right == '1' and demand.left == '0':					# Right
			if y_angle_tmp == -desired or y_angle_tmp == desired:				# If Forward/Backward set
				x_angle_tmp += desired
			else:														# If not set
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
			sensor.acc_offset_x += 0.1
			sensor.acc_offset_y += -0.1
		if demand.backward == '1' and demand.forward == '0':
			sensor.acc_offset_x += -0.1
			sensor.acc_offset_y += 0.1
		if demand.left == '1' and demand.right == '0':
			sensor.acc_offset_x += 0.1
			sensor.acc_offset_y += 0.1
		if demand.right == '1' and demand.left == '0':
			sensor.acc_offset_x += -0.1
			sensor.acc_offset_y += -0.1
		
		sensor.mpu6050.readOffsets(sensor.acc_offset_x, sensor.acc_offset_y, sensor.acc_offset_z, sensor.gyro_offset_x, sensor.gyro_offset_y, sensor.gyro_offset_z, sensor.__k_norm)
		timer.lastCalib = time_start
	
####################################################################
####################################################################

	
### MOTOR FUNCTIONS ################################################

def calibrate_ESC():									# Calibrate ESC on boot. SHOULD ONLY BE MADE ONCE
	drone.set_servo_pulsewidth(motco.MOTOR1, 2000)
	drone.set_servo_pulsewidth(motco.MOTOR2, 2000)
	drone.set_servo_pulsewidth(motco.MOTOR3, 2000)
	drone.set_servo_pulsewidth(motco.MOTOR4, 2000)
	time.sleep(3)
	drone.set_servo_pulsewidth(motco.MOTOR1, 1000)
	drone.set_servo_pulsewidth(motco.MOTOR2, 1000)
	drone.set_servo_pulsewidth(motco.MOTOR3, 1000)
	drone.set_servo_pulsewidth(motco.MOTOR4, 1000)
	time.sleep(1.7)
	drone.set_servo_pulsewidth(motco.MOTOR1, 1045)
	drone.set_servo_pulsewidth(motco.MOTOR2, 1045)
	drone.set_servo_pulsewidth(motco.MOTOR3, 1045)
	drone.set_servo_pulsewidth(motco.MOTOR4, 1045)
	time.sleep(0.2)

def motorcontrol(FL, FR, RR, RL):	
	motco.motorFL_pw = 1000 + demand.throttle + FL + pid.Motor_FL_yaw
	motco.motorFR_pw = 1000 + demand.throttle + FR + pid.Motor_FR_yaw
	motco.motorRR_pw = 1000 + demand.throttle + RR + pid.Motor_RR_yaw
	motco.motorRL_pw = 1000 + demand.throttle + RL + pid.Motor_RL_yaw  + 4
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

####################################################################
####################################################################


### TCP SOCKET CONNECTION ##########################################

def server_response(*args):
	failsafe_reset()
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

def server_adjustpid_p(*args):
	data = args
	try:
		temp = data[0].encode('ascii')
		kp = float(temp)
		SetTunings('kp', kp)
		socketIO.emit('logger', "P adjusted to " + str(kp))
	except:
		pass

def server_adjustpid_i(*args):
	data = args
	try:
		temp = data[0].encode('ascii')
		ki = float(temp)
		SetTunings('ki', ki)
		socketIO.emit('logger', "I adjusted to " + str(ki))
	except:
		pass

def server_adjustpid_d(*args):
	data = args
	try:
		temp = data[0].encode('ascii')
		kd = float(temp)
		SetTunings('kd' , kd)
		socketIO.emit('logger', "D adjusted to " + str(kd))
	except:
		pass

def server_adjustpidmax(*args):
	data = args[0].encode('ascii')
	try:
		pid.ITerm_max = int(data)
		pid.ITerm_min = -pid.outMax
		socketIO.emit('logger', "Imax adjusted to " + str(pid.outMax))
	except:
		pass

def server_calibration(*args):
	data = str(args)
	IMU_calibrate()											# Calibrate gyroscope 
	socketIO.emit('logger', "IMU calibrated")
		
def server_calibration_tuning(*args):
	data = str(args)
	try:
		temp = data[0].encode('ascii')
		for index in temp:
			if index == 1:
				index = -0.25
			elif index == 2:
				index = 0.25
		sensor.xRot_calib += float(temp[0])
		sensor.yRot_calib += float(temp[1])
	except:
		pass

def failsafe_reset():
	timer.failsafe = time.clock()*1000
	if timer.failsafe_triggered == True:
		timer.failsafe_triggered = False
		demand.throttle = timer.failsafe_throttle

def failsafe():
	timeChange = time.clock()*1000 - timer.failsafe
	if timeChange >= 2000:
		timer.failsafe_triggered = True
		timer.failsafe_throttle = demand.throttle
		demand.throttle = 250
		
####################################################################
####################################################################


### THREADS ########################################################

### THREAD 1 - COMMUNICATION ###############################	
# // Sends IMU data, receives control commands, assigns commands to global variables	
def thread1():
	timer.COMMcount_start = time.clock()*1000
	count = 0												# Count thread loops per second
	failsafe_reset()										# Reset failsafe timer on start of the thread
	while True:
		time_start = time.clock() * 1000
		timeChange = time_start - timer.thread1_last
		timeChange2 = time_start - timer.thread1_last_pidvalues
		socketIO.wait(0.001)								# Continuously listen for server messages
		
		if timeChange >= 1000:								# Send IMU data to server each second
			socketIO.emit('DRONErequest', str(response.gyro))
			timer.thread1_last = time.clock() * 1000
		if timeChange2 >= 10000:							# Send PID values / other data to server
			socketIO.emit('logger', "PID values: " + str(pid.kp) +",  "+ str(pid.ki) +",  "+ str(pid.kd))
			timer.thread1_last_pidvalues = time.clock() * 1000
		count += 1
		
		timeChange = time.clock()*1000 - timer.COMMcount_start	# Save and reset counter data each second
		if timeChange >= 1000:
			timer.COMMcountpersec = count
			count = 0
			timer.COMMcount_start = time.clock()*1000
		
		time.sleep(0.002)
		timer.time_thread_1 = round((time.clock() * 1000) - time_start)

### THREAD 2 - MOTOR CONTROL ###############################	
# // Motor control thread. Read user demands, calculate PID values, Control ESC, FAILSAFE 	
def thread2():
	timer.PIDcount_start = time.clock()*1000
	while True:
		time_start = time.clock() * 1000
		check_demand()															# Check directions where user wants to go
		PID_update_x()															# Compute x output
		PID_update_y()															# Compute y output
		PID_update_z()															# Compute z output
		failsafe()
		motorcontrol(pid.Motor_FL, pid.Motor_FR, pid.Motor_RR, pid.Motor_RL)	# Commands to motors!
		time.sleep(0.001)
		timer.time_thread_2 = round((time.clock()*1000) - time_start)
			
### THREAD 3 - GYROSCOPE ###################################
# // Read and parse IMU data
def thread3():
	timer.samplecount_start = time.clock()*1000
	count = 0
	while True:
		time_start = time.clock() * 1000
		try:
			IMU_convert_data()
			count += 1
		except:
			pass
		
		timeChange = time.clock()*1000 - timer.samplecount_start
		if timeChange >= 1000:
			timer.samplecountpersec = count
			count = 0
			timer.samplecount_start = time.clock()*1000
			
		#time_end = time.clock() * 1000
		time.sleep(0.0015)		# Sleep for a while to prevent thread crash..
		timer.time_thread_3 = round((time.clock()*1000) - time_start)

####################################################################
####################################################################


# MAIN INITIALIZATION ##############################################
terminal = False
drone = pigpio.pi()											# IO ports, pigpio-library
timer = var_timer()											# General timing class
response = var_response()									# Class for saving server responses
pid = var_pid()												# Class for PID-calculations
demand = var_demand()										# Client demands
sensor = var_gyro()											# IMU data
motco = var_motco()											# Motor controller variables

IMU_init()													# Initialize IMU
IMU_calibrate()											# Update IMU offsets and save to file
IMU_load_calibration()										# Load IMU offsets from file
time.sleep(0.05)
SetTunings('kp', 0.15)										# PID tunings
SetTunings('ki', 0.02)
SetTunings('kd', 0)
SetTunings_yaw(0.4, 0.01, 0)									# Yaw PID tunings here!

received = False
while received == False:
	try:
		drone_ip = ni.ifaddresses('ppp0')[2][0]['addr']		# Get IP from system ifaddresses
		received = True
	except:
		pass
print drone_ip												# Print IP on boot
socketIO = SocketIO('http://avela.ddns.net', 3000)			# Initializes connection to avela.ddns.net, port 3000
socketIO.emit('DRONEboot', drone_ip)						# Send IP to server
socketIO.on('server_pid_p', server_adjustpid_p)
socketIO.on('server_pid_i', server_adjustpid_i)
socketIO.on('server_pid_d', server_adjustpid_d)
socketIO.on('server_adjustpidmax', server_adjustpidmax)
socketIO.on('calibrate', server_calibration)
socketIO.on('IOanswer', server_response)
socketIO.wait(0.005)
print "Connected to server"

calibrate_ESC()

try:
	thread.start_new_thread( thread1, () )
	print "Thread 1 OK"
	thread.start_new_thread( thread2, () )
	print "Thread 2 OK"
	thread.start_new_thread( thread3, () )
	print "Thread 3 OK"
except:
	print "Error: unable to start thread"

time.sleep(0.001)
if terminal == True: #os.isatty(sys.stdin.fileno()):
										# Below starts CLI for debugging purposes via terminal
	try:
		screen = curses.initscr()							# Init CURSES
		screen.border(0)									# Give cool borders
		screen.addstr(1, 5,  "DRONE COMMAND CENTER")		# Print stuff..
		screen.addstr(3, 5,  "PID variables #####")
		screen.addstr(7, 5,  "Angular error #####")
		screen.addstr(11, 5, "Gyroscope data ####")
		screen.addstr(15, 5, "Thread time (ms) #####")
		screen.addstr(3, 31, "Motor speeds ######")
		screen.addstr(5, 33, "\\")
		screen.addstr(6, 34, "\\")
		screen.addstr(7, 36, "O")
		screen.addstr(8, 34, "/")
		screen.addstr(9, 33, "/")
		screen.addstr(5, 39, "/")
		screen.addstr(6, 38, "/")
		screen.addstr(8, 38, "\\")
		screen.addstr(9, 39, "\\")
		screen.addstr(15, 31, "1/second ######")
		screen.addstr(20, 31, "PID values ####")
		screen.addstr(21, 31, "P:")
		screen.addstr(22, 31, "I:")
		screen.addstr(23, 31, "D:")
			
		screen.addstr(4, 5, "X:")
		screen.addstr(5, 5, "Y:")
		screen.addstr(6, 5, "Z:")
				
		screen.addstr(8, 5, "X:")
		screen.addstr(9, 5, "Y:")
		screen.addstr(10, 5, "Z:")
				
		screen.addstr(12, 5, "X:")
		screen.addstr(13, 5, "Y:")
		screen.addstr(14, 5, "Z:")
			
		screen.addstr(16, 5, "T1:")
		screen.addstr(17, 5, "T2:")
		screen.addstr(18, 5, "T3:")
		while True:
			if pid.kp != 0:
				screen.addstr(1, 31, drone_ip)
				screen.addstr(1, 45, "READY")
			elif pid.kp != 1:
				screen.addstr(1, 31, "booting..")
				
			screen.addstr(4, 9, str(pid.Output_x))
			screen.addstr(5, 9, str(pid.Output_y))
			screen.addstr(6, 9, str(pid.Output_z))
				
			screen.addstr(8, 9, str(pid.error_x))
			screen.addstr(9, 9, str(pid.error_y))
			screen.addstr(10, 9, str(pid.error_z))
				
			screen.addstr(12, 9, str(sensor.xRot))
			screen.addstr(13, 9, str(sensor.yRot))
			screen.addstr(14, 9, str(sensor.zAccel))
				
			screen.addstr(16, 9, str(timer.time_thread_1))
			screen.addstr(17, 9, str(timer.time_thread_2))
			screen.addstr(18, 9, str(timer.time_thread_3))
				
			screen.addstr(16, 32, str(timer.COMMcountpersec))
			screen.addstr(17, 32, str(timer.PIDcountpersec))
			screen.addstr(18, 32, str(timer.samplecountpersec))
				
			screen.addstr(21, 34, str(pid.P_value_x))
			screen.addstr(22, 34, str(pid.I_value_x))
			screen.addstr(23, 34, str(pid.D_value_x))
				
			screen.addstr(4, 31, str(motco.motorFL_pw))
			screen.addstr(4, 38, str(motco.motorFR_pw))
			screen.addstr(10, 31, str(motco.motorRL_pw))
			screen.addstr(10, 38, str(motco.motorRR_pw))
				
				#screen.addstr(11, 38, str(timer.asd))
				#screen.addstr(12, 38, str(timer.asd2))
				
			screen.refresh()
			#curses.endwin() # shuts down curse

	except (KeyboardInterrupt, SystemExit):
		# quit
		curses.endwin()
		demand.throttle = 0
		motorcontrol(0,0,0,0)
		print "Quitting.."
		time.sleep(1)
		exit()

else:
	while True:
		time.sleep(0.005)
		pass
			
		
#while terminal == False:
#	time.sleep(0.005)
#	os.system('clear')
#	print sensor.xRot
#	print sensor.yRot
#	print sensor.zAccel
