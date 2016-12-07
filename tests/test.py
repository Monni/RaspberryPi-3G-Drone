import unittest
from unittest.mock import MagicMock
import numbers
from drone import *

class droneTests(unittest.TestCase):

    # Setup method to create a test object
    def setUp(self):
        self.drone = drone()
        print ("setUp")

    # Teardown method to delete the test object
    def tearDown(self):
        del self.drone
        print ("tearDown")

    #####################################################
    # DEMAND class tests                                #
    #####################################################

    #def test_check_demand(self):

   ######################################################
   #                PID class tests                     #
   ######################################################

    def test_pidUpdateX_updatesFromKp(self):
        self.drone.pid.lastTime_x = -1000 # bypass timer
        self.drone.sensor.xRot = 100
        self.drone.pid.kp = 5
        Output_x_temp = self.drone.pid.Output_x
        self.drone.pidUpdateX()
        self.assertNotEqual(self.drone.pid.Output_x, Output_x_temp, "pidUpdateX doesn't calculate from kp value")
   
    def test_pidUpdateY_updatesFromKp(self):
        self.drone.pid.lastTime_y = -1000 # bypass timer
        self.drone.sensor.yRot = 100
        self.drone.pid.kp = 5
        Output_y_temp = self.drone.pid.Output_y
        self.drone.pidUpdateY()
        self.assertNotEqual(self.drone.pid.Output_y, Output_y_temp, "pidUpdateY doesn't calculate from kp value")
        
    def test_pidUpdateZ_updatesFromKp(self):
        self.drone.pid.lastTime_z = -1000 # bypass timer
        self.drone.sensor.zAccel = 100
        self.drone.pid.kp_yaw = 5
        Output_z_temp = self.drone.pid.Output_z
        self.drone.pidUpdateZ()
        self.assertNotEqual(self.drone.pid.Output_z, Output_z_temp, "pidUpdateZ doesn't calculate from kp value")

    def test_pidUpdateX_updatesFromKi(self):
        self.drone.pid.lastTime_x = -1000 # bypass timer
        self.drone.sensor.xRot = 100
        self.drone.pid.ki = 5
        Output_x_temp = self.drone.pid.Output_x
        self.drone.pidUpdateX()
        self.assertNotEqual(self.drone.pid.Output_x, Output_x_temp, "pidUpdateX doesn't calculate from ki value")

    def test_pidUpdateY_updatesFromKi(self):
        self.drone.pid.lastTime_y = -1000 # bypass timer
        self.drone.sensor.yRot = 100
        self.drone.pid.ki = 5
        Output_y_temp = self.drone.pid.Output_y
        self.drone.pidUpdateY()
        self.assertNotEqual(self.drone.pid.Output_y, Output_y_temp, "pidUpdateY doesn't calculate from ki value")
        
    def test_pidUpdateZ_updatesFromKi(self):
        self.drone.pid.lastTime_z = -1000 # bypass timer
        self.drone.sensor.zAccel = 100
        self.drone.pid.ki_yaw = 5
        Output_z_temp = self.drone.pid.Output_z
        self.drone.pidUpdateZ()
        self.assertNotEqual(self.drone.pid.Output_z, Output_z_temp, "pidUpdateZ doesn't calculate from ki value")

    def test_pidUpdateX_updatesFromKd(self):
        self.drone.pid.lastTime_x = -1000 # bypass timer
        self.drone.sensor.xRot = 100
        self.drone.pid.kd = 5
        Output_x_temp = self.drone.pid.Output_x
        self.drone.pidUpdateX()
        self.assertNotEqual(self.drone.pid.Output_x, Output_x_temp, "pidUpdateX doesn't calculate from kd value")

    def test_pidUpdateY_updatesFromKd(self):
        self.drone.pid.lastTime_y = -1000 # bypass timer
        self.drone.sensor.yRot = 100
        self.drone.pid.kd = 5
        Output_y_temp = self.drone.pid.Output_y
        self.drone.pidUpdateY()
        self.assertNotEqual(self.drone.pid.Output_y, Output_y_temp, "pidUpdateY doesn't calculate from kd value")
        
    def test_pidUpdateZ_updatesFromKd(self):
        self.drone.pid.lastTime_z = -1000 # bypass timer
        self.drone.sensor.zAccel = 100
        self.drone.pid.kd_yaw = 5
        Output_z_temp = self.drone.pid.Output_z
        self.drone.pidUpdateZ()
        self.assertNotEqual(self.drone.pid.Output_z, Output_z_temp, "pidUpdateZ doesn't calculate from kd value")
        
    def test_pidUpdateX_returnsNumber(self):
        self.drone.pid.lastTime_x = -100 # Bypass timer
        self.drone.sensor.xRot = 100
        self.drone.pid.kp = 5
        self.drone.pidUpdateX()
        self.assertIsInstance(self.drone.pid.Output_x, numbers.Number, "pidUpdateX doesn't return numerical value")

    def test_pidUpdateY_returnsNumber(self):
        self.drone.pid.lastTime_y = -100 # Bypass timer
        self.drone.sensor.yRot = 100
        self.drone.pid.kp = 5
        self.drone.pidUpdateY()
        self.assertIsInstance(self.drone.pid.Output_y, numbers.Number, "pidUpdateX doesn't return numerical value")

    def test_pidUpdateZ_returnsNumber(self):
        self.drone.pid.lastTime_z = -100 # Bypass timer
        self.drone.sensor.zAccel = 100
        self.drone.pid.kp = 5
        self.drone.pidUpdateZ()
        self.assertIsInstance(self.drone.pid.Output_z, numbers.Number, "pidUpdateX doesn't return numerical value")
        
    def test_SetTunings_kp(self):
        self.drone.pid.SetTunings("kp",1)
        self.assertEqual(self.drone.pid.kp, 1, "SetTunings doesn't accept kp-value")

    def test_SetTunings_ki(self):
        self.drone.pid.SetTunings("ki", 1)
        self.assertEqual(self.drone.pid.ki, 1, "SetTunings doesn't accept ki-value")

    def test_SetTunings_kd(self):
        self.drone.pid.SetTunings("kd", 1)
        self.assertEqual(self.drone.pid.kd, 1, "SetTunings doesn't accept kd-value")

    def test_SetTunings_yaw(self):
        self.drone.pid.SetTunings_yaw(1, 1, 1)
        self.assertEqual(self.drone.pid.kp_yaw, 1, "SetTunings_yaw doesn't accept kp_yaw-value")
        self.assertEqual(self.drone.pid.ki_yaw, 1, "SetTunings_yaw doesn't accept ki_yaw-value")
        self.assertEqual(self.drone.pid.kd_yaw, 1, "SetTunings_yaw doesn't accept kd_yaw-value")

    ######################################################
    #             var_gyro class tests                   #
    ######################################################

    #def test_imuInit_wakesImu(self):
    #   self.drone.sensor.imuInit()      

    def test_getEulerAngles_pitchReturnsNumber(self):
        pitch, roll, yaw = self.drone.sensor.getEulerAngles(10, 10, 1)
        self.assertIsInstance(pitch, numbers.Number, "sensor.getEulerAngles pitch doesn't return numerical value")

    def test_getEulerAngles_rollReturnsNumber(self):
        pitch, roll, yaw = self.drone.sensor.getEulerAngles(10, 10, 1)
        self.assertIsInstance(roll, numbers.Number, "sensor.getEulerAngles roll doesn't return numerical value")

    def test_getEulerAngles_yawReturnsNumber(self):
        pitch, roll, yaw = self.drone.sensor.getEulerAngles(10, 10, 1)
        self.assertIsInstance(yaw, numbers.Number, "sensor.getEulerAngles yaw doesn't return numerical value")

    #def test_imuCalibrate_(self):
        #kaikki data toimii

    #def test_imuSaveCalibration_savesDataToFile(self):

    #def test_imuLoadCalibration_loadsDataFromFile(self):

    ######################################################
    #               Motco class tests                    #
    ######################################################

    def test_init_worksCorrectly(self):
        # Motor assignment to GPIO ports
        self.assertEquals(self.drone.motco.MOTOR1, 4, "Motor1 GPIO port incorrect, should be 4")
        self.assertEquals(self.drone.motco.MOTOR2, 17, "Motor2 GPIO port incorrect, should be 17")
        self.assertEquals(self.drone.motco.MOTOR3, 27, "Motor3 GPIO port incorrect, should be 27")
        self.assertEquals(self.drone.motco.MOTOR4, 22, "Motor4 GPIO port incorrect, should be 22")
        # Main pulsewidth variables
        self.assertEquals(self.drone.motco.motorFL_pw, 1000, "MotorFL_pw formatted wrong, should be 1000")
        self.assertEquals(self.drone.motco.motorFR_pw, 1000, "MotorFR_pw formatted wrong, should be 1000")
        self.assertEquals(self.drone.motco.motorRR_pw, 1000, "MotorRR_pw formatted wrong, should be 1000")
        self.assertEquals(self.drone.motco.motorRL_pw, 1000, "MotorRL_pw formatted wrong, should be 1000")
        # Minimum and maximum pulsewidths
        self.assertEquals(self.drone.motco.MIN_PW, 1000, "MIN_PW formatted wrong, should be 1000")
        self.assertEquals(self.drone.motco.MAX_PW, 1999, "MAX_PW formatted wrong, should be 1000")

    ######################################################
    #           CONNECTION class tests                   #
    ######################################################

    def test_init_worksCorrectly(self):
        self.assertEqual(self.drone.connection.gyro, '', "gyro variable formatted wrong in connection")
        self.assertEqual(self.drone.connection.throttle, 0, "throttle variable formatted wrong in connection")
        self.assertEqual(self.drone.connection.ip, 0, "ip variable formatted wrong in connection")
        
    #def test_setIP_setsIp(self):

    #def test_connect_opensConnection(self): # Needs python SocketIO-client 0.6.4 to work
    #    self.drone.connection.connect()

    #####################################################
    # Drone class tests                                 #
    #####################################################

    #####
    # Motor functions

    #def test_calibrate_ESC(self):

    #def test_motorcontrol_adjustsPulsewidth(self):

    #####
    # Connection functions

    #def test_server_response_sendKeymapToKeymapper(self):

    #def test_server_keymapper_setsDemands(self):

    #def test_server_adjustpid_p_passesVariableToFunction(self):

    #def test_server_adjustpid_i_passesVariableToFunction(self):

    #def test_server_adjustpid_d_passesVariableToFunction(self):

    #def test_server_adjustpidmax_passesVariableToFunction(self):

    #def test_server_calibration_passesVariableToFunction(self):

    #def test_server_calibration_tuning_adjustsCalibValues(self):

    #def test_failsafe_reset_resetsThrottle(self):

    #def test_failsafe_cutsThrottle(self):
        





#    @patch('self.drone.sensor.mpu6050.readStatus')
 #   def test_imuCalibrate_updatesGyroOffsetX(self, mock_function):
  #      mock_function.return_value = MagickMock(0)
   #     self.drone.sensor.mpu6050.readStatus()
    #    offset_temp = self.drone.sensor.gyro_offset_x
     #   self.drone.sensor.imuCalibrate()
      #  self.assertNotEqual(self.drone.sensor.gyro_offset_x, offset_temp, "sensor.imuCalibrate doesn't update gyro_offset_x")

    #def test_tuneCalibration_updatesOffsets(self):        

    #def test_imuConvertData_KLUP(self):

    

if __name__ == '__main__':
    unittest.main()
