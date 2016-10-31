import unittest
#from unittest.mock import MagicMock
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

   ######################################################
   # PID class tests
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
   #           DRONE class tests (main)                 #
   ######################################################



   ######################################################
   #           CONNECTION class tests                   #
   ######################################################

    #def test_setIP_setsIp(self):

    #def test_connect_opensConnection(self): # Needs python SocketIO-client 0.6.4 to work
    #    self.drone.connection.connect()


    ######################################################
    #             var_gyro class tests                   #
    ######################################################

    #def test_imuInit_wakesImu(self):
    #   self.drone.sensor.imuInit()      

    def test_getEulerAngles_pitchIsNumber(self):
        pitch, roll, yaw = self.drone.sensor.getEulerAngles(10, 10, 1)
        self.assertIsInstance(pitch, numbers.Number, "sensor.getEulerAngles pitch doesn't return numerical value")

    def test_getEulerAngles_rollIsNumber(self):
        pitch, roll, yaw = self.drone.sensor.getEulerAngles(10, 10, 1)
        self.assertIsInstance(roll, numbers.Number, "sensor.getEulerAngles roll doesn't return numerical value")

    def test_getEulerAngles_yawIsNumber(self):
        pitch, roll, yaw = self.drone.sensor.getEulerAngles(10, 10, 1)
        self.assertIsInstance(yaw, numbers.Number, "sensor.getEulerAngles yaw doesn't return numerical value")

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
