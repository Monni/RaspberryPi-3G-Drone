import unittest
#from unittest.mock import MagicMock
import numbers
from drone import *

class droneTests(unittest.TestCase):

    # Setup method to create a test object
   # @classmethod
    def setUp(self):
        self.drone = drone()
        #cls.pid = var_pid()
        #self.drone = drone()
       # pid = var_pid()
        #self.drone.demand = var_demand()
        #self.connection = var_connection()
        #self.sensor = var_gyro()
        
        print ("setUp")

    # Teardown method to delete the test object
    #@classmethod
    def tearDown(self):
        del self.drone
        #del self.drone
        #del self.connection
        print ("tearDown")

   # def test_imu_init(self):
   #     drone.IMU_init()
   # #?    self.assertEqual(1,1,"asd")


   ######################################################
   # PID tests
   ######################################################

    def test_PID_update_x(self):
        self.drone.pid.lastTime_x = -1000 # bypass timer
        self.drone.sensor.xRot = 100
        self.drone.pid.kp = 5
        Output_x_temp = self.drone.pid.Output_x
        self.drone.PID_update_x()
        self.assertNotEqual(self.drone.pid.Output_x, Output_x_temp, "asd")
   
    def test_PID_update_y(self):
        self.drone.pid.lastTime_y = -1000 # bypass timer
        self.drone.sensor.yRot = 100
        self.drone.pid.kp = 5
        Output_y_temp = self.drone.pid.Output_y
        self.drone.PID_update_y()
        self.assertNotEqual(self.drone.pid.Output_y, Output_y_temp, "asd")
        
    def test_PID_update_z(self):
        self.drone.pid.lastTime_z = -1000 # bypass timer
        self.drone.sensor.zAccel = 100
        self.drone.pid.kp_yaw = 5
        Output_z_temp = self.drone.pid.Output_z
        self.drone.PID_update_z()
        self.assertNotEqual(self.drone.pid.Output_z, Output_z_temp, "asd")

    def test_SetTunings_kp(self):
        self.drone.pid.SetTunings("kp",1)
        self.assertEqual(self.drone.pid.kp, 1, "SetTunings doesn't accept kp-value")

#    def test_SetTunings_ki(self):
 #       self.drone.pid.SetTunings("ki", 1)
  #      self.assertEqual(self.drone.pid.ki, 1, "SetTunings doesn't accept ki-value")

  #  def test_SetTunings_kd(self):
   #     self.drone.pid.SetTunings("kd", 1)
    #    self.assertEqual(self.drone.pid.kd, 1, "SetTunings doesn't accept kd-value")

#    def test_setIP(self):
#        self.connection.setIP()
#        self.assertIsInstance(self.connection.drone_ip, str, "Couldn't get IP address from ifaddresses")

#    def test_SetTunings_yaw(self):
 #       self.drone.pid.SetTunings_yaw(1, 1, 1)
  #      self.assertEqual(self.drone.pid.kp_yaw, 1, "SetTunings_yaw doesn't accept kp_yaw-value")
   #     self.assertEqual(self.drone.pid.ki_yaw, 1, "SetTunings_yaw doesn't accept ki_yaw-value")
    #    self.assertEqual(self.drone.pid.kd_yaw, 1, "SetTunings_yaw doesn't accept kd_yaw-value")

#    def test_PID_update_x(self):
 #       self.drone.pid.lastTime_x = -2000 #bypass timer
  #      self.drone.pid.error_x = 100
   #     x_temp = self.drone.pid.Output_x
    #    self.drone.pid.PID_update_x()
     #   assertNotEqual(self.drone.pid.Output_x, x_temp, "PID_update_x output doesn't update")
        
        

   # def test_startThreads(self):
   #check if threads start

    # IMU
    #def IMU_init()

 #   def test_getEulerAngles_pitch(self):
  #      pitch, roll, yaw = self.sensor.getEulerAngles(10, 10, 1)
   #     self.assertIsInstance(pitch, numbers.Number, "sensor.getEulerAngles pitch value is not a number")

 #   def test_getEulerAngles_roll(self):
  #      pitch, roll, yaw = self.sensor.getEulerAngles(10, 10, 1)
   #     self.assertIsInstance(roll, numbers.Number, "sensor.getEulerAngles roll value is not a number")

  #  def test_getEulerAngles_yaw(self):
   #     pitch, roll, yaw = self.sensor.getEulerAngles(10, 10, 1)
    #    self.assertIsInstance(yaw, numbers.Number, "sensor.getEulerAngles yaw value is not a number")

    # def IMU_calibrate(this)

    #def test_IMU_save_calibration(self):

    #def test_IMU_load_calibration(self):

    #def test_tune_calibration(self):

    #def test_IMU_convert_data(self):
    

if __name__ == '__main__':
    unittest.main()
