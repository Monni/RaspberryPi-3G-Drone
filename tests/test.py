import unittest
from drone import *

class droneTests(unittest.TestCase):

    # Setup method to create a test object
    def setUp(self):
        self.pid = var_pid()
        self.drone = drone()
        self.connection = var_connection()
        print ("setUp")

    # Teardown method to delete the test object
    def tearDown(self):
        del self.pid
        del self.drone
        del self.connection
        print ("tearDown")

   # def test_imu_init(self):
   #     drone.IMU_init()
   # #?    self.assertEqual(1,1,"asd")


    def test_SetTunings_kp(self):
        self.pid.SetTunings("kp", 1)
        self.assertEqual(self.pid.kp, 1, "SetTunings doesn't accept kp-value")

    def test_SetTunings_ki(self):
        self.pid.SetTunings("ki", 1)
        self.assertEqual(self.pid.ki, 1, "SetTunings doesn't accept ki-value")

    def test_SetTunings_kd(self):
        self.pid.SetTunings("kd", 1)
        self.assertEqual(self.pid.kd, 1, "SetTunings doesn't accept kd-value")

#    def test_setIP(self):
#        self.connection.setIP()
#        self.assertIsInstance(self.connection.drone_ip, str, "Couldn't get IP address from ifaddresses")

    def test_SetTunings_yaw(self):
        self.pid.SetTunings_yaw(1, 1, 1)
        self.assertEqual(self.pid.kp_yaw, 1, "SetTunings_yaw doesn't accept kp_yaw-value")
        self.assertEqual(self.pid.ki_yaw, 1, "SetTunings_yaw doesn't accept ki_yaw-value")
        self.assertEqual(self.pid.kd_yaw, 1, "SetTunings_yaw doesn't accept kd_yaw-value")

   # def test_startThreads(self):
   #check if threads start

        

if __name__ == '__main__':
    unittest.main()
