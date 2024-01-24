import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import serial
import math
import threading
from rclpy import qos

PI = 3.141592

def split(data, delimiter):
    return data.split(delimiter)

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu')
        self.publish_rate = 500
        self.ser = serial.Serial()

        self.imu_pub0 = self.create_publisher(Imu, '/imu/data', qos.qos_profile_sensor_data)

        try:
            self.ser.port = '/dev/ttyUSB0'
            self.ser.baudrate = 921600
            self.ser.timeout = 0.01
            self.ser.open()
        except serial.SerialException as e:
            self.get_logger().error(f"Unable to open port: {e}")
            return

        if self.ser.is_open:
            self.get_logger().info("Serial Port initialized")
        else:
            return
        
        self.read_serial()


    def read_serial(self):
        while True:
            if self.ser.in_waiting:
                self.imu_data = Imu()
                self.imu_data.header.stamp = self.get_clock().now().to_msg()
                self.imu_data.header.frame_id = 'imu'
                data = self.ser.readline().decode()
                lines = data.strip().split('\n')
                print(lines)
                for line in lines:
                    tempresult = line.split(',')
                    print(len(tempresult))
                    if len(tempresult) == 10:
                        tempresult[0] = tempresult[0].lstrip('*')
                        quaternion_w = float(tempresult[3])
                        quaternion_x = float(tempresult[2])
                        quaternion_y = -float(tempresult[1])
                        quaternion_z = -float(tempresult[0])
                        angular_velocity_x = float(tempresult[4])
                        angular_velocity_y = float(tempresult[5])
                        angular_velocity_z = float(tempresult[6])
                        linear_acceleration_x = float(tempresult[7])
                        linear_acceleration_y = float(tempresult[8])
                        linear_acceleration_z = float(tempresult[9])

                        q = Quaternion()
                        q.x = quaternion_x
                        q.y = quaternion_y
                        q.z = quaternion_z
                        q.w = quaternion_w
                        q = self.normalize_quaternion(q)

                        self.imu_data.orientation = q
                        self.imu_data.linear_acceleration = Vector3(x=linear_acceleration_x, y=linear_acceleration_y, z=linear_acceleration_z)
                        self.imu_data.angular_velocity = Vector3(x=angular_velocity_x * (PI / 180.0), y=angular_velocity_y * (PI / 180.0), z=angular_velocity_z * (PI / 180.0))
                        self.imu_pub0.publish(self.imu_data)

    def normalize_quaternion(self, q):
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return q

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
