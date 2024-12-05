'''
manual_control_pub.py

Defines the publisher node for manual control.
Provides function for commanding the robot, including locomotion, 
 excavation, construction, and stoppage functionality
'''

from .action_map import ActionMap
from std_msgs.msg import Byte
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

class ManualControlPublisher(Node):
    def __init__(self):
        super().__init__("manual_control_publisher")
        self.publisher_ = self.create_publisher(
            Byte, "robotCmds/manual", 10)

        # Setting up robot values
        # Loco initial values
        self.DEFAULT_LOCO_SPEED = 25
        self.loco_speed = self.DEFAULT_LOCO_SPEED
        self.loco_direction = None

        # Min and max speeds
        self.MIN_LOCO_SPEED = 25
        self.MAX_LOCO_SPEED = 100

        # Mapping from % to bytes (loco)
        self.LOCO_SPEED_MAP = {
            25: b'\x00',
            50: b'\x01',
            75: b'\x02',
            100: b'\x03'
        }

        # Clients for starting autonomy
        self.excavation_cli = self.create_client(Empty, "excavation_autonomy")
        self.construction_cli = self.create_client(Empty, "construction_autonomy")

        # Making sure services are available
        while not self.excavation_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Excavation service not available, waiting again...")
        self.excavation_req = Empty.Request()

        while not self.excavation_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Construction service not available, waiting again...")
        self.construction_req = Empty.Request()

    def publish_msg(self, data: bytes):
        '''
        Publish a msg to a topic
        :param data: data to be published
        :return: None
        '''
        msg = Byte()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing: 0x{:02x}".format(int.from_bytes(data, 'little')))

    def clamp(self, value: int, lower: int, upper: int) -> int:
        '''
        Helper method to check value within bounds
        :param value: value to be checked
        :param lower: lower bound
        :param upper: upper bound
        :return: int value within [lower, upper]
        '''
        if value < lower:
            return lower
        elif value > upper:
            return upper
        return value

    def loco_forward(self) -> None:
        '''
        Method to move robot forward
        :return: None
        '''
        # Getting forward cmd & speed value
        cmd = ActionMap.LOCO_FORWARD.value[0]
        speed = self.LOCO_SPEED_MAP[self.loco_speed][0]
        cmd |= speed

        # Publishing direction & speed
        self.publish_msg(cmd.to_bytes(1, 'little'))

        # Setting most recent direction to forward
        self.loco_direction = self.loco_forward

    def loco_backward(self):
        '''
        Method to move the robot backward
        :return: None
        '''
        # Getting backward cmd & speed value
        cmd = ActionMap.LOCO_BACKWARD.value[0]
        speed = self.LOCO_SPEED_MAP[self.loco_speed][0]
        cmd |= speed

        # Publishing direction & speed
        self.publish_msg(cmd.to_bytes(1, 'little'))

        # Setting most recent direction to backward
        self.loco_direction = self.loco_backward

    def loco_left(self):
        '''
        Method to move robot left
        :return: None
        '''
        # Getting left cmd & speed value
        cmd = ActionMap.LOCO_LEFT.value[0]
        speed = self.LOCO_SPEED_MAP[self.loco_speed][0]
        cmd |= speed

        # Publishing direction & speed
        self.publish_msg(cmd.to_bytes(1, 'little'))

        # Setting most recent direction to forward
        self.loco_direction = self.loco_left

    def loco_right(self):
        '''
        Method to move robot right
        :return: None
        '''
        # Getting left cmd & speed value
        cmd = ActionMap.LOCO_RIGHT.value[0]
        speed = self.LOCO_SPEED_MAP[self.loco_speed][0]
        cmd |= speed

        # Publishing direction & speed
        self.publish_msg(cmd.to_bytes(1, 'little'))

        # Setting most recent direction to forward
        self.loco_direction = self.loco_right

    def loco_stop(self):
        '''
        Method to stop robot
        :return: None
        '''
        
        # Getting stop cmd
        cmd = ActionMap.LOCO_STOP.value
        self.publish_msg(cmd)

        # Setting most recent direction to None
        self.loco_direction = None

        # Setting speed to default
        self.loco_speed = self.DEFAULT_LOCO_SPEED

    def increase_loco_speed(self):
        '''
        Method to increase robot speed
        :return: None
        '''
        # Adjust speed if robot moving
        if self.loco_direction:
            # Add 25% to speed
            self.loco_speed += 25

            # Check if speed is valid
            self.loco_speed = self.clamp(
                self.loco_speed, self.MIN_LOCO_SPEED, self.MAX_LOCO_SPEED)

            # Publish new speed
            self.loco_direction()

    def decrease_loco_speed(self):
        '''
        Method to decrease robot speed
        :return: None
        '''
        # Adjust speed if robot moving
        if self.loco_direction:
            # Subtract 25% to speed
            self.loco_speed -= 25

            # Check if speed is valid
            self.loco_speed = self.clamp(
                self.loco_speed, self.MIN_LOCO_SPEED, self.MAX_LOCO_SPEED)

            # Publish new speed
            self.loco_direction()

    def belt_collect(self):
        '''
        Method to start collecting
        :return: None
        '''
        # Getting dig cmd & speed
        cmd = ActionMap.BELT_COLLECT.value[0]

        # Publish direction & speed
        self.publish_msg(cmd.to_bytes(1, 'little'))

    def belt_dump(self):
        '''
        Method to start dumping
        :return: None
        '''
        # Getting collect cmd & speed
        cmd = ActionMap.BELT_DUMP.value[0]

        # Publish direction & speed
        self.publish_msg(cmd.to_bytes(1, 'little'))

    def belt_stop(self):
        '''
        Method to stop collecting/dumping
        :return: None
        '''
        # Getting collect cmd & speed
        cmd = ActionMap.BELT_STOP.value[0]

        # Publish direction & speed
        self.publish_msg(cmd.to_bytes(1, 'little'))

    def pivot_position_loco(self):
        '''
        Method to pivot to loco position
        :return: None
        '''
        cmd = ActionMap.PIVOT_TO_LOCO.value
        self.publish_msg(cmd)

    def pivot_position_const(self):
        '''
        Method to pivot to depo position
        :return: None
        '''
        cmd = ActionMap.PIVOT_TO_CONST.value
        self.publish_msg(cmd)

    def pivot_position_exca(self):
        '''
        Method to pivot to exca postiion
        :return: None
        '''
        cmd = ActionMap.PIVOT_TO_EXCA.value
        self.publish_msg(cmd)

    def pivot_zero(self):
        '''
        Method to zero pivot
        :return: None
        '''
        cmd = ActionMap.PIVOT_ZERO.value
        self.publish_msg(cmd)

    def pivot_stop(self):
        '''
        Method to stop pivot
        :return: None
        '''
        cmd = ActionMap.PIVOT_STOP.value
        self.publish_msg(cmd)

    def pivot_belt_out(self):
        '''
        Method to pivot up
        :return: None
        '''
        cmd = ActionMap.PIVOT_BELT_OUT.value
        self.publish_msg(cmd)

    def pivot_belt_in(self):
        '''
        Method to pivot down
        :return: None
        '''
        cmd = ActionMap.PIVOT_BELT_IN.value
        self.publish_msg(cmd)

    def pivot_depo_stop(self):
        '''
        Method to stop depo pivot
        :return: None
        '''
        cmd = ActionMap.PIVOT_DEPO_STOP.value
        self.publish_msg(cmd)

    def pivot_depo_up(self):
        '''
        Method to pivot depo bin up
        :return: None
        '''
        cmd = ActionMap.PIVOT_DEPO_UP.value
        self.publish_msg(cmd)

    def hinge_up(self):
        '''
        Method to mode the hinge up
        :return: None
        '''
        cmd = ActionMap.HINGE_UP.value
        self.publish_msg(cmd)

    def hinge_down(self):
        '''
        Method to mode the hinge down
        :return: None
        '''
        cmd = ActionMap.HINGE_DOWN.value
        self.publish_msg(cmd)

    def pivot_depo_down(self):
        '''
        Method to pivot depo bin down
        :return: None
        '''
        cmd = ActionMap.PIVOT_DEPO_DOWN.value
        self.publish_msg(cmd)

    def request_sensor_data(self):
        '''
        Method to request sensor data
        :return: None
        '''
        cmd = ActionMap.REQUEST_DATA.value
        self.publish_msg(cmd)

    def start_auto(self):
        '''
        Method to start loco auto
        :return: None
        '''
        cmd = ActionMap.START_AUTO.value
        self.publish_msg(cmd)

    def emergency_stop(self):
        '''
        Method to e stop
        :return: None
        '''
        cmd = ActionMap.EMERGENCY_STOP.value
        self.publish_msg(cmd)

    def start_excavation(self):
        '''
        Method to start excavation autonomy
        :return: None
        '''
        self.excavation_future = self.excavation_cli.call_async(self.excavation_req)
        rclpy.spin_until_future_complete(self, self.excavation_future)

    def start_construction(self):
        '''
        Method to start construction autonomy
        :return: None
        '''
        self.construction_future = self.construction_cli.call_async(self.construction_req)
        rclpy.spin_until_future_complete(self, self.construction_future)