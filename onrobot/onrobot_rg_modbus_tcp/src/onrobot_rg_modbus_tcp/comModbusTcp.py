#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
from pymodbus.client.sync import ModbusTcpClient


class Communication(Node):
    """Communication sends commands and receives the status of RG gripper."""

    def __init__(self, dummy=False):
        super().__init__('communication')
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()
        self.changer_addr = 65

    def connect_to_device(self, ip, port):
        """Connects to the client device (gripper)."""
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return

        self.client = ModbusTcpClient(ip, port=port, stopbits=1, bytesize=8, parity='E', baudrate=115200, timeout=1)
        self.client.connect()

    def disconnect_from_device(self):
        """Closes connection."""
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return

        self.client.close()

    def send_command(self, message):
        """Sends a command to the Gripper."""
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return

        # Sending a command to the device (address 0 ~ 2)
        if message!= []:
            with self.lock:
                self.client.write_registers(address=0, values=message, unit=self.changer_addr)

    def restart_power_cycle(self):
        """Restarts the power cycle of Compute Box."""
        message = 2
        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
            self.client.write_registers(address=0, values=message, unit=restart_address)

    def get_status(self):
        """Sends a request to read and returns the gripper status."""
        response = [0] * 18
        if self.dummy:
            self.get_logger().info(f"{self.get_name()}: {sys._getframe().f_code.co_name}")
            return response

        # Getting status from the device (address 258 ~ 275)
        with self.lock:
            response = self.client.read_holding_registers(address=258, count=18, unit=self.changer_addr).registers

        # Output the result
        return response


def main(args=None):
    rclpy.init(args=args)
    communication_node = Communication()
    try:
        rclpy.spin(communication_node)
    except KeyboardInterrupt:
        pass
    finally:
        communication_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
