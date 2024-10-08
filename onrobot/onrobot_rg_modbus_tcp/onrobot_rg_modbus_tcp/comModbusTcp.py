#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
import rclpy
import threading
from pymodbus.client.sync import ModbusTcpClient
#from pymodbus.client.sync import ModbusSerialClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
import time

class communication:

    def __init__(self, dummy=False):
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()
        self.logger = None

    def connectToDevice(self, ip: str, port: str, changer_addr: int = 65) -> bool:
        """Connects to the client.
           The method takes the IP address and port number
           (as a string, e.g. '192.168.1.1' and '502') as arguments.
        """
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return True

        self.client = ModbusTcpClient(
            host=ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.changer_addr = changer_addr
        return self.client.connect()

    def disconnectFromDevice(self):
        """Closes connection."""
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return

        self.client.close()

    def sendCommand(self, message):
        """Sends a command to the Gripper.
           The method takes a list of uint8 as an argument.
        """
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return

        # Send a command to the device (address 0 ~ 2)
        if message != []:
            with self.lock:
                self.client.write_registers(
                    address=0, values=message, unit=self.changer_addr)

    def restartPowerCycle(self):
        """Restarts the power cycle of Compute Box
           Necessary is Safety Switch of the grippers are pressed
           Writing 2 to this field powers the tool off for a short amount of time and then powers them back
        """
        message = 2
        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
                self.client.write_registers(
                    address=0, values=message, unit=restart_address)

    def getStatus(self):
        """Sends a request to read, wait for the response
           and returns the Gripper status.
           The method gets by specifying register address as an argument.
        """
        response = [0] * 18
        if self.dummy:
            if self.logger:
                self.logger.info(
                    sys._getframe().f_code.co_name)
            return response

        # Get status from the device (address 258 ~ 275)
        with self.lock:
            response = self.client.read_holding_registers(
                address=258, count=18, unit=self.changer_addr)
            retries = 50
            # To get around spuratic - object has no attribute 'registers'
            while not isinstance(response, ReadHoldingRegistersResponse):
                response = self.client.read_holding_registers(
                    address=258, count=18, unit=self.changer_addr)
                retries -= 1
                time.sleep(0.01)
                if retries <= 0: raise TypeError("Failed to get status after 50 tries") 
              #except TypeError as e:
              #print(e)

        # Output the result
        return response.registers
