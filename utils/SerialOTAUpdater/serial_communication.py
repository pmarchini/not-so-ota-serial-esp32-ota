import serial
from serial.serialutil import PARITY_NONE

class SerialCommunication:
    def __init__(self, port, baudrate=115200, timeout=0.07):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def open(self):
        self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.serial.parity = PARITY_NONE

    def close(self):
        if self.serial:
            self.serial.close()

    def write(self, data):
        if self.serial:
            self.serial.write(data)

    def read(self):
        if self.serial:
            return self.serial.read()
        return None