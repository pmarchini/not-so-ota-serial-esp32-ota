# TODO: remove Modbus dependency, can be replaced with digital signals or any other trigger 

import array
import os
import struct
import serial
import sys
import time
from serial.serialutil import PARITY_NONE, Timeout
import getopt
import crc8

from pymodbus.client import ModbusSerialClient as ModbusClient

from struct import *

class Modbus(object):
    def __init__(self):
        self.instrument = None
        self.address = 0
        self.old_bits = []
        self.connected = False

    def open(self, port, address):
        self.instrument = ModbusClient(
            method='rtu', port=port, baudrate=38400, timeout=0.05)
        self.address = address
        ret = self.instrument.connect()
        # self.instrument = minimalmodbus.Instrument(port, address)  # port name, slave address (in decimal)
        # self.instrument.serial.baudrate = 38400
        if ret:
            return 1
        else:
            return 0

    def close(self):
        self.instrument.close()

    def set_OTA_restart_broadcast(self):
        self.instrument.write_register(12, 255, unit=0x00)

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


class serial_ota_communication_header:
    magic_word = []
    header_crc = 0x0
    ota_update_mode = 0x0
    ota_chunk_size = 0
    packet_header = []

    def __init__(self, magic_word=[0xC0, 0xFF, 0xFE, 0xAA, 0x55, 0x90]):
        self.magic_word = magic_word
        self.ota_update_mode = 0x0
        self.ota_chunk_size = 512
        self.header_crc = 0x0

    def pack_header(self):
        self.packet_header = self.magic_word.copy()
        self.packet_header.append(self.ota_update_mode)
        self.packet_header = bytearray(self.packet_header)
        self.packet_header += struct.pack("=H",self.ota_chunk_size)
        self.packet_header += self.header_crc

    def set_chunk_size(self,chunk_size):
        self.ota_chunk_size = int(chunk_size)
        print("Chunk size set to ",self.ota_chunk_size)
    

    def calc_crc8(self):
        t_var = []
        t_var = self.magic_word.copy()
        t_var.append(self.ota_update_mode)
        t_var = bytearray(t_var)
        t_var += struct.pack("=H",self.ota_chunk_size)
        if(t_var != []):
            hash = crc8.crc8()
            hash.update(t_var)
            self.header_crc = hash.digest()
        else:
            self.header_crc = b'\x00'

    def create_packet(self):
        self.calc_crc8()
        self.pack_header()


class packet:
    def __init__(self, id=0):
        self.packet_id = id
        self.packet_crc = 0
        self.packet_data_len = 0
        self.packet_data = []
        self.packet_header = []
        self.packet = []

    def set_data(self, data):
        self.packet_data = data

    def set_id(self, id):
        self.id = id

    def calcCRC8(self):
        if(self.packet_data != []):
            hash = crc8.crc8()
            hash.update(self.packet_data)
            self.packet_crc = hash.digest()
        else:
            self.packet_crc = b'\x00'

    def calculate_packet_data_len(self):
        self.packet_data_len = len(self.packet_data)

    def pack_header(self):
        self.packet_header = pack('=HBH',
                                  self.packet_id,
                                  int.from_bytes(
                                      self.packet_crc,
                                      byteorder='big',
                                      signed=False),
                                  self.packet_data_len)


    def create_packet(self):
        self.calcCRC8()
        self.calculate_packet_data_len()
        self.pack_header()
        if(self.packet_data != []):
            self.packet = self.packet_header + self.packet_data
        else:
            self.packet = self.packet_header

class FirmwareUploader:
    MAX_RESEND = 10

    def __init__(
            self,
            serial_communication,
            firmware_path,
            chunk_size=512,
            restart_callback=lambda: None
        ):
        self.serial_communication = serial_communication
        self.firmware_path = firmware_path
        self.chunk_size = chunk_size
        self.restart_callback = restart_callback

    def upload_firmware(self):
        if not os.path.exists(self.firmware_path):
            raise FileNotFoundError("Firmware file does not exist")

        self.serial_communication.open()
        self.send_restart_signal()
        self.upload_chunks()
        self.serial_communication.close()

    def send_restart_signal(self):
        self.restart_callback()

    def upload_chunks(self):
        with open(self.firmware_path, "rb") as file:
            packet_id = 0
            total_len = 0
            while True:
                data = file.read(self.chunk_size)
                if not data:
                    break

                packet = packet(packet_id)
                packet.set_data(data)
                packet.create_packet()
                self.send_packet_with_retry(packet)
                
                total_len += len(data)
                packet_id += 1
                time.sleep(0.1)

            self.send_closing_packet(packet_id)

    def send_packet_with_retry(self, packet):
        retries = 0
        while retries < self.MAX_RESEND:
            self.serial_communication.write(packet.packet)
            if self.serial_communication.read() == b'':
                break
            retries += 1
            time.sleep(0.5)

    def send_closing_packet(self, packet_id):
        closing_packet = packet(packet_id)
        closing_packet.set_data([])
        closing_packet.packet_data_len = 0
        closing_packet.create_packet()
        self.serial_communication.write(closing_packet.packet)

def main(argv):

    i_port = ''
    i_firmware = ''
    i_mode = ''
    i_chunk_size = 512
    max_resend = 10

    try:
        opts, args = getopt.getopt(
            argv, "hi:f:m:s:", ["iport=", "ifirmware=", "imode=", "ichunk_size="])
    except getopt.GetoptError:
        print('update.py -i <input port> -f <firmware.bin>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('update.py -i <input port> -f <firmware.bin>')
            sys.exit()
        elif opt in ("-i", "--iport"):
            i_port = arg
        elif opt in ("-f", "--ifirmware"):
            i_firmware = arg
        elif opt in ("-m", "--imode"):
            i_mode = arg
        elif opt in ("-s", "--ichunk_size="):
            i_chunk_size = int(arg)
        
    print('Input port is', i_port)
    print('Input file is', i_firmware)

    if (i_port == '') or (i_firmware == ''):
        print('update.py -i <input port> -f <firmware.bin>')
        sys.exit()

    if(not os.path.exists(i_firmware)):
        print("Firmware file doesn't exist, exit...")
        sys.exit()

    if(i_chunk_size < 512 or i_chunk_size > 16384):
        i_chunk_size = 512

    #Send restart message via Modbus
    modbus = Modbus()
    modbus.open(i_port,0)
    modbus.set_OTA_restart_broadcast()
    modbus.close()
    time.sleep(5)

    s = serial.Serial(i_port, 115200)
    s.parity = PARITY_NONE
    s.baudrate = 115200
    s.timeout = 0.07
    time.sleep(1)
    # PreSync message
    preSyncHeader = serial_ota_communication_header(magic_word=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    preSyncHeader.create_packet()
    # Header ota message
    otaHeader = serial_ota_communication_header()
    if(i_mode == '--force'):
        otaHeader.ota_update_mode = 0x1
    
    otaHeader.set_chunk_size(i_chunk_size)
    otaHeader.create_packet()
    # sending pre sync messages
    for i in range(0, 3):
        s.write(preSyncHeader.packet_header)
        time.sleep(1)
    # sending sync message
    s.write(otaHeader.packet_header)

    time.sleep(1)

    starting_time = time.time()

    with open(i_firmware, "rb") as file:
        t_len = 0
        f_wait = True
        packet_id = 0
        resend_flag = 0
        t_resend = 0
        while True:
            t_pack = packet(id=packet_id)
            t_pack.set_data(file.read(int(i_chunk_size)))
            t_pack.calculate_packet_data_len()
            t_len += t_pack.packet_data_len
            t_pack.create_packet()
            resend_flag = 1
            while resend_flag:
                if(t_resend > max_resend):
                    break
                s.write(t_pack.packet)
                if(s.read() == b''):
                    t_resend = 0
                    resend_flag = 0
                else:
                    t_resend += 1
                    time.sleep(0.5)

            print("sent size : ", t_len)
            packet_id += 1

            if(t_len >= 512 and f_wait):
                print("Head sent, waiting 10s for esp to open partition")
                f_wait = False
                time.sleep(10)

            if len(t_pack.packet_data) < i_chunk_size:
                break
            time.sleep(0.0001)

        print("Waiting before sending closing packet")
        time.sleep(10)
        print("Sending closing packet")
        # sending closing packet
        t_pack = packet(id=packet_id)
        t_pack.set_data([])
        t_pack.packet_data_len = 0
        t_pack.create_packet()
        s.write(t_pack.packet)

        ending_time = time.time()
        print("Firmware transfer completed in : ", int(ending_time-starting_time),"s")


if __name__ == "__main__":
    main(sys.argv[1:])