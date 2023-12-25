# TODO: remove Modbus dependency, can be replaced with digital signals or
# any other trigger

import os
import sys
import getopt
import time

from firmware_uploader import FirmwareUploader, FirmwareUploadMode
from serial_communication import SerialCommunication

from pymodbus.client import ModbusSerialClient as ModbusClient


class Modbus(object):
    def __init__(self):
        self.instrument = None
        self.address = 0
        self.old_bits = []
        self.connected = False

    def open(self, port, address):
        self.instrument = ModbusClient(
            method='rtu',
            port=port,
            baudrate=38400,
            timeout=0.05
        )
        self.address = address
        ret = self.instrument.connect()
        if ret:
            return 1
        else:
            return 0

    def close(self):
        self.instrument.close()

    def set_OTA_restart_broadcast(self):
        self.instrument.write_register(12, 255, unit=0x00)


def parse_arguments(argv):
    try:
        opts, args = getopt.getopt(
            argv, "hi:f:m:s:", [
                "iport=", "ifirmware=", "imode=", "ichunk_size="])
    except getopt.GetoptError:
        print('Usage: update.py -i <input port> -f <firmware.bin>')
        sys.exit(2)

    i_port, i_firmware, i_mode, i_chunk_size = '', '', '', 512
    for opt, arg in opts:
        if opt == '-h':
            print('Usage: update.py -i <input port> -f <firmware.bin>')
            sys.exit()
        elif opt in ("-i", "--iport"):
            i_port = arg
        elif opt in ("-f", "--ifirmware"):
            i_firmware = arg
        elif opt in ("-m", "--imode"):
            i_mode = arg
        elif opt in ("-s", "--ichunk_size="):
            i_chunk_size = int(arg)

    return i_port, i_firmware, i_mode, i_chunk_size


def main(argv):
    i_port, i_firmware, i_mode, i_chunk_size = parse_arguments(argv)

    if not i_port or not i_firmware:
        print('Missing arguments. Usage: update.py -i <input port> -f <firmware.bin>')
        sys.exit()

    if not os.path.exists(i_firmware):
        print("Firmware file doesn't exist, exit...")
        sys.exit()

    if i_chunk_size < 512 or i_chunk_size > 16384:
        i_chunk_size = 512

    # This callback is used to restart the device in flash mode
    # in this case the implementation is specific to the modbus protocol used in the project
    # it should be replaced with a callback that restarts the device in flash
    # mode
    def specific_restart_callback():
        modbus = Modbus()
        modbus.open(i_port, 0)
        modbus.set_OTA_restart_broadcast()
        modbus.close()
        time.sleep(5)

    firmware_uploader = FirmwareUploader(
        SerialCommunication(i_port),
        i_firmware,
        i_chunk_size,
        specific_restart_callback
    )

    if i_mode == '--force':
        firmware_uploader.set_upload_mode(FirmwareUploadMode.FORCE_MODE)

    firmware_uploader.upload_firmware()


if __name__ == "__main__":
    main(sys.argv[1:])
