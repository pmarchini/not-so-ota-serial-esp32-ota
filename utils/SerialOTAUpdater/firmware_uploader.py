import time
import os
from serial_communication import SerialCommunication
from communication_packets import serial_ota_communication_header, packet

class FirmwareUploadMode():
    STANDARD = 0x0
    FORCE_MODE = 0x1

class FirmwareUploader:
    MAX_RESEND = 10

    def __init__(
            self,
            serial_communication: SerialCommunication,
            firmware_path,
            chunk_size=512,
            restart_callback=lambda: None,
            mode=FirmwareUploadMode.STANDARD,
        ):
        self.serial_communication = serial_communication
        self.firmware_path = firmware_path
        self.chunk_size = chunk_size
        self.restart_callback = restart_callback
        self.mode = mode

    def upload_firmware(self):
        if not os.path.exists(self.firmware_path):
            raise FileNotFoundError("Firmware file does not exist")

        self.serial_communication.open()
        time.sleep(1)
        self.send_restart_signal()
        self.send_pre_sync_header()
        self.send_sync_header()
        time.sleep(1)
        starting_time = time.time()
        self.upload_chunks() # contains closing packet
        ending_time = time.time()
        print("Firmware transfer completed in {} seconds".format(ending_time - starting_time))
        self.serial_communication.close()

    def set_upload_mode(self, mode):
        self.mode = mode

    def send_pre_sync_header(self):
        pre_sync_header = serial_ota_communication_header(magic_word=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
        pre_sync_header.create_packet()
        for i in range(0, 3):
            self.serial_communication.write(pre_sync_header.packet_header)
            time.sleep(1)

    def send_sync_header(self):
        ota_header = serial_ota_communication_header()
        ota_header.set_chunk_size(self.chunk_size)
        ota_header.set_update_mode(self.mode)
        ota_header.create_packet()
        self.serial_communication.write(ota_header.packet_header)
    
    def send_restart_signal(self):
        self.restart_callback()

    def upload_chunks(self):
        with open(self.firmware_path, "rb") as file:
            packet_id = 0
            total_len = 0
            is_partition_open = False
            while True:
                data = file.read(self.chunk_size)
                # TODO: I presume this should be here, but it's not in the original code 
                # Should be tested manually before uncommenting
                #if not data: 
                #    break

                packet_to_send = packet(packet_id)
                packet_to_send.set_data(data)
                packet_to_send.create_packet()
                self.send_packet_with_retry(packet_to_send)
                
                total_len += len(data)
                packet_id += 1

                if(total_len >= 512 and not is_partition_open):
                    is_partition_open = True
                    print("Head sent, waiting 10s for esp to open partition")
                    time.sleep(10)
                
                if len(packet_to_send.packet_data) < self.chunk_size:
                    break

                time.sleep(0.0001)

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
        print("Waiting before sending closing packet")
        time.sleep(10)
        print("Sending closing packet")
        closing_packet = packet(packet_id)
        closing_packet.set_data([])
        closing_packet.packet_data_len = 0
        closing_packet.create_packet()
        self.serial_communication.write(closing_packet.packet)
