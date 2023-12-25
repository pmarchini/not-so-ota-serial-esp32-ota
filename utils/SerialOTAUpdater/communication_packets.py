import crc8
from struct import pack

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
        self.packet_header += pack("=H",self.ota_chunk_size)
        self.packet_header += self.header_crc

    def set_chunk_size(self,chunk_size):
        self.ota_chunk_size = int(chunk_size)
        #print("Chunk size set to ",self.ota_chunk_size)

    def set_update_mode(self,mode):
        self.ota_update_mode = int(mode)
        #print("Update mode set to ",self.ota_update_mode)
    

    def calc_crc8(self):
        t_var = []
        t_var = self.magic_word.copy()
        t_var.append(self.ota_update_mode)
        t_var = bytearray(t_var)
        t_var += pack("=H",self.ota_chunk_size)
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
