# test_update.py
from unittest.mock import patch, MagicMock, mock_open
import update
import sys
from io import StringIO
import pytest

@pytest.fixture
def mock_serial():
    with patch('update.serial.Serial') as mock:
        yield mock

@pytest.fixture
def mock_modbus_client():
    with patch('update.ModbusClient') as mock:
        yield mock

@pytest.fixture
def mock_exists():
    with patch('update.os.path.exists', return_value=True) as mock:
        yield mock

def test_main_happy_path_with_snapshot(snapshot, mock_exists, mock_modbus_client, mock_serial):
    mock_modbus_instance = MagicMock()
    mock_modbus_client.return_value = mock_modbus_instance
    mock_modbus_instance.connect.return_value = True
    mock_modbus_instance.write_register.return_value = None
    mock_modbus_instance.close.return_value = None

    written_data = []

    def record_write(data):
        written_data.append(data)

    mock_serial_instance = MagicMock()
    mock_serial.return_value = mock_serial_instance
    mock_serial_instance.write.side_effect = record_write
    mock_serial_instance.read.return_value = b''

    mock_1024_byte_firmware_file = b'0' * 1024
    mock_file = mock_open(read_data=mock_1024_byte_firmware_file)

    captured_output = StringIO()
    sys.stdout = captured_output

    test_args = ["script_name", "-i", "COM1", "-f", "firmware.bin"]
    with patch.object(sys, 'argv', test_args):
        with patch('builtins.open', mock_file):
            update.main(sys.argv[1:])

    sys.stdout = sys.__stdout__

    assert "Firmware transfer completed" in captured_output.getvalue()

    # Convert bytearrays to a hex string representation
    serial_output_str = '\n'.join([bytes(data).hex() for data in written_data])
    # Snapshot testing for serial output
    snapshot.assert_match(serial_output_str, 'serial_output_snapshot.txt')