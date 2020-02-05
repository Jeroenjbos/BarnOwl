
    # uint8_t type;          // 1 B, packet type
    # uint8_t size;          // 1 B, packet size
    # uint16_t crc16;        // 2 B, CRC16
    # unsigned long packetID;// 4 B, running packet count

    # unsigned long ts_start;// 4 B, gather start timestamp
    # unsigned long ts_end;  // 4 B, transmit timestamp
    # long states[8];        // 16 B, state variables (encoder, speed, etc)

    # uint16_t digitalIn;    // 2 B, digital inputs
    # uint8_t digitalOut;    // 1 B, digital outputs
    # uint8_t padding[1];    // 1 B, align to 4B

import logging
import struct
from collections import namedtuple

DataPacketDesc = {'type': 'B',
                  'size': 'B',
                  'crc16': 'H',
                  'packetID': 'I',
                  'us_start': 'I',
                  'us_end': 'I',
                  'states': '8l',
                  'digitalIn': 'H',
                  'digitalOut': 'B',
                  'padding': 'x'}

DataPacket = namedtuple('DataPacket', DataPacketDesc.keys())
DataPacketStruct = '<' + ''.join(DataPacketDesc.values())
DataPacketSize = struct.calcsize(DataPacketStruct)
logging.info(f"Packet size: {DataPacketSize} Bytes in {DataPacketStruct}")

CommandPacket = {'type': 'B',
                 'command': 'B',
                 'data': 'b'}

CommandPacketStruct = '<BBbx'