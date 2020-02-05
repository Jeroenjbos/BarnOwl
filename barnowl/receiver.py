import argparse

import serial
from serial.threaded import Packetizer, ReaderThread
import sys
import traceback
import time
from cobs import cobs

from websocket_server import WebsocketServer
import json
import logging
import threading
import sys

import http.server
import socketserver

import struct

import numpy as np

from SerialDummy import SerialDummy
from packet import DataPacket, DataPacketSize, DataPacketStruct, CommandPacketStruct
import base64

LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)

WS_PORT = 5678
HTTP_PORT = 8000
USE_DUMMY = True
SERIAL_PORT = 'COM6'


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

def bitlist(num, nbits=16):
    """Integer to list of bits"""
    # TODO: pad zeros with string formatter
    bl = list(map(int, bin(num)[2:]))
    # pad with leading zeros
    bl = [0]*(nbits-len(bl)) + bl
    return bl


class PacketReceiver(serial.threaded.Packetizer):
    commander = None
    log_file = None

    def connection_made(self , transport):
        super(PacketReceiver, self).connection_made(transport)
        self.log_file = open('log_file.dat', 'w+b')

    def handle_packet(self, arr):
        """Handle an incoming packet from the serial port. The packetizer has stripped the 
        line-termination \0 byte from it already.
        """
        # write data immediately to the log file, no matter what. For future parsing, we need to
        # re-append the line termination symbol.
        self.log_file.write(base64.b64encode(arr + b'\0')+b'\n')

        # COBS decode the array
        if not len(arr):
            return
        try:
            dec = cobs.decode(arr)
        except cobs.DecodeError as e:
            logging.warning(str(e))
            return
        
        packet_type = dec[0]
        if packet_type == 0:
            self.handle_data_packet(dec)

        elif packet_type == 1:
            self.handle_command_packet(dec)

        elif packet_type == 2:
            logging.error(f'Received error packet {dec}')

        else:
            logging.error(f'Received unknown packet type: {packet_type} in packet {dec}')

    def handle_data_packet(self, arr):
        """Handle a data packet by extracting it's fields.
        """
        if len(arr) != DataPacketSize:
            logging.warning(f"Incorrect data size. Is: {len(arr)}, expected: {DataPacketSize}. Packet: {arr}")
            return

        # stupid manual struct unpacking is stupid
        s = struct.unpack(DataPacketStruct, arr)
        dp = DataPacket(type=s[0], size=s[1], crc16=s[2], packetID=s[3], us_start=s[4], us_end=s[5],
                        states=s[6:14], digitalIn=s[14], digitalOut=s[15], padding=None)
        
        # let the TeensyCommander deal with the actual packet content
        if self.commander:
            self.commander.handle_packet(dp)
        
    def handle_command_packet(self, arr):
        print("Command packet: ", arr)

    def connection_lost(self, exc):
        self.log_file.close()
        if exc:
            print('Serial connection loss: ', exc)
            traceback.print_exc()


class BarnOwlCommunicator:
    def __init__(self, websocket_port, serial_port):
        self.n_packet = 0
        try:
            self.ser = serial.Serial(serial_port)
        except serial.SerialException as e:
            logging.error("Can't find teensy: {}".format(e))
            if USE_DUMMY:
                logging.warning('Using serial dummy')
                self.dummy = SerialDummy()
                self.ser = self.dummy.ser
            else:
                exit()
        self.serial_reader = ReaderThread(self.ser, PacketReceiver).__enter__()
        self.serial_reader.commander = self

        # Websocket server
        self.websocket_server = WebsocketServer(websocket_port)
        self.websocket_server.set_fn_message_received(self.ws_msg_rcv)
        self.websocket_server.set_fn_new_client(self.ws_client_connect)
        self.websocket_server.set_fn_client_left(self.ws_client_left)
        self.websocket_thread = threading.Thread(target=self.websocket_server.run_forever)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()

    def ws_client_connect(self, client, server):
        # server.send_message_to_all(f'New client connected: {client} to server {server}')
        logging.info('New client connected to WebSocket.')

    def ws_client_left(self, client, server):
        logging.info(f'Client disconnected from WebSocket.')

    def ws_msg_rcv(self, client, server, message):
        # TODO: Match pinout, which pins are input, which output
        if message.startswith('digital'):
            pin = int(message.split('digital')[1]) - 16
            print(f'toggling pin {pin}')
            cmd_p = struct.pack(CommandPacketStruct, *[1, 1, 7-pin])
            enc = cobs.encode(cmd_p)
            self.ser.write(enc + b'\0')
        else:
            delta = int(float(message)*127)
            if abs(delta) > 30:
                cmd_p = struct.pack(CommandPacketStruct, *[1, 1, delta])
                enc = cobs.encode(cmd_p)
                self.ser.write(enc + b'\0')

    def run_forever(self):
        while True: 
            time.sleep(1)

    def handle_packet(self, packet):
        self.n_packet += 1
        js = json.dumps({'us_start': packet.us_start, 'us_end': packet.us_end, 'states': packet.states,
                'digitalIn': bitlist(packet.digitalIn, nbits=16),
                'digitalOut': bitlist(packet.digitalOut, nbits=8)},
                cls=NumpyEncoder)

        if self.websocket_server:
            if not self.n_packet % 100:
                print('Packet: ', packet)
            if not self.n_packet % 5:
                self.websocket_server.send_message_to_all(js)


if __name__ == "__main__":
    # TODO: reconnecting serial connection
    parser = argparse.ArgumentParser()
    parser.add_argument('--http_port', help='HTTP server port', default=HTTP_PORT)
    parser.add_argument('--ws_port', help='WebSocket port', default=WS_PORT)
    parser.add_argument('--serial_port', help='Serial port', default=SERIAL_PORT)
    parser.add_argument('-v', '--verbose', help='Log debug messages', action='store_true')

    cli_args = parser.parse_args()

    log_level = logging.DEBUG if cli_args.verbose else logging.INFO
    logging.basicConfig(level=log_level,
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    Handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("127.0.0.1", cli_args.http_port), Handler) as httpd:
        logging.info(f"HTTP server at port {cli_args.http_port}")
        hst = threading.Thread(target=httpd.serve_forever)
        hst.daemon = True
        hst.start()

        tc = BarnOwlCommunicator(websocket_port=cli_args.ws_port, serial_port=cli_args.serial_port)
        tc.run_forever()
