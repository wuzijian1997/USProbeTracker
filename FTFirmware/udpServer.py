# This example code is in the Public Domain (or CC0 licensed, at your option.)

# Unless required by applicable law or agreed to in writing, this
# software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied.

# -*- coding: utf-8 -*-

from __future__ import print_function, unicode_literals

import os
import re
import socket
import sys
import time
import numpy as np
from builtins import input
from threading import Event, Thread

# -----------  Config  ----------
PORT = 5004
NSENSORS = 6
IMU_ACC_SCALE = 4
IMU_GYR_SCALE = 1000
TEMP_SENS = 256
TEMP_BIAS = 25.0
# -------------------------------

msgLen = (NSENSORS+8)*2

class UdpServer:

    def __init__(self, port, family_addr, persist=False):
        self.port = port
        self.family_addr = family_addr
        self.socket = socket.socket(family_addr, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.settimeout(60.0)
        self.shutdown = Event()
        self.persist = persist
        self.count = 0

    def __enter__(self):
        try:
            self.socket.bind(('', self.port))
        except socket.error as e:
            print('Bind failed:{}'.format(e))
            raise

        print('Starting server on port={} family_addr={}'.format(self.port, self.family_addr))
        self.server_thread = Thread(target=self.run_server)
        self.server_thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.persist:
            sock = socket.socket(self.family_addr, socket.SOCK_DGRAM)
            sock.sendto(b'Stop', ('localhost', self.port))
            sock.close()
            self.shutdown.set()
        self.server_thread.join()
        self.socket.close()

    def int2float(self, num,bits,fullscale):
        return num * fullscale / 2**bits

    def bytes2float(self, b1,b2,idx):
        # Combine MSB and LSB
        num = (b1<<8) + b2

        # Check for sign bit
        if num & 0x8000 != 0:
            # Number is negative 2's complement
            num = -(((~num)+1)&0xFFFF)
        
        # Convert to float
        if idx < NSENSORS:
            # This is a sensor reading
            return self.int2float(num,12,2)
        else:
            # This is an IMU reading
            if idx == NSENSORS:
                # Temperature
                return num / TEMP_SENS + TEMP_BIAS
            elif idx >= NSENSORS+4:
                # Acceleration
                return self.int2float(num,15,IMU_ACC_SCALE)
            else:
                # Gyroscope
                return self.int2float(num,15,IMU_GYR_SCALE)

    def handleSample(self, data):
        if len(data) == (NSENSORS+8)*2:
            out = np.zeros(NSENSORS+7)
            for i in range(NSENSORS+7):
                out[i] = self.bytes2float( data[2*i+2], data[2*i+1+2], i)
            return out
        else:
            print("Incorrect packet length")
            print(data)

    def run_server(self):
        while not self.shutdown.is_set():
            try:
                dataRaw, addr = self.socket.recvfrom(msgLen)
                if not dataRaw:
                    return
                print(str(time.time()))
                #for i in range(len(dataRaw)):
                #    print(str(dataRaw[i]) + ', ', end='', flush=True)
                #print('\n')

                if (dataRaw == b'Stop'):
                    break
                data = self.handleSample(dataRaw)
                # print(str(self.count) + ':\nSensor: [{},{},{},{},{},{}]\nTemperature: {}\nGyroscope: [{},{},{}]\nAcceleration: [{},{},{}]'.format(*data))
                # print('\t{}\t{}\t{}\t{}\t{}\t{}'.format(*data))


                # data = data.decode()
                # print('Reply[' + addr[0] + ':' + str(addr[1]) + '] - ' + data)
                reply = 'Received message'
                self.socket.sendto(reply.encode(), addr)

                self.count += 1
            except socket.error as e:
                print('Running server failed:{}'.format(e))
                raise
            if not self.persist:
                break

if __name__ == '__main__':
    family_addr = socket.AF_INET
    with UdpServer(PORT, family_addr, persist=True) as s:
        print(input('Press Enter stop the server...'))
