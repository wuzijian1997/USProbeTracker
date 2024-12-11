import serial
import numpy as np
import struct
import time
import socket
import netifaces as nf

NSENSORS = 6
IMU_ACC_SCALE = 4
IMU_GYR_SCALE = 1000
TEMP_SENS = 256
TEMP_BIAS = 25.0

wifiSSID = "Rogers-5G-FWA"
wifiPass = "Rogers2022!"

# wifiSSID = "TELUS40AA"
# wifiPass = "JhAV9J3J7KhN"
SEND_MODE = 1 # 0 = UART, 1 = UDP
update = False

updateSettings = bytearray([0, 0, 1, SEND_MODE, 2, len(wifiSSID), 3, len(wifiPass), 4, 1])
dontUpdateSettings = bytearray([0, 1, 1, 2, 2, 0, 3, 0, 4, 0])

print("Initializing sensor (you may have to press RESET)")

def int2float(num,bits,fullscale):
    return num * fullscale / 2**bits

def bytes2float(b1,b2,idx):
    # Combine MSB and LSB
    num = (b1<<8) + b2

    # Check for sign bit
    if num & 0x8000 != 0:
        # Number is negative 2's complement
        num = -(((~num)+1)&0xFFFF)
    
    # Convert to float
    if idx < NSENSORS:
        # This is a sensor reading
        return int2float(num,12,2)
    else:
        # This is an IMU reading
        if idx == NSENSORS:
            # Temperature
            return num / TEMP_SENS + TEMP_BIAS
        elif idx >= NSENSORS+4:
            # Acceleration
            return int2float(num,15,IMU_ACC_SCALE)
        else:
            # Gyroscope
            return int2float(num,15,IMU_GYR_SCALE)

def addr2intarr(addr):
    return [int(addr.split('.')[i]) for i in range(4)]

def getWifiInfo():
    try:
        gwys = nf.gateways()['default'][2]
        gatewayAddr = addr2intarr(gwys[0])
        interface = gwys[1]
        ip = [int(nf.ifaddresses(interface)[2][0]['addr'].split('.')[i]) for i in range(4)]
        netmask = [int(nf.ifaddresses(interface)[2][0]['netmask'].split('.')[i]) for i in range(4)]
    except:
        print("Could not determine default gateway, subnet mask, or IP address.")
        gatewayAddr = addr2intarr(input("Please enter default gateway address in format 255.255.255.255:\n"))
        ip = addr2intarr(input("Please enter IP address in format 255.255.255.255:\n"))
        netmask = addr2intarr(input("Please enter subnet address in format 255.255.255.255:\n"))
    return ip, netmask, gatewayAddr

def sendWifiInfo(ser, ip, netmask, gwy):
    ser.write(wifiSSID.encode(encoding='UTF-8'))
    time.sleep(0.2)
    ser.write(wifiPass.encode(encoding='UTF-8'))
    time.sleep(0.2)

    ser.write(bytearray(ip))
    time.sleep(0.2)
    ser.write(bytearray(netmask))
    time.sleep(0.2)
    ser.write(bytearray(gatewayAddr))

ip, netmask, gatewayAddr = getWifiInfo()

trying = True
while trying:
    try:
        with serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1) as ser:
            while ser.is_open:
                # Read until we find a start byte
                sLast = 0
                sLastLast = 0
                while True:
                    # Read 1 byte
                    s = ser.read(1)

                    if s == b'':
                        continue

                    # Check for start sequence
                    if sLast == 0xFF and s[0] == 0xFF:
                        if sLastLast != 0x00:
                            break
                        else:
                            # Set UART/UDP mode, WiFi SSID, WiFi Password
                            if (update):
                                ser.write(updateSettings)
                                time.sleep(0.2)
                                sendWifiInfo(ser, ip, netmask, gatewayAddr)
                            else:
                                print("Current settings:")
                                ser.write(dontUpdateSettings)

                    # Print any UTF-8 encoded text
                    if s[0] < 128:
                        print(s.decode("utf-8"), end='', flush=True)
                    sLastLast = sLast
                    sLast = s[0]

                print("\n----- New packet")
                sLast = 0

                # Read a packet
                s = ser.read(((NSENSORS+7)*2))
                if len(s) == (NSENSORS+7)*2:
                    for i in range(NSENSORS+7):
                        print( bytes2float( s[2*i], s[2*i+1], i) )
                else:
                    print("Incorrect packet length")
                    print(s)
                print("----- Finished package")
        trying = False
    except serial.SerialException:
        print("Failed to connect. Trying again...")
        time.sleep(0.5)
