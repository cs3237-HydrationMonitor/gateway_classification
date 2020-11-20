# -*- coding: utf-8 -*-
"""
TI CC2650 SensorTag
-------------------

Adapted by Ashwin from the following sources:
 - https://github.com/IanHarvey/bluepy/blob/a7f5db1a31dba50f77454e036b5ee05c3b7e2d6e/bluepy/sensortag.py
 - https://github.com/hbldh/bleak/blob/develop/examples/sensortag.py

"""
import asyncio
import platform
import struct
import time
import json
from json import JSONEncoder
import requests
import pickle
import os
import sys
from bleak import BleakClient

import paho.mqtt.client as mqtt
import numpy as np
import time

global accelDataPoints_10
global gyroDataPoints_10
global awsUri
global sensorMac
global expectedOutput

global gyroQueue
global accelQueue

global gyroCounter
global accelCounter
global trigger
global previousGyroSum


global client


class Service:
    """
    Here is a good documentation about the concepts in ble;
    https://learn.adafruit.com/introduction-to-bluetooth-low-energy/gatt

    In TI SensorTag there is a control characteristic and a data characteristic which define a service or sensor
    like the Light Sensor, Humidity Sensor etc

    Please take a look at the official TI user guide as well at
    https://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide
    """

    def __init__(self):
        self.data_uuid = None
        self.ctrl_uuid = None


class Sensor(Service):

    def callback(self, sender: int, data: bytearray):
        raise NotImplementedError()

    async def start_listener(self, client, *args):
        # start the sensor on the device
        write_value = bytearray([0x01])
        await client.write_gatt_char(self.ctrl_uuid, write_value)

        # listen using the handler
        await client.start_notify(self.data_uuid, self.callback)


class MovementSensorMPU9250SubService:

    def __init__(self):
        self.bits = 0

    def enable_bits(self):
        return self.bits

    def cb_sensor(self, data):
        raise NotImplementedError

class MovementSensorMPU9250(Sensor):
    GYRO_XYZ = 7
    ACCEL_XYZ = 7 << 3
    MAG_XYZ = 1 << 6
    ACCEL_RANGE_2G  = 0 << 8
    ACCEL_RANGE_4G  = 1 << 8
    ACCEL_RANGE_8G  = 2 << 8
    ACCEL_RANGE_16G = 3 << 8

    def __init__(self):
        super().__init__()
        self.data_uuid = "f000aa81-0451-4000-b000-000000000000"
        self.ctrl_uuid = "f000aa82-0451-4000-b000-000000000000"
        self.ctrlBits = 0

        self.sub_callbacks = []

    def register(self, cls_obj: MovementSensorMPU9250SubService):
        self.ctrlBits |= cls_obj.enable_bits()
        self.sub_callbacks.append(cls_obj.cb_sensor)

    async def start_listener(self, client, *args):
        # start the sensor on the device
        await client.write_gatt_char(self.ctrl_uuid, struct.pack("<H", self.ctrlBits))

        # listen using the handler
        await client.start_notify(self.data_uuid, self.callback)

    def callback(self, sender: int, data: bytearray):
        unpacked_data = struct.unpack("<hhhhhhhhh", data)
        for cb in self.sub_callbacks:
            cb(unpacked_data)

# code to help format the numpy array into a json serializable format
# done to be able to json.dumps, and transfer data
class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)

#code to read and record accelerometer data
class AccelerometerSensorMovementSensorMPU9250(MovementSensorMPU9250SubService):
    def __init__(self):
        super().__init__()
        self.bits = MovementSensorMPU9250.ACCEL_XYZ | MovementSensorMPU9250.ACCEL_RANGE_4G
        self.scale = 8.0/32768.0 # TODO: why not 4.0, as documented? @Ashwin Need to verify

    def cb_sensor(self, data):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        global accelDataPoints_10
        global accelCounter
        global trigger

        rawVals = data[0:3]
        accelTuple = tuple([ v*self.scale for v in rawVals ])

        # to maintain a history of 3 sensor data

        # to increase length of queue to 3 sensor data
        if (len(accelQueue) < 3):
            accelQueue.append(accelTuple)
        # this executes when len(gyroQueue) == 3
        # this pops the earliest inserted element and insert latest
        else:
            accelQueue.append(accelTuple)
            accelQueue.pop(0)


        # the trigger piece of code
        # if there is a 300% increase in movement then we trigger it
        # change trigger to 1 so this code will not execute during the next 7 trials
        if (trigger == 1):
            trigger = trigger + 1
            #adding the -3 data points
            accelDataPoints_10 = list(accelQueue)
            #pops the newly inserted element, as this will be added back later
            accelDataPoints_10.pop(2)
        else:
            pass

        #making the list become -3 + 7
        #when trigger = 1, and length of list < 10
        #continue to append list till size 10
        if (trigger != 0 and len(accelDataPoints_10)<10):
            accelDataPoints_10.append(accelTuple)
            #for trigger = 0, is triggered in the gyroscope code below
        else:
            pass

        accelCounter=accelCounter+1
        print("[MovementSensor] Accelerometer:", accelTuple)

class GyroscopeSensorMovementSensorMPU9250(MovementSensorMPU9250SubService):
    def __init__(self):
        super().__init__()
        self.bits = MovementSensorMPU9250.GYRO_XYZ
        self.scale = 500.0/65536.0

    def cb_sensor(self, data):
        '''Returns (x_gyro, y_gyro, z_gyro) in units of degrees/sec'''
        global gyroDataPoints_10
        global gyroCounter
        global previousGyroSum
        global trigger

        rawVals = data[0:3]
        gyroTuple = list([ v*self.scale for v in rawVals ])

        #calculate the sum of the values
        currentGyroSum = 0
        for i in gyroTuple:
            currentGyroSum = currentGyroSum + abs(i)

        # if (previousGyroSum != 0):
        #     print("Ratio of Current/Previous : {}".format(abs(currentGyroSum/previousGyroSum)))

        # to maintain a history of 3 sensor data
        # to increase length of queue to 3 sensor data
        if (len(gyroQueue) < 3):
            gyroQueue.append(gyroTuple)
        # this executes when len(gyroQueue) == 3
        # this pops the earliest inserted element and insert latest
        else:
            gyroQueue.append(gyroTuple)
            gyroQueue.pop(0)


        # the trigger piece of code
        # if there is a 900% increase in movement then we trigger it
        # change trigger to 1 so this code will not execute during the next 7 trials
        if (trigger == 0 and gyroCounter > 5 and (previousGyroSum > 0) and (abs(currentGyroSum/previousGyroSum) >= 9) and (currentGyroSum > 5)):
            print("There is a unignorable change in motion of the bottle. Triggered.")
            trigger = 1
            #adding the -3 data points
            gyroDataPoints_10 = list(gyroQueue)
            gyroDataPoints_10.pop(2)
        else:
            pass

        #making the list become -3 + 7
        #when trigger = 1, and length of list < 10
        #continue to append list till size 10
        if (trigger != 0 and len(gyroDataPoints_10)<10):
            gyroDataPoints_10.append(gyroTuple)

        elif (trigger != 0 and (len(gyroDataPoints_10)==10) ):
            #wait for the other side to reach 10 also
            while(len(accelDataPoints_10) < 10):
                time.sleep(0.5)
            postToAws()
            trigger = 0
        else:
            pass

        previousGyroSum = currentGyroSum
        gyroCounter=gyroCounter+1 #to get at least 5 readings to stabilize the sensor before triggering anything
        print("[MovementSensor] Gyroscope:", tuple([ v*self.scale for v in rawVals ]))

#takes an input of dictionary with keys "gyro" & "accel" with list of 10 with 3 elements each
#convert the data dictionary into a (10,3,2) numpy shape format
def convertData(data):
    gyroPoints = data["Gyro"]
    accelPoints = data["Accel"]
    input_data_array = np.empty((1, 10, 3, 2))
    input_data_array[0,:,:,0] = np.asarray(gyroPoints).reshape(10,-1)
    input_data_array[0,:,:,1] = np.asarray(accelPoints).reshape(10,-1)
    return input_data_array[0]

def postToAws():

    global gyroDataPoints_10
    global accelDataPoints_10
    global client

    data = {
        "Gyro": gyroDataPoints_10,
        "Accel": accelDataPoints_10,
    }


    data = convertData(data)
    print("\nSending data to AWS.")
    send_data(client, data)
    print("\n\nData sent to AWS: ", data)

    gyroDataPoints_10.clear()
    accelDataPoints_10.clear()

async def run(address):
    async with BleakClient(address) as client:
        global awsUri
        global expectedOutput

        x = await client.is_connected()
        print("Sensor Connected: {0}".format(x))


        acc_sensor = AccelerometerSensorMovementSensorMPU9250()
        gyro_sensor = GyroscopeSensorMovementSensorMPU9250()

        movement_sensor = MovementSensorMPU9250()
        movement_sensor.register(acc_sensor)
        movement_sensor.register(gyro_sensor)

        await movement_sensor.start_listener(client)

        cntr = 0

        while True:
            # we don't want to exit the "with" block initiating the client object as the connection is disconnected
            # unless the object is stored
            await asyncio.sleep(1.0)

            cntr += 1

            if cntr == 10:
                cntr = 0

def on_connect(client, userdata, flags, rc):
    if rc == 0 :
        print("Connected to Broker")
        client.subscribe("g19/iot/predict")
        print("Subscribed to g19/iot/predict")
    else:
        print ("Failed to connect. Error code: %d. " %rc )

#server returning classification result but irrelevant
#repsonse also sent to android app which is the more relevant one
def on_message(client, userdata, msg):
    print("Received message from server")
    response = json.loads(msg.payload)
    print("The response is ", response)



#set up connection
def setup(hostname):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(hostname)
    client.loop_start()
    return client

#data to be send in dictionary?
def send_data(client, data):
    client.publish("g19/iot/classify", json.dumps(data, cls=NumpyArrayEncoder))

if __name__ == "__main__":
    """
    To find the address, once your sensor tag is blinking the green led after pressing the button, run the discover.py
    file which was provided as an example from bleak to identify the sensor tag device
    """

    if len(sys.argv) != 4:
        print("Syntax: python " + sys.argv[0] + " <SensorMacAddress> <AWS IP>")

    global accelDataPoints_10
    global gyroDataPoints_10
    global awsUri
    global sensorMac
    global expectedOutput


    global gyroCounter
    global accelCounter
    global trigger
    global previousGyroSum
    global gyroQueue
    global accelQueue

    # initialize the values
    accelDataPoints_10 = []
    gyroDataPoints_10 = []
    gyroCounter = 0
    accelCounter=  0
    trigger = 0
    previousGyroSum = 0.1
    gyroQueue = []
    accelQueue = []

    sensorMac = sys.argv[1]
    awsIP = sys.argv[2]
    # sensorMac = "F0:F8:F2:86:BB:85"

    global client
    client=setup(awsIP) #input IP
    time.sleep(1)

    os.environ["PYTHONASYNCIODEBUG"] = str(1)
    address = (
        sensorMac
        if platform.system() != "Darwin"
        else "6FFBA6AE-0802-4D92-B1CD-041BE4B4FEB9"
    )
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(address))
    loop.run_forever()
