#!/usr/bin/env python
#-----------------------------------------------------------------------------
# ex1_qwiic_ICM20948.py
#
# Simple Example for the Qwiic ICM20948 Device
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, March 2020
# 
# This python library supports the SparkFun Electroncis qwiic 
# qwiic sensor/board ecosystem on a Raspberry Pi (and compatable) single
# board computers. 
#
# More information on qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#
#==================================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the "Software"), to deal 
# in the Software without restriction, including without limitation the rights 
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
# copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all 
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
# SOFTWARE.
#==================================================================================
# Example 1

# from __future__ import print_function は、Python 2.x のコードで Python 3.x の機能を使用するための特殊な文です。
from __future__ import print_function
import qwiic_icm20948
from .madgwick_py import madgwickahrs
import time
import sys

def runExample():
    print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
    # インスタンスの設定
    IMU = qwiic_icm20948.QwiicIcm20948()
    madgwick = madgwickahrs.MadgwickAHRS()
    time_count = 0
    # 繋がっているかの確認
    if IMU.connected == False:
        print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", file=sys.stderr)
        return

    # start IMU data accessing
    IMU.begin()
    # csvファイルへの書き込み
    file = open("accel_data.csv", "w")
    while True:
        file.write(",ax,ay,az,gx,gy,gz,roll,pitch,yaw,time")

        for i in range(1000):
            if IMU.dataReady():
                IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
                # 6桁の数字で、タブ区切り
                print('{:7.2f}'.format(IMU.axRaw), '\t', '{:7.2f}'.format(IMU.ayRaw), '\t', '{:7.2f}'.format(IMU.azRaw),
                      '\t', '{:7.2f}'.format(IMU.gxRaw), '\t', '{:7.2f}'.format(IMU.gyRaw), '\t', '{:7.2f}'.format(IMU.gzRaw),
                      '\t', '{:7.2f}'.format(IMU.mxRaw), '\t', '{:7.2f}'.format(IMU.myRaw), '\t', '{:7.2f}'.format(IMU.mzRaw))
                accelerometer = [IMU.axRaw, IMU.ayRaw, IMU.azRaw]
                gyroscope = [IMU.gxRaw, IMU.gyRaw, IMU.gzRaw]
                roll, pitch, yaw = madgwick.update_imu(gyroscope, accelerometer)
                # csvファイルに書き込み
                file.write(str(i) + "," + str(IMU.axRaw) + "," + str(IMU.ayRaw) + "," + str(IMU.azRaw) + "," + str(IMU.gxRaw) + ","
                           + str(IMU.gyRaw) + "," + str(IMU.gzRaw) + "," + str(roll) + ","
                           + str(pitch) + "," + str(yaw) + "," + str(time) + "\n")
                # 一定の時間で実行されるために、time.sleepを入れてる.
                # サンプリングレートは100HZ
                time_count += 0.01
                time.sleep(0.01)
            else:
                print("Waiting for data")
                time.sleep(0.5)
        break
    file.close()

if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example 1")
        sys.exit(0)

