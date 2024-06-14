#!/usr/bin/env python

# USE PYTHON 2 IDEALY. OR ELSE SOME CHANGES TO INPUT COMMANDS WILL NEED TO BE MADE (raw_input() versus input() )
# Writes to a file under Documents called "0_aposMeasurement.csv" :D
# 

####################################################################################################
####################################################################################################
##                                                                                                ##
## Hove's Raspberry Pi Python Quadcopter Flight Controller.  Open Source @ GitHub                 ##
## PiStuffing/Quadcopter under GPL for non-commercial application.  Any code derived from         ##
## this should retain this copyright comment.                                                     ##
##                                                                                                ##
## Copyright 2012 - 2018 Andy Baker (Hove) - andy@pistuffing.co.uk                                ##
##                                                                                                ##
####################################################################################################
####################################################################################################


from __future__ import division
from __future__ import with_statement
import signal
import socket
import time
import sys
import getopt
import math
from array import *
import smbus
import select
import os
import io
import logging
import csv
import RPi.GPIO as GPIO
import subprocess
import ctypes
from ctypes.util import find_library
import struct
import serial

FULL_FIFO_BATCHES = 20 # << int(512 / 12)

####################################################################################################
#
#  Adafruit i2c interface enhanced with performance / error handling enhancements
#
####################################################################################################
class I2C:

    def __init__(self, address, bus=smbus.SMBus(1)):
        self.address = address
        self.bus = bus
        self.misses = 0

    def writeByte(self, value):
        self.bus.write_byte(self.address, value)

    def write8(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def writeList(self, reg, list):
        self.bus.write_i2c_block_data(self.address, reg, list)

    def readU8(self, reg):
        result = self.bus.read_byte_data(self.address, reg)
        return result

    def readS8(self, reg):
        result = self.bus.read_byte_data(self.address, reg)
        result = result - 256 if result > 127 else result
        return result

    def readU16(self, reg):
        hibyte = self.bus.read_byte_data(self.address, reg)
        result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
        return result

    def readS16(self, reg):
        hibyte = self.bus.read_byte_data(self.address, reg)
        hibyte = hibyte - 256 if hibyte > 127 else hibyte
        result = (hibyte << 8) + self.bus.read_byte_data(self.address, reg+1)
        return result

    def readList(self, reg, length):
        "Reads a byte array value from the I2C device. The content depends on the device.  The "
        "FIFO read return sequential values from the same register.  For all other, sequestial"
        "regester values are returned"
        result = self.bus.read_i2c_block_data(self.address, reg, length)
        return result


####################################################################################################
#
#  Gyroscope / Accelerometer class for reading position / movement.  Works with the Invensense IMUs:
#  - MPU-6050 register addresses
#
####################################################################################################
class MPU6050:
    i2c = None

    # Registers/etc.
    __MPU6050_RA_SELF_TEST_XG = 0x00
    __MPU6050_RA_SELF_TEST_YG = 0x01
    __MPU6050_RA_SELF_TEST_ZG = 0x02
    __MPU6050_RA_SELF_TEST_XA = 0x0D
    __MPU6050_RA_SELF_TEST_YA = 0x0E
    __MPU6050_RA_SELF_TEST_ZA = 0x0F
    __MPU6050_RA_XG_OFFS_USRH = 0x13
    __MPU6050_RA_XG_OFFS_USRL = 0x14
    __MPU6050_RA_YG_OFFS_USRH = 0x15
    __MPU6050_RA_YG_OFFS_USRL = 0x16
    __MPU6050_RA_ZG_OFFS_USRH = 0x17
    __MPU6050_RA_ZG_OFFS_USRL = 0x18
    __MPU6050_RA_SMPLRT_DIV = 0x19
    __MPU6050_RA_CONFIG = 0x1A
    __MPU6050_RA_GYRO_CONFIG = 0x1B
    __MPU6050_RA_ACCEL_CONFIG = 0x1C
    __MPU9250_RA_ACCEL_CFG_2 = 0x1D
    __MPU6050_RA_FF_THR = 0x1D
    __MPU6050_RA_FF_DUR = 0x1E
    __MPU6050_RA_MOT_THR = 0x1F
    __MPU6050_RA_MOT_DUR = 0x20
    __MPU6050_RA_ZRMOT_THR = 0x21
    __MPU6050_RA_ZRMOT_DUR = 0x22
    __MPU6050_RA_FIFO_EN = 0x23
    __MPU6050_RA_I2C_MST_CTRL = 0x24
    __MPU6050_RA_I2C_SLV0_ADDR = 0x25
    __MPU6050_RA_I2C_SLV0_REG = 0x26
    __MPU6050_RA_I2C_SLV0_CTRL = 0x27
    __MPU6050_RA_I2C_SLV1_ADDR = 0x28
    __MPU6050_RA_I2C_SLV1_REG = 0x29
    __MPU6050_RA_I2C_SLV1_CTRL = 0x2A
    __MPU6050_RA_I2C_SLV2_ADDR = 0x2B
    __MPU6050_RA_I2C_SLV2_REG = 0x2C
    __MPU6050_RA_I2C_SLV2_CTRL = 0x2D
    __MPU6050_RA_I2C_SLV3_ADDR = 0x2E
    __MPU6050_RA_I2C_SLV3_REG = 0x2F
    __MPU6050_RA_I2C_SLV3_CTRL = 0x30
    __MPU6050_RA_I2C_SLV4_ADDR = 0x31
    __MPU6050_RA_I2C_SLV4_REG = 0x32
    __MPU6050_RA_I2C_SLV4_DO = 0x33
    __MPU6050_RA_I2C_SLV4_CTRL = 0x34
    __MPU6050_RA_I2C_SLV4_DI = 0x35
    __MPU6050_RA_I2C_MST_STATUS = 0x36
    __MPU6050_RA_INT_PIN_CFG = 0x37
    __MPU6050_RA_INT_ENABLE = 0x38
    __MPU6050_RA_DMP_INT_STATUS = 0x39
    __MPU6050_RA_INT_STATUS = 0x3A
    __MPU6050_RA_ACCEL_XOUT_H = 0x3B
    __MPU6050_RA_ACCEL_XOUT_L = 0x3C
    __MPU6050_RA_ACCEL_YOUT_H = 0x3D
    __MPU6050_RA_ACCEL_YOUT_L = 0x3E
    __MPU6050_RA_ACCEL_ZOUT_H = 0x3F
    __MPU6050_RA_ACCEL_ZOUT_L = 0x40
    __MPU6050_RA_TEMP_OUT_H = 0x41
    __MPU6050_RA_TEMP_OUT_L = 0x42
    __MPU6050_RA_GYRO_XOUT_H = 0x43
    __MPU6050_RA_GYRO_XOUT_L = 0x44
    __MPU6050_RA_GYRO_YOUT_H = 0x45
    __MPU6050_RA_GYRO_YOUT_L = 0x46
    __MPU6050_RA_GYRO_ZOUT_H = 0x47
    __MPU6050_RA_GYRO_ZOUT_L = 0x48
    __MPU6050_RA_EXT_SENS_DATA_00 = 0x49
    __MPU6050_RA_EXT_SENS_DATA_01 = 0x4A
    __MPU6050_RA_EXT_SENS_DATA_02 = 0x4B
    __MPU6050_RA_EXT_SENS_DATA_03 = 0x4C
    __MPU6050_RA_EXT_SENS_DATA_04 = 0x4D
    __MPU6050_RA_EXT_SENS_DATA_05 = 0x4E
    __MPU6050_RA_EXT_SENS_DATA_06 = 0x4F
    __MPU6050_RA_EXT_SENS_DATA_07 = 0x50
    __MPU6050_RA_EXT_SENS_DATA_08 = 0x51
    __MPU6050_RA_EXT_SENS_DATA_09 = 0x52
    __MPU6050_RA_EXT_SENS_DATA_10 = 0x53
    __MPU6050_RA_EXT_SENS_DATA_11 = 0x54
    __MPU6050_RA_EXT_SENS_DATA_12 = 0x55
    __MPU6050_RA_EXT_SENS_DATA_13 = 0x56
    __MPU6050_RA_EXT_SENS_DATA_14 = 0x57
    __MPU6050_RA_EXT_SENS_DATA_15 = 0x58
    __MPU6050_RA_EXT_SENS_DATA_16 = 0x59
    __MPU6050_RA_EXT_SENS_DATA_17 = 0x5A
    __MPU6050_RA_EXT_SENS_DATA_18 = 0x5B
    __MPU6050_RA_EXT_SENS_DATA_19 = 0x5C
    __MPU6050_RA_EXT_SENS_DATA_20 = 0x5D
    __MPU6050_RA_EXT_SENS_DATA_21 = 0x5E
    __MPU6050_RA_EXT_SENS_DATA_22 = 0x5F
    __MPU6050_RA_EXT_SENS_DATA_23 = 0x60
    __MPU6050_RA_MOT_DETECT_STATUS = 0x61
    __MPU6050_RA_I2C_SLV0_DO = 0x63
    __MPU6050_RA_I2C_SLV1_DO = 0x64
    __MPU6050_RA_I2C_SLV2_DO = 0x65
    __MPU6050_RA_I2C_SLV3_DO = 0x66
    __MPU6050_RA_I2C_MST_DELAY_CTRL = 0x67
    __MPU6050_RA_SIGNAL_PATH_RESET = 0x68
    __MPU6050_RA_MOT_DETECT_CTRL = 0x69
    __MPU6050_RA_USER_CTRL = 0x6A
    __MPU6050_RA_PWR_MGMT_1 = 0x6B
    __MPU6050_RA_PWR_MGMT_2 = 0x6C
    __MPU6050_RA_BANK_SEL = 0x6D
    __MPU6050_RA_MEM_START_ADDR = 0x6E
    __MPU6050_RA_MEM_R_W = 0x6F
    __MPU6050_RA_DMP_CFG_1 = 0x70
    __MPU6050_RA_DMP_CFG_2 = 0x71
    __MPU6050_RA_FIFO_COUNTH = 0x72
    __MPU6050_RA_FIFO_COUNTL = 0x73
    __MPU6050_RA_FIFO_R_W = 0x74
    __MPU6050_RA_WHO_AM_I = 0x75

    __RANGE_ACCEL = 8                                                            #AB: +/- 8g
    __RANGE_GYRO = 250                                                           #AB: +/- 250o/s

    __SCALE_GYRO = math.radians(2 * __RANGE_GYRO / 65536)
    __SCALE_ACCEL = 2 * __RANGE_ACCEL / 65536

    def __init__(self, address=0x68, alpf=2, glpf=1):
        self.i2c = I2C(address)
        self.address = address

        self.min_az = 0.0
        self.max_az = 0.0
        self.min_gx = 0.0
        self.max_gx = 0.0
        self.min_gy = 0.0
        self.max_gy = 0.0
        self.min_gz = 0.0
        self.max_gz = 0.0

        self.ax_offset = 0.0
        self.ay_offset = 0.0
        self.az_offset = 0.0

        self.gx_offset = 0.0
        self.gy_offset = 0.0
        self.gz_offset = 0.0

        #-------------------------------------------------------------------------------------------
        # Reset all registers
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        
        #-------------------------------------------------------------------------------------------
        # Sets sample rate to 1kHz/(1+0) = 1kHz or 1ms (note 1kHz assumes dlpf is on - setting
        # dlpf to 0 or 7 changes 1kHz to 8kHz and therefore will require sample rate divider
        # to be changed to 7 to obtain the same 1kHz sample rate.
        #-------------------------------------------------------------------------------------------
        sample_rate_divisor = int(round(adc_frequency / sampling_rate))
##        print('sample rate: ' +str(sample_rate_divisor))
        self.i2c.write8(self.__MPU6050_RA_SMPLRT_DIV, sample_rate_divisor - 1)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Sets clock source to gyro reference w/ PLL
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Gyro DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 =  250Hz @ 8kHz sampling - DO NOT USE, THE ACCELEROMETER STILL SAMPLES AT 1kHz WHICH PRODUCES EXPECTED BUT NOT CODED FOR TIMING AND FIFO CONTENT PROBLEMS
        # 0x01 =  184Hz
        # 0x02 =   92Hz
        # 0x03 =   41Hz
        # 0x04 =   20Hz
        # 0x05 =   10Hz
        # 0x06 =    5Hz
        # 0x07 = 3600Hz @ 8kHz
        #
        # 0x0* FIFO overflow overwrites oldest FIFO contents
        # 0x4* FIFO overflow does not overwrite full FIFO contents
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_CONFIG, 0x40 | glpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable gyro self tests, scale of +/- 250 degrees/s
        #
        # 0x00 =  +/- 250 degrees/s
        # 0x08 =  +/- 500 degrees/s
        # 0x10 = +/- 1000 degrees/s
        # 0x18 = +/- 2000 degrees/s
        # See SCALE_GYRO for conversion from raw data to units of radians per second
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_GYRO_CONFIG, int(round(math.log(self.__RANGE_GYRO / 250, 2))) << 3)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Accel DLPF => 1kHz sample frequency used above divided by the sample divide factor.
        #
        # 0x00 = 460Hz
        # 0x01 = 184Hz
        # 0x02 =  92Hz
        # 0x03 =  41Hz
        # 0x04 =  20Hz
        # 0x05 =  10Hz
        # 0x06 =   5Hz
        # 0x07 = 460Hz
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU9250_RA_ACCEL_CFG_2, alpf)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Disable accel self tests, scale of +/-8g
        #
        # 0x00 =  +/- 2g
        # 0x08 =  +/- 4g
        # 0x10 =  +/- 8g
        # 0x18 = +/- 16g
        # See SCALE_ACCEL for convertion from raw data to units of meters per second squared
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_ACCEL_CONFIG, 0x10)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Set INT pin to push/pull, latch 'til read, any read to clear
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_PIN_CFG, 0x30)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Initialize the FIFO overflow interrupt 0x10 (turned off at startup).
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)
        time.sleep(0.1)

        #-------------------------------------------------------------------------------------------
        # Enabled the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_USER_CTRL, 0x40)

        #-------------------------------------------------------------------------------------------
        # Accelerometer / gyro goes into FIFO later on - see flushFIFO()
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)
        
    def enableFIFOOverflowISR(self):
        overflowState = 0
        enableChecker = 0
        
        # enable Overflow Interupt
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x10)
        
        # check the value of register once enabled
        enableChecker = self.i2c.readU8(self.__MPU6050_RA_INT_ENABLE)
        overflowState = self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)
        
        bin_enableChecker = bin(enableChecker)
        
        print('OVERFLOW ENABLED')
##        print('enabled register value: ' +str(enableChecker))
##        print('enabled register value in binary (0b): ' +str(bin_enableChecker))
##        print('clean status value: ' +str(overflowState))
        
        
    def disableFIFOOverflowISR(self):
        enableChecker = 0
        overflowState = 0
        
        # disable Overflow Interrupt
        self.i2c.write8(self.__MPU6050_RA_INT_ENABLE, 0x00)
        
        # check the status of the register once disabled
        enableChecker = self.i2c.readU8(self.__MPU6050_RA_INT_ENABLE)
        overflowState = self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)
        
        print('OVERFLOW DISABLED')
##        print('disabled register value: ' +str(enableChecker))
##        print('disabled register overflow status: ' +str(overflowState))
        
    def numFIFOBatches(self):
        #-------------------------------------------------------------------------------------------
        # The FIFO is 512 bytes long, and we're storing 6 signed shorts (ax, ay, az, gx, gy, gz) i.e.
        # 12 bytes per batch of sensor readings
        # *THEREFORE 6 bytes since only ax, ay, az are grabbed
        #-------------------------------------------------------------------------------------------
        
        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)
#        fifo_batches = int(fifo_bytes / 12) # This rounds down
        fifo_batches = int(fifo_bytes / 6)  # This rounds down* 
        return fifo_batches

    def readFIFO(self, fifo_batches):
        #-------------------------------------------------------------------------------------------
        # Read n x 12 bytes of FIFO data averaging, and return the averaged values and inferred time
        # based upon the sampling rate and the number of samples.
        #-------------------------------------------------------------------------------------------
        
        sensorDataComplete = []

        for ii in range(fifo_batches):
            sensor_data = []
#            fifo_batch = self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, 12)
            fifo_batch = self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, 6) #* down to only 6 bytes for accel only values
            for jj in range(0, 6, 2):
                hibyte = fifo_batch[jj]
                hibyte = hibyte - 256 if hibyte > 127 else hibyte
                lobyte = fifo_batch[jj + 1]
                sensorDataComplete.append((hibyte << 8) + lobyte)

            
        return sensorDataComplete

    def flushFIFO(self):
        #-------------------------------------------------------------------------------------------
        # First shut off the feed in the FIFO.
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x00)

        #-------------------------------------------------------------------------------------------
        # Empty the FIFO by reading whatever is there
        #-------------------------------------------------------------------------------------------
        SMBUS_MAX_BUF_SIZE = 32
        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(int(fifo_bytes / SMBUS_MAX_BUF_SIZE)):
            self.i2c.readList(self.__MPU6050_RA_FIFO_R_W, SMBUS_MAX_BUF_SIZE)

        fifo_bytes = self.i2c.readU16(self.__MPU6050_RA_FIFO_COUNTH)

        for ii in range(fifo_bytes):
            self.i2c.readU8(self.__MPU6050_RA_FIFO_R_W)

        #-------------------------------------------------------------------------------------------
        # Finally start feeding the FIFO with sensor data again
        #-------------------------------------------------------------------------------------------
        self.i2c.write8(self.__MPU6050_RA_FIFO_EN, 0x08) ## this sets to only write acceleration data into FIFO

    # used to check and read overflow status (whether the FIFO overflowed or not)
    def checkFifoOverflow(self):
        overflowValue = 0
        
        # value of the interupt register
        overflowValue = self.i2c.readU8(self.__MPU6050_RA_INT_STATUS)
                
        # convert unsigned 8bit (1byte) to binary
        bin_overflowValue  = bin(overflowValue)
        
##        print('overflow value: ' +str(overflowValue))
##        print('overflow value in binary (0b): ' +str(bin_overflowValue))
        
        return (overflowValue, bin_overflowValue)
    
    def setGyroOffsets(self, gx, gy, gz):
        self.gx_offset = gx
        self.gy_offset = gy
        self.gz_offset = gz

    def scaleSensors(self, ax, ay, az, gx, gy, gz):
        qax = (ax - self.ax_offset) * self.__SCALE_ACCEL
        qay = (ay - self.ay_offset) * self.__SCALE_ACCEL
        qaz = (az - self.az_offset) * self.__SCALE_ACCEL

        qrx = (gx - self.gx_offset) * self.__SCALE_GYRO
        qry = (gy - self.gy_offset) * self.__SCALE_GYRO
        qrz = (gz - self.gz_offset) * self.__SCALE_GYRO

        return qax, qay, qaz, qrx, qry, qrz

    def getStats(self):
        return (self.max_az * self.__SCALE_ACCEL,
                self.min_az * self.__SCALE_ACCEL,
                self.max_gx * self.__SCALE_GYRO,
                self.min_gx * self.__SCALE_GYRO,
                self.max_gy * self.__SCALE_GYRO,
                self.min_gy * self.__SCALE_GYRO,
                self.max_gz * self.__SCALE_GYRO,
                self.min_gz * self.__SCALE_GYRO)

####################################################################################################
#
# Butterwork IIR Filter calculator and actor - this is carried out in the earth frame as we are track
# gravity drift over time from 0, 0, 1 (the primer values for egx, egy and egz)
#
# Code is derived from http://www.exstrom.com/journal/sigproc/bwlpf.c
#
####################################################################################################
class BUTTERWORTH:
    def __init__(self, sampling, cutoff, order, primer):

        self.n = int(round(order / 2))
        self.A = []
        self.d1 = []
        self.d2 = []
        self.w0 = []
        self.w1 = []
        self.w2 = []

        a = math.tan(math.pi * cutoff / sampling)
        a2 = math.pow(a, 2.0)

        for ii in range(0, self.n):
            r = math.sin(math.pi * (2.0 * ii + 1.0) / (4.0 * self.n))
            s = a2 + 2.0 * a * r + 1.0
            self.A.append(a2 / s)
            self.d1.append(2.0 * (1 - a2) / s)
            self.d2.append(-(a2 - 2.0 * a * r + 1.0) / s)

            self.w0.append(primer / (self.A[ii] * 4))
            self.w1.append(primer / (self.A[ii] * 4))
            self.w2.append(primer / (self.A[ii] * 4))

    def filter(self, input):
        for ii in range(0, self.n):
            self.w0[ii] = self.d1[ii] * self.w1[ii] + self.d2[ii] * self.w2[ii] + input
            output = self.A[ii] * (self.w0[ii] + 2.0 * self.w1[ii] + self.w2[ii])
            self.w2[ii] = self.w1[ii]
            self.w1[ii] = self.w0[ii]

        return output

####################################################################################################
#
# Functions to lock memory to prevent paging, and move child processes in different process groups
# such that a Ctrl-C / SIGINT to one isn't distributed automatically to all children.
#
####################################################################################################
MCL_CURRENT = 1
MCL_FUTURE  = 2
def mlockall(flags = MCL_CURRENT| MCL_FUTURE):
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.mlockall(flags)
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def munlockall():
    libc_name = ctypes.util.find_library("c")
    libc = ctypes.CDLL(libc_name, use_errno=True)
    result = libc.munlockall()
    if result != 0:
        raise Exception("cannot lock memory, errno=%s" % ctypes.get_errno())

def Daemonize():
    #-----------------------------------------------------------------------------------------------
    # Discondect child processes so ctrl-C doesn't kill them
    # Increment priority such that Motion is -10, Autopilot and Video are -5, and Sweep and GPS are 0.
    #-----------------------------------------------------------------------------------------------
    os.setpgrp()
    os.nice(5)

    '''
    #AB: ###########################################################################################
    #AB: # Consider here munlockall() to allow paging for lower priority processes i.e. all be main and video
    #AB: ###########################################################################################
    '''

####################################################################################################
#
# Class to split initialation, flight startup and flight control
#
####################################################################################################
class Quadcopter:

    #===============================================================================================
    # One-off initialization
    #===============================================================================================
    def __init__(self):
        
        # prioritize code/program?
        os.nice(-10)

        #-------------------------------------------------------------------------------------------
        # Set the signal handler here so the core processing loop can be stopped (or not started) by
        # Ctrl-C.
        #-------------------------------------------------------------------------------------------
        signal.signal(signal.SIGINT, self.shutdownSignalHandler)
        
        # lock code into memory?
        mlockall()
        print('MEMORY LOCKED')
        
        #===========================================================================================
        # Globals for the IMU setup
        # adc_frequency      - the sampling rate of the ADC
        # sampling_rate      - the data sampling rate and thus data ready interrupt rate
        #                      motion processing.
        # motion_rate        - the target frequency motion processing occurs under perfect conditions.
        # fusion_rate        - the sampling rate of the GLL and the video frame rate
        # alpf               - the accelerometer low pass filter
        # glpf               - the gyrometer low pass filter
        #===========================================================================================
        global adc_frequency
        global sampling_rate
        global motion_rate
        global fusion_rate

        adc_frequency = 1000         # defined by dlpf >= 1; DO NOT USE ZERO => 8000 adc_frequency
        fusion_rate = 10

        sampling_rate = 1000
        motion_rate = 75
        glpf = 1                     # 184Hz

#        #-------------------------------------------------------------------------------------------
#        # This is not for antialiasing: the accelerometer low pass filter happens between the ADC
#        # rate and our IMU sampling rate.  ADC rate is 1kHz through this case.  However, I've seen poor
#        # behavious in double integration when IMU sampling rate is 500Hz and alpf = 460Hz.
#        #-------------------------------------------------------------------------------------------
#        if sampling_rate == 1000:    # SRD = 0 (1kHz)
#            alpf = 0                 # alpf = 460Hz
#        elif sampling_rate == 500:   # SRD = 1 (500Hz)
#            alpf = 1                 # alpf = 184Hz
#        elif sampling_rate >= 200:   # SRD = 2, 3, 4 (333, 250, 200Hz)
#            alpf = 2                 # alpf = 92Hz
#        elif sampling_rate >= 100:   # SRD = 5, 6, 7, 8, 9 (166, 143, 125, 111, 100Hz)
#            alpf = 3                 # alpf = 41Hz
#        else:
#            #--------------------------------------------------------------------------------------
#            # There's no point going less than 100Hz IMU sampling; we need about 100Hz motion
#            # processing for some degree of level of stability.
#            #--------------------------------------------------------------------------------------
#            print "SRD + alpf useless: forget it!"
#            return
        
        alpf = 0

        global mpu6050
        mpu6050 = MPU6050(0x68, alpf, glpf)
        
        #-------------------------------------------------------------------------------------------
        # Scheduling parameters defining standard, and critical FIFO block counts
        #
        # FIFO_MINIMUM - The least number of batches of collect and average for running them through
        #                the motion processor
        # FIFO_MAXIMUM - The most number of batches to be allowed through the motion processor; any
        #                higher risks FIFO overflow.
        #
        # 512/12 is the maximum number of batches in the IMU FIFO
        #
        #-------------------------------------------------------------------------------------------
        self.FIFO_MINIMUM = int(round(sampling_rate / motion_rate))
        self.FIFO_MAXIMUM = int(round(512 / 12)) - self.FIFO_MINIMUM

    #===============================================================================================
    # Keyboard / command line input between flights for CLI update etc
    #===============================================================================================
    def go(self):     

        print ("Initializing!")
        
        ################################### INITIALIZE VARIABLES ###################################

        mpu6050.flushFIFO()
        
        qax = 0.0
        qay = 0.0
        qaz = 0.0
        qrx = 0.0
        qry = 0.0
        qrz = 0.0
        motion_dt = 0
        
        # create global variables to store resulting data
        # making it accessible across multiple functions
        global totalFifoData # stores accel data from FIFO
        totalFifoData = []
        
        # AFS_SEL 0 +- 2g   #
        #         1 +- 4g   #
        #         2 +- 8g   #
        #         3 +- 16g  #
        AFS_SEL = 2
        global negAccScale # scaling value
        global accScale # scaling value
        negAccScale = -16384/(2**AFS_SEL) # apparently z is inverted 
        accScale = 16384/(2**AFS_SEL) # but x and y are not
        
        # for trouble shooting, making sure these values can be tracked from anywhere
        global batchTracker # tracks how many TOTAL batches were gathered
        batchTracker = 0
        global runTracker # tracks how many times the "read FIFO" function was triggered
        runTracker = 0
        global timeAdder
        timeAdder = 0 # tracks how much time total was spent running the code within the loop
        global overflowCounter
        overflowCounter = 0 # tracks how many times overflow is triggered
        
        sigma_dt = 0.0
        loops = 0

        # Flush the IMU FIFO and enable the FIFO overflow interrupt
        #------------------------------------------------------------------------------------------
        mpu6050.flushFIFO()
        
        # enable overflow int for troubleshooting
        mpu6050.enableFIFOOverflowISR()
        
        # disable overflow for troubleshooting
##        mpu6050.disableFIFOOverflowISR()
        #-------------------------------------------------------------------------------------------
        
        #-------------------------------------------------------------------------------------------
        # ask for how long code is ran for
##        lengthToRun = input('how long do you want to run for (in sec): ')
##        intLengthToRun = int(lengthToRun)
##        t_end = time.time() + intLengthToRun
##        print('time start: ' +str(time.time()))
##        print('time end: ' +str(t_end))
        #-------------------------------------------------------------------------------------------
        
        #-------------------------------------------------------------------------------------------
        # ask to run?
        userAnswer = raw_input('Do You Want To Run (Y/N)? ')
        if userAnswer == 'Y':
            print('Sit tight! The program will begin shortly')
        elif userAnswer == 'y':
            print('The program will begin shortly')
        elif userAnswer == 'N':
            print('Terminating program because answer was ' +str(userAnswer))
            sys.exit(0)
        elif userAnswer == 'n':
            print('Terminating program because answer was ' +str(userAnswer))
            sys.exit(0)
        else:
            print('Error, not recognizable command. Terminating.')
            sys.exit(0)
        
        #-------------------------------------------------------------------------------------------
        
        time.sleep(1)
        
        #-------------------------------------------------------------------------------------------
        # clear FIFO and check whether overflow value is high  
        print('FLSUH FIFO')
        mpu6050.flushFIFO()
##        overflowChecker666, bin_overflowChecker666 = mpu6050.checkFifoOverflow()
        #-------------------------------------------------------------------------------------------
        
        #----------------------------------------MAIN-----------------------------------------------
        #-------------------------------------------------------------------------------------------
        # while within the desired amount of time, grab data from FIFO and store it in a list
        print('RUNNING! hit Ctrl+c to end sampling. Then please wait for data to be saved before proceeding.')
##        while time.time() < t_end:
        while True:
            startRead = time.time()
            
            # check overflow values
            overflowChecker, bin_overflowChecker = mpu6050.checkFifoOverflow()
##            overflowCondition = 0b10000 # i guess i didn't program the binary method properly :/
##            overflowCondition2 = 0b10001
            
            #if FIFO overflows notify
            if overflowChecker == 16 or overflowChecker == 17:
                print('OVERFLOW!')
                overflowCounter += 1
            
            # get number of batches in the FIFO
            nfb = mpu6050.numFIFOBatches()
##            print('number of batches: ' +str(nfb))
            
            # if number of batches inside FIFO is greater or equal to desired number. read and store
            if nfb >= 5:
                
                # stores the gained data into a list by extending onto the end
                FIFOdataString = mpu6050.readFIFO(nfb)
                totalFifoData.extend(FIFOdataString)
                
                # turns out manually reseting is not required therefore mpu6050.flushFIFO command is not necessary
                # flushFIFO command also takes too long to execute (aprox. 0.2573s)
                # values tracking for troubleshooting
                batchTracker += nfb
                runTracker += 1
                
            endRead = time.time()
            timeAdder += endRead-startRead
##            print('time requied to read on run: ' +str(endRead-startRead), (runTracker))
        #-------------------------------------------------------------------------------------------
        #-----------------------------------------END-----------------------------------------------
        
        # unlock code from memory
        munlockall()
        print('MEMORY UNLOCKED')

        print('_________________________________________________________________________________')
        print('Total resulting FIFO length: ' +str(len(totalFifoData)))
        print('Total number of batches (sets of x,y,z, acceleration data): ' +str(batchTracker))
        print('The number of times the FIFO was read: ' +str(runTracker))
        print('Total read time accumulated: ' +str(timeAdder))
        print('Total times overflow counter was triggered: ' +str(overflowCounter))
        print('_________________________________________________________________________________')
        
        #-------------------------------------------------------------------------------------------
        # generate location from which data will be written into
        fileLocationAndName = "/home/pi/Documents/0_aposMeasurement.csv"
        
        inferedTime = 0
        # write data into .csv
        with open(fileLocationAndName, "a") as log:
            print('Please wait. Writing to csv file: ' +str (fileLocationAndName))
            for ii in range(0, len(totalFifoData), 3):
                # manual z offset using a estimated value such that gravity exerts a reading of -1 as oppose to approx. -0.88
                # from observation of the resulting data, the entire output seems to be similarly shifted and offsetted.
                dataZacccelScaledAndOffsetted = (totalFifoData[ii+2]/negAccScale)-0.145
                log.write("{0},{1},{2},{3}\n".format(inferedTime, totalFifoData[ii]/accScale, totalFifoData[ii+1]/accScale, (totalFifoData[ii+2]/negAccScale)-0.145)) # a manual z offset :/
                inferedTime += 0.01
                
        print('DONE')
            
    ################################################################################################
    #
    # Shutdown triggered by early Ctrl-C or end of script
    #
    ################################################################################################
    def shutdown(self):

        # Stop the signal handler
        signal.signal(signal.SIGINT, signal.SIG_IGN)

        # Unlock memory we've used from RAM
        print('MEMORY UNLOCKED case #2')
        munlockall()

        # initial pauses used to "Clean up PWM / GPIO, but pause beforehand to give the ESCs time to stop properly"
        #now as just placeholders. Highly doubt these pauses are at all necessary
        print('WAITING...')
        time.sleep(0.5)
        print('CLEANING UP...')
        time.sleep(0.5)

        # Reset the signal handler to default
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        # place holder to pause code (terminal) incase sys.exit closes terminal
        placeHolder = raw_input('Click Enter to exit: ')
        sys.exit(0)

    ####################################################################################################
    #
    # Signal handler for Ctrl-C => abort cleanly; should really be just a "try: except KeyboardInterrupt:"
    #
    ####################################################################################################
    def shutdownSignalHandler(self, signal, frame):
       
        # print for trouble shooting and monitoring
        print('_________________________________________________________________________________')
        print('Total resulting FIFO length: ' +str(len(totalFifoData)))
        print('Total number of batches (sets of x,y,z, acceleration data): ' +str(batchTracker))
        print('The number of times the FIFO was read: ' +str(runTracker))
        print('Total read time accumulated: ' +str(timeAdder))
        print('Total times overflow counter was triggered: ' +str(overflowCounter))
        print('_________________________________________________________________________________')
        
        #-------------------------------------------------------------------------------------------
        # generate location from which data will be written into
        fileLocationAndName = "/home/pi/Documents/0_aposMeasurement.csv"
        
        inferedTime = 0
        # write data into .csv
        with open(fileLocationAndName, "a") as log:
            print('Please wait... Writing to csv file: ' +str (fileLocationAndName))
            for ii in range(0, len(totalFifoData), 3):
                # manual z offset using a estimated value such that gravity exerts a reading of -1 as oppose to approx. -0.88
                # from observation of the resulting data, the entire output seems to be similarly shifted and offsetted.
                dataZacccelScaledAndOffsetted = (totalFifoData[ii+2]/negAccScale)-0.145
                log.write("{0},{1},{2},{3}\n".format(inferedTime, totalFifoData[ii]/accScale, totalFifoData[ii+1]/accScale, (totalFifoData[ii+2]/negAccScale)-0.145)) # a manual z offset
                inferedTime += 0.001                
        print('DONE WRITING')
        
        self.shutdown()
           