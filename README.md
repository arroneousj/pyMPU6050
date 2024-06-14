# pyMPU6050

This goal of this project was to use the MPU-6050, a 3-axis gyroscope and 3-axis accelerometer package, with a Raspberry Pi. \
The goal was to measure the forces undergone by a track guided APOS system (Absolute Positioning) to ensure that the forces are acceptable.

This code was originally written in 2018 and utilizes the library found [here](https://github.com/PiStuffing/Quadcopter/tree/master) for use in a quadcopter. \
It alters it in that the FIFO buffer is needed only to store accelerometer data and that all 1000 samples are desired to be collected instead of an average.

The code is written for python 2 and is ran using the command: \
`sudo python accelerometerRunner.py`

The program is stopped by pressing `Ctrl + c` in which all the data measured and accumulated will be written into a csv file before terminating. \
The timestamps for each measurement is inferred based on the sample rate of 1kHz.

