# RC_Car
Code used to build an RC car

bfs.c, bfs.h, rc_code.xc
created by Daniel Henderson & Harrison Kuzbyt


Open source software:

i2c-mm.xc, i2c.h ~ XMOS

mpu6050.h/xc & mpu6050regs.h ~ Jeff Rowberg

ABOUT:

This code uses a Wifi interface to communcate with the device from one's computer

It uses an IMU to orient itself, the IMU works by detecting the angle the car is currently at from the point when it was turned on

The car has a breadth first search implemented to determine the opitmal route through a maze with obstacles
