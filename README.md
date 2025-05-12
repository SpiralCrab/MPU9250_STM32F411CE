MPU9250_STM32F411CE
===================

Introduction
------------
This repository is a modified version of [Kris Winer's MPU9250 library](https://github.com/kriswiner/MPU9250/tree/master). The library is modified to fit with STM32CubeIDE environment and HAL APIs. Also it has been ported from C++ to C.

Major difference
----------------
1. The functions have been ported to use HAL API.
2. Language change from C++ to C
3. Added a new function called `Madgwick2`, which is derived from Madgwick's internal report released on 2010. This function uses `zeta`, thus compensating gyro drift form long-term use.
4. Added a new structure `imu_t` for storing IMU data and other parameters in order to easily use multiple IMUs.
