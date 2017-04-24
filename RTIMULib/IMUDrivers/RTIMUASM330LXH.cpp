////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMUASM330LXH.h"
#include "RTIMUSettings.h"

//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

RTIMUASM330LXH::RTIMUASM330LXH(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 100;
}

RTIMUASM330LXH::~RTIMUASM330LXH()
{
}

bool RTIMUASM330LXH::IMUInit()
{
    unsigned char result;

#ifdef ASM330LXH_CACHE_MODE
    m_firstTime = true;
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif
    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_gyroSlaveAddr = m_settings->m_I2CSlaveAddress;

    // work out accelmag address

    if (m_settings->HALRead(ASM330LXH_ACCELMAG_ADDRESS0, ASM330LXH_WHO_AM_I, 1, &result, "")) {
        if (result == ASM300LXH_GYRO_ID) {
            m_accelCompassSlaveAddr = ASM330LXH_ACCELMAG_ADDRESS0;
        }
    } else {
        m_accelCompassSlaveAddr = ASM330LXH_ACCELMAG_ADDRESS1;
    }

    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    //  Set up the gyro

    if (!m_settings->HALWrite(m_gyroSlaveAddr, ASM330LXH_GYRO_CTRL5, 0x80, "Failed to boot ASM330LXH"))
        return false;

    if (!m_settings->HALRead(m_gyroSlaveAddr, ASM330LXH_GYRO_WHO_AM_I, 1, &result, "Failed to read ASM330LXH gyro id"))
        return false;

    if (result != ASM330LXH_GYRO_ID) {
        HAL_ERROR1("Incorrect ASM330LXH gyro id %d\n", result);
        return false;
    }

    if (!setGyroSampleRate())
            return false;

    if (!setGyroCTRL2())
            return false;

    if (!setGyroCTRL4())
            return false;

    //  Set up the accel

    if (!m_settings->HALRead(m_accelCompassSlaveAddr, ASM330LXH_WHO_AM_I, 1, &result, "Failed to read ASM330LXH accel/mag id"))
        return false;

    if (result != ASM330LXH_ACCELMAG_ID) {
        HAL_ERROR1("Incorrect ASM330LXH accel/mag id %d\n", result);
        return false;
    }

    if (!setAccelCTRL1())
        return false;

    if (!setAccelCTRL2())
        return false;


    //  Set up the mag
    //copied from 303D 20H gyro setup
    // Mag is MAG3110

    //Correct variables
    if (!m_settings->HALWrite(m_magSlaveAddr, L3GD20H_LOW_ODR, 0x04, "Failed to reset L3GD20H"))
        return false;

    //Correct variables
    if (!m_settings->HALWrite(m_magSlaveAddr, L3GD20H_CTRL5, 0x80, "Failed to boot L3GD20H"))
        return false;
    //Correct variables
    if (!m_settings->HALRead(m_magSlaveAddr, L3GD20H_WHO_AM_I, 1, &result, "Failed to read L3GD20H id"))
        return false;

    //Correct variables
    if (result != L3GD20H_ID) {
        HAL_ERROR1("Incorrect L3GD20H id %d\n", result);
        return false;
    }

    //Correct variables
    if (!setMagSampleRate())
            return false;

    //Correct variables
        //Create func
    if (!setMagCTRL2())
            return false;

    //Correct variables
        //Create func
    if (!setMagCTRL4())
            return false;

#ifdef ASM330LXH_CACHE_MODE

    //  turn on gyro fifo

    if (!m_settings->HALWrite(m_gyroSlaveAddr, ASM330LXH_GYRO_FIFO_CTRL, 0x3f, "Failed to set ASM330LXH FIFO mode"))
        return false;
#endif

    if (!setGyroCTRL5())
            return false;

    gyroBiasInit();

    HAL_INFO("ASM330LXH init complete\n");
    return true;
}

// TO DO : Align Gyro Sample Rate regs
// DSO : CTRL_REG_1_G (20h)
// ASM : CTRL2_G (11h)



bool RTIMUASM330LXH::setGyroSampleRate()
{
    unsigned char ctrl1;

    switch (m_settings->m_ASM330LXHGyroSampleRate) {
    case ASM330LXH_GYRO_SAMPLERATE_95:
        ctrl1 = 0x0f;
        m_sampleRate = 95;
        break;

    case ASM330LXH_GYRO_SAMPLERATE_190:
        ctrl1 = 0x4f;
        m_sampleRate = 190;
        break;

    case ASM330LXH_GYRO_SAMPLERATE_380:
        ctrl1 = 0x8f;
        m_sampleRate = 380;
        break;

    case ASM330LXH_GYRO_SAMPLERATE_760:
        ctrl1 = 0xcf;
        m_sampleRate = 760;
        break;

    default:
        HAL_ERROR1("Illegal ASM330LXH gyro sample rate code %d\n", m_settings->m_ASM330LXHGyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_ASM330LXHGyroBW) {
    case ASM330LXH_GYRO_BANDWIDTH_0:
        ctrl1 |= 0x00;
        break;

    case ASM330LXH_GYRO_BANDWIDTH_1:
        ctrl1 |= 0x10;
        break;

    case ASM330LXH_GYRO_BANDWIDTH_2:
        ctrl1 |= 0x20;
        break;

    case ASM330LXH_GYRO_BANDWIDTH_3:
        ctrl1 |= 0x30;
        break;

    }

    return (m_settings->HALWrite(m_gyroSlaveAddr, ASM330LXH_GYRO_CTRL1, ctrl1, "Failed to set ASM330LXH gyro CTRL1"));
}

//TO DO : Align Gyro high pass filter
//DSO0:

bool RTIMUASM330LXH::setGyroCTRL2()
{
    if ((m_settings->m_ASM330LXHGyroHpf < ASM330LXH_GYRO_HPF_0) || (m_settings->m_ASM330LXHGyroHpf > ASM330LXH_GYRO_HPF_9)) {
        HAL_ERROR1("Illegal ASM330LXH gyro high pass filter code %d\n", m_settings->m_ASM330LXHGyroHpf);
        return false;
    }
    return m_settings->HALWrite(m_gyroSlaveAddr,  ASM330LXH_GYRO_CTRL2, m_settings->m_ASM330LXHGyroHpf, "Failed to set ASM330LXH gyro CTRL2");
}

//TO DO: Align Gyrro FSR 
// DSO : CTRL_REG4_G (23h)
// ASM : 

bool RTIMUASM330LXH::setGyroCTRL4()
{
    unsigned char ctrl4;

    switch (m_settings->m_ASM330LXHGyroFsr) {
    case ASM330LXH_GYRO_FSR_250:
    // DS0: 245 DPS
        ctrl4 = 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case ASM330LXH_GYRO_FSR_500:
    // DSO : 500 DPS
        ctrl4 = 0x10;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case ASM330LXH_GYRO_FSR_2000:
    // DSO : 2000 DPS
        ctrl4 = 0x20;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;



    default:
        HAL_ERROR1("Illegal ASM330LXH gyro FSR code %d\n", m_settings->m_ASM330LXHGyroFsr);
        return false;
    }

    return m_settings->HALWrite(m_gyroSlaveAddr,  ASM330LXH_GYRO_CTRL4, ctrl4, "Failed to set ASM330LXH gyro CTRL4");
}


//TO DO: Align CTRL5
// DSO: CTRL_REG5_G
// ASM: Mixed, below

bool RTIMUASM330LXH::setGyroCTRL5()
{
    unsigned char ctrl5;

    //  Turn on hpf

    // ASM: REG: CTRL7_G

    ctrl5 = 0x10;

#ifdef ASM330LXH_CACHE_MODE
    //  turn on fifo

    // ASM: Reg: FIFO_CTRL (1Ah)

    ctrl5 |= 0x40;
#endif

    return m_settings->HALWrite(m_gyroSlaveAddr,  ASM330LXH_GYRO_CTRL5, ctrl5, "Failed to set ASM330LXH gyro CTRL5");
}


//TO DO: ALign CTRL1
// DO0: CTRL_REG1_XM(20h)
// ASM: CTRL1_XL (10h)
// Align Hz

bool RTIMUASM330LXH::setAccelCTRL1()
{
    unsigned char ctrl1;

    if ((m_settings->m_ASM330LXHAccelSampleRate < 0) || (m_settings->m_ASM330LXHAccelSampleRate > 10)) {
        HAL_ERROR1("Illegal ASM330LXH accel sample rate code %d\n", m_settings->m_ASM330LXHAccelSampleRate);
        return false;
    }

    ctrl1 = (m_settings->m_ASM330LXHAccelSampleRate << 4) | 0x07;

    return m_settings->HALWrite(m_accelCompassSlaveAddr,  ASM330LXH_CTRL1, ctrl1, "Failed to set ASM330LXH accell CTRL1");
}

//TO DO: Align CTRL2
// DO0: CTRL_REG2_XM(21h)
// ASM: CTRL1_XL(10h)
// Align full scale selectoion and low pass 

bool RTIMUASM330LXH::setAccelCTRL2()
{
    unsigned char ctrl2;

    if ((m_settings->m_ASM330LXHAccelLpf < 0) || (m_settings->m_ASM330LXHAccelLpf > 3)) {
        HAL_ERROR1("Illegal ASM330LXH accel low pass fiter code %d\n", m_settings->m_ASM330LXHAccelLpf);
        return false;
    }

    switch (m_settings->m_ASM330LXHAccelFsr) {
    case ASM330LXH_ACCEL_FSR_2:
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case ASM330LXH_ACCEL_FSR_4:
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case ASM330LXH_ACCEL_FSR_6:
        m_accelScale = (RTFLOAT)0.000183;
        break;

    case ASM330LXH_ACCEL_FSR_8:
        m_accelScale = (RTFLOAT)0.000244;
        break;

    case ASM330LXH_ACCEL_FSR_16:
        m_accelScale = (RTFLOAT)0.000732;
        break;

    default:
        HAL_ERROR1("Illegal ASM330LXH accel FSR code %d\n", m_settings->m_ASM330LXHAccelFsr);
        return false;
    }

    ctrl2 = (m_settings->m_ASM330LXHAccelLpf << 6) | (m_settings->m_ASM330LXHAccelFsr << 3);

    return m_settings->HALWrite(m_accelCompassSlaveAddr,  ASM330LXH_CTRL2, ctrl2, "Failed to set ASM330LXH accel CTRL2");
}




int RTIMUASM330LXH::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUASM330LXH::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    // lol wut compass
    unsigned char compassData[6];


#ifdef ASM330LXH_CACHE_MODE
    int count;

    if (!m_settings->HALRead(m_gyroSlaveAddr, ASM330LXH_GYRO_FIFO_SRC, 1, &status, "Failed to read ASM330LXH gyro fifo status"))
        return false;

    if ((status & 0x40) != 0) {
        HAL_INFO("ASM330LXH gyro fifo overrun\n");
        if (!m_settings->HALWrite(m_gyroSlaveAddr, ASM330LXH_GYRO_CTRL5, 0x10, "Failed to set ASM330LXH gyro CTRL5"))
            return false;

        if (!m_settings->HALWrite(m_gyroSlaveAddr, ASM330LXH_GYRO_FIFO_CTRL, 0x0, "Failed to set ASM330LXH gyro FIFO mode"))
            return false;

        if (!m_settings->HALWrite(m_gyroSlaveAddr, ASM330LXH_GYRO_FIFO_CTRL, 0x3f, "Failed to set ASM330LXH gyro FIFO mode"))
            return false;

        if (!setGyroCTRL5())
            return false;

        m_imuData.timestamp += m_sampleInterval * 32;
        return false;
    }

    // get count of samples in fifo
    count = status & 0x1f;

    if ((m_cacheCount == 0) && (count > 0) && (count < ASM330LXH_FIFO_THRESH)) {
        // special case of a small fifo and nothing cached - just handle as simple read

        if (!m_settings->HALRead(m_gyroSlaveAddr, 0x80 | ASM330LXH_GYRO_OUT_X_L, 6, gyroData, "Failed to read ASM330LXH gyro data"))
            return false;

        if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | ASM330LXH_OUT_X_L_A, 6, accelData, "Failed to read ASM330LXH accel data"))
            return false;

        if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | ASM330LXH_OUT_X_L_M, 6, compassData, "Failed to read ASM330LXH compass data"))
            return false;

        if (m_firstTime)
            m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        else
            m_imuData.timestamp += m_sampleInterval;

        m_firstTime = false;
   } else {
        if (count >=  ASM330LXH_FIFO_THRESH) {
            // need to create a cache block

            if (m_cacheCount == ASM330LXH_CACHE_BLOCK_COUNT) {
                // all cache blocks are full - discard oldest and update timestamp to account for lost samples
                m_imuData.timestamp += m_sampleInterval * m_cache[m_cacheOut].count;
                if (++m_cacheOut == ASM330LXH_CACHE_BLOCK_COUNT)
                    m_cacheOut = 0;
                m_cacheCount--;
            }

            if (!m_settings->HALRead(m_gyroSlaveAddr, 0x80 | ASM330LXH_GYRO_OUT_X_L, ASM330LXH_FIFO_CHUNK_SIZE * ASM330LXH_FIFO_THRESH,
                         m_cache[m_cacheIn].data, "Failed to read ASM330LXH fifo data"))
                return false;

            if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | ASM330LXH_OUT_X_L_A, 6,
                         m_cache[m_cacheIn].accel, "Failed to read ASM330LXH accel data"))
                return false;

            if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | ASM330LXH_OUT_X_L_M, 6,
                         m_cache[m_cacheIn].compass, "Failed to read ASM330LXH compass data"))
                return false;

            m_cache[m_cacheIn].count = ASM330LXH_FIFO_THRESH;
            m_cache[m_cacheIn].index = 0;

            m_cacheCount++;
            if (++m_cacheIn == ASM330LXH_CACHE_BLOCK_COUNT)
                m_cacheIn = 0;

        }

        //  now fifo has been read if necessary, get something to process

        if (m_cacheCount == 0)
            return false;

        memcpy(gyroData, m_cache[m_cacheOut].data + m_cache[m_cacheOut].index, ASM330LXH_FIFO_CHUNK_SIZE);
        memcpy(accelData, m_cache[m_cacheOut].accel, 6);
        memcpy(compassData, m_cache[m_cacheOut].compass, 6);

        m_cache[m_cacheOut].index += ASM330LXH_FIFO_CHUNK_SIZE;

        if (--m_cache[m_cacheOut].count == 0) {
            //  this cache block is now empty

            if (++m_cacheOut == ASM330LXH_CACHE_BLOCK_COUNT)
                m_cacheOut = 0;
            m_cacheCount--;
        }
        if (m_firstTime)
            m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
        else
            m_imuData.timestamp += m_sampleInterval;

        m_firstTime = false;
    }

#else
    if (!m_settings->HALRead(m_gyroSlaveAddr, ASM330LXH_GYRO_STATUS, 1, &status, "Failed to read ASM330LXH status"))
        return false;

    if ((status & 0x8) == 0)
        return false;

    if (!m_settings->HALRead(m_gyroSlaveAddr, 0x80 | ASM330LXH_GYRO_OUT_X_L, 6, gyroData, "Failed to read ASM330LXH gyro data"))
        return false;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | ASM330LXH_OUT_X_L_A, 6, accelData, "Failed to read ASM330LXH accel data"))
        return false;

    if (!m_settings->HALRead(m_accelCompassSlaveAddr, 0x80 | ASM330LXH_OUT_X_L_M, 6, compassData, "Failed to read ASM330LXH compass data"))
        return false;

#endif

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);

    //  sort out gyro axes and correct for bias

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  sort out compass axes

    m_imuData.compass.setY(-m_imuData.compass.y());

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    //  now update the filter

    updateFusion();

    return true;
}
