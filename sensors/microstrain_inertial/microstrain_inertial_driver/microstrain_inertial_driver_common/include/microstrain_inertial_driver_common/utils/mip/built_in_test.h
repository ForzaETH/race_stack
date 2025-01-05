/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Driver Definition File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
//
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MICROSTRAINIlDrCnUsBtInTt_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_BUILT_IN_TEST_H

#include "mip/definitions/data_system.hpp"

namespace mip::data_system
{

/**
 * \brief Wrapper class to be used by the device specific built in test classes
*/
struct ContinuousBuiltInTest : public BuiltInTest
{
 protected:
  /**
   * \brief non-explicit constructor that allows implicit conversion from a BuiltInTest to a Device BuiltInTest
  */
  ContinuousBuiltInTest(BuiltInTest b)
  {
    memcpy(result, b.result, sizeof(result) / sizeof(result[0]));
  }

  /**
   * \brief Gets the BIT value given the byte in the result and the bit in the byte
   * \param byte_in_result The zero indexed address of the byte in the result to get the value from
   * \param bit_in_byte The zero indexed bit in the byte to get the value from
   * \return true if the bit requested was 1, false if 0
  */
  inline bool getBitInByte(const uint8_t byte_in_result, const uint8_t bit_in_byte)
  {
    return (result[byte_in_result] >> bit_in_byte) & 0x01;
  }
};

/**
 * \brief GQ7 specific built in test
*/
struct Gq7ContinuousBuiltInTest : public ContinuousBuiltInTest
{
  /**
   * \brief non-explicit constructor that allows implicit conversion from a BuiltInTest to a Gq7ContinuousBuiltInTest
  */
  Gq7ContinuousBuiltInTest(BuiltInTest b) : ContinuousBuiltInTest(b) {}

  inline bool systemClockFailure()
  {
    return getBitInByte(0, 0);
  }
  
  inline bool powerFault()
  {
    return getBitInByte(0, 1);
  }
  
  inline bool firmwareFault()
  {
    return getBitInByte(0, 4);
  }
  
  inline bool timingOverload()
  {
    return getBitInByte(0, 5);
  }
  
  inline bool bufferOverrun()
  {
    return getBitInByte(0, 6);
  }
  
  inline bool imuIpcFault()
  {
    return getBitInByte(2, 0) || getBitInByte(2, 1) || getBitInByte(2, 2) || getBitInByte(2, 3);
  }
  
  inline bool filterIpcFault()
  {
    return getBitInByte(2, 4) || getBitInByte(2, 5) || getBitInByte(2, 6) || getBitInByte(2, 7);
  }
  
  inline bool gnssIpcFault()
  {
    return getBitInByte(3, 0) || getBitInByte(3, 1) || getBitInByte(3, 2) || getBitInByte(3, 3);
  }
  
  inline bool imuClockFault()
  {
    return getBitInByte(4, 0);
  }
  
  inline bool imuCommunicationFault()
  {
    return getBitInByte(4, 1);
  }
  
  inline bool imuTimingOverrun()
  {
    return getBitInByte(4, 2);
  }
  
  inline bool imuCalibrationErrorAccel()
  {
    return getBitInByte(4, 4);
  }
  
  inline bool imuCalibrationErrorGyro()
  {
    return getBitInByte(4, 5);
  }
  
  inline bool imuCalibrationErrorMag()
  {
    return getBitInByte(4, 6);
  }
  
  inline bool accelerometerGeneralFault()
  {
    return getBitInByte(5, 0);
  }
  
  inline bool accelerometerOverRange()
  {
    return getBitInByte(5, 1);
  }
  
  inline bool accelerometerSelfTestFail()
  {
    return getBitInByte(5, 2);
  }
  
  inline bool gyroscopeGeneralFault()
  {
    return getBitInByte(5, 4);
  }
  
  inline bool gyroscopeOverRange()
  {
    return getBitInByte(5, 5);
  }
  
  inline bool gyroscopeSelfTestFail()
  {
    return getBitInByte(5, 6);
  }
  
  inline bool magnetometerGeneralFault()
  {
    return getBitInByte(6, 0);
  }
  
  inline bool magnetometerOverRange()
  {
    return getBitInByte(6, 1);
  }
  
  inline bool magnetometerSelfTestFail()
  {
    return getBitInByte(6, 2);
  }
  
  inline bool pressureSensorGeneralFault()
  {
    return getBitInByte(6, 4);
  }
  
  inline bool pressureSensorOverRange()
  {
    return getBitInByte(6, 5);
  }
  
  inline bool pressureSensorSelfTestFail()
  {
    return getBitInByte(6, 6);
  }
  
  inline bool filterClockFault()
  {
    return getBitInByte(8, 0);
  }
  
  inline bool filterHardwareFault()
  {
    return getBitInByte(8, 1);
  }
  
  inline bool filterTimingOverrun()
  {
    return getBitInByte(8, 2);
  }
  
  inline bool filterTimingUnderrun()
  {
    return getBitInByte(8, 3);
  }
  
  inline bool filterCommunicationError()
  {
    return getBitInByte(8, 4) || getBitInByte(8, 5) || getBitInByte(8, 6) || getBitInByte(8, 7);
  }
  
  inline bool gnssClockFault()
  {
    return getBitInByte(12, 0);
  }
  
  inline bool gnssHardwareFault()
  {
    return getBitInByte(12, 1);
  }
  
  inline bool gnssCommunicationError()
  {
    return getBitInByte(12, 2) || getBitInByte(12, 3) || getBitInByte(12, 4);
  }
  
  inline bool gpsTimeFault()
  {
    return getBitInByte(12, 5);
  }
  
  inline bool gnssTimingOverrun()
  {
    return getBitInByte(12, 6);
  }
  
  inline bool gnssReceiver1PowerFault()
  {
    return getBitInByte(13, 0);
  }
  
  inline bool gnssReceiver1Fault()
  {
    return getBitInByte(13, 1);
  }
  
  inline bool gnssAntenna1Shorted()
  {
    return getBitInByte(13, 2);
  }
  
  inline bool gnssAntenna1Open()
  {
    return getBitInByte(13, 3);
  }
  
  inline bool gnssReceiver1SolutionFault()
  {
    return getBitInByte(13, 4);
  }
  
  inline bool gnssReceiver2PowerFault()
  {
    return getBitInByte(13, 5);
  }
  
  inline bool gnssReceiver2Fault()
  {
    return getBitInByte(13, 6);
  }
  
  inline bool gnssAntenna2Shorted()
  {
    return getBitInByte(13, 7);
  }
  
  inline bool gnssAntenna2Open()
  {
    return getBitInByte(14, 0);
  }
  
  inline bool gnssReceiver2SolutionFault()
  {
    return getBitInByte(14, 1);
  }
  
  inline bool rtcmCommunicationFault()
  {
    return getBitInByte(14, 2);
  }
  
  inline bool rtkDongleFault()
  {
    return getBitInByte(14, 3);
  }
};

/**
 * \brief CV7 specific built in test
*/
struct Cv7ContinuousBuiltInTest : public ContinuousBuiltInTest
{
  /**
   * \brief non-explicit constructor that allows implicit conversion from a BuiltInTest to a Cv7ContinuousBuiltInTest
  */
  Cv7ContinuousBuiltInTest(BuiltInTest b) : ContinuousBuiltInTest(b) {}

  inline bool systemClockFailure()
  {
    return getBitInByte(0, 0);
  }

  inline bool powerFault()
  {
    return getBitInByte(0, 1);
  }

  inline bool firmwareFault()
  {
    return getBitInByte(0, 4);
  }

  inline bool timingOverload()
  {
    return getBitInByte(0, 5);
  }

  inline bool bufferOverrun()
  {
    return getBitInByte(0, 6);
  }

  inline bool imuProcessFault()
  {
    return getBitInByte(2, 0);
  }

  inline bool imuDataRateMismatch()
  {
    return getBitInByte(2, 1);
  }

  inline bool imuOverrunDroppedData()
  {
    return getBitInByte(2, 2);
  }

  inline bool imuStuck()
  {
    return getBitInByte(2, 3);
  }

  inline bool filterProcessFault()
  {
    return getBitInByte(2, 4);
  }

  inline bool filterDroppedData()
  {
    return getBitInByte(2, 5);
  }

  inline bool filterRateMismatch()
  {
    return getBitInByte(2, 6);
  }

  inline bool filterStuck()
  {
    return getBitInByte(2, 7);
  }

  inline bool imuClockFault()
  {
    return getBitInByte(4, 0);
  }

  inline bool imuCommunicationFault()
  {
    return getBitInByte(4, 1);
  }

  inline bool imuTimingOverrun()
  {
    return getBitInByte(4, 2);
  }

  inline bool imuCalibrationErrorAccelerometer()
  {
    return getBitInByte(4, 4);
  }

  inline bool imuCalibrationErrorGyroscope()
  {
    return getBitInByte(4, 5);
  }

  inline bool imuCalibrationErrorMagnetometer()
  {
    return getBitInByte(4, 6);
  }

  inline bool accelerometerGeneralFault()
  {
    return getBitInByte(5, 0);
  }

  inline bool accelerometerOverRange()
  {
    return getBitInByte(5, 1);
  }

  inline bool accelerometerSelfTestFail()
  {
    return getBitInByte(5, 2);
  }

  inline bool gyroscopeGeneralFault()
  {
    return getBitInByte(5, 4);
  }

  inline bool gyroscopeOverRange()
  {
    return getBitInByte(5, 5);
  }

  inline bool gyroscopeSelfTestFail()
  {
    return getBitInByte(5, 6);
  }

  inline bool magnetometerGeneralFault()
  {
    return getBitInByte(6, 0);
  }

  inline bool magnetometerOverRange()
  {
    return getBitInByte(6, 1);
  }

  inline bool magnetometerSelfTestFail()
  {
    return getBitInByte(6, 2);
  }

  inline bool pressureSensorGeneralFault()
  {
    return getBitInByte(6, 4);
  }

  inline bool pressureSensorOverRange()
  {
    return getBitInByte(6, 5);
  }

  inline bool pressureSensorSelfTestFail()
  {
    return getBitInByte(6, 6);
  }

  inline bool factoryBitsInvalid()
  {
    return getBitInByte(7, 0);
  }

  inline bool filterFault()
  {
    return getBitInByte(8, 0);
  }

  inline bool filterTimingOverrun()
  {
    return getBitInByte(8, 2);
  }

  inline bool filterTimingUnderrun()
  {
    return getBitInByte(8, 3);
  }
};

}

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_BUILT_IN_TEST_H