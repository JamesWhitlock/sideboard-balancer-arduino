#include <Wire.h>
#include <FastIMU.h>

#include "imu_factory.h"

struct IMUListEntry {
  uint8_t Address;
  uint8_t WhoAmI_Register;
  uint8_t WhoAmI_Value;
  IMUBase* (*maker)(TwoWire& wire);
};

/* Space for the imu is statically allocated to avoid using the heap */
union IMU_Union{
  MPU6050 mpu6050;
  MPU6500 mpu6500;
  ICM20689 icm20689;

  /* As the union is statically allocated by default the first member will be constructed at boot.
     This bool is added to the union so that no imu is constructed by default */
  bool init;
  constexpr IMU_Union() : init{false} {}
  ~IMU_Union() {}
} imu_union;

template <typename T>
IMUBase* make(TwoWire& wire) {
  return static_cast<IMUBase*>(new(&imu_union) T(wire));
}

const IMUListEntry IMUList[] =
{
  { 0x68, 0x75, 0x68, make<MPU6050> },
  { 0x68, 0x75, 0x70, make<MPU6500> },
  { 0x68, 0x75, 0x98, make<ICM20689> }, // MPU6050A
};

IMUBase* GetImu(TwoWire& wire = Wire)
{
  /* Scan through IMU types to try detect an IMU IC */
  for (auto &imutype : IMUList)
  {
    wire.beginTransmission(imutype.Address);
    wire.write(imutype.WhoAmI_Register);
    wire.endTransmission();
    wire.requestFrom(imutype.Address, (uint8_t) 1);

    if (wire.read() == imutype.WhoAmI_Value)
    {
      return imutype.maker(wire);
    }
  }
  return nullptr;
}

std::tuple<float, float, float> QuatToEuler(float q0, float q1, float q2, float q3){
  float roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
  float pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
  float yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
  return std::make_tuple(roll, pitch, yaw);
}