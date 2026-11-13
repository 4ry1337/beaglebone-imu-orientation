#ifndef ALTIMU_H_
#define ALTIMU_H_

#include "LIS3MDL.h"
#include "LSM6DS33.h"
#include <string>

namespace BB {

class AltIMU {
private:
  LSM6DS33 imu_sensor;
  LIS3MDL mag_senseor;
  float beta;
  float q0, q1, q2, q3;

  void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay,
                          float az, float mx, float my, float mz);
  void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay,
                             float az);
  float invSqrt(float x);
  void calculate_quaternions();

public:
  // Sensors output at 52 Hz, so the caller must read slightly slower than that.
  static constexpr float SAMPLE_FREQ_HZ = 50.0f;

  AltIMU(unsigned int I2CBus, unsigned int imu_address = 0x6B,
         unsigned int mag_address = 0x1E);
  virtual int read_sensors_state();
  virtual void debug();
  virtual void display();
  virtual ~AltIMU();
  virtual std::string getQuaternionString();
};
} // namespace BB

#endif // !ALTIMU_H_
