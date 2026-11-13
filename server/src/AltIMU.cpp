#include "AltIMU.h"
#include <iostream>
#include <cmath>
#include <ostream>
#include <sstream>

using namespace BB;

namespace {
// Sensitivities from the datasheets for the ranges set in the CTRL registers.
constexpr float MG_PER_LSB = 0.061f;        // LSM6DS33 at ±2 g
constexpr float MDPS_PER_LSB = 8.75f;       // LSM6DS33 at ±245 dps
constexpr float LSB_PER_GAUSS = 6842.0f;    // LIS3MDL at ±4 gauss
constexpr float STANDARD_GRAVITY = 9.81f;   // m/s^2
constexpr float DEG_TO_RAD = 3.14159265358979f / 180.0f;

constexpr float ACCEL_SCALE = MG_PER_LSB * 0.001f * STANDARD_GRAVITY; // m/s^2
// Madgwick integrates the gyro rate, so it must be in rad/s.
constexpr float GYRO_SCALE = MDPS_PER_LSB * 0.001f * DEG_TO_RAD;
constexpr float MAG_SCALE = 1.0f / LSB_PER_GAUSS; // gauss

constexpr float BETA = 0.1f; // 2 * proportional gain
constexpr float SAMPLE_PERIOD_S = 1.0f / AltIMU::SAMPLE_FREQ_HZ;
} // namespace

AltIMU::AltIMU(unsigned int I2CBus, unsigned int imu_address,
               unsigned int mag_address)
    : imu_sensor(I2CBus, imu_address), mag_senseor(I2CBus, mag_address) {
  this->beta = BETA;
  this->q0 = 1.0f;
  this->q1 = 0.0f;
  this->q2 = 0.0f;
  this->q3 = 0.0f;
}

int AltIMU::read_sensors_state() {
  if (this->imu_sensor.read_sensor_state() < 0) {
    std::cerr << "LSM6DS33: Failed to read sensor state" << std::endl;
    return -1;
  }
  if (this->mag_senseor.read_sensor_state() < 0) {
    std::cerr << "LIS3MDL: Failed to read sensor state" << std::endl;
    return -1;
  }
  this->calculate_quaternions();
  return 0;
}

void AltIMU::calculate_quaternions() {
  float gx = this->imu_sensor.get_gx() * GYRO_SCALE;
  float gy = this->imu_sensor.get_gy() * GYRO_SCALE;
  float gz = this->imu_sensor.get_gz() * GYRO_SCALE;

  float ax = this->imu_sensor.get_ax() * ACCEL_SCALE;
  float ay = this->imu_sensor.get_ay() * ACCEL_SCALE;
  float az = this->imu_sensor.get_az() * ACCEL_SCALE;

  float mx = this->mag_senseor.get_mx() * MAG_SCALE;
  float my = this->mag_senseor.get_my() * MAG_SCALE;
  float mz = this->mag_senseor.get_mz() * MAG_SCALE;
  MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
}

void AltIMU::MadgwickAHRSupdate(float gx, float gy, float gz, float ax,
                                float ay, float az, float mx, float my,
                                float mz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
      _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
      q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
  // magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
         _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
         my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
           _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3 + _2bz * q1) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2 + _2bz * q0) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3 - _4bz * q1) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2 - _2bz * q0) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1 + _2bz * q3) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0 - _4bz * q2) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3 + _2bz * q1) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0 + _2bz * q2) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * SAMPLE_PERIOD_S;
  q1 += qDot2 * SAMPLE_PERIOD_S;
  q2 += qDot3 * SAMPLE_PERIOD_S;
  q3 += qDot4 * SAMPLE_PERIOD_S;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AltIMU::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax,
                                   float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * SAMPLE_PERIOD_S;
  q1 += qDot2 * SAMPLE_PERIOD_S;
  q2 += qDot3 * SAMPLE_PERIOD_S;
  q3 += qDot4 * SAMPLE_PERIOD_S;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

float AltIMU::invSqrt(float x) { return 1.0f / std::sqrt(x); }

void AltIMU::debug() {
  std::cout << "IMU: ";
  this->imu_sensor.debug();
  std::cout << "\nMagnetometer: ";
  this->mag_senseor.debug();
  std::cout << std::flush;
}

void AltIMU::display() {
  std::cout << "Accelerometer\t" << this->imu_sensor.get_ax() << "\t"
            << this->imu_sensor.get_ay() << "\t" << this->imu_sensor.get_az()
            << "\n"
            << "Gyro\t" << this->imu_sensor.get_gx() << "\t"
            << this->imu_sensor.get_gy() << "\t" << this->imu_sensor.get_gz()
            << "\n"
            << "Magnet\t" << this->mag_senseor.get_mx() << "\t"
            << this->mag_senseor.get_my() << "\t" << this->mag_senseor.get_mz()
            << "\n\n"
            << std::flush;
}

std::string AltIMU::getQuaternionString() {
  std::ostringstream oss;
  // The newline marks the end of a message on the TCP stream.
  oss << this->q0 << "," << this->q1 << "," << this->q2 << "," << this->q3
      << "\n";
  return oss.str();
}

AltIMU::~AltIMU() {}
