#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "MadgwickAHRS.h"

#define LSM6DS33_ADDR 0x6B
#define LIS3MDL_ADDR 0x1E

#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL9_XL 0x18
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22

#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

#define OUTX_L_M 0x28
#define OUTX_H_M 0x29
#define OUTY_L_M 0x2A
#define OUTY_H_M 0x2B
#define OUTZ_L_M 0x2C
#define OUTZ_H_M 0x2D

// Sensitivities from the datasheets for the ranges set below.
#define LA_SCALE (0.061f * 9.81f * 0.001f)               // m/s^2 per LSB, ±2 g
#define G_SCALE (8.75f * 0.001f * 3.14159265f / 180.0f) // rad/s per LSB, ±245 dps
#define M_SCALE (1.0f / 6842.0f)                         // gauss per LSB, ±4 gauss

// Must match sampleFreq (50 Hz) in MadgwickAHRS.c.
#define SAMPLE_PERIOD_NS 20000000L

int selectDevice(int file, int address) {
  if (ioctl(file, I2C_SLAVE, address) < 0) {
    perror("Failed to select I2C device");
    return 1;
  }
  return 0;
}

int writeRegister(int file, unsigned char reg, unsigned char value) {
  unsigned char buffer[2] = {reg, value};
  if (write(file, buffer, 2) != 2) {
    perror("Failed to write to register");
    return 1;
  }
  return 0;
}

unsigned char readRegister(int file, unsigned char reg) {
  unsigned char buffer[1] = {reg};
  if (write(file, buffer, 1) != 1) {
    perror("Failed to set register address");
    return 1;
  }
  unsigned char value[1];
  if (read(file, value, 1) != 1) {
    perror("Failed to read register value");
    return 1;
  }
  return value[0];
}

short combineValues(unsigned char high, unsigned char low) {
  return ((short)high << 8) | (short)low;
}

short readAxis(int file, unsigned char high_reg, unsigned char low_reg) {
  return combineValues(readRegister(file, high_reg),
                       readRegister(file, low_reg));
}

// Sets the ranges and rates from Task 4: 52 Hz, ±2 g, ±245 dps, ±4 gauss.
// CTRL_REG3 = 0 starts the magnetometer; its power-on default is power-down.
int configureSensors(int file) {
  return selectDevice(file, LSM6DS33_ADDR) ||
         writeRegister(file, CTRL1_XL, 0x30) ||
         writeRegister(file, CTRL2_G, 0x30) ||
         writeRegister(file, CTRL9_XL, 0x38) ||
         selectDevice(file, LIS3MDL_ADDR) ||
         writeRegister(file, CTRL_REG1, 0x0C) ||
         writeRegister(file, CTRL_REG2, 0x00) ||
         writeRegister(file, CTRL_REG3, 0x00);
}

int main() {
  int file;
  printf("Starting the IMU sensor application\n");
  if ((file = open("/dev/i2c-2", O_RDWR)) < 0) {
    perror("Failed to open I2C bus");
    return 1;
  }
  if (configureSensors(file) != 0) {
    close(file);
    return 1;
  }

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  while (1) {
    if (selectDevice(file, LSM6DS33_ADDR) != 0) {
      break;
    }
    float gx = readAxis(file, OUTX_H_G, OUTX_L_G) * G_SCALE;
    float gy = readAxis(file, OUTY_H_G, OUTY_L_G) * G_SCALE;
    float gz = readAxis(file, OUTZ_H_G, OUTZ_L_G) * G_SCALE;
    float ax = readAxis(file, OUTX_H_XL, OUTX_L_XL) * LA_SCALE;
    float ay = readAxis(file, OUTY_H_XL, OUTY_L_XL) * LA_SCALE;
    float az = readAxis(file, OUTZ_H_XL, OUTZ_L_XL) * LA_SCALE;

    if (selectDevice(file, LIS3MDL_ADDR) != 0) {
      break;
    }
    float mx = readAxis(file, OUTX_H_M, OUTX_L_M) * M_SCALE;
    float my = readAxis(file, OUTY_H_M, OUTY_L_M) * M_SCALE;
    float mz = readAxis(file, OUTZ_H_M, OUTZ_L_M) * M_SCALE;

    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
    printf("Quaternion - q0=%.6f q1=%.6f q2=%.6f q3=%.6f\n", q0, q1, q2, q3);

    // Sleep to an absolute time so the rate stays fixed.
    next.tv_nsec += SAMPLE_PERIOD_NS;
    if (next.tv_nsec >= 1000000000L) {
      next.tv_nsec -= 1000000000L;
      next.tv_sec += 1;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
  }
  close(file);
  return 1;
}
