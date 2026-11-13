#ifndef LIS3MDL_H_
#define LIS3MDL_H_

#include "I2CDevice.h"

#define LIS3MDL_BUFFER_SIZE 0x34

namespace BB {

class LIS3MDL : protected I2CDevice {
private:
  unsigned int I2CBus, I2CAddress;
  unsigned char registers[LIS3MDL_BUFFER_SIZE];

  short mx, my, mz;

  short combine_registers(unsigned char msb, unsigned char lsb);

public:
  LIS3MDL(unsigned int I2CBus, unsigned int I2CAddress = 0x1E);
  virtual int read_sensor_state();

  virtual short get_mx() { return mx; }
  virtual short get_my() { return my; }
  virtual short get_mz() { return mz; }

  virtual void debug();
  virtual void display();
  virtual ~LIS3MDL();
};
} /* namespace BB */

#endif /* LIS3MDL_H_ */
