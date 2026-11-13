#include "AltIMU.h"
#include "SocketServer.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace {
constexpr int PORT = 54321;
constexpr unsigned int I2C_BUS = 2; // P9_19 (SCL) and P9_20 (SDA)
} // namespace

int main() {
  std::cout << "Starting Beagle Board Server" << std::endl;
  BB::AltIMU imu(I2C_BUS);

  BB::SocketServer server(PORT);
  if (server.listen() != 0) {
    std::cerr << "Socket Server failed to start." << std::endl;
    return 1;
  }
  std::cout << "Server started. Waiting for connection..." << std::endl;
  if (server.accept() != 0) {
    return 1;
  }
  std::cout << "Client connected." << std::endl;

  // The filter assumes one update every sample period, so keep a fixed rate.
  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<float>(1.0f / BB::AltIMU::SAMPLE_FREQ_HZ));
  auto next_sample = std::chrono::steady_clock::now();
  while (true) {
    next_sample += period;
    std::this_thread::sleep_until(next_sample);

    if (imu.read_sensors_state() < 0) {
      continue;
    }
    std::string quaternionData = imu.getQuaternionString();
    std::cout << "Data: " << quaternionData << std::flush;
    if (server.send(quaternionData) != 0) {
      std::cerr << "Client lost. Waiting for a new connection..." << std::endl;
      if (server.accept() != 0) {
        return 1;
      }
      std::cout << "Client connected." << std::endl;
      next_sample = std::chrono::steady_clock::now();
    }
  }
}
