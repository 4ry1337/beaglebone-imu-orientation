#include "SocketClient.h"
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr const char *SERVER_IP = "192.168.7.2";
constexpr int PORT = 54321;

// Prints one "q0,q1,q2,q3" message.
void print_quaternion(const std::string &line) {
  std::stringstream ss(line);
  std::string token;
  std::vector<float> qs;

  while (getline(ss, token, ',')) {
    try {
      qs.push_back(stof(token));
    } catch (const std::exception &e) {
      std::cerr << "Parsing error for token: " << token << std::endl;
    }
  }

  if (qs.size() == 4) {
    std::cout << "q0=" << qs[0] << ", q1=" << qs[1] << ", q2=" << qs[2]
              << ", q3=" << qs[3] << std::endl;
  } else {
    std::cout << "Incomplete data received: " << line << std::endl;
  }
}
} // namespace

int main() {
  std::cout << "Starting Beagle Board Client Example" << std::endl;
  BB::SocketClient client(SERVER_IP, PORT);
  if (client.connectToServer() != 0) {
    std::cerr << "Error: Failed to connect to server." << std::endl;
    return 1;
  }
  std::cout << "Connected to server" << std::endl;

  // TCP is a byte stream: one read can hold part of a message or several.
  std::string pending;
  while (true) {
    std::string data = client.receive(1024);
    if (data.empty()) {
      break;
    }
    pending += data;

    size_t end;
    while ((end = pending.find('\n')) != std::string::npos) {
      print_quaternion(pending.substr(0, end));
      pending.erase(0, end + 1);
    }
  }

  client.disconnectFromServer();

  std::cout << "End of Beagle Board Client" << std::endl;
  return 0;
}
