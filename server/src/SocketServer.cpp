/*
 * SocketServer.cpp  Created on: 13 Jul 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 * Made available for the book "Exploring BeagleBone"
 * See: www.exploringbeaglebone.com
 * Licensed under the EUPL V.1.1
 *
 * This Software is provided to You under the terms of the European
 * Union Public License (the "EUPL") version 1.1 as published by the
 * European Union. Any use of this Software, other than as authorized
 * under this License is strictly prohibited (to the extent such use
 * is covered by a right of the copyright holder of this Software).
 *
 * This Software is provided under the License on an "AS IS" basis and
 * without warranties of any kind concerning the Software, including
 * without limitation merchantability, fitness for a particular purpose,
 * absence of defects or errors, accuracy, and non-infringement of
 * intellectual property rights other than copyright. This disclaimer
 * of warranty is an essential part of the License and a condition for
 * the grant of any rights to this Software.
 *
 * For more details, see http://www.derekmolloy.ie/
 */

#include "SocketServer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
using namespace std;

namespace BB {

SocketServer::SocketServer(int portNumber) {
  this->socketfd = -1;
  this->clientSocketfd = -1;
  this->portNumber = portNumber;
}

int SocketServer::listen() {
  this->socketfd = socket(AF_INET, SOCK_STREAM, 0);
  if (this->socketfd < 0) {
    perror("Socket Server: error opening socket.\n");
    return 1;
  }
  // Lets a restarted server bind while the old socket is still in TIME_WAIT.
  int reuse = 1;
  if (setsockopt(this->socketfd, SOL_SOCKET, SO_REUSEADDR, &reuse,
                 sizeof(reuse)) < 0) {
    perror("Socket Server: error setting SO_REUSEADDR.\n");
    return 1;
  }
  bzero((char *)&serverAddress, sizeof(serverAddress));
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = INADDR_ANY;
  serverAddress.sin_port = htons(this->portNumber);
  if (bind(socketfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) <
      0) {
    perror("Socket Server: error on binding the socket.\n");
    return 1;
  }
  if (::listen(this->socketfd, 5) < 0) {
    perror("Socket Server: error listening on the socket.\n");
    return 1;
  }
  return 0;
}

int SocketServer::accept() {
  this->closeClient();
  socklen_t clientLength = sizeof(this->clientAddress);
  this->clientSocketfd = ::accept(
      this->socketfd, (struct sockaddr *)&this->clientAddress, &clientLength);
  if (this->clientSocketfd < 0) {
    perror("Socket Server: Failed to bind the client socket properly.\n");
    return 1;
  }
  return 0;
}

int SocketServer::send(std::string message) {
  const char *writeBuffer = message.data();
  size_t remaining = message.length();
  while (remaining > 0) {
    // MSG_NOSIGNAL: a closed client must return an error, not kill the process
    // with SIGPIPE.
    ssize_t n = ::send(this->clientSocketfd, writeBuffer, remaining,
                       MSG_NOSIGNAL);
    if (n < 0) {
      perror("Socket Server: error writing to server socket.");
      return 1;
    }
    writeBuffer += n;
    remaining -= n;
  }
  return 0;
}

string SocketServer::receive(int size) {
  string readBuffer(size, '\0');
  ssize_t n = read(this->clientSocketfd, &readBuffer[0], size);
  if (n < 0) {
    perror("Socket Server: error reading from server socket.");
    n = 0;
  }
  readBuffer.resize(n);
  return readBuffer;
}

void SocketServer::closeClient() {
  if (this->clientSocketfd >= 0) {
    close(this->clientSocketfd);
    this->clientSocketfd = -1;
  }
}

SocketServer::~SocketServer() {
  this->closeClient();
  if (this->socketfd >= 0) {
    close(this->socketfd);
  }
}

} // namespace BB
