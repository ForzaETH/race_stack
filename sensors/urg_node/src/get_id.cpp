/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <memory>
#include <string>
#include <vector>

#include "urg_node/urg_c_wrapper.hpp"

#include "rclcpp/rclcpp.hpp"

#ifdef _WIN32
  #include <io.h>
  #define pipe(X) _pipe(X, 4096, O_BINARY)
  #define fileno _fileno
  #define dup2 _dup2
  #define read _read
#elif  __linux__ || __unix || __unix__ || __APPLE__
  #include <unistd.h>
#endif

std::vector<std::string> & split(
  const std::string & s, char delim,
  std::vector<std::string> & elems)
{
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> split(const std::string & s, char delim)
{
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    fprintf(
      stderr,
      "usage: getID /dev/ttyACM? [quiet]\n"
      "Outputs the device ID of a hokuyo at /dev/ttyACM? or IP address"
      " (specified as 192.168.1.6:10940). Add a second argument for script friendly output.\n");
    return 1;
  }

  bool verbose = (argc == 2);

  int save_stdout = dup(STDOUT_FILENO);

  if (!verbose) {  // Block urg's prints
    int fds[2];
    int res;
    int so;

    res = pipe(fds);
    assert(res == 0);

    so = fileno(stdout);
    // close stdout handle and make the writable part of fds the new stdout.
    res = dup2(fds[1], so);
    assert(res != -1);
  }

  bool publish_intensity = false;
  bool publish_multiecho = false;
  int serial_baud = 115200;
  int ip_port = 10940;
  std::string ip_address = "";
  std::string serial_port = "";

  std::vector<std::string> ip_split = split(argv[1], ':');
  if (ip_split.size() < 2) {  // Not an IP address
    serial_port = argv[1];
  } else if (ip_split.size() == 2) {  // IP address formatted as IP:PORT
    ip_address = ip_split[0];
    ip_port = atoi(ip_split[1].c_str());
  } else {  // Invalid
    if (verbose) {
      printf("getID failed due to invalid specifier.\n");
      return 1;
    }
  }

  std::shared_ptr<urg_node::URGCWrapper> urg_;

  for (int retries = 10; retries; retries--) {
    // Set up the urgwidget
    try {
      if (ip_address != "") {
        urg_node::EthernetConnection connection{ip_address, ip_port};
        urg_.reset(
          new urg_node::URGCWrapper(
            connection,
            publish_intensity, publish_multiecho));
      } else {
        urg_node::SerialConnection connection{serial_port, serial_baud};
        urg_.reset(
          new urg_node::URGCWrapper(
            connection,
            publish_intensity, publish_multiecho));
      }
      std::string device_id = urg_->getDeviceID();
      if (verbose) {
        if (ip_address != "") {
          printf("Device at %s:%i has ID ", ip_address.c_str(), ip_port);
        } else {
          printf("Device at %s has ID ", serial_port.c_str());
        }
      }
      // Print this in either mode
      fflush(NULL);  // Clear whatever we aren't printing
      dup2(save_stdout, STDOUT_FILENO);  // Restore std::out
      printf("%s\n", device_id.c_str());
      return 0;
    } catch (std::runtime_error & e) {
      printf("getID failed: %s\n", e.what());
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (verbose) {
    printf("getID failed for 10 seconds. Giving up.\n");
    if (ip_address != "") {
      printf("Device at %s:%i\n", ip_address.c_str(), ip_port);
    } else {
      printf("Device at %s\n", serial_port.c_str());
    }
  }
  return 1;
}
