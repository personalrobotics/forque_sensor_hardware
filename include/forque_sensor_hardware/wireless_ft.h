// Copyright 2023 Personal Robotics Lab, University of Washington
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Author: Ethan K. Gordon
// Based on https://github.com/TAMS-Group/tams_wireless_ft

#ifndef FORQUE_SENSOR_HARDWARE_WIRELESS_FT_H_
#define FORQUE_SENSOR_HARDWARE_WIRELESS_FT_H_

#include <memory>
#include <mutex>
#include <string>

// status bits in UDP data packet, status word 1

// conversion factors, see WirelessFTSensorPanel.java
//
#define CONVERT_FORCE_POUND_LBF 1.0;
#define CONVERT_FORCE_KILOPOUND_KLBF 1000.0;
#define CONVERT_FORCE_NEWTON_N 4.448222;
#define CONVERT_FORCE_KILONEWTON_KN 0.004448222;
#define CONVERT_FORCE_GRAM_G 453.5924;
#define CONVERT_FORCE_KILOGRAM_KG 0.4535924;

#define CONVERT_TORQUE_POUND_INCHES_LBFIN 1.0;
#define CONVERT_TORQUE_POUND_FEET_LBFFT 0.0833333;
#define CONVERT_TORQUE_NEWTON_METER_NM 0.1129848;
#define CONVERT_TORQUE_NEWTON_MILLIMETER_NMM 112.984829;
#define CONVERT_TORQUE_KILOGRAM_CENTIMETER_KGCM 1.15212462;
#define CONVERT_TORQUE_KILONEWTON_METER 0.000112985;

// status bits in UDP data packet, status word 1
//
#define WFT_STATUS1_BRIDGE_VOLTAGE_LOW_T3 (1 << 29)
#define WFT_STATUS1_BRIDGE_VOLTAGE_LOW_T2 (1 << 28)
#define WFT_STATUS1_BRIDGE_VOLTAGE_LOW_T1 (1 << 27)
#define WFT_STATUS1_SATURATED_DATA_T3 (1 << 26)
#define WFT_STATUS1_SATURATED_DATA_T2 (1 << 25)
#define WFT_STATUS1_SATURATED_DATA_T1 (1 << 24)

#define WFT_STATUS1_T3_BRIDGE (1 << 21)
#define WFT_STATUS1_T3_AFE (1 << 20)
#define WFT_STATUS1_T2_BRIDGE (1 << 19)
#define WFT_STATUS1_T2_AFE (1 << 18)
#define WFT_STATUS1_T1_BRIDGE (1 << 17)
#define WFT_STATUS1_T1_AFE (1 << 16)

#define WFT_STATUS1_BATTERY_GREEN (1 << 11)
#define WFT_STATUS1_BATTERY_RED (1 << 10)
#define WFT_STATUS1_EXT_POWER_GREEN (1 << 9)
#define WFT_STATUS1_EXT_POWER_RED (1 << 8)
#define WFT_STATUS1_WLAN_GREEN (1 << 7)
#define WFT_STATUS1_WLAN_RED (1 << 6)
#define WFT_STATUS1_T3_GREEN (1 << 5)
#define WFT_STATUS1_T3_RED (1 << 4)
#define WFT_STATUS1_T2_GREEN (1 << 3)
#define WFT_STATUS1_T2_RED (1 << 2)
#define WFT_STATUS1_T1_GREEN (1 << 1)
#define WFT_STATUS1_T1_RED (1 << 0)

// status bits in UDP data packet, status word 1
//
#define WFT_STATUS2_BRIDGE_VOLTAGE_LOW_T6 (1 << 29)
#define WFT_STATUS2_BRIDGE_VOLTAGE_LOW_T5 (1 << 28)
#define WFT_STATUS2_BRIDGE_VOLTAGE_LOW_T4 (1 << 27)
#define WFT_STATUS2_SATURATED_DATA_T6 (1 << 26)
#define WFT_STATUS2_SATURATED_DATA_T5 (1 << 25)
#define WFT_STATUS2_SATURATED_DATA_T4 (1 << 24)

#define WFT_STATUS2_T6_BRIDGE (1 << 21)
#define WFT_STATUS2_T6_AFE (1 << 20)
#define WFT_STATUS2_T5_BRIDGE (1 << 19)
#define WFT_STATUS2_T5_AFE (1 << 18)
#define WFT_STATUS2_T4_BRIDGE (1 << 17)
#define WFT_STATUS2_T4_AFE (1 << 16)

#define WFT_STATUS2_T6_GREEN (1 << 5)
#define WFT_STATUS2_T6_RED (1 << 4)
#define WFT_STATUS2_T5_GREEN (1 << 3)
#define WFT_STATUS2_T5_RED (1 << 2)
#define WFT_STATUS2_T4_GREEN (1 << 1)
#define WFT_STATUS2_T4_RED (1 << 0)

// Default Port Numbers
#define DEFAULT_TELNET_PORT 23
#define DEFAULT_UDP_PORT 49152

// Other WFT Configs
#define WFT_MAX_RATE 4000
#define WFT_MIN_RATE 5

namespace forque_sensor_hardware
{

// UDP data packet definition
//
#define NUMBER_OF_ANALOG_BOARDS 2
#define NUMBER_OF_TRANSDUCERS 6
#define NUMBER_OF_STRAIN_GAGES 6
#define NUMBER_OF_CALIBRATIONS 3
struct UDPPacket
{
  std::uint32_t timestamp;
  std::uint32_t sequence;
  std::uint32_t statusCode[NUMBER_OF_ANALOG_BOARDS];
  std::uint8_t batteryLevel;
  std::uint8_t transMask;
  std::int32_t sg[NUMBER_OF_TRANSDUCERS][NUMBER_OF_STRAIN_GAGES];
} __attribute__((__packed__));

typedef struct WirelessFTDataPacket
{
  bool valid = false;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds> timestamp;
  bool transducer_present[NUMBER_OF_TRANSDUCERS];
  std::int32_t counts[NUMBER_OF_TRANSDUCERS][NUMBER_OF_STRAIN_GAGES];
} WirelessFTDataPacket;

class WirelessFT
{
public:
  WirelessFT(bool verbose = false);

  // Stop streaming and disconnect/close all sockets.
  ~WirelessFT();

  // Connect to telnet socket on given hostname and port
  // Returns 0 on success, -1 on failure
  bool telnetConnect(std::string hostname, int port = DEFAULT_TELNET_PORT);
  bool telnetDisconnect();

  // Basic Commands

  // rate * oversample cannot exceed 4000
  bool setRate(unsigned int rate, unsigned int oversample = 1);

  // 0 == all transducers
  bool setBias(bool bias = true, unsigned int transducer = 0);

  // Mostly for internal use
  bool telnetCommand(std::string & response, std::string command, unsigned int micros = 500000);

  bool udpConfigure(std::string hostname, int port = DEFAULT_UDP_PORT);
  bool udpClose();
  bool udpStartStreaming();
  bool udpStopStreaming();
  bool udpResetTelnet();

  // Reads a single datagram
  WirelessFTDataPacket readDataPacket();

private:
  unsigned char mUDPCommandSequence;
  unsigned short crcBuf(char * buff, int len);
  unsigned short crcByte(unsigned short crc, char ch);

  bool mIsStreaming;
  bool mVerbose;

  // networking stuff
  int mTelnetSocket;
  int mUDPSocket;

  // thread safety
  std::mutex mTCPMutex;
  std::mutex mUDPMutex;

};  // end class WirelessFT

// String Formatting Utility
template <typename... Args>
std::string string_format(const std::string & format, Args... args)
{
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;  // Extra space for '\0'
  if (size_s <= 0) {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

};  // end namespace forque_sensor_hardware

#endif
