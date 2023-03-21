#include <forque_sensor_hardware/wireless_ft.h>
#include <netdb.h>  // gethostbyname
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <chrono>
#include <cmath>
#include <format>
#include <mutex>
#include <string>

namespace forque_sensor_hardware
{

static inline void verbosePrint(bool verbose, std::string str)
{
  if (verbose) {
    std::cout << "[WirelessFT] " << str << std::endl;
  }
}

static inline void errorPrint(std::string str) { std::cerr << "[WirelessFT] " << str << std::endl; }

WirelessFT::WirelessFT(bool verbose)
: mIsStreaming(false), mVerbose(verbose), mTelnetSocket(-1), mUDPSocket(-1)
{
  verbosePrint(verbose, "Init");
  mUDPCommandSequence = 0;
}

WirelessFT::~WirelessFT()
{
  udpStopStreaming();
  udpClose();
  telnetDisconnect();
}

bool WirelessFT::telnetConnect(std::string hostname, int port)
{
  std::lock_guard<std::mutex> guard(mTCPMutex);
  verbosePrint(mVerbose, "telnetConnect");

  if (mTelnetSocket >= 0) {
    verbosePrint(mVerbose, "Socket already exists.");
    return true;
  }

  // Initialize Host + Port Info
  struct hostent * server = gethostbyname(hostname.c_str());
  if (server == NULL) {
    errorPrint(std::string("No such host: ") + hostname);
    return false;
  }

  struct sockaddr_in serv_addr;
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

  // Initialize Socket
  mTelnetSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (mTelnetSocket < 0) {
    errorPrint("Cannot init socket");
    return false;
  }

  // Connect to Socket
  if (connect(mTelnetSocket, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    errorPrint(
      std::format("error connecting to socket at host '{}' port '{}'", hostname.c_str(), port));
    close(mTelnetSocket);
    mTelnetSocket = -1;
    return false;
  }

  // Set NoDelay Socket Option
  int nodelay = 1;  // 1=on, 0=off
  int result = setsockopt(telnetSocket, IPPROTO_TCP, TCP_NODELAY, (char *)&nodelay, sizeof(int));
  if (result < 0) {
    errorPrint("Warning: failed to set TCP_NODELAY on the socket");
  }

  return true;
}

bool WirelessFT::telnetDisconnect()
{
  verbosePrint(mVerbose, "telnetDisconnect");
  std::lock_guard<std::mutex> guard(mTCPMutex);

  try {
    if (mTelnetSocket >= 0) {
      ::shutdown(mTelnetSocket, SHUT_RDWR);
      close(mTelnetSocket);
      mTelnetSocket = -1;
    }
  } catch (...) {
    return false;
  }

  return true;
}

bool WirelessFT::telnetCommand(std::string & response, std::string command, unsigned int micros)
{
  if (mTelnetSocket < 0) {
    response = "";
    return false;
  }

  // Check for and add carriage return
  if (command.substr(command.size() - 2) != "\r\n") {
    command += "\r\n";
  }

  try {
    verbosePrint(mVerbose, std::format("sending telnet command '{}'", cmd.c_str()));

    // size is big enough for all documented Wireless FT data packets
    char buffer[2048];
    response = "";

    int n;
    strncpy(buffer, command.c_str(), 2047);
    n = send(mTelnetSocket, buffer, strlen(buffer), MSG_NOSIGNAL);
    if (n < 0) {
      errorPrint("Error writing to telnet socket.");
      response = "";
      return false;
    } else {
      verbosePrint(mVerbose, std::format("socket write: sent {} bytes, ok.", n));
    }

    // sleep a bit to give the device time to respond
    usleep(micros);

    // read Wireless FT response
    bzero(buffer, 2048);
    n = recv(telnetSocket, buffer, 2047, MSG_WAITALL);
    if (n < 0) {
      errorPrint("Error reading from telnet socket");
      response = "";
      return false;
    } else {
      verbosePrint(mVerbose, std::format("telnet socket read: got {} bytes.", n));
      response = std::string(buffer);
    }

    verbosePrint(mVerbose, std::format("telnet response: '{}'", response.c_str()));
  } catch (...) {
    response = "";
    return false;
  }

  return true;
}

bool WirelessFT::setRate(unsigned int rate, unsigned int oversample)
{
  std::string response;
  std::string command = std::format("rate {} {}\r\n", rate, oversample);

  if (rate * oversample > WFT_MAX_RATE) {
    errorPrint("Cannot request ADC rate over WFT_MAX_RATE");
    return false;
  }

  return telnetCommand(response, command);
}

bool WirelessFT::setBias(bool bias, unsigned int transducer = 0)
{
  std::string command, response;

  if (transducer > NUMBER_OF_TRANSDUCERS) {
    errorPrint("Out of bounds transducer for bias.")
  } else if (transducer == 0) {
    command = std::format("bias * {}\r\n", rate, bias ? "ON" : "OFF");
  } else {
    command = std::format("bias {} {}\r\n", transducer, bias ? "ON" : "OFF");
  }

  return telnetCommand(response, command);
}

bool WirelessFT::udpConfigure(std::string hostname, int port)
{
  std::lock_guard<std::mutex> guard(mUDPMutex);
  verbosePrint(mVerbose, "udpConfigure");

  if (mUDPSocket >= 0) {
    verbosePrint(mVerbose, "Socket already exists.");
    return true;
  }

  // Initialize Host + Port Info
  struct hostent * server = gethostbyname(hostname.c_str());
  if (server == NULL) {
    errorPrint(std::string("No such host: ") + hostname);
    return false;
  }

  struct sockaddr_in serv_addr;
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(port);
  bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);

  // Initialize Socket
  mUDPSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (mUDPSocket < 0) {
    errorPrint("Cannot init socket");
    return false;
  }

  // Connect to Socket
  if (connect(mUDPSocket, (sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
    errorPrint(
      std::format("error connecting to socket at host '{}' port '{}'", hostname.c_str(), port));
    close(mUDPSocket);
    mUDPSocket = -1;
    return false;
  }

  return true;
}

bool WirelessFT::udpClose()
{
  verbosePrint(mVerbose, "udpClose");

  if (mIsStreaming) {
    udpStopStreaming();
  }

  std::lock_guard<std::mutex> guard(mUDPMutex);

  try {
    if (mUDPMutex >= 0) {
      ::shutdown(mUDPSocket, SHUT_RDWR);
      close(mUDPSocket);
      mUDPSocket = -1;
    }
  } catch (...) {
    return false;
  }

  return true;
}

bool WirelessFT::udpStartStreaming()
{
  verbosePrint(mVerbose, "udpStartStreaming");
  // bytes  start streaming:
  //  2     length including crc = 10
  //  1     sequence number, 0,1,2,...255,0,...
  //  1     value 1 = start_streaming_command
  //  4     number_of_packets, 0 for infinite
  //  2     crc

  char buffer[10];
  buffer[0] = 0;
  buffer[1] = (unsigned char)(0x00FF & 10);  // length
  buffer[2] = mUDPCommandSequence++;         // seq-number
  buffer[3] = (unsigned char)1;              // start_streaming
  buffer[4] = (unsigned char)0;              // number of packets
  buffer[5] = (unsigned char)0;
  buffer[6] = (unsigned char)0;  // 10
  buffer[7] = (unsigned char)0;  // 255;

  unsigned short crc = crcBuf(buffer, 8);
  buffer[8] = (unsigned char)(crc >> 8);
  buffer[9] = (unsigned char)(crc & 0x00ff);

  unsigned int length = 10;
  int n = send(udpSocket, buffer, length, MSG_NOSIGNAL);
  verbosePrint(mVerbose, std::format("udpStartStreaming: wrote {} bytes", n));
  mIsStreaming = (n == length) || mIsStreaming;

  return (n == length);
}

bool WirelessFT::udpStopStreaming()
{
  verbosePrint(mVerbose, "udpStopStreaming");
  // bytes  stop streaming:
  //  2     length including crc = 6
  //  1     sequence number, 0,1,2,...255,0,...
  //  1     value 1 = start_streaming_command
  //  2     crc

  char buffer[6];
  buffer[0] = 0;
  buffer[1] = (unsigned char)(0x00FF & 6);  // length including crc
  buffer[2] = mUDPCommandSequence++;        // seq-number
  buffer[3] = (unsigned char)2;             // stop_streaming

  unsigned short crc = crcBuf(buffer, 4);
  buffer[4] = (unsigned char)(crc >> 8);
  buffer[5] = (unsigned char)(crc & 0x00ff);

  unsigned int length = 6;
  int n = send(udpSocket, buffer, length, MSG_NOSIGNAL);
  verbosePrint(mVerbose, std::format("udpStopStreaming: wrote {} bytes", n));
  mIsStreaming = false;

  return (n == length);
}

/**
 * CRC checksum calculation, converted from ATi's crc.java
 * @author Sam Skuce
 */
unsigned short WirelessFT::crcByte(unsigned short crc, char ch)
{
  static int ccitt_crc16_table[32 * 8] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};
  return (short)(ccitt_crc16_table[((crc >> 8) ^ ch) & 0xff] ^ (crc << 8));
}

unsigned short WirelessFT::crcBuf(char * buff, int len)
{
  int i;
  short crc = 0x1234;  // CRC seed.

  for (i = 0; i < len; i++) {
    crc = crcByte(crc, buff[i]);
  }
  return crc;
}

#define NTP_TO_UNIX 2208988800L
static std::chrono::time_point<std::chrono::system_clock> toSystemTimestamp(unsigned long timestamp)
{
  // Convert device time to duration
  double secs = (double)(timestamp >> 12);
  for (int i = 0; i < 12; i++) {
    secs += ((timestamp & 0xFFF) >> (11 - i)) * pow(0.5, i + 1)
  }
  std::chrono::duration<double> deviceTime(secs);

  // Get Last 2^20 rollover point in Unix Time
  std::uint64_t seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();
  auto deviceBase =
    std::chrono::system_clock::now() -
    std::chrono::seconds((seconds_since_epoch % (1 << 20)) + (NTP_TO_UNIX % (1 << 20)));

  return deviceBase + deviceTime;
}

WirelessFTDataPacket WirelessFT::readDataPacket()
{
  WirelessFTDataPacket ret;
  if (!mIsStreaming) return ret;

  // Read packet from socket
  UDPPacket buffer;
  bzero(buffer, sizeof(UDPPacket));

  int n = recv(udpSocket, buffer, sizeof(UDPPacket), MSG_WAITALL);
  if (n < 0) {
    errorPrint("Cannot read UDP Data Packet");
    return ret;
  }

  // Convert timestamp to system time
  ret.timestamp = toSystemTimestamp(buffer.timestamp);

  // See what transducers are available
  std::vector<int> transducers;
  for (int i = 0; i < NUMBER_OF_TRANSDUCERS; i++) {
    bool status = buffer.transMask & (1 << i);
    ret.transducer_present[i] = status;
    if (status) transducers.push_back(i);
  }

  // Copy data over
  for (int i = 0; i < transducers.size(); i++) {
    memcpy(
      &(ret.counts[transducers[i]][0]), &(buffer.sg[i][0]),
      sizeof(signed long) * NUMBER_OF_STRAIN_GAGES);
  }

  ret.valid = true;
  return ret;
}

};  // end namespace forque_sensor_hardware
