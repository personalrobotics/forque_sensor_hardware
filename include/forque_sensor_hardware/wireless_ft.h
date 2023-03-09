/* wireless_ft.h - Class for communicating with ATI Wireless F/T
 *
 * Based on https://github.com/TAMS-Group/tams_wireless_ft
 * 
 * This file and its implementation are independent of ROS2.
 * It provides basic configuration (transducer and calibration
 * selection), 
 *
 * (C) 2023, ekgordon@cs.uw.edu
 */

#ifndef FORQUE_SENSOR_HARDWARE_WIRELESS_FT_H_
#define FORQUE_SENSOR_HARDWARE_WIRELESS_FT_H_

// conversion factors, see WirelessFTSensorPanel.java
// 
#define CONVERT_FORCE_POUND_LBF           1.0;
#define CONVERT_FORCE_KILOPOUND_KLBF   1000.0;
#define CONVERT_FORCE_NEWTON_N            4.448222;
#define CONVERT_FORCE_KILONEWTON_KN       0.004448222;
#define CONVERT_FORCE_GRAM_G            453.5924;
#define CONVERT_FORCE_KILOGRAM_KG         0.4535924;

#define CONVERT_TORQUE_POUND_INCHES_LBFIN           1.0;
#define CONVERT_TORQUE_POUND_FEET_LBFFT             0.0833333;
#define CONVERT_TORQUE_NEWTON_METER_NM              0.1129848;
#define CONVERT_TORQUE_NEWTON_MILLIMETER_NMM      112.984829;
#define CONVERT_TORQUE_KILOGRAM_CENTIMETER_KGCM     1.15212462;
#define CONVERT_TORQUE_KILONEWTON_METER             0.000112985;

// status bits in UDP data packet, status word 1
//
#define WFT_STATUS1_BRIDGE_VOLTAGE_LOW_T3     (1 << 29)
#define WFT_STATUS1_BRIDGE_VOLTAGE_LOW_T2     (1 << 28)
#define WFT_STATUS1_BRIDGE_VOLTAGE_LOW_T1     (1 << 27)
#define WFT_STATUS1_SATURATED_DATA_T3         (1 << 26)
#define WFT_STATUS1_SATURATED_DATA_T2         (1 << 25)
#define WFT_STATUS1_SATURATED_DATA_T1         (1 << 24)

#define WFT_STATUS1_T3_BRIDGE                 (1 << 21)
#define WFT_STATUS1_T3_AFE                    (1 << 20)
#define WFT_STATUS1_T2_BRIDGE                 (1 << 19)
#define WFT_STATUS1_T2_AFE                    (1 << 18)
#define WFT_STATUS1_T1_BRIDGE                 (1 << 17)
#define WFT_STATUS1_T1_AFE                    (1 << 16)

#define WFT_STATUS1_BATTERY_GREEN             (1 << 11)
#define WFT_STATUS1_BATTERY_RED               (1 << 10)
#define WFT_STATUS1_EXT_POWER_GREEN           (1 <<  9)
#define WFT_STATUS1_EXT_POWER_RED             (1 <<  8)
#define WFT_STATUS1_WLAN_GREEN                (1 <<  7)
#define WFT_STATUS1_WLAN_RED                  (1 <<  6)
#define WFT_STATUS1_T3_GREEN                  (1 <<  5)
#define WFT_STATUS1_T3_RED                    (1 <<  4)
#define WFT_STATUS1_T2_GREEN                  (1 <<  3)
#define WFT_STATUS1_T2_RED                    (1 <<  2)
#define WFT_STATUS1_T1_GREEN                  (1 <<  1)
#define WFT_STATUS1_T1_RED                    (1 <<  0)

// status bits in UDP data packet, status word 1
//
#define WFT_STATUS2_BRIDGE_VOLTAGE_LOW_T6     (1 << 29)
#define WFT_STATUS2_BRIDGE_VOLTAGE_LOW_T5     (1 << 28)
#define WFT_STATUS2_BRIDGE_VOLTAGE_LOW_T4     (1 << 27)
#define WFT_STATUS2_SATURATED_DATA_T6         (1 << 26)
#define WFT_STATUS2_SATURATED_DATA_T5         (1 << 25)
#define WFT_STATUS2_SATURATED_DATA_T4         (1 << 24)

#define WFT_STATUS2_T6_BRIDGE                 (1 << 21)
#define WFT_STATUS2_T6_AFE                    (1 << 20)
#define WFT_STATUS2_T5_BRIDGE                 (1 << 19)
#define WFT_STATUS2_T5_AFE                    (1 << 18)
#define WFT_STATUS2_T4_BRIDGE                 (1 << 17)
#define WFT_STATUS2_T4_AFE                    (1 << 16)

#define WFT_STATUS2_T6_GREEN                  (1 <<  5)
#define WFT_STATUS2_T6_RED                    (1 <<  4)
#define WFT_STATUS2_T5_GREEN                  (1 <<  3)
#define WFT_STATUS2_T5_RED                    (1 <<  2)
#define WFT_STATUS2_T4_GREEN                  (1 <<  1)
#define WFT_STATUS2_T4_RED                    (1 <<  0)


// Default Port Numbers
#define DEFAULT_TELNET_PORT 23
#define DEFAULT_UDP_PORT 49152

namespace forque_sensor_hardware {

// UDP data packet definition
// 
#define NUMBER_OF_ANALOG_BOARDS 2
#define NUMBER_OF_TRANSDUCERS 6
#define NUMBER_OF_STRAIN_GAGES 6
struct UDPPacket
{
    unsigned long timeStamp;
    unsigned long sequence;
    unsigned long statusCode[NUMBER_OF_ANALOG_BOARDS];
    unsigned char batteryLevel;
    unsigned char transMask;
    signed long sg[NUMBER_OF_TRANSDUCERS][NUMBER_OF_STRAIN_GAGES];
} __attribute__ ((__packed__));

typedef struct WirelessFTDataPacket {
    bool valid = false;
    double timestamp; // s since 1/1/2010
    unsigned long sequence;
    bool transducer_present[NUMBER_OF_TRANSDUCERS];
    signed long counts[NUMBER_OF_TRANSDUCERS][NUMBER_OF_STRAIN_GAGES];
} WirelessFTDataPacket;

class WirelessFT {
  public:
    WirelessFT(bool verbose = false);

    // Stop streaming and disconnect/close all sockets.
    ~WirelessFT();

    // Connect to telnet socket on given hostname and port
    // Returns 0 on success, -1 on failure
    int  telnetConnect( std::string hostname, int port = DEFAULT_TELNET_PORT);
    int  telnetDisconnect();
    int  telnetCommand( std::string & response, std::string command, unsigned int micros = 1000000 );

    int  udpConfigure( std::string hostname, int port = DEFAULT_UDP_PORT);
    int  udpClose();
    int  udpStartStreaming();
    int  udpStopStreaming();

    // Reads a single datagram 
    WirelessFTDataPacket  readDataPacket();

  private:
    unsigned char  udpCommandSequence;
    unsigned short crcBuf( char* buff, int len );
    unsigned short crcByte( unsigned short crc, char ch );

    int active_channels_mask; // bit i <=> channel (i+1) active
    bool verbose;

    // networking stuff
    int telnetSocket;
    int udpSocket;

}; // end class WirelessFT

}; // end namespace forque_sensor_hardware

#endif
