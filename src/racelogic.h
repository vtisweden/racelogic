/*
 * _racelogic.h
 *
 *  Created on: Jan 16, 2015
 *      Author: Bjorn Blissing
 */
#ifndef _RACELOGIC_H_
#define _RACELOGIC_H_

#include <stdint.h>

#ifdef __cplusplus
#include <windows.h>
#include <string>
#include <sstream>
#include <vector>

class RaceLogicDevice {

        enum VBOX3i_Datafields {
            SATELITES	= 0x00000001, // 1 byte unsigned
            TIME		= 0x00000002, // 3 bytes unsigned
            LATITUDE	= 0x00000004, // 4 bytes signed
            LONGITUDE	= 0x00000008, // 4 bytes signed

            VELOCITY			= 0x00000010, // 2 bytes unsigned
            HEADING				= 0x00000020, // 2 bytes unsigned
            HEIGHT				= 0x00000040, // 3 bytes unsigned
            VERTICAL_VELOCITY	= 0x00000080, // 2 bytes signed

            LONG_ACC_GPS	= 0x00000100, // 2 bytes signed
            LAT_ACC_GPS		= 0x00000200, // 2 bytes signed
            BREAK_DISTANCE	= 0x00000400, // 4 bytes unsigned
            DISTANCE		= 0x00000800, // 4 bytes unsigned

            INTERAL_ANALOGUE_CH1 = 0x00001000, // 4 bytes signed
            INTERAL_ANALOGUE_CH2 = 0x00002000, // 4 bytes signed
            INTERAL_ANALOGUE_CH3 = 0x00004000, // 4 bytes signed
            INTERAL_ANALOGUE_CH4 = 0x00008000, // 4 bytes signed

            GLONASS_SATELITES	= 0x00010000, // 1 byte unsigned
            GPS_SATELITES		= 0x00020000,// 1 byte unsigned
            RESERVED1			= 0x00040000, // 2 bytes
            RESERVED2			= 0x00080000, // 2 bytes

            RESERVED3				= 0x00100000, // 2 bytes
            VBOX_SERIAL_NUMBER		= 0x00200000, // 2 bytes unsigned
            KALMAN_FILTER_STATUS	= 0x00400000, // 2 bytes unsigned
            SOLUTION_TYPE			= 0x00800000, // 2 bytes unsigned

            VELOCITY_QUALITY		= 0x01000000, // 4 bytes unsigned
            INTERNAL_TEMPERATURE	= 0x02000000, // 4 bytes signed
            CF_BUFFER_SIZE			= 0x04000000, // 2 bytes unsigned
            RAM_ADDRESS				= 0x08000000, // 3 bytes unsigned

            EVENT_TIME_1		= 0x10000000, // 4 bytes unsigned
            EVENT_TIME_2		= 0x20000000, // 2 bytes unsigned
            BATTERY_1_VOLTAGE	= 0x40000000, // 2 bytes unsigned
            BATTERY_2_VOLTAGE	= 0x80000000 // 2 bytes unsigned
        };

    public:
        enum VBOX3I_ERROR_CODES {
            SUCCESS = 0x0000,
            OPEN_COM_PORT_ERROR = 0x0001,
            CLOSE_COM_PORT_ERROR = 0x0002,
            GET_COM_STATE_ERROR = 0x0003,
            SET_COM_STATE_ERROR = 0x0004,
            SET_COM_TIMEOUT_ERROR = 0x0005,
            PORT_NOT_OPEN_ERROR = 0x0006,
            READ_ERROR = 0x0007,
            CRC_ERROR = 0x0008
        };

        RaceLogicDevice(std::string port);
        virtual ~RaceLogicDevice();
        int32_t open();
        int32_t close();
        int32_t read();
        bool isOpen() const { return m_isOpen; }
        std::string port() const { return m_port; }

        /* get number of satelites */
        uint8_t satelites() const { return m_satelites; };
        /* get number of 10ms ticks since midnight UTC */
        uint32_t time() const { return m_time; }
        /* get latitude */
        double latitude() const { return std::abs(m_latitude / 100000.0); }
        /* get latitude south */
        bool south() const { return (0x80000000 & m_latitude) == 0x80000000; }
        /* get longitude */
        double longitude() const { return std::abs(m_longitude / 100000.0); }
        /* get longitude east */
        bool east() const { return (0x80000000 & m_longitude) == 0x80000000; }
        /* get velocity in meters per second */
        float velocity() const { return 0.51444f*float(m_velocity)/100.0f; }
        /* get heading in degrees from true north */
        float heading() const { return float(m_heading)/100.0f; }
        /* get altitude in meters WGS84 */
        double height() const { return double(m_height)/100.0; }
        /* get vertical velocity in m/s */
        float verticalVelocity() const { return float(m_verticalVelocity)/100.0f; }
        /* get longitude acceleration */
        float longAcc() const { return float(m_longAcc)/100.0f; }
        /* get latitude acceleration */
        float latAcc() const { return float(m_latAcc)/100.0f; }
        /* get brake distance */
        double brakeDistance() const { return double (m_brakeDistance)/12800; }
        /* get distance */
        double distance() const { return double (m_distance)/12800; }
        /* get internal analogue 1 value */
        float internalAnalogue1() const { float f; memcpy(&f, &m_internalAnalogue1, 4); return f; }
        /* get internal analogue 2 value */
        float internalAnalogue2() const { float f; memcpy(&f, &m_internalAnalogue2, 4); return f;; }
        /* get internal analogue 3 value */
        float internalAnalogue3() const { float f; memcpy(&f, &m_internalAnalogue3, 4); return f;; }
        /* get internal analogue 4 value */
        float internalAnalogue4() const { float f; memcpy(&f, &m_internalAnalogue4, 4); return f; }
        /* get number of GLONASS satelites */
        uint8_t glonass() const { return m_glonass; }
        /* get number of GPS satelites */
        uint8_t gps() const { return m_gps; }
        /* get serial number */
        uint16_t serialNumber() const { return m_serialNumber; }
        /* get Kalman filter status */
        uint16_t kalmanFilterStatus() const { return m_kalmanFilterStatus; }
        /* get solution type */
        uint16_t solutionType() const { return m_solutionType; }
        /* get velocity quality */
        uint32_t velocityQuality() const { return m_velocityQuality; }
        /* get internal temperature */
        float internalTemperature() const { float f; memcpy(&f, &m_interalTemperature, 4); return f; }
        /* get CF Buffer size */
        uint16_t cfBufferSize() const { return m_cfBufferSize; }
        /* get RAM address */
        uint32_t ramAddress() const { return m_ramAddress; }
        /* get event time 1 */
        uint32_t eventTime1() const { return m_eventTime1; }
        /* get event time 2 */
        uint16_t eventTime2() const { return m_eventTime2; }
        /* get battery 1 voltage */
        uint16_t battery1Voltage() const { return m_battery1Voltage; }
        /* get battery 2 voltage */
        uint16_t battery2Voltage() const { return m_battery2Voltage; }
        /* get NEWCAN value at position id */
        float newCanValue(const size_t id) const { return (id < newcan_max_length) ? m_newCan[id] : 0.0f; }
		/* get time string */
		std::string timeString() const;
		/* get latitude string */
		std::string latitudeString() const;
		/* get longitude string */
		std::string longitudeString() const;
    protected:
        int32_t readMessage();
        int32_t readVBox3iMessage();
        int32_t readNewCanMessage();
        static int16_t calculateChecksum(char* message, size_t length);

        // Helper functions
        template <class T> static void readValue(T& value, const char* data, size_t& pos);
        static void readValue(char* value, const char* data, size_t& pos, size_t length);
        static std::wstring stringToWide(const std::string& s);
        static std::string intToHexString(int32_t i);
        static std::string charToHexString(char c);
		static std::string degreesFromMinutes(double decimalMinutes);

        // Member variables
        std::string m_port;
        bool m_isOpen;
        HANDLE m_serialHandle;

        // VBox Data
        uint8_t m_satelites; // Number Of Satelites;
        uint32_t m_time; // number of tenths since midnight UTC;
        int32_t m_latitude;
        int32_t m_longitude;
        uint16_t m_velocity;
        uint16_t m_heading;
        int32_t m_height; // Altitude in meters WGS84*100
        int16_t m_verticalVelocity; // m/s
        int16_t m_longAcc;
        int16_t m_latAcc;;
        uint32_t m_brakeDistance;
        uint32_t m_distance;
        uint32_t m_internalAnalogue1;
        uint32_t m_internalAnalogue2;
        uint32_t m_internalAnalogue3;
        uint32_t m_internalAnalogue4;
        uint8_t m_glonass;
        uint8_t m_gps;
        uint16_t m_serialNumber;
        uint16_t m_kalmanFilterStatus;
        uint16_t m_solutionType;
        uint32_t m_velocityQuality;
        uint32_t m_interalTemperature;
        uint16_t m_cfBufferSize;
        uint32_t m_ramAddress;
        uint32_t m_eventTime1;
        uint16_t m_eventTime2;
        uint16_t m_battery1Voltage;
        uint16_t m_battery2Voltage;

        // New Can Data
        enum { newcan_max_length = 32 };
        float m_newCan[newcan_max_length];
};


#else
typedef struct RaceLogicDevice RaceLogicDevice;
#endif /* __cplusplus*/


/* The following defines the c function interface. */
/* It is important to note that the following functions are not generally thread-safe. */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus*/
/* This function grabs a device with the given specifications. It will return NULL if there is any error. */
extern RaceLogicDevice* rl3_get_device(char*);

/* Open device for reading */
extern int32_t rl3_open_device(RaceLogicDevice* device);

/* Close device */
extern int32_t rl3_close_device(RaceLogicDevice* device);

/* Delete device from memory */
extern int32_t rl3_delete_device(RaceLogicDevice* device);

/* Read data from opened device */
extern int32_t rl3_read_data(RaceLogicDevice* device);

/* get number of satelites */
extern uint8_t rl3_get_satelites(RaceLogicDevice* device);
/* get number of GLONASS satelites */
extern uint8_t rl3_get_glonass_satelites(RaceLogicDevice* device);
/* get number of GPS satelites */
extern uint8_t rl3_get_gps_satelites(RaceLogicDevice* device);

/* get number of 10ms ticks since midnight UTC */
extern int32_t rl3_get_time(RaceLogicDevice* device);

/* get latitude */
extern double rl3_get_latitude(RaceLogicDevice* device);
/* get latitude south */
extern uint8_t rl3_get_south(RaceLogicDevice* device);
/* get longitude */
extern double rl3_get_longitude(RaceLogicDevice* device);
/* get longitude east */
extern uint8_t rl3_get_east(RaceLogicDevice* device);

/* get velocity in meters per second */
extern float rl3_get_velocity(RaceLogicDevice* device);
/* get heading in degrees from true north */
extern float rl3_get_heading(RaceLogicDevice* device);
/* get altitude in meters WGS84 */
extern double rl3_get_height(RaceLogicDevice* device);
/* get vertical velocity in m/s */
extern float rl3_get_vertical_velocity(RaceLogicDevice* device);

/* get latitude acceleration */
extern float rl3_get_lat_acc(RaceLogicDevice* device);
/* get longitudinal acceleration */
extern float rl3_get_long_acc(RaceLogicDevice* device);

/* get brake distance */
extern double rl3_get_brake_distance(RaceLogicDevice* device);
/* get distance */
extern double rl3_get_distance(RaceLogicDevice* device);

/* get internal analogue 1 value */
extern float rl3_get_internal_analogue_value1(RaceLogicDevice* device);
/* get internal analogue 2 value */
extern float rl3_get_internal_analogue_value2(RaceLogicDevice* device);
/* get internal analogue 3 value */
extern float rl3_get_internal_analogue_value3(RaceLogicDevice* device);
/* get internal analogue 4 value */
extern float rl3_get_internal_analogue_value4(RaceLogicDevice* device);

/* get serial number */
extern uint16_t rl3_get_serial_number(RaceLogicDevice* device);
/* get Kalman filter status */
extern uint16_t rl3_get_kalman_filter_status(RaceLogicDevice* device);
/* get solution type */
extern uint16_t rl3_get_solution_type(RaceLogicDevice* device);
/* get velocity quality */
extern uint32_t rl3_get_velocity_quality(RaceLogicDevice* device);
/* get internal temperature */
extern float rl3_get_internal_temperature(RaceLogicDevice* device);

/* get CF Buffer size */
extern uint16_t rl3_get_cf_buffer_size(RaceLogicDevice* device);
/* get RAM address */
extern uint32_t rl3_get_ram_address(RaceLogicDevice* device);

/* get event time 1 */
extern uint32_t rl3_get_event_time1(RaceLogicDevice* device);
/* get event time 2 */
extern uint16_t rl3_get_event_time2(RaceLogicDevice* device);

/* get battery 1 voltage */
extern uint16_t rl3_get_battery1_voltage(RaceLogicDevice* device);
/* get battery 2 voltage */
extern uint16_t rl3_get_battery2_voltage(RaceLogicDevice* device);

/* get NEWCAN value at position id */
extern float rl3_get_newcan_value(RaceLogicDevice* device, uint8_t id);

/* get length of formated time string */
extern size_t rl3_get_time_str_len(RaceLogicDevice* device);

/* get time as formatted string */
extern int rl3_get_time_str(RaceLogicDevice* device, char* str, size_t len);

/* get length of formated coordinate string */
extern size_t rl3_get_coordinate_str_len(RaceLogicDevice* device);

/* get latitude as formatted string */
extern int rl3_get_latitude_str(RaceLogicDevice* device, char* str, size_t len);

/* get longitude as formatted string */
extern int rl3_get_longitude_str(RaceLogicDevice* device, char* str, size_t len);
#ifdef __cplusplus
}
#endif /* __cplusplus*/


#endif