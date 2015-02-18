/*
 * _racelogic.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Bjorn Blissing
 */
#include "racelogic.h"

#ifdef __cplusplus
#include <Winsock2.h>
#include <iostream>
#include <iomanip>
#include <bitset>

// Disable warning about zero initialization of array
#pragma warning( disable : 4351 )

RaceLogicDevice::RaceLogicDevice(std::string port) : m_port(port), m_isOpen(false), m_satelites(0), m_time(0), m_latitude(0), m_longitude(0), m_velocity(0), m_heading(0), m_height(0), m_verticalVelocity(0), m_longAcc(0), m_latAcc(0), m_brakeDistance(0), m_distance(0), m_internalAnalogue1(0), m_internalAnalogue2(0), m_internalAnalogue3(0), m_internalAnalogue4(0), m_glonass(0), m_gps(0), m_serialNumber(0), m_kalmanFilterStatus(0), m_solutionType(0), m_velocityQuality(0), m_interalTemperature(0), m_cfBufferSize(0), m_ramAddress(0), m_eventTime1(0), m_eventTime2(0), m_battery1Voltage(0), m_battery2Voltage(0), m_newCan()  {}

RaceLogicDevice::~RaceLogicDevice()
{
    close();
}

int32_t RaceLogicDevice::open()
{
    std::wstring szPortName = stringToWide(m_port);
    // Open serial port
    m_serialHandle = CreateFileW(szPortName.c_str(), GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

    if ( m_serialHandle == INVALID_HANDLE_VALUE ) {
        std::cerr << "Failed to open COM Port: " << m_port << " Reason: " << GetLastError() << std::endl;
        return OPEN_COM_PORT_ERROR;
    }

    // Do some basic settings
    DCB serialParams = { 0 };
    serialParams.DCBlength = sizeof(serialParams);

    if ( !::GetCommState(m_serialHandle,&serialParams) ) {
        std::cerr << "Failed to Get Comm State! Reason: " << GetLastError() << std::endl;
        return GET_COM_STATE_ERROR;
    }

    serialParams.BaudRate = 115200; // baudrate;
    serialParams.Parity = 0; // no parity;
    serialParams.ByteSize = 8; //data bits;
    serialParams.StopBits = 0; // stopBits;

    if ( !::SetCommState(m_serialHandle, &serialParams) ) {
        std::cerr << "Failed to Set Comm State! Reason: " << GetLastError() << std::endl;
        return SET_COM_STATE_ERROR;
    }

    // Set timeouts
    COMMTIMEOUTS timeout = { 0 };
    timeout.ReadIntervalTimeout = 50;
    timeout.ReadTotalTimeoutConstant = 50;
    timeout.ReadTotalTimeoutMultiplier = 50;
    timeout.WriteTotalTimeoutConstant = 50;
    timeout.WriteTotalTimeoutMultiplier = 10;

    if ( !::SetCommTimeouts(m_serialHandle, &timeout) ) {
        std::cerr << "Failed to Set Comm Timeouts! Reason: " << GetLastError() << std::endl;
        return SET_COM_TIMEOUT_ERROR;
    }

    // If we got here the port opened successfully
    m_isOpen = true;
    return SUCCESS;
}

int32_t RaceLogicDevice::close()
{
    if (m_isOpen) {
        if (::CloseHandle(m_serialHandle) ) {
            m_isOpen = false;
            return SUCCESS;
        } else {
            std::cerr << "Failed to close COM port: " << m_port << " Reason: " << GetLastError() << std::endl;
            return CLOSE_COM_PORT_ERROR;
        }
    }

    return PORT_NOT_OPEN_ERROR;
}

int32_t RaceLogicDevice::read()
{
    if (!m_isOpen) {
        //std::cerr << "COM Port has not been opened correctly" << std::endl;
        return PORT_NOT_OPEN_ERROR;
    }

    return readMessage();
}

int32_t RaceLogicDevice::readMessage()
{
    const unsigned int HEADER_SIZE = 7;
    char header[HEADER_SIZE];
    DWORD dwRead;
    OVERLAPPED osReader = {0};
    // Read data from serial port
    BOOL isRead = ReadFile(m_serialHandle, header, HEADER_SIZE, &dwRead, &osReader);

    if (isRead) {
        // Interpret header
        if (strncmp(header, "$VBOX3i", HEADER_SIZE) == 0) {
            return readVBox3iMessage();
        } else if (strncmp(header, "$NEWCAN", HEADER_SIZE) == 0) {
            return readNewCanMessage();
        } else {
            std::cerr << "Unknown header: " << header << std::endl;
            close();
            return READ_ERROR;
        }
    } else {
        std::cerr << "Reading from serial port failed" << std::endl;
        close();
        return READ_ERROR;
    }
}

int32_t RaceLogicDevice::readVBox3iMessage()
{
    const unsigned int HEADER_SIZE = 10;
    char header[HEADER_SIZE];
    DWORD dwRead;
    OVERLAPPED osReader = {0};
    size_t bufPos = 0;
    // Read data from serial port
    ReadFile(m_serialHandle, header, HEADER_SIZE, &dwRead, &osReader);
    // Comma
    ++bufPos;
    // Channel Presence
    int channel = 0;
    readValue(channel, header, bufPos);
    UINT32 channelHost = ntohl(channel);
    // Zeros
    unsigned long zeros = 0;
    readValue(zeros, header, bufPos);
    // Comma
    ++bufPos;
    // Build buffer to get binary data
    size_t dataSize = 0;

    if (channelHost & SATELITES) {
        dataSize += 1; // 1 Byte
    }

    if (channelHost & TIME) {
        dataSize += 3; // 3 Byte
    }

    if (channelHost & LATITUDE) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & LONGITUDE) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & VELOCITY) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & HEADING) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & HEIGHT) {
        dataSize += 3; // 3 Byte
    }

    if (channelHost & VERTICAL_VELOCITY) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & LONG_ACC_GPS) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & LAT_ACC_GPS) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & BREAK_DISTANCE) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & DISTANCE) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & INTERAL_ANALOGUE_CH1) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & INTERAL_ANALOGUE_CH2) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & INTERAL_ANALOGUE_CH3) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & INTERAL_ANALOGUE_CH4) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & GLONASS_SATELITES) {
        dataSize += 1; // 1 Byte
    }

    if (channelHost & GPS_SATELITES) {
        dataSize += 1; // 1 Byte
    }

    if (channelHost & RESERVED1) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & RESERVED2) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & RESERVED3) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & VBOX_SERIAL_NUMBER) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & KALMAN_FILTER_STATUS) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & SOLUTION_TYPE) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & VELOCITY_QUALITY) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & INTERNAL_TEMPERATURE) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & CF_BUFFER_SIZE) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & RAM_ADDRESS) {
        dataSize += 3; // 3 Byte
    }

    if (channelHost & EVENT_TIME_1) {
        dataSize += 4; // 4 Byte
    }

    if (channelHost & EVENT_TIME_2) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & BATTERY_1_VOLTAGE) {
        dataSize += 2; // 2 Byte
    }

    if (channelHost & BATTERY_2_VOLTAGE) {
        dataSize += 2; // 2 Byte
    }

    // Checksum
    dataSize += 2; // 2 Byte
    char* data = new char[dataSize]();
    // Read data from serial port
    ReadFile(m_serialHandle, data, dataSize, &dwRead, &osReader);
    // Interpret data
    bufPos = 0;

    if (channelHost & SATELITES) {
        readValue(m_satelites, data, bufPos);
    }

    if (channelHost & TIME) {
        m_time = 0;
        memcpy(&m_time, &(data[bufPos]), 3); // Special: 3 bytes read into 4 byte int.
        m_time = (m_time << 8);
        m_time = ntohl(m_time);
        bufPos += 3;
    }

    if (channelHost & LATITUDE) {
        readValue(m_latitude, data, bufPos);
        m_latitude = ntohl(m_latitude);
    }

    if (channelHost & LONGITUDE) {
        readValue(m_longitude, data, bufPos);
        m_longitude = ntohl(m_longitude);
    }

    if (channelHost & VELOCITY) {
        readValue(m_velocity, data, bufPos);
        m_velocity = ntohs(m_velocity);
    }

    if (channelHost & HEADING) {
        readValue(m_heading, data, bufPos);
        m_heading = ntohs(m_heading);
    }

    if (channelHost & HEIGHT) {
        memcpy(&m_height, &(data[bufPos]), 3); // Special: 3 bytes read into 4 byte int.
        m_height = (m_height << 8);
        m_height = ntohl(m_height);

        // Convert true signed 24-bit to signed int
        if (m_height & 0x800000) {
            m_height |= ~0xffffff;
        }

        bufPos += 3;
    }

    if (channelHost & VERTICAL_VELOCITY) {
        readValue(m_verticalVelocity, data, bufPos);
        m_verticalVelocity = ntohs(m_verticalVelocity);
    }

    if (channelHost & LONG_ACC_GPS) {
        readValue(m_longAcc, data, bufPos);
        m_longAcc = ntohs(m_longAcc);
    }

    if (channelHost & LAT_ACC_GPS) {
        readValue(m_latAcc, data, bufPos);
        m_latAcc = ntohs(m_latAcc);
    }

    if (channelHost & BREAK_DISTANCE) {
        readValue(m_brakeDistance, data, bufPos);
        m_brakeDistance = ntohl(m_brakeDistance);
    }

    if (channelHost & DISTANCE) {
        readValue(m_distance, data, bufPos);
        m_distance = ntohl(m_distance);
    }

    if (channelHost & INTERAL_ANALOGUE_CH1) {
        readValue(m_internalAnalogue1, data, bufPos);
        m_internalAnalogue1 = ntohl(m_internalAnalogue1);
    }

    if (channelHost & INTERAL_ANALOGUE_CH2) {
        readValue(m_internalAnalogue2, data, bufPos);
        m_internalAnalogue2 = ntohl(m_internalAnalogue2);
    }

    if (channelHost & INTERAL_ANALOGUE_CH3) {
        readValue(m_internalAnalogue3, data, bufPos);
        m_internalAnalogue3 = ntohl(m_internalAnalogue3);
    }

    if (channelHost & INTERAL_ANALOGUE_CH4) {
        readValue(m_internalAnalogue4, data, bufPos);
        m_internalAnalogue4 = ntohl(m_internalAnalogue4);
    }

    if (channelHost & GLONASS_SATELITES) {
        readValue(m_glonass, data, bufPos);
    }

    if (channelHost & GPS_SATELITES) {
        readValue(m_gps, data, bufPos);
    }

    if (channelHost & RESERVED1) {
        bufPos += 2;
    }

    if (channelHost & RESERVED2) {
        bufPos += 2;
    }

    if (channelHost & RESERVED3) {
        bufPos += 2;
    }

    if (channelHost & VBOX_SERIAL_NUMBER) {
        readValue(m_serialNumber, data, bufPos);
        m_serialNumber = ntohs(m_serialNumber);
    }

    if (channelHost & KALMAN_FILTER_STATUS) {
        readValue(m_kalmanFilterStatus, data, bufPos);
        m_kalmanFilterStatus = ntohs(m_kalmanFilterStatus);
    }

    if (channelHost & SOLUTION_TYPE) {
        readValue(m_solutionType, data, bufPos);
        m_solutionType = ntohs(m_solutionType);
    }

    if (channelHost & VELOCITY_QUALITY) {
        readValue(m_velocityQuality, data, bufPos);
        m_velocityQuality = ntohl(m_velocityQuality);
    }

    if (channelHost & INTERNAL_TEMPERATURE) {
        readValue(m_interalTemperature, data, bufPos);
        m_interalTemperature = ntohl(m_interalTemperature);
    }

    if (channelHost & CF_BUFFER_SIZE) {
        readValue(m_cfBufferSize, data, bufPos);
        m_cfBufferSize = ntohs(m_cfBufferSize);
    }

    if (channelHost & RAM_ADDRESS) {
        memcpy(&m_ramAddress, &(data[bufPos]), 3); // Special: 3 bytes read into 4 byte int.
        m_ramAddress = (m_ramAddress << 8);
        m_ramAddress = ntohl(m_ramAddress);
        bufPos += 3;
    }

    if (channelHost & EVENT_TIME_1) {
        readValue(m_eventTime1, data, bufPos);
        m_eventTime1 = ntohl(m_eventTime1);
    }

    if (channelHost & EVENT_TIME_2) {
        readValue(m_eventTime2, data, bufPos);
        m_eventTime2 = ntohs(m_eventTime2);
    }

    if (channelHost & BATTERY_1_VOLTAGE) {
        readValue(m_battery1Voltage, data, bufPos);
        m_battery1Voltage = ntohs(m_battery1Voltage);
    }

    if (channelHost & BATTERY_2_VOLTAGE) {
        readValue(m_battery2Voltage, data, bufPos);
        m_battery2Voltage = ntohs(m_battery2Voltage);
    }

    // Read checksum
    short crc = 0;
    readValue(crc, data, bufPos);
    short checkSum = ntohs(crc);
    // Validate checksum
    const size_t MESSAGE_HEADER_SIZE = 7;
    size_t messageLength = MESSAGE_HEADER_SIZE+HEADER_SIZE+dataSize-2;
    char* message = new char[messageLength]();
    char messageHeader[]= "$VBOX3i";
    memcpy(message, messageHeader, MESSAGE_HEADER_SIZE);
    memcpy(message+MESSAGE_HEADER_SIZE, header, HEADER_SIZE);
    memcpy(message+MESSAGE_HEADER_SIZE+HEADER_SIZE, data, dataSize-2);
    short myCrc = calculateChecksum(message, messageLength);
    delete[] message;
    delete[] data;

    if (checkSum != myCrc) {
        std::cerr << "Warning: Message CheckSum did not match!" << std::endl;
        return CRC_ERROR;
    }

    return SUCCESS;
}

int32_t RaceLogicDevice::readNewCanMessage()
{
    // Clear the old previous NEWCAN data
    for (size_t i = 0; i < newcan_max_length; ++i) {
        m_newCan[i] = 0.0f;
    }

    // Read the packet
    const unsigned int HEADER_SIZE = 6;
    char header[HEADER_SIZE];
    DWORD dwRead;
    OVERLAPPED osReader = {0};
    size_t bufPos = 0;
    // Read data from serial port
    ReadFile(m_serialHandle, header, HEADER_SIZE, &dwRead, &osReader);
    // Comma
    ++bufPos;
    // Byte groups to read
    UINT32 bytesToRead = 0;
    readValue(bytesToRead, header, bufPos);
    int bytesToReadHost = int(ntohl(bytesToRead));
    std::bitset<32> byteGroup(bytesToReadHost);
    size_t dataSize = 4*byteGroup.count();
    // Comma
    ++bufPos;
    // Checksum
    dataSize += 2; // 2 Byte
    char* data = new char[dataSize]();
    // Read data from serial port
    ReadFile(m_serialHandle, data, dataSize, &dwRead, &osReader);
    size_t i = 0;

    for (bufPos = 0; bufPos < dataSize - 2; ++i) {
        float value = 0.0;
        readValue(value, data, bufPos);
        m_newCan[i] = value;
    }

    // Read checksum
    short crc = 0;
    readValue(crc, data, bufPos);
    short checkSum = ntohs(crc);
    // Validate checksum
    const size_t MESSAGE_HEADER_SIZE = 7;
    size_t messageLength = MESSAGE_HEADER_SIZE+HEADER_SIZE+dataSize-2;
    char* message = new char[messageLength]();
    char messageHeader[]= "$NEWCAN";
    memcpy(message, messageHeader, MESSAGE_HEADER_SIZE);
    memcpy(message+MESSAGE_HEADER_SIZE, header, HEADER_SIZE);
    memcpy(message+MESSAGE_HEADER_SIZE+HEADER_SIZE, data, dataSize-2);
    short myCrc = calculateChecksum(message, messageLength);
    delete[] message;
    delete[] data;

    if (checkSum != myCrc) {
        std::cerr << "Warning: Message CheckSum did not match!" << std::endl;
        return CRC_ERROR;
    }

    return SUCCESS;
}

short RaceLogicDevice::calculateChecksum(char* message, size_t length)
{
    int crc = 0;
    short polynomial = 0x00001021; // 4129 Magic number from CRC algorithm

    for (size_t loop = 0; loop < length; ++loop) {
        char temp = message[loop];
        crc = crc ^ (int(temp)*0x00000100); // 256
        crc = crc % 0x00010000; // mod 65536

        for (size_t i = 8; i > 0; --i) {
            if ((crc & 0x00008000) == 0x00008000) { // 32768
                crc = crc * 2;
                crc = crc ^ polynomial;
            } else {
                crc = crc * 2;
            }

            crc = crc % 0x00010000; // mod 65536
        }
    }

    return short(crc);
}

std::string RaceLogicDevice::timeString() const {
	uint32_t t = time();
	int hours = (t / 100 / 60 / 60) % 24;
	int minutes = (t / 100 / 60) % 60;
	int seconds = (t / 100 ) % 60;
	int hundredths = t % 100;
	std::stringstream ss;
	ss << std::setw(2) << std::setfill ('0') << hours;
	ss <<  ":" << std::setw(2) << std::setfill ('0') << minutes;
	ss <<  ":" << std::setw(2) << std::setfill ('0') << seconds;
	ss <<  ":" << std::setw(2) << std::setfill ('0') << hundredths;
	return ss.str();
}

std::string RaceLogicDevice::latitudeString() const {
	std::stringstream ss;
	char latitudeChar = south() ? 'S' : 'N';
	ss << degreesFromMinutes(latitude()) << " " << latitudeChar;
	return ss.str();
}

std::string RaceLogicDevice::longitudeString() const {
	std::stringstream ss;
	char longitudeChar = east() ? 'E' : 'W';
	ss << degreesFromMinutes(longitude()) << " " << longitudeChar;
	return ss.str();
}

template <class T> void RaceLogicDevice::readValue(T& value, const char* data, size_t& pos)
{
    memcpy(&value, &(data[pos]), sizeof(value));
    pos += sizeof(value);
}
void RaceLogicDevice::readValue(char* value, const char* data, size_t& pos, size_t length)
{
    memcpy(value, &data[pos], length);
    pos += length;
}
std::wstring RaceLogicDevice::stringToWide(const std::string& s)
{
    int len;
    int slength = (int)s.length() + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}
std::string RaceLogicDevice::intToHexString(int i)
{
    std::stringstream stream;
    stream << "0x"	<< std::setfill ('0') << std::setw(8)  << std::hex << (unsigned long long) i;
    return stream.str();
}
std::string RaceLogicDevice::charToHexString(char c)
{
    std::stringstream stream;
    stream << "0x" << std::hex << std::setfill ('0') << std::setw(2) << (unsigned int)(unsigned char)(c);
    return stream.str();
}

std::string RaceLogicDevice::degreesFromMinutes(double decimalMinutes)
{
	std::stringstream ss;
	double degrees = floor(decimalMinutes / 60);
	double minutes = floor(decimalMinutes - degrees*60);
	double seconds = floor((decimalMinutes - degrees*60 - minutes)*1E6);
	ss << std::setfill ('0') << std::setw(2) << degrees << "\370" << std::setw(2) << minutes << "," << std::setw(6) << seconds;
	return ss.str();
}

// c-wrapped functions
RaceLogicDevice* rl3_get_device(char* port) { return new RaceLogicDevice(std::string(port)); }
int32_t rl3_open_device(RaceLogicDevice* device) { return device->open() == 0 ? EXIT_SUCCESS : EXIT_FAILURE; }
int32_t rl3_close_device(RaceLogicDevice* device) { return device->close() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;}
int32_t rl3_delete_device(RaceLogicDevice* device) { delete device; return EXIT_SUCCESS;}
int32_t rl3_read_data(RaceLogicDevice* device) { return device->read() == 0 ? EXIT_SUCCESS : EXIT_FAILURE; }
uint8_t rl3_get_satelites(RaceLogicDevice* device) { return device->satelites(); }
uint8_t rl3_get_glonass_satelites(RaceLogicDevice* device) { return device->glonass(); }
uint8_t rl3_get_gps_satelites(RaceLogicDevice* device) { return device->gps(); }
int32_t rl3_get_time(RaceLogicDevice* device) { return device->time(); }
double rl3_get_latitude(RaceLogicDevice* device) { return device->latitude(); }
uint8_t rl3_get_south(RaceLogicDevice* device) { return device->south(); }
double rl3_get_longitude(RaceLogicDevice* device) { return device->longitude(); }
uint8_t rl3_get_east(RaceLogicDevice* device) { return device->east(); }
float rl3_get_velocity(RaceLogicDevice* device) { return device->velocity(); }
float rl3_get_heading(RaceLogicDevice* device) { return device->heading(); }
double rl3_get_height(RaceLogicDevice* device) { return device->height(); }
float rl3_get_vertical_velocity(RaceLogicDevice* device) { return device->verticalVelocity(); }
float rl3_get_lat_acc(RaceLogicDevice* device) { return device->latAcc(); }
float rl3_get_long_acc(RaceLogicDevice* device) { return device->longAcc(); }
double rl3_get_brake_distance(RaceLogicDevice* device) { return device->brakeDistance(); }
double rl3_get_distance(RaceLogicDevice* device) { return device->distance(); }
float rl3_get_internal_analogue_value1(RaceLogicDevice* device) { return device->internalAnalogue1(); }
float rl3_get_internal_analogue_value2(RaceLogicDevice* device) { return device->internalAnalogue2(); }
float rl3_get_internal_analogue_value3(RaceLogicDevice* device) { return device->internalAnalogue3(); }
float rl3_get_internal_analogue_value4(RaceLogicDevice* device) { return device->internalAnalogue4(); }
uint16_t rl3_get_serial_number(RaceLogicDevice* device) { return device->serialNumber();  }
uint16_t rl3_get_kalman_filter_status(RaceLogicDevice* device) { return device->kalmanFilterStatus(); }
uint16_t rl3_get_velocity_type(RaceLogicDevice* device) { return device->solutionType(); }
uint16_t rl3_get_solution_type(RaceLogicDevice* device) { return device->solutionType(); }
uint32_t rl3_get_velocity_quality(RaceLogicDevice* device) { return device->velocityQuality(); }
float rl3_get_internal_temperature(RaceLogicDevice* device) { return device->internalTemperature(); }
uint16_t rl3_get_cf_buffer_size(RaceLogicDevice* device) { return device->cfBufferSize(); }
uint32_t rl3_get_ram_address(RaceLogicDevice* device) { return device->ramAddress(); }
uint32_t rl3_get_event_time1(RaceLogicDevice* device) { return device->eventTime1(); }
uint16_t rl3_get_event_time2(RaceLogicDevice* device) { return device->eventTime2(); }
uint16_t rl3_get_battery1_voltage(RaceLogicDevice* device) { return device->battery1Voltage(); }
uint16_t rl3_get_battery2_voltage(RaceLogicDevice* device) { return device->battery2Voltage(); }
float rl3_get_newcan_value(RaceLogicDevice* device, uint8_t id) { return device->newCanValue(size_t(id)); }
size_t rl3_get_time_str_len(RaceLogicDevice* device) { return device->timeString().size() + 1; }
size_t rl3_get_coordinate_str_len(RaceLogicDevice* device) { return std::max(device->latitudeString().size() + 1, device->longitudeString().size() + 1); }

int rl3_get_time_str(RaceLogicDevice* device, char* str, size_t len) { 
	std::string time = device->timeString();
	if (len < time.size()) return EXIT_FAILURE;
	strcpy_s(str, time.size() + 1, time.c_str());
	return EXIT_SUCCESS;
}

int rl3_get_latitude_str(RaceLogicDevice* device, char* str, size_t len) { 
	std::string latitude = device->latitudeString();
	if (len < latitude.size()) return EXIT_FAILURE;
	strcpy_s(str, latitude.size() + 1, latitude.c_str());
	return EXIT_SUCCESS;
}
int rl3_get_longitude_str(RaceLogicDevice* device, char* str, size_t len) { 
	std::string longitude = device->longitudeString();
	if (len < longitude.size()) return EXIT_FAILURE;
	strcpy_s(str, longitude.size() + 1, longitude.c_str());
	return EXIT_SUCCESS;
}
#endif