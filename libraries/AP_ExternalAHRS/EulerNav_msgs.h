#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_EULERNAV_ENABLED

#include <cstdint>
#include <AP_Common/AP_Common.h>


static constexpr uint8_t EULERNAV_MESSAGE_MARKER_1 = 0x4E; // 'N'
static constexpr uint8_t EULERNAV_MESSAGE_MARKER_2 = 0x45; // 'E'
static constexpr uint8_t EULERNAV_CHECKSUM_LENGTH = 4;

static constexpr uint32_t EULERNAV_INIT_CRC = 0xFFFFFFFF;

static constexpr size_t EulerNav_AlignTo32Bit(size_t len) {
    return (4 - (len % 4)) % 4;
}

struct PACKED EulerNav_ProtocolHeader {
        const uint8_t marker1 = EULERNAV_MESSAGE_MARKER_1;
        const uint8_t marker2 = EULERNAV_MESSAGE_MARKER_2;
        const uint16_t version = 0x02;
};

enum class EulerNavMessageType: uint8_t {
    InertialOutput = 0x01,
    NavigationOutput = 0x02,
    AccuracyInfo = 0x03,
    TimeNavigation = 0x04,
    TimeInertial = 0x05,
    TimeSync = 0x06,
    Version = 0x0F
};

struct PACKED EulerNav_inertial_output_data {
        EulerNav_ProtocolHeader header;
        const EulerNavMessageType messageType = EulerNavMessageType::InertialOutput;
        uint8_t sequenceCounter;
        int16_t specificForceX;
        int16_t specificForceY;
        int16_t specificForceZ;
        int16_t angularRateX;
        int16_t angularRateY;
        int16_t angularRateZ;
        uint8_t validityFlags;
        // Validity flags (0 - invalid, 1 - valid):
        // Bit 0: specific force X
        // Bit 1: specific force Y
        // Bit 2: specific force Z
        // Bit 3: angular rate X
        // Bit 4: angular rate Y
        // Bit 5: angular rate Z
        // Bits 6 to 7: reserved

        // Specific force in [m/s^2] = 1.495384e-3 * (integer value).
        // Angular rate in [rad/s] = 1.597921e-4 * (integer value).
};

struct PACKED EulerNav_navigation_output_data {
        EulerNav_ProtocolHeader header;
        const EulerNavMessageType messageType = EulerNavMessageType::NavigationOutput; // Message type for BAHRS navigation output data
        uint8_t sequenceCounter;
        int16_t pressureHeight;
        int16_t velocityDownwards;
        int16_t rollAngle;
        int16_t pitchAngle;
        uint16_t magneticHeading;
        uint8_t validityFlags;
//        Height in meters = (0.16784924 * (integer value) – 1000).
//        Velocity in meters per second = 9.155413e-3 * (integer value).
//        Angle in radians = 9.587526e-5 * (integer value)
};

struct PACKED EulerNav_accuracy_info_data {
    EulerNav_ProtocolHeader header;
    const EulerNavMessageType messageType = EulerNavMessageType::AccuracyInfo; // Message type for BAHRS accuracy information
    uint8_t sequenceCounter;
    uint16_t attitudeStdDevN;
    uint16_t attitudeStdDevE;
    uint16_t magneticHeadingStdDev;
    uint64_t microcontrollerTime;
};
struct PACKED EulerNav_time_navigation_data {
    EulerNav_ProtocolHeader header;
    const EulerNavMessageType messageType = EulerNavMessageType::TimeNavigation; // Message type for Time of navigation data
    uint8_t sequenceCounter;
    uint8_t navigationDataSequenceCounter;
    uint64_t microcontrollerTime;
};
struct PACKED EulerNav_time_inertial_data {
    EulerNav_ProtocolHeader header;
    const EulerNavMessageType messageType = EulerNavMessageType::TimeInertial; // Message type for Time of inertial data
    uint8_t sequenceCounter;
    uint8_t inertialDataSequenceCounter;
    uint64_t microcontrollerTime;
};
struct PACKED EulerNav_time_sync_data {
    EulerNav_ProtocolHeader header;
    const EulerNavMessageType messageType = EulerNavMessageType::TimeSync; // Message type for Time of the latest synchronization pulse
    uint8_t sequenceCounter;
    uint64_t microcontrollerTime;
};
struct PACKED EulerNav_version_data {
    EulerNav_ProtocolHeader header;
    const EulerNavMessageType messageType = EulerNavMessageType::Version; // Message type for Software version
    char projectCode[3]; // Assuming ASCII encoding
    uint16_t majorVersion;
    uint16_t minorVersion;
};

// Calculate the size of each message type
constexpr size_t InertialOutputSize = sizeof(EulerNav_inertial_output_data);
constexpr size_t NavigationOutputSize = sizeof(EulerNav_navigation_output_data);
constexpr size_t AccuracyInfoSize = sizeof(EulerNav_accuracy_info_data);
constexpr size_t TimeNavigationSize = sizeof(EulerNav_time_navigation_data);
constexpr size_t TimeInertialSize = sizeof(EulerNav_time_inertial_data);
constexpr size_t TimeSyncSize = sizeof(EulerNav_time_sync_data);
constexpr size_t VersionSize = sizeof(EulerNav_version_data);

// Calculate the size of the padding for each message type
constexpr size_t InertialOutputPaddingSize = EulerNav_AlignTo32Bit(InertialOutputSize);
constexpr size_t NavigationOutputPaddingSize = EulerNav_AlignTo32Bit(NavigationOutputSize);
constexpr size_t AccuracyInfoPaddingSize = EulerNav_AlignTo32Bit(AccuracyInfoSize);
constexpr size_t TimeNavigationPaddingSize = EulerNav_AlignTo32Bit(TimeNavigationSize);
constexpr size_t TimeInertialPaddingSize = EulerNav_AlignTo32Bit(TimeInertialSize);
constexpr size_t TimeSyncPaddingSize = EulerNav_AlignTo32Bit(TimeSyncSize);
constexpr size_t VersionPaddingSize = EulerNav_AlignTo32Bit(VersionSize);

// Return the total size of the message from the message type without padding
// todo make constexpr when c++ > 14
size_t EulerNavMessageSize(EulerNavMessageType messageType);

// Return the size of the padding for the message type
size_t EulerNavMessagePaddingSize(EulerNavMessageType messageType);

#endif // AP_EXTERNAL_AHRS_EULERNAV_ENABLED
