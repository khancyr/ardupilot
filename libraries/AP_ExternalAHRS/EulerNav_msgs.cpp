#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_EULERNAV_ENABLED

#include <cstdint>
#include <AP_Common/AP_Common.h>
#include <AP_ExternalAHRS/EulerNav_msgs.h>

size_t EulerNavMessageSize(EulerNavMessageType messageType) {
    switch (messageType) {
        case EulerNavMessageType::InertialOutput:
            return InertialOutputSize;
        case EulerNavMessageType::NavigationOutput:
            return NavigationOutputSize;
        case EulerNavMessageType::AccuracyInfo:
            return AccuracyInfoSize;
        case EulerNavMessageType::TimeNavigation:
            return TimeNavigationSize;
        case EulerNavMessageType::TimeInertial:
            return TimeInertialSize;
        case EulerNavMessageType::TimeSync:
            return TimeSyncSize;
        case EulerNavMessageType::Version:
            return VersionSize;
        default:
            return 0;
    }
}

size_t EulerNavMessagePaddingSize(EulerNavMessageType messageType) {
    switch (messageType) {
        case EulerNavMessageType::InertialOutput:
            return InertialOutputPaddingSize;
        case EulerNavMessageType::NavigationOutput:
            return NavigationOutputPaddingSize;
        case EulerNavMessageType::AccuracyInfo:
            return AccuracyInfoPaddingSize;
        case EulerNavMessageType::TimeNavigation:
            return TimeNavigationPaddingSize;
        case EulerNavMessageType::TimeInertial:
            return TimeInertialPaddingSize;
        case EulerNavMessageType::TimeSync:
            return TimeSyncPaddingSize;
        case EulerNavMessageType::Version:
            return VersionPaddingSize;
        default:
            return 0;
    }
}

#endif // AP_EXTERNAL_AHRS_EULERNAV_ENABLED
