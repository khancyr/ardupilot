/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Support for EulerNav serially connected AHRS Systems
  Usage in SITL with hardware for debugging:
    $ sim_vehicle.py -v Rover -A "--serial3=uart:/dev/EulerNav" --console --map -DG
    $ ./Tools/autotest/sim_vehicle.py -v Rover -A "--serial3=uart:/dev/EulerNav" -DG
    param set AHRS_EKF_TYPE 11
    param set EAHRS_TYPE 11
    param set SERIAL3_BAUD 115
    param set SERIAL3_PROTOCOL 36
  UDEV rules for repeatable USB connection:
    $ cat /etc/udev/rules.d/99-usb-serial.rules
    SUBSYSTEM=="tty", ATTRS{manufacturer}=="Lord Microstrain", SYMLINK+="EulerNav"
  Usage with simulated MicroStrain7:
    ./Tools/autotest/sim_vehicle.py -v Rover -A "--serial3=sim:EulerNav" --console --map -DG
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_EULERNAV_ENABLED

#include "AP_ExternalAHRS_EulerNav.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_EulerNav::AP_ExternalAHRS_EulerNav(AP_ExternalAHRS *_frontend,
                                                           AP_ExternalAHRS::state_t &_state) :
        AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "EulerNav ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    // don't offer IMU by default, at 200Hz it is too slow for many aircraft

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_EulerNav::update_thread, void), "EulerNav", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("EulerNav Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "EulerNav ExternalAHRS initialised");
}

void AP_ExternalAHRS_EulerNav::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        build_packet();
        hal.scheduler->delay_microseconds(100);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_EulerNav::build_packet()
{
    if (uart == nullptr) {
        return;
    }

    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        uint8_t b;
        if (!uart->read(b)) {
            break;
        }
        EulerNavMessageType messageType;
        handle_byte(b, messageType);
    }
}

void AP_ExternalAHRS_EulerNav::handle_byte(const uint8_t b, EulerNavMessageType& messageType)
{
    switch (message_in.state) {
        case ParseState::WaitingFor_SyncOne:
            if (b == EULERNAV_MESSAGE_MARKER_1) {
                message_in.state = ParseState::WaitingFor_SyncTwo;
            }
            break;
        case ParseState::WaitingFor_SyncTwo:
            if (b == EULERNAV_MESSAGE_MARKER_2) {
                message_in.state = ParseState::WaitingFor_Version;
            } else {
                message_in.state = ParseState::WaitingFor_SyncOne;
            }
            break;
        case ParseState::WaitingFor_Version:
            if (b == 0x02) {
                message_in.state = ParseState::WaitingFor_MessageType;
            } else {
                message_in.state = ParseState::WaitingFor_SyncOne;
            }
            break;

        case ParseState::WaitingFor_MessageType: {
            const auto msgType = static_cast<EulerNavMessageType>(b);
            message_in.data_length = EulerNavMessageSize(message_in.packet.messageType);
            if (message_in.data_length == 0) {
                message_in.state = ParseState::WaitingFor_SyncOne;
                message_in.index = 0;
                return;
            }
            message_in.packet.messageType = msgType;
            message_in.state = ParseState::WaitingFor_Data;
            message_in.padding_length = EulerNavMessagePaddingSize(message_in.packet.messageType);
            message_in.index = 0;
            break;
        }
        case ParseState::WaitingFor_Data:
            message_in.packet.payload[message_in.index++] = b;
            if (message_in.index >= message_in.data_length) {
                message_in.state = ParseState::WaitingFor_Padding;
                message_in.index = 0;
            }
            break;
        case ParseState::WaitingFor_Padding:
            message_in.packet.payload[message_in.index++] = b;
            // todo : failed if b != 0 ?
            if (message_in.index >= message_in.padding_length) {
                message_in.state = ParseState::WaitingFor_Checksum;
                message_in.index = 0;
            }
            break;
        case ParseState::WaitingFor_Checksum:
            message_in.packet.checksum[message_in.index++] = b;
            if (message_in.index >= EULERNAV_CHECKSUM_LENGTH) {
                message_in.state = ParseState::WaitingFor_SyncOne;
                message_in.index = 0;

                if (valid_packet(message_in.packet, message_in.data_length + message_in.padding_length)) {
                    messageType = handle_packet(message_in.packet);
                }
            }
            break;
    }
}

bool AP_ExternalAHRS_EulerNav::valid_packet(const EulerNav_Packet & packet, const uint8_t full_length)
{
    const uint32_t crc = crc32_small(EULERNAV_INIT_CRC, (const uint8_t *)&packet, full_length);
    return crc == be32toh_ptr(packet.checksum);
}

EulerNavMessageType AP_ExternalAHRS_EulerNav::handle_packet(const EulerNav_Packet& packet)
{
    const EulerNavMessageType descriptor = packet.messageType;
    switch (descriptor) {
        case EulerNavMessageType::InertialOutput:
            handle_imu(packet);
            break;
        case EulerNavMessageType::NavigationOutput:
            handle_nav(packet);
            break;
        case EulerNavMessageType::AccuracyInfo:
            // handle_acc(packet);
            break;
            case EulerNavMessageType::TimeNavigation:
            case EulerNavMessageType::TimeInertial:
            case EulerNavMessageType::TimeSync:
            case EulerNavMessageType::Version:
                break;
        default:
            break;
    }
    return descriptor;
}


void AP_ExternalAHRS_EulerNav::handle_imu(const EulerNav_Packet& packet)
{
    last_imu_pkt = AP_HAL::millis();
    const auto accel = Vector3f(
            be16toh_ptr(packet.payload + 1),
            be16toh_ptr(packet.payload + 3),
            be16toh_ptr(packet.payload + 5)
    );
    const auto gyro = Vector3f(
            be16toh_ptr(packet.payload + 7),
            be16toh_ptr(packet.payload + 9),
            be16toh_ptr(packet.payload + 11)
    );
    imu_validity_flags = packet.payload[14];
    imu_healthy = (imu_validity_flags & 0x3F) == 0x3F;

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro = gyro;
    }

    {
        // *INDENT-OFF*
        AP_ExternalAHRS::ins_data_message_t ins{
                accel: accel,
                gyro: gyro,
                temperature: -300
        };
        AP::ins().handle_external(ins);
    }

}

void AP_ExternalAHRS_EulerNav::handle_nav(const EulerNav_Packet& packet)
{
    last_nav_pkt = AP_HAL::millis();
//    const auto pressure = be16toh_ptr(packet.payload + 1);
//    const auto velocity_z = be16toh_ptr(packet.payload + 3);
    const auto roll_angle = be16toh_ptr(packet.payload + 5);
    const auto pitch_angle = be16toh_ptr(packet.payload + 7);
    const auto heading_angle = be16toh_ptr(packet.payload + 9);
    {
        WITH_SEMAPHORE(state.sem);
        state.quat.from_euler(roll_angle, pitch_angle, heading_angle);
        state.have_quaternion = true;
    }
    nav_validity_flags = packet.payload[11];

}

void AP_ExternalAHRS_EulerNav::handle_acc(const EulerNav_Packet& packet)
{
    last_acc_pkt = AP_HAL::millis();
}

int8_t AP_ExternalAHRS_EulerNav::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

bool AP_ExternalAHRS_EulerNav::healthy(void) const
{
    uint32_t now = AP_HAL::millis();

    // Expect the following rates:
    // * Navigation Filter: 25Hz = 40mS
    // * GPS: 2Hz = 500mS
    // * IMU: 25Hz = 40mS

    // SoftwareVersion once at start
    //5ms : InertialData
// 10Hz : TimeOfInertialData
// 10ms : navigation data
// 10Hz: Accuracy data, TimeOfNavigationData

    // Allow for some slight variance of 10%
    constexpr float RateFoS = 1.1;

    constexpr uint32_t expected_imu_time_delta_ms = 5;
    constexpr uint32_t expected_nav_time_delta_ms = 10;
    constexpr uint32_t expected_acc_time_delta_ms = 100;

    const bool times_healthy = (now - last_imu_pkt < expected_imu_time_delta_ms * RateFoS) &&
                               (now - last_nav_pkt < expected_nav_time_delta_ms * RateFoS) &&
                               (now - last_acc_pkt < expected_acc_time_delta_ms * RateFoS);

    return times_healthy && imu_healthy;
}

bool AP_ExternalAHRS_EulerNav::initialised(void) const
{
    const bool got_packets = last_imu_pkt != 0 && last_nav_pkt != 0 && last_acc_pkt != 0;
    return got_packets && imu_healthy;
}


bool AP_ExternalAHRS_EulerNav::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "EulerNav unhealthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_EulerNav::get_filter_status(nav_filter_status &status) const
{
    // TODO fix, add check for healthy data
    memset(&status, 0, sizeof(status));
    if (last_imu_pkt != 0 && last_nav_pkt != 0) {
        status.flags.initalized = true;
    }
    if (healthy() && last_imu_pkt != 0) {
        status.flags.attitude = false;
        status.flags.vert_vel = false;
        status.flags.vert_pos = false;

        status.flags.horiz_vel = false;
        status.flags.horiz_pos_rel = false;
        status.flags.horiz_pos_abs = false;
        status.flags.pred_horiz_pos_rel = false;
        status.flags.pred_horiz_pos_abs = false;
        status.flags.using_gps = false;

    }
}

void AP_ExternalAHRS_EulerNav::send_status_report(GCS_MAVLINK &link) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // TODO fix
    //     packet.velocity_variance = velocity_variance;
    //    packet.pos_horiz_variance = pos_horiz_variance;
    //    packet.pos_vert_variance = pos_vert_variance;
    //    packet.compass_variance = compass_variance;
    //    packet.terrain_alt_variance = terrain_alt_variance;
    //    packet.flags = flags;
    //    packet.airspeed_variance = airspeed_variance;
    // send message
//    const float vel_gate = 10;
//    const float pos_gate = 10;
//    const float hgt_gate = 10;
//    const float mag_var = 0;
//    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
//                                       state2.kf_vel_covariance.length()/vel_gate,
//                                       state2.kf_pos_covariance.xy().length()/pos_gate,
//                                       state2.kf_pos_covariance.z/hgt_gate,
//                                       mag_var, 0, 0);

}

#endif // AP_EXTERNAL_AHRS_EULERNAV_ENABLED
