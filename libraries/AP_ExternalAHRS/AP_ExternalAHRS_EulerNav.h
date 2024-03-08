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
  support for serial connected EulerNav INS system
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_EULERNAV_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_ExternalAHRS/EulerNav_msgs.h>

class AP_ExternalAHRS_EulerNav : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_EulerNav(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override {
        build_packet();
    }

    // Get model/type name
    const char* get_name() const override {
        return "EulerNav";
    }

    AP_ExternalAHRS::mag_data_message_t mag_data;
    AP_ExternalAHRS::baro_data_message_t baro_data;
    AP_ExternalAHRS::ins_data_message_t ins_data;


private:
    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;
    int8_t port_num;
    bool port_open = false;
    uint32_t baudrate;
    bool setup_complete;
    uint32_t last_imu_pkt;
    uint32_t last_nav_pkt;
    uint32_t last_acc_pkt;

    uint8_t imu_validity_flags;
    bool imu_healthy;
    uint8_t nav_validity_flags;

    void update_thread();
    void build_packet();

    struct EulerNav_Packet {
        EulerNav_ProtocolHeader header;
        EulerNavMessageType messageType;

        uint8_t payload[32];
        uint8_t checksum[4];

    };

    enum class ParseState {
        WaitingFor_SyncOne,
        WaitingFor_SyncTwo,
        WaitingFor_Version,
        WaitingFor_MessageType,
        WaitingFor_Data,
        WaitingFor_Padding,
        WaitingFor_Checksum
    };

    struct {
        EulerNav_Packet packet;
        uint8_t data_length;
        uint8_t padding_length;
        ParseState state;
        uint8_t index;
    } message_in;

    void post_nav() const;
    void post_acc() const;
    // If the byte matches a descriptor, it returns true and that type should be handled.
    void handle_byte(const uint8_t b, EulerNavMessageType& messageType);
    // Returns true if the fletcher checksum for the packet is valid, else false.
    bool valid_packet(const EulerNav_Packet & packet, const uint8_t full_length);
    // Calls the correct functions based on the packet descriptor of the packet
    EulerNavMessageType handle_packet(const EulerNav_Packet &packet);
    // Collects data from an imu packet into `imu_data`
    void handle_imu(const EulerNav_Packet &packet);
    void handle_nav(const EulerNav_Packet& packet);
    void handle_acc(const EulerNav_Packet& packet);
};

#endif  // AP_EXTERNAL_AHRS_EULERNAV_ENABLED
