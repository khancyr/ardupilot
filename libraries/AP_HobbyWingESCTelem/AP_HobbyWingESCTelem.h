/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC.

 - this driver is used in two different ways:
  - by the main firmware as an AP_ESC_Telem_Backend
  - by AP_Periph using a simple getter for the decoded telemetry data

 */

#pragma once

#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#ifndef AP_HOBBYWING_ESC_TELEM_ENABLED
#define AP_HOBBYWING_ESC_TELEM_ENABLED HAL_WITH_ESC_TELEM
#endif

#ifndef AP_HOBBYWING_ESC_TELEM_ESC_TELEM_BACKEND_ENABLED
#define AP_HOBBYWING_ESC_TELEM_ESC_TELEM_BACKEND_ENABLED (AP_HOBBYWING_ESC_TELEM_ENABLED && HAL_WITH_ESC_TELEM)
#endif

#if AP_HOBBYWING_ESC_TELEM_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/crc.h>

enum class ESC_TELEM_PACKET_TYPE: uint8_t {
    SINGLE = 1,
    DATALINK = 2
};

class AP_HobbyWingESCTelem {
public:
    AP_HobbyWingESCTelem();

    void init(class AP_HAL::UARTDriver *uart, ESC_TELEM_PACKET_TYPE type, uint8_t mot_pole_pairs);
    bool update();

    struct HWESC {
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        float rpm;
        float voltage;
        float phase_current;
        float current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint32_t error_count;
    };

    const HWESC &get_telem(uint8_t instance) const{
        return decoded[instance];
    }

private:
    AP_HAL::UARTDriver *uart;
    uint8_t pole_pairs;

    static const uint8_t TELEM_HEADER = 0x9B;
    static const uint8_t TELEM_LEN_SINGLE = 0x16;
    static const uint8_t TELEM_LEN_DATALINK = 160;
    static const uint8_t TELEM_SW_VERSION = 0x01;
    static const uint8_t TELEM_CMD_WORD = 0x02;
    static const uint8_t MAX_ESC_PKT = 8;



    struct PACKED Packet_Header {
        const uint8_t header = TELEM_HEADER; // 0x9B
        uint8_t pkt_len;
        uint8_t sw_version;  // default to 1
        uint8_t cmd_data;  // 0x02 for reading
        uint16_t seq;
    };
    struct PACKED ESCInfo {
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        uint16_t current;
        uint16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
    };

    struct PACKED {
        Packet_Header header;
        ESCInfo info;
        uint16_t crc;

        uint16_t get_crc(const uint8_t *data, uint16_t length) {
            return crc_sum16(data, length);
        }
    } pkt_single;

    struct PACKED ESCInfoGroup {
        uint8_t esc_channel_number;
        uint16_t seq;
        ESCInfo info;
    };

    struct PACKED Packet {
        Packet_Header header;
        ESCInfoGroup motor[MAX_ESC_PKT];
        uint16_t crc;

        uint16_t get_crc(const uint8_t *data, uint16_t length) {
            return crc_xmodem(data, length);
        }

    } pkt_datalink;

    uint8_t len;
    uint32_t last_read_ms;
    uint32_t error_count;
    ESC_TELEM_PACKET_TYPE pkt_type;

    struct HWESC decoded[MAX_ESC_PKT];

    bool parse_packet_single();
    bool parse_packet_datalink();
    uint8_t temperature_decode(uint8_t temp_raw) const;

};


// AP_ESC_Telem interface
#if HAL_WITH_ESC_TELEM
class AP_HobbyWingESCTelem_ESCTelem_Backend
    : AP_ESC_Telem_Backend
{
public:

    // constructor
    AP_HobbyWingESCTelem_ESCTelem_Backend();

    static const struct AP_Param::GroupInfo var_info[];

    void init();

    void update_telemetry();

private:
    AP_Int32 channel_mask;
    AP_Enum<ESC_TELEM_PACKET_TYPE> telem_type;
    AP_Int8 pole_pairs;

    static const uint8_t MAX_BACKENDS { 32 };
    AP_HobbyWingESCTelem *backends[MAX_BACKENDS];
    uint8_t servo_channel[MAX_BACKENDS];

    uint8_t num_backends;
};
#endif  // HAL_WITH_ESC_TELEM

#endif  // AP_HOBBYWING_ESC_TELEM_ENABLED
