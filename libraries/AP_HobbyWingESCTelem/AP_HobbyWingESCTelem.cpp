/*
  ESC Telemetry for Hobbywing Pro 80A HV ESC.

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */
#include "AP_HobbyWingESCTelem.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if AP_HOBBYWING_ESC_TELEM_ENABLED

extern const AP_HAL::HAL& hal;


// constructor
AP_HobbyWingESCTelem::AP_HobbyWingESCTelem()
{
}

void AP_HobbyWingESCTelem::init(AP_HAL::UARTDriver *_uart, ESC_TELEM_PACKET_TYPE type, uint8_t mot_pole_pairs)
{
    pkt_type = type;
    uart = _uart;
    uart->begin(19200);
    uart->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
    pole_pairs = mot_pole_pairs;
}

/*
  update ESC telemetry
 */
bool AP_HobbyWingESCTelem::update()
{
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }

    // we expect at least 50ms idle between frames
    uint32_t now = AP_HAL::native_millis();
    bool frame_gap = (now - last_read_ms) > 10;

    last_read_ms = now;

    // don't read too much in one loop to prevent too high CPU load
    if (n > 700) {  // datalink is 640bytes so add some to allow catch a frame per read
        n = 700;
    }
    if (len == 0 && !frame_gap) {
        uart->discard_input();
        return false;
    }

    if (frame_gap) {
        len = 0;
    }

    bool ret = false;


    while (n--) {
        uint8_t b = uart->read();
        //hal.console->printf("t=%u 0x%02x\n", now, b);
        if (len == 0 && b != TELEM_HEADER) {
            continue;
        }
        if (len == 0) {
            len++;
            continue;
        }
        if (len == 1 && b != TELEM_LEN_SINGLE ) {
            len = 0;
            continue;
        }
        if (len == 1 && b != TELEM_LEN_DATALINK ) {
            len = 0;
            continue;
        }

        if (len == 2 && b != TELEM_SW_VERSION) {
            len = 0;
            continue;
        }
        if (len == 3 && b != TELEM_CMD_WORD) {
            len = 0;
            continue;
        }
        if (pkt_type == ESC_TELEM_PACKET_TYPE::SINGLE) {
            auto buf = (uint8_t *)&pkt_single;
            buf[len++] = b;
            if (len == TELEM_LEN_SINGLE) {
                ret = parse_packet_single();
                len = 0;
            }
        }
        if (pkt_type == ESC_TELEM_PACKET_TYPE::DATALINK) {
            auto buf = (uint8_t *)&pkt_datalink;
            buf[len++] = b;
            if (len == TELEM_LEN_DATALINK) {
                ret = parse_packet_datalink();
                len = 0;
            }
        }


    }
    return ret;
}

static const struct {
    uint8_t adc_temp;
    uint8_t temp_C;
} temp_table[] = {
    { 241, 	0}, 	{ 240, 	1}, 	{ 239, 	2}, 	{ 238, 	3}, 	{ 237, 	4}, 	{ 236, 	5}, 	{ 235, 	6}, 	{ 234, 	7}, 	{ 233, 	8}, 	{ 232, 	9},
    { 231, 	10}, 	{ 230, 	11}, 	{ 229, 	12}, 	{ 228, 	13}, 	{ 227, 	14}, 	{ 226, 	15}, 	{ 224, 	16}, 	{ 223, 	17}, 	{ 222, 	18}, 	{ 220, 	19},
    { 219, 	20}, 	{ 217, 	21}, 	{ 216, 	22}, 	{ 214, 	23}, 	{ 213, 	24}, 	{ 211, 	25}, 	{ 209, 	26}, 	{ 208, 	27}, 	{ 206, 	28}, 	{ 204, 	29},
    { 202, 	30}, 	{ 201, 	31}, 	{ 199, 	32}, 	{ 197, 	33}, 	{ 195, 	34}, 	{ 193, 	35}, 	{ 191, 	36}, 	{ 189, 	37}, 	{ 187, 	38}, 	{ 185, 	39},
    { 183, 	40}, 	{ 181, 	41}, 	{ 179, 	42}, 	{ 177, 	43}, 	{ 174, 	44}, 	{ 172, 	45}, 	{ 170, 	46}, 	{ 168, 	47}, 	{ 166, 	48}, 	{ 164, 	49},
    { 161, 	50}, 	{ 159, 	51}, 	{ 157, 	52}, 	{ 154, 	53}, 	{ 152, 	54}, 	{ 150, 	55}, 	{ 148, 	56}, 	{ 146, 	57}, 	{ 143, 	58}, 	{ 141, 	59},
    { 139, 	60}, 	{ 136, 	61}, 	{ 134, 	62}, 	{ 132, 	63}, 	{ 130, 	64}, 	{ 128, 	65}, 	{ 125, 	66}, 	{ 123, 	67}, 	{ 121, 	68}, 	{ 119, 	69},
    { 117, 	70}, 	{ 115, 	71}, 	{ 113, 	72}, 	{ 111, 	73}, 	{ 109, 	74}, 	{ 106, 	75}, 	{ 105, 	76}, 	{ 103, 	77}, 	{ 101, 	78}, 	{ 99, 	79},
    { 97, 	80}, 	{ 95, 	81}, 	{ 93, 	82}, 	{ 91, 	83}, 	{ 90, 	84}, 	{ 88, 	85}, 	{ 85, 	86}, 	{ 84, 	87}, 	{ 82, 	88}, 	{ 81, 	89},
    { 79, 	90}, 	{ 77, 	91}, 	{ 76, 	92}, 	{ 74, 	93}, 	{ 73, 	94}, 	{ 72, 	95}, 	{ 69, 	96}, 	{ 68, 	97}, 	{ 66, 	98}, 	{ 65, 	99},
    { 64, 	100}, 	{ 62, 	101}, 	{ 62, 	102}, 	{ 61, 	103}, 	{ 59, 	104}, 	{ 58, 	105}, 	{ 56, 	106}, 	{ 54, 	107}, 	{ 54, 	108}, 	{ 53, 	109},
    { 51, 	110}, 	{ 51, 	111}, 	{ 50, 	112}, 	{ 48, 	113}, 	{ 48, 	114}, 	{ 46, 	115}, 	{ 46, 	116}, 	{ 44, 	117}, 	{ 43, 	118}, 	{ 43, 	119},
    { 41, 	120}, 	{ 41, 	121}, 	{ 39, 	122}, 	{ 39, 	123}, 	{ 39, 	124}, 	{ 37, 	125}, 	{ 37, 	126}, 	{ 35, 	127}, 	{ 35, 	128}, 	{ 33, 	129},
};

uint8_t AP_HobbyWingESCTelem::temperature_decode(uint8_t temp_raw) const
{
    for (auto i : temp_table) {
        if (i.adc_temp <= temp_raw) {
            return i.temp_C;
        }
    }
    return 130U;
}

/*
  parse packet
 */
bool AP_HobbyWingESCTelem::parse_packet_single()
{
    uint16_t crc = pkt_single.get_crc((uint8_t *)&pkt_single, TELEM_LEN_SINGLE-2);
    if (crc != pkt_single.crc) {
        return false;
    }

    decoded[0].counter = be16toh(pkt_single.header.seq);
    decoded[0].throttle_req = be16toh(pkt_single.info.throttle_req);  // * 100.0 / 1024.0 to have pct
    decoded[0].throttle = be16toh(pkt_single.info.throttle);  // * 100.0 / 1024.0 to have pct
    decoded[0].rpm = be16toh(pkt_single.info.rpm) * 10.0 / pole_pairs; // * 5.0 / 7.0; // scale from eRPM to RPM // TODO
    decoded[0].voltage = be16toh(pkt_single.info.voltage) * 0.1;
    decoded[0].phase_current = int16_t(be16toh(pkt_single.info.phase_current)) * 0.01;
    decoded[0].current = int16_t(be16toh(pkt_single.info.current)) / 64.0;
    decoded[0].mos_temperature = temperature_decode(pkt_single.info.mos_temperature);
    decoded[0].cap_temperature = temperature_decode(pkt_single.info.cap_temperature);
    decoded[0].status = be16toh(pkt_single.info.status);
    if (decoded[0].status != 0) {
        decoded[0].error_count++;
    }

    return true;
}

bool AP_HobbyWingESCTelem::parse_packet_datalink()
{
    uint16_t crc = pkt_single.get_crc((uint8_t *)&pkt_datalink, TELEM_LEN_DATALINK-2);
    if (crc != pkt_single.crc) {
        return false;
    }
    for (auto & i : pkt_datalink.motor) {
        decoded[i.esc_channel_number].counter = be16toh(pkt_datalink.header.seq);
        decoded[i.esc_channel_number].throttle_req = be16toh(i.info.throttle_req);  // * 100.0 / 1024.0 to have pct
        decoded[i.esc_channel_number].throttle = be16toh(i.info.throttle);  // * 100.0 / 1024.0 to have pct
        decoded[i.esc_channel_number].rpm = be16toh(i.info.rpm) * 10.0 / pole_pairs; // * 5.0 / 7.0; // scale from eRPM to RPM // TODO
        decoded[i.esc_channel_number].voltage = be16toh(i.info.voltage) * 0.1;
        decoded[i.esc_channel_number].phase_current = int16_t(be16toh(i.info.phase_current)) * 0.01;
        decoded[i.esc_channel_number].current = int16_t(be16toh(i.info.current)) / 64.0;
        decoded[i.esc_channel_number].mos_temperature = temperature_decode(i.info.mos_temperature);
        decoded[i.esc_channel_number].cap_temperature = temperature_decode(i.info.cap_temperature);
        decoded[i.esc_channel_number].status = be16toh(i.info.status);
        if (decoded[i.esc_channel_number].status != 0) {
            decoded[i.esc_channel_number].error_count++;
        }
    }


    return true;
}

#if AP_HOBBYWING_ESC_TELEM_ESC_TELEM_BACKEND_ENABLED
const AP_Param::GroupInfo AP_HobbyWingESCTelem_ESCTelem_Backend::var_info[] = {
    // @Param: MASK
    // @DisplayName: HobbyWingESC Channel Bitmask
    // @Description: Mask of which channels are returning data on serial ports
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MASK",  1, AP_HobbyWingESCTelem_ESCTelem_Backend, channel_mask, 0),
    // @Param: TYPE
    // @DisplayName: HobbyWingESC Telem Type
    // @Description: Select if Single channel telemetry or Datalink
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TYPE",  2, AP_HobbyWingESCTelem_ESCTelem_Backend, telem_type, float(ESC_TELEM_PACKET_TYPE::SINGLE)),
    // @Param: POLE
    // @DisplayName: MOTOR Pole Pairs
    // @Description: MOTOR Pole Pairs, used for RPM calculation
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("POLPAIR", 3, AP_HobbyWingESCTelem_ESCTelem_Backend, pole_pairs, 14),
};

// constructor
AP_HobbyWingESCTelem_ESCTelem_Backend::AP_HobbyWingESCTelem_ESCTelem_Backend(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_HobbyWingESCTelem_ESCTelem_Backend::init()
{
    for (uint8_t i=0; i<32 && num_backends< MAX_BACKENDS; i++) {
        // work out our servo channel offset.  First instance of this
        // class gets the channel corresponding to the first bit set in
        // the mask etc
        if (!((uint32_t)channel_mask & (1U<<i))) {
            continue;
        }
        // see if there's another serial port configured for the correct protocol:
        AP_HAL::UARTDriver *uart { AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_HobbyWingESC, num_backends) };
        if (uart == nullptr) {
            break;
        }
        backends[num_backends] = new AP_HobbyWingESCTelem();
        if (backends[num_backends] == nullptr) {
            break;
        }
        backends[num_backends]->init(uart, telem_type, pole_pairs.get());
        servo_channel[num_backends++] = i+1;
    }
}

void AP_HobbyWingESCTelem_ESCTelem_Backend::update_telemetry()
{
    for (uint8_t i=0; i<num_backends; i++) {
        AP_HobbyWingESCTelem &backend = *backends[i];
        if (!backend.update()) {
            continue;
        }
        if (telem_type == ESC_TELEM_PACKET_TYPE::SINGLE) {
            const auto &decoded = backend.get_telem(0);
            update_rpm(servo_channel[i], decoded.rpm, 0);

            const TelemetryData t {
                    .temperature_cdeg = int16_t(decoded.mos_temperature * 100),
                    .voltage = float(decoded.voltage) * 0.01f,
                    .current = float(decoded.current) * 0.01f,
            };
            update_telem_data(servo_channel[i], t,
                              AP_ESC_Telem_Backend::TelemetryType::CURRENT
                              | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                              | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
        }
        if (telem_type == ESC_TELEM_PACKET_TYPE::DATALINK) {
            // TODO how to match datalink link channels and AP Channels ? For now datalink1 is motor 1 - 8, datalink2 motors 9 - 18 etc.
            for (auto j=0; j<8; j++) {
                const auto &decoded = backend.get_telem(j);
                update_rpm((i * 8) + j, decoded.rpm, 0);

                const TelemetryData t {
                        .temperature_cdeg = int16_t(decoded.mos_temperature * 100),
                        .voltage = float(decoded.voltage) * 0.01f,
                        .current = float(decoded.current) * 0.01f,
                };
                update_telem_data((i * 8) + j, t,
                                  AP_ESC_Telem_Backend::TelemetryType::CURRENT
                                  | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                                  | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
            }
        }
    }
}
#endif  // AP_HOBBYWING_ESC_TELEM_ESC_TELEM_ENABLED

#endif  // AP_HOBBYWING_ESC_TELEM_ENABLED

