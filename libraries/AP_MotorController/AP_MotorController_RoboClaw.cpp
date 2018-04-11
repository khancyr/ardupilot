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
 Original C Code by Marvelmind (https://bitbucket.org/marvelmind_robotics/)
 Adapted into Ardupilot by Karthik Desai, Amilcar Lucas
 April 2017
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorController_RoboClaw.h"

extern const AP_HAL::HAL &hal;

AP_MotorController_RoboClaw::AP_MotorController_RoboClaw(AP_MotorController &frontend, AP_SerialManager *serial_manager)
    :
    AP_MotorController_Backend(frontend) {
    uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_ESCTelemetry, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager->find_baudrate(AP_SerialManager::SerialProtocol_ESCTelemetry, 0));
        robotclaw = RoboClaw(uart, 10000);
    }
}

void AP_MotorController_RoboClaw::update(uint32_t motor1, uint32_t motor2) {
    if (uart == nullptr) {
        return;
    }

    switch ((AP_MotorController::MotorController_Control) get_control_type()) {
        /*
            34 - Drive M1 / M2 With Signed Duty Cycle
            Drive both M1 and M2 using a duty cycle value. The duty cycle is used to control the speed of
            the motor without a quadrature encoder. The command syntax:
            Send: [Address, 34, DutyM1(2 Bytes), DutyM2(2 Bytes), CRC(2 bytes)]
            Receive: [0xFF]
            The duty value is signed and the range is -32768 to +32767 (eg. +-100% duty).
        */
        case AP_MotorController::MotorController_Control::CONTROL_PWM :
        default:
            robotclaw.DutyM1M2(RoboClaw::ROBOCLAW_ADDR,
                               static_cast<uint16_t >(motor1),
                               static_cast<uint16_t>(motor2));
            break;
            /*
                37 - Drive M1 / M2 With Signed Speed
                Drive M1 and M2 in the same command using a signed speed value. The sign indicates which
                direction the motor will turn. This command is used to drive both motors by quad pulses per
                second. Different quadrature encoders will have different rates at which they generate the
                incoming pulses. The values used will differ from one encoder to another. Once a value is sent
                the motor will begin to accelerate as fast as possible until the rate defined is reached.
             */
        case AP_MotorController::MotorController_Control::CONTROL_SPEED :
            robotclaw.SpeedM1M2(RoboClaw::ROBOCLAW_ADDR,
                                motor1,
                                motor2);
            break;
//        case AP_MotorController::MotorController_Control::CONTROL_POSITION :
//            robotclaw.SpeedAccelDeccelPositionM1M2(RoboClaw::ROBOCLAW_ADDR);
    }
}

///////////////////////////////////////////////////////////////////////////////

#define MAXRETRY 2
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

//
// Constructor
//
AP_MotorController_RoboClaw::RoboClaw::RoboClaw()
{
}

AP_MotorController_RoboClaw::RoboClaw::RoboClaw(AP_HAL::UARTDriver *uart, uint32_t tout) :
    _timeout(tout),
    hserial(uart) {
}

//
// Destructor
//
AP_MotorController_RoboClaw::RoboClaw::~RoboClaw() = default;

size_t AP_MotorController_RoboClaw::RoboClaw::write(uint8_t byte) {
    if (hserial) {
        return hserial->write(byte);
    }
    return 0;
}

int16_t AP_MotorController_RoboClaw::RoboClaw::read() {
    if (hserial) {
        return hserial->read();
    }
    return 0;
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::available() {
    if (hserial) {
        return hserial->available();
    }
    return 0;
}

void AP_MotorController_RoboClaw::RoboClaw::flush() {
    if (hserial) {
        hserial->flush();
    }
}

int16_t AP_MotorController_RoboClaw::RoboClaw::read(uint32_t timeout) {
    // TODO change
    if (hserial) {
        uint32_t start = AP_HAL::micros();
        // Empty buffer?
        while (!hserial->available()) {
            if ((AP_HAL::micros() - start) >= timeout)
                return -1;
        }
        return hserial->read();
    }
    return 0;
}

void AP_MotorController_RoboClaw::RoboClaw::clear() {
    if (hserial) {
        while (hserial->available()) {
            hserial->read();
        }
    }
}

void AP_MotorController_RoboClaw::RoboClaw::crc_clear() {
    crc = 0;
}

void AP_MotorController_RoboClaw::RoboClaw::crc_update(uint8_t data) {
    crc = crc ^ ((uint16_t) data << 8);
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
}

uint16_t AP_MotorController_RoboClaw::RoboClaw::crc_get() {
    return crc;
}

bool AP_MotorController_RoboClaw::RoboClaw::write_n(uint8_t cnt, ...) {
    uint8_t trys = MAXRETRY;
    do {
        crc_clear();
        //send data with crc
        va_list marker;
        va_start(marker, cnt);     /* Initialize variable arguments. */
        for (uint8_t index = 0; index < cnt; index++) {
            uint8_t data = va_arg(marker, int);
            crc_update(data);
            write(data);
        }
        va_end(marker);              /* Reset variable arguments.      */
        uint16_t lcrc = crc_get();
        write(lcrc >> 8);
        write(lcrc);
        if (read(_timeout) == 0xFF) {
            return true;
        }
    } while (trys--);
    return false;
}

bool AP_MotorController_RoboClaw::RoboClaw::read_n(uint8_t cnt, uint8_t address, uint8_t cmd, ...) {
    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do {
        flush();

        data = 0;
        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        //send data with crc
        va_list marker;
        va_start(marker, cmd);     /* Initialize variable arguments. */
        for (uint8_t index = 0; index < cnt; index++) {
            uint32_t *ptr = va_arg(marker, uint32_t *);

            if (data != -1) {
                data = read(_timeout);
                crc_update(data);
                value = (uint32_t) data << 24;
            } else {
                break;
            }

            if (data != -1) {
                data = read(_timeout);
                crc_update(data);
                value |= (uint32_t) data << 16;
            } else {
                break;
            }

            if (data != -1) {
                data = read(_timeout);
                crc_update(data);
                value |= (uint32_t) data << 8;
            } else {
                break;
            }

            if (data != -1) {
                data = read(_timeout);
                crc_update(data);
                value |= (uint32_t) data;
            } else {
                break;
            }

            *ptr = value;
        }
        va_end(marker);              /* Reset variable arguments.      */

        if (data != -1) {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1) {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1) {
                    ccrc |= data;
                    return crc_get() == ccrc;
                }
            }
        }
    } while (trys--);

    return false;
}

uint8_t AP_MotorController_RoboClaw::RoboClaw::Read1(uint8_t address, uint8_t cmd, bool *valid) {
    if (valid)
        *valid = false;

    uint8_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = data;

        if (data != -1) {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1) {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1) {
                    ccrc |= data;
                    if (crc_get() == ccrc) {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

uint16_t AP_MotorController_RoboClaw::RoboClaw::Read2(uint8_t address, uint8_t cmd, bool *valid) {
    if (valid)
        *valid = false;

    uint16_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = (uint16_t) data << 8;

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            value |= (uint16_t) data;
        }

        if (data != -1) {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1) {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1) {
                    ccrc |= data;
                    if (crc_get() == ccrc) {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::Read4(uint8_t address, uint8_t cmd, bool *valid) {
    if (valid)
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = (uint32_t) data << 24;

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t) data << 16;
        }

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t) data << 8;
        }

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t) data;
        }

        if (data != -1) {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1) {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1) {
                    ccrc |= data;
                    if (crc_get() == ccrc) {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid) {
    if (valid)
        *valid = false;

    uint32_t value = 0;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(cmd);
        crc_update(cmd);

        data = read(_timeout);
        crc_update(data);
        value = (uint32_t) data << 24;

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t) data << 16;
        }

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t) data << 8;
        }

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            value |= (uint32_t) data;
        }

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            if (status)
                *status = data;
        }

        if (data != -1) {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1) {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1) {
                    ccrc |= data;
                    if (crc_get() == ccrc) {
                        *valid = true;
                        return value;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

bool AP_MotorController_RoboClaw::RoboClaw::ForwardM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, M1FORWARD, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::BackwardM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, M1BACKWARD, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetMinVoltageMainBattery(uint8_t address, uint8_t voltage) {
    return write_n(3, address, SETMINMB, voltage);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage) {
    return write_n(3, address, SETMAXMB, voltage);
}

bool AP_MotorController_RoboClaw::RoboClaw::ForwardM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, M2FORWARD, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::BackwardM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, M2BACKWARD, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::ForwardBackwardM1(uint8_t address, uint8_t speed) {
    return write_n(3, address, M17BIT, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::ForwardBackwardM2(uint8_t address, uint8_t speed) {
    return write_n(3, address, M27BIT, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::ForwardMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, MIXEDFORWARD, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::BackwardMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, MIXEDBACKWARD, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::TurnRightMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, MIXEDRIGHT, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::TurnLeftMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, MIXEDLEFT, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::ForwardBackwardMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, MIXEDFB, speed);
}

bool AP_MotorController_RoboClaw::RoboClaw::LeftRightMixed(uint8_t address, uint8_t speed) {
    return write_n(3, address, MIXEDLR, speed);
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::ReadEncM1(uint8_t address, uint8_t *status, bool *valid) {
    return Read4_1(address, GETM1ENC, status, valid);
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::ReadEncM2(uint8_t address, uint8_t *status, bool *valid) {
    return Read4_1(address, GETM2ENC, status, valid);
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::ReadSpeedM1(uint8_t address, uint8_t *status, bool *valid) {
    return Read4_1(address, GETM1SPEED, status, valid);
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::ReadSpeedM2(uint8_t address, uint8_t *status, bool *valid) {
    return Read4_1(address, GETM2SPEED, status, valid);
}

bool AP_MotorController_RoboClaw::RoboClaw::ResetEncoders(uint8_t address) {
    return write_n(2, address, RESETENC);
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadVersion(uint8_t address, char *version) {
    int16_t data;
    uint8_t trys = MAXRETRY;
    do {
        flush();

        data = 0;

        crc_clear();
        write(address);
        crc_update(address);
        write(GETVERSION);
        crc_update(GETVERSION);

        uint8_t i;
        for (i = 0; i < 48; i++) {
            data = read(_timeout);
            if (data != -1) {
                version[i] = data;
                crc_update(version[i]);
                if (version[i] == 0) {
                    uint16_t ccrc;
                    data = read(_timeout);
                    if (data != -1) {
                        ccrc = data << 8;
                        data = read(_timeout);
                        if (data != -1) {
                            ccrc |= data;
                            return crc_get() == ccrc;
                        }
                    }
                    break;
                }
            } else {
                break;
            }
        }
    } while (trys--);

    return false;
}

bool AP_MotorController_RoboClaw::RoboClaw::SetEncM1(uint8_t address, int32_t val) {
    return write_n(6, address, SETM1ENCCOUNT, SetDWORDval(val));
}

bool AP_MotorController_RoboClaw::RoboClaw::SetEncM2(uint8_t address, int32_t val) {
    return write_n(6, address, SETM2ENCCOUNT, SetDWORDval(val));
}

uint16_t AP_MotorController_RoboClaw::RoboClaw::ReadMainBatteryVoltage(uint8_t address, bool *valid) {
    return Read2(address, GETMBATT, valid);
}

uint16_t AP_MotorController_RoboClaw::RoboClaw::ReadLogicBatteryVoltage(uint8_t address, bool *valid) {
    return Read2(address, GETLBATT, valid);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage) {
    return write_n(3, address, SETMINLB, voltage);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage) {
    return write_n(3, address, SETMAXLB, voltage);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM1VelocityPID(uint8_t address,
                                                             float kp_fp,
                                                             float ki_fp,
                                                             float kd_fp,
                                                             uint32_t qpps) {
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n(18, address, SETM1PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM2VelocityPID(uint8_t address,
                                                             float kp_fp,
                                                             float ki_fp,
                                                             float kd_fp,
                                                             uint32_t qpps) {
    uint32_t kp = kp_fp * 65536;
    uint32_t ki = ki_fp * 65536;
    uint32_t kd = kd_fp * 65536;
    return write_n(18, address, SETM2PID, SetDWORDval(kd), SetDWORDval(kp), SetDWORDval(ki), SetDWORDval(qpps));
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::ReadISpeedM1(uint8_t address, uint8_t *status, bool *valid) {
    return Read4_1(address, GETM1ISPEED, status, valid);
}

uint32_t AP_MotorController_RoboClaw::RoboClaw::ReadISpeedM2(uint8_t address, uint8_t *status, bool *valid) {
    return Read4_1(address, GETM2ISPEED, status, valid);
}

bool AP_MotorController_RoboClaw::RoboClaw::DutyM1(uint8_t address, uint16_t duty) {
    return write_n(4, address, M1DUTY, SetWORDval(duty));
}

bool AP_MotorController_RoboClaw::RoboClaw::DutyM2(uint8_t address, uint16_t duty) {
    return write_n(4, address, M2DUTY, SetWORDval(duty));
}

bool AP_MotorController_RoboClaw::RoboClaw::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2) {
    return write_n(6, address, MIXEDDUTY, SetWORDval(duty1), SetWORDval(duty2));
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedM1(uint8_t address, uint32_t speed) {
    return write_n(6, address, M1SPEED, SetDWORDval(speed));
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedM2(uint8_t address, uint32_t speed) {
    return write_n(6, address, M2SPEED, SetDWORDval(speed));
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2) {
    return write_n(10, address, MIXEDSPEED, SetDWORDval(speed1), SetDWORDval(speed2));
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed) {
    return write_n(10, address, M1SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed) {
    return write_n(10, address, M2SPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed));
}
bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelM1M2(uint8_t address,
                                                           uint32_t accel,
                                                           uint32_t speed1,
                                                           uint32_t speed2) {
    return write_n(14, address, MIXEDSPEEDACCEL, SetDWORDval(accel), SetDWORDval(speed1), SetDWORDval(speed2));
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedDistanceM1(uint8_t address,
                                                            uint32_t speed,
                                                            uint32_t distance,
                                                            uint8_t flag) {
    return write_n(11, address, M1SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedDistanceM2(uint8_t address,
                                                            uint32_t speed,
                                                            uint32_t distance,
                                                            uint8_t flag) {
    return write_n(11, address, M2SPEEDDIST, SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedDistanceM1M2(uint8_t address,
                                                              uint32_t speed1,
                                                              uint32_t distance1,
                                                              uint32_t speed2,
                                                              uint32_t distance2,
                                                              uint8_t flag) {
    return write_n(19,
                   address,
                   MIXEDSPEEDDIST,
                   SetDWORDval(speed1),
                   SetDWORDval(distance1),
                   SetDWORDval(speed2),
                   SetDWORDval(distance2),
                   flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelDistanceM1(uint8_t address,
                                                                 uint32_t accel,
                                                                 uint32_t speed,
                                                                 uint32_t distance,
                                                                 uint8_t flag) {
    return write_n(15, address, M1SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelDistanceM2(uint8_t address,
                                                                 uint32_t accel,
                                                                 uint32_t speed,
                                                                 uint32_t distance,
                                                                 uint8_t flag) {
    return write_n(15, address, M2SPEEDACCELDIST, SetDWORDval(accel), SetDWORDval(speed), SetDWORDval(distance), flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelDistanceM1M2(uint8_t address,
                                                                   uint32_t accel,
                                                                   uint32_t speed1,
                                                                   uint32_t distance1,
                                                                   uint32_t speed2,
                                                                   uint32_t distance2,
                                                                   uint8_t flag) {
    return write_n(23,
                   address,
                   MIXEDSPEEDACCELDIST,
                   SetDWORDval(accel),
                   SetDWORDval(speed1),
                   SetDWORDval(distance1),
                   SetDWORDval(speed2),
                   SetDWORDval(distance2),
                   flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2) {
    bool valid;
    uint16_t value = Read2(address, GETBUFFERS, &valid);
    if (valid) {
        depth1 = value >> 8;
        depth2 = value;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2) {
    bool valid;
    uint32_t value = Read4(address, GETPWMS, &valid);
    if (valid) {
        pwm1 = value >> 16;
        pwm2 = value & 0xFFFF;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2) {
    bool valid;
    uint32_t value = Read4(address, GETCURRENTS, &valid);
    if (valid) {
        current1 = value >> 16;
        current2 = value & 0xFFFF;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelM1M2_2(uint8_t address,
                                                             uint32_t accel1,
                                                             uint32_t speed1,
                                                             uint32_t accel2,
                                                             uint32_t speed2) {
    return write_n(18,
                   address,
                   MIXEDSPEED2ACCEL,
                   SetDWORDval(accel1),
                   SetDWORDval(speed1),
                   SetDWORDval(accel2),
                   SetDWORDval(speed2));
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelDistanceM1M2_2(uint8_t address,
                                                                     uint32_t accel1,
                                                                     uint32_t speed1,
                                                                     uint32_t distance1,
                                                                     uint32_t accel2,
                                                                     uint32_t speed2,
                                                                     uint32_t distance2,
                                                                     uint8_t flag) {
    return write_n(27,
                   address,
                   MIXEDSPEED2ACCELDIST,
                   SetDWORDval(accel1),
                   SetDWORDval(speed1),
                   SetDWORDval(distance1),
                   SetDWORDval(accel2),
                   SetDWORDval(speed2),
                   SetDWORDval(distance2),
                   flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel) {
    return write_n(8, address, M1DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

bool AP_MotorController_RoboClaw::RoboClaw::DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel) {
    return write_n(8, address, M2DUTYACCEL, SetWORDval(duty), SetDWORDval(accel));
}

bool AP_MotorController_RoboClaw::RoboClaw::DutyAccelM1M2(uint8_t address,
                                                          uint16_t duty1,
                                                          uint32_t accel1,
                                                          uint16_t duty2,
                                                          uint32_t accel2) {
    return write_n(14,
                   address,
                   MIXEDDUTYACCEL,
                   SetWORDval(duty1),
                   SetDWORDval(accel1),
                   SetWORDval(duty2),
                   SetDWORDval(accel2));
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadM1VelocityPID(uint8_t address,
                                                              float &Kp_fp,
                                                              float &Ki_fp,
                                                              float &Kd_fp,
                                                              uint32_t &qpps) {
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(4, address, READM1PID, &Kp, &Ki, &Kd, &qpps);
    Kp_fp = ((float) Kp) / 65536;
    Ki_fp = ((float) Ki) / 65536;
    Kd_fp = ((float) Kd) / 65536;
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadM2VelocityPID(uint8_t address,
                                                              float &Kp_fp,
                                                              float &Ki_fp,
                                                              float &Kd_fp,
                                                              uint32_t &qpps) {
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(4, address, READM2PID, &Kp, &Ki, &Kd, &qpps);
    Kp_fp = ((float) Kp) / 65536;
    Ki_fp = ((float) Ki) / 65536;
    Kd_fp = ((float) Kd) / 65536;
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::SetMainVoltages(uint8_t address, uint16_t min, uint16_t max) {
    return write_n(6, address, SETMAINVOLTAGES, SetWORDval(min), SetWORDval(max));
}

bool AP_MotorController_RoboClaw::RoboClaw::SetLogicVoltages(uint8_t address, uint16_t min, uint16_t max) {
    return write_n(6, address, SETLOGICVOLTAGES, SetWORDval(min), SetWORDval(max));
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadMinMaxMainVoltages(uint8_t address, uint16_t &min, uint16_t &max) {
    bool valid;
    uint32_t value = Read4(address, GETMINMAXMAINVOLTAGES, &valid);
    if (valid) {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadMinMaxLogicVoltages(uint8_t address, uint16_t &min, uint16_t &max) {
    bool valid;
    uint32_t value = Read4(address, GETMINMAXLOGICVOLTAGES, &valid);
    if (valid) {
        min = value >> 16;
        max = value & 0xFFFF;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM1PositionPID(uint8_t address,
                                                             float kp_fp,
                                                             float ki_fp,
                                                             float kd_fp,
                                                             uint32_t kiMax,
                                                             uint32_t deadzone,
                                                             uint32_t min,
                                                             uint32_t max) {
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n(30,
                   address,
                   SETM1POSPID,
                   SetDWORDval(kd),
                   SetDWORDval(kp),
                   SetDWORDval(ki),
                   SetDWORDval(kiMax),
                   SetDWORDval(deadzone),
                   SetDWORDval(min),
                   SetDWORDval(max));
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM2PositionPID(uint8_t address,
                                                             float kp_fp,
                                                             float ki_fp,
                                                             float kd_fp,
                                                             uint32_t kiMax,
                                                             uint32_t deadzone,
                                                             uint32_t min,
                                                             uint32_t max) {
    uint32_t kp = kp_fp * 1024;
    uint32_t ki = ki_fp * 1024;
    uint32_t kd = kd_fp * 1024;
    return write_n(30,
                   address,
                   SETM2POSPID,
                   SetDWORDval(kd),
                   SetDWORDval(kp),
                   SetDWORDval(ki),
                   SetDWORDval(kiMax),
                   SetDWORDval(deadzone),
                   SetDWORDval(min),
                   SetDWORDval(max));
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadM1PositionPID(uint8_t address,
                                                              float &Kp_fp,
                                                              float &Ki_fp,
                                                              float &Kd_fp,
                                                              uint32_t &KiMax,
                                                              uint32_t &DeadZone,
                                                              uint32_t &Min,
                                                              uint32_t &Max) {
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(7, address, READM1POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
    Kp_fp = ((float) Kp) / 1024;
    Ki_fp = ((float) Ki) / 1024;
    Kd_fp = ((float) Kd) / 1024;
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadM2PositionPID(uint8_t address,
                                                              float &Kp_fp,
                                                              float &Ki_fp,
                                                              float &Kd_fp,
                                                              uint32_t &KiMax,
                                                              uint32_t &DeadZone,
                                                              uint32_t &Min,
                                                              uint32_t &Max) {
    uint32_t Kp, Ki, Kd;
    bool valid = read_n(7, address, READM2POSPID, &Kp, &Ki, &Kd, &KiMax, &DeadZone, &Min, &Max);
    Kp_fp = ((float) Kp) / 1024;
    Ki_fp = ((float) Ki) / 1024;
    Kd_fp = ((float) Kd) / 1024;
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelDeccelPositionM1(uint8_t address,
                                                                       uint32_t accel,
                                                                       uint32_t speed,
                                                                       uint32_t deccel,
                                                                       uint32_t position,
                                                                       uint8_t flag) {
    return write_n(19,
                   address,
                   M1SPEEDACCELDECCELPOS,
                   SetDWORDval(accel),
                   SetDWORDval(speed),
                   SetDWORDval(deccel),
                   SetDWORDval(position),
                   flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelDeccelPositionM2(uint8_t address,
                                                                       uint32_t accel,
                                                                       uint32_t speed,
                                                                       uint32_t deccel,
                                                                       uint32_t position,
                                                                       uint8_t flag) {
    return write_n(19,
                   address,
                   M2SPEEDACCELDECCELPOS,
                   SetDWORDval(accel),
                   SetDWORDval(speed),
                   SetDWORDval(deccel),
                   SetDWORDval(position),
                   flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SpeedAccelDeccelPositionM1M2(uint8_t address,
                                                                         uint32_t accel1,
                                                                         uint32_t speed1,
                                                                         uint32_t deccel1,
                                                                         uint32_t position1,
                                                                         uint32_t accel2,
                                                                         uint32_t speed2,
                                                                         uint32_t deccel2,
                                                                         uint32_t position2,
                                                                         uint8_t flag) {
    return write_n(35,
                   address,
                   MIXEDSPEEDACCELDECCELPOS,
                   SetDWORDval(accel1),
                   SetDWORDval(speed1),
                   SetDWORDval(deccel1),
                   SetDWORDval(position1),
                   SetDWORDval(accel2),
                   SetDWORDval(speed2),
                   SetDWORDval(deccel2),
                   SetDWORDval(position2),
                   flag);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM1DefaultAccel(uint8_t address, uint32_t accel) {
    return write_n(6, address, SETM1DEFAULTACCEL, SetDWORDval(accel));
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM2DefaultAccel(uint8_t address, uint32_t accel) {
    return write_n(6, address, SETM2DEFAULTACCEL, SetDWORDval(accel));
}

bool AP_MotorController_RoboClaw::RoboClaw::SetPinFunctions(uint8_t address,
                                                            uint8_t S3mode,
                                                            uint8_t S4mode,
                                                            uint8_t S5mode) {
    return write_n(5, address, SETPINFUNCTIONS, S3mode, S4mode, S5mode);
}

bool AP_MotorController_RoboClaw::RoboClaw::GetPinFunctions(uint8_t address,
                                                            uint8_t &S3mode,
                                                            uint8_t &S4mode,
                                                            uint8_t &S5mode) {
    uint8_t val1, val2, val3;
    uint8_t trys = MAXRETRY;
    int16_t data;
    do {
        flush();

        crc_clear();
        write(address);
        crc_update(address);
        write(GETPINFUNCTIONS);
        crc_update(GETPINFUNCTIONS);

        data = read(_timeout);
        crc_update(data);
        val1 = data;

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            val2 = data;
        }

        if (data != -1) {
            data = read(_timeout);
            crc_update(data);
            val3 = data;
        }

        if (data != -1) {
            uint16_t ccrc;
            data = read(_timeout);
            if (data != -1) {
                ccrc = data << 8;
                data = read(_timeout);
                if (data != -1) {
                    ccrc |= data;
                    if (crc_get() == ccrc) {
                        S3mode = val1;
                        S4mode = val2;
                        S5mode = val3;
                        return true;
                    }
                }
            }
        }
    } while (trys--);

    return false;
}

bool AP_MotorController_RoboClaw::RoboClaw::SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max) {
    return write_n(4, address, SETDEADBAND, Min, Max);
}

bool AP_MotorController_RoboClaw::RoboClaw::GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max) {
    bool valid;
    uint16_t value = Read2(address, GETDEADBAND, &valid);
    if (valid) {
        Min = value >> 8;
        Max = value;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadEncoders(uint8_t address, uint32_t &enc1, uint32_t &enc2) {
    bool valid = read_n(2, address, GETENCODERS, &enc1, &enc2);
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadISpeeds(uint8_t address, uint32_t &ispeed1, uint32_t &ispeed2) {
    bool valid = read_n(2, address, GETISPEEDS, &ispeed1, &ispeed2);
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::RestoreDefaults(uint8_t address) {
    return write_n(2, address, RESTOREDEFAULTS);
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadTemp(uint8_t address, uint16_t &temp) {
    bool valid;
    temp = Read2(address, GETTEMP, &valid);
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadTemp2(uint8_t address, uint16_t &temp) {
    bool valid;
    temp = Read2(address, GETTEMP2, &valid);
    return valid;
}

uint16_t AP_MotorController_RoboClaw::RoboClaw::ReadError(uint8_t address, bool *valid) {
    return Read2(address, GETERROR, valid);
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode) {
    bool valid;
    uint16_t value = Read2(address, GETENCODERMODE, &valid);
    if (valid) {
        M1mode = value >> 8;
        M2mode = value;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM1EncoderMode(uint8_t address, uint8_t mode) {
    return write_n(3, address, SETM1ENCODERMODE, mode);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM2EncoderMode(uint8_t address, uint8_t mode) {
    return write_n(3, address, SETM2ENCODERMODE, mode);
}

bool AP_MotorController_RoboClaw::RoboClaw::WriteNVM(uint8_t address) {
    return write_n(6, address, WRITENVM, SetDWORDval(0xE22EAB7A));
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadNVM(uint8_t address) {
    return write_n(2, address, READNVM);
}

bool AP_MotorController_RoboClaw::RoboClaw::SetConfig(uint8_t address, uint16_t config) {
    return write_n(4, address, SETCONFIG, SetWORDval(config));
}

bool AP_MotorController_RoboClaw::RoboClaw::GetConfig(uint8_t address, uint16_t &config) {
    bool valid;
    uint16_t value = Read2(address, GETCONFIG, &valid);
    if (valid) {
        config = value;
    }
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM1MaxCurrent(uint8_t address, uint32_t max) {
    return write_n(10, address, SETM1MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

bool AP_MotorController_RoboClaw::RoboClaw::SetM2MaxCurrent(uint8_t address, uint32_t max) {
    return write_n(10, address, SETM2MAXCURRENT, SetDWORDval(max), SetDWORDval(0));
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadM1MaxCurrent(uint8_t address, uint32_t &max) {
    uint32_t tmax, dummy;
    bool valid = read_n(2, address, GETM1MAXCURRENT, &tmax, &dummy);
    if (valid)
        max = tmax;
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::ReadM2MaxCurrent(uint8_t address, uint32_t &max) {
    uint32_t tmax, dummy;
    bool valid = read_n(2, address, GETM2MAXCURRENT, &tmax, &dummy);
    if (valid)
        max = tmax;
    return valid;
}

bool AP_MotorController_RoboClaw::RoboClaw::SetPWMMode(uint8_t address, uint8_t mode) {
    return write_n(3, address, SETPWMMODE, mode);
}

bool AP_MotorController_RoboClaw::RoboClaw::GetPWMMode(uint8_t address, uint8_t &mode) {
    bool valid;
    uint8_t value = Read1(address, GETPWMMODE, &valid);
    if (valid) {
        mode = value;
    }
    return valid;
}

