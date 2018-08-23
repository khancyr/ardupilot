#pragma once

#include "OpticalFlow.h"
#include "OpticalFlow_backend.h"
#include <AP_SerialManager/AP_SerialManager.h>

class AP_OpticalFlow_CXOF : public OpticalFlow_backend
{

public:
    // constructor
    AP_OpticalFlow_CXOF(OpticalFlow &_frontend, AP_SerialManager &serial_manager);

    // init - initialise the sensor
    void init() override {}

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update() override;

protected:

private:
    // get a reading
    bool get_reading();
    static const uint8_t CXOF_PACKET_SIZE = 9;
    uint8_t input_buffer[CXOF_PACKET_SIZE];
    uint8_t pkt_count;

    AP_HAL::UARTDriver *uart = nullptr;
};
