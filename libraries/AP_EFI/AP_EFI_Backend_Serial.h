#pragma once

#include "AP_EFI_Backend.h"

class AP_EFI_Backend_Serial : public AP_EFI_Backend
{
public:
    // constructor
    AP_EFI_Backend_Serial(EFI_State &_state, uint8_t serial_instance);

    // static detection function
    static bool detect(uint8_t serial_instance);

protected:

    // baudrate used during object construction:
    virtual uint32_t initial_baudrate(uint8_t serial_instance) const;

    // the value 0 is special to the UARTDriver - it's "use default"
    virtual uint16_t rx_bufsize() const { return 0; }
    virtual uint16_t tx_bufsize() const { return 0; }

    AP_HAL::UARTDriver *port = nullptr;

    // update state; not all backends call this!
    virtual void update(void) override;

    // it is essential that anyone relying on the base-class update to
    // implement this:
    virtual bool get_reading() { return false; }

    // maximum time between readings before we change state to NoData:
    virtual uint16_t read_timeout_ms() const { return 200; }
};
