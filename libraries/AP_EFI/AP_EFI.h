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

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>

#define EFI_ENABLED !HAL_MINIMIZE_FEATURES

#if EFI_ENABLED
#include "AP_EFI_State.h"
#include <AP_Math/AP_Math.h>


/*
 * This library aims to read data from Electronic Fuel Injection 
 * or Engine Control units. It is focused around the generic
 * internal combustion engine state message provided by the 
 * UAVCAN protocol due to its comprehensiveness, but is extensible
 * to use other forms of data transfer besides UAVCAN. 
 * 
 *
 *
 * Authors: Sriram Sami and David Ingraham
 * With direction from Andrew Tridgell, Robert Lefebvre, Francisco Ferreira and
 * Pavel Kirienko.
 * Thanks to Yonah, SpektreWorks Inc, and HFE International.
 */

class AP_EFI {
    friend class AP_EFI_Backend;
public:

    // For parameter initialization
    AP_EFI();

   /* Do not allow copies */
    AP_EFI(const AP_EFI &other) = delete;
    AP_EFI &operator=(const AP_EFI&) = delete;

    // Initializes backend
    void init(void);

    // Requests backend to update the frontend. Should be called at 10Hz.
    void update();
    
    // Returns the RPM
    uint32_t get_rpm(uint8_t i) const { return state[i].engine_speed_rpm; }

    // returns enabled state of EFI
    bool enabled(uint8_t i) const { return (enum EFI_Communication_Type )param[i].type.get() != EFI_COMMUNICATION_TYPE_NONE; }

    bool is_healthy(uint8_t i) const;

    // return the current fuel_level in liter
    bool get_fuel_level(float &tfl);

    void set_log_efi_bit(uint32_t log_efi_bit) { _log_efi_bit = log_efi_bit; }

    // Parameter info
    static const struct AP_Param::GroupInfo var_info[];

    // Backend driver types
    enum class EFI_Communication_Type {
        EFI_COMMUNICATION_TYPE_NONE         = 0,
        EFI_COMMUNICATION_TYPE_SERIAL_MS    = 1,
        EFI_COMMUNICATION_TYPE_SERIAL_FIALA =2,
    };

    // find first efi instance with the specified orientation
    AP_EFI_Backend *find_instance(enum Rotation orientation) const;   
    AP_EFI_Backend *get_backend(uint8_t id) const;
    Status status_orient(enum Rotation orientation) const;
    // return true if we have an efi with the specified orientation
    bool has_orientation(enum Rotation orientation) const;
    bool has_data_orient(enum Rotation orientation) const;
    uint8_t range_valid_count_orient(enum Rotation orientation) const;
    uint32_t last_reading_ms(enum Rotation orientation) const;
    // send EFI_STATUS
    void send_mavlink_efi_status(mavlink_channel_t chan);

    void send_mavlink_efi2_status(mavlink_channel_t chan);

    static AP_EFI *get_singleton(void) {
        return singleton;
    }

protected:

    // Back end Parameters
    AP_Float coef1;
    AP_Float coef2;
    AP_Float ratio;
    struct {
        AP_Int8  type;
        AP_Int8  orientation;
        AP_Int8  index;
    } param[EFI_MAX_INSTANCES];

private:

    HAL_Semaphore sem;
    // Tracking backends
    AP_EFI_Backend *drivers[EFI_MAX_INSTANCES];
    EFI_State state[EFI_MAX_INSTANCES];
    static AP_EFI *singleton;
    uint8_t num_instances:2;
    AP_HAL::AnalogSource *source;

    // write to log
    void log_status(uint8_t i);
    void log_efi();
    uint32_t _log_efi_bit = -1;
};

namespace AP {
    AP_EFI *EFI();
};

#endif // EFI_ENABLED
