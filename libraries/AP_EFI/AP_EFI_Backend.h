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
#include "AP_EFI.h"
#include "AP_EFI_State.h"

//class AP_EFI; //forward declaration

class AP_EFI_Backend {
public:    
    // Constructor with initialization
    AP_EFI_Backend(AP_EFI &_frontend, EFI_State &_state, uint8_t _instance);

    // Virtual destructor that efi backends can override 
    virtual ~AP_EFI_Backend(void) {}

    // Update the state structure
    virtual void update() = 0;
    enum Rotation orientation() const { return (Rotation)frontend.param[instance].orientation.get(); }
    Status status() const;
    AP_EFI::EFI_Communication_Type type() const { return (AP_EFI::EFI_Communication_Type)frontend.param[instance].type.get(); }
    // true if efi is returning data
    bool has_data() const;
    // returns count of consecutive good readings
    uint8_t range_valid_count() const { return state.range_valid_count; }
    // return system time of last successful read from the sensor
    uint32_t last_reading_ms() const { return state.last_reading_ms; }

protected:

    // update status based on rpm measurment
    void update_status();

    // set status and update valid_count
    void set_status(Status status);
    AP_EFI &frontend;
    EFI_State &state;
    uint8_t instance;
    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;
    //Type Backend initialised with
    AP_EFI::EFI_Communication_Type _backend_type;
    int8_t get_uavcan_node_id(void) const;
    float get_coef1(void) const;
    float get_coef2(void) const;
};
