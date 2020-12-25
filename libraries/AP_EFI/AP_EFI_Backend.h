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

#include "AP_EFI.h"
#include "AP_EFI_State.h"
#include <AP_HAL/AP_HAL.h>

class AP_EFI; //forward declaration

class AP_EFI_Backend {
public:    
    // Constructor with initialization
    AP_EFI_Backend(EFI_State &_state);

    // Virtual destructor that efi backends can override 
    virtual ~AP_EFI_Backend(void) {}

    // Update the state structure
    virtual void update() = 0;

protected:

    // update status based on rpm measurment
    void update_status();

    // set status and update valid_count
    void set_status(Status status);
    
    EFI_State &state;

    int8_t get_uavcan_node_id(void) const;
    float get_coef1(void) const;
    float get_coef2(void) const;
};
