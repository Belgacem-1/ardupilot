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
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_EFI.h"

#if EFI_ENABLED

#include "AP_EFI_Backend.h"


extern const AP_HAL::HAL &hal;

AP_EFI_Backend::AP_EFI_Backend(AP_EFI &_frontend, EFI_State &_state, uint8_t _instance) : frontend(_frontend), state(_state), instance(_instance)
{
    _backend_type = type();
}

Status AP_EFI_Backend::status() const {
    if (type() == AP_EFI::EFI_Communication_Type::EFI_COMMUNICATION_TYPE_NONE) {
        // turned off at runtime?
        return Status::NotConnected;
    }
    return state.status;
}

// true if sensor is returning data
bool AP_EFI_Backend::has_data() const {
    return ((state.status != Status::NotConnected) &&
            (state.status != Status::NoData));
}

// update status based on rpm measurement
void AP_EFI_Backend::update_status()
{
    // check rpm
    if ((int16_t)state.engine_speed_rpm > 7000) {
        set_status(Status::OutOfRangeHigh);
    } else if ((int16_t)state.engine_speed_rpm < 100) {
        set_status(Status::OutOfRangeLow);
    } else {
        set_status(Status::Good);
    }
}

// set status and update valid count
void AP_EFI_Backend::set_status(Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == Status::Good) {
        if (state.range_valid_count < 10) {
            state.range_valid_count++;
        }
    } else {
        state.range_valid_count = 0;
    }
}

float AP_EFI_Backend::get_coef1(void) const
{
    return frontend.coef1;
}

float AP_EFI_Backend::get_coef2(void) const
{
    return frontend.coef2;
}
#endif // EFI_ENABLED
