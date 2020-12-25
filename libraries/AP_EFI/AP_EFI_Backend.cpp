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

#include "AP_EFI.h"

#if EFI_ENABLED

#include "AP_EFI_Backend.h"

extern const AP_HAL::HAL &hal;

AP_EFI_Backend::AP_EFI_Backend(EFI_State &_state) : state(_state)
{
}

// update status based on rpm measurement
void AP_EFI_Backend::update_status()
{
    // check distance
    if ((int16_t)state.distance_cm > params.max_distance_cm) {
        set_status(Status::OutOfRangeHigh);
    } else if ((int16_t)state.distance_cm < params.min_distance_cm) {
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
