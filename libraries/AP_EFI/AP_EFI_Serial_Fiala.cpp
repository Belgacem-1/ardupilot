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
 
#include <AP_HAL/AP_HAL.h>
#include "AP_EFI_Serial_Fiala.h"

#if EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

AP_EFI_Serial_Fiala::AP_EFI_Serial_Fiala(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    internal_state.estimated_consumed_fuel_volume_cm3 = 0; // Just to be sure
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI_Fiala, 0);
}


void AP_EFI_Serial_Fiala::update()
{
    if (!port) {
        return;
    }

    uint32_t now = AP_HAL::millis();

    const uint32_t expected_bytes = 2 + (RT_LAST_OFFSET - RT_FIRST_OFFSET) + 4;
    if (port->available() >= expected_bytes && read_incoming_realtime_data()) {
        last_response_ms = now;
        copy_to_frontend();
    }

    if (port->available() == 0 || now - last_response_ms > 200) {
        port->discard_input();
}

bool AP_EFI_Serial_Fiala::read_incoming_realtime_data() 
{
    // Data is parsed directly from the buffer, otherwise we would need to allocate
    // several hundred bytes for the entire realtime data table or request every
    // value individiually
    // reset sum before reading new data
    sum = 0;

    // Response Flag (see "response_codes" enum)
    head_flag1 = read_byte();
    head_flag2 = read_byte();
    packet_flag = read_byte();
    if (head_flag1 != HEAD_BYTE_1 && head_flag2 != HEAD_BYTE_2 && packet_flag!= PACKET_ID) {
        // abort read if we did not receive the correct response code;
        return false;
    }
    
    // Iterate over the payload bytes 
    for ( uint8_t offset=RT_FIRST_OFFSET; offset < (RT_LAST_OFFSET - RT_FIRST_OFFSET + 1); offset++) {
        uint8_t data = read_byte();
        float temp_float;
        switch (offset) {
            case CHT1_MSB:
                offset++;
                internal_state.cylinder_status[0].cylinder_head_temperature = (float)((data << 8) + read_byte());
                break;
            case CHT2_MSB:
                offset++;
                internal_state.cylinder_status[1].cylinder_head_temperature = (float)((data << 8) + read_byte());
                break;
            case EGT1_MSB:
                offset++;
                internal_state.cylinder_status[0].exhaust_gas_temperature = (float)((data << 8) + read_byte());
                break;
            case EGT2_MSB:
                offset++;
                internal_state.cylinder_status[1].exhaust_gas_temperature = (float)((data << 8) + read_byte());
                break;
            case OT_MSB:
                internal_state.outside_temperature = (float)((data << 8) + read_byte());
                offset++;
            case RPM_MSB:
                internal_state.engine_speed_rpm = (data << 8) + read_byte();
                offset++;
                break;
            case INV_MSB:
                internal_state.input_voltage = (float)((data << 8) + read_byte())/10.0f;
                offset++;
                break;
            case SV_MSB:
                internal_state.servo_voltage = (float)((data << 8) + read_byte())/10.0f;
                offset++;
                break;
            case TPS_MSB:
                offset++;
                internal_state.throttle_position_percent = (float)((data << 8) + read_byte());
                break;
            case CPS_MSB:
                offset++;
                internal_state.cooler_position_percent = data;
                break;
            case FTL_MSB:
                offset++;
                temp_float = (float)((data << 8) + read_byte())/10.0f;
                internal_state.fuel_tank_level = temp_float;
                break;
            case FP_MSB:
                // Fiala Fuel Pressure is unitless, store as bar anyway
                temp_float = (float)((data << 8) + read_byte())/10.0f;
                internal_state.fuel_pressure = temp_float;
                offset++;
                break;
            case FCR_MSB:
                temp_float = ((float)((data << 8) + read_byte())/100.0f)*16.66;
                internal_state.fuel_consumption_rate_cm3pm = temp_float;
                offset++;
                break; 
            case IL_MSB:
                temp_float = (float)((data << 8) + read_byte())/1000.0f;
                internal_state.cylinder_status[0].injection_time_ms = temp_float;
                offset++;
                break;               
        }
    }
    
    // Read the four CRC bytes
    uint8_t received_sum;
    received_sum = port->read();
                        
    if (received_sum != sum) {
        // hal.console->printf("EFI CRC: 0x%08x 0x%08x\n", received_CRC, checksum);
        return false;
    }

    // Calculate consumed Fuel 
    uint32_t current_time = AP_HAL::millis();
    // Super Simplified integration method - Error Analysis TBD
    // This calcualtion gives erroneous results when the engine isn't running
    if (internal_state.engine_speed_rpm > RPM_THRESHOLD) {
        internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * (current_time - internal_state.last_updated_ms)/60000.0f;
    }
    internal_state.last_updated_ms = current_time;
    
    return true;
         
}

uint8_t AP_EFI_Serial_Fiala::read_byte()
{   
    // Read a byte and update the CRC 
    uint8_t data = port->read();
    sum = compute_byte(sum, data);
    return data;
}

// Sum matching Fiala
uint8_t AP_EFI_Serial_Fiala::compute_byte(uint8_t sum_b, uint8_t data)
{
    sum = sum + data;
    return sum;
}

#endif // EFI_ENABLED
