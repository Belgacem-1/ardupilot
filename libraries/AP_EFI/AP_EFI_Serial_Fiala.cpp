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
#include <stdio.h>
#if EFI_ENABLED

extern const AP_HAL::HAL &hal;

bool AP_EFI_Serial_Fiala::get_reading()
{
    if (port==nullptr) {
		hal.console->printf("port not available before\n");
        return false;
    }

    uint32_t now = AP_HAL::millis();

    const uint32_t expected_bytes = 2 + (RT_LAST_OFFSET - RT_FIRST_OFFSET) + 4;
    if (port->available() >= expected_bytes && read_incoming_realtime_data()) {
        last_response_ms = now;
    }

    if (port->available() == 0 || now - last_response_ms > 200) {
		hal.console->printf("now-last_response_ms= %lu",now - last_response_ms);
		hal.console->printf("port not available after\n");
        port->discard_input();
        return false;
    }
    return true;
}

// read - return last value measured by sensor
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
    hal.console->printf("Reading bytes\n");
    if (head_flag1 != HEAD_BYTE_1 && head_flag2 != HEAD_BYTE_2 && packet_flag!= PACKET_ID) {
        // abort read if we did not receive the correct response code;
        return false;
    }
    
    // Iterate over the payload bytes 
    for ( uint8_t offset=RT_FIRST_OFFSET-1; offset < RT_LAST_OFFSET; offset++) {
        uint8_t data = read_byte();
        hal.console->printf("offset= %u\n",offset);
        float temp_float;
        switch (offset) {
            case CHT1_MSB:
                offset++;
                state.cylinder_status[0].cylinder_head_temperature = (float)(data + (read_byte()<< 8));
                break;
            case CHT2_MSB:
                offset++;
                state.cylinder_status[1].cylinder_head_temperature = (float)(data + (read_byte()<< 8));
                break;
            case EGT1_MSB:
                offset++;
                state.cylinder_status[0].exhaust_gas_temperature = (float)(data + (read_byte()<< 8));
                break;
            case EGT2_MSB:
                offset++;
                state.cylinder_status[1].exhaust_gas_temperature = (float)(data + (read_byte()<< 8));
                break;
            case OT_MSB:
                state.outside_temperature = (float)(data + (read_byte()<< 8));
                offset++;
                break;
            case RPM_MSB:
                state.engine_speed_rpm = (float)(data + (read_byte()<< 8));
                offset++;
                break;
            case INV_MSB:
                state.input_voltage = (float)(data + (read_byte()<< 8))/10.0f;
                offset++;
                break;
            case SV_MSB:
                state.servo_voltage = (float)(data + (read_byte()<< 8))/10.0f;
                offset++;
                break;
            case TPS_MSB:
                offset++;
                state.throttle_position_percent = (float)(data + (read_byte()<< 8));
                break;
            case CPS_MSB:
                state.cooler_position_percent = data;
                break;
            case FTL_MSB:
                offset++;
                temp_float = (float)(data + (read_byte()<< 8))/10.0f;
                state.fuel_tank_level = temp_float;
                break;
            case FP_MSB:
                // Fiala Fuel Pressure is unitless, store as bar anyway
                temp_float = (float)(data + (read_byte()<< 8))/10.0f;
                state.fuel_pressure = temp_float;
                offset++;
                break;
            case FCR_MSB:
                temp_float = (float)(data + (read_byte()<< 8))/100.0f;
                state.fuel_consumption_rate = temp_float;
                offset++;
                break; 
            case IL_MSB:
                temp_float = (float)(data + (read_byte()<< 8));
                state.cylinder_status[0].injection_time_ms = temp_float;
                offset++;
                break;               
        }
    }
    
    // Read the four CRC bytes
    uint8_t received_sum;
    received_sum = port->read();
                        
    if (received_sum != sum) {
        // hal.console->printf("EFI CRC: 0x%08x 0x%08x\n", received_CRC, checksum);
        hal.console->printf("packet not ok\n");
        return false;
    }

    // Calculate consumed Fuel 
    uint32_t current_time = AP_HAL::millis();
    // Super Simplified integration method - Error Analysis TBD
    // This calcualtion gives erroneous results when the engine isn't running
    if (state.engine_speed_rpm > RPM_THRESHOLD) {
        state.estimated_consumed_fuel += state.fuel_consumption_rate * (current_time - state.last_reading_ms)/3600000.0f;
    }
    else {
        state.fuel_consumption_rate = 0;
    }
    
    return true;
         
}

uint8_t AP_EFI_Serial_Fiala::read_byte()
{   
    hal.console->printf("update crc\n");
    // Read a byte and update the CRC 
    uint8_t data = port->read();
    sum = compute_byte(sum, data);
    return data;
}

// Sum matching Fiala
uint8_t AP_EFI_Serial_Fiala::compute_byte(uint8_t sum_b, uint8_t data)
{
    hal.console->printf("crc computing\n");
    sum = sum + data;
    return sum;
}

#endif // EFI_ENABLED
