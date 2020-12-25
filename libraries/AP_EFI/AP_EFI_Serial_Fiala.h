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
#include "AP_EFI_Backend_Serial.h"

// RPM Threshold for fuel consumption estimator
#define RPM_THRESHOLD                100

class AP_EFI_Serial_Fiala : public AP_EFI_Backend {
    
public:

    using AP_EFI_Backend_Serial::AP_EFI_Backend_Serial;

private:

   // get a reading
    bool get_reading() override;
    bool read_incoming_realtime_data();
    uint8_t read_byte();
    uint8_t compute_byte(uint8_t inCrc32, uint8_t data);
    
    // Serial Protocol Variables
    uint8_t sum;
    uint8_t head_flag1;
    uint8_t head_flag2;
    uint8_t packet_flag;
    uint32_t last_response_ms;

    // Header Codes
    enum header {
        HEAD_BYTE_1 =0xA5,
        HEAD_BYTE_2 =0x5A,
        PACKET_ID =0x01
    };
    
    // Realtime Data Table Locations
    enum realtime_data {
        CHT1_MSB = 4,
        CHT1_LSB,
        CHT2_MSB,
        CHT2_LSB,
        EGT1_MSB,
        EGT1_LSB,
        EGT2_MSB,
        EGT2_LSB,
        EGT3_MSB,
        EGT3_LSB,
        EGT4_MSB,
        EGT4_LSB,
        OT_MSB,
        OT_LSB,
        RPM_MSB,
        RPM_LSB,
        INV_MSB,
        INV_LSB,
        SV_MSB,
        SV_LSB,
        TPS_MSB =26,
        TPS_LSB,
        CPS_MSB,
        FTL_MSB = 30,
        FTL_LSB,
        FP_MSB,
        FP_LSB,
        FCR_MSB,
        FC_LSB,
        IL_MSB = 44,
        IL_LSB,
        RT_FIRST_OFFSET = CHT1_MSB,
        RT_LAST_OFFSET = IL_LSB
    };
};
