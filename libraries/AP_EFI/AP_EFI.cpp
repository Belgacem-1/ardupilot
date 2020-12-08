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

#include "AP_EFI_Serial_MS.h"
#include "AP_EFI_Serial_Fiala.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_EFI::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: EFI communication type
    // @Description: What method of communication is used for EFI #1
    // @Values: 0:None,1:Serial-MS,2:Serial-fiala
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_EFI, type[0], 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _COEF1
    // @DisplayName: EFI Calibration Coefficient 1
    // @Description: Used to calibrate fuel flow for MS protocol (Slope)
    // @Range: 0 1
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("_COEF1", 2, AP_EFI, coef1, 0),

    // @Param: _COEF2
    // @DisplayName: EFI Calibration Coefficient 2
    // @Description: Used to calibrate fuel flow for MS protocol (Offset)
    // @Range: 0 10
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("_COEF2", 3, AP_EFI, coef2, 0),

#if EFI_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second EFI communication type
    // @Description: What method of communication is used for EFI #2
    // @Values: 0:None,1:Serial-MS,2:Fiala-EM
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 4, AP_EFI, type[1], 0, AP_PARAM_FLAG_ENABLE),
#endif

    AP_GROUPEND
};

AP_EFI *AP_EFI::singleton;

// Initialize parameters
AP_EFI::AP_EFI()
{
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Initialize backends based on existing params
void AP_EFI::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<EFI_MAX_INSTANCES; i++) {
        // Check for MegaSquirt Serial EFI
        if (type[i] == EFI_COMMUNICATION_TYPE_SERIAL_MS) {
           backend[i] = new AP_EFI_Serial_MS(*this, i);
        }
        // Check for Fiala EM
        if (type[i] == EFI_COMMUNICATION_TYPE_SERIAL_FIALA) {
           backend[i] = new AP_EFI_Serial_Fiala(*this, i);
        }
        if (backend[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1; // num_instances is a high-water-mark
        }
    } 
}

// Ask all backends to update the frontend
void AP_EFI::update()
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (backend[i]) {
           backend[i]->update();
           log_status(i);
        }
    }   
}

bool AP_EFI::is_healthy(uint8_t i) const
{
    return (backend[i] && (AP_HAL::millis() - state[i].last_updated_ms) < HEALTHY_LAST_RECEIVED_MS);
}

/*
  write status to log
 */
void AP_EFI::log_status(uint8_t i)
{
// @LoggerMessage: EFI
// @Description: Electronic Fuel Injection system data
// @Field: TimeUS: Time since system startup
// @Field: LP: Reported engine load
// @Field: Rpm: Reported engine RPM
// @Field: IV: Input voltage
// @Field: SV: Servo voltage
// @Field: FTL: Fuel tank level
// @Field: IMT: Intake manifold temperature
// @Field: ECT: Engine Coolant Temperature
// @Field: OilP: Oil Pressure
// @Field: OilT: Oil temperature
// @Field: FP: Fuel Pressure
// @Field: FCR: Fuel Consumption Rate
// @Field: CFV: Consumed fueld volume
// @Field: TPS: Throttle Position
// @Field: IDX: Index of the publishing ECU
    AP::logger().Write("EFI"+i,
                       "TimeUS,LP,Rpm,IV,SV,FTL,IMT,ECT,OilP,OilT,FP,FCR,CFV,TPS,IDX",
                       "s%qvv-OOPOP--%-",
                       "F0000000-0-0000",
                       "QBIffffffffffHB",
                       AP_HAL::micros64(),
                       uint8_t(state[i].engine_load_percent),
                       uint32_t(state[i].engine_speed_rpm),
                       float(state[i].input_voltage),
                       float(state[i].servo_voltage),
                       float(state[i].fuel_tank_level),
                       float(state[i].intake_manifold_temperature),
                       float(state[i].coolant_temperature),
                       float(state[i].oil_pressure),
                       float(state[i].oil_temperature),
                       float(state[i].fuel_pressure),
                       float(state[i].fuel_consumption_rate_cm3pm),
                       float(state[i].estimated_consumed_fuel_volume_cm3),
                       uint16_t(state[i].throttle_position_percent),
                       uint8_t(state[i].ecu_index));
 
// @LoggerMessage: EM
// @Description: Electronic Fuel Injection system data - redux
// @Field: TimeUS: Time since system startup
// @Field: Healthy: True if EFI is healthy
// @Field: ES: Engine state
// @Field: GE: General error
// @Field: CSE: Crankshaft sensor status
// @Field: TS: Temperature status
// @Field: FPS: Fuel pressure status
// @Field: OPS: Oil pressure status
// @Field: DS: Detonation status
// @Field: MS: Misfire status
// @Field: DebS: Debris status
// @Field: SPU: Spark plug usage
// @Field: IDX: Index of the publishing ECU
    AP::logger().Write("EM"+i,
                       "TimeUS,Healthy,ES,GE,CSE,TS,FPS,OPS,DS,MS,DebS,SPU,IDX",
                       "s------------",
                       "F------------",
                       "QBBBBBBBBBBBB",
                       AP_HAL::micros64(),
                       uint8_t(is_healthy(i)),
                       uint8_t(state[i].engine_state),
                       uint8_t(state[i].general_error),
                       uint8_t(state[i].crankshaft_sensor_status),
                       uint8_t(state[i].temperature_status),
                       uint8_t(state[i].fuel_pressure_status),
                       uint8_t(state[i].oil_pressure_status),
                       uint8_t(state[i].detonation_status),
                       uint8_t(state[i].misfire_status),
                       uint8_t(state[i].debris_status),
                       uint8_t(state[i].spark_plug_usage),
                       uint8_t(state[i].ecu_index));

    for (uint8_t j = 0; j < ENGINE_MAX_CYLINDERS; j++) {
// @LoggerMessage: ECYL
// @Description: EFI per-cylinder information
// @Field: TimeUS: Time since system startup
// @Field: Inst: Cylinder this data belongs to
// @Field: IgnT: Ignition timing
// @Field: InjT: Injection time
// @Field: CHT: Cylinder head temperature
// @Field: EGT: Exhaust gas temperature
// @Field: Lambda: Estimated lambda coefficient (dimensionless ratio)
// @Field: IDX: Index of the publishing ECU
        AP::logger().Write("ECY"+i,
                           "TimeUS,Inst,IgnT,InjT,CHT,EGT,Lambda,IDX",
                           "s#dsOO--",
                           "F-0C0000",
                           "QBfffffB",
                           AP_HAL::micros64(),
                           j,
                           state[i].cylinder_status[j].ignition_timing_deg,
                           state[i].cylinder_status[j].injection_time_ms,
                           state[i].cylinder_status[j].cylinder_head_temperature,
                           state[i].cylinder_status[j].exhaust_gas_temperature,
                           state[i].cylinder_status[j].lambda_coefficient,
                           state[i].ecu_index);
    }
}

/*
  send EFI_STATUS
 */
void AP_EFI::send_mavlink_efi_status(mavlink_channel_t chan)
{
    if (!backend) {
        return;
    }
    mavlink_msg_efi_status_send(
        chan,
        AP_EFI::is_healthy(0),
        state[0].ecu_index,
        state[0].engine_speed_rpm,
        state[0].estimated_consumed_fuel_volume_cm3,
        state[0].fuel_consumption_rate_cm3pm,
        0,
        state[0].throttle_position_percent,
        0, 0,
        state[0].intake_manifold_pressure_kpa,
        0,
        state[0].cylinder_status[0].cylinder_head_temperature,
        0,
        state[0].cylinder_status[0].injection_time_ms,
        0, 0, 0,
        state[0].cylinder_status[1].cylinder_head_temperature,
        state[0].fuel_tank_level,
        state[0].input_voltage,
        state[0].servo_voltage);
}

#if EFI_MAX_INSTANCES > 1
void AP_EFI::send_mavlink_efi2_status(mavlink_channel_t chan)
{
    if (!backend) {
        return;
    }
    mavlink_msg_efi2_status_send(
        chan,
        AP_EFI::is_healthy(1),
        state[1].engine_speed_rpm,
        state[1].estimated_consumed_fuel_volume_cm3,
        state[1].fuel_consumption_rate_cm3pm,
        state[1].throttle_position_percent,
        state[1].intake_manifold_pressure_kpa,
        state[1].cylinder_status[0].cylinder_head_temperature,
        state[1].cylinder_status[0].injection_time_ms,
        state[1].cylinder_status[1].cylinder_head_temperature,
        state[1].input_voltage,
        state[1].servo_voltage);
}
#endif
namespace AP {
AP_EFI *EFI()
{
    return AP_EFI::get_singleton();
}
}

#endif // EFI_ENABLED

