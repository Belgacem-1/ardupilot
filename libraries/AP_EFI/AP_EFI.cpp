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
#include <stdio.h>

extern const AP_HAL::HAL& hal;
#define VOLTS_TO_LITER 3.49f

// table of user settable parameters
const AP_Param::GroupInfo AP_EFI::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: EFI communication type
    // @Description: What method of communication is used for EFI #1
    // @Values: 0:None,1:Serial-MS,2:Serial-fiala
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_EFI, param[0].type, 0, AP_PARAM_FLAG_ENABLE),

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

    // @Param: ORIENT
    // @DisplayName: EFI orientation
    // @Description: Orientation of efi
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("_ORIENT", 4, AP_EFI, param[0].orientation, ROTATION_PITCH_270),

    // @Param: _INDEX
    // @DisplayName: ECU index
    // @Description: Used to identify the reference of ecu
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("_INDEX", 5, AP_EFI, param[0].index, 0),

    // @Param: _RATIO
    // @DisplayName: fuel tank level ratio
    // @Description: Calibrate fuel tank level sensor volt to liter. Increasing this value will indicate a higher fuel quantity at any given dynamic level.
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_RATIO",  6, AP_EFI, ratio, 3.49f),

#if EFI_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second EFI communication type
    // @Description: What method of communication is used for EFI #2
    // @Values: 0:None,1:Serial-MS,2:Fiala-EM
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("2_TYPE", 7, AP_EFI, param[1].type, 0, AP_PARAM_FLAG_ENABLE),
#endif
    // @Param: 2_ORIENT
    // @DisplayName: EFI orientation
    // @Description: Orientation of efi
    // @Values: 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("2_ORIENT", 8, AP_EFI, param[1].orientation, ROTATION_PITCH_270),

    // @Param: 2_INDEX
    // @DisplayName: ECU index
    // @Description: Used to identify the reference of ecu
    // @User: Standard
    // @RebootRequired: False
    AP_GROUPINFO("2_INDEX", 9, AP_EFI, param[1].index, 0),

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
    source = hal.analogin->channel(15);
    printf("EFI init\n");
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<EFI_MAX_INSTANCES; i++) {
        WITH_SEMAPHORE(sem);
        const EFI_Communication_Type _type = (EFI_Communication_Type)param[i].type.get();
        switch (_type) {
            case EFI_Communication_Type::EFI_COMMUNICATION_TYPE_NONE:
                // nothing to do
                break;
            case EFI_Communication_Type::EFI_COMMUNICATION_TYPE_SERIAL_MS:
                drivers[i] = new AP_EFI_Serial_MS(*this, state[i], i);
                break;
            case EFI_Communication_Type::EFI_COMMUNICATION_TYPE_SERIAL_FIALA:
                // Check for Fiala EM
                if (AP_EFI_Serial_Fiala::detect(i)) {
		            printf("Fiala instance %u\n",i);
                    drivers[i] = new AP_EFI_Serial_Fiala(*this, state[i], i);
                }
                break;
        }

        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1; // num_instances is a high-water-mark
        }
        // initialise status
        state[i].status = Status::NotConnected;
        state[i].range_valid_count = 0;
    } 
}

// Ask all backends to update the frontend
void AP_EFI::update()
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i]) {
           if((EFI_Communication_Type)param[i].type.get() != EFI_Communication_Type::EFI_COMMUNICATION_TYPE_NONE){
                // allow user to disable a rangefinder at runtime
                state[i].status = Status::NotConnected;
                state[i].range_valid_count = 0;
                continue;
            }
        }
		   printf("EFI update %u\n",i);
           drivers[i]->update();
           //log_status(i);
    }
    log_efi(); 
}  

AP_EFI_Backend *AP_EFI::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if ((EFI_Communication_Type)param[id].type.get() == EFI_Communication_Type::EFI_COMMUNICATION_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

Status AP_EFI::status_orient(enum Rotation orientation) const
{
    AP_EFI_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return Status::NotConnected;
    }
    return backend->status();
}

// return true if we have a range finder with the specified orientation
bool AP_EFI::has_orientation(enum Rotation orientation) const
{
    return (find_instance(orientation) != nullptr);
}

bool AP_EFI::has_data_orient(enum Rotation orientation) const
{
    AP_EFI_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return false;
    }
    return backend->has_data();
}

uint8_t AP_EFI::range_valid_count_orient(enum Rotation orientation) const
{
    AP_EFI_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->range_valid_count();
}

uint32_t AP_EFI::last_reading_ms(enum Rotation orientation) const
{
    AP_EFI_Backend *backend = find_instance(orientation);
    if (backend == nullptr) {
        return 0;
    }
    return backend->last_reading_ms();
}

// find first efi instance with the specified orientation
AP_EFI_Backend *AP_EFI::find_instance(enum Rotation orientation) const
{
    // first try for an efi that is in range
    for (uint8_t i=0; i<num_instances; i++) {
        AP_EFI_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation &&
            backend->status() == Status::Good) {
            return backend;
        }
    }
    // if none in range then return first with correct orientation
    for (uint8_t i=0; i<num_instances; i++) {
        AP_EFI_Backend *backend = get_backend(i);
        if (backend != nullptr &&
            backend->orientation() == orientation) {
            return backend;
        }
    }
    return nullptr;
}

bool AP_EFI::is_healthy(uint8_t i) const
{
    return (drivers[i] && (AP_HAL::millis() - state[i].last_reading_ms) < HEALTHY_LAST_RECEIVED_MS);
}

bool AP_EFI::get_fuel_level(float &tfl)
{
    if (source == nullptr) {
        return false;
    }
    // allow pin to change
    source->set_pin(15);
    tfl = source->voltage_average_ratiometric() * ratio;
    return true;
}

// Write an RFND (rangefinder) packet
void AP_EFI::log_efi()
{
    if (_log_efi_bit == uint32_t(-1)) {
        return;
    }

    AP_Logger &logger = AP::logger();
    if (!logger.should_log(_log_efi_bit)) {
        return;
    }
    float fuel_level = 0;
    for (uint8_t i=0; i<EFI_MAX_INSTANCES; i++) {
        const AP_EFI_Backend *s = get_backend(i);
        if (s == nullptr) {
            continue;
        }

        const struct log_EFI pkt = {
                LOG_PACKET_HEADER_INIT(LOG_EFI_MSG),
                time_us           : AP_HAL::micros64(),
                instance          : i,
                rpm               : uint32_t(state[i].engine_speed_rpm),
                status            : uint8_t(state[i].status),
                ecu_index         : uint8_t(param[i].index),
                tps               : uint8_t(state[i].throttle_position_percent),
                cht1              : float(state[i].cylinder_status[0].cylinder_head_temperature),
                cht2              : float(state[i].cylinder_status[1].cylinder_head_temperature),
                injection_length  : float(state[i].cylinder_status[0].injection_time_ms),
                input_voltage     : float(state[i].input_voltage),
                servo_voltage     : float(state[i].servo_voltage),
                fuel_pressure     : float(state[i].fuel_pressure),
                fuel_cons_rate    : float(state[i].fuel_consumption_rate),
                est_cons_fuel     : float(state[i].estimated_consumed_fuel),
                fuel_tank_level   : get_fuel_level(fuel_level)?fuel_level:float(state[i].fuel_tank_level),
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}

/*
  write status to log
 */
/*void AP_EFI::log_status(uint8_t i)
{
    float fuel_level = 0;   
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
                       get_fuel_level(fuel_level)?fuel_level:float(state[i].fuel_tank_level),
                       float(state[i].intake_manifold_temperature),
                       float(state[i].coolant_temperature),
                       float(state[i].oil_pressure),
                       float(state[i].oil_temperature),
                       float(state[i].intake_manifold_pressure_kpa),
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
}*/

/*
  send EFI_STATUS
 */
void AP_EFI::send_mavlink_efi_status(mavlink_channel_t chan)
{
    if (!drivers[0]) {
        return;
    }
    printf("send mavlink message motor1\n");
    float fuel_level = 0;
    mavlink_msg_efi_status_send(
        chan,
        AP_EFI::is_healthy(0),
        param[0].index,
        AP_HAL::micros64(),
        state[0].engine_speed_rpm,
        state[0].estimated_consumed_fuel,
        state[0].fuel_consumption_rate,
        0,
        state[0].throttle_position_percent,
        0, 0,
        state[0].fuel_pressure,
        0,
        state[0].cylinder_status[0].cylinder_head_temperature,
        0,
        state[0].cylinder_status[0].injection_time_ms,
        0, 0, 0,
        state[0].cylinder_status[1].cylinder_head_temperature,
        get_fuel_level(fuel_level)?fuel_level:state[0].fuel_tank_level,
        state[0].input_voltage,
        state[0].servo_voltage);
}

#if EFI_MAX_INSTANCES > 1
void AP_EFI::send_mavlink_efi2_status(mavlink_channel_t chan)
{
    if (!drivers[1]) {
        return;
    }
    printf("send mavlink message motor2\n");
    mavlink_msg_efi2_status_send(
        chan,
        AP_EFI::is_healthy(1),
        param[1].index,
        AP_HAL::micros64(),
        state[1].engine_speed_rpm,
        state[1].estimated_consumed_fuel,
        state[1].fuel_consumption_rate,
        state[1].throttle_position_percent,
        state[1].fuel_pressure,
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

