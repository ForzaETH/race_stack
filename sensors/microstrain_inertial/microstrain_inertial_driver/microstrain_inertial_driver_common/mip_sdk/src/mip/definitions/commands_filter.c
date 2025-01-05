
#include "commands_filter.h"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;


////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert_mip_filter_reference_frame(struct mip_serializer* serializer, const mip_filter_reference_frame self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_reference_frame(struct mip_serializer* serializer, mip_filter_reference_frame* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_mag_param_source(struct mip_serializer* serializer, const mip_filter_mag_param_source self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_mag_param_source(struct mip_serializer* serializer, mip_filter_mag_param_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_adaptive_measurement(struct mip_serializer* serializer, const mip_filter_adaptive_measurement self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_adaptive_measurement(struct mip_serializer* serializer, mip_filter_adaptive_measurement* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

mip_cmd_result mip_filter_reset(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RESET_FILTER, NULL, 0);
}
void insert_mip_filter_set_initial_attitude_command(mip_serializer* serializer, const mip_filter_set_initial_attitude_command* self)
{
    insert_float(serializer, self->roll);
    
    insert_float(serializer, self->pitch);
    
    insert_float(serializer, self->heading);
    
}
void extract_mip_filter_set_initial_attitude_command(mip_serializer* serializer, mip_filter_set_initial_attitude_command* self)
{
    extract_float(serializer, &self->roll);
    
    extract_float(serializer, &self->pitch);
    
    extract_float(serializer, &self->heading);
    
}

mip_cmd_result mip_filter_set_initial_attitude(struct mip_interface* device, float roll, float pitch, float heading)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, roll);
    
    insert_float(&serializer, pitch);
    
    insert_float(&serializer, heading);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_ATTITUDE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_estimation_control_command(mip_serializer* serializer, const mip_filter_estimation_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_estimation_control_command_enable_flags(serializer, self->enable);
        
    }
}
void extract_mip_filter_estimation_control_command(mip_serializer* serializer, mip_filter_estimation_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_estimation_control_command_enable_flags(serializer, &self->enable);
        
    }
}

void insert_mip_filter_estimation_control_response(mip_serializer* serializer, const mip_filter_estimation_control_response* self)
{
    insert_mip_filter_estimation_control_command_enable_flags(serializer, self->enable);
    
}
void extract_mip_filter_estimation_control_response(mip_serializer* serializer, mip_filter_estimation_control_response* self)
{
    extract_mip_filter_estimation_control_command_enable_flags(serializer, &self->enable);
    
}

void insert_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, const mip_filter_estimation_control_command_enable_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_filter_estimation_control_command_enable_flags(struct mip_serializer* serializer, mip_filter_estimation_control_command_enable_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_estimation_control(struct mip_interface* device, mip_filter_estimation_control_command_enable_flags enable)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_estimation_control_command_enable_flags(&serializer, enable);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_estimation_control(struct mip_interface* device, mip_filter_estimation_control_command_enable_flags* enable_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_mip_filter_estimation_control_command_enable_flags(&deserializer, enable_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_estimation_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_estimation_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_estimation_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_external_gnss_update_command(mip_serializer* serializer, const mip_filter_external_gnss_update_command* self)
{
    insert_double(serializer, self->gps_time);
    
    insert_u16(serializer, self->gps_week);
    
    insert_double(serializer, self->latitude);
    
    insert_double(serializer, self->longitude);
    
    insert_double(serializer, self->height);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->vel_uncertainty[i]);
    
}
void extract_mip_filter_external_gnss_update_command(mip_serializer* serializer, mip_filter_external_gnss_update_command* self)
{
    extract_double(serializer, &self->gps_time);
    
    extract_u16(serializer, &self->gps_week);
    
    extract_double(serializer, &self->latitude);
    
    extract_double(serializer, &self->longitude);
    
    extract_double(serializer, &self->height);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->vel_uncertainty[i]);
    
}

mip_cmd_result mip_filter_external_gnss_update(struct mip_interface* device, double gps_time, uint16_t gps_week, double latitude, double longitude, double height, const float* velocity, const float* pos_uncertainty, const float* vel_uncertainty)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_double(&serializer, gps_time);
    
    insert_u16(&serializer, gps_week);
    
    insert_double(&serializer, latitude);
    
    insert_double(&serializer, longitude);
    
    insert_double(&serializer, height);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(pos_uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, pos_uncertainty[i]);
    
    assert(vel_uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, vel_uncertainty[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_GNSS_UPDATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_external_heading_update_command(mip_serializer* serializer, const mip_filter_external_heading_update_command* self)
{
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->heading_uncertainty);
    
    insert_u8(serializer, self->type);
    
}
void extract_mip_filter_external_heading_update_command(mip_serializer* serializer, mip_filter_external_heading_update_command* self)
{
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->heading_uncertainty);
    
    extract_u8(serializer, &self->type);
    
}

mip_cmd_result mip_filter_external_heading_update(struct mip_interface* device, float heading, float heading_uncertainty, uint8_t type)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, heading);
    
    insert_float(&serializer, heading_uncertainty);
    
    insert_u8(&serializer, type);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_external_heading_update_with_time_command(mip_serializer* serializer, const mip_filter_external_heading_update_with_time_command* self)
{
    insert_double(serializer, self->gps_time);
    
    insert_u16(serializer, self->gps_week);
    
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->heading_uncertainty);
    
    insert_u8(serializer, self->type);
    
}
void extract_mip_filter_external_heading_update_with_time_command(mip_serializer* serializer, mip_filter_external_heading_update_with_time_command* self)
{
    extract_double(serializer, &self->gps_time);
    
    extract_u16(serializer, &self->gps_week);
    
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->heading_uncertainty);
    
    extract_u8(serializer, &self->type);
    
}

mip_cmd_result mip_filter_external_heading_update_with_time(struct mip_interface* device, double gps_time, uint16_t gps_week, float heading, float heading_uncertainty, uint8_t type)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_double(&serializer, gps_time);
    
    insert_u16(&serializer, gps_week);
    
    insert_float(&serializer, heading);
    
    insert_float(&serializer, heading_uncertainty);
    
    insert_u8(&serializer, type);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_tare_orientation_command(mip_serializer* serializer, const mip_filter_tare_orientation_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_tare_orientation_command_mip_tare_axes(serializer, self->axes);
        
    }
}
void extract_mip_filter_tare_orientation_command(mip_serializer* serializer, mip_filter_tare_orientation_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_tare_orientation_command_mip_tare_axes(serializer, &self->axes);
        
    }
}

void insert_mip_filter_tare_orientation_response(mip_serializer* serializer, const mip_filter_tare_orientation_response* self)
{
    insert_mip_filter_tare_orientation_command_mip_tare_axes(serializer, self->axes);
    
}
void extract_mip_filter_tare_orientation_response(mip_serializer* serializer, mip_filter_tare_orientation_response* self)
{
    extract_mip_filter_tare_orientation_command_mip_tare_axes(serializer, &self->axes);
    
}

void insert_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, const mip_filter_tare_orientation_command_mip_tare_axes self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_tare_orientation_command_mip_tare_axes(struct mip_serializer* serializer, mip_filter_tare_orientation_command_mip_tare_axes* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_tare_orientation(struct mip_interface* device, mip_filter_tare_orientation_command_mip_tare_axes axes)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_tare_orientation_command_mip_tare_axes(&serializer, axes);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_tare_orientation(struct mip_interface* device, mip_filter_tare_orientation_command_mip_tare_axes* axes_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(axes_out);
        extract_mip_filter_tare_orientation_command_mip_tare_axes(&deserializer, axes_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_tare_orientation(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_tare_orientation(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_tare_orientation(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_vehicle_dynamics_mode_command(mip_serializer* serializer, const mip_filter_vehicle_dynamics_mode_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, self->mode);
        
    }
}
void extract_mip_filter_vehicle_dynamics_mode_command(mip_serializer* serializer, mip_filter_vehicle_dynamics_mode_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, &self->mode);
        
    }
}

void insert_mip_filter_vehicle_dynamics_mode_response(mip_serializer* serializer, const mip_filter_vehicle_dynamics_mode_response* self)
{
    insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, self->mode);
    
}
void extract_mip_filter_vehicle_dynamics_mode_response(mip_serializer* serializer, mip_filter_vehicle_dynamics_mode_response* self)
{
    extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(serializer, &self->mode);
    
}

void insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(struct mip_serializer* serializer, const mip_filter_vehicle_dynamics_mode_command_dynamics_mode self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(struct mip_serializer* serializer, mip_filter_vehicle_dynamics_mode_command_dynamics_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_vehicle_dynamics_mode(struct mip_interface* device, mip_filter_vehicle_dynamics_mode_command_dynamics_mode mode)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(&serializer, mode);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_vehicle_dynamics_mode(struct mip_interface* device, mip_filter_vehicle_dynamics_mode_command_dynamics_mode* mode_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(mode_out);
        extract_mip_filter_vehicle_dynamics_mode_command_dynamics_mode(&deserializer, mode_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_vehicle_dynamics_mode(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_vehicle_dynamics_mode(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_vehicle_dynamics_mode(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_rotation_euler_command(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_float(serializer, self->roll);
        
        insert_float(serializer, self->pitch);
        
        insert_float(serializer, self->yaw);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_rotation_euler_command(mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_euler_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_float(serializer, &self->roll);
        
        extract_float(serializer, &self->pitch);
        
        extract_float(serializer, &self->yaw);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_rotation_euler_response(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_euler_response* self)
{
    insert_float(serializer, self->roll);
    
    insert_float(serializer, self->pitch);
    
    insert_float(serializer, self->yaw);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_euler_response(mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_euler_response* self)
{
    extract_float(serializer, &self->roll);
    
    extract_float(serializer, &self->pitch);
    
    extract_float(serializer, &self->yaw);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float roll, float pitch, float yaw)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_float(&serializer, roll);
    
    insert_float(&serializer, pitch);
    
    insert_float(&serializer, yaw);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_euler(struct mip_interface* device, float* roll_out, float* pitch_out, float* yaw_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(roll_out);
        extract_float(&deserializer, roll_out);
        
        assert(pitch_out);
        extract_float(&deserializer, pitch_out);
        
        assert(yaw_out);
        extract_float(&deserializer, yaw_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_euler(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_rotation_dcm_command(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            insert_float(serializer, self->dcm[i]);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_rotation_dcm_command(mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_dcm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            extract_float(serializer, &self->dcm[i]);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_rotation_dcm_response(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_dcm_response* self)
{
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->dcm[i]);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_dcm_response(mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_dcm_response* self)
{
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->dcm[i]);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, const float* dcm)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(dcm || (9 == 0));
    for(unsigned int i=0; i < 9; i++)
        insert_float(&serializer, dcm[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_dcm(struct mip_interface* device, float* dcm_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(dcm_out || (9 == 0));
        for(unsigned int i=0; i < 9; i++)
            extract_float(&deserializer, &dcm_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_dcm(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_command(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            insert_float(serializer, self->quat[i]);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_command(mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_quaternion_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            extract_float(serializer, &self->quat[i]);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_rotation_quaternion_response(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_rotation_quaternion_response* self)
{
    for(unsigned int i=0; i < 4; i++)
        insert_float(serializer, self->quat[i]);
    
}
void extract_mip_filter_sensor_to_vehicle_rotation_quaternion_response(mip_serializer* serializer, mip_filter_sensor_to_vehicle_rotation_quaternion_response* self)
{
    for(unsigned int i=0; i < 4; i++)
        extract_float(serializer, &self->quat[i]);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, const float* quat)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(quat || (4 == 0));
    for(unsigned int i=0; i < 4; i++)
        insert_float(&serializer, quat[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device, float* quat_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(quat_out || (4 == 0));
        for(unsigned int i=0; i < 4; i++)
            extract_float(&deserializer, &quat_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_rotation_quaternion(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_sensor_to_vehicle_offset_command(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->offset[i]);
        
    }
}
void extract_mip_filter_sensor_to_vehicle_offset_command(mip_serializer* serializer, mip_filter_sensor_to_vehicle_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->offset[i]);
        
    }
}

void insert_mip_filter_sensor_to_vehicle_offset_response(mip_serializer* serializer, const mip_filter_sensor_to_vehicle_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
    
}
void extract_mip_filter_sensor_to_vehicle_offset_response(mip_serializer* serializer, mip_filter_sensor_to_vehicle_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
    
}

mip_cmd_result mip_filter_write_sensor_to_vehicle_offset(struct mip_interface* device, const float* offset)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, offset[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_sensor_to_vehicle_offset(struct mip_interface* device, float* offset_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(offset_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &offset_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_sensor_to_vehicle_offset(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_sensor_to_vehicle_offset(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_sensor_to_vehicle_offset(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_antenna_offset_command(mip_serializer* serializer, const mip_filter_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->offset[i]);
        
    }
}
void extract_mip_filter_antenna_offset_command(mip_serializer* serializer, mip_filter_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->offset[i]);
        
    }
}

void insert_mip_filter_antenna_offset_response(mip_serializer* serializer, const mip_filter_antenna_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->offset[i]);
    
}
void extract_mip_filter_antenna_offset_response(mip_serializer* serializer, mip_filter_antenna_offset_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->offset[i]);
    
}

mip_cmd_result mip_filter_write_antenna_offset(struct mip_interface* device, const float* offset)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, offset[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_antenna_offset(struct mip_interface* device, float* offset_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(offset_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &offset_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_antenna_offset(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_antenna_offset(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_antenna_offset(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_gnss_source_command(mip_serializer* serializer, const mip_filter_gnss_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_gnss_source_command_source(serializer, self->source);
        
    }
}
void extract_mip_filter_gnss_source_command(mip_serializer* serializer, mip_filter_gnss_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_gnss_source_command_source(serializer, &self->source);
        
    }
}

void insert_mip_filter_gnss_source_response(mip_serializer* serializer, const mip_filter_gnss_source_response* self)
{
    insert_mip_filter_gnss_source_command_source(serializer, self->source);
    
}
void extract_mip_filter_gnss_source_response(mip_serializer* serializer, mip_filter_gnss_source_response* self)
{
    extract_mip_filter_gnss_source_command_source(serializer, &self->source);
    
}

void insert_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, const mip_filter_gnss_source_command_source self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_gnss_source_command_source(struct mip_serializer* serializer, mip_filter_gnss_source_command_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_gnss_source(struct mip_interface* device, mip_filter_gnss_source_command_source source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_gnss_source_command_source(&serializer, source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gnss_source(struct mip_interface* device, mip_filter_gnss_source_command_source* source_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_gnss_source_command_source(&deserializer, source_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gnss_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gnss_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gnss_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_heading_source_command(mip_serializer* serializer, const mip_filter_heading_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_heading_source_command_source(serializer, self->source);
        
    }
}
void extract_mip_filter_heading_source_command(mip_serializer* serializer, mip_filter_heading_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_heading_source_command_source(serializer, &self->source);
        
    }
}

void insert_mip_filter_heading_source_response(mip_serializer* serializer, const mip_filter_heading_source_response* self)
{
    insert_mip_filter_heading_source_command_source(serializer, self->source);
    
}
void extract_mip_filter_heading_source_response(mip_serializer* serializer, mip_filter_heading_source_response* self)
{
    extract_mip_filter_heading_source_command_source(serializer, &self->source);
    
}

void insert_mip_filter_heading_source_command_source(struct mip_serializer* serializer, const mip_filter_heading_source_command_source self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_heading_source_command_source(struct mip_serializer* serializer, mip_filter_heading_source_command_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_heading_source(struct mip_interface* device, mip_filter_heading_source_command_source source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_heading_source_command_source(&serializer, source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_heading_source(struct mip_interface* device, mip_filter_heading_source_command_source* source_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_heading_source_command_source(&deserializer, source_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_heading_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_heading_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_heading_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_auto_init_control_command(mip_serializer* serializer, const mip_filter_auto_init_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->enable);
        
    }
}
void extract_mip_filter_auto_init_control_command(mip_serializer* serializer, mip_filter_auto_init_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->enable);
        
    }
}

void insert_mip_filter_auto_init_control_response(mip_serializer* serializer, const mip_filter_auto_init_control_response* self)
{
    insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_auto_init_control_response(mip_serializer* serializer, mip_filter_auto_init_control_response* self)
{
    extract_u8(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_auto_init_control(struct mip_interface* device, uint8_t enable)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_auto_init_control(struct mip_interface* device, uint8_t* enable_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_AUTOINIT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_auto_init_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_auto_init_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_auto_init_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_accel_noise_command(mip_serializer* serializer, const mip_filter_accel_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_accel_noise_command(mip_serializer* serializer, mip_filter_accel_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_accel_noise_response(mip_serializer* serializer, const mip_filter_accel_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_accel_noise_response(mip_serializer* serializer, mip_filter_accel_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_accel_noise(struct mip_interface* device, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_accel_noise(struct mip_interface* device, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ACCEL_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_accel_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_accel_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_accel_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_gyro_noise_command(mip_serializer* serializer, const mip_filter_gyro_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_gyro_noise_command(mip_serializer* serializer, mip_filter_gyro_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_gyro_noise_response(mip_serializer* serializer, const mip_filter_gyro_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_gyro_noise_response(mip_serializer* serializer, mip_filter_gyro_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_gyro_noise(struct mip_interface* device, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gyro_noise(struct mip_interface* device, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GYRO_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gyro_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gyro_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gyro_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_accel_bias_model_command(mip_serializer* serializer, const mip_filter_accel_bias_model_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_accel_bias_model_command(mip_serializer* serializer, mip_filter_accel_bias_model_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_accel_bias_model_response(mip_serializer* serializer, const mip_filter_accel_bias_model_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_accel_bias_model_response(mip_serializer* serializer, mip_filter_accel_bias_model_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_accel_bias_model(struct mip_interface* device, const float* beta, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(beta || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, beta[i]);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_accel_bias_model(struct mip_interface* device, float* beta_out, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(beta_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &beta_out[i]);
        
        assert(noise_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_accel_bias_model(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_accel_bias_model(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_accel_bias_model(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_gyro_bias_model_command(mip_serializer* serializer, const mip_filter_gyro_bias_model_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_gyro_bias_model_command(mip_serializer* serializer, mip_filter_gyro_bias_model_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_gyro_bias_model_response(mip_serializer* serializer, const mip_filter_gyro_bias_model_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_gyro_bias_model_response(mip_serializer* serializer, mip_filter_gyro_bias_model_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_gyro_bias_model(struct mip_interface* device, const float* beta, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(beta || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, beta[i]);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gyro_bias_model(struct mip_interface* device, float* beta_out, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GYRO_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(beta_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &beta_out[i]);
        
        assert(noise_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gyro_bias_model(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gyro_bias_model(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gyro_bias_model(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_altitude_aiding_command(mip_serializer* serializer, const mip_filter_altitude_aiding_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_altitude_aiding_command_aiding_selector(serializer, self->selector);
        
    }
}
void extract_mip_filter_altitude_aiding_command(mip_serializer* serializer, mip_filter_altitude_aiding_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_altitude_aiding_command_aiding_selector(serializer, &self->selector);
        
    }
}

void insert_mip_filter_altitude_aiding_response(mip_serializer* serializer, const mip_filter_altitude_aiding_response* self)
{
    insert_mip_filter_altitude_aiding_command_aiding_selector(serializer, self->selector);
    
}
void extract_mip_filter_altitude_aiding_response(mip_serializer* serializer, mip_filter_altitude_aiding_response* self)
{
    extract_mip_filter_altitude_aiding_command_aiding_selector(serializer, &self->selector);
    
}

void insert_mip_filter_altitude_aiding_command_aiding_selector(struct mip_serializer* serializer, const mip_filter_altitude_aiding_command_aiding_selector self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_altitude_aiding_command_aiding_selector(struct mip_serializer* serializer, mip_filter_altitude_aiding_command_aiding_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_altitude_aiding(struct mip_interface* device, mip_filter_altitude_aiding_command_aiding_selector selector)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_altitude_aiding_command_aiding_selector(&serializer, selector);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_altitude_aiding(struct mip_interface* device, mip_filter_altitude_aiding_command_aiding_selector* selector_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(selector_out);
        extract_mip_filter_altitude_aiding_command_aiding_selector(&deserializer, selector_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_altitude_aiding(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_altitude_aiding(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_altitude_aiding(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_pitch_roll_aiding_command(mip_serializer* serializer, const mip_filter_pitch_roll_aiding_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, self->source);
        
    }
}
void extract_mip_filter_pitch_roll_aiding_command(mip_serializer* serializer, mip_filter_pitch_roll_aiding_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, &self->source);
        
    }
}

void insert_mip_filter_pitch_roll_aiding_response(mip_serializer* serializer, const mip_filter_pitch_roll_aiding_response* self)
{
    insert_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, self->source);
    
}
void extract_mip_filter_pitch_roll_aiding_response(mip_serializer* serializer, mip_filter_pitch_roll_aiding_response* self)
{
    extract_mip_filter_pitch_roll_aiding_command_aiding_source(serializer, &self->source);
    
}

void insert_mip_filter_pitch_roll_aiding_command_aiding_source(struct mip_serializer* serializer, const mip_filter_pitch_roll_aiding_command_aiding_source self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_pitch_roll_aiding_command_aiding_source(struct mip_serializer* serializer, mip_filter_pitch_roll_aiding_command_aiding_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_pitch_roll_aiding(struct mip_interface* device, mip_filter_pitch_roll_aiding_command_aiding_source source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_pitch_roll_aiding_command_aiding_source(&serializer, source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_pitch_roll_aiding(struct mip_interface* device, mip_filter_pitch_roll_aiding_command_aiding_source* source_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_pitch_roll_aiding_command_aiding_source(&deserializer, source_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_pitch_roll_aiding(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_pitch_roll_aiding(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_pitch_roll_aiding(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_auto_zupt_command(mip_serializer* serializer, const mip_filter_auto_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->enable);
        
        insert_float(serializer, self->threshold);
        
    }
}
void extract_mip_filter_auto_zupt_command(mip_serializer* serializer, mip_filter_auto_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->enable);
        
        extract_float(serializer, &self->threshold);
        
    }
}

void insert_mip_filter_auto_zupt_response(mip_serializer* serializer, const mip_filter_auto_zupt_response* self)
{
    insert_u8(serializer, self->enable);
    
    insert_float(serializer, self->threshold);
    
}
void extract_mip_filter_auto_zupt_response(mip_serializer* serializer, mip_filter_auto_zupt_response* self)
{
    extract_u8(serializer, &self->enable);
    
    extract_float(serializer, &self->threshold);
    
}

mip_cmd_result mip_filter_write_auto_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    insert_float(&serializer, threshold);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_auto_zupt(struct mip_interface* device, uint8_t* enable_out, float* threshold_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        assert(threshold_out);
        extract_float(&deserializer, threshold_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_auto_zupt(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_auto_zupt(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_auto_zupt(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_auto_angular_zupt_command(mip_serializer* serializer, const mip_filter_auto_angular_zupt_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->enable);
        
        insert_float(serializer, self->threshold);
        
    }
}
void extract_mip_filter_auto_angular_zupt_command(mip_serializer* serializer, mip_filter_auto_angular_zupt_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->enable);
        
        extract_float(serializer, &self->threshold);
        
    }
}

void insert_mip_filter_auto_angular_zupt_response(mip_serializer* serializer, const mip_filter_auto_angular_zupt_response* self)
{
    insert_u8(serializer, self->enable);
    
    insert_float(serializer, self->threshold);
    
}
void extract_mip_filter_auto_angular_zupt_response(mip_serializer* serializer, mip_filter_auto_angular_zupt_response* self)
{
    extract_u8(serializer, &self->enable);
    
    extract_float(serializer, &self->threshold);
    
}

mip_cmd_result mip_filter_write_auto_angular_zupt(struct mip_interface* device, uint8_t enable, float threshold)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    insert_float(&serializer, threshold);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_auto_angular_zupt(struct mip_interface* device, uint8_t* enable_out, float* threshold_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        assert(threshold_out);
        extract_float(&deserializer, threshold_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_auto_angular_zupt(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_auto_angular_zupt(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_auto_angular_zupt(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_commanded_zupt(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ZUPT, NULL, 0);
}
mip_cmd_result mip_filter_commanded_angular_zupt(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}
void insert_mip_filter_mag_capture_auto_cal_command(mip_serializer* serializer, const mip_filter_mag_capture_auto_cal_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
}
void extract_mip_filter_mag_capture_auto_cal_command(mip_serializer* serializer, mip_filter_mag_capture_auto_cal_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
}

mip_cmd_result mip_filter_write_mag_capture_auto_cal(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_save_mag_capture_auto_cal(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_gravity_noise_command(mip_serializer* serializer, const mip_filter_gravity_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_gravity_noise_command(mip_serializer* serializer, mip_filter_gravity_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_gravity_noise_response(mip_serializer* serializer, const mip_filter_gravity_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_gravity_noise_response(mip_serializer* serializer, mip_filter_gravity_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_gravity_noise(struct mip_interface* device, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gravity_noise(struct mip_interface* device, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_GRAVITY_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gravity_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gravity_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gravity_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_pressure_altitude_noise_command(mip_serializer* serializer, const mip_filter_pressure_altitude_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_float(serializer, self->noise);
        
    }
}
void extract_mip_filter_pressure_altitude_noise_command(mip_serializer* serializer, mip_filter_pressure_altitude_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_float(serializer, &self->noise);
        
    }
}

void insert_mip_filter_pressure_altitude_noise_response(mip_serializer* serializer, const mip_filter_pressure_altitude_noise_response* self)
{
    insert_float(serializer, self->noise);
    
}
void extract_mip_filter_pressure_altitude_noise_response(mip_serializer* serializer, mip_filter_pressure_altitude_noise_response* self)
{
    extract_float(serializer, &self->noise);
    
}

mip_cmd_result mip_filter_write_pressure_altitude_noise(struct mip_interface* device, float noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_float(&serializer, noise);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_pressure_altitude_noise(struct mip_interface* device, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_PRESSURE_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out);
        extract_float(&deserializer, noise_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_pressure_altitude_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_pressure_altitude_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_pressure_altitude_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_hard_iron_offset_noise_command(mip_serializer* serializer, const mip_filter_hard_iron_offset_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_hard_iron_offset_noise_command(mip_serializer* serializer, mip_filter_hard_iron_offset_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_hard_iron_offset_noise_response(mip_serializer* serializer, const mip_filter_hard_iron_offset_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_hard_iron_offset_noise_response(mip_serializer* serializer, mip_filter_hard_iron_offset_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_hard_iron_offset_noise(struct mip_interface* device, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_hard_iron_offset_noise(struct mip_interface* device, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_hard_iron_offset_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_hard_iron_offset_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_hard_iron_offset_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_soft_iron_matrix_noise_command(mip_serializer* serializer, const mip_filter_soft_iron_matrix_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_soft_iron_matrix_noise_command(mip_serializer* serializer, mip_filter_soft_iron_matrix_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_soft_iron_matrix_noise_response(mip_serializer* serializer, const mip_filter_soft_iron_matrix_noise_response* self)
{
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_soft_iron_matrix_noise_response(mip_serializer* serializer, mip_filter_soft_iron_matrix_noise_response* self)
{
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_soft_iron_matrix_noise(struct mip_interface* device, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise || (9 == 0));
    for(unsigned int i=0; i < 9; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_soft_iron_matrix_noise(struct mip_interface* device, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out || (9 == 0));
        for(unsigned int i=0; i < 9; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_soft_iron_matrix_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_soft_iron_matrix_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_soft_iron_matrix_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_mag_noise_command(mip_serializer* serializer, const mip_filter_mag_noise_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->noise[i]);
        
    }
}
void extract_mip_filter_mag_noise_command(mip_serializer* serializer, mip_filter_mag_noise_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->noise[i]);
        
    }
}

void insert_mip_filter_mag_noise_response(mip_serializer* serializer, const mip_filter_mag_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->noise[i]);
    
}
void extract_mip_filter_mag_noise_response(mip_serializer* serializer, mip_filter_mag_noise_response* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->noise[i]);
    
}

mip_cmd_result mip_filter_write_mag_noise(struct mip_interface* device, const float* noise)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, noise[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_noise(struct mip_interface* device, float* noise_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAG_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(noise_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &noise_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_noise(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_inclination_source_command(mip_serializer* serializer, const mip_filter_inclination_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_mag_param_source(serializer, self->source);
        
        insert_float(serializer, self->inclination);
        
    }
}
void extract_mip_filter_inclination_source_command(mip_serializer* serializer, mip_filter_inclination_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_mag_param_source(serializer, &self->source);
        
        extract_float(serializer, &self->inclination);
        
    }
}

void insert_mip_filter_inclination_source_response(mip_serializer* serializer, const mip_filter_inclination_source_response* self)
{
    insert_mip_filter_mag_param_source(serializer, self->source);
    
    insert_float(serializer, self->inclination);
    
}
void extract_mip_filter_inclination_source_response(mip_serializer* serializer, mip_filter_inclination_source_response* self)
{
    extract_mip_filter_mag_param_source(serializer, &self->source);
    
    extract_float(serializer, &self->inclination);
    
}

mip_cmd_result mip_filter_write_inclination_source(struct mip_interface* device, mip_filter_mag_param_source source, float inclination)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_mag_param_source(&serializer, source);
    
    insert_float(&serializer, inclination);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_inclination_source(struct mip_interface* device, mip_filter_mag_param_source* source_out, float* inclination_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_INCLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_mag_param_source(&deserializer, source_out);
        
        assert(inclination_out);
        extract_float(&deserializer, inclination_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_inclination_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_inclination_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_inclination_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_magnetic_declination_source_command(mip_serializer* serializer, const mip_filter_magnetic_declination_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_mag_param_source(serializer, self->source);
        
        insert_float(serializer, self->declination);
        
    }
}
void extract_mip_filter_magnetic_declination_source_command(mip_serializer* serializer, mip_filter_magnetic_declination_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_mag_param_source(serializer, &self->source);
        
        extract_float(serializer, &self->declination);
        
    }
}

void insert_mip_filter_magnetic_declination_source_response(mip_serializer* serializer, const mip_filter_magnetic_declination_source_response* self)
{
    insert_mip_filter_mag_param_source(serializer, self->source);
    
    insert_float(serializer, self->declination);
    
}
void extract_mip_filter_magnetic_declination_source_response(mip_serializer* serializer, mip_filter_magnetic_declination_source_response* self)
{
    extract_mip_filter_mag_param_source(serializer, &self->source);
    
    extract_float(serializer, &self->declination);
    
}

mip_cmd_result mip_filter_write_magnetic_declination_source(struct mip_interface* device, mip_filter_mag_param_source source, float declination)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_mag_param_source(&serializer, source);
    
    insert_float(&serializer, declination);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_magnetic_declination_source(struct mip_interface* device, mip_filter_mag_param_source* source_out, float* declination_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_mag_param_source(&deserializer, source_out);
        
        assert(declination_out);
        extract_float(&deserializer, declination_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_magnetic_declination_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_magnetic_declination_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_magnetic_declination_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_mag_field_magnitude_source_command(mip_serializer* serializer, const mip_filter_mag_field_magnitude_source_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_mag_param_source(serializer, self->source);
        
        insert_float(serializer, self->magnitude);
        
    }
}
void extract_mip_filter_mag_field_magnitude_source_command(mip_serializer* serializer, mip_filter_mag_field_magnitude_source_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_mag_param_source(serializer, &self->source);
        
        extract_float(serializer, &self->magnitude);
        
    }
}

void insert_mip_filter_mag_field_magnitude_source_response(mip_serializer* serializer, const mip_filter_mag_field_magnitude_source_response* self)
{
    insert_mip_filter_mag_param_source(serializer, self->source);
    
    insert_float(serializer, self->magnitude);
    
}
void extract_mip_filter_mag_field_magnitude_source_response(mip_serializer* serializer, mip_filter_mag_field_magnitude_source_response* self)
{
    extract_mip_filter_mag_param_source(serializer, &self->source);
    
    extract_float(serializer, &self->magnitude);
    
}

mip_cmd_result mip_filter_write_mag_field_magnitude_source(struct mip_interface* device, mip_filter_mag_param_source source, float magnitude)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_mag_param_source(&serializer, source);
    
    insert_float(&serializer, magnitude);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_field_magnitude_source(struct mip_interface* device, mip_filter_mag_param_source* source_out, float* magnitude_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_mip_filter_mag_param_source(&deserializer, source_out);
        
        assert(magnitude_out);
        extract_float(&deserializer, magnitude_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_field_magnitude_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_field_magnitude_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_field_magnitude_source(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_reference_position_command(mip_serializer* serializer, const mip_filter_reference_position_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_bool(serializer, self->enable);
        
        insert_double(serializer, self->latitude);
        
        insert_double(serializer, self->longitude);
        
        insert_double(serializer, self->altitude);
        
    }
}
void extract_mip_filter_reference_position_command(mip_serializer* serializer, mip_filter_reference_position_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_bool(serializer, &self->enable);
        
        extract_double(serializer, &self->latitude);
        
        extract_double(serializer, &self->longitude);
        
        extract_double(serializer, &self->altitude);
        
    }
}

void insert_mip_filter_reference_position_response(mip_serializer* serializer, const mip_filter_reference_position_response* self)
{
    insert_bool(serializer, self->enable);
    
    insert_double(serializer, self->latitude);
    
    insert_double(serializer, self->longitude);
    
    insert_double(serializer, self->altitude);
    
}
void extract_mip_filter_reference_position_response(mip_serializer* serializer, mip_filter_reference_position_response* self)
{
    extract_bool(serializer, &self->enable);
    
    extract_double(serializer, &self->latitude);
    
    extract_double(serializer, &self->longitude);
    
    extract_double(serializer, &self->altitude);
    
}

mip_cmd_result mip_filter_write_reference_position(struct mip_interface* device, bool enable, double latitude, double longitude, double altitude)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_bool(&serializer, enable);
    
    insert_double(&serializer, latitude);
    
    insert_double(&serializer, longitude);
    
    insert_double(&serializer, altitude);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_reference_position(struct mip_interface* device, bool* enable_out, double* latitude_out, double* longitude_out, double* altitude_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_REFERENCE_POSITION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_bool(&deserializer, enable_out);
        
        assert(latitude_out);
        extract_double(&deserializer, latitude_out);
        
        assert(longitude_out);
        extract_double(&deserializer, longitude_out);
        
        assert(altitude_out);
        extract_double(&deserializer, altitude_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_reference_position(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_reference_position(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_reference_position(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_accel_magnitude_error_adaptive_measurement_command(mip_serializer* serializer, const mip_filter_accel_magnitude_error_adaptive_measurement_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
        
        insert_float(serializer, self->frequency);
        
        insert_float(serializer, self->low_limit);
        
        insert_float(serializer, self->high_limit);
        
        insert_float(serializer, self->low_limit_uncertainty);
        
        insert_float(serializer, self->high_limit_uncertainty);
        
        insert_float(serializer, self->minimum_uncertainty);
        
    }
}
void extract_mip_filter_accel_magnitude_error_adaptive_measurement_command(mip_serializer* serializer, mip_filter_accel_magnitude_error_adaptive_measurement_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
        
        extract_float(serializer, &self->frequency);
        
        extract_float(serializer, &self->low_limit);
        
        extract_float(serializer, &self->high_limit);
        
        extract_float(serializer, &self->low_limit_uncertainty);
        
        extract_float(serializer, &self->high_limit_uncertainty);
        
        extract_float(serializer, &self->minimum_uncertainty);
        
    }
}

void insert_mip_filter_accel_magnitude_error_adaptive_measurement_response(mip_serializer* serializer, const mip_filter_accel_magnitude_error_adaptive_measurement_response* self)
{
    insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
    
    insert_float(serializer, self->frequency);
    
    insert_float(serializer, self->low_limit);
    
    insert_float(serializer, self->high_limit);
    
    insert_float(serializer, self->low_limit_uncertainty);
    
    insert_float(serializer, self->high_limit_uncertainty);
    
    insert_float(serializer, self->minimum_uncertainty);
    
}
void extract_mip_filter_accel_magnitude_error_adaptive_measurement_response(mip_serializer* serializer, mip_filter_accel_magnitude_error_adaptive_measurement_response* self)
{
    extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
    
    extract_float(serializer, &self->frequency);
    
    extract_float(serializer, &self->low_limit);
    
    extract_float(serializer, &self->high_limit);
    
    extract_float(serializer, &self->low_limit_uncertainty);
    
    extract_float(serializer, &self->high_limit_uncertainty);
    
    extract_float(serializer, &self->minimum_uncertainty);
    
}

mip_cmd_result mip_filter_write_accel_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement adaptive_measurement, float frequency, float low_limit, float high_limit, float low_limit_uncertainty, float high_limit_uncertainty, float minimum_uncertainty)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_adaptive_measurement(&serializer, adaptive_measurement);
    
    insert_float(&serializer, frequency);
    
    insert_float(&serializer, low_limit);
    
    insert_float(&serializer, high_limit);
    
    insert_float(&serializer, low_limit_uncertainty);
    
    insert_float(&serializer, high_limit_uncertainty);
    
    insert_float(&serializer, minimum_uncertainty);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_accel_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement* adaptive_measurement_out, float* frequency_out, float* low_limit_out, float* high_limit_out, float* low_limit_uncertainty_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(adaptive_measurement_out);
        extract_mip_filter_adaptive_measurement(&deserializer, adaptive_measurement_out);
        
        assert(frequency_out);
        extract_float(&deserializer, frequency_out);
        
        assert(low_limit_out);
        extract_float(&deserializer, low_limit_out);
        
        assert(high_limit_out);
        extract_float(&deserializer, high_limit_out);
        
        assert(low_limit_uncertainty_out);
        extract_float(&deserializer, low_limit_uncertainty_out);
        
        assert(high_limit_uncertainty_out);
        extract_float(&deserializer, high_limit_uncertainty_out);
        
        assert(minimum_uncertainty_out);
        extract_float(&deserializer, minimum_uncertainty_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_accel_magnitude_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_accel_magnitude_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_accel_magnitude_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_mag_magnitude_error_adaptive_measurement_command(mip_serializer* serializer, const mip_filter_mag_magnitude_error_adaptive_measurement_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
        
        insert_float(serializer, self->frequency);
        
        insert_float(serializer, self->low_limit);
        
        insert_float(serializer, self->high_limit);
        
        insert_float(serializer, self->low_limit_uncertainty);
        
        insert_float(serializer, self->high_limit_uncertainty);
        
        insert_float(serializer, self->minimum_uncertainty);
        
    }
}
void extract_mip_filter_mag_magnitude_error_adaptive_measurement_command(mip_serializer* serializer, mip_filter_mag_magnitude_error_adaptive_measurement_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
        
        extract_float(serializer, &self->frequency);
        
        extract_float(serializer, &self->low_limit);
        
        extract_float(serializer, &self->high_limit);
        
        extract_float(serializer, &self->low_limit_uncertainty);
        
        extract_float(serializer, &self->high_limit_uncertainty);
        
        extract_float(serializer, &self->minimum_uncertainty);
        
    }
}

void insert_mip_filter_mag_magnitude_error_adaptive_measurement_response(mip_serializer* serializer, const mip_filter_mag_magnitude_error_adaptive_measurement_response* self)
{
    insert_mip_filter_adaptive_measurement(serializer, self->adaptive_measurement);
    
    insert_float(serializer, self->frequency);
    
    insert_float(serializer, self->low_limit);
    
    insert_float(serializer, self->high_limit);
    
    insert_float(serializer, self->low_limit_uncertainty);
    
    insert_float(serializer, self->high_limit_uncertainty);
    
    insert_float(serializer, self->minimum_uncertainty);
    
}
void extract_mip_filter_mag_magnitude_error_adaptive_measurement_response(mip_serializer* serializer, mip_filter_mag_magnitude_error_adaptive_measurement_response* self)
{
    extract_mip_filter_adaptive_measurement(serializer, &self->adaptive_measurement);
    
    extract_float(serializer, &self->frequency);
    
    extract_float(serializer, &self->low_limit);
    
    extract_float(serializer, &self->high_limit);
    
    extract_float(serializer, &self->low_limit_uncertainty);
    
    extract_float(serializer, &self->high_limit_uncertainty);
    
    extract_float(serializer, &self->minimum_uncertainty);
    
}

mip_cmd_result mip_filter_write_mag_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement adaptive_measurement, float frequency, float low_limit, float high_limit, float low_limit_uncertainty, float high_limit_uncertainty, float minimum_uncertainty)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_adaptive_measurement(&serializer, adaptive_measurement);
    
    insert_float(&serializer, frequency);
    
    insert_float(&serializer, low_limit);
    
    insert_float(&serializer, high_limit);
    
    insert_float(&serializer, low_limit_uncertainty);
    
    insert_float(&serializer, high_limit_uncertainty);
    
    insert_float(&serializer, minimum_uncertainty);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_magnitude_error_adaptive_measurement(struct mip_interface* device, mip_filter_adaptive_measurement* adaptive_measurement_out, float* frequency_out, float* low_limit_out, float* high_limit_out, float* low_limit_uncertainty_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(adaptive_measurement_out);
        extract_mip_filter_adaptive_measurement(&deserializer, adaptive_measurement_out);
        
        assert(frequency_out);
        extract_float(&deserializer, frequency_out);
        
        assert(low_limit_out);
        extract_float(&deserializer, low_limit_out);
        
        assert(high_limit_out);
        extract_float(&deserializer, high_limit_out);
        
        assert(low_limit_uncertainty_out);
        extract_float(&deserializer, low_limit_uncertainty_out);
        
        assert(high_limit_uncertainty_out);
        extract_float(&deserializer, high_limit_uncertainty_out);
        
        assert(minimum_uncertainty_out);
        extract_float(&deserializer, minimum_uncertainty_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_magnitude_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_magnitude_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_magnitude_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_mag_dip_angle_error_adaptive_measurement_command(mip_serializer* serializer, const mip_filter_mag_dip_angle_error_adaptive_measurement_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_bool(serializer, self->enable);
        
        insert_float(serializer, self->frequency);
        
        insert_float(serializer, self->high_limit);
        
        insert_float(serializer, self->high_limit_uncertainty);
        
        insert_float(serializer, self->minimum_uncertainty);
        
    }
}
void extract_mip_filter_mag_dip_angle_error_adaptive_measurement_command(mip_serializer* serializer, mip_filter_mag_dip_angle_error_adaptive_measurement_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_bool(serializer, &self->enable);
        
        extract_float(serializer, &self->frequency);
        
        extract_float(serializer, &self->high_limit);
        
        extract_float(serializer, &self->high_limit_uncertainty);
        
        extract_float(serializer, &self->minimum_uncertainty);
        
    }
}

void insert_mip_filter_mag_dip_angle_error_adaptive_measurement_response(mip_serializer* serializer, const mip_filter_mag_dip_angle_error_adaptive_measurement_response* self)
{
    insert_bool(serializer, self->enable);
    
    insert_float(serializer, self->frequency);
    
    insert_float(serializer, self->high_limit);
    
    insert_float(serializer, self->high_limit_uncertainty);
    
    insert_float(serializer, self->minimum_uncertainty);
    
}
void extract_mip_filter_mag_dip_angle_error_adaptive_measurement_response(mip_serializer* serializer, mip_filter_mag_dip_angle_error_adaptive_measurement_response* self)
{
    extract_bool(serializer, &self->enable);
    
    extract_float(serializer, &self->frequency);
    
    extract_float(serializer, &self->high_limit);
    
    extract_float(serializer, &self->high_limit_uncertainty);
    
    extract_float(serializer, &self->minimum_uncertainty);
    
}

mip_cmd_result mip_filter_write_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device, bool enable, float frequency, float high_limit, float high_limit_uncertainty, float minimum_uncertainty)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_bool(&serializer, enable);
    
    insert_float(&serializer, frequency);
    
    insert_float(&serializer, high_limit);
    
    insert_float(&serializer, high_limit_uncertainty);
    
    insert_float(&serializer, minimum_uncertainty);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device, bool* enable_out, float* frequency_out, float* high_limit_out, float* high_limit_uncertainty_out, float* minimum_uncertainty_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_bool(&deserializer, enable_out);
        
        assert(frequency_out);
        extract_float(&deserializer, frequency_out);
        
        assert(high_limit_out);
        extract_float(&deserializer, high_limit_out);
        
        assert(high_limit_uncertainty_out);
        extract_float(&deserializer, high_limit_uncertainty_out);
        
        assert(minimum_uncertainty_out);
        extract_float(&deserializer, minimum_uncertainty_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_mag_dip_angle_error_adaptive_measurement(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_aiding_measurement_enable_command(mip_serializer* serializer, const mip_filter_aiding_measurement_enable_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, self->aiding_source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_bool(serializer, self->enable);
        
    }
}
void extract_mip_filter_aiding_measurement_enable_command(mip_serializer* serializer, mip_filter_aiding_measurement_enable_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    extract_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, &self->aiding_source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_bool(serializer, &self->enable);
        
    }
}

void insert_mip_filter_aiding_measurement_enable_response(mip_serializer* serializer, const mip_filter_aiding_measurement_enable_response* self)
{
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, self->aiding_source);
    
    insert_bool(serializer, self->enable);
    
}
void extract_mip_filter_aiding_measurement_enable_response(mip_serializer* serializer, mip_filter_aiding_measurement_enable_response* self)
{
    extract_mip_filter_aiding_measurement_enable_command_aiding_source(serializer, &self->aiding_source);
    
    extract_bool(serializer, &self->enable);
    
}

void insert_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, const mip_filter_aiding_measurement_enable_command_aiding_source self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_filter_aiding_measurement_enable_command_aiding_source(struct mip_serializer* serializer, mip_filter_aiding_measurement_enable_command_aiding_source* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool enable)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    insert_bool(&serializer, enable);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source, bool* enable_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_mip_filter_aiding_measurement_enable_command_aiding_source(&deserializer, &aiding_source);
        
        assert(enable_out);
        extract_bool(&deserializer, enable_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_aiding_measurement_enable(struct mip_interface* device, mip_filter_aiding_measurement_enable_command_aiding_source aiding_source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_mip_filter_aiding_measurement_enable_command_aiding_source(&serializer, aiding_source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_run(struct mip_interface* device)
{
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_RUN, NULL, 0);
}
void insert_mip_filter_kinematic_constraint_command(mip_serializer* serializer, const mip_filter_kinematic_constraint_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->acceleration_constraint_selection);
        
        insert_u8(serializer, self->velocity_constraint_selection);
        
        insert_u8(serializer, self->angular_constraint_selection);
        
    }
}
void extract_mip_filter_kinematic_constraint_command(mip_serializer* serializer, mip_filter_kinematic_constraint_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->acceleration_constraint_selection);
        
        extract_u8(serializer, &self->velocity_constraint_selection);
        
        extract_u8(serializer, &self->angular_constraint_selection);
        
    }
}

void insert_mip_filter_kinematic_constraint_response(mip_serializer* serializer, const mip_filter_kinematic_constraint_response* self)
{
    insert_u8(serializer, self->acceleration_constraint_selection);
    
    insert_u8(serializer, self->velocity_constraint_selection);
    
    insert_u8(serializer, self->angular_constraint_selection);
    
}
void extract_mip_filter_kinematic_constraint_response(mip_serializer* serializer, mip_filter_kinematic_constraint_response* self)
{
    extract_u8(serializer, &self->acceleration_constraint_selection);
    
    extract_u8(serializer, &self->velocity_constraint_selection);
    
    extract_u8(serializer, &self->angular_constraint_selection);
    
}

mip_cmd_result mip_filter_write_kinematic_constraint(struct mip_interface* device, uint8_t acceleration_constraint_selection, uint8_t velocity_constraint_selection, uint8_t angular_constraint_selection)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, acceleration_constraint_selection);
    
    insert_u8(&serializer, velocity_constraint_selection);
    
    insert_u8(&serializer, angular_constraint_selection);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_kinematic_constraint(struct mip_interface* device, uint8_t* acceleration_constraint_selection_out, uint8_t* velocity_constraint_selection_out, uint8_t* angular_constraint_selection_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(acceleration_constraint_selection_out);
        extract_u8(&deserializer, acceleration_constraint_selection_out);
        
        assert(velocity_constraint_selection_out);
        extract_u8(&deserializer, velocity_constraint_selection_out);
        
        assert(angular_constraint_selection_out);
        extract_u8(&deserializer, angular_constraint_selection_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_kinematic_constraint(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_kinematic_constraint(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_kinematic_constraint(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_initialization_configuration_command(mip_serializer* serializer, const mip_filter_initialization_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->wait_for_run_command);
        
        insert_mip_filter_initialization_configuration_command_initial_condition_source(serializer, self->initial_cond_src);
        
        insert_mip_filter_initialization_configuration_command_alignment_selector(serializer, self->auto_heading_alignment_selector);
        
        insert_float(serializer, self->initial_heading);
        
        insert_float(serializer, self->initial_pitch);
        
        insert_float(serializer, self->initial_roll);
        
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->initial_position[i]);
        
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->initial_velocity[i]);
        
        insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
        
    }
}
void extract_mip_filter_initialization_configuration_command(mip_serializer* serializer, mip_filter_initialization_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->wait_for_run_command);
        
        extract_mip_filter_initialization_configuration_command_initial_condition_source(serializer, &self->initial_cond_src);
        
        extract_mip_filter_initialization_configuration_command_alignment_selector(serializer, &self->auto_heading_alignment_selector);
        
        extract_float(serializer, &self->initial_heading);
        
        extract_float(serializer, &self->initial_pitch);
        
        extract_float(serializer, &self->initial_roll);
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->initial_position[i]);
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->initial_velocity[i]);
        
        extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
        
    }
}

void insert_mip_filter_initialization_configuration_response(mip_serializer* serializer, const mip_filter_initialization_configuration_response* self)
{
    insert_u8(serializer, self->wait_for_run_command);
    
    insert_mip_filter_initialization_configuration_command_initial_condition_source(serializer, self->initial_cond_src);
    
    insert_mip_filter_initialization_configuration_command_alignment_selector(serializer, self->auto_heading_alignment_selector);
    
    insert_float(serializer, self->initial_heading);
    
    insert_float(serializer, self->initial_pitch);
    
    insert_float(serializer, self->initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->initial_velocity[i]);
    
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    
}
void extract_mip_filter_initialization_configuration_response(mip_serializer* serializer, mip_filter_initialization_configuration_response* self)
{
    extract_u8(serializer, &self->wait_for_run_command);
    
    extract_mip_filter_initialization_configuration_command_initial_condition_source(serializer, &self->initial_cond_src);
    
    extract_mip_filter_initialization_configuration_command_alignment_selector(serializer, &self->auto_heading_alignment_selector);
    
    extract_float(serializer, &self->initial_heading);
    
    extract_float(serializer, &self->initial_pitch);
    
    extract_float(serializer, &self->initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->initial_velocity[i]);
    
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    
}

void insert_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, const mip_filter_initialization_configuration_command_alignment_selector self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_initialization_configuration_command_alignment_selector(struct mip_serializer* serializer, mip_filter_initialization_configuration_command_alignment_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, const mip_filter_initialization_configuration_command_initial_condition_source self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_initialization_configuration_command_initial_condition_source(struct mip_serializer* serializer, mip_filter_initialization_configuration_command_initial_condition_source* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_initialization_configuration(struct mip_interface* device, uint8_t wait_for_run_command, mip_filter_initialization_configuration_command_initial_condition_source initial_cond_src, mip_filter_initialization_configuration_command_alignment_selector auto_heading_alignment_selector, float initial_heading, float initial_pitch, float initial_roll, const float* initial_position, const float* initial_velocity, mip_filter_reference_frame reference_frame_selector)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, wait_for_run_command);
    
    insert_mip_filter_initialization_configuration_command_initial_condition_source(&serializer, initial_cond_src);
    
    insert_mip_filter_initialization_configuration_command_alignment_selector(&serializer, auto_heading_alignment_selector);
    
    insert_float(&serializer, initial_heading);
    
    insert_float(&serializer, initial_pitch);
    
    insert_float(&serializer, initial_roll);
    
    assert(initial_position || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, initial_position[i]);
    
    assert(initial_velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, initial_velocity[i]);
    
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_initialization_configuration(struct mip_interface* device, uint8_t* wait_for_run_command_out, mip_filter_initialization_configuration_command_initial_condition_source* initial_cond_src_out, mip_filter_initialization_configuration_command_alignment_selector* auto_heading_alignment_selector_out, float* initial_heading_out, float* initial_pitch_out, float* initial_roll_out, float* initial_position_out, float* initial_velocity_out, mip_filter_reference_frame* reference_frame_selector_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(wait_for_run_command_out);
        extract_u8(&deserializer, wait_for_run_command_out);
        
        assert(initial_cond_src_out);
        extract_mip_filter_initialization_configuration_command_initial_condition_source(&deserializer, initial_cond_src_out);
        
        assert(auto_heading_alignment_selector_out);
        extract_mip_filter_initialization_configuration_command_alignment_selector(&deserializer, auto_heading_alignment_selector_out);
        
        assert(initial_heading_out);
        extract_float(&deserializer, initial_heading_out);
        
        assert(initial_pitch_out);
        extract_float(&deserializer, initial_pitch_out);
        
        assert(initial_roll_out);
        extract_float(&deserializer, initial_roll_out);
        
        assert(initial_position_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &initial_position_out[i]);
        
        assert(initial_velocity_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &initial_velocity_out[i]);
        
        assert(reference_frame_selector_out);
        extract_mip_filter_reference_frame(&deserializer, reference_frame_selector_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_initialization_configuration(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_initialization_configuration(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_initialization_configuration(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_adaptive_filter_options_command(mip_serializer* serializer, const mip_filter_adaptive_filter_options_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->level);
        
        insert_u16(serializer, self->time_limit);
        
    }
}
void extract_mip_filter_adaptive_filter_options_command(mip_serializer* serializer, mip_filter_adaptive_filter_options_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->level);
        
        extract_u16(serializer, &self->time_limit);
        
    }
}

void insert_mip_filter_adaptive_filter_options_response(mip_serializer* serializer, const mip_filter_adaptive_filter_options_response* self)
{
    insert_u8(serializer, self->level);
    
    insert_u16(serializer, self->time_limit);
    
}
void extract_mip_filter_adaptive_filter_options_response(mip_serializer* serializer, mip_filter_adaptive_filter_options_response* self)
{
    extract_u8(serializer, &self->level);
    
    extract_u16(serializer, &self->time_limit);
    
}

mip_cmd_result mip_filter_write_adaptive_filter_options(struct mip_interface* device, uint8_t level, uint16_t time_limit)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, level);
    
    insert_u16(&serializer, time_limit);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_adaptive_filter_options(struct mip_interface* device, uint8_t* level_out, uint16_t* time_limit_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(level_out);
        extract_u8(&deserializer, level_out);
        
        assert(time_limit_out);
        extract_u16(&deserializer, time_limit_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_adaptive_filter_options(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_adaptive_filter_options(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_adaptive_filter_options(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_multi_antenna_offset_command(mip_serializer* serializer, const mip_filter_multi_antenna_offset_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    insert_u8(serializer, self->receiver_id);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->antenna_offset[i]);
        
    }
}
void extract_mip_filter_multi_antenna_offset_command(mip_serializer* serializer, mip_filter_multi_antenna_offset_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    extract_u8(serializer, &self->receiver_id);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->antenna_offset[i]);
        
    }
}

void insert_mip_filter_multi_antenna_offset_response(mip_serializer* serializer, const mip_filter_multi_antenna_offset_response* self)
{
    insert_u8(serializer, self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->antenna_offset[i]);
    
}
void extract_mip_filter_multi_antenna_offset_response(mip_serializer* serializer, mip_filter_multi_antenna_offset_response* self)
{
    extract_u8(serializer, &self->receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->antenna_offset[i]);
    
}

mip_cmd_result mip_filter_write_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, const float* antenna_offset)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, receiver_id);
    
    assert(antenna_offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, antenna_offset[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id, float* antenna_offset_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_u8(&deserializer, &receiver_id);
        
        assert(antenna_offset_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &antenna_offset_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_multi_antenna_offset(struct mip_interface* device, uint8_t receiver_id)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_u8(&serializer, receiver_id);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_rel_pos_configuration_command(mip_serializer* serializer, const mip_filter_rel_pos_configuration_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->source);
        
        insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
        
        for(unsigned int i=0; i < 3; i++)
            insert_double(serializer, self->reference_coordinates[i]);
        
    }
}
void extract_mip_filter_rel_pos_configuration_command(mip_serializer* serializer, mip_filter_rel_pos_configuration_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->source);
        
        extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
        
        for(unsigned int i=0; i < 3; i++)
            extract_double(serializer, &self->reference_coordinates[i]);
        
    }
}

void insert_mip_filter_rel_pos_configuration_response(mip_serializer* serializer, const mip_filter_rel_pos_configuration_response* self)
{
    insert_u8(serializer, self->source);
    
    insert_mip_filter_reference_frame(serializer, self->reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->reference_coordinates[i]);
    
}
void extract_mip_filter_rel_pos_configuration_response(mip_serializer* serializer, mip_filter_rel_pos_configuration_response* self)
{
    extract_u8(serializer, &self->source);
    
    extract_mip_filter_reference_frame(serializer, &self->reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->reference_coordinates[i]);
    
}

mip_cmd_result mip_filter_write_rel_pos_configuration(struct mip_interface* device, uint8_t source, mip_filter_reference_frame reference_frame_selector, const double* reference_coordinates)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, source);
    
    insert_mip_filter_reference_frame(&serializer, reference_frame_selector);
    
    assert(reference_coordinates || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_double(&serializer, reference_coordinates[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_rel_pos_configuration(struct mip_interface* device, uint8_t* source_out, mip_filter_reference_frame* reference_frame_selector_out, double* reference_coordinates_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(source_out);
        extract_u8(&deserializer, source_out);
        
        assert(reference_frame_selector_out);
        extract_mip_filter_reference_frame(&deserializer, reference_frame_selector_out);
        
        assert(reference_coordinates_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_double(&deserializer, &reference_coordinates_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_rel_pos_configuration(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_rel_pos_configuration(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_rel_pos_configuration(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_ref_point_lever_arm_command(mip_serializer* serializer, const mip_filter_ref_point_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, self->ref_point_sel);
        
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->lever_arm_offset[i]);
        
    }
}
void extract_mip_filter_ref_point_lever_arm_command(mip_serializer* serializer, mip_filter_ref_point_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, &self->ref_point_sel);
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->lever_arm_offset[i]);
        
    }
}

void insert_mip_filter_ref_point_lever_arm_response(mip_serializer* serializer, const mip_filter_ref_point_lever_arm_response* self)
{
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, self->ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
    
}
void extract_mip_filter_ref_point_lever_arm_response(mip_serializer* serializer, mip_filter_ref_point_lever_arm_response* self)
{
    extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(serializer, &self->ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
    
}

void insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, const mip_filter_ref_point_lever_arm_command_reference_point_selector self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(struct mip_serializer* serializer, mip_filter_ref_point_lever_arm_command_reference_point_selector* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_filter_write_ref_point_lever_arm(struct mip_interface* device, mip_filter_ref_point_lever_arm_command_reference_point_selector ref_point_sel, const float* lever_arm_offset)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_filter_ref_point_lever_arm_command_reference_point_selector(&serializer, ref_point_sel);
    
    assert(lever_arm_offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, lever_arm_offset[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_ref_point_lever_arm(struct mip_interface* device, mip_filter_ref_point_lever_arm_command_reference_point_selector* ref_point_sel_out, float* lever_arm_offset_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(ref_point_sel_out);
        extract_mip_filter_ref_point_lever_arm_command_reference_point_selector(&deserializer, ref_point_sel_out);
        
        assert(lever_arm_offset_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &lever_arm_offset_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_ref_point_lever_arm(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_ref_point_lever_arm(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_ref_point_lever_arm(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_speed_measurement_command(mip_serializer* serializer, const mip_filter_speed_measurement_command* self)
{
    insert_u8(serializer, self->source);
    
    insert_float(serializer, self->time_of_week);
    
    insert_float(serializer, self->speed);
    
    insert_float(serializer, self->speed_uncertainty);
    
}
void extract_mip_filter_speed_measurement_command(mip_serializer* serializer, mip_filter_speed_measurement_command* self)
{
    extract_u8(serializer, &self->source);
    
    extract_float(serializer, &self->time_of_week);
    
    extract_float(serializer, &self->speed);
    
    extract_float(serializer, &self->speed_uncertainty);
    
}

mip_cmd_result mip_filter_speed_measurement(struct mip_interface* device, uint8_t source, float time_of_week, float speed, float speed_uncertainty)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_u8(&serializer, source);
    
    insert_float(&serializer, time_of_week);
    
    insert_float(&serializer, speed);
    
    insert_float(&serializer, speed_uncertainty);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_MEASUREMENT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_speed_lever_arm_command(mip_serializer* serializer, const mip_filter_speed_lever_arm_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    insert_u8(serializer, self->source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->lever_arm_offset[i]);
        
    }
}
void extract_mip_filter_speed_lever_arm_command(mip_serializer* serializer, mip_filter_speed_lever_arm_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    extract_u8(serializer, &self->source);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->lever_arm_offset[i]);
        
    }
}

void insert_mip_filter_speed_lever_arm_response(mip_serializer* serializer, const mip_filter_speed_lever_arm_response* self)
{
    insert_u8(serializer, self->source);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->lever_arm_offset[i]);
    
}
void extract_mip_filter_speed_lever_arm_response(mip_serializer* serializer, mip_filter_speed_lever_arm_response* self)
{
    extract_u8(serializer, &self->source);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->lever_arm_offset[i]);
    
}

mip_cmd_result mip_filter_write_speed_lever_arm(struct mip_interface* device, uint8_t source, const float* lever_arm_offset)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, source);
    
    assert(lever_arm_offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, lever_arm_offset[i]);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_speed_lever_arm(struct mip_interface* device, uint8_t source, float* lever_arm_offset_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_FILTER_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_u8(&deserializer, &source);
        
        assert(lever_arm_offset_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &lever_arm_offset_out[i]);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_speed_lever_arm(struct mip_interface* device, uint8_t source)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_u8(&serializer, source);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_wheeled_vehicle_constraint_control_command(mip_serializer* serializer, const mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->enable);
        
    }
}
void extract_mip_filter_wheeled_vehicle_constraint_control_command(mip_serializer* serializer, mip_filter_wheeled_vehicle_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->enable);
        
    }
}

void insert_mip_filter_wheeled_vehicle_constraint_control_response(mip_serializer* serializer, const mip_filter_wheeled_vehicle_constraint_control_response* self)
{
    insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_wheeled_vehicle_constraint_control_response(mip_serializer* serializer, mip_filter_wheeled_vehicle_constraint_control_response* self)
{
    extract_u8(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t enable)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_wheeled_vehicle_constraint_control(struct mip_interface* device, uint8_t* enable_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_wheeled_vehicle_constraint_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_WHEELED_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_vertical_gyro_constraint_control_command(mip_serializer* serializer, const mip_filter_vertical_gyro_constraint_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->enable);
        
    }
}
void extract_mip_filter_vertical_gyro_constraint_control_command(mip_serializer* serializer, mip_filter_vertical_gyro_constraint_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->enable);
        
    }
}

void insert_mip_filter_vertical_gyro_constraint_control_response(mip_serializer* serializer, const mip_filter_vertical_gyro_constraint_control_response* self)
{
    insert_u8(serializer, self->enable);
    
}
void extract_mip_filter_vertical_gyro_constraint_control_response(mip_serializer* serializer, mip_filter_vertical_gyro_constraint_control_response* self)
{
    extract_u8(serializer, &self->enable);
    
}

mip_cmd_result mip_filter_write_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t enable)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_vertical_gyro_constraint_control(struct mip_interface* device, uint8_t* enable_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_vertical_gyro_constraint_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_vertical_gyro_constraint_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_vertical_gyro_constraint_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_VERTICAL_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_gnss_antenna_cal_control_command(mip_serializer* serializer, const mip_filter_gnss_antenna_cal_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_u8(serializer, self->enable);
        
        insert_float(serializer, self->max_offset);
        
    }
}
void extract_mip_filter_gnss_antenna_cal_control_command(mip_serializer* serializer, mip_filter_gnss_antenna_cal_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_u8(serializer, &self->enable);
        
        extract_float(serializer, &self->max_offset);
        
    }
}

void insert_mip_filter_gnss_antenna_cal_control_response(mip_serializer* serializer, const mip_filter_gnss_antenna_cal_control_response* self)
{
    insert_u8(serializer, self->enable);
    
    insert_float(serializer, self->max_offset);
    
}
void extract_mip_filter_gnss_antenna_cal_control_response(mip_serializer* serializer, mip_filter_gnss_antenna_cal_control_response* self)
{
    extract_u8(serializer, &self->enable);
    
    extract_float(serializer, &self->max_offset);
    
}

mip_cmd_result mip_filter_write_gnss_antenna_cal_control(struct mip_interface* device, uint8_t enable, float max_offset)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, enable);
    
    insert_float(&serializer, max_offset);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_read_gnss_antenna_cal_control(struct mip_interface* device, uint8_t* enable_out, float* max_offset_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(enable_out);
        extract_u8(&deserializer, enable_out);
        
        assert(max_offset_out);
        extract_float(&deserializer, max_offset_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_filter_save_gnss_antenna_cal_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_load_gnss_antenna_cal_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_filter_default_gnss_antenna_cal_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_GNSS_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_filter_set_initial_heading_command(mip_serializer* serializer, const mip_filter_set_initial_heading_command* self)
{
    insert_float(serializer, self->heading);
    
}
void extract_mip_filter_set_initial_heading_command(mip_serializer* serializer, mip_filter_set_initial_heading_command* self)
{
    extract_float(serializer, &self->heading);
    
}

mip_cmd_result mip_filter_set_initial_heading(struct mip_interface* device, float heading)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_float(&serializer, heading);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_FILTER_CMD_DESC_SET, MIP_CMD_DESC_FILTER_SET_INITIAL_HEADING, buffer, (uint8_t)mip_serializer_length(&serializer));
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

