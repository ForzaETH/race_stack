
#include "commands_aiding.h"

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

void insert_mip_time(mip_serializer* serializer, const mip_time* self)
{
    insert_mip_time_timebase(serializer, self->timebase);
    
    insert_u8(serializer, self->reserved);
    
    insert_u64(serializer, self->nanoseconds);
    
}
void extract_mip_time(mip_serializer* serializer, mip_time* self)
{
    extract_mip_time_timebase(serializer, &self->timebase);
    
    extract_u8(serializer, &self->reserved);
    
    extract_u64(serializer, &self->nanoseconds);
    
}

void insert_mip_time_timebase(struct mip_serializer* serializer, const mip_time_timebase self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_time_timebase(struct mip_serializer* serializer, mip_time_timebase* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_aiding_frame_config_command(mip_serializer* serializer, const mip_aiding_frame_config_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    insert_u8(serializer, self->frame_id);
    
    if( self->function == MIP_FUNCTION_WRITE || self->function == MIP_FUNCTION_READ )
    {
        insert_mip_aiding_frame_config_command_format(serializer, self->format);
        
    }
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_bool(serializer, self->tracking_enabled);
        
        for(unsigned int i=0; i < 3; i++)
            insert_float(serializer, self->translation[i]);
        
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
        {
            insert_mip_vector3f(serializer, self->rotation.euler);
            
        }
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
        {
            insert_mip_quatf(serializer, self->rotation.quaternion);
            
        }
    }
}
void extract_mip_aiding_frame_config_command(mip_serializer* serializer, mip_aiding_frame_config_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    extract_u8(serializer, &self->frame_id);
    
    if( self->function == MIP_FUNCTION_WRITE || self->function == MIP_FUNCTION_READ )
    {
        extract_mip_aiding_frame_config_command_format(serializer, &self->format);
        
    }
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_bool(serializer, &self->tracking_enabled);
        
        for(unsigned int i=0; i < 3; i++)
            extract_float(serializer, &self->translation[i]);
        
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
        {
            extract_mip_vector3f(serializer, self->rotation.euler);
            
        }
        if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
        {
            extract_mip_quatf(serializer, self->rotation.quaternion);
            
        }
    }
}

void insert_mip_aiding_frame_config_response(mip_serializer* serializer, const mip_aiding_frame_config_response* self)
{
    insert_u8(serializer, self->frame_id);
    
    insert_mip_aiding_frame_config_command_format(serializer, self->format);
    
    insert_bool(serializer, self->tracking_enabled);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->translation[i]);
    
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
    {
        insert_mip_vector3f(serializer, self->rotation.euler);
        
    }
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
    {
        insert_mip_quatf(serializer, self->rotation.quaternion);
        
    }
}
void extract_mip_aiding_frame_config_response(mip_serializer* serializer, mip_aiding_frame_config_response* self)
{
    extract_u8(serializer, &self->frame_id);
    
    extract_mip_aiding_frame_config_command_format(serializer, &self->format);
    
    extract_bool(serializer, &self->tracking_enabled);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->translation[i]);
    
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
    {
        extract_mip_vector3f(serializer, self->rotation.euler);
        
    }
    if( self->format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
    {
        extract_mip_quatf(serializer, self->rotation.quaternion);
        
    }
}

void insert_mip_aiding_frame_config_command_format(struct mip_serializer* serializer, const mip_aiding_frame_config_command_format self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_aiding_frame_config_command_format(struct mip_serializer* serializer, mip_aiding_frame_config_command_format* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_write_frame_config(struct mip_interface* device, uint8_t frame_id, mip_aiding_frame_config_command_format format, bool tracking_enabled, const float* translation, const mip_aiding_frame_config_command_rotation* rotation)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_u8(&serializer, frame_id);
    
    insert_mip_aiding_frame_config_command_format(&serializer, format);
    
    insert_bool(&serializer, tracking_enabled);
    
    assert(translation || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, translation[i]);
    
    if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
    {
        insert_mip_vector3f(&serializer, rotation->euler);
        
    }
    if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
    {
        insert_mip_quatf(&serializer, rotation->quaternion);
        
    }
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_read_frame_config(struct mip_interface* device, uint8_t frame_id, mip_aiding_frame_config_command_format format, bool* tracking_enabled_out, float* translation_out, mip_aiding_frame_config_command_rotation* rotation_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    insert_u8(&serializer, frame_id);
    
    insert_mip_aiding_frame_config_command_format(&serializer, format);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_FRAME_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        extract_u8(&deserializer, &frame_id);
        
        extract_mip_aiding_frame_config_command_format(&deserializer, &format);
        
        assert(tracking_enabled_out);
        extract_bool(&deserializer, tracking_enabled_out);
        
        assert(translation_out || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract_float(&deserializer, &translation_out[i]);
        
        if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER )
        {
            extract_mip_vector3f(&deserializer, rotation_out->euler);
            
        }
        if( format == MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_QUATERNION )
        {
            extract_mip_quatf(&deserializer, rotation_out->quaternion);
            
        }
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_aiding_save_frame_config(struct mip_interface* device, uint8_t frame_id)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    insert_u8(&serializer, frame_id);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_load_frame_config(struct mip_interface* device, uint8_t frame_id)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    insert_u8(&serializer, frame_id);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_default_frame_config(struct mip_interface* device, uint8_t frame_id)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    insert_u8(&serializer, frame_id);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_aiding_echo_control_command(mip_serializer* serializer, const mip_aiding_aiding_echo_control_command* self)
{
    insert_mip_function_selector(serializer, self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        insert_mip_aiding_aiding_echo_control_command_mode(serializer, self->mode);
        
    }
}
void extract_mip_aiding_aiding_echo_control_command(mip_serializer* serializer, mip_aiding_aiding_echo_control_command* self)
{
    extract_mip_function_selector(serializer, &self->function);
    
    if( self->function == MIP_FUNCTION_WRITE )
    {
        extract_mip_aiding_aiding_echo_control_command_mode(serializer, &self->mode);
        
    }
}

void insert_mip_aiding_aiding_echo_control_response(mip_serializer* serializer, const mip_aiding_aiding_echo_control_response* self)
{
    insert_mip_aiding_aiding_echo_control_command_mode(serializer, self->mode);
    
}
void extract_mip_aiding_aiding_echo_control_response(mip_serializer* serializer, mip_aiding_aiding_echo_control_response* self)
{
    extract_mip_aiding_aiding_echo_control_command_mode(serializer, &self->mode);
    
}

void insert_mip_aiding_aiding_echo_control_command_mode(struct mip_serializer* serializer, const mip_aiding_aiding_echo_control_command_mode self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_aiding_aiding_echo_control_command_mode(struct mip_serializer* serializer, mip_aiding_aiding_echo_control_command_mode* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_write_aiding_echo_control(struct mip_interface* device, mip_aiding_aiding_echo_control_command_mode mode)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_WRITE);
    
    insert_mip_aiding_aiding_echo_control_command_mode(&serializer, mode);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_read_aiding_echo_control(struct mip_interface* device, mip_aiding_aiding_echo_control_command_mode* mode_out)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_READ);
    
    assert(mip_serializer_is_ok(&serializer));
    
    uint8_t responseLength = sizeof(buffer);
    mip_cmd_result result = mip_interface_run_command_with_response(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), MIP_REPLY_DESC_AIDING_ECHO_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        mip_serializer deserializer;
        mip_serializer_init_insertion(&deserializer, buffer, responseLength);
        
        assert(mode_out);
        extract_mip_aiding_aiding_echo_control_command_mode(&deserializer, mode_out);
        
        if( mip_serializer_remaining(&deserializer) != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
mip_cmd_result mip_aiding_save_aiding_echo_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_SAVE);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_load_aiding_echo_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_LOAD);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
mip_cmd_result mip_aiding_default_aiding_echo_control(struct mip_interface* device)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    insert_mip_function_selector(&serializer, MIP_FUNCTION_RESET);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_ecef_pos_command(mip_serializer* serializer, const mip_aiding_ecef_pos_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ecef_pos_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ecef_pos_command(mip_serializer* serializer, mip_aiding_ecef_pos_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ecef_pos_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ecef_pos_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ecef_pos_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_ecef_pos_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ecef_pos_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_ecef_pos(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const double* position, const float* uncertainty, mip_aiding_ecef_pos_command_valid_flags valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    assert(position || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_double(&serializer, position[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_ecef_pos_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_POS_ECEF, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_llh_pos_command(mip_serializer* serializer, const mip_aiding_llh_pos_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    insert_double(serializer, self->latitude);
    
    insert_double(serializer, self->longitude);
    
    insert_double(serializer, self->height);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_llh_pos_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_llh_pos_command(mip_serializer* serializer, mip_aiding_llh_pos_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    extract_double(serializer, &self->latitude);
    
    extract_double(serializer, &self->longitude);
    
    extract_double(serializer, &self->height);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_llh_pos_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_llh_pos_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_llh_pos_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_llh_pos_command_valid_flags(struct mip_serializer* serializer, mip_aiding_llh_pos_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_llh_pos(struct mip_interface* device, const mip_time* time, uint8_t frame_id, double latitude, double longitude, double height, const float* uncertainty, mip_aiding_llh_pos_command_valid_flags valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    insert_double(&serializer, latitude);
    
    insert_double(&serializer, longitude);
    
    insert_double(&serializer, height);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_llh_pos_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_POS_LLH, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_height_command(mip_serializer* serializer, const mip_aiding_height_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    insert_float(serializer, self->height);
    
    insert_float(serializer, self->uncertainty);
    
    insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_height_command(mip_serializer* serializer, mip_aiding_height_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    extract_float(serializer, &self->height);
    
    extract_float(serializer, &self->uncertainty);
    
    extract_u16(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_height(struct mip_interface* device, const mip_time* time, uint8_t frame_id, float height, float uncertainty, uint16_t valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    insert_float(&serializer, height);
    
    insert_float(&serializer, uncertainty);
    
    insert_u16(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_HEIGHT_ABS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_ecef_vel_command(mip_serializer* serializer, const mip_aiding_ecef_vel_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ecef_vel_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ecef_vel_command(mip_serializer* serializer, mip_aiding_ecef_vel_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ecef_vel_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ecef_vel_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ecef_vel_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_ecef_vel_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ecef_vel_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_ecef_vel(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_ecef_vel_command_valid_flags valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_ecef_vel_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_VEL_ECEF, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_ned_vel_command(mip_serializer* serializer, const mip_aiding_ned_vel_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_ned_vel_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_ned_vel_command(mip_serializer* serializer, mip_aiding_ned_vel_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_ned_vel_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_ned_vel_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_ned_vel_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_ned_vel_command_valid_flags(struct mip_serializer* serializer, mip_aiding_ned_vel_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_ned_vel(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_ned_vel_command_valid_flags valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_ned_vel_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_VEL_NED, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_vehicle_fixed_frame_velocity_command(mip_serializer* serializer, const mip_aiding_vehicle_fixed_frame_velocity_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_vehicle_fixed_frame_velocity_command(mip_serializer* serializer, mip_aiding_vehicle_fixed_frame_velocity_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(struct mip_serializer* serializer, mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_vehicle_fixed_frame_velocity(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* velocity, const float* uncertainty, mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_vehicle_fixed_frame_velocity_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_VEL_ODOM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_true_heading_command(mip_serializer* serializer, const mip_aiding_true_heading_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->uncertainty);
    
    insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_true_heading_command(mip_serializer* serializer, mip_aiding_true_heading_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->uncertainty);
    
    extract_u16(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_true_heading(struct mip_interface* device, const mip_time* time, uint8_t frame_id, float heading, float uncertainty, uint16_t valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    insert_float(&serializer, heading);
    
    insert_float(&serializer, uncertainty);
    
    insert_u16(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_HEADING_TRUE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_magnetic_field_command(mip_serializer* serializer, const mip_aiding_magnetic_field_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->magnetic_field[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->uncertainty[i]);
    
    insert_mip_aiding_magnetic_field_command_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_aiding_magnetic_field_command(mip_serializer* serializer, mip_aiding_magnetic_field_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->magnetic_field[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->uncertainty[i]);
    
    extract_mip_aiding_magnetic_field_command_valid_flags(serializer, &self->valid_flags);
    
}

void insert_mip_aiding_magnetic_field_command_valid_flags(struct mip_serializer* serializer, const mip_aiding_magnetic_field_command_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_aiding_magnetic_field_command_valid_flags(struct mip_serializer* serializer, mip_aiding_magnetic_field_command_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

mip_cmd_result mip_aiding_magnetic_field(struct mip_interface* device, const mip_time* time, uint8_t frame_id, const float* magnetic_field, const float* uncertainty, mip_aiding_magnetic_field_command_valid_flags valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    assert(magnetic_field || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, magnetic_field[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert_float(&serializer, uncertainty[i]);
    
    insert_mip_aiding_magnetic_field_command_valid_flags(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_MAGNETIC_FIELD, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert_mip_aiding_pressure_command(mip_serializer* serializer, const mip_aiding_pressure_command* self)
{
    insert_mip_time(serializer, &self->time);
    
    insert_u8(serializer, self->frame_id);
    
    insert_float(serializer, self->pressure);
    
    insert_float(serializer, self->uncertainty);
    
    insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_aiding_pressure_command(mip_serializer* serializer, mip_aiding_pressure_command* self)
{
    extract_mip_time(serializer, &self->time);
    
    extract_u8(serializer, &self->frame_id);
    
    extract_float(serializer, &self->pressure);
    
    extract_float(serializer, &self->uncertainty);
    
    extract_u16(serializer, &self->valid_flags);
    
}

mip_cmd_result mip_aiding_pressure(struct mip_interface* device, const mip_time* time, uint8_t frame_id, float pressure, float uncertainty, uint16_t valid_flags)
{
    mip_serializer serializer;
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    mip_serializer_init_insertion(&serializer, buffer, sizeof(buffer));
    
    assert(time);
    insert_mip_time(&serializer, time);
    
    insert_u8(&serializer, frame_id);
    
    insert_float(&serializer, pressure);
    
    insert_float(&serializer, uncertainty);
    
    insert_u16(&serializer, valid_flags);
    
    assert(mip_serializer_is_ok(&serializer));
    
    return mip_interface_run_command(device, MIP_AIDING_CMD_DESC_SET, MIP_CMD_DESC_AIDING_PRESSURE, buffer, (uint8_t)mip_serializer_length(&serializer));
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

