
#include "data_gnss.h"

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

void insert_mip_gnss_constellation_id(struct mip_serializer* serializer, const mip_gnss_constellation_id self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_constellation_id(struct mip_serializer* serializer, mip_gnss_constellation_id* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_signal_id(struct mip_serializer* serializer, const mip_gnss_signal_id self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_signal_id(struct mip_serializer* serializer, mip_gnss_signal_id* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_sbas_system(struct mip_serializer* serializer, const mip_sbas_system self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_sbas_system(struct mip_serializer* serializer, mip_sbas_system* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_gnss_pos_llh_data(mip_serializer* serializer, const mip_gnss_pos_llh_data* self)
{
    insert_double(serializer, self->latitude);
    
    insert_double(serializer, self->longitude);
    
    insert_double(serializer, self->ellipsoid_height);
    
    insert_double(serializer, self->msl_height);
    
    insert_float(serializer, self->horizontal_accuracy);
    
    insert_float(serializer, self->vertical_accuracy);
    
    insert_mip_gnss_pos_llh_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_pos_llh_data(mip_serializer* serializer, mip_gnss_pos_llh_data* self)
{
    extract_double(serializer, &self->latitude);
    
    extract_double(serializer, &self->longitude);
    
    extract_double(serializer, &self->ellipsoid_height);
    
    extract_double(serializer, &self->msl_height);
    
    extract_float(serializer, &self->horizontal_accuracy);
    
    extract_float(serializer, &self->vertical_accuracy);
    
    extract_mip_gnss_pos_llh_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_pos_llh_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_pos_llh_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_pos_llh_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_pos_llh_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_pos_llh_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_pos_llh_data_valid_flags(struct mip_serializer* serializer, mip_gnss_pos_llh_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_pos_ecef_data(mip_serializer* serializer, const mip_gnss_pos_ecef_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->x[i]);
    
    insert_float(serializer, self->x_accuracy);
    
    insert_mip_gnss_pos_ecef_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_pos_ecef_data(mip_serializer* serializer, mip_gnss_pos_ecef_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->x[i]);
    
    extract_float(serializer, &self->x_accuracy);
    
    extract_mip_gnss_pos_ecef_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_pos_ecef_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_pos_ecef_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_pos_ecef_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_pos_ecef_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_pos_ecef_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_pos_ecef_data_valid_flags(struct mip_serializer* serializer, mip_gnss_pos_ecef_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_vel_ned_data(mip_serializer* serializer, const mip_gnss_vel_ned_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->v[i]);
    
    insert_float(serializer, self->speed);
    
    insert_float(serializer, self->ground_speed);
    
    insert_float(serializer, self->heading);
    
    insert_float(serializer, self->speed_accuracy);
    
    insert_float(serializer, self->heading_accuracy);
    
    insert_mip_gnss_vel_ned_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_vel_ned_data(mip_serializer* serializer, mip_gnss_vel_ned_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->v[i]);
    
    extract_float(serializer, &self->speed);
    
    extract_float(serializer, &self->ground_speed);
    
    extract_float(serializer, &self->heading);
    
    extract_float(serializer, &self->speed_accuracy);
    
    extract_float(serializer, &self->heading_accuracy);
    
    extract_mip_gnss_vel_ned_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_vel_ned_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_vel_ned_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_vel_ned_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_vel_ned_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_vel_ned_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_vel_ned_data_valid_flags(struct mip_serializer* serializer, mip_gnss_vel_ned_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_vel_ecef_data(mip_serializer* serializer, const mip_gnss_vel_ecef_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->v[i]);
    
    insert_float(serializer, self->v_accuracy);
    
    insert_mip_gnss_vel_ecef_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_vel_ecef_data(mip_serializer* serializer, mip_gnss_vel_ecef_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->v[i]);
    
    extract_float(serializer, &self->v_accuracy);
    
    extract_mip_gnss_vel_ecef_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_vel_ecef_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_vel_ecef_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_vel_ecef_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_vel_ecef_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_vel_ecef_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_vel_ecef_data_valid_flags(struct mip_serializer* serializer, mip_gnss_vel_ecef_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_dop_data(mip_serializer* serializer, const mip_gnss_dop_data* self)
{
    insert_float(serializer, self->gdop);
    
    insert_float(serializer, self->pdop);
    
    insert_float(serializer, self->hdop);
    
    insert_float(serializer, self->vdop);
    
    insert_float(serializer, self->tdop);
    
    insert_float(serializer, self->ndop);
    
    insert_float(serializer, self->edop);
    
    insert_mip_gnss_dop_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_dop_data(mip_serializer* serializer, mip_gnss_dop_data* self)
{
    extract_float(serializer, &self->gdop);
    
    extract_float(serializer, &self->pdop);
    
    extract_float(serializer, &self->hdop);
    
    extract_float(serializer, &self->vdop);
    
    extract_float(serializer, &self->tdop);
    
    extract_float(serializer, &self->ndop);
    
    extract_float(serializer, &self->edop);
    
    extract_mip_gnss_dop_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_dop_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_dop_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_dop_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_dop_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_dop_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_dop_data_valid_flags(struct mip_serializer* serializer, mip_gnss_dop_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_utc_time_data(mip_serializer* serializer, const mip_gnss_utc_time_data* self)
{
    insert_u16(serializer, self->year);
    
    insert_u8(serializer, self->month);
    
    insert_u8(serializer, self->day);
    
    insert_u8(serializer, self->hour);
    
    insert_u8(serializer, self->min);
    
    insert_u8(serializer, self->sec);
    
    insert_u32(serializer, self->msec);
    
    insert_mip_gnss_utc_time_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_utc_time_data(mip_serializer* serializer, mip_gnss_utc_time_data* self)
{
    extract_u16(serializer, &self->year);
    
    extract_u8(serializer, &self->month);
    
    extract_u8(serializer, &self->day);
    
    extract_u8(serializer, &self->hour);
    
    extract_u8(serializer, &self->min);
    
    extract_u8(serializer, &self->sec);
    
    extract_u32(serializer, &self->msec);
    
    extract_mip_gnss_utc_time_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_utc_time_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_utc_time_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_utc_time_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_utc_time_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_utc_time_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_utc_time_data_valid_flags(struct mip_serializer* serializer, mip_gnss_utc_time_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_gps_time_data(mip_serializer* serializer, const mip_gnss_gps_time_data* self)
{
    insert_double(serializer, self->tow);
    
    insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_gps_time_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_time_data(mip_serializer* serializer, mip_gnss_gps_time_data* self)
{
    extract_double(serializer, &self->tow);
    
    extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_gps_time_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_time_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_time_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_time_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_time_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_gps_time_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_gps_time_data_valid_flags(struct mip_serializer* serializer, mip_gnss_gps_time_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_clock_info_data(mip_serializer* serializer, const mip_gnss_clock_info_data* self)
{
    insert_double(serializer, self->bias);
    
    insert_double(serializer, self->drift);
    
    insert_double(serializer, self->accuracy_estimate);
    
    insert_mip_gnss_clock_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_clock_info_data(mip_serializer* serializer, mip_gnss_clock_info_data* self)
{
    extract_double(serializer, &self->bias);
    
    extract_double(serializer, &self->drift);
    
    extract_double(serializer, &self->accuracy_estimate);
    
    extract_mip_gnss_clock_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_clock_info_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_clock_info_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_clock_info_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_clock_info_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_clock_info_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_clock_info_data_valid_flags(struct mip_serializer* serializer, mip_gnss_clock_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_fix_info_data(mip_serializer* serializer, const mip_gnss_fix_info_data* self)
{
    insert_mip_gnss_fix_info_data_fix_type(serializer, self->fix_type);
    
    insert_u8(serializer, self->num_sv);
    
    insert_mip_gnss_fix_info_data_fix_flags(serializer, self->fix_flags);
    
    insert_mip_gnss_fix_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_fix_info_data(mip_serializer* serializer, mip_gnss_fix_info_data* self)
{
    extract_mip_gnss_fix_info_data_fix_type(serializer, &self->fix_type);
    
    extract_u8(serializer, &self->num_sv);
    
    extract_mip_gnss_fix_info_data_fix_flags(serializer, &self->fix_flags);
    
    extract_mip_gnss_fix_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_fix_info_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_fix_info_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_fix_info_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_fix_info_data_fix_type(struct mip_serializer* serializer, const mip_gnss_fix_info_data_fix_type self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_fix_info_data_fix_type(struct mip_serializer* serializer, mip_gnss_fix_info_data_fix_type* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_fix_info_data_fix_flags(struct mip_serializer* serializer, const mip_gnss_fix_info_data_fix_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_fix_info_data_fix_flags(struct mip_serializer* serializer, mip_gnss_fix_info_data_fix_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_fix_info_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_fix_info_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_fix_info_data_valid_flags(struct mip_serializer* serializer, mip_gnss_fix_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_sv_info_data(mip_serializer* serializer, const mip_gnss_sv_info_data* self)
{
    insert_u8(serializer, self->channel);
    
    insert_u8(serializer, self->sv_id);
    
    insert_u16(serializer, self->carrier_noise_ratio);
    
    insert_s16(serializer, self->azimuth);
    
    insert_s16(serializer, self->elevation);
    
    insert_mip_gnss_sv_info_data_svflags(serializer, self->sv_flags);
    
    insert_mip_gnss_sv_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_sv_info_data(mip_serializer* serializer, mip_gnss_sv_info_data* self)
{
    extract_u8(serializer, &self->channel);
    
    extract_u8(serializer, &self->sv_id);
    
    extract_u16(serializer, &self->carrier_noise_ratio);
    
    extract_s16(serializer, &self->azimuth);
    
    extract_s16(serializer, &self->elevation);
    
    extract_mip_gnss_sv_info_data_svflags(serializer, &self->sv_flags);
    
    extract_mip_gnss_sv_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_sv_info_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_sv_info_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_sv_info_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_sv_info_data_svflags(struct mip_serializer* serializer, const mip_gnss_sv_info_data_svflags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_sv_info_data_svflags(struct mip_serializer* serializer, mip_gnss_sv_info_data_svflags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_sv_info_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_sv_info_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_sv_info_data_valid_flags(struct mip_serializer* serializer, mip_gnss_sv_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_hw_status_data(mip_serializer* serializer, const mip_gnss_hw_status_data* self)
{
    insert_mip_gnss_hw_status_data_receiver_state(serializer, self->receiver_state);
    
    insert_mip_gnss_hw_status_data_antenna_state(serializer, self->antenna_state);
    
    insert_mip_gnss_hw_status_data_antenna_power(serializer, self->antenna_power);
    
    insert_mip_gnss_hw_status_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_hw_status_data(mip_serializer* serializer, mip_gnss_hw_status_data* self)
{
    extract_mip_gnss_hw_status_data_receiver_state(serializer, &self->receiver_state);
    
    extract_mip_gnss_hw_status_data_antenna_state(serializer, &self->antenna_state);
    
    extract_mip_gnss_hw_status_data_antenna_power(serializer, &self->antenna_power);
    
    extract_mip_gnss_hw_status_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_hw_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_hw_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_hw_status_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_hw_status_data_receiver_state(struct mip_serializer* serializer, const mip_gnss_hw_status_data_receiver_state self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_hw_status_data_receiver_state(struct mip_serializer* serializer, mip_gnss_hw_status_data_receiver_state* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_hw_status_data_antenna_state(struct mip_serializer* serializer, const mip_gnss_hw_status_data_antenna_state self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_hw_status_data_antenna_state(struct mip_serializer* serializer, mip_gnss_hw_status_data_antenna_state* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_hw_status_data_antenna_power(struct mip_serializer* serializer, const mip_gnss_hw_status_data_antenna_power self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_hw_status_data_antenna_power(struct mip_serializer* serializer, mip_gnss_hw_status_data_antenna_power* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_hw_status_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_hw_status_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_hw_status_data_valid_flags(struct mip_serializer* serializer, mip_gnss_hw_status_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_dgps_info_data(mip_serializer* serializer, const mip_gnss_dgps_info_data* self)
{
    insert_u8(serializer, self->sv_id);
    
    insert_float(serializer, self->age);
    
    insert_float(serializer, self->range_correction);
    
    insert_float(serializer, self->range_rate_correction);
    
    insert_mip_gnss_dgps_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_dgps_info_data(mip_serializer* serializer, mip_gnss_dgps_info_data* self)
{
    extract_u8(serializer, &self->sv_id);
    
    extract_float(serializer, &self->age);
    
    extract_float(serializer, &self->range_correction);
    
    extract_float(serializer, &self->range_rate_correction);
    
    extract_mip_gnss_dgps_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_dgps_info_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_dgps_info_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_dgps_info_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_dgps_info_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_dgps_info_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_dgps_info_data_valid_flags(struct mip_serializer* serializer, mip_gnss_dgps_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_dgps_channel_data(mip_serializer* serializer, const mip_gnss_dgps_channel_data* self)
{
    insert_u8(serializer, self->sv_id);
    
    insert_float(serializer, self->age);
    
    insert_float(serializer, self->range_correction);
    
    insert_float(serializer, self->range_rate_correction);
    
    insert_mip_gnss_dgps_channel_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_dgps_channel_data(mip_serializer* serializer, mip_gnss_dgps_channel_data* self)
{
    extract_u8(serializer, &self->sv_id);
    
    extract_float(serializer, &self->age);
    
    extract_float(serializer, &self->range_correction);
    
    extract_float(serializer, &self->range_rate_correction);
    
    extract_mip_gnss_dgps_channel_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_dgps_channel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_dgps_channel_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_dgps_channel_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_dgps_channel_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_dgps_channel_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_dgps_channel_data_valid_flags(struct mip_serializer* serializer, mip_gnss_dgps_channel_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_clock_info_2_data(mip_serializer* serializer, const mip_gnss_clock_info_2_data* self)
{
    insert_double(serializer, self->bias);
    
    insert_double(serializer, self->drift);
    
    insert_double(serializer, self->bias_accuracy_estimate);
    
    insert_double(serializer, self->drift_accuracy_estimate);
    
    insert_mip_gnss_clock_info_2_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_clock_info_2_data(mip_serializer* serializer, mip_gnss_clock_info_2_data* self)
{
    extract_double(serializer, &self->bias);
    
    extract_double(serializer, &self->drift);
    
    extract_double(serializer, &self->bias_accuracy_estimate);
    
    extract_double(serializer, &self->drift_accuracy_estimate);
    
    extract_mip_gnss_clock_info_2_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_clock_info_2_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_clock_info_2_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_clock_info_2_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_clock_info_2_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_clock_info_2_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_clock_info_2_data_valid_flags(struct mip_serializer* serializer, mip_gnss_clock_info_2_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_gps_leap_seconds_data(mip_serializer* serializer, const mip_gnss_gps_leap_seconds_data* self)
{
    insert_u8(serializer, self->leap_seconds);
    
    insert_mip_gnss_gps_leap_seconds_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_leap_seconds_data(mip_serializer* serializer, mip_gnss_gps_leap_seconds_data* self)
{
    extract_u8(serializer, &self->leap_seconds);
    
    extract_mip_gnss_gps_leap_seconds_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_leap_seconds_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_leap_seconds_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_leap_seconds_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_leap_seconds_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_gps_leap_seconds_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_gps_leap_seconds_data_valid_flags(struct mip_serializer* serializer, mip_gnss_gps_leap_seconds_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_sbas_info_data(mip_serializer* serializer, const mip_gnss_sbas_info_data* self)
{
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_mip_sbas_system(serializer, self->sbas_system);
    
    insert_u8(serializer, self->sbas_id);
    
    insert_u8(serializer, self->count);
    
    insert_mip_gnss_sbas_info_data_sbas_status(serializer, self->sbas_status);
    
    insert_mip_gnss_sbas_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_sbas_info_data(mip_serializer* serializer, mip_gnss_sbas_info_data* self)
{
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_mip_sbas_system(serializer, &self->sbas_system);
    
    extract_u8(serializer, &self->sbas_id);
    
    extract_u8(serializer, &self->count);
    
    extract_mip_gnss_sbas_info_data_sbas_status(serializer, &self->sbas_status);
    
    extract_mip_gnss_sbas_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_sbas_info_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_sbas_info_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_sbas_info_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_sbas_info_data_sbas_status(struct mip_serializer* serializer, const mip_gnss_sbas_info_data_sbas_status self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_sbas_info_data_sbas_status(struct mip_serializer* serializer, mip_gnss_sbas_info_data_sbas_status* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_sbas_info_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_sbas_info_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_sbas_info_data_valid_flags(struct mip_serializer* serializer, mip_gnss_sbas_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_sbas_correction_data(mip_serializer* serializer, const mip_gnss_sbas_correction_data* self)
{
    insert_u8(serializer, self->index);
    
    insert_u8(serializer, self->count);
    
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_constellation_id(serializer, self->gnss_id);
    
    insert_u8(serializer, self->sv_id);
    
    insert_u8(serializer, self->udrei);
    
    insert_float(serializer, self->pseudorange_correction);
    
    insert_float(serializer, self->iono_correction);
    
    insert_mip_gnss_sbas_correction_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_sbas_correction_data(mip_serializer* serializer, mip_gnss_sbas_correction_data* self)
{
    extract_u8(serializer, &self->index);
    
    extract_u8(serializer, &self->count);
    
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_constellation_id(serializer, &self->gnss_id);
    
    extract_u8(serializer, &self->sv_id);
    
    extract_u8(serializer, &self->udrei);
    
    extract_float(serializer, &self->pseudorange_correction);
    
    extract_float(serializer, &self->iono_correction);
    
    extract_mip_gnss_sbas_correction_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_sbas_correction_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_sbas_correction_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_sbas_correction_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_sbas_correction_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_sbas_correction_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_sbas_correction_data_valid_flags(struct mip_serializer* serializer, mip_gnss_sbas_correction_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_rf_error_detection_data(mip_serializer* serializer, const mip_gnss_rf_error_detection_data* self)
{
    insert_mip_gnss_rf_error_detection_data_rfband(serializer, self->rf_band);
    
    insert_mip_gnss_rf_error_detection_data_jamming_state(serializer, self->jamming_state);
    
    insert_mip_gnss_rf_error_detection_data_spoofing_state(serializer, self->spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        insert_u8(serializer, self->reserved[i]);
    
    insert_mip_gnss_rf_error_detection_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_rf_error_detection_data(mip_serializer* serializer, mip_gnss_rf_error_detection_data* self)
{
    extract_mip_gnss_rf_error_detection_data_rfband(serializer, &self->rf_band);
    
    extract_mip_gnss_rf_error_detection_data_jamming_state(serializer, &self->jamming_state);
    
    extract_mip_gnss_rf_error_detection_data_spoofing_state(serializer, &self->spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        extract_u8(serializer, &self->reserved[i]);
    
    extract_mip_gnss_rf_error_detection_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_rf_error_detection_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_rf_error_detection_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_rf_error_detection_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_rf_error_detection_data_rfband(struct mip_serializer* serializer, const mip_gnss_rf_error_detection_data_rfband self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_rf_error_detection_data_rfband(struct mip_serializer* serializer, mip_gnss_rf_error_detection_data_rfband* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_rf_error_detection_data_jamming_state(struct mip_serializer* serializer, const mip_gnss_rf_error_detection_data_jamming_state self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_rf_error_detection_data_jamming_state(struct mip_serializer* serializer, mip_gnss_rf_error_detection_data_jamming_state* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_rf_error_detection_data_spoofing_state(struct mip_serializer* serializer, const mip_gnss_rf_error_detection_data_spoofing_state self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_rf_error_detection_data_spoofing_state(struct mip_serializer* serializer, mip_gnss_rf_error_detection_data_spoofing_state* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_rf_error_detection_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_rf_error_detection_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_rf_error_detection_data_valid_flags(struct mip_serializer* serializer, mip_gnss_rf_error_detection_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_base_station_info_data(mip_serializer* serializer, const mip_gnss_base_station_info_data* self)
{
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->ecef_pos[i]);
    
    insert_float(serializer, self->height);
    
    insert_u16(serializer, self->station_id);
    
    insert_mip_gnss_base_station_info_data_indicator_flags(serializer, self->indicators);
    
    insert_mip_gnss_base_station_info_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_base_station_info_data(mip_serializer* serializer, mip_gnss_base_station_info_data* self)
{
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->ecef_pos[i]);
    
    extract_float(serializer, &self->height);
    
    extract_u16(serializer, &self->station_id);
    
    extract_mip_gnss_base_station_info_data_indicator_flags(serializer, &self->indicators);
    
    extract_mip_gnss_base_station_info_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_base_station_info_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_base_station_info_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_base_station_info_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_base_station_info_data_indicator_flags(struct mip_serializer* serializer, const mip_gnss_base_station_info_data_indicator_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_base_station_info_data_indicator_flags(struct mip_serializer* serializer, mip_gnss_base_station_info_data_indicator_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_base_station_info_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_base_station_info_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_base_station_info_data_valid_flags(struct mip_serializer* serializer, mip_gnss_base_station_info_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_rtk_corrections_status_data(mip_serializer* serializer, const mip_gnss_rtk_corrections_status_data* self)
{
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_rtk_corrections_status_data_epoch_status(serializer, self->epoch_status);
    
    insert_u32(serializer, self->dongle_status);
    
    insert_float(serializer, self->gps_correction_latency);
    
    insert_float(serializer, self->glonass_correction_latency);
    
    insert_float(serializer, self->galileo_correction_latency);
    
    insert_float(serializer, self->beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        insert_u32(serializer, self->reserved[i]);
    
    insert_mip_gnss_rtk_corrections_status_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_rtk_corrections_status_data(mip_serializer* serializer, mip_gnss_rtk_corrections_status_data* self)
{
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_rtk_corrections_status_data_epoch_status(serializer, &self->epoch_status);
    
    extract_u32(serializer, &self->dongle_status);
    
    extract_float(serializer, &self->gps_correction_latency);
    
    extract_float(serializer, &self->glonass_correction_latency);
    
    extract_float(serializer, &self->galileo_correction_latency);
    
    extract_float(serializer, &self->beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        extract_u32(serializer, &self->reserved[i]);
    
    extract_mip_gnss_rtk_corrections_status_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_rtk_corrections_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_rtk_corrections_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_rtk_corrections_status_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_rtk_corrections_status_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_rtk_corrections_status_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_rtk_corrections_status_data_valid_flags(struct mip_serializer* serializer, mip_gnss_rtk_corrections_status_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_rtk_corrections_status_data_epoch_status(struct mip_serializer* serializer, const mip_gnss_rtk_corrections_status_data_epoch_status self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_rtk_corrections_status_data_epoch_status(struct mip_serializer* serializer, mip_gnss_rtk_corrections_status_data_epoch_status* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_satellite_status_data(mip_serializer* serializer, const mip_gnss_satellite_status_data* self)
{
    insert_u8(serializer, self->index);
    
    insert_u8(serializer, self->count);
    
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_mip_gnss_constellation_id(serializer, self->gnss_id);
    
    insert_u8(serializer, self->satellite_id);
    
    insert_float(serializer, self->elevation);
    
    insert_float(serializer, self->azimuth);
    
    insert_bool(serializer, self->health);
    
    insert_mip_gnss_satellite_status_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_satellite_status_data(mip_serializer* serializer, mip_gnss_satellite_status_data* self)
{
    extract_u8(serializer, &self->index);
    
    extract_u8(serializer, &self->count);
    
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_mip_gnss_constellation_id(serializer, &self->gnss_id);
    
    extract_u8(serializer, &self->satellite_id);
    
    extract_float(serializer, &self->elevation);
    
    extract_float(serializer, &self->azimuth);
    
    extract_bool(serializer, &self->health);
    
    extract_mip_gnss_satellite_status_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_satellite_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_satellite_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_satellite_status_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_satellite_status_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_satellite_status_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_satellite_status_data_valid_flags(struct mip_serializer* serializer, mip_gnss_satellite_status_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_raw_data(mip_serializer* serializer, const mip_gnss_raw_data* self)
{
    insert_u8(serializer, self->index);
    
    insert_u8(serializer, self->count);
    
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_u16(serializer, self->receiver_id);
    
    insert_u8(serializer, self->tracking_channel);
    
    insert_mip_gnss_constellation_id(serializer, self->gnss_id);
    
    insert_u8(serializer, self->satellite_id);
    
    insert_mip_gnss_signal_id(serializer, self->signal_id);
    
    insert_float(serializer, self->signal_strength);
    
    insert_mip_gnss_raw_data_gnss_signal_quality(serializer, self->quality);
    
    insert_double(serializer, self->pseudorange);
    
    insert_double(serializer, self->carrier_phase);
    
    insert_float(serializer, self->doppler);
    
    insert_float(serializer, self->range_uncert);
    
    insert_float(serializer, self->phase_uncert);
    
    insert_float(serializer, self->doppler_uncert);
    
    insert_float(serializer, self->lock_time);
    
    insert_mip_gnss_raw_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_raw_data(mip_serializer* serializer, mip_gnss_raw_data* self)
{
    extract_u8(serializer, &self->index);
    
    extract_u8(serializer, &self->count);
    
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_u16(serializer, &self->receiver_id);
    
    extract_u8(serializer, &self->tracking_channel);
    
    extract_mip_gnss_constellation_id(serializer, &self->gnss_id);
    
    extract_u8(serializer, &self->satellite_id);
    
    extract_mip_gnss_signal_id(serializer, &self->signal_id);
    
    extract_float(serializer, &self->signal_strength);
    
    extract_mip_gnss_raw_data_gnss_signal_quality(serializer, &self->quality);
    
    extract_double(serializer, &self->pseudorange);
    
    extract_double(serializer, &self->carrier_phase);
    
    extract_float(serializer, &self->doppler);
    
    extract_float(serializer, &self->range_uncert);
    
    extract_float(serializer, &self->phase_uncert);
    
    extract_float(serializer, &self->doppler_uncert);
    
    extract_float(serializer, &self->lock_time);
    
    extract_mip_gnss_raw_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_raw_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_raw_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_raw_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_raw_data_gnss_signal_quality(struct mip_serializer* serializer, const mip_gnss_raw_data_gnss_signal_quality self)
{
    insert_u8(serializer, (uint8_t)(self));
}
void extract_mip_gnss_raw_data_gnss_signal_quality(struct mip_serializer* serializer, mip_gnss_raw_data_gnss_signal_quality* self)
{
    uint8_t tmp = 0;
    extract_u8(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_raw_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_raw_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_raw_data_valid_flags(struct mip_serializer* serializer, mip_gnss_raw_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_gps_ephemeris_data(mip_serializer* serializer, const mip_gnss_gps_ephemeris_data* self)
{
    insert_u8(serializer, self->index);
    
    insert_u8(serializer, self->count);
    
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_u8(serializer, self->satellite_id);
    
    insert_u8(serializer, self->health);
    
    insert_u8(serializer, self->iodc);
    
    insert_u8(serializer, self->iode);
    
    insert_double(serializer, self->t_oc);
    
    insert_double(serializer, self->af0);
    
    insert_double(serializer, self->af1);
    
    insert_double(serializer, self->af2);
    
    insert_double(serializer, self->t_gd);
    
    insert_double(serializer, self->ISC_L1CA);
    
    insert_double(serializer, self->ISC_L2C);
    
    insert_double(serializer, self->t_oe);
    
    insert_double(serializer, self->a);
    
    insert_double(serializer, self->a_dot);
    
    insert_double(serializer, self->mean_anomaly);
    
    insert_double(serializer, self->delta_mean_motion);
    
    insert_double(serializer, self->delta_mean_motion_dot);
    
    insert_double(serializer, self->eccentricity);
    
    insert_double(serializer, self->argument_of_perigee);
    
    insert_double(serializer, self->omega);
    
    insert_double(serializer, self->omega_dot);
    
    insert_double(serializer, self->inclination);
    
    insert_double(serializer, self->inclination_dot);
    
    insert_double(serializer, self->c_ic);
    
    insert_double(serializer, self->c_is);
    
    insert_double(serializer, self->c_uc);
    
    insert_double(serializer, self->c_us);
    
    insert_double(serializer, self->c_rc);
    
    insert_double(serializer, self->c_rs);
    
    insert_mip_gnss_gps_ephemeris_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_ephemeris_data(mip_serializer* serializer, mip_gnss_gps_ephemeris_data* self)
{
    extract_u8(serializer, &self->index);
    
    extract_u8(serializer, &self->count);
    
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_u8(serializer, &self->satellite_id);
    
    extract_u8(serializer, &self->health);
    
    extract_u8(serializer, &self->iodc);
    
    extract_u8(serializer, &self->iode);
    
    extract_double(serializer, &self->t_oc);
    
    extract_double(serializer, &self->af0);
    
    extract_double(serializer, &self->af1);
    
    extract_double(serializer, &self->af2);
    
    extract_double(serializer, &self->t_gd);
    
    extract_double(serializer, &self->ISC_L1CA);
    
    extract_double(serializer, &self->ISC_L2C);
    
    extract_double(serializer, &self->t_oe);
    
    extract_double(serializer, &self->a);
    
    extract_double(serializer, &self->a_dot);
    
    extract_double(serializer, &self->mean_anomaly);
    
    extract_double(serializer, &self->delta_mean_motion);
    
    extract_double(serializer, &self->delta_mean_motion_dot);
    
    extract_double(serializer, &self->eccentricity);
    
    extract_double(serializer, &self->argument_of_perigee);
    
    extract_double(serializer, &self->omega);
    
    extract_double(serializer, &self->omega_dot);
    
    extract_double(serializer, &self->inclination);
    
    extract_double(serializer, &self->inclination_dot);
    
    extract_double(serializer, &self->c_ic);
    
    extract_double(serializer, &self->c_is);
    
    extract_double(serializer, &self->c_uc);
    
    extract_double(serializer, &self->c_us);
    
    extract_double(serializer, &self->c_rc);
    
    extract_double(serializer, &self->c_rs);
    
    extract_mip_gnss_gps_ephemeris_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_ephemeris_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_ephemeris_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_ephemeris_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_ephemeris_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_gps_ephemeris_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_gps_ephemeris_data_valid_flags(struct mip_serializer* serializer, mip_gnss_gps_ephemeris_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_galileo_ephemeris_data(mip_serializer* serializer, const mip_gnss_galileo_ephemeris_data* self)
{
    insert_u8(serializer, self->index);
    
    insert_u8(serializer, self->count);
    
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_u8(serializer, self->satellite_id);
    
    insert_u8(serializer, self->health);
    
    insert_u8(serializer, self->iodc);
    
    insert_u8(serializer, self->iode);
    
    insert_double(serializer, self->t_oc);
    
    insert_double(serializer, self->af0);
    
    insert_double(serializer, self->af1);
    
    insert_double(serializer, self->af2);
    
    insert_double(serializer, self->t_gd);
    
    insert_double(serializer, self->ISC_L1CA);
    
    insert_double(serializer, self->ISC_L2C);
    
    insert_double(serializer, self->t_oe);
    
    insert_double(serializer, self->a);
    
    insert_double(serializer, self->a_dot);
    
    insert_double(serializer, self->mean_anomaly);
    
    insert_double(serializer, self->delta_mean_motion);
    
    insert_double(serializer, self->delta_mean_motion_dot);
    
    insert_double(serializer, self->eccentricity);
    
    insert_double(serializer, self->argument_of_perigee);
    
    insert_double(serializer, self->omega);
    
    insert_double(serializer, self->omega_dot);
    
    insert_double(serializer, self->inclination);
    
    insert_double(serializer, self->inclination_dot);
    
    insert_double(serializer, self->c_ic);
    
    insert_double(serializer, self->c_is);
    
    insert_double(serializer, self->c_uc);
    
    insert_double(serializer, self->c_us);
    
    insert_double(serializer, self->c_rc);
    
    insert_double(serializer, self->c_rs);
    
    insert_mip_gnss_galileo_ephemeris_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_galileo_ephemeris_data(mip_serializer* serializer, mip_gnss_galileo_ephemeris_data* self)
{
    extract_u8(serializer, &self->index);
    
    extract_u8(serializer, &self->count);
    
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_u8(serializer, &self->satellite_id);
    
    extract_u8(serializer, &self->health);
    
    extract_u8(serializer, &self->iodc);
    
    extract_u8(serializer, &self->iode);
    
    extract_double(serializer, &self->t_oc);
    
    extract_double(serializer, &self->af0);
    
    extract_double(serializer, &self->af1);
    
    extract_double(serializer, &self->af2);
    
    extract_double(serializer, &self->t_gd);
    
    extract_double(serializer, &self->ISC_L1CA);
    
    extract_double(serializer, &self->ISC_L2C);
    
    extract_double(serializer, &self->t_oe);
    
    extract_double(serializer, &self->a);
    
    extract_double(serializer, &self->a_dot);
    
    extract_double(serializer, &self->mean_anomaly);
    
    extract_double(serializer, &self->delta_mean_motion);
    
    extract_double(serializer, &self->delta_mean_motion_dot);
    
    extract_double(serializer, &self->eccentricity);
    
    extract_double(serializer, &self->argument_of_perigee);
    
    extract_double(serializer, &self->omega);
    
    extract_double(serializer, &self->omega_dot);
    
    extract_double(serializer, &self->inclination);
    
    extract_double(serializer, &self->inclination_dot);
    
    extract_double(serializer, &self->c_ic);
    
    extract_double(serializer, &self->c_is);
    
    extract_double(serializer, &self->c_uc);
    
    extract_double(serializer, &self->c_us);
    
    extract_double(serializer, &self->c_rc);
    
    extract_double(serializer, &self->c_rs);
    
    extract_mip_gnss_galileo_ephemeris_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_galileo_ephemeris_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_galileo_ephemeris_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_galileo_ephemeris_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_galileo_ephemeris_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_galileo_ephemeris_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_galileo_ephemeris_data_valid_flags(struct mip_serializer* serializer, mip_gnss_galileo_ephemeris_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_glo_ephemeris_data(mip_serializer* serializer, const mip_gnss_glo_ephemeris_data* self)
{
    insert_u8(serializer, self->index);
    
    insert_u8(serializer, self->count);
    
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    insert_u8(serializer, self->satellite_id);
    
    insert_s8(serializer, self->freq_number);
    
    insert_u32(serializer, self->tk);
    
    insert_u32(serializer, self->tb);
    
    insert_u8(serializer, self->sat_type);
    
    insert_double(serializer, self->gamma);
    
    insert_double(serializer, self->tau_n);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->x[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->v[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->a[i]);
    
    insert_u8(serializer, self->health);
    
    insert_u8(serializer, self->P);
    
    insert_u8(serializer, self->NT);
    
    insert_float(serializer, self->delta_tau_n);
    
    insert_u8(serializer, self->Ft);
    
    insert_u8(serializer, self->En);
    
    insert_u8(serializer, self->P1);
    
    insert_u8(serializer, self->P2);
    
    insert_u8(serializer, self->P3);
    
    insert_u8(serializer, self->P4);
    
    insert_mip_gnss_glo_ephemeris_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_glo_ephemeris_data(mip_serializer* serializer, mip_gnss_glo_ephemeris_data* self)
{
    extract_u8(serializer, &self->index);
    
    extract_u8(serializer, &self->count);
    
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    extract_u8(serializer, &self->satellite_id);
    
    extract_s8(serializer, &self->freq_number);
    
    extract_u32(serializer, &self->tk);
    
    extract_u32(serializer, &self->tb);
    
    extract_u8(serializer, &self->sat_type);
    
    extract_double(serializer, &self->gamma);
    
    extract_double(serializer, &self->tau_n);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->x[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->v[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->a[i]);
    
    extract_u8(serializer, &self->health);
    
    extract_u8(serializer, &self->P);
    
    extract_u8(serializer, &self->NT);
    
    extract_float(serializer, &self->delta_tau_n);
    
    extract_u8(serializer, &self->Ft);
    
    extract_u8(serializer, &self->En);
    
    extract_u8(serializer, &self->P1);
    
    extract_u8(serializer, &self->P2);
    
    extract_u8(serializer, &self->P3);
    
    extract_u8(serializer, &self->P4);
    
    extract_mip_gnss_glo_ephemeris_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_glo_ephemeris_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_glo_ephemeris_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_glo_ephemeris_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_glo_ephemeris_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_glo_ephemeris_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_glo_ephemeris_data_valid_flags(struct mip_serializer* serializer, mip_gnss_glo_ephemeris_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_gps_iono_corr_data(mip_serializer* serializer, const mip_gnss_gps_iono_corr_data* self)
{
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    for(unsigned int i=0; i < 4; i++)
        insert_double(serializer, self->alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        insert_double(serializer, self->beta[i]);
    
    insert_mip_gnss_gps_iono_corr_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_gps_iono_corr_data(mip_serializer* serializer, mip_gnss_gps_iono_corr_data* self)
{
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    for(unsigned int i=0; i < 4; i++)
        extract_double(serializer, &self->alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        extract_double(serializer, &self->beta[i]);
    
    extract_mip_gnss_gps_iono_corr_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_gps_iono_corr_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_gps_iono_corr_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_gps_iono_corr_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_gps_iono_corr_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_gps_iono_corr_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_gps_iono_corr_data_valid_flags(struct mip_serializer* serializer, mip_gnss_gps_iono_corr_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_gnss_galileo_iono_corr_data(mip_serializer* serializer, const mip_gnss_galileo_iono_corr_data* self)
{
    insert_double(serializer, self->time_of_week);
    
    insert_u16(serializer, self->week_number);
    
    for(unsigned int i=0; i < 3; i++)
        insert_double(serializer, self->alpha[i]);
    
    insert_u8(serializer, self->disturbance_flags);
    
    insert_mip_gnss_galileo_iono_corr_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_gnss_galileo_iono_corr_data(mip_serializer* serializer, mip_gnss_galileo_iono_corr_data* self)
{
    extract_double(serializer, &self->time_of_week);
    
    extract_u16(serializer, &self->week_number);
    
    for(unsigned int i=0; i < 3; i++)
        extract_double(serializer, &self->alpha[i]);
    
    extract_u8(serializer, &self->disturbance_flags);
    
    extract_mip_gnss_galileo_iono_corr_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_gnss_galileo_iono_corr_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_gnss_galileo_iono_corr_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_gnss_galileo_iono_corr_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_gnss_galileo_iono_corr_data_valid_flags(struct mip_serializer* serializer, const mip_gnss_galileo_iono_corr_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_gnss_galileo_iono_corr_data_valid_flags(struct mip_serializer* serializer, mip_gnss_galileo_iono_corr_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

