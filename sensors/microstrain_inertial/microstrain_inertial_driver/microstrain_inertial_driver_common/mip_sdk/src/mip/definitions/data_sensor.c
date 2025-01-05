
#include "data_sensor.h"

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


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert_mip_sensor_raw_accel_data(mip_serializer* serializer, const mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->raw_accel[i]);
    
}
void extract_mip_sensor_raw_accel_data(mip_serializer* serializer, mip_sensor_raw_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->raw_accel[i]);
    
}
bool extract_mip_sensor_raw_accel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_accel_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_accel_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_gyro_data(mip_serializer* serializer, const mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->raw_gyro[i]);
    
}
void extract_mip_sensor_raw_gyro_data(mip_serializer* serializer, mip_sensor_raw_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->raw_gyro[i]);
    
}
bool extract_mip_sensor_raw_gyro_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_gyro_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_gyro_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_mag_data(mip_serializer* serializer, const mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->raw_mag[i]);
    
}
void extract_mip_sensor_raw_mag_data(mip_serializer* serializer, mip_sensor_raw_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->raw_mag[i]);
    
}
bool extract_mip_sensor_raw_mag_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_mag_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_mag_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_raw_pressure_data(mip_serializer* serializer, const mip_sensor_raw_pressure_data* self)
{
    insert_float(serializer, self->raw_pressure);
    
}
void extract_mip_sensor_raw_pressure_data(mip_serializer* serializer, mip_sensor_raw_pressure_data* self)
{
    extract_float(serializer, &self->raw_pressure);
    
}
bool extract_mip_sensor_raw_pressure_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_raw_pressure_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_raw_pressure_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_accel_data(mip_serializer* serializer, const mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->scaled_accel[i]);
    
}
void extract_mip_sensor_scaled_accel_data(mip_serializer* serializer, mip_sensor_scaled_accel_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->scaled_accel[i]);
    
}
bool extract_mip_sensor_scaled_accel_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_accel_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_accel_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_gyro_data(mip_serializer* serializer, const mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->scaled_gyro[i]);
    
}
void extract_mip_sensor_scaled_gyro_data(mip_serializer* serializer, mip_sensor_scaled_gyro_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->scaled_gyro[i]);
    
}
bool extract_mip_sensor_scaled_gyro_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_gyro_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_gyro_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_mag_data(mip_serializer* serializer, const mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->scaled_mag[i]);
    
}
void extract_mip_sensor_scaled_mag_data(mip_serializer* serializer, mip_sensor_scaled_mag_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->scaled_mag[i]);
    
}
bool extract_mip_sensor_scaled_mag_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_mag_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_mag_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_scaled_pressure_data(mip_serializer* serializer, const mip_sensor_scaled_pressure_data* self)
{
    insert_float(serializer, self->scaled_pressure);
    
}
void extract_mip_sensor_scaled_pressure_data(mip_serializer* serializer, mip_sensor_scaled_pressure_data* self)
{
    extract_float(serializer, &self->scaled_pressure);
    
}
bool extract_mip_sensor_scaled_pressure_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_scaled_pressure_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_scaled_pressure_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_delta_theta_data(mip_serializer* serializer, const mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->delta_theta[i]);
    
}
void extract_mip_sensor_delta_theta_data(mip_serializer* serializer, mip_sensor_delta_theta_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->delta_theta[i]);
    
}
bool extract_mip_sensor_delta_theta_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_delta_theta_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_delta_theta_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_delta_velocity_data(mip_serializer* serializer, const mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->delta_velocity[i]);
    
}
void extract_mip_sensor_delta_velocity_data(mip_serializer* serializer, mip_sensor_delta_velocity_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->delta_velocity[i]);
    
}
bool extract_mip_sensor_delta_velocity_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_delta_velocity_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_delta_velocity_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_orientation_matrix_data(mip_serializer* serializer, const mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->m[i]);
    
}
void extract_mip_sensor_comp_orientation_matrix_data(mip_serializer* serializer, mip_sensor_comp_orientation_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->m[i]);
    
}
bool extract_mip_sensor_comp_orientation_matrix_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_orientation_matrix_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_orientation_matrix_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_quaternion_data(mip_serializer* serializer, const mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        insert_float(serializer, self->q[i]);
    
}
void extract_mip_sensor_comp_quaternion_data(mip_serializer* serializer, mip_sensor_comp_quaternion_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        extract_float(serializer, &self->q[i]);
    
}
bool extract_mip_sensor_comp_quaternion_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_quaternion_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_quaternion_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_euler_angles_data(mip_serializer* serializer, const mip_sensor_comp_euler_angles_data* self)
{
    insert_float(serializer, self->roll);
    
    insert_float(serializer, self->pitch);
    
    insert_float(serializer, self->yaw);
    
}
void extract_mip_sensor_comp_euler_angles_data(mip_serializer* serializer, mip_sensor_comp_euler_angles_data* self)
{
    extract_float(serializer, &self->roll);
    
    extract_float(serializer, &self->pitch);
    
    extract_float(serializer, &self->yaw);
    
}
bool extract_mip_sensor_comp_euler_angles_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_euler_angles_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_euler_angles_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_comp_orientation_update_matrix_data(mip_serializer* serializer, const mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        insert_float(serializer, self->m[i]);
    
}
void extract_mip_sensor_comp_orientation_update_matrix_data(mip_serializer* serializer, mip_sensor_comp_orientation_update_matrix_data* self)
{
    for(unsigned int i=0; i < 9; i++)
        extract_float(serializer, &self->m[i]);
    
}
bool extract_mip_sensor_comp_orientation_update_matrix_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_comp_orientation_update_matrix_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_comp_orientation_update_matrix_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_orientation_raw_temp_data(mip_serializer* serializer, const mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        insert_u16(serializer, self->raw_temp[i]);
    
}
void extract_mip_sensor_orientation_raw_temp_data(mip_serializer* serializer, mip_sensor_orientation_raw_temp_data* self)
{
    for(unsigned int i=0; i < 4; i++)
        extract_u16(serializer, &self->raw_temp[i]);
    
}
bool extract_mip_sensor_orientation_raw_temp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_orientation_raw_temp_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_orientation_raw_temp_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_internal_timestamp_data(mip_serializer* serializer, const mip_sensor_internal_timestamp_data* self)
{
    insert_u32(serializer, self->counts);
    
}
void extract_mip_sensor_internal_timestamp_data(mip_serializer* serializer, mip_sensor_internal_timestamp_data* self)
{
    extract_u32(serializer, &self->counts);
    
}
bool extract_mip_sensor_internal_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_internal_timestamp_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_internal_timestamp_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_pps_timestamp_data(mip_serializer* serializer, const mip_sensor_pps_timestamp_data* self)
{
    insert_u32(serializer, self->seconds);
    
    insert_u32(serializer, self->useconds);
    
}
void extract_mip_sensor_pps_timestamp_data(mip_serializer* serializer, mip_sensor_pps_timestamp_data* self)
{
    extract_u32(serializer, &self->seconds);
    
    extract_u32(serializer, &self->useconds);
    
}
bool extract_mip_sensor_pps_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_pps_timestamp_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_pps_timestamp_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_gps_timestamp_data(mip_serializer* serializer, const mip_sensor_gps_timestamp_data* self)
{
    insert_double(serializer, self->tow);
    
    insert_u16(serializer, self->week_number);
    
    insert_mip_sensor_gps_timestamp_data_valid_flags(serializer, self->valid_flags);
    
}
void extract_mip_sensor_gps_timestamp_data(mip_serializer* serializer, mip_sensor_gps_timestamp_data* self)
{
    extract_double(serializer, &self->tow);
    
    extract_u16(serializer, &self->week_number);
    
    extract_mip_sensor_gps_timestamp_data_valid_flags(serializer, &self->valid_flags);
    
}
bool extract_mip_sensor_gps_timestamp_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_gps_timestamp_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_gps_timestamp_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, const mip_sensor_gps_timestamp_data_valid_flags self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_sensor_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, mip_sensor_gps_timestamp_data_valid_flags* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_sensor_temperature_abs_data(mip_serializer* serializer, const mip_sensor_temperature_abs_data* self)
{
    insert_float(serializer, self->min_temp);
    
    insert_float(serializer, self->max_temp);
    
    insert_float(serializer, self->mean_temp);
    
}
void extract_mip_sensor_temperature_abs_data(mip_serializer* serializer, mip_sensor_temperature_abs_data* self)
{
    extract_float(serializer, &self->min_temp);
    
    extract_float(serializer, &self->max_temp);
    
    extract_float(serializer, &self->mean_temp);
    
}
bool extract_mip_sensor_temperature_abs_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_temperature_abs_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_temperature_abs_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_up_vector_data(mip_serializer* serializer, const mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->up[i]);
    
}
void extract_mip_sensor_up_vector_data(mip_serializer* serializer, mip_sensor_up_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->up[i]);
    
}
bool extract_mip_sensor_up_vector_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_up_vector_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_up_vector_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_north_vector_data(mip_serializer* serializer, const mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        insert_float(serializer, self->north[i]);
    
}
void extract_mip_sensor_north_vector_data(mip_serializer* serializer, mip_sensor_north_vector_data* self)
{
    for(unsigned int i=0; i < 3; i++)
        extract_float(serializer, &self->north[i]);
    
}
bool extract_mip_sensor_north_vector_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_north_vector_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_north_vector_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_overrange_status_data(mip_serializer* serializer, const mip_sensor_overrange_status_data* self)
{
    insert_mip_sensor_overrange_status_data_status(serializer, self->status);
    
}
void extract_mip_sensor_overrange_status_data(mip_serializer* serializer, mip_sensor_overrange_status_data* self)
{
    extract_mip_sensor_overrange_status_data_status(serializer, &self->status);
    
}
bool extract_mip_sensor_overrange_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_overrange_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_overrange_status_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_sensor_overrange_status_data_status(struct mip_serializer* serializer, const mip_sensor_overrange_status_data_status self)
{
    insert_u16(serializer, (uint16_t)(self));
}
void extract_mip_sensor_overrange_status_data_status(struct mip_serializer* serializer, mip_sensor_overrange_status_data_status* self)
{
    uint16_t tmp = 0;
    extract_u16(serializer, &tmp);
    *self = tmp;
}

void insert_mip_sensor_odometer_data_data(mip_serializer* serializer, const mip_sensor_odometer_data_data* self)
{
    insert_float(serializer, self->speed);
    
    insert_float(serializer, self->uncertainty);
    
    insert_u16(serializer, self->valid_flags);
    
}
void extract_mip_sensor_odometer_data_data(mip_serializer* serializer, mip_sensor_odometer_data_data* self)
{
    extract_float(serializer, &self->speed);
    
    extract_float(serializer, &self->uncertainty);
    
    extract_u16(serializer, &self->valid_flags);
    
}
bool extract_mip_sensor_odometer_data_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_sensor_odometer_data_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_sensor_odometer_data_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

