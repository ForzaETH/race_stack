
#include "data_system.h"

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

void insert_mip_system_built_in_test_data(mip_serializer* serializer, const mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        insert_u8(serializer, self->result[i]);
    
}
void extract_mip_system_built_in_test_data(mip_serializer* serializer, mip_system_built_in_test_data* self)
{
    for(unsigned int i=0; i < 16; i++)
        extract_u8(serializer, &self->result[i]);
    
}
bool extract_mip_system_built_in_test_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_system_built_in_test_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_system_built_in_test_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_system_time_sync_status_data(mip_serializer* serializer, const mip_system_time_sync_status_data* self)
{
    insert_bool(serializer, self->time_sync);
    
    insert_u8(serializer, self->last_pps_rcvd);
    
}
void extract_mip_system_time_sync_status_data(mip_serializer* serializer, mip_system_time_sync_status_data* self)
{
    extract_bool(serializer, &self->time_sync);
    
    extract_u8(serializer, &self->last_pps_rcvd);
    
}
bool extract_mip_system_time_sync_status_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_system_time_sync_status_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_system_time_sync_status_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_system_gpio_state_data(mip_serializer* serializer, const mip_system_gpio_state_data* self)
{
    insert_u8(serializer, self->states);
    
}
void extract_mip_system_gpio_state_data(mip_serializer* serializer, mip_system_gpio_state_data* self)
{
    extract_u8(serializer, &self->states);
    
}
bool extract_mip_system_gpio_state_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_system_gpio_state_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_system_gpio_state_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}

void insert_mip_system_gpio_analog_value_data(mip_serializer* serializer, const mip_system_gpio_analog_value_data* self)
{
    insert_u8(serializer, self->gpio_id);
    
    insert_float(serializer, self->value);
    
}
void extract_mip_system_gpio_analog_value_data(mip_serializer* serializer, mip_system_gpio_analog_value_data* self)
{
    extract_u8(serializer, &self->gpio_id);
    
    extract_float(serializer, &self->value);
    
}
bool extract_mip_system_gpio_analog_value_data_from_field(const mip_field* field, void* ptr)
{
    assert(ptr);
    mip_system_gpio_analog_value_data* self = ptr;
    struct mip_serializer serializer;
    mip_serializer_init_from_field(&serializer, field);
    extract_mip_system_gpio_analog_value_data(&serializer, self);
    return mip_serializer_is_complete(&serializer);
}


#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

