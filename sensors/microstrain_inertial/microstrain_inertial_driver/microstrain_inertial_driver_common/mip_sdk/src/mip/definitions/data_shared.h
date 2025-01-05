#pragma once

#include "common.h"
#include "descriptors.h"
#include "../mip_result.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
namespace mip {
namespace C {
extern "C" {

#endif // __cplusplus
struct mip_interface;
struct mip_serializer;
struct mip_field;

////////////////////////////////////////////////////////////////////////////////
///@addtogroup MipData_c  MIP Data [C]
///@{
///@defgroup shared_data_c  Shared Data [C]
///
///@{

////////////////////////////////////////////////////////////////////////////////
// Descriptors
////////////////////////////////////////////////////////////////////////////////

enum 
{
    MIP_SHARED_DATA_DESC_SET            = 0xFF,
    
    
    MIP_DATA_DESC_SHARED_EVENT_SOURCE   = 0xD0,
    MIP_DATA_DESC_SHARED_TICKS          = 0xD1,
    MIP_DATA_DESC_SHARED_DELTA_TICKS    = 0xD2,
    MIP_DATA_DESC_SHARED_GPS_TIME       = 0xD3,
    MIP_DATA_DESC_SHARED_DELTA_TIME     = 0xD4,
    MIP_DATA_DESC_SHARED_REFERENCE_TIME = 0xD5,
    MIP_DATA_DESC_SHARED_REF_TIME_DELTA = 0xD6,
    MIP_DATA_DESC_SHARED_EXTERNAL_TIME  = 0xD7,
    MIP_DATA_DESC_SHARED_SYS_TIME_DELTA = 0xD8,
    MIP_DATA_DESC_SHARED_DEBUG_TICKS    = 0xFF,
};

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

enum { MIP_DATA_DESC_SHARED_START = 0xD0 };

////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_event_source  (0xFF,0xD0) Event Source [C]
/// Identifies which event trigger caused this packet to be emitted.
/// 
/// Generally this is used to determine whether a packet was emitted
/// due to scheduled streaming or due to an event.
///
///@{

struct mip_shared_event_source_data
{
    uint8_t trigger_id; ///< Trigger ID number. If 0, this message was emitted due to being scheduled in the 3DM Message Format Command (0x0C,0x0F).
    
};
typedef struct mip_shared_event_source_data mip_shared_event_source_data;
void insert_mip_shared_event_source_data(struct mip_serializer* serializer, const mip_shared_event_source_data* self);
void extract_mip_shared_event_source_data(struct mip_serializer* serializer, mip_shared_event_source_data* self);
bool extract_mip_shared_event_source_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_ticks  (0xFF,0xD1) Ticks [C]
/// Time since powerup in multiples of the base rate.
/// 
/// The counter will wrap around to 0 after approximately 50 days.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct mip_shared_ticks_data
{
    uint32_t ticks; ///< Ticks since powerup.
    
};
typedef struct mip_shared_ticks_data mip_shared_ticks_data;
void insert_mip_shared_ticks_data(struct mip_serializer* serializer, const mip_shared_ticks_data* self);
void extract_mip_shared_ticks_data(struct mip_serializer* serializer, mip_shared_ticks_data* self);
bool extract_mip_shared_ticks_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_delta_ticks  (0xFF,0xD2) Delta Ticks [C]
/// Ticks since the last output of this field.
/// 
/// This field can be used to track the amount of time passed between
/// event occurrences.
/// One tick is equivalent to one base period (reciprocal of the base rate).
///
///@{

struct mip_shared_delta_ticks_data
{
    uint32_t ticks; ///< Ticks since last output.
    
};
typedef struct mip_shared_delta_ticks_data mip_shared_delta_ticks_data;
void insert_mip_shared_delta_ticks_data(struct mip_serializer* serializer, const mip_shared_delta_ticks_data* self);
void extract_mip_shared_delta_ticks_data(struct mip_serializer* serializer, mip_shared_delta_ticks_data* self);
bool extract_mip_shared_delta_ticks_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_gps_timestamp  (0xFF,0xD3) Gps Timestamp [C]
/// Outputs the current GPS system time in time-of-week and week number format.
/// 
/// For events, this is the time of the event trigger.
/// In order to be valid, a PPS signal needs to be present, and both a valid GPS time-of-week and week number command (0x0C, 0x72) need to be received after PPS sync has been achieved.
///
///@{

typedef uint16_t mip_shared_gps_timestamp_data_valid_flags;
static const mip_shared_gps_timestamp_data_valid_flags MIP_SHARED_GPS_TIMESTAMP_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_shared_gps_timestamp_data_valid_flags MIP_SHARED_GPS_TIMESTAMP_DATA_VALID_FLAGS_TOW         = 0x0001; ///<  Whole number seconds TOW has been set
static const mip_shared_gps_timestamp_data_valid_flags MIP_SHARED_GPS_TIMESTAMP_DATA_VALID_FLAGS_WEEK_NUMBER = 0x0002; ///<  Week number has been set
static const mip_shared_gps_timestamp_data_valid_flags MIP_SHARED_GPS_TIMESTAMP_DATA_VALID_FLAGS_TIME_VALID  = 0x0003; ///<  Both TOW and Week Number have been set
static const mip_shared_gps_timestamp_data_valid_flags MIP_SHARED_GPS_TIMESTAMP_DATA_VALID_FLAGS_ALL         = 0x0003;

struct mip_shared_gps_timestamp_data
{
    double tow; ///< GPS Time of Week [seconds]
    uint16_t week_number; ///< GPS Week Number since 1980 [weeks]
    mip_shared_gps_timestamp_data_valid_flags valid_flags;
    
};
typedef struct mip_shared_gps_timestamp_data mip_shared_gps_timestamp_data;
void insert_mip_shared_gps_timestamp_data(struct mip_serializer* serializer, const mip_shared_gps_timestamp_data* self);
void extract_mip_shared_gps_timestamp_data(struct mip_serializer* serializer, mip_shared_gps_timestamp_data* self);
bool extract_mip_shared_gps_timestamp_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_shared_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, const mip_shared_gps_timestamp_data_valid_flags self);
void extract_mip_shared_gps_timestamp_data_valid_flags(struct mip_serializer* serializer, mip_shared_gps_timestamp_data_valid_flags* self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_delta_time  (0xFF,0xD4) Delta Time [C]
/// Time in the synchronized clock domain since the last output of this field within the same descriptor set and event instance.
/// 
/// This can be used to track the amount of time passed between
/// event occurrences. See the manual page on delta time quantities.
/// 
/// This field contains the same value as the delta external time field, 0xD8,
/// but is expressed in seconds. Transmission of either of these fields
/// restarts a shared counter, so only one should be streamed at a time to
/// avoid confusion. The counter is not shared across descriptors sets or
/// between event instances.
///
///@{

struct mip_shared_delta_time_data
{
    double seconds; ///< Seconds since last output.
    
};
typedef struct mip_shared_delta_time_data mip_shared_delta_time_data;
void insert_mip_shared_delta_time_data(struct mip_serializer* serializer, const mip_shared_delta_time_data* self);
void extract_mip_shared_delta_time_data(struct mip_serializer* serializer, mip_shared_delta_time_data* self);
bool extract_mip_shared_delta_time_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_reference_timestamp  (0xFF,0xD5) Reference Timestamp [C]
/// Internal reference timestamp.
/// 
/// This timestamp represents the time at which the corresponding
/// data was sampled, according to the internal reference clock.
/// 
/// This is a monotonic clock which never jumps. The value is always valid.
/// 
/// For events, this is the time of the event trigger.
///
///@{

struct mip_shared_reference_timestamp_data
{
    uint64_t nanoseconds; ///< Nanoseconds since initialization.
    
};
typedef struct mip_shared_reference_timestamp_data mip_shared_reference_timestamp_data;
void insert_mip_shared_reference_timestamp_data(struct mip_serializer* serializer, const mip_shared_reference_timestamp_data* self);
void extract_mip_shared_reference_timestamp_data(struct mip_serializer* serializer, mip_shared_reference_timestamp_data* self);
bool extract_mip_shared_reference_timestamp_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_reference_time_delta  (0xFF,0xD6) Reference Time Delta [C]
/// Delta time since the last packet.
/// 
/// Difference between the time as reported by the shared reference time field, 0xD5,
/// and the previous output of this delta quantity within the same descriptor set and event instance.
/// 
/// The delta is based on the reference time which never jumps. The value
/// is always valid.
/// 
/// This can be used to track the amount of time passed between
/// event occurrences. See the manual page on delta time quantities.
///
///@{

struct mip_shared_reference_time_delta_data
{
    uint64_t dt_nanos; ///< Nanoseconds since the last occurrence of this field in a packet of the same descriptor set and event source.
    
};
typedef struct mip_shared_reference_time_delta_data mip_shared_reference_time_delta_data;
void insert_mip_shared_reference_time_delta_data(struct mip_serializer* serializer, const mip_shared_reference_time_delta_data* self);
void extract_mip_shared_reference_time_delta_data(struct mip_serializer* serializer, mip_shared_reference_time_delta_data* self);
bool extract_mip_shared_reference_time_delta_data_from_field(const struct mip_field* field, void* ptr);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_external_timestamp  (0xFF,0xD7) External Timestamp [C]
/// External timestamp in nanoseconds.
/// 
/// This timestamp represents the time at which the corresponding
/// data was sampled in the external clock domain.
/// Equivalent to the GPS Timestamp but in nanoseconds.
/// 
/// For events, this is the time of the event trigger.
/// 
/// To be valid, external clock sync must be achieved using the PPS input.
///
///@{

typedef uint16_t mip_shared_external_timestamp_data_valid_flags;
static const mip_shared_external_timestamp_data_valid_flags MIP_SHARED_EXTERNAL_TIMESTAMP_DATA_VALID_FLAGS_NONE        = 0x0000;
static const mip_shared_external_timestamp_data_valid_flags MIP_SHARED_EXTERNAL_TIMESTAMP_DATA_VALID_FLAGS_NANOSECONDS = 0x0001; ///<  
static const mip_shared_external_timestamp_data_valid_flags MIP_SHARED_EXTERNAL_TIMESTAMP_DATA_VALID_FLAGS_ALL         = 0x0001;

struct mip_shared_external_timestamp_data
{
    uint64_t nanoseconds;
    mip_shared_external_timestamp_data_valid_flags valid_flags;
    
};
typedef struct mip_shared_external_timestamp_data mip_shared_external_timestamp_data;
void insert_mip_shared_external_timestamp_data(struct mip_serializer* serializer, const mip_shared_external_timestamp_data* self);
void extract_mip_shared_external_timestamp_data(struct mip_serializer* serializer, mip_shared_external_timestamp_data* self);
bool extract_mip_shared_external_timestamp_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_shared_external_timestamp_data_valid_flags(struct mip_serializer* serializer, const mip_shared_external_timestamp_data_valid_flags self);
void extract_mip_shared_external_timestamp_data_valid_flags(struct mip_serializer* serializer, mip_shared_external_timestamp_data_valid_flags* self);


///@}
///
////////////////////////////////////////////////////////////////////////////////
///@defgroup c_shared_external_time_delta  (0xFF,0xD8) External Time Delta [C]
/// Delta time since the last packet containing delta external (0xFF,0xD4) or delta gps time (0xFF,0xD8).
/// 
/// Difference between the time as reported by the shared external time field, 0xD7,
/// and the previous output of this delta quantity within the same descriptor set and event instance.
/// 
/// This can be used to track the amount of time passed between
/// event occurrences. See the manual page on delta time quantities.
/// 
/// This field contains the same value as the delta gps time field, 0xD4,
/// but is expressed in nanoseconds. Transmission of either of these fields
/// restarts a shared counter, so only one should be streamed at a time to
/// avoid confusion. The counter is not shared across descriptors sets or
/// between event instances.
///
///@{

typedef uint16_t mip_shared_external_time_delta_data_valid_flags;
static const mip_shared_external_time_delta_data_valid_flags MIP_SHARED_EXTERNAL_TIME_DELTA_DATA_VALID_FLAGS_NONE     = 0x0000;
static const mip_shared_external_time_delta_data_valid_flags MIP_SHARED_EXTERNAL_TIME_DELTA_DATA_VALID_FLAGS_DT_NANOS = 0x0001; ///<  
static const mip_shared_external_time_delta_data_valid_flags MIP_SHARED_EXTERNAL_TIME_DELTA_DATA_VALID_FLAGS_ALL      = 0x0001;

struct mip_shared_external_time_delta_data
{
    uint64_t dt_nanos; ///< Nanoseconds since the last occurrence of this field in a packet of the same descriptor set and event source.
    mip_shared_external_time_delta_data_valid_flags valid_flags;
    
};
typedef struct mip_shared_external_time_delta_data mip_shared_external_time_delta_data;
void insert_mip_shared_external_time_delta_data(struct mip_serializer* serializer, const mip_shared_external_time_delta_data* self);
void extract_mip_shared_external_time_delta_data(struct mip_serializer* serializer, mip_shared_external_time_delta_data* self);
bool extract_mip_shared_external_time_delta_data_from_field(const struct mip_field* field, void* ptr);

void insert_mip_shared_external_time_delta_data_valid_flags(struct mip_serializer* serializer, const mip_shared_external_time_delta_data_valid_flags self);
void extract_mip_shared_external_time_delta_data_valid_flags(struct mip_serializer* serializer, mip_shared_external_time_delta_data_valid_flags* self);


///@}
///

///@}
///@}
///
////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
} // namespace C
} // namespace mip
} // extern "C"
#endif // __cplusplus

