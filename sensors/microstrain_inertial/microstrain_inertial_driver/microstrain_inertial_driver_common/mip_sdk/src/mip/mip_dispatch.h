#pragma once

#include "mip_types.h"
#include "mip_field.h"
#include "mip_packet.h"
#include "definitions/descriptors.h"

#include <stdbool.h>


#ifdef __cplusplus
namespace mip {
namespace C {
#endif


////////////////////////////////////////////////////////////////////////////////
///@addtogroup mip_c
///@{


////////////////////////////////////////////////////////////////////////////////
///@defgroup MipDispatch_c  Mip Dispatch [C]
///
///@brief System for issuing callbacks from MIP packets or fields.
///
///@{



////////////////////////////////////////////////////////////////////////////////
///@brief Signature for packet-level callbacks.
///
///@param context   User-supplied data pointer.
///@param packet    The MIP packet triggering this callback.
///@param timestamp The approximate parse time of the packet.
///
typedef void (*mip_dispatch_packet_callback)(void* context, const mip_packet* packet, mip_timestamp timestamp);

////////////////////////////////////////////////////////////////////////////////
///@brief Signature for field-level callbacks.
///
///@param context   User-supplied data pointer.
///@param field     The MIP field triggering this callback.
///@param timestamp The approximate parse time of the packet.
///
typedef void (*mip_dispatch_field_callback )(void* context, const mip_field* field, mip_timestamp timestamp);

////////////////////////////////////////////////////////////////////////////////
///@brief Signature for extraction callbacks.
///
///@param field A valid mip_field.
///@param ptr   A pointer to the destination field structure.
///
typedef bool (*mip_dispatch_extractor)(const mip_field* field, void* ptr);


enum {
    /// Wildcard descriptor set which only includes data packets.
    ///
    ///@note This is the same value as shared data descriptor set, but the shared
    /// data descriptor set is not real and will never be sent by the device.
    MIP_DISPATCH_ANY_DATA_SET = 0xFF,

    /// Any descriptor set or field descriptor.
    /// This is not a valid MIP descriptor.
    MIP_DISPATCH_ANY_DESCRIPTOR = 0x00,
};

////////////////////////////////////////////////////////////////////////////////
///@defgroup mip_dispatch_handler mip_dispatch_handler
///
/// This represents a binding between a MIP descriptor pair and a callback
/// function.
///
/// This object must be valid for the duration of its registration in a
/// mip_dispatcher. It cannot be reinitialized while registered.
///
///@note This should be considered an "opaque" structure; its members should be
/// considered an internal implementation detail. Avoid accessing them directly
/// as they are subject to change in future versions of this software.
///
///@{

///@brief Handler information for MIP Packet or Field callbacks.
///
typedef struct mip_dispatch_handler
{
    struct mip_dispatch_handler* _next;                  ///<@private Pointer to the next handler in the list.
    union
    {
        mip_dispatch_packet_callback  _packet_callback;  ///<@private User function for packets. Valid if _type is MIP_DISPATCH_TYPE_PACKET_*.
        mip_dispatch_field_callback   _field_callback;   ///<@private User callback for data fields. Valid if _type is MIP_DISPATCH_TYPE_FIELD.
        mip_dispatch_extractor        _extract_callback; ///<@private User callback for data fields. Valid if _type is MIP_DISPATCH_TYPE_EXTRACT.
    };
    void*   _user_data;                                 ///<@private User-provided pointer which is passed directly to the callback.
    uint8_t _type;                                      ///<@private Type of the callback. (Using u8 for better struct packing.) @see mip_dispatch_type
    uint8_t _descriptor_set;                            ///<@private MIP descriptor set for this callback.
    uint8_t _field_descriptor;                          ///<@private MIP field descriptor for this callback. If 0x00, the callback is a packet callback.
    uint8_t _enabled;                                   ///<@private If false, the handler will be ignored. (Using u8 for better struct packing.)
} mip_dispatch_handler;


void mip_dispatch_handler_init_packet_handler(mip_dispatch_handler* handler, uint8_t descriptor_set, bool after_fields, mip_dispatch_packet_callback callback, void* context);
void mip_dispatch_handler_init_field_handler(mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* context);
void mip_dispatch_handler_init_extractor(mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_extractor extractor, void* field_ptr);

void mip_dispatch_handler_set_enabled(mip_dispatch_handler* handler, bool enable);
bool mip_dispatch_handler_is_enabled(mip_dispatch_handler* handler);


///@}
////////////////////////////////////////////////////////////////////////////////
///@defgroup MipDispatchHandler mip_dispatch_handler - Represents a callback
///
///@{


///@brief Holds the state of the MIP dispatch system.
///
typedef struct mip_dispatcher
{
    mip_dispatch_handler* _first_handler;   ///<@private Pointer to the first dispatch handler. May be NULL.
} mip_dispatcher;


void mip_dispatcher_init(mip_dispatcher* self);
void mip_dispatcher_add_handler(mip_dispatcher* self, mip_dispatch_handler* handler);
void mip_dispatcher_remove_handler(mip_dispatcher* self, mip_dispatch_handler* handler);
void mip_dispatcher_remove_all_handlers(mip_dispatcher* self);

void mip_dispatcher_dispatch_packet(mip_dispatcher* self, const mip_packet* packet, mip_timestamp timestamp);

///@}
///@}
///@}
////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
} // namespace C
} // namespace mip
#endif
