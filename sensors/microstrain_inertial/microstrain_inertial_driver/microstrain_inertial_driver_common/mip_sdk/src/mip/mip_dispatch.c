
#include "mip_dispatch.h"

#include "mip_packet.h"
#include "mip_field.h"

#include <assert.h>


#ifdef __cplusplus
namespace mip {
namespace C {
#endif

////////////////////////////////////////////////////////////////////////////////
///@brief Type of dispatch callback.
///
///@internal
///
enum mip_dispatch_type
{
    MIP_DISPATCH_TYPE_PACKET_PRE  = 1,  ///<@private Packet callback, before field callbacks. Use handler._packet_callback.
    MIP_DISPATCH_TYPE_PACKET_POST = 2,  ///<@private Packet callback, after field callbacks. Use handler._packet_callback.
    MIP_DISPATCH_TYPE_FIELD       = 3,  ///<@private Field callback. Use handler._field_callback.
    MIP_DISPATCH_TYPE_EXTRACT     = 4,  ///<@private Extraction callback. Use handler._extract_callback.
};


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the dispatch handler with a packet callback.
///
/// Packets which match the descriptor set will cause the callback to be
/// executed.
///
///@param handler
///
///@param descriptor_set
///       The callback will only be invoked for fields belonging to this
///       descriptor set. Can be MIP_DISPATCH_ANY_DESCRIPTOR to match any packet,
///       or MIP_DISPATCH_ANY_DATA_SET to match only data packets.
///@param post_callback
///       If false, the callback is called before any field callbacks from the
///       same packet. If true, the callback is called after field callbacks.
///@param callback
///       The callback function.
///@param user_data
///       Any pointer the user wants to pass into the callback.
///
void mip_dispatch_handler_init_packet_handler(mip_dispatch_handler* handler, uint8_t descriptor_set, bool post_callback, mip_dispatch_packet_callback callback, void* user_data)
{
    handler->_next             = NULL;
    handler->_packet_callback  = callback;
    handler->_user_data        = user_data;
    handler->_type             = post_callback ? MIP_DISPATCH_TYPE_PACKET_POST : MIP_DISPATCH_TYPE_PACKET_PRE;
    handler->_descriptor_set   = descriptor_set;
    handler->_field_descriptor = MIP_INVALID_FIELD_DESCRIPTOR;
    handler->_enabled          = true;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the dispatch handler with a field callback.
///
/// Fields which match both the descriptor set and field descriptor will cause
/// the callback function to be executed.
///
///@param handler
///
///@param descriptor_set
///       The callback will only be invoked for fields belonging to this
///       descriptor set. Can be MIP_DISPATCH_ANY_DESCRIPTOR to match any
///       packet, or MIP_DISPATCH_ANY_DATA_SET to match only data packets.
///
///@param field_descriptor
///       The callback will only be invoked for fields of this field descriptor.
///       It can be MIP_DISPATCH_ANY_DESCRIPTOR to select all field descriptors.
///
///@param callback
///       The callback function.
///
///@param user_data
///       Any pointer the user wants to pass into the callback.
///
void mip_dispatch_handler_init_field_handler(mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_field_callback callback, void* user_data)
{
    handler->_next             = NULL;
    handler->_field_callback   = callback;
    handler->_user_data        = user_data;
    handler->_type             = MIP_DISPATCH_TYPE_FIELD;

    handler->_descriptor_set   = descriptor_set;
    handler->_field_descriptor = field_descriptor;
    handler->_enabled          = true;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Initialize the dispatch handler with an extraction callback.
///
/// Use this function to automatically populate your data structures with data
/// as it arrives. This avoids the need to implement a switch/case or lots of
/// duplicated code to handle extraction of various data quantities.
///
///@param handler
///
///@param descriptor_set
///       The callback will only be invoked for fields belonging to this
///       descriptor set. It must match the descriptor set corresponding to the
///       extract function and object pointed to by field_ptr.
///       This may be MIP_DISPATCH_ANY_DATA_SET if the field descriptor is from
///       the shared data region.
///
///@param field_descriptor
///       The callback will only be invoked for fields of this field descriptor.
///       It must match the field descriptor corresponding to the extract
///       function and object pointed to by field_ptr.
///
///@param extractor
///       The extraction callback function. This is one of the functions in
///       the definitions/data_*.h files with the name `extract_<field-type>_from_field`.
///
///@param field_ptr
///       A pointer to the data structure corresponding to the field type
///       handled by the callback function. Cannot be NULL.
///
///@warning The type of field_ptr must match what the callback function expects.
///         Otherwise, the behavior is undefined and your program may experience
///         memory corruption or process crash.
///
void mip_dispatch_handler_init_extractor(mip_dispatch_handler* handler, uint8_t descriptor_set, uint8_t field_descriptor, mip_dispatch_extractor extractor, void* field_ptr)
{
    // The only wildcard allowed is MIP_DISPATCH_ANY_DATA_SET when field_descriptor is from the shared data set.
    assert(descriptor_set != MIP_DISPATCH_ANY_DESCRIPTOR);
    assert(field_descriptor != MIP_DISPATCH_ANY_DESCRIPTOR && field_descriptor != MIP_DISPATCH_ANY_DATA_SET);
    assert(!mip_is_data_descriptor_set(descriptor_set) || (descriptor_set != MIP_DISPATCH_ANY_DATA_SET) || mip_is_shared_data_field_descriptor(field_descriptor));
    assert(field_ptr);

    handler->_next             = NULL;
    handler->_extract_callback = extractor;
    handler->_user_data        = field_ptr;

    handler->_type             = MIP_DISPATCH_TYPE_EXTRACT;
    handler->_descriptor_set   = descriptor_set;
    handler->_field_descriptor = field_descriptor;
    handler->_enabled          = true;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Enables or disables the handler.
///
///@param handler
///@param enable If true, the callback is enabled. If false, it will not be called.
///
void mip_dispatch_handler_set_enabled(mip_dispatch_handler* handler, bool enable)
{
    handler->_enabled = enable;
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the handler is currently enabled.
///
///@returns true if the handler is enabled, false otherwise.
///
bool mip_dispatch_handler_is_enabled(mip_dispatch_handler* handler)
{
    return handler->_enabled;
}

////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
///@brief Initializes the mip_dispatcher object.
///
void mip_dispatcher_init(mip_dispatcher* self)
{
    self->_first_handler = NULL;
}


////////////////////////////////////////////////////////////////////////////////
///@brief Registers a handler in the dispatch system.
///
/// This is necessary for the handler function to be executed.
///
void mip_dispatcher_add_handler(mip_dispatcher* self, mip_dispatch_handler* handler)
{
    if( self->_first_handler == NULL )
        self->_first_handler = handler;
    else
    {
        mip_dispatch_handler* last = self->_first_handler;

        while(last->_next != NULL)
            last = last->_next;

        last->_next = handler;
    }
}


////////////////////////////////////////////////////////////////////////////////
///@brief Removes a handler from the dispatch system.
///
/// This will prevent the handler from being executed.
///
void mip_dispatcher_remove_handler(mip_dispatcher* self, mip_dispatch_handler* handler)
{
    if( self->_first_handler == NULL )
        return;

    mip_dispatch_handler* query = self->_first_handler;

    if( query == handler )
    {
        self->_first_handler = handler->_next;
        handler->_next = NULL;
        return;
    }

    while(query->_next != NULL)
    {
        if( query->_next == handler )
        {
            query->_next = handler->_next;
            handler->_next = NULL;
            return;
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
///@brief Removes all handlers from the dispatcher.
///
void mip_dispatcher_remove_all_handlers(mip_dispatcher* self)
{
    mip_dispatch_handler* query = self->_first_handler;

    self->_first_handler = NULL;

    // Break the chain (technically not necessary, but aids debugging)
    while(query)
    {
        mip_dispatch_handler* next = query->_next;
        query->_next = NULL;
        query = next;
    }
}

static bool mip_dispatch_is_descriptor_set_match(uint8_t desc_set, uint8_t handler_desc_set)
{
    return (
        (handler_desc_set == desc_set) ||
        (handler_desc_set == MIP_DISPATCH_ANY_DESCRIPTOR) ||
        ((handler_desc_set == MIP_DISPATCH_ANY_DATA_SET) && mip_is_data_descriptor_set(desc_set))
    );
}

////////////////////////////////////////////////////////////////////////////////
///@brief Called to dispatch packet callback before and after field iteration.
///@internal
///
///@param self
///@param packet    Valid MIP packet.
///@param timestamp Packet parse time.
///@param post      If true, this is called after field iteration. Otherwise before.
///
static void mip_dispatcher_call_packet_callbacks(mip_dispatcher* self, const mip_packet* packet, mip_timestamp timestamp, bool post)
{
    const uint8_t descriptor_set = mip_packet_descriptor_set(packet);

    // Iterate all packet handlers for this packet.
    for(mip_dispatch_handler* handler = self->_first_handler; handler != NULL; handler = handler->_next)
    {
        switch(handler->_type)
        {
        case MIP_DISPATCH_TYPE_PACKET_POST: if(!post) continue; else break;
        case MIP_DISPATCH_TYPE_PACKET_PRE:  if( post) continue; else break;
        default: continue;
        }

        if( mip_dispatch_is_descriptor_set_match(descriptor_set, handler->_descriptor_set) )
            handler->_packet_callback(handler->_user_data, packet, timestamp);
    }
}

////////////////////////////////////////////////////////////////////////////////
///@brief Determines if the field matches the dispatcher.
///
///@param desc_set           Packet descriptor set.
///@param field_desc         Field descriptor.
///@param handler_desc_set   Handler descriptor set filter.
///@param handler_field_desc Handler field descriptor filter.
///
///@returns true if the field matches.
///
static bool mip_dispatch_is_descriptor_match(uint8_t desc_set, uint8_t field_desc, uint8_t handler_desc_set, uint8_t handler_field_desc)
{
    return mip_dispatch_is_descriptor_set_match(desc_set, handler_desc_set) && (
        (handler_field_desc == field_desc) ||
        (handler_field_desc == MIP_DISPATCH_ANY_DESCRIPTOR)
    );
}

////////////////////////////////////////////////////////////////////////////////
///@brief Called to dispatch packet callback before and after field iteration.
///@internal
///
///@param self
///@param field     Valid MIP field.
///@param timestamp Packet parse time.
///
static void mip_dispatcher_call_field_callbacks(mip_dispatcher* self, const mip_field* field, mip_timestamp timestamp)
{
    const uint8_t descriptor_set   = mip_field_descriptor_set(field);
    const uint8_t field_descriptor = mip_field_field_descriptor(field);

    // Iterate all field handlers for this field.
    for(mip_dispatch_handler* handler = self->_first_handler; handler != NULL; handler = handler->_next)
    {
        if(handler->_type == MIP_DISPATCH_TYPE_FIELD)
        {
            if( mip_dispatch_is_descriptor_match(descriptor_set, field_descriptor, handler->_descriptor_set, handler->_field_descriptor) )
            {
                handler->_field_callback(handler->_user_data, field, timestamp);
            }
        }
        else if(handler->_type == MIP_DISPATCH_TYPE_EXTRACT)
        {
            if( handler->_descriptor_set == descriptor_set && handler->_field_descriptor == field_descriptor )
            {
                handler->_extract_callback(field, handler->_user_data);
            }
        }
    };
}

////////////////////////////////////////////////////////////////////////////////
///@brief Called to dispatch the callbacks for a given packet.
///
///@param self
///
///@param packet
///       The MIP packet to dispatch.
///
///@param timestamp
///        The approximate parse time of the packet.
///
void mip_dispatcher_dispatch_packet(mip_dispatcher* self, const mip_packet* packet, mip_timestamp timestamp)
{
    mip_dispatcher_call_packet_callbacks(self, packet, timestamp, false);

    mip_field field;
    mip_field_init_empty(&field);
    while( mip_field_next_in_packet(&field, packet) )
    {
        mip_dispatcher_call_field_callbacks(self, &field, timestamp);
    }

    mip_dispatcher_call_packet_callbacks(self, packet, timestamp, true);
}

#ifdef __cplusplus
} // namespace C
} // namespace mip
#endif
