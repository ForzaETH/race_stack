
#include "data_shared.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_shared {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const EventSource& self)
{
    insert(serializer, self.trigger_id);
    
}
void extract(Serializer& serializer, EventSource& self)
{
    extract(serializer, self.trigger_id);
    
}

void insert(Serializer& serializer, const Ticks& self)
{
    insert(serializer, self.ticks);
    
}
void extract(Serializer& serializer, Ticks& self)
{
    extract(serializer, self.ticks);
    
}

void insert(Serializer& serializer, const DeltaTicks& self)
{
    insert(serializer, self.ticks);
    
}
void extract(Serializer& serializer, DeltaTicks& self)
{
    extract(serializer, self.ticks);
    
}

void insert(Serializer& serializer, const GpsTimestamp& self)
{
    insert(serializer, self.tow);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GpsTimestamp& self)
{
    extract(serializer, self.tow);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const DeltaTime& self)
{
    insert(serializer, self.seconds);
    
}
void extract(Serializer& serializer, DeltaTime& self)
{
    extract(serializer, self.seconds);
    
}

void insert(Serializer& serializer, const ReferenceTimestamp& self)
{
    insert(serializer, self.nanoseconds);
    
}
void extract(Serializer& serializer, ReferenceTimestamp& self)
{
    extract(serializer, self.nanoseconds);
    
}

void insert(Serializer& serializer, const ReferenceTimeDelta& self)
{
    insert(serializer, self.dt_nanos);
    
}
void extract(Serializer& serializer, ReferenceTimeDelta& self)
{
    extract(serializer, self.dt_nanos);
    
}

void insert(Serializer& serializer, const ExternalTimestamp& self)
{
    insert(serializer, self.nanoseconds);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, ExternalTimestamp& self)
{
    extract(serializer, self.nanoseconds);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const ExternalTimeDelta& self)
{
    insert(serializer, self.dt_nanos);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, ExternalTimeDelta& self)
{
    extract(serializer, self.dt_nanos);
    
    extract(serializer, self.valid_flags);
    
}


} // namespace data_shared
} // namespace mip

