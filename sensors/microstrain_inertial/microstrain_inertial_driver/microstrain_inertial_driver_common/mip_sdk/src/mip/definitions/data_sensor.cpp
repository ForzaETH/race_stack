
#include "data_sensor.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_sensor {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const RawAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_accel[i]);
    
}
void extract(Serializer& serializer, RawAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_accel[i]);
    
}

void insert(Serializer& serializer, const RawGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_gyro[i]);
    
}
void extract(Serializer& serializer, RawGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_gyro[i]);
    
}

void insert(Serializer& serializer, const RawMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.raw_mag[i]);
    
}
void extract(Serializer& serializer, RawMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.raw_mag[i]);
    
}

void insert(Serializer& serializer, const RawPressure& self)
{
    insert(serializer, self.raw_pressure);
    
}
void extract(Serializer& serializer, RawPressure& self)
{
    extract(serializer, self.raw_pressure);
    
}

void insert(Serializer& serializer, const ScaledAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_accel[i]);
    
}
void extract(Serializer& serializer, ScaledAccel& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_accel[i]);
    
}

void insert(Serializer& serializer, const ScaledGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_gyro[i]);
    
}
void extract(Serializer& serializer, ScaledGyro& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_gyro[i]);
    
}

void insert(Serializer& serializer, const ScaledMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.scaled_mag[i]);
    
}
void extract(Serializer& serializer, ScaledMag& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.scaled_mag[i]);
    
}

void insert(Serializer& serializer, const ScaledPressure& self)
{
    insert(serializer, self.scaled_pressure);
    
}
void extract(Serializer& serializer, ScaledPressure& self)
{
    extract(serializer, self.scaled_pressure);
    
}

void insert(Serializer& serializer, const DeltaTheta& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.delta_theta[i]);
    
}
void extract(Serializer& serializer, DeltaTheta& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.delta_theta[i]);
    
}

void insert(Serializer& serializer, const DeltaVelocity& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.delta_velocity[i]);
    
}
void extract(Serializer& serializer, DeltaVelocity& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.delta_velocity[i]);
    
}

void insert(Serializer& serializer, const CompOrientationMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.m[i]);
    
}
void extract(Serializer& serializer, CompOrientationMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.m[i]);
    
}

void insert(Serializer& serializer, const CompQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.q[i]);
    
}
void extract(Serializer& serializer, CompQuaternion& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.q[i]);
    
}

void insert(Serializer& serializer, const CompEulerAngles& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.yaw);
    
}
void extract(Serializer& serializer, CompEulerAngles& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.yaw);
    
}

void insert(Serializer& serializer, const CompOrientationUpdateMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.m[i]);
    
}
void extract(Serializer& serializer, CompOrientationUpdateMatrix& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.m[i]);
    
}

void insert(Serializer& serializer, const OrientationRawTemp& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.raw_temp[i]);
    
}
void extract(Serializer& serializer, OrientationRawTemp& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.raw_temp[i]);
    
}

void insert(Serializer& serializer, const InternalTimestamp& self)
{
    insert(serializer, self.counts);
    
}
void extract(Serializer& serializer, InternalTimestamp& self)
{
    extract(serializer, self.counts);
    
}

void insert(Serializer& serializer, const PpsTimestamp& self)
{
    insert(serializer, self.seconds);
    
    insert(serializer, self.useconds);
    
}
void extract(Serializer& serializer, PpsTimestamp& self)
{
    extract(serializer, self.seconds);
    
    extract(serializer, self.useconds);
    
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

void insert(Serializer& serializer, const TemperatureAbs& self)
{
    insert(serializer, self.min_temp);
    
    insert(serializer, self.max_temp);
    
    insert(serializer, self.mean_temp);
    
}
void extract(Serializer& serializer, TemperatureAbs& self)
{
    extract(serializer, self.min_temp);
    
    extract(serializer, self.max_temp);
    
    extract(serializer, self.mean_temp);
    
}

void insert(Serializer& serializer, const UpVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.up[i]);
    
}
void extract(Serializer& serializer, UpVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.up[i]);
    
}

void insert(Serializer& serializer, const NorthVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.north[i]);
    
}
void extract(Serializer& serializer, NorthVector& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.north[i]);
    
}

void insert(Serializer& serializer, const OverrangeStatus& self)
{
    insert(serializer, self.status);
    
}
void extract(Serializer& serializer, OverrangeStatus& self)
{
    extract(serializer, self.status);
    
}

void insert(Serializer& serializer, const OdometerData& self)
{
    insert(serializer, self.speed);
    
    insert(serializer, self.uncertainty);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, OdometerData& self)
{
    extract(serializer, self.speed);
    
    extract(serializer, self.uncertainty);
    
    extract(serializer, self.valid_flags);
    
}


} // namespace data_sensor
} // namespace mip

