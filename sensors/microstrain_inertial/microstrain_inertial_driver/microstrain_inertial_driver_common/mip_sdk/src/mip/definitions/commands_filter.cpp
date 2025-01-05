
#include "commands_filter.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_filter {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const Reset& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, Reset& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<Reset> reset(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RESET_FILTER, NULL, 0);
}
void insert(Serializer& serializer, const SetInitialAttitude& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.heading);
    
}
void extract(Serializer& serializer, SetInitialAttitude& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.heading);
    
}

TypedResult<SetInitialAttitude> setInitialAttitude(C::mip_interface& device, float roll, float pitch, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, roll);
    
    insert(serializer, pitch);
    
    insert(serializer, heading);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_ATTITUDE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const EstimationControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, EstimationControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const EstimationControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, EstimationControl::Response& self)
{
    extract(serializer, self.enable);
    
}

TypedResult<EstimationControl> writeEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EstimationControl> readEstimationControl(C::mip_interface& device, EstimationControl::EnableFlags* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EstimationControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ESTIMATION_CONTROL_FLAGS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EstimationControl> saveEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EstimationControl> loadEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EstimationControl> defaultEstimationControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ESTIMATION_CONTROL_FLAGS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ExternalGnssUpdate& self)
{
    insert(serializer, self.gps_time);
    
    insert(serializer, self.gps_week);
    
    insert(serializer, self.latitude);
    
    insert(serializer, self.longitude);
    
    insert(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.vel_uncertainty[i]);
    
}
void extract(Serializer& serializer, ExternalGnssUpdate& self)
{
    extract(serializer, self.gps_time);
    
    extract(serializer, self.gps_week);
    
    extract(serializer, self.latitude);
    
    extract(serializer, self.longitude);
    
    extract(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.pos_uncertainty[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.vel_uncertainty[i]);
    
}

TypedResult<ExternalGnssUpdate> externalGnssUpdate(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, double latitude, double longitude, double height, const float* velocity, const float* posUncertainty, const float* velUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, gpsTime);
    
    insert(serializer, gpsWeek);
    
    insert(serializer, latitude);
    
    insert(serializer, longitude);
    
    insert(serializer, height);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(posUncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, posUncertainty[i]);
    
    assert(velUncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velUncertainty[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_GNSS_UPDATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ExternalHeadingUpdate& self)
{
    insert(serializer, self.heading);
    
    insert(serializer, self.heading_uncertainty);
    
    insert(serializer, self.type);
    
}
void extract(Serializer& serializer, ExternalHeadingUpdate& self)
{
    extract(serializer, self.heading);
    
    extract(serializer, self.heading_uncertainty);
    
    extract(serializer, self.type);
    
}

TypedResult<ExternalHeadingUpdate> externalHeadingUpdate(C::mip_interface& device, float heading, float headingUncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, heading);
    
    insert(serializer, headingUncertainty);
    
    insert(serializer, type);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ExternalHeadingUpdateWithTime& self)
{
    insert(serializer, self.gps_time);
    
    insert(serializer, self.gps_week);
    
    insert(serializer, self.heading);
    
    insert(serializer, self.heading_uncertainty);
    
    insert(serializer, self.type);
    
}
void extract(Serializer& serializer, ExternalHeadingUpdateWithTime& self)
{
    extract(serializer, self.gps_time);
    
    extract(serializer, self.gps_week);
    
    extract(serializer, self.heading);
    
    extract(serializer, self.heading_uncertainty);
    
    extract(serializer, self.type);
    
}

TypedResult<ExternalHeadingUpdateWithTime> externalHeadingUpdateWithTime(C::mip_interface& device, double gpsTime, uint16_t gpsWeek, float heading, float headingUncertainty, uint8_t type)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, gpsTime);
    
    insert(serializer, gpsWeek);
    
    insert(serializer, heading);
    
    insert(serializer, headingUncertainty);
    
    insert(serializer, type);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_EXTERNAL_HEADING_UPDATE_WITH_TIME, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const TareOrientation& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.axes);
        
    }
}
void extract(Serializer& serializer, TareOrientation& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.axes);
        
    }
}

void insert(Serializer& serializer, const TareOrientation::Response& self)
{
    insert(serializer, self.axes);
    
}
void extract(Serializer& serializer, TareOrientation::Response& self)
{
    extract(serializer, self.axes);
    
}

TypedResult<TareOrientation> writeTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes axes)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, axes);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<TareOrientation> readTareOrientation(C::mip_interface& device, TareOrientation::MipTareAxes* axesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<TareOrientation> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_TARE_ORIENTATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(axesOut);
        extract(deserializer, *axesOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<TareOrientation> saveTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<TareOrientation> loadTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<TareOrientation> defaultTareOrientation(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_TARE_ORIENTATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VehicleDynamicsMode& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.mode);
        
    }
}
void extract(Serializer& serializer, VehicleDynamicsMode& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.mode);
        
    }
}

void insert(Serializer& serializer, const VehicleDynamicsMode::Response& self)
{
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, VehicleDynamicsMode::Response& self)
{
    extract(serializer, self.mode);
    
}

TypedResult<VehicleDynamicsMode> writeVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<VehicleDynamicsMode> readVehicleDynamicsMode(C::mip_interface& device, VehicleDynamicsMode::DynamicsMode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<VehicleDynamicsMode> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_VEHICLE_DYNAMICS_MODE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<VehicleDynamicsMode> saveVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<VehicleDynamicsMode> loadVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<VehicleDynamicsMode> defaultVehicleDynamicsMode(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_DYNAMICS_MODE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleRotationEuler& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.roll);
        
        insert(serializer, self.pitch);
        
        insert(serializer, self.yaw);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleRotationEuler& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.roll);
        
        extract(serializer, self.pitch);
        
        extract(serializer, self.yaw);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleRotationEuler::Response& self)
{
    insert(serializer, self.roll);
    
    insert(serializer, self.pitch);
    
    insert(serializer, self.yaw);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationEuler::Response& self)
{
    extract(serializer, self.roll);
    
    extract(serializer, self.pitch);
    
    extract(serializer, self.yaw);
    
}

TypedResult<SensorToVehicleRotationEuler> writeSensorToVehicleRotationEuler(C::mip_interface& device, float roll, float pitch, float yaw)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, roll);
    
    insert(serializer, pitch);
    
    insert(serializer, yaw);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationEuler> readSensorToVehicleRotationEuler(C::mip_interface& device, float* rollOut, float* pitchOut, float* yawOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleRotationEuler> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_ROTATION_EULER, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(rollOut);
        extract(deserializer, *rollOut);
        
        assert(pitchOut);
        extract(deserializer, *pitchOut);
        
        assert(yawOut);
        extract(deserializer, *yawOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleRotationEuler> saveSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationEuler> loadSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationEuler> defaultSensorToVehicleRotationEuler(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_EULER, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleRotationDcm& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            insert(serializer, self.dcm[i]);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleRotationDcm& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, self.dcm[i]);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleRotationDcm::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.dcm[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationDcm::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.dcm[i]);
    
}

TypedResult<SensorToVehicleRotationDcm> writeSensorToVehicleRotationDcm(C::mip_interface& device, const float* dcm)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(dcm || (9 == 0));
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, dcm[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationDcm> readSensorToVehicleRotationDcm(C::mip_interface& device, float* dcmOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleRotationDcm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_ROTATION_DCM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(dcmOut || (9 == 0));
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, dcmOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleRotationDcm> saveSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationDcm> loadSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationDcm> defaultSensorToVehicleRotationDcm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_DCM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            insert(serializer, self.quat[i]);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 4; i++)
            extract(serializer, self.quat[i]);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleRotationQuaternion::Response& self)
{
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.quat[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleRotationQuaternion::Response& self)
{
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.quat[i]);
    
}

TypedResult<SensorToVehicleRotationQuaternion> writeSensorToVehicleRotationQuaternion(C::mip_interface& device, const float* quat)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(quat || (4 == 0));
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, quat[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationQuaternion> readSensorToVehicleRotationQuaternion(C::mip_interface& device, float* quatOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleRotationQuaternion> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(quatOut || (4 == 0));
        for(unsigned int i=0; i < 4; i++)
            extract(deserializer, quatOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleRotationQuaternion> saveSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationQuaternion> loadSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleRotationQuaternion> defaultSensorToVehicleRotationQuaternion(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_ROTATION_QUATERNION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SensorToVehicleOffset& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.offset[i]);
        
    }
}
void extract(Serializer& serializer, SensorToVehicleOffset& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.offset[i]);
        
    }
}

void insert(Serializer& serializer, const SensorToVehicleOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, SensorToVehicleOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

TypedResult<SensorToVehicleOffset> writeSensorToVehicleOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleOffset> readSensorToVehicleOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SensorToVehicleOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SENSOR2VEHICLE_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SensorToVehicleOffset> saveSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleOffset> loadSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SensorToVehicleOffset> defaultSensorToVehicleOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SENSOR2VEHICLE_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AntennaOffset& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.offset[i]);
        
    }
}
void extract(Serializer& serializer, AntennaOffset& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.offset[i]);
        
    }
}

void insert(Serializer& serializer, const AntennaOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.offset[i]);
    
}
void extract(Serializer& serializer, AntennaOffset::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.offset[i]);
    
}

TypedResult<AntennaOffset> writeAntennaOffset(C::mip_interface& device, const float* offset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(offset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, offset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AntennaOffset> readAntennaOffset(C::mip_interface& device, float* offsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AntennaOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(offsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, offsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AntennaOffset> saveAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AntennaOffset> loadAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AntennaOffset> defaultAntennaOffset(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GnssSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
    }
}
void extract(Serializer& serializer, GnssSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
    }
}

void insert(Serializer& serializer, const GnssSource::Response& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, GnssSource::Response& self)
{
    extract(serializer, self.source);
    
}

TypedResult<GnssSource> writeGnssSource(C::mip_interface& device, GnssSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssSource> readGnssSource(C::mip_interface& device, GnssSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GNSS_SOURCE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssSource> saveGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssSource> loadGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssSource> defaultGnssSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GNSS_SOURCE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const HeadingSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
    }
}
void extract(Serializer& serializer, HeadingSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
    }
}

void insert(Serializer& serializer, const HeadingSource::Response& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, HeadingSource::Response& self)
{
    extract(serializer, self.source);
    
}

TypedResult<HeadingSource> writeHeadingSource(C::mip_interface& device, HeadingSource::Source source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<HeadingSource> readHeadingSource(C::mip_interface& device, HeadingSource::Source* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<HeadingSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_HEADING_UPDATE_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<HeadingSource> saveHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<HeadingSource> loadHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<HeadingSource> defaultHeadingSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_UPDATE_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AutoInitControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, AutoInitControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const AutoInitControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, AutoInitControl::Response& self)
{
    extract(serializer, self.enable);
    
}

TypedResult<AutoInitControl> writeAutoInitControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoInitControl> readAutoInitControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AutoInitControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_AUTOINIT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AutoInitControl> saveAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoInitControl> loadAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoInitControl> defaultAutoInitControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AUTOINIT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AccelNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, AccelNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const AccelNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, AccelNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<AccelNoise> writeAccelNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelNoise> readAccelNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ACCEL_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelNoise> saveAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelNoise> loadAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelNoise> defaultAccelNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GyroNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, GyroNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const GyroNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, GyroNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<GyroNoise> writeGyroNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroNoise> readGyroNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GyroNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GyroNoise> saveGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroNoise> loadGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroNoise> defaultGyroNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AccelBiasModel& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, AccelBiasModel& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const AccelBiasModel::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, AccelBiasModel::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<AccelBiasModel> writeAccelBiasModel(C::mip_interface& device, const float* beta, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(beta || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, beta[i]);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelBiasModel> readAccelBiasModel(C::mip_interface& device, float* betaOut, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelBiasModel> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ACCEL_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(betaOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, betaOut[i]);
        
        assert(noiseOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelBiasModel> saveAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelBiasModel> loadAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelBiasModel> defaultAccelBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GyroBiasModel& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, GyroBiasModel& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.beta[i]);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const GyroBiasModel::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, GyroBiasModel::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.beta[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<GyroBiasModel> writeGyroBiasModel(C::mip_interface& device, const float* beta, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(beta || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, beta[i]);
    
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroBiasModel> readGyroBiasModel(C::mip_interface& device, float* betaOut, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GyroBiasModel> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_BIAS_MODEL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(betaOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, betaOut[i]);
        
        assert(noiseOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GyroBiasModel> saveGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroBiasModel> loadGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GyroBiasModel> defaultGyroBiasModel(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_BIAS_MODEL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AltitudeAiding& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.selector);
        
    }
}
void extract(Serializer& serializer, AltitudeAiding& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.selector);
        
    }
}

void insert(Serializer& serializer, const AltitudeAiding::Response& self)
{
    insert(serializer, self.selector);
    
}
void extract(Serializer& serializer, AltitudeAiding::Response& self)
{
    extract(serializer, self.selector);
    
}

TypedResult<AltitudeAiding> writeAltitudeAiding(C::mip_interface& device, AltitudeAiding::AidingSelector selector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, selector);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AltitudeAiding> readAltitudeAiding(C::mip_interface& device, AltitudeAiding::AidingSelector* selectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AltitudeAiding> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ALTITUDE_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(selectorOut);
        extract(deserializer, *selectorOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AltitudeAiding> saveAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AltitudeAiding> loadAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AltitudeAiding> defaultAltitudeAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ALTITUDE_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const PitchRollAiding& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
    }
}
void extract(Serializer& serializer, PitchRollAiding& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
    }
}

void insert(Serializer& serializer, const PitchRollAiding::Response& self)
{
    insert(serializer, self.source);
    
}
void extract(Serializer& serializer, PitchRollAiding::Response& self)
{
    extract(serializer, self.source);
    
}

TypedResult<PitchRollAiding> writePitchRollAiding(C::mip_interface& device, PitchRollAiding::AidingSource source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PitchRollAiding> readPitchRollAiding(C::mip_interface& device, PitchRollAiding::AidingSource* sourceOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<PitchRollAiding> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<PitchRollAiding> savePitchRollAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PitchRollAiding> loadPitchRollAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PitchRollAiding> defaultPitchRollAiding(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SECONDARY_PITCH_ROLL_AIDING_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AutoZupt& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.threshold);
        
    }
}
void extract(Serializer& serializer, AutoZupt& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.threshold);
        
    }
}

void insert(Serializer& serializer, const AutoZupt::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.threshold);
    
}
void extract(Serializer& serializer, AutoZupt::Response& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.threshold);
    
}

TypedResult<AutoZupt> writeAutoZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, threshold);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoZupt> readAutoZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AutoZupt> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(thresholdOut);
        extract(deserializer, *thresholdOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AutoZupt> saveAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoZupt> loadAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoZupt> defaultAutoZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AutoAngularZupt& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.threshold);
        
    }
}
void extract(Serializer& serializer, AutoAngularZupt& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.threshold);
        
    }
}

void insert(Serializer& serializer, const AutoAngularZupt::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.threshold);
    
}
void extract(Serializer& serializer, AutoAngularZupt::Response& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.threshold);
    
}

TypedResult<AutoAngularZupt> writeAutoAngularZupt(C::mip_interface& device, uint8_t enable, float threshold)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, threshold);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoAngularZupt> readAutoAngularZupt(C::mip_interface& device, uint8_t* enableOut, float* thresholdOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AutoAngularZupt> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ANGULAR_ZUPT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(thresholdOut);
        extract(deserializer, *thresholdOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AutoAngularZupt> saveAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoAngularZupt> loadAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AutoAngularZupt> defaultAutoAngularZupt(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANGULAR_ZUPT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, CommandedZupt& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<CommandedZupt> commandedZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ZUPT, NULL, 0);
}
void insert(Serializer& serializer, const CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, CommandedAngularZupt& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<CommandedAngularZupt> commandedAngularZupt(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_COMMANDED_ANGULAR_ZUPT, NULL, 0);
}
void insert(Serializer& serializer, const MagCaptureAutoCal& self)
{
    insert(serializer, self.function);
    
}
void extract(Serializer& serializer, MagCaptureAutoCal& self)
{
    extract(serializer, self.function);
    
}

TypedResult<MagCaptureAutoCal> writeMagCaptureAutoCal(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagCaptureAutoCal> saveMagCaptureAutoCal(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_CAPTURE_AUTO_CALIBRATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GravityNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, GravityNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const GravityNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, GravityNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<GravityNoise> writeGravityNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GravityNoise> readGravityNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GravityNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GRAVITY_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GravityNoise> saveGravityNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GravityNoise> loadGravityNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GravityNoise> defaultGravityNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GRAVITY_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const PressureAltitudeNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.noise);
        
    }
}
void extract(Serializer& serializer, PressureAltitudeNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.noise);
        
    }
}

void insert(Serializer& serializer, const PressureAltitudeNoise::Response& self)
{
    insert(serializer, self.noise);
    
}
void extract(Serializer& serializer, PressureAltitudeNoise::Response& self)
{
    extract(serializer, self.noise);
    
}

TypedResult<PressureAltitudeNoise> writePressureAltitudeNoise(C::mip_interface& device, float noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, noise);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PressureAltitudeNoise> readPressureAltitudeNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<PressureAltitudeNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_PRESSURE_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut);
        extract(deserializer, *noiseOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<PressureAltitudeNoise> savePressureAltitudeNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PressureAltitudeNoise> loadPressureAltitudeNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<PressureAltitudeNoise> defaultPressureAltitudeNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const HardIronOffsetNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, HardIronOffsetNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const HardIronOffsetNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, HardIronOffsetNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<HardIronOffsetNoise> writeHardIronOffsetNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<HardIronOffsetNoise> readHardIronOffsetNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<HardIronOffsetNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_HARD_IRON_OFFSET_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<HardIronOffsetNoise> saveHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<HardIronOffsetNoise> loadHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<HardIronOffsetNoise> defaultHardIronOffsetNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HARD_IRON_OFFSET_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SoftIronMatrixNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, SoftIronMatrixNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 9; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const SoftIronMatrixNoise::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, SoftIronMatrixNoise::Response& self)
{
    for(unsigned int i=0; i < 9; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<SoftIronMatrixNoise> writeSoftIronMatrixNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(noise || (9 == 0));
    for(unsigned int i=0; i < 9; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SoftIronMatrixNoise> readSoftIronMatrixNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SoftIronMatrixNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SOFT_IRON_MATRIX_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut || (9 == 0));
        for(unsigned int i=0; i < 9; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SoftIronMatrixNoise> saveSoftIronMatrixNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SoftIronMatrixNoise> loadSoftIronMatrixNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SoftIronMatrixNoise> defaultSoftIronMatrixNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SOFT_IRON_MATRIX_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagNoise& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.noise[i]);
        
    }
}
void extract(Serializer& serializer, MagNoise& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.noise[i]);
        
    }
}

void insert(Serializer& serializer, const MagNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.noise[i]);
    
}
void extract(Serializer& serializer, MagNoise::Response& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.noise[i]);
    
}

TypedResult<MagNoise> writeMagNoise(C::mip_interface& device, const float* noise)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    assert(noise || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, noise[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagNoise> readMagNoise(C::mip_interface& device, float* noiseOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagNoise> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MAG_NOISE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(noiseOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, noiseOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagNoise> saveMagNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagNoise> loadMagNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagNoise> defaultMagNoise(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_NOISE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const InclinationSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
        insert(serializer, self.inclination);
        
    }
}
void extract(Serializer& serializer, InclinationSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
        extract(serializer, self.inclination);
        
    }
}

void insert(Serializer& serializer, const InclinationSource::Response& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.inclination);
    
}
void extract(Serializer& serializer, InclinationSource::Response& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.inclination);
    
}

TypedResult<InclinationSource> writeInclinationSource(C::mip_interface& device, FilterMagParamSource source, float inclination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    insert(serializer, inclination);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<InclinationSource> readInclinationSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* inclinationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<InclinationSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_INCLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(inclinationOut);
        extract(deserializer, *inclinationOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<InclinationSource> saveInclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<InclinationSource> loadInclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<InclinationSource> defaultInclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INCLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagneticDeclinationSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
        insert(serializer, self.declination);
        
    }
}
void extract(Serializer& serializer, MagneticDeclinationSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
        extract(serializer, self.declination);
        
    }
}

void insert(Serializer& serializer, const MagneticDeclinationSource::Response& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.declination);
    
}
void extract(Serializer& serializer, MagneticDeclinationSource::Response& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.declination);
    
}

TypedResult<MagneticDeclinationSource> writeMagneticDeclinationSource(C::mip_interface& device, FilterMagParamSource source, float declination)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    insert(serializer, declination);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagneticDeclinationSource> readMagneticDeclinationSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* declinationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagneticDeclinationSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_DECLINATION_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(declinationOut);
        extract(deserializer, *declinationOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagneticDeclinationSource> saveMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagneticDeclinationSource> loadMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagneticDeclinationSource> defaultMagneticDeclinationSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_DECLINATION_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagFieldMagnitudeSource& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
        insert(serializer, self.magnitude);
        
    }
}
void extract(Serializer& serializer, MagFieldMagnitudeSource& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
        extract(serializer, self.magnitude);
        
    }
}

void insert(Serializer& serializer, const MagFieldMagnitudeSource::Response& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.magnitude);
    
}
void extract(Serializer& serializer, MagFieldMagnitudeSource::Response& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.magnitude);
    
}

TypedResult<MagFieldMagnitudeSource> writeMagFieldMagnitudeSource(C::mip_interface& device, FilterMagParamSource source, float magnitude)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    insert(serializer, magnitude);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagFieldMagnitudeSource> readMagFieldMagnitudeSource(C::mip_interface& device, FilterMagParamSource* sourceOut, float* magnitudeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagFieldMagnitudeSource> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MAGNETIC_MAGNITUDE_SOURCE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(magnitudeOut);
        extract(deserializer, *magnitudeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagFieldMagnitudeSource> saveMagFieldMagnitudeSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagFieldMagnitudeSource> loadMagFieldMagnitudeSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagFieldMagnitudeSource> defaultMagFieldMagnitudeSource(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_MAGNITUDE_SOURCE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const ReferencePosition& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.latitude);
        
        insert(serializer, self.longitude);
        
        insert(serializer, self.altitude);
        
    }
}
void extract(Serializer& serializer, ReferencePosition& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.latitude);
        
        extract(serializer, self.longitude);
        
        extract(serializer, self.altitude);
        
    }
}

void insert(Serializer& serializer, const ReferencePosition::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.latitude);
    
    insert(serializer, self.longitude);
    
    insert(serializer, self.altitude);
    
}
void extract(Serializer& serializer, ReferencePosition::Response& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.latitude);
    
    extract(serializer, self.longitude);
    
    extract(serializer, self.altitude);
    
}

TypedResult<ReferencePosition> writeReferencePosition(C::mip_interface& device, bool enable, double latitude, double longitude, double altitude)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, latitude);
    
    insert(serializer, longitude);
    
    insert(serializer, altitude);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ReferencePosition> readReferencePosition(C::mip_interface& device, bool* enableOut, double* latitudeOut, double* longitudeOut, double* altitudeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<ReferencePosition> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_REFERENCE_POSITION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(latitudeOut);
        extract(deserializer, *latitudeOut);
        
        assert(longitudeOut);
        extract(deserializer, *longitudeOut);
        
        assert(altitudeOut);
        extract(deserializer, *altitudeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<ReferencePosition> saveReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ReferencePosition> loadReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<ReferencePosition> defaultReferencePosition(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REFERENCE_POSITION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.adaptive_measurement);
        
        insert(serializer, self.frequency);
        
        insert(serializer, self.low_limit);
        
        insert(serializer, self.high_limit);
        
        insert(serializer, self.low_limit_uncertainty);
        
        insert(serializer, self.high_limit_uncertainty);
        
        insert(serializer, self.minimum_uncertainty);
        
    }
}
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.adaptive_measurement);
        
        extract(serializer, self.frequency);
        
        extract(serializer, self.low_limit);
        
        extract(serializer, self.high_limit);
        
        extract(serializer, self.low_limit_uncertainty);
        
        extract(serializer, self.high_limit_uncertainty);
        
        extract(serializer, self.minimum_uncertainty);
        
    }
}

void insert(Serializer& serializer, const AccelMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    insert(serializer, self.adaptive_measurement);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.low_limit);
    
    insert(serializer, self.high_limit);
    
    insert(serializer, self.low_limit_uncertainty);
    
    insert(serializer, self.high_limit_uncertainty);
    
    insert(serializer, self.minimum_uncertainty);
    
}
void extract(Serializer& serializer, AccelMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    extract(serializer, self.adaptive_measurement);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.low_limit);
    
    extract(serializer, self.high_limit);
    
    extract(serializer, self.low_limit_uncertainty);
    
    extract(serializer, self.high_limit_uncertainty);
    
    extract(serializer, self.minimum_uncertainty);
    
}

TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> writeAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, adaptiveMeasurement);
    
    insert(serializer, frequency);
    
    insert(serializer, lowLimit);
    
    insert(serializer, highLimit);
    
    insert(serializer, lowLimitUncertainty);
    
    insert(serializer, highLimitUncertainty);
    
    insert(serializer, minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> readAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(adaptiveMeasurementOut);
        extract(deserializer, *adaptiveMeasurementOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(lowLimitOut);
        extract(deserializer, *lowLimitOut);
        
        assert(highLimitOut);
        extract(deserializer, *highLimitOut);
        
        assert(lowLimitUncertaintyOut);
        extract(deserializer, *lowLimitUncertaintyOut);
        
        assert(highLimitUncertaintyOut);
        extract(deserializer, *highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        extract(deserializer, *minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> saveAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> loadAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AccelMagnitudeErrorAdaptiveMeasurement> defaultAccelMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ACCEL_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.adaptive_measurement);
        
        insert(serializer, self.frequency);
        
        insert(serializer, self.low_limit);
        
        insert(serializer, self.high_limit);
        
        insert(serializer, self.low_limit_uncertainty);
        
        insert(serializer, self.high_limit_uncertainty);
        
        insert(serializer, self.minimum_uncertainty);
        
    }
}
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.adaptive_measurement);
        
        extract(serializer, self.frequency);
        
        extract(serializer, self.low_limit);
        
        extract(serializer, self.high_limit);
        
        extract(serializer, self.low_limit_uncertainty);
        
        extract(serializer, self.high_limit_uncertainty);
        
        extract(serializer, self.minimum_uncertainty);
        
    }
}

void insert(Serializer& serializer, const MagMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    insert(serializer, self.adaptive_measurement);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.low_limit);
    
    insert(serializer, self.high_limit);
    
    insert(serializer, self.low_limit_uncertainty);
    
    insert(serializer, self.high_limit_uncertainty);
    
    insert(serializer, self.minimum_uncertainty);
    
}
void extract(Serializer& serializer, MagMagnitudeErrorAdaptiveMeasurement::Response& self)
{
    extract(serializer, self.adaptive_measurement);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.low_limit);
    
    extract(serializer, self.high_limit);
    
    extract(serializer, self.low_limit_uncertainty);
    
    extract(serializer, self.high_limit_uncertainty);
    
    extract(serializer, self.minimum_uncertainty);
    
}

TypedResult<MagMagnitudeErrorAdaptiveMeasurement> writeMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement adaptiveMeasurement, float frequency, float lowLimit, float highLimit, float lowLimitUncertainty, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, adaptiveMeasurement);
    
    insert(serializer, frequency);
    
    insert(serializer, lowLimit);
    
    insert(serializer, highLimit);
    
    insert(serializer, lowLimitUncertainty);
    
    insert(serializer, highLimitUncertainty);
    
    insert(serializer, minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> readMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device, FilterAdaptiveMeasurement* adaptiveMeasurementOut, float* frequencyOut, float* lowLimitOut, float* highLimitOut, float* lowLimitUncertaintyOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagMagnitudeErrorAdaptiveMeasurement> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(adaptiveMeasurementOut);
        extract(deserializer, *adaptiveMeasurementOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(lowLimitOut);
        extract(deserializer, *lowLimitOut);
        
        assert(highLimitOut);
        extract(deserializer, *highLimitOut);
        
        assert(lowLimitUncertaintyOut);
        extract(deserializer, *lowLimitUncertaintyOut);
        
        assert(highLimitUncertaintyOut);
        extract(deserializer, *highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        extract(deserializer, *minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> saveMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> loadMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagMagnitudeErrorAdaptiveMeasurement> defaultMagMagnitudeErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_MAGNITUDE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.frequency);
        
        insert(serializer, self.high_limit);
        
        insert(serializer, self.high_limit_uncertainty);
        
        insert(serializer, self.minimum_uncertainty);
        
    }
}
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.frequency);
        
        extract(serializer, self.high_limit);
        
        extract(serializer, self.high_limit_uncertainty);
        
        extract(serializer, self.minimum_uncertainty);
        
    }
}

void insert(Serializer& serializer, const MagDipAngleErrorAdaptiveMeasurement::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.frequency);
    
    insert(serializer, self.high_limit);
    
    insert(serializer, self.high_limit_uncertainty);
    
    insert(serializer, self.minimum_uncertainty);
    
}
void extract(Serializer& serializer, MagDipAngleErrorAdaptiveMeasurement::Response& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.frequency);
    
    extract(serializer, self.high_limit);
    
    extract(serializer, self.high_limit_uncertainty);
    
    extract(serializer, self.minimum_uncertainty);
    
}

TypedResult<MagDipAngleErrorAdaptiveMeasurement> writeMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool enable, float frequency, float highLimit, float highLimitUncertainty, float minimumUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, frequency);
    
    insert(serializer, highLimit);
    
    insert(serializer, highLimitUncertainty);
    
    insert(serializer, minimumUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> readMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device, bool* enableOut, float* frequencyOut, float* highLimitOut, float* highLimitUncertaintyOut, float* minimumUncertaintyOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MagDipAngleErrorAdaptiveMeasurement> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(frequencyOut);
        extract(deserializer, *frequencyOut);
        
        assert(highLimitOut);
        extract(deserializer, *highLimitOut);
        
        assert(highLimitUncertaintyOut);
        extract(deserializer, *highLimitUncertaintyOut);
        
        assert(minimumUncertaintyOut);
        extract(deserializer, *minimumUncertaintyOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> saveMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> loadMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MagDipAngleErrorAdaptiveMeasurement> defaultMagDipAngleErrorAdaptiveMeasurement(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAG_DIP_ANGLE_ERROR_ADAPTIVE_MEASUREMENT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AidingMeasurementEnable& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.aiding_source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, AidingMeasurementEnable& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.aiding_source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const AidingMeasurementEnable::Response& self)
{
    insert(serializer, self.aiding_source);
    
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, AidingMeasurementEnable::Response& self)
{
    extract(serializer, self.aiding_source);
    
    extract(serializer, self.enable);
    
}

TypedResult<AidingMeasurementEnable> writeAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, aidingSource);
    
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AidingMeasurementEnable> readAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource, bool* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AidingMeasurementEnable> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_AIDING_MEASUREMENT_ENABLE, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, aidingSource);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AidingMeasurementEnable> saveAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AidingMeasurementEnable> loadAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AidingMeasurementEnable> defaultAidingMeasurementEnable(C::mip_interface& device, AidingMeasurementEnable::AidingSource aidingSource)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, aidingSource);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_AIDING_MEASUREMENT_ENABLE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const Run& self)
{
    (void)serializer;
    (void)self;
}
void extract(Serializer& serializer, Run& self)
{
    (void)serializer;
    (void)self;
}

TypedResult<Run> run(C::mip_interface& device)
{
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_RUN, NULL, 0);
}
void insert(Serializer& serializer, const KinematicConstraint& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.acceleration_constraint_selection);
        
        insert(serializer, self.velocity_constraint_selection);
        
        insert(serializer, self.angular_constraint_selection);
        
    }
}
void extract(Serializer& serializer, KinematicConstraint& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.acceleration_constraint_selection);
        
        extract(serializer, self.velocity_constraint_selection);
        
        extract(serializer, self.angular_constraint_selection);
        
    }
}

void insert(Serializer& serializer, const KinematicConstraint::Response& self)
{
    insert(serializer, self.acceleration_constraint_selection);
    
    insert(serializer, self.velocity_constraint_selection);
    
    insert(serializer, self.angular_constraint_selection);
    
}
void extract(Serializer& serializer, KinematicConstraint::Response& self)
{
    extract(serializer, self.acceleration_constraint_selection);
    
    extract(serializer, self.velocity_constraint_selection);
    
    extract(serializer, self.angular_constraint_selection);
    
}

TypedResult<KinematicConstraint> writeKinematicConstraint(C::mip_interface& device, uint8_t accelerationConstraintSelection, uint8_t velocityConstraintSelection, uint8_t angularConstraintSelection)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, accelerationConstraintSelection);
    
    insert(serializer, velocityConstraintSelection);
    
    insert(serializer, angularConstraintSelection);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<KinematicConstraint> readKinematicConstraint(C::mip_interface& device, uint8_t* accelerationConstraintSelectionOut, uint8_t* velocityConstraintSelectionOut, uint8_t* angularConstraintSelectionOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<KinematicConstraint> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_KINEMATIC_CONSTRAINT, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(accelerationConstraintSelectionOut);
        extract(deserializer, *accelerationConstraintSelectionOut);
        
        assert(velocityConstraintSelectionOut);
        extract(deserializer, *velocityConstraintSelectionOut);
        
        assert(angularConstraintSelectionOut);
        extract(deserializer, *angularConstraintSelectionOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<KinematicConstraint> saveKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<KinematicConstraint> loadKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<KinematicConstraint> defaultKinematicConstraint(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_KINEMATIC_CONSTRAINT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const InitializationConfiguration& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.wait_for_run_command);
        
        insert(serializer, self.initial_cond_src);
        
        insert(serializer, self.auto_heading_alignment_selector);
        
        insert(serializer, self.initial_heading);
        
        insert(serializer, self.initial_pitch);
        
        insert(serializer, self.initial_roll);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.initial_position[i]);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.initial_velocity[i]);
        
        insert(serializer, self.reference_frame_selector);
        
    }
}
void extract(Serializer& serializer, InitializationConfiguration& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.wait_for_run_command);
        
        extract(serializer, self.initial_cond_src);
        
        extract(serializer, self.auto_heading_alignment_selector);
        
        extract(serializer, self.initial_heading);
        
        extract(serializer, self.initial_pitch);
        
        extract(serializer, self.initial_roll);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.initial_position[i]);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.initial_velocity[i]);
        
        extract(serializer, self.reference_frame_selector);
        
    }
}

void insert(Serializer& serializer, const InitializationConfiguration::Response& self)
{
    insert(serializer, self.wait_for_run_command);
    
    insert(serializer, self.initial_cond_src);
    
    insert(serializer, self.auto_heading_alignment_selector);
    
    insert(serializer, self.initial_heading);
    
    insert(serializer, self.initial_pitch);
    
    insert(serializer, self.initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.initial_velocity[i]);
    
    insert(serializer, self.reference_frame_selector);
    
}
void extract(Serializer& serializer, InitializationConfiguration::Response& self)
{
    extract(serializer, self.wait_for_run_command);
    
    extract(serializer, self.initial_cond_src);
    
    extract(serializer, self.auto_heading_alignment_selector);
    
    extract(serializer, self.initial_heading);
    
    extract(serializer, self.initial_pitch);
    
    extract(serializer, self.initial_roll);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.initial_velocity[i]);
    
    extract(serializer, self.reference_frame_selector);
    
}

TypedResult<InitializationConfiguration> writeInitializationConfiguration(C::mip_interface& device, uint8_t waitForRunCommand, InitializationConfiguration::InitialConditionSource initialCondSrc, InitializationConfiguration::AlignmentSelector autoHeadingAlignmentSelector, float initialHeading, float initialPitch, float initialRoll, const float* initialPosition, const float* initialVelocity, FilterReferenceFrame referenceFrameSelector)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, waitForRunCommand);
    
    insert(serializer, initialCondSrc);
    
    insert(serializer, autoHeadingAlignmentSelector);
    
    insert(serializer, initialHeading);
    
    insert(serializer, initialPitch);
    
    insert(serializer, initialRoll);
    
    assert(initialPosition || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initialPosition[i]);
    
    assert(initialVelocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, initialVelocity[i]);
    
    insert(serializer, referenceFrameSelector);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<InitializationConfiguration> readInitializationConfiguration(C::mip_interface& device, uint8_t* waitForRunCommandOut, InitializationConfiguration::InitialConditionSource* initialCondSrcOut, InitializationConfiguration::AlignmentSelector* autoHeadingAlignmentSelectorOut, float* initialHeadingOut, float* initialPitchOut, float* initialRollOut, float* initialPositionOut, float* initialVelocityOut, FilterReferenceFrame* referenceFrameSelectorOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<InitializationConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_INITIALIZATION_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(waitForRunCommandOut);
        extract(deserializer, *waitForRunCommandOut);
        
        assert(initialCondSrcOut);
        extract(deserializer, *initialCondSrcOut);
        
        assert(autoHeadingAlignmentSelectorOut);
        extract(deserializer, *autoHeadingAlignmentSelectorOut);
        
        assert(initialHeadingOut);
        extract(deserializer, *initialHeadingOut);
        
        assert(initialPitchOut);
        extract(deserializer, *initialPitchOut);
        
        assert(initialRollOut);
        extract(deserializer, *initialRollOut);
        
        assert(initialPositionOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, initialPositionOut[i]);
        
        assert(initialVelocityOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, initialVelocityOut[i]);
        
        assert(referenceFrameSelectorOut);
        extract(deserializer, *referenceFrameSelectorOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<InitializationConfiguration> saveInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<InitializationConfiguration> loadInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<InitializationConfiguration> defaultInitializationConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_INITIALIZATION_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const AdaptiveFilterOptions& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.level);
        
        insert(serializer, self.time_limit);
        
    }
}
void extract(Serializer& serializer, AdaptiveFilterOptions& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.level);
        
        extract(serializer, self.time_limit);
        
    }
}

void insert(Serializer& serializer, const AdaptiveFilterOptions::Response& self)
{
    insert(serializer, self.level);
    
    insert(serializer, self.time_limit);
    
}
void extract(Serializer& serializer, AdaptiveFilterOptions::Response& self)
{
    extract(serializer, self.level);
    
    extract(serializer, self.time_limit);
    
}

TypedResult<AdaptiveFilterOptions> writeAdaptiveFilterOptions(C::mip_interface& device, uint8_t level, uint16_t timeLimit)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, level);
    
    insert(serializer, timeLimit);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AdaptiveFilterOptions> readAdaptiveFilterOptions(C::mip_interface& device, uint8_t* levelOut, uint16_t* timeLimitOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<AdaptiveFilterOptions> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ADAPTIVE_FILTER_OPTIONS, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(levelOut);
        extract(deserializer, *levelOut);
        
        assert(timeLimitOut);
        extract(deserializer, *timeLimitOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<AdaptiveFilterOptions> saveAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AdaptiveFilterOptions> loadAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<AdaptiveFilterOptions> defaultAdaptiveFilterOptions(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ADAPTIVE_FILTER_OPTIONS, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MultiAntennaOffset& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.receiver_id);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.antenna_offset[i]);
        
    }
}
void extract(Serializer& serializer, MultiAntennaOffset& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.receiver_id);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.antenna_offset[i]);
        
    }
}

void insert(Serializer& serializer, const MultiAntennaOffset::Response& self)
{
    insert(serializer, self.receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.antenna_offset[i]);
    
}
void extract(Serializer& serializer, MultiAntennaOffset::Response& self)
{
    extract(serializer, self.receiver_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.antenna_offset[i]);
    
}

TypedResult<MultiAntennaOffset> writeMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, const float* antennaOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, receiverId);
    
    assert(antennaOffset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, antennaOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MultiAntennaOffset> readMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId, float* antennaOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<MultiAntennaOffset> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_MULTI_ANTENNA_OFFSET, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, receiverId);
        
        assert(antennaOffsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, antennaOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<MultiAntennaOffset> saveMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MultiAntennaOffset> loadMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<MultiAntennaOffset> defaultMultiAntennaOffset(C::mip_interface& device, uint8_t receiverId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, receiverId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MULTI_ANTENNA_OFFSET, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const RelPosConfiguration& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.source);
        
        insert(serializer, self.reference_frame_selector);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.reference_coordinates[i]);
        
    }
}
void extract(Serializer& serializer, RelPosConfiguration& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.source);
        
        extract(serializer, self.reference_frame_selector);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.reference_coordinates[i]);
        
    }
}

void insert(Serializer& serializer, const RelPosConfiguration::Response& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.reference_coordinates[i]);
    
}
void extract(Serializer& serializer, RelPosConfiguration::Response& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.reference_frame_selector);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.reference_coordinates[i]);
    
}

TypedResult<RelPosConfiguration> writeRelPosConfiguration(C::mip_interface& device, uint8_t source, FilterReferenceFrame referenceFrameSelector, const double* referenceCoordinates)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    insert(serializer, referenceFrameSelector);
    
    assert(referenceCoordinates || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, referenceCoordinates[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RelPosConfiguration> readRelPosConfiguration(C::mip_interface& device, uint8_t* sourceOut, FilterReferenceFrame* referenceFrameSelectorOut, double* referenceCoordinatesOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<RelPosConfiguration> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_REL_POS_CONFIGURATION, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(sourceOut);
        extract(deserializer, *sourceOut);
        
        assert(referenceFrameSelectorOut);
        extract(deserializer, *referenceFrameSelectorOut);
        
        assert(referenceCoordinatesOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, referenceCoordinatesOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<RelPosConfiguration> saveRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RelPosConfiguration> loadRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RelPosConfiguration> defaultRelPosConfiguration(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REL_POS_CONFIGURATION, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const RefPointLeverArm& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.ref_point_sel);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.lever_arm_offset[i]);
        
    }
}
void extract(Serializer& serializer, RefPointLeverArm& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.ref_point_sel);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.lever_arm_offset[i]);
        
    }
}

void insert(Serializer& serializer, const RefPointLeverArm::Response& self)
{
    insert(serializer, self.ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
    
}
void extract(Serializer& serializer, RefPointLeverArm::Response& self)
{
    extract(serializer, self.ref_point_sel);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
    
}

TypedResult<RefPointLeverArm> writeRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector refPointSel, const float* leverArmOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, refPointSel);
    
    assert(leverArmOffset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RefPointLeverArm> readRefPointLeverArm(C::mip_interface& device, RefPointLeverArm::ReferencePointSelector* refPointSelOut, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<RefPointLeverArm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_REF_POINT_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(refPointSelOut);
        extract(deserializer, *refPointSelOut);
        
        assert(leverArmOffsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, leverArmOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<RefPointLeverArm> saveRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RefPointLeverArm> loadRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<RefPointLeverArm> defaultRefPointLeverArm(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_REF_POINT_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SpeedMeasurement& self)
{
    insert(serializer, self.source);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.speed);
    
    insert(serializer, self.speed_uncertainty);
    
}
void extract(Serializer& serializer, SpeedMeasurement& self)
{
    extract(serializer, self.source);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.speed);
    
    extract(serializer, self.speed_uncertainty);
    
}

TypedResult<SpeedMeasurement> speedMeasurement(C::mip_interface& device, uint8_t source, float timeOfWeek, float speed, float speedUncertainty)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, source);
    
    insert(serializer, timeOfWeek);
    
    insert(serializer, speed);
    
    insert(serializer, speedUncertainty);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_MEASUREMENT, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SpeedLeverArm& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.lever_arm_offset[i]);
        
    }
}
void extract(Serializer& serializer, SpeedLeverArm& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.source);
    
    if( self.function == FunctionSelector::WRITE )
    {
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.lever_arm_offset[i]);
        
    }
}

void insert(Serializer& serializer, const SpeedLeverArm::Response& self)
{
    insert(serializer, self.source);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.lever_arm_offset[i]);
    
}
void extract(Serializer& serializer, SpeedLeverArm::Response& self)
{
    extract(serializer, self.source);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.lever_arm_offset[i]);
    
}

TypedResult<SpeedLeverArm> writeSpeedLeverArm(C::mip_interface& device, uint8_t source, const float* leverArmOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, source);
    
    assert(leverArmOffset || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, leverArmOffset[i]);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SpeedLeverArm> readSpeedLeverArm(C::mip_interface& device, uint8_t source, float* leverArmOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<SpeedLeverArm> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_SPEED_LEVER_ARM, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, source);
        
        assert(leverArmOffsetOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, leverArmOffsetOut[i]);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<SpeedLeverArm> saveSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SpeedLeverArm> loadSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<SpeedLeverArm> defaultSpeedLeverArm(C::mip_interface& device, uint8_t source)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, source);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SPEED_LEVER_ARM, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const WheeledVehicleConstraintControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, WheeledVehicleConstraintControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const WheeledVehicleConstraintControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, WheeledVehicleConstraintControl::Response& self)
{
    extract(serializer, self.enable);
    
}

TypedResult<WheeledVehicleConstraintControl> writeWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<WheeledVehicleConstraintControl> readWheeledVehicleConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<WheeledVehicleConstraintControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_VEHICLE_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<WheeledVehicleConstraintControl> saveWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<WheeledVehicleConstraintControl> loadWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<WheeledVehicleConstraintControl> defaultWheeledVehicleConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEHICLE_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VerticalGyroConstraintControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
    }
}
void extract(Serializer& serializer, VerticalGyroConstraintControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
    }
}

void insert(Serializer& serializer, const VerticalGyroConstraintControl::Response& self)
{
    insert(serializer, self.enable);
    
}
void extract(Serializer& serializer, VerticalGyroConstraintControl::Response& self)
{
    extract(serializer, self.enable);
    
}

TypedResult<VerticalGyroConstraintControl> writeVerticalGyroConstraintControl(C::mip_interface& device, uint8_t enable)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<VerticalGyroConstraintControl> readVerticalGyroConstraintControl(C::mip_interface& device, uint8_t* enableOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<VerticalGyroConstraintControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_GYRO_CONSTRAINT_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<VerticalGyroConstraintControl> saveVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<VerticalGyroConstraintControl> loadVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<VerticalGyroConstraintControl> defaultVerticalGyroConstraintControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_GYRO_CONSTRAINT_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const GnssAntennaCalControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.enable);
        
        insert(serializer, self.max_offset);
        
    }
}
void extract(Serializer& serializer, GnssAntennaCalControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.enable);
        
        extract(serializer, self.max_offset);
        
    }
}

void insert(Serializer& serializer, const GnssAntennaCalControl::Response& self)
{
    insert(serializer, self.enable);
    
    insert(serializer, self.max_offset);
    
}
void extract(Serializer& serializer, GnssAntennaCalControl::Response& self)
{
    extract(serializer, self.enable);
    
    extract(serializer, self.max_offset);
    
}

TypedResult<GnssAntennaCalControl> writeGnssAntennaCalControl(C::mip_interface& device, uint8_t enable, float maxOffset)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, enable);
    
    insert(serializer, maxOffset);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssAntennaCalControl> readGnssAntennaCalControl(C::mip_interface& device, uint8_t* enableOut, float* maxOffsetOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<GnssAntennaCalControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ANTENNA_CALIBRATION_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(enableOut);
        extract(deserializer, *enableOut);
        
        assert(maxOffsetOut);
        extract(deserializer, *maxOffsetOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<GnssAntennaCalControl> saveGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssAntennaCalControl> loadGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<GnssAntennaCalControl> defaultGnssAntennaCalControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ANTENNA_CALIBRATION_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const SetInitialHeading& self)
{
    insert(serializer, self.heading);
    
}
void extract(Serializer& serializer, SetInitialHeading& self)
{
    extract(serializer, self.heading);
    
}

TypedResult<SetInitialHeading> setInitialHeading(C::mip_interface& device, float heading)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, heading);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_SET_INITIAL_HEADING, buffer, (uint8_t)mip_serializer_length(&serializer));
}

} // namespace commands_filter
} // namespace mip

