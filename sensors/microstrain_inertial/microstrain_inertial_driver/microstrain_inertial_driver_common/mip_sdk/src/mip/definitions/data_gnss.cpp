
#include "data_gnss.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace data_gnss {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const PosLlh& self)
{
    insert(serializer, self.latitude);
    
    insert(serializer, self.longitude);
    
    insert(serializer, self.ellipsoid_height);
    
    insert(serializer, self.msl_height);
    
    insert(serializer, self.horizontal_accuracy);
    
    insert(serializer, self.vertical_accuracy);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, PosLlh& self)
{
    extract(serializer, self.latitude);
    
    extract(serializer, self.longitude);
    
    extract(serializer, self.ellipsoid_height);
    
    extract(serializer, self.msl_height);
    
    extract(serializer, self.horizontal_accuracy);
    
    extract(serializer, self.vertical_accuracy);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const PosEcef& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.x[i]);
    
    insert(serializer, self.x_accuracy);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, PosEcef& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.x[i]);
    
    extract(serializer, self.x_accuracy);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const VelNed& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.v[i]);
    
    insert(serializer, self.speed);
    
    insert(serializer, self.ground_speed);
    
    insert(serializer, self.heading);
    
    insert(serializer, self.speed_accuracy);
    
    insert(serializer, self.heading_accuracy);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, VelNed& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.v[i]);
    
    extract(serializer, self.speed);
    
    extract(serializer, self.ground_speed);
    
    extract(serializer, self.heading);
    
    extract(serializer, self.speed_accuracy);
    
    extract(serializer, self.heading_accuracy);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const VelEcef& self)
{
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.v[i]);
    
    insert(serializer, self.v_accuracy);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, VelEcef& self)
{
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.v[i]);
    
    extract(serializer, self.v_accuracy);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const Dop& self)
{
    insert(serializer, self.gdop);
    
    insert(serializer, self.pdop);
    
    insert(serializer, self.hdop);
    
    insert(serializer, self.vdop);
    
    insert(serializer, self.tdop);
    
    insert(serializer, self.ndop);
    
    insert(serializer, self.edop);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, Dop& self)
{
    extract(serializer, self.gdop);
    
    extract(serializer, self.pdop);
    
    extract(serializer, self.hdop);
    
    extract(serializer, self.vdop);
    
    extract(serializer, self.tdop);
    
    extract(serializer, self.ndop);
    
    extract(serializer, self.edop);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const UtcTime& self)
{
    insert(serializer, self.year);
    
    insert(serializer, self.month);
    
    insert(serializer, self.day);
    
    insert(serializer, self.hour);
    
    insert(serializer, self.min);
    
    insert(serializer, self.sec);
    
    insert(serializer, self.msec);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, UtcTime& self)
{
    extract(serializer, self.year);
    
    extract(serializer, self.month);
    
    extract(serializer, self.day);
    
    extract(serializer, self.hour);
    
    extract(serializer, self.min);
    
    extract(serializer, self.sec);
    
    extract(serializer, self.msec);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const GpsTime& self)
{
    insert(serializer, self.tow);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GpsTime& self)
{
    extract(serializer, self.tow);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const ClockInfo& self)
{
    insert(serializer, self.bias);
    
    insert(serializer, self.drift);
    
    insert(serializer, self.accuracy_estimate);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, ClockInfo& self)
{
    extract(serializer, self.bias);
    
    extract(serializer, self.drift);
    
    extract(serializer, self.accuracy_estimate);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const FixInfo& self)
{
    insert(serializer, self.fix_type);
    
    insert(serializer, self.num_sv);
    
    insert(serializer, self.fix_flags);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, FixInfo& self)
{
    extract(serializer, self.fix_type);
    
    extract(serializer, self.num_sv);
    
    extract(serializer, self.fix_flags);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const SvInfo& self)
{
    insert(serializer, self.channel);
    
    insert(serializer, self.sv_id);
    
    insert(serializer, self.carrier_noise_ratio);
    
    insert(serializer, self.azimuth);
    
    insert(serializer, self.elevation);
    
    insert(serializer, self.sv_flags);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, SvInfo& self)
{
    extract(serializer, self.channel);
    
    extract(serializer, self.sv_id);
    
    extract(serializer, self.carrier_noise_ratio);
    
    extract(serializer, self.azimuth);
    
    extract(serializer, self.elevation);
    
    extract(serializer, self.sv_flags);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const HwStatus& self)
{
    insert(serializer, self.receiver_state);
    
    insert(serializer, self.antenna_state);
    
    insert(serializer, self.antenna_power);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, HwStatus& self)
{
    extract(serializer, self.receiver_state);
    
    extract(serializer, self.antenna_state);
    
    extract(serializer, self.antenna_power);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const DgpsInfo& self)
{
    insert(serializer, self.sv_id);
    
    insert(serializer, self.age);
    
    insert(serializer, self.range_correction);
    
    insert(serializer, self.range_rate_correction);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, DgpsInfo& self)
{
    extract(serializer, self.sv_id);
    
    extract(serializer, self.age);
    
    extract(serializer, self.range_correction);
    
    extract(serializer, self.range_rate_correction);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const DgpsChannel& self)
{
    insert(serializer, self.sv_id);
    
    insert(serializer, self.age);
    
    insert(serializer, self.range_correction);
    
    insert(serializer, self.range_rate_correction);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, DgpsChannel& self)
{
    extract(serializer, self.sv_id);
    
    extract(serializer, self.age);
    
    extract(serializer, self.range_correction);
    
    extract(serializer, self.range_rate_correction);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const ClockInfo2& self)
{
    insert(serializer, self.bias);
    
    insert(serializer, self.drift);
    
    insert(serializer, self.bias_accuracy_estimate);
    
    insert(serializer, self.drift_accuracy_estimate);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, ClockInfo2& self)
{
    extract(serializer, self.bias);
    
    extract(serializer, self.drift);
    
    extract(serializer, self.bias_accuracy_estimate);
    
    extract(serializer, self.drift_accuracy_estimate);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const GpsLeapSeconds& self)
{
    insert(serializer, self.leap_seconds);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GpsLeapSeconds& self)
{
    extract(serializer, self.leap_seconds);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const SbasInfo& self)
{
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.sbas_system);
    
    insert(serializer, self.sbas_id);
    
    insert(serializer, self.count);
    
    insert(serializer, self.sbas_status);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, SbasInfo& self)
{
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.sbas_system);
    
    extract(serializer, self.sbas_id);
    
    extract(serializer, self.count);
    
    extract(serializer, self.sbas_status);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const SbasCorrection& self)
{
    insert(serializer, self.index);
    
    insert(serializer, self.count);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.gnss_id);
    
    insert(serializer, self.sv_id);
    
    insert(serializer, self.udrei);
    
    insert(serializer, self.pseudorange_correction);
    
    insert(serializer, self.iono_correction);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, SbasCorrection& self)
{
    extract(serializer, self.index);
    
    extract(serializer, self.count);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.gnss_id);
    
    extract(serializer, self.sv_id);
    
    extract(serializer, self.udrei);
    
    extract(serializer, self.pseudorange_correction);
    
    extract(serializer, self.iono_correction);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const RfErrorDetection& self)
{
    insert(serializer, self.rf_band);
    
    insert(serializer, self.jamming_state);
    
    insert(serializer, self.spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.reserved[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, RfErrorDetection& self)
{
    extract(serializer, self.rf_band);
    
    extract(serializer, self.jamming_state);
    
    extract(serializer, self.spoofing_state);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.reserved[i]);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const BaseStationInfo& self)
{
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.ecef_pos[i]);
    
    insert(serializer, self.height);
    
    insert(serializer, self.station_id);
    
    insert(serializer, self.indicators);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, BaseStationInfo& self)
{
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.ecef_pos[i]);
    
    extract(serializer, self.height);
    
    extract(serializer, self.station_id);
    
    extract(serializer, self.indicators);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const RtkCorrectionsStatus& self)
{
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.epoch_status);
    
    insert(serializer, self.dongle_status);
    
    insert(serializer, self.gps_correction_latency);
    
    insert(serializer, self.glonass_correction_latency);
    
    insert(serializer, self.galileo_correction_latency);
    
    insert(serializer, self.beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.reserved[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, RtkCorrectionsStatus& self)
{
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.epoch_status);
    
    extract(serializer, self.dongle_status);
    
    extract(serializer, self.gps_correction_latency);
    
    extract(serializer, self.glonass_correction_latency);
    
    extract(serializer, self.galileo_correction_latency);
    
    extract(serializer, self.beidou_correction_latency);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.reserved[i]);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const SatelliteStatus& self)
{
    insert(serializer, self.index);
    
    insert(serializer, self.count);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.gnss_id);
    
    insert(serializer, self.satellite_id);
    
    insert(serializer, self.elevation);
    
    insert(serializer, self.azimuth);
    
    insert(serializer, self.health);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, SatelliteStatus& self)
{
    extract(serializer, self.index);
    
    extract(serializer, self.count);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.gnss_id);
    
    extract(serializer, self.satellite_id);
    
    extract(serializer, self.elevation);
    
    extract(serializer, self.azimuth);
    
    extract(serializer, self.health);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const Raw& self)
{
    insert(serializer, self.index);
    
    insert(serializer, self.count);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.receiver_id);
    
    insert(serializer, self.tracking_channel);
    
    insert(serializer, self.gnss_id);
    
    insert(serializer, self.satellite_id);
    
    insert(serializer, self.signal_id);
    
    insert(serializer, self.signal_strength);
    
    insert(serializer, self.quality);
    
    insert(serializer, self.pseudorange);
    
    insert(serializer, self.carrier_phase);
    
    insert(serializer, self.doppler);
    
    insert(serializer, self.range_uncert);
    
    insert(serializer, self.phase_uncert);
    
    insert(serializer, self.doppler_uncert);
    
    insert(serializer, self.lock_time);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, Raw& self)
{
    extract(serializer, self.index);
    
    extract(serializer, self.count);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.receiver_id);
    
    extract(serializer, self.tracking_channel);
    
    extract(serializer, self.gnss_id);
    
    extract(serializer, self.satellite_id);
    
    extract(serializer, self.signal_id);
    
    extract(serializer, self.signal_strength);
    
    extract(serializer, self.quality);
    
    extract(serializer, self.pseudorange);
    
    extract(serializer, self.carrier_phase);
    
    extract(serializer, self.doppler);
    
    extract(serializer, self.range_uncert);
    
    extract(serializer, self.phase_uncert);
    
    extract(serializer, self.doppler_uncert);
    
    extract(serializer, self.lock_time);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const GpsEphemeris& self)
{
    insert(serializer, self.index);
    
    insert(serializer, self.count);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.satellite_id);
    
    insert(serializer, self.health);
    
    insert(serializer, self.iodc);
    
    insert(serializer, self.iode);
    
    insert(serializer, self.t_oc);
    
    insert(serializer, self.af0);
    
    insert(serializer, self.af1);
    
    insert(serializer, self.af2);
    
    insert(serializer, self.t_gd);
    
    insert(serializer, self.ISC_L1CA);
    
    insert(serializer, self.ISC_L2C);
    
    insert(serializer, self.t_oe);
    
    insert(serializer, self.a);
    
    insert(serializer, self.a_dot);
    
    insert(serializer, self.mean_anomaly);
    
    insert(serializer, self.delta_mean_motion);
    
    insert(serializer, self.delta_mean_motion_dot);
    
    insert(serializer, self.eccentricity);
    
    insert(serializer, self.argument_of_perigee);
    
    insert(serializer, self.omega);
    
    insert(serializer, self.omega_dot);
    
    insert(serializer, self.inclination);
    
    insert(serializer, self.inclination_dot);
    
    insert(serializer, self.c_ic);
    
    insert(serializer, self.c_is);
    
    insert(serializer, self.c_uc);
    
    insert(serializer, self.c_us);
    
    insert(serializer, self.c_rc);
    
    insert(serializer, self.c_rs);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GpsEphemeris& self)
{
    extract(serializer, self.index);
    
    extract(serializer, self.count);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.satellite_id);
    
    extract(serializer, self.health);
    
    extract(serializer, self.iodc);
    
    extract(serializer, self.iode);
    
    extract(serializer, self.t_oc);
    
    extract(serializer, self.af0);
    
    extract(serializer, self.af1);
    
    extract(serializer, self.af2);
    
    extract(serializer, self.t_gd);
    
    extract(serializer, self.ISC_L1CA);
    
    extract(serializer, self.ISC_L2C);
    
    extract(serializer, self.t_oe);
    
    extract(serializer, self.a);
    
    extract(serializer, self.a_dot);
    
    extract(serializer, self.mean_anomaly);
    
    extract(serializer, self.delta_mean_motion);
    
    extract(serializer, self.delta_mean_motion_dot);
    
    extract(serializer, self.eccentricity);
    
    extract(serializer, self.argument_of_perigee);
    
    extract(serializer, self.omega);
    
    extract(serializer, self.omega_dot);
    
    extract(serializer, self.inclination);
    
    extract(serializer, self.inclination_dot);
    
    extract(serializer, self.c_ic);
    
    extract(serializer, self.c_is);
    
    extract(serializer, self.c_uc);
    
    extract(serializer, self.c_us);
    
    extract(serializer, self.c_rc);
    
    extract(serializer, self.c_rs);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const GalileoEphemeris& self)
{
    insert(serializer, self.index);
    
    insert(serializer, self.count);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.satellite_id);
    
    insert(serializer, self.health);
    
    insert(serializer, self.iodc);
    
    insert(serializer, self.iode);
    
    insert(serializer, self.t_oc);
    
    insert(serializer, self.af0);
    
    insert(serializer, self.af1);
    
    insert(serializer, self.af2);
    
    insert(serializer, self.t_gd);
    
    insert(serializer, self.ISC_L1CA);
    
    insert(serializer, self.ISC_L2C);
    
    insert(serializer, self.t_oe);
    
    insert(serializer, self.a);
    
    insert(serializer, self.a_dot);
    
    insert(serializer, self.mean_anomaly);
    
    insert(serializer, self.delta_mean_motion);
    
    insert(serializer, self.delta_mean_motion_dot);
    
    insert(serializer, self.eccentricity);
    
    insert(serializer, self.argument_of_perigee);
    
    insert(serializer, self.omega);
    
    insert(serializer, self.omega_dot);
    
    insert(serializer, self.inclination);
    
    insert(serializer, self.inclination_dot);
    
    insert(serializer, self.c_ic);
    
    insert(serializer, self.c_is);
    
    insert(serializer, self.c_uc);
    
    insert(serializer, self.c_us);
    
    insert(serializer, self.c_rc);
    
    insert(serializer, self.c_rs);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GalileoEphemeris& self)
{
    extract(serializer, self.index);
    
    extract(serializer, self.count);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.satellite_id);
    
    extract(serializer, self.health);
    
    extract(serializer, self.iodc);
    
    extract(serializer, self.iode);
    
    extract(serializer, self.t_oc);
    
    extract(serializer, self.af0);
    
    extract(serializer, self.af1);
    
    extract(serializer, self.af2);
    
    extract(serializer, self.t_gd);
    
    extract(serializer, self.ISC_L1CA);
    
    extract(serializer, self.ISC_L2C);
    
    extract(serializer, self.t_oe);
    
    extract(serializer, self.a);
    
    extract(serializer, self.a_dot);
    
    extract(serializer, self.mean_anomaly);
    
    extract(serializer, self.delta_mean_motion);
    
    extract(serializer, self.delta_mean_motion_dot);
    
    extract(serializer, self.eccentricity);
    
    extract(serializer, self.argument_of_perigee);
    
    extract(serializer, self.omega);
    
    extract(serializer, self.omega_dot);
    
    extract(serializer, self.inclination);
    
    extract(serializer, self.inclination_dot);
    
    extract(serializer, self.c_ic);
    
    extract(serializer, self.c_is);
    
    extract(serializer, self.c_uc);
    
    extract(serializer, self.c_us);
    
    extract(serializer, self.c_rc);
    
    extract(serializer, self.c_rs);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const GloEphemeris& self)
{
    insert(serializer, self.index);
    
    insert(serializer, self.count);
    
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    insert(serializer, self.satellite_id);
    
    insert(serializer, self.freq_number);
    
    insert(serializer, self.tk);
    
    insert(serializer, self.tb);
    
    insert(serializer, self.sat_type);
    
    insert(serializer, self.gamma);
    
    insert(serializer, self.tau_n);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.x[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.v[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.a[i]);
    
    insert(serializer, self.health);
    
    insert(serializer, self.P);
    
    insert(serializer, self.NT);
    
    insert(serializer, self.delta_tau_n);
    
    insert(serializer, self.Ft);
    
    insert(serializer, self.En);
    
    insert(serializer, self.P1);
    
    insert(serializer, self.P2);
    
    insert(serializer, self.P3);
    
    insert(serializer, self.P4);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GloEphemeris& self)
{
    extract(serializer, self.index);
    
    extract(serializer, self.count);
    
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    extract(serializer, self.satellite_id);
    
    extract(serializer, self.freq_number);
    
    extract(serializer, self.tk);
    
    extract(serializer, self.tb);
    
    extract(serializer, self.sat_type);
    
    extract(serializer, self.gamma);
    
    extract(serializer, self.tau_n);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.x[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.v[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.a[i]);
    
    extract(serializer, self.health);
    
    extract(serializer, self.P);
    
    extract(serializer, self.NT);
    
    extract(serializer, self.delta_tau_n);
    
    extract(serializer, self.Ft);
    
    extract(serializer, self.En);
    
    extract(serializer, self.P1);
    
    extract(serializer, self.P2);
    
    extract(serializer, self.P3);
    
    extract(serializer, self.P4);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const GpsIonoCorr& self)
{
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        insert(serializer, self.beta[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GpsIonoCorr& self)
{
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.alpha[i]);
    
    for(unsigned int i=0; i < 4; i++)
        extract(serializer, self.beta[i]);
    
    extract(serializer, self.valid_flags);
    
}

void insert(Serializer& serializer, const GalileoIonoCorr& self)
{
    insert(serializer, self.time_of_week);
    
    insert(serializer, self.week_number);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.alpha[i]);
    
    insert(serializer, self.disturbance_flags);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, GalileoIonoCorr& self)
{
    extract(serializer, self.time_of_week);
    
    extract(serializer, self.week_number);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.alpha[i]);
    
    extract(serializer, self.disturbance_flags);
    
    extract(serializer, self.valid_flags);
    
}


} // namespace data_gnss
} // namespace mip

