# Message definition for the Status Flags (GX5, CV5, CX5, etc.) field of https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_status.htm?Highlight=filter%20status
#   Note: This message will never be published on it's own, only included in other messages.

bool init_no_attitude
bool init_no_position_velocity
bool run_imu_unavailable
bool run_gps_unavailable
bool run_matrix_singularity
bool run_position_covariance_warning
bool run_velocity_covariance_warning
bool run_attitude_covariance_warning
bool run_nan_in_solution_warning
bool run_gyro_bias_est_high_warning
bool run_accel_bias_est_high_warning
bool run_gyro_scale_factor_est_high_warning
bool run_accel_scale_factor_est_high_warning
bool run_mag_bias_est_high_warning
bool run_ant_offset_correction_est_high_warning
bool run_mag_hard_iron_est_high_warning
bool run_mag_soft_iron_est_high_warning