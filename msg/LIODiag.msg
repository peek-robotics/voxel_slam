# LIO Diagnostics Message
# Runtime-varying health + degeneracy signals for Voxel SLAM.
# Keep this message focused on values that change at runtime and help debugging/monitoring.

Header header

# ===== SYSTEM / MODE =====
bool initialized                  # Initialization complete (true after motion_init succeeds)
int32 win_count                   # Sliding-window frame count (monotonic while running)
int32 reset_count                 # Total system resets performed since node start
string last_reset_reason          # Last reset reason (empty if none)

# ===== DEGENERACY / HEALTH =====
int32 degrade_cnt                 # Degeneracy counter (increments on estimation failure; decrements on stable success)
int32 combined_degradation        # degrade_cnt + wheel_odom_violation_count
uint8 degrade_state               # {1,2,4,8,16} derived from combined_degradation (16 triggers reset)
bool estimation_success           # Whether last LiDAR update succeeded

# Motion stability heuristic used alongside degrade_cnt
float32 rotation_angle_deg        # Inter-frame rotation magnitude (deg)
bool motion_stable                # True if rotation under stability threshold

# Basic scan stats
int32 point_cloud_size            # Points in current scan after downsampling
int32 match_count                 # Successful point-to-plane matches (if exposed)

# ===== SIMPLE EIGENVALUE METRIC (Current) =====
# From normal matrix accumulated from plane normals (3x3) in lio_state_estimation.
float32 normal_eigenvalue_min     # Smallest eigenvalue (lower => more degenerate)
float32 normal_observability      # min/max ratio (0..1)

# ===== WHEEL ODOMETRY WATCHDOG (If Enabled) =====
int32 wheel_odom_violation_count  # Increments on violations, cools off (decrements) on non-violations
float32 wheel_odom_diff           # |lio_vx - wheel_vx| (m/s)
float32 wheel_odom_tolerance      # max(|wheel_vx|*ratio, min_tol) (m/s)

# ===== IMU / STATE SANITY =====
float32 gravity_norm              # |g| estimate (m/s^2). Healthy is ~9.8

# # ===== COVARIANCE ADAPTATION =====
# bool covariance_inflated          # True if covariances were inflated due to degeneracy
# float32[3] cov_inflation_pos      # Inflation applied to x-position/velocity
# float32[3] cov_inflation_rot      # Inflation applied to roll/pitch/yaw

# ===== HESSIAN-BASED DIRECTIONAL DEGENERACY =====
# Filled when we analyze the BA Hessian to detect weak directions (per D^2-LIO-style metrics).
# float32 hessian_trans_eig_min
# float32 hessian_trans_obs_ratio

# float32 hessian_rot_eig_min
# float32 hessian_rot_obs_ratio

# geometry_msgs/Vector3 degen_direction_pos  # Unit vector, most degenerate translation direction
# geometry_msgs/Vector3 degen_direction_rot    # Unit vector, most degenerate rotation axis
# float32[3] axis_degeneracy                    # |dot(degen_dir, x)| (0..1)

# ===== TIMING =====
float32 processing_time_ms        # End-to-end processing time for this scan (ms)
