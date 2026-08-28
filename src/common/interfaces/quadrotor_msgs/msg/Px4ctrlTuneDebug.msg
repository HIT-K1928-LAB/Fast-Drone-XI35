# One controller-cycle snapshot for PID tuning and disturbance-response analysis.
Header header

# PX4CtrlFSM::State_t numeric value.
uint8 FSM_MANUAL_CTRL=1
uint8 FSM_AUTO_HOVER=2
uint8 FSM_CMD_CTRL=3
uint8 FSM_AUTO_TAKEOFF=4
uint8 FSM_AUTO_LAND=5
uint8 fsm_state

geometry_msgs/Point position_des
geometry_msgs/Point position_meas
geometry_msgs/Vector3 position_error

geometry_msgs/Vector3 velocity_des
geometry_msgs/Vector3 velocity_meas
geometry_msgs/Vector3 velocity_filtered
geometry_msgs/Vector3 velocity_error

geometry_msgs/Vector3 acceleration_feedforward
geometry_msgs/Vector3 gravity_compensation
geometry_msgs/Vector3 acceleration_proportional
geometry_msgs/Vector3 acceleration_derivative
geometry_msgs/Vector3 acceleration_feedback
geometry_msgs/Vector3 acceleration_integral
# Before and after configured acceleration limits.
geometry_msgs/Vector3 acceleration_command_raw
geometry_msgs/Vector3 acceleration_command

# IMU linear acceleration/specific-force samples in the body frame [m/s^2].
geometry_msgs/Vector3 specific_force_body_meas
geometry_msgs/Vector3 specific_force_body_filtered

# The current controller integrates velocity error, not position error.
geometry_msgs/Vector3 velocity_error_integral

geometry_msgs/Vector3 attitude_des_rpy
geometry_msgs/Vector3 attitude_meas_rpy
geometry_msgs/Vector3 attitude_error_rpy
geometry_msgs/Vector3 body_rate_meas

geometry_msgs/Vector3 kp
geometry_msgs/Vector3 ki
geometry_msgs/Vector3 kd

float64 thrust_raw
float64 thrust_command
float64 thrust_to_accel
float64 tilt_raw
float64 tilt_command

# These report whether the raw request was constrained by the controller.
bool acceleration_limit_exceeded
bool tilt_limit_exceeded
bool thrust_limit_exceeded

float64 control_dt
