#ifndef SIMULINK_FOC_TYPES_H
#define SIMULINK_FOC_TYPES_H

#include "Arduino.h"
#include "../foc_utils.h"

/**
 * Simulink FOC Types
 *
 * This header defines all data structures required for the Simulink code-generated
 * FOC algorithm interface. These structures match the Simulink model's bus definitions.
 */

/**
 * Motor parameters structure
 * Contains physical motor characteristics needed for FOC calculations
 */
typedef struct {
    float phase_resistance;     // Motor phase resistance [Ohm]
    float phase_inductance;     // Motor phase inductance [H]
    float pole_pairs;           // Number of pole pairs
    float KV_rating;            // Motor KV rating [rpm/V]
} SimulinkFOC_MotorParams_T;

/**
 * Sensor calibration parameters
 * Contains sensor alignment and direction information
 */
typedef struct {
    float zero_electric_angle;  // Electrical zero offset [rad]
    int8_t sensor_direction;    // Sensor direction: 1=CW, -1=CCW
    float sensor_offset;        // User-defined sensor offset [rad]
} SimulinkFOC_SensorCalib_T;

/**
 * Control loop gains structure
 * Contains all PID and filter parameters for the control loops
 */
typedef struct {
    // Q-axis current PID
    float current_q_P;
    float current_q_I;
    float current_q_D;
    float current_q_ramp;
    float current_q_limit;

    // D-axis current PID
    float current_d_P;
    float current_d_I;
    float current_d_D;
    float current_d_ramp;
    float current_d_limit;

    // Velocity PID
    float velocity_P;
    float velocity_I;
    float velocity_D;
    float velocity_ramp;
    float velocity_limit;

    // Position P controller
    float angle_P;
    float angle_limit;

    // Low-pass filter time constants
    float current_q_filter_Tf;
    float current_d_filter_Tf;
    float velocity_filter_Tf;
    float angle_filter_Tf;
} SimulinkFOC_ControlGains_T;

/**
 * System limits structure
 */
typedef struct {
    float voltage_limit;        // Maximum voltage [V]
    float current_limit;        // Maximum current [A]
    float velocity_limit;       // Maximum velocity [rad/s]
    float voltage_power_supply; // Power supply voltage [V]
} SimulinkFOC_Limits_T;

/**
 * Control mode enumerations
 */
typedef enum {
    SFOC_TORQUE_VOLTAGE = 0,    // Torque control using voltage
    SFOC_TORQUE_DC_CURRENT = 1, // Torque control using DC current
    SFOC_TORQUE_FOC_CURRENT = 2 // Torque control using DQ currents
} SimulinkFOC_TorqueMode_T;

typedef enum {
    SFOC_MOTION_TORQUE = 0,         // Torque control
    SFOC_MOTION_VELOCITY = 1,       // Velocity control
    SFOC_MOTION_ANGLE = 2,          // Position control
    SFOC_MOTION_VELOCITY_OL = 3,    // Open-loop velocity
    SFOC_MOTION_ANGLE_OL = 4        // Open-loop position
} SimulinkFOC_MotionMode_T;

typedef enum {
    SFOC_MOD_SINE_PWM = 0,          // Sinusoidal PWM
    SFOC_MOD_SPACE_VECTOR = 1,      // Space vector PWM
    SFOC_MOD_TRAPEZOID_120 = 2,     // 120° trapezoidal
    SFOC_MOD_TRAPEZOID_150 = 3      // 150° trapezoidal
} SimulinkFOC_ModulationType_T;

/**
 * Control configuration structure
 */
typedef struct {
    SimulinkFOC_TorqueMode_T torque_mode;
    SimulinkFOC_MotionMode_T motion_mode;
    SimulinkFOC_ModulationType_T modulation_type;
    uint8_t modulation_centered;    // 1=centered, 0=pulled to zero
    uint32_t motion_downsample;     // Downsampling ratio for move command
} SimulinkFOC_ControlConfig_T;

/**
 * FOC Algorithm Inputs structure
 * All inputs required by the Simulink FOC algorithm per execution step
 */
typedef struct {
    // Sensor inputs
    float mechanical_angle;     // Mechanical angle from sensor [rad]
    float shaft_velocity;       // Shaft velocity [rad/s]

    // Current measurements (from current sense)
    float phase_current_a;      // Phase A current [A]
    float phase_current_b;      // Phase B current [A]
    float phase_current_c;      // Phase C current [A]
    uint8_t current_sense_available; // Flag: current sensing enabled

    // Control target
    float target;               // Target value (voltage/current/velocity/angle)
    float feed_forward_velocity; // Optional velocity feedforward [rad/s]

    // Timing
    float dt;                   // Time step [s]

    // Enable flag
    uint8_t enabled;            // Motor enabled flag
} SimulinkFOC_Inputs_T;

/**
 * FOC Algorithm Outputs structure
 * All outputs produced by the Simulink FOC algorithm
 */
typedef struct {
    // Phase voltages for driver
    float Ua;                   // Phase A voltage [V]
    float Ub;                   // Phase B voltage [V]
    float Uc;                   // Phase C voltage [V]

    // Phase states for driver (for trapezoidal modes)
    uint8_t phase_state_a;      // Phase A state (0=OFF, 1=ON)
    uint8_t phase_state_b;      // Phase B state
    uint8_t phase_state_c;      // Phase C state

    // Calculated angles
    float electrical_angle;     // Calculated electrical angle [rad]

    // Intermediate values (for monitoring/debugging)
    float Ualpha;               // Alpha voltage component [V]
    float Ubeta;                // Beta voltage component [V]
    float voltage_q;            // Q-axis voltage [V]
    float voltage_d;            // D-axis voltage [V]

    // Measured/filtered currents
    float current_q;            // Q-axis current [A]
    float current_d;            // D-axis current [A]

    // Control loop outputs
    float current_sp;           // Current setpoint [A]
    float shaft_velocity_sp;    // Velocity setpoint [rad/s]
    float shaft_angle_sp;       // Angle setpoint [rad]

    // Estimated values
    float voltage_bemf;         // Estimated back-EMF voltage [V]
    float shaft_angle;          // Calculated shaft angle [rad] (for open-loop)
} SimulinkFOC_Outputs_T;

/**
 * FOC Algorithm Internal State structure
 * Persistent state variables maintained between algorithm calls
 */
typedef struct {
    // PID controller states - Q current
    float pid_current_q_integral;
    float pid_current_q_prev_error;
    float pid_current_q_prev_output;

    // PID controller states - D current
    float pid_current_d_integral;
    float pid_current_d_prev_error;
    float pid_current_d_prev_output;

    // PID controller states - Velocity
    float pid_velocity_integral;
    float pid_velocity_prev_error;
    float pid_velocity_prev_output;

    // P controller states - Angle
    float p_angle_prev_output;

    // Low-pass filter states
    float lpf_current_q_prev;
    float lpf_current_d_prev;
    float lpf_velocity_prev;
    float lpf_angle_prev;

    // Open-loop state
    float open_loop_angle;
    uint32_t open_loop_timestamp;

    // Downsampling counter
    uint32_t motion_counter;

    // Initialization flag
    uint8_t initialized;
} SimulinkFOC_State_T;

/**
 * Complete FOC Algorithm Parameters Bundle
 * All configuration needed to initialize and run the FOC algorithm
 */
typedef struct {
    SimulinkFOC_MotorParams_T motor;
    SimulinkFOC_SensorCalib_T sensor;
    SimulinkFOC_ControlGains_T gains;
    SimulinkFOC_Limits_T limits;
    SimulinkFOC_ControlConfig_T config;
} SimulinkFOC_Params_T;

#endif // SIMULINK_FOC_TYPES_H
