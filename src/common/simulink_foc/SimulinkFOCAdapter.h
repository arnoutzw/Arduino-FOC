#ifndef SIMULINK_FOC_ADAPTER_H
#define SIMULINK_FOC_ADAPTER_H

#include "Arduino.h"
#include "SimulinkFOC.h"
#include "SimulinkFOCTypes.h"
#include "SimulinkFOCConfig.h"
#include "../base_classes/FOCMotor.h"
#include "../base_classes/CurrentSense.h"
#include "../base_classes/Sensor.h"
#include "../base_classes/BLDCDriver.h"
#include "../time_utils.h"

/**
 * SimulinkFOCAdapter
 *
 * This adapter bridges the SimpleFOC motor infrastructure with the
 * Simulink code-generated FOC algorithm. It handles:
 * - Converting SimpleFOC data structures to Simulink format
 * - Managing timing and step execution
 * - Routing outputs to the driver
 *
 * Usage within BLDCMotor:
 *   SimulinkFOCAdapter adapter;
 *   adapter.init(this);  // Pass motor pointer
 *
 *   // In loopFOC:
 *   adapter.runLoopFOC();
 *
 *   // In move:
 *   adapter.runMove(target);
 */
class SimulinkFOCAdapter {
public:
    /**
     * Constructor
     */
    SimulinkFOCAdapter();

    /**
     * Initialize the adapter with motor reference
     * @param motor Pointer to the FOCMotor instance
     * @param driver Pointer to the BLDC driver
     * @param sensor Pointer to the position sensor (can be null)
     * @param current_sense Pointer to current sensing (can be null)
     * @return 0 on success, non-zero on failure
     */
    int init(FOCMotor* motor, BLDCDriver* driver, Sensor* sensor, CurrentSense* current_sense);

    /**
     * Synchronize parameters from motor to Simulink component
     * Call this after changing motor parameters
     */
    void syncParameters();

    /**
     * Run the inner FOC loop (equivalent to loopFOC)
     * Call this at high frequency
     */
    void runLoopFOC();

    /**
     * Run the outer control loop (equivalent to move)
     * Call this at control rate
     * @param target The control target
     */
    void runMove(float target);

    /**
     * Reset the FOC algorithm state
     */
    void reset();

    /**
     * Check if adapter is initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Get reference to the underlying SimulinkFOC component
     */
    SimulinkFOC& getFOC() { return foc_; }
    const SimulinkFOC& getFOC() const { return foc_; }

    /**
     * Get the last calculated phase voltages
     */
    float getUa() const { return foc_.getUa(); }
    float getUb() const { return foc_.getUb(); }
    float getUc() const { return foc_.getUc(); }

    /**
     * Get phase states for driver
     */
    uint8_t getPhaseStateA() const { return foc_.getPhaseStateA(); }
    uint8_t getPhaseStateB() const { return foc_.getPhaseStateB(); }
    uint8_t getPhaseStateC() const { return foc_.getPhaseStateC(); }

private:
    /**
     * Convert motor parameters to Simulink format
     */
    void buildMotorParams(SimulinkFOC_MotorParams_T* params);

    /**
     * Convert sensor calibration to Simulink format
     */
    void buildSensorCalib(SimulinkFOC_SensorCalib_T* calib);

    /**
     * Convert control gains to Simulink format
     */
    void buildControlGains(SimulinkFOC_ControlGains_T* gains);

    /**
     * Convert limits to Simulink format
     */
    void buildLimits(SimulinkFOC_Limits_T* limits);

    /**
     * Convert control config to Simulink format
     */
    void buildControlConfig(SimulinkFOC_ControlConfig_T* config);

    /**
     * Build full parameters structure
     */
    void buildParams(SimulinkFOC_Params_T* params);

    /**
     * Build inputs structure from current motor state
     */
    void buildInputs(SimulinkFOC_Inputs_T* inputs, float target);

    /**
     * Apply outputs to motor state and driver
     */
    void applyOutputs(const SimulinkFOC_Outputs_T& outputs);

    /**
     * Convert PhaseState enum to driver format
     */
    PhaseState convertPhaseState(uint8_t state);

    // References to motor components
    FOCMotor* motor_;
    BLDCDriver* driver_;
    Sensor* sensor_;
    CurrentSense* current_sense_;

    // The Simulink FOC component
    SimulinkFOC foc_;

    // Timing
    unsigned long last_loop_timestamp_;
    unsigned long last_move_timestamp_;

    // Initialization flag
    bool initialized_;
};

// ============================================================================
// Inline Implementation
// ============================================================================

inline SimulinkFOCAdapter::SimulinkFOCAdapter()
    : motor_(nullptr)
    , driver_(nullptr)
    , sensor_(nullptr)
    , current_sense_(nullptr)
    , last_loop_timestamp_(0)
    , last_move_timestamp_(0)
    , initialized_(false)
{
}

inline int SimulinkFOCAdapter::init(FOCMotor* motor, BLDCDriver* driver,
                                    Sensor* sensor, CurrentSense* current_sense) {
    if (motor == nullptr || driver == nullptr) {
        return -1;
    }

    motor_ = motor;
    driver_ = driver;
    sensor_ = sensor;
    current_sense_ = current_sense;

    // Build and set parameters
    SimulinkFOC_Params_T params;
    buildParams(&params);

    int result = foc_.init(&params);
    if (result != 0) {
        return result;
    }

    last_loop_timestamp_ = _micros();
    last_move_timestamp_ = _micros();
    initialized_ = true;

    return 0;
}

inline void SimulinkFOCAdapter::syncParameters() {
    if (!initialized_) return;

    SimulinkFOC_Params_T params;
    buildParams(&params);
    foc_.updateParams(&params);
}

inline void SimulinkFOCAdapter::runLoopFOC() {
    if (!initialized_) return;

    // Update sensor if available
    if (sensor_) sensor_->update();

    // Skip for open-loop modes
    if (motor_->controller == MotionControlType::angle_openloop ||
        motor_->controller == MotionControlType::velocity_openloop) {
        return;
    }

    // Skip if disabled
    if (!motor_->enabled) return;

    // Calculate dt
    unsigned long now = _micros();
    float dt = (now - last_loop_timestamp_) * 1e-6f;
    if (dt <= 0 || dt > 0.5f) dt = 1e-3f;
    last_loop_timestamp_ = now;

    // Build inputs
    SimulinkFOC_Inputs_T inputs;
    buildInputs(&inputs, motor_->target);
    inputs.dt = dt;

    // Set inputs and run inner loop
    foc_.setInputs(&inputs);
    foc_.runInnerLoop();

    // Get outputs and apply to motor/driver
    SimulinkFOC_Outputs_T outputs;
    foc_.getOutputs(&outputs);
    applyOutputs(outputs);
}

inline void SimulinkFOCAdapter::runMove(float target) {
    if (!initialized_) return;

    // Calculate dt
    unsigned long now = _micros();
    float dt = (now - last_move_timestamp_) * 1e-6f;
    if (dt <= 0 || dt > 0.5f) dt = 1e-3f;
    last_move_timestamp_ = now;

    // Update shaft angle/velocity from sensor
    if (motor_->controller != MotionControlType::angle_openloop &&
        motor_->controller != MotionControlType::velocity_openloop) {
        if (sensor_) {
            motor_->shaft_angle = motor_->shaftAngle();
        }
    }
    motor_->shaft_velocity = motor_->shaftVelocity();

    // Skip if disabled
    if (!motor_->enabled) return;

    // Build inputs
    SimulinkFOC_Inputs_T inputs;
    buildInputs(&inputs, target);
    inputs.dt = dt;

    // Set inputs and run outer loop
    foc_.setInputs(&inputs);
    foc_.runOuterLoop();

    // Get outputs and apply
    SimulinkFOC_Outputs_T outputs;
    foc_.getOutputs(&outputs);

    // Update motor state from outputs
    motor_->current_sp = outputs.current_sp;
    motor_->shaft_velocity_sp = outputs.shaft_velocity_sp;
    motor_->shaft_angle_sp = outputs.shaft_angle_sp;
    motor_->voltage.q = outputs.voltage_q;
    motor_->voltage.d = outputs.voltage_d;
    motor_->voltage_bemf = outputs.voltage_bemf;

    // For open-loop modes, apply phase voltages here
    if (motor_->controller == MotionControlType::angle_openloop ||
        motor_->controller == MotionControlType::velocity_openloop) {
        applyOutputs(outputs);
        motor_->shaft_angle = outputs.shaft_angle;
    }
}

inline void SimulinkFOCAdapter::reset() {
    if (initialized_) {
        foc_.reset();
        last_loop_timestamp_ = _micros();
        last_move_timestamp_ = _micros();
    }
}

inline void SimulinkFOCAdapter::buildMotorParams(SimulinkFOC_MotorParams_T* params) {
    params->phase_resistance = motor_->phase_resistance;
    params->phase_inductance = motor_->phase_inductance;
    params->pole_pairs = (float)motor_->pole_pairs;
    params->KV_rating = motor_->KV_rating;
}

inline void SimulinkFOCAdapter::buildSensorCalib(SimulinkFOC_SensorCalib_T* calib) {
    calib->zero_electric_angle = motor_->zero_electric_angle;
    calib->sensor_direction = (int8_t)motor_->sensor_direction;
    calib->sensor_offset = motor_->sensor_offset;
}

inline void SimulinkFOCAdapter::buildControlGains(SimulinkFOC_ControlGains_T* gains) {
    // Q-axis current PID
    gains->current_q_P = motor_->PID_current_q.P;
    gains->current_q_I = motor_->PID_current_q.I;
    gains->current_q_D = motor_->PID_current_q.D;
    gains->current_q_ramp = motor_->PID_current_q.output_ramp;
    gains->current_q_limit = motor_->PID_current_q.limit;

    // D-axis current PID
    gains->current_d_P = motor_->PID_current_d.P;
    gains->current_d_I = motor_->PID_current_d.I;
    gains->current_d_D = motor_->PID_current_d.D;
    gains->current_d_ramp = motor_->PID_current_d.output_ramp;
    gains->current_d_limit = motor_->PID_current_d.limit;

    // Velocity PID
    gains->velocity_P = motor_->PID_velocity.P;
    gains->velocity_I = motor_->PID_velocity.I;
    gains->velocity_D = motor_->PID_velocity.D;
    gains->velocity_ramp = motor_->PID_velocity.output_ramp;
    gains->velocity_limit = motor_->PID_velocity.limit;

    // Position P controller
    gains->angle_P = motor_->P_angle.P;
    gains->angle_limit = motor_->P_angle.limit;

    // Low-pass filters
    gains->current_q_filter_Tf = motor_->LPF_current_q.Tf;
    gains->current_d_filter_Tf = motor_->LPF_current_d.Tf;
    gains->velocity_filter_Tf = motor_->LPF_velocity.Tf;
    gains->angle_filter_Tf = motor_->LPF_angle.Tf;
}

inline void SimulinkFOCAdapter::buildLimits(SimulinkFOC_Limits_T* limits) {
    limits->voltage_limit = motor_->voltage_limit;
    limits->current_limit = motor_->current_limit;
    limits->velocity_limit = motor_->velocity_limit;
    limits->voltage_power_supply = driver_->voltage_power_supply;
}

inline void SimulinkFOCAdapter::buildControlConfig(SimulinkFOC_ControlConfig_T* config) {
    // Torque mode
    switch (motor_->torque_controller) {
        case TorqueControlType::voltage:
            config->torque_mode = SFOC_TORQUE_VOLTAGE;
            break;
        case TorqueControlType::dc_current:
            config->torque_mode = SFOC_TORQUE_DC_CURRENT;
            break;
        case TorqueControlType::foc_current:
            config->torque_mode = SFOC_TORQUE_FOC_CURRENT;
            break;
        default:
            config->torque_mode = SFOC_TORQUE_VOLTAGE;
            break;
    }

    // Motion mode
    switch (motor_->controller) {
        case MotionControlType::torque:
            config->motion_mode = SFOC_MOTION_TORQUE;
            break;
        case MotionControlType::velocity:
            config->motion_mode = SFOC_MOTION_VELOCITY;
            break;
        case MotionControlType::angle:
            config->motion_mode = SFOC_MOTION_ANGLE;
            break;
        case MotionControlType::velocity_openloop:
            config->motion_mode = SFOC_MOTION_VELOCITY_OL;
            break;
        case MotionControlType::angle_openloop:
            config->motion_mode = SFOC_MOTION_ANGLE_OL;
            break;
        default:
            config->motion_mode = SFOC_MOTION_TORQUE;
            break;
    }

    // Modulation type
    switch (motor_->foc_modulation) {
        case FOCModulationType::SinePWM:
            config->modulation_type = SFOC_MOD_SINE_PWM;
            break;
        case FOCModulationType::SpaceVectorPWM:
            config->modulation_type = SFOC_MOD_SPACE_VECTOR;
            break;
        case FOCModulationType::Trapezoid_120:
            config->modulation_type = SFOC_MOD_TRAPEZOID_120;
            break;
        case FOCModulationType::Trapezoid_150:
            config->modulation_type = SFOC_MOD_TRAPEZOID_150;
            break;
        default:
            config->modulation_type = SFOC_MOD_SINE_PWM;
            break;
    }

    config->modulation_centered = motor_->modulation_centered;
    config->motion_downsample = motor_->motion_downsample;
}

inline void SimulinkFOCAdapter::buildParams(SimulinkFOC_Params_T* params) {
    buildMotorParams(&params->motor);
    buildSensorCalib(&params->sensor);
    buildControlGains(&params->gains);
    buildLimits(&params->limits);
    buildControlConfig(&params->config);
}

inline void SimulinkFOCAdapter::buildInputs(SimulinkFOC_Inputs_T* inputs, float target) {
    memset(inputs, 0, sizeof(SimulinkFOC_Inputs_T));

    // Sensor inputs
    if (sensor_) {
        inputs->mechanical_angle = sensor_->getMechanicalAngle();
        inputs->shaft_velocity = motor_->shaft_velocity;
    }

    // Current measurements
    if (current_sense_) {
        PhaseCurrent_s currents = current_sense_->getPhaseCurrents();
        inputs->phase_current_a = currents.a;
        inputs->phase_current_b = currents.b;
        inputs->phase_current_c = currents.c;
        inputs->current_sense_available = 1;
    } else {
        inputs->current_sense_available = 0;
    }

    // Target and feedforward
    inputs->target = target;
    inputs->feed_forward_velocity = motor_->feed_forward_velocity;

    // Enable flag
    inputs->enabled = motor_->enabled;
}

inline void SimulinkFOCAdapter::applyOutputs(const SimulinkFOC_Outputs_T& outputs) {
    // Update motor state
    motor_->electrical_angle = outputs.electrical_angle;
    motor_->Ualpha = outputs.Ualpha;
    motor_->Ubeta = outputs.Ubeta;
    motor_->voltage.q = outputs.voltage_q;
    motor_->voltage.d = outputs.voltage_d;
    motor_->current.q = outputs.current_q;
    motor_->current.d = outputs.current_d;

    // Set phase states for driver
    driver_->setPhaseState(
        convertPhaseState(outputs.phase_state_a),
        convertPhaseState(outputs.phase_state_b),
        convertPhaseState(outputs.phase_state_c)
    );

    // Set phase voltages to driver
    driver_->setPwm(outputs.Ua, outputs.Ub, outputs.Uc);
}

inline PhaseState SimulinkFOCAdapter::convertPhaseState(uint8_t state) {
    return state ? PhaseState::PHASE_ON : PhaseState::PHASE_OFF;
}

#endif // SIMULINK_FOC_ADAPTER_H
