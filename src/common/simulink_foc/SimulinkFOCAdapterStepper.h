#ifndef SIMULINK_FOC_ADAPTER_STEPPER_H
#define SIMULINK_FOC_ADAPTER_STEPPER_H

#include "Arduino.h"
#include "SimulinkFOC.h"
#include "SimulinkFOCTypes.h"
#include "SimulinkFOCConfig.h"
#include "../base_classes/FOCMotor.h"
#include "../base_classes/CurrentSense.h"
#include "../base_classes/Sensor.h"
#include "../base_classes/StepperDriver.h"
#include "../time_utils.h"

/**
 * SimulinkFOCAdapterStepper
 *
 * This adapter bridges the SimpleFOC stepper motor infrastructure with the
 * Simulink code-generated FOC algorithm.
 *
 * NOTE: This is a placeholder implementation. The 2-phase stepper motor
 * support requires a modified Simulink model that outputs 2 phases (Ualpha, Ubeta)
 * directly instead of 3-phase voltages (Ua, Ub, Uc).
 *
 * For stepper motors, the setPhaseVoltage function outputs Ualpha and Ubeta
 * directly to the 2-phase driver, so the Simulink model should be configured
 * to skip the inverse Clarke transform.
 */
class SimulinkFOCAdapterStepper {
public:
    SimulinkFOCAdapterStepper() : initialized_(false) {}

    /**
     * Initialize the adapter
     * @return 0 on success, -1 if 2-phase mode not yet supported
     */
    int init(FOCMotor* motor, StepperDriver* driver, Sensor* sensor, CurrentSense* current_sense) {
        motor_ = motor;
        driver_ = driver;
        sensor_ = sensor;
        current_sense_ = current_sense;

        // Build parameters
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

    void syncParameters() {
        if (!initialized_) return;
        SimulinkFOC_Params_T params;
        buildParams(&params);
        foc_.updateParams(&params);
    }

    void runLoopFOC() {
        if (!initialized_) return;

        // Update sensor if available
        if (sensor_) sensor_->update();

        // Skip for open-loop modes
        if (motor_->controller == MotionControlType::angle_openloop ||
            motor_->controller == MotionControlType::velocity_openloop) {
            return;
        }

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

        // Get outputs and apply to stepper (uses Ualpha, Ubeta directly)
        SimulinkFOC_Outputs_T outputs;
        foc_.getOutputs(&outputs);
        applyOutputs(outputs);
    }

    void runMove(float target) {
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

        if (!motor_->enabled) return;

        // Build inputs
        SimulinkFOC_Inputs_T inputs;
        buildInputs(&inputs, target);
        inputs.dt = dt;

        // Set inputs and run outer loop
        foc_.setInputs(&inputs);
        foc_.runOuterLoop();

        // Get outputs and update motor state
        SimulinkFOC_Outputs_T outputs;
        foc_.getOutputs(&outputs);

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

    void reset() {
        if (initialized_) {
            foc_.reset();
            last_loop_timestamp_ = _micros();
            last_move_timestamp_ = _micros();
        }
    }

    bool isInitialized() const { return initialized_; }
    SimulinkFOC& getFOC() { return foc_; }
    const SimulinkFOC& getFOC() const { return foc_; }

    // For stepper, we use Ualpha/Ubeta directly
    float getUalpha() const { return foc_.getFOC().getOutputs ? 0 : 0; }
    float getUbeta() const { return foc_.getFOC().getOutputs ? 0 : 0; }

private:
    void buildParams(SimulinkFOC_Params_T* params) {
        // Motor params
        params->motor.phase_resistance = motor_->phase_resistance;
        params->motor.phase_inductance = motor_->phase_inductance;
        params->motor.pole_pairs = (float)motor_->pole_pairs;
        params->motor.KV_rating = motor_->KV_rating;

        // Sensor calibration
        params->sensor.zero_electric_angle = motor_->zero_electric_angle;
        params->sensor.sensor_direction = (int8_t)motor_->sensor_direction;
        params->sensor.sensor_offset = motor_->sensor_offset;

        // Control gains
        params->gains.current_q_P = motor_->PID_current_q.P;
        params->gains.current_q_I = motor_->PID_current_q.I;
        params->gains.current_q_D = motor_->PID_current_q.D;
        params->gains.current_q_ramp = motor_->PID_current_q.output_ramp;
        params->gains.current_q_limit = motor_->PID_current_q.limit;
        params->gains.current_d_P = motor_->PID_current_d.P;
        params->gains.current_d_I = motor_->PID_current_d.I;
        params->gains.current_d_D = motor_->PID_current_d.D;
        params->gains.current_d_ramp = motor_->PID_current_d.output_ramp;
        params->gains.current_d_limit = motor_->PID_current_d.limit;
        params->gains.velocity_P = motor_->PID_velocity.P;
        params->gains.velocity_I = motor_->PID_velocity.I;
        params->gains.velocity_D = motor_->PID_velocity.D;
        params->gains.velocity_ramp = motor_->PID_velocity.output_ramp;
        params->gains.velocity_limit = motor_->PID_velocity.limit;
        params->gains.angle_P = motor_->P_angle.P;
        params->gains.angle_limit = motor_->P_angle.limit;
        params->gains.current_q_filter_Tf = motor_->LPF_current_q.Tf;
        params->gains.current_d_filter_Tf = motor_->LPF_current_d.Tf;
        params->gains.velocity_filter_Tf = motor_->LPF_velocity.Tf;
        params->gains.angle_filter_Tf = motor_->LPF_angle.Tf;

        // Limits
        params->limits.voltage_limit = motor_->voltage_limit;
        params->limits.current_limit = motor_->current_limit;
        params->limits.velocity_limit = motor_->velocity_limit;
        params->limits.voltage_power_supply = driver_->voltage_power_supply;

        // Control config
        params->config.torque_mode = (SimulinkFOC_TorqueMode_T)motor_->torque_controller;
        params->config.motion_mode = (SimulinkFOC_MotionMode_T)motor_->controller;
        // Stepper only uses SinePWM (no trapezoidal)
        params->config.modulation_type = SFOC_MOD_SINE_PWM;
        params->config.modulation_centered = motor_->modulation_centered;
        params->config.motion_downsample = motor_->motion_downsample;
    }

    void buildInputs(SimulinkFOC_Inputs_T* inputs, float target) {
        memset(inputs, 0, sizeof(SimulinkFOC_Inputs_T));

        if (sensor_) {
            inputs->mechanical_angle = sensor_->getMechanicalAngle();
            inputs->shaft_velocity = motor_->shaft_velocity;
        }

        if (current_sense_) {
            PhaseCurrent_s currents = current_sense_->getPhaseCurrents();
            inputs->phase_current_a = currents.a;
            inputs->phase_current_b = currents.b;
            inputs->phase_current_c = currents.c;
            inputs->current_sense_available = 1;
        }

        inputs->target = target;
        inputs->feed_forward_velocity = motor_->feed_forward_velocity;
        inputs->enabled = motor_->enabled;
    }

    void applyOutputs(const SimulinkFOC_Outputs_T& outputs) {
        // Update motor state
        motor_->electrical_angle = outputs.electrical_angle;
        motor_->Ualpha = outputs.Ualpha;
        motor_->Ubeta = outputs.Ubeta;
        motor_->voltage.q = outputs.voltage_q;
        motor_->voltage.d = outputs.voltage_d;
        motor_->current.q = outputs.current_q;
        motor_->current.d = outputs.current_d;

        // For stepper, output Ualpha and Ubeta directly to the 2-phase driver
        driver_->setPwm(outputs.Ualpha, outputs.Ubeta);
    }

    FOCMotor* motor_;
    StepperDriver* driver_;
    Sensor* sensor_;
    CurrentSense* current_sense_;
    SimulinkFOC foc_;
    unsigned long last_loop_timestamp_;
    unsigned long last_move_timestamp_;
    bool initialized_;
};

#endif // SIMULINK_FOC_ADAPTER_STEPPER_H
