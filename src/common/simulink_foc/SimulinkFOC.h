#ifndef SIMULINK_FOC_H
#define SIMULINK_FOC_H

#include "Arduino.h"
#include "SimulinkFOCTypes.h"
#include "../foc_utils.h"

/**
 * SimulinkFOC - Code-Generated FOC Algorithm Component
 *
 * This class provides the interface for the Simulink code-generated
 * Field Oriented Control algorithm. It encapsulates all FOC calculations
 * including:
 * - Clarke and Park transforms
 * - Current control loops (PID)
 * - Velocity control loops (PID)
 * - Position control loops (P)
 * - PWM modulation (Sine, Space Vector, Trapezoidal)
 * - Open-loop control modes
 *
 * Usage:
 *   SimulinkFOC foc;
 *   foc.init(&params);
 *
 *   // In control loop:
 *   foc.setInputs(&inputs);
 *   foc.step();
 *   foc.getOutputs(&outputs);
 *
 * Note: This is a template implementation with dummy interfaces.
 * Replace the step() implementation with actual Simulink-generated code.
 */
class SimulinkFOC {
public:
    /**
     * Default constructor
     */
    SimulinkFOC();

    /**
     * Destructor
     */
    ~SimulinkFOC();

    /**
     * Initialize the FOC algorithm with parameters
     * @param params Pointer to the complete parameter structure
     * @return 0 on success, non-zero on failure
     */
    int init(const SimulinkFOC_Params_T* params);

    /**
     * Reset the FOC algorithm state to initial conditions
     * Call this when re-enabling the motor or changing modes
     */
    void reset();

    /**
     * Set inputs for the next algorithm step
     * @param inputs Pointer to the input structure
     */
    void setInputs(const SimulinkFOC_Inputs_T* inputs);

    /**
     * Execute one step of the FOC algorithm
     * This is the main entry point that would be replaced with
     * Simulink code-generated function
     */
    void step();

    /**
     * Get outputs from the last algorithm step
     * @param outputs Pointer to output structure to fill
     */
    void getOutputs(SimulinkFOC_Outputs_T* outputs) const;

    /**
     * Update parameters at runtime
     * @param params Pointer to new parameters
     */
    void updateParams(const SimulinkFOC_Params_T* params);

    /**
     * Update only the control gains at runtime
     * @param gains Pointer to new control gains
     */
    void updateGains(const SimulinkFOC_ControlGains_T* gains);

    /**
     * Update only the control configuration at runtime
     * @param config Pointer to new control configuration
     */
    void updateConfig(const SimulinkFOC_ControlConfig_T* config);

    /**
     * Update sensor calibration values at runtime
     * @param calib Pointer to new sensor calibration
     */
    void updateSensorCalib(const SimulinkFOC_SensorCalib_T* calib);

    /**
     * Update system limits at runtime
     * @param limits Pointer to new limits
     */
    void updateLimits(const SimulinkFOC_Limits_T* limits);

    /**
     * Get the current internal state (for debugging)
     * @param state Pointer to state structure to fill
     */
    void getState(SimulinkFOC_State_T* state) const;

    /**
     * Check if the algorithm is initialized
     * @return true if initialized
     */
    bool isInitialized() const;

    /**
     * Run loopFOC equivalent - inner FOC loop only
     * Handles: sensor update, electrical angle, current control, phase voltage
     */
    void runInnerLoop();

    /**
     * Run move equivalent - outer control loop only
     * Handles: position/velocity control, target processing
     */
    void runOuterLoop();

    // Direct access to outputs for convenience (read-only)
    float getUa() const { return outputs_.Ua; }
    float getUb() const { return outputs_.Ub; }
    float getUc() const { return outputs_.Uc; }
    float getElectricalAngle() const { return outputs_.electrical_angle; }
    float getVoltageQ() const { return outputs_.voltage_q; }
    float getVoltageD() const { return outputs_.voltage_d; }
    float getCurrentQ() const { return outputs_.current_q; }
    float getCurrentD() const { return outputs_.current_d; }
    uint8_t getPhaseStateA() const { return outputs_.phase_state_a; }
    uint8_t getPhaseStateB() const { return outputs_.phase_state_b; }
    uint8_t getPhaseStateC() const { return outputs_.phase_state_c; }

private:
    // Internal algorithm components - these represent the code-generated blocks

    /**
     * Calculate electrical angle from mechanical angle
     */
    float calculateElectricalAngle(float mechanical_angle);

    /**
     * Clarke transform: 3-phase to alpha-beta
     */
    void clarkeTransform(float ia, float ib, float ic, float* i_alpha, float* i_beta);

    /**
     * Inverse Clarke transform: alpha-beta to 3-phase
     */
    void inverseClarkeTransform(float u_alpha, float u_beta, float* ua, float* ub, float* uc);

    /**
     * Park transform: alpha-beta to d-q
     */
    void parkTransform(float i_alpha, float i_beta, float angle, float* id, float* iq);

    /**
     * Inverse Park transform: d-q to alpha-beta
     */
    void inverseParkTransform(float ud, float uq, float angle, float* u_alpha, float* u_beta);

    /**
     * PID controller step
     */
    float pidController(float error, float dt, float P, float I, float D,
                       float ramp, float limit,
                       float* integral, float* prev_error, float* prev_output);

    /**
     * Low-pass filter step
     */
    float lowPassFilter(float input, float dt, float Tf, float* prev_output);

    /**
     * Calculate modulated phase voltages
     */
    void calculateModulation(float uq, float ud, float angle_el);

    /**
     * Sine/Space Vector PWM modulation
     */
    void sineSpaceVectorPWM(float uq, float ud, float angle_el);

    /**
     * Trapezoidal 120° modulation
     */
    void trapezoid120Modulation(float uq, float angle_el);

    /**
     * Trapezoidal 150° modulation
     */
    void trapezoid150Modulation(float uq, float angle_el);

    /**
     * Process torque control mode
     */
    void processTorqueControl();

    /**
     * Process DC current control
     */
    void processDCCurrentControl();

    /**
     * Process FOC current control
     */
    void processFOCCurrentControl();

    /**
     * Process velocity control mode
     */
    void processVelocityControl();

    /**
     * Process position/angle control mode
     */
    void processPositionControl();

    /**
     * Process open-loop velocity mode
     */
    void processOpenLoopVelocity();

    /**
     * Process open-loop angle mode
     */
    void processOpenLoopAngle();

    /**
     * Constrain value to limits
     */
    static float constrain(float value, float min_val, float max_val);

    /**
     * Normalize angle to [0, 2*PI]
     */
    static float normalizeAngle(float angle);

    // Algorithm parameters (cached from init)
    SimulinkFOC_Params_T params_;

    // Algorithm inputs (set before each step)
    SimulinkFOC_Inputs_T inputs_;

    // Algorithm outputs (available after each step)
    SimulinkFOC_Outputs_T outputs_;

    // Algorithm internal state (persistent across steps)
    SimulinkFOC_State_T state_;

    // Initialization flag
    bool initialized_;
};

#endif // SIMULINK_FOC_H
