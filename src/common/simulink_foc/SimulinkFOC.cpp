#include "SimulinkFOC.h"

/**
 * SimulinkFOC Implementation
 *
 * This is a TEMPLATE implementation that demonstrates the interface for
 * Simulink code-generated FOC algorithm. Replace the algorithm implementations
 * with actual Simulink-generated code.
 *
 * The structure follows the Simulink model convention with:
 * - Initialization function (init)
 * - Step function (step)
 * - Parameter update functions
 */

// Lookup tables for trapezoidal modulation (same as original)
static const int trap_120_map[6][3] = {
    {0, 1, -1},
    {-1, 1, 0},
    {-1, 0, 1},
    {0, -1, 1},
    {1, -1, 0},
    {1, 0, -1}
};

static const int trap_150_map[12][3] = {
    {0, 1, -1},
    {-1, 1, -1},
    {-1, 1, 0},
    {-1, 1, 1},
    {-1, 0, 1},
    {-1, -1, 1},
    {0, -1, 1},
    {1, -1, 1},
    {1, -1, 0},
    {1, -1, -1},
    {1, 0, -1},
    {1, 1, -1}
};

SimulinkFOC::SimulinkFOC() : initialized_(false) {
    memset(&params_, 0, sizeof(params_));
    memset(&inputs_, 0, sizeof(inputs_));
    memset(&outputs_, 0, sizeof(outputs_));
    memset(&state_, 0, sizeof(state_));
}

SimulinkFOC::~SimulinkFOC() {
    // Cleanup if needed
}

int SimulinkFOC::init(const SimulinkFOC_Params_T* params) {
    if (params == nullptr) {
        return -1;
    }

    // Copy parameters
    memcpy(&params_, params, sizeof(SimulinkFOC_Params_T));

    // Reset state
    reset();

    initialized_ = true;
    state_.initialized = 1;

    return 0;
}

void SimulinkFOC::reset() {
    // Reset all PID states
    state_.pid_current_q_integral = 0.0f;
    state_.pid_current_q_prev_error = 0.0f;
    state_.pid_current_q_prev_output = 0.0f;

    state_.pid_current_d_integral = 0.0f;
    state_.pid_current_d_prev_error = 0.0f;
    state_.pid_current_d_prev_output = 0.0f;

    state_.pid_velocity_integral = 0.0f;
    state_.pid_velocity_prev_error = 0.0f;
    state_.pid_velocity_prev_output = 0.0f;

    state_.p_angle_prev_output = 0.0f;

    // Reset filter states
    state_.lpf_current_q_prev = 0.0f;
    state_.lpf_current_d_prev = 0.0f;
    state_.lpf_velocity_prev = 0.0f;
    state_.lpf_angle_prev = 0.0f;

    // Reset open-loop state
    state_.open_loop_angle = 0.0f;
    state_.open_loop_timestamp = 0;

    // Reset motion counter
    state_.motion_counter = 0;

    // Clear outputs
    memset(&outputs_, 0, sizeof(outputs_));

    // Initialize phase states to ON
    outputs_.phase_state_a = 1;
    outputs_.phase_state_b = 1;
    outputs_.phase_state_c = 1;
}

void SimulinkFOC::setInputs(const SimulinkFOC_Inputs_T* inputs) {
    if (inputs != nullptr) {
        memcpy(&inputs_, inputs, sizeof(SimulinkFOC_Inputs_T));
    }
}

void SimulinkFOC::step() {
    /*
     * SIMULINK CODE-GENERATED ENTRY POINT
     *
     * This function represents the main step function that would be
     * replaced with Simulink-generated code. The current implementation
     * is a template that mirrors the original Arduino-FOC algorithm.
     *
     * In a real Simulink integration, this would call:
     *   SimulinkFOC_Model_step();
     *
     * The Simulink model would have:
     * - Inport blocks for inputs_
     * - Outport blocks for outputs_
     * - Data Store blocks for state_
     * - Parameter blocks for params_
     */

    if (!initialized_ || !inputs_.enabled) {
        // Output zero voltages when disabled
        outputs_.Ua = 0.0f;
        outputs_.Ub = 0.0f;
        outputs_.Uc = 0.0f;
        return;
    }

    // Run the inner FOC loop (high frequency)
    runInnerLoop();

    // Run the outer control loop (can be downsampled)
    runOuterLoop();
}

void SimulinkFOC::runInnerLoop() {
    /*
     * Inner FOC Loop - High Frequency
     *
     * This corresponds to the loopFOC() function in the original code.
     * It handles:
     * 1. Electrical angle calculation
     * 2. Current sensing and transformation
     * 3. Current control (PID)
     * 4. Phase voltage calculation
     */

    // Skip for open-loop modes
    if (params_.config.motion_mode == SFOC_MOTION_VELOCITY_OL ||
        params_.config.motion_mode == SFOC_MOTION_ANGLE_OL) {
        return;
    }

    // Calculate electrical angle
    outputs_.electrical_angle = calculateElectricalAngle(inputs_.mechanical_angle);

    // Process based on torque control type
    switch (params_.config.torque_mode) {
        case SFOC_TORQUE_VOLTAGE:
            processTorqueControl();
            break;

        case SFOC_TORQUE_DC_CURRENT:
            if (inputs_.current_sense_available) {
                processDCCurrentControl();
            }
            break;

        case SFOC_TORQUE_FOC_CURRENT:
            if (inputs_.current_sense_available) {
                processFOCCurrentControl();
            }
            break;

        default:
            break;
    }

    // Calculate modulated phase voltages
    calculateModulation(outputs_.voltage_q, outputs_.voltage_d, outputs_.electrical_angle);
}

void SimulinkFOC::runOuterLoop() {
    /*
     * Outer Control Loop - Can be Downsampled
     *
     * This corresponds to the move() function in the original code.
     * It handles:
     * 1. Velocity/position processing
     * 2. Control mode execution
     * 3. Target processing
     */

    // Downsampling
    if (params_.config.motion_downsample > 0) {
        state_.motion_counter++;
        if (state_.motion_counter < params_.config.motion_downsample) {
            return;
        }
        state_.motion_counter = 0;
    }

    // Process based on motion control mode
    switch (params_.config.motion_mode) {
        case SFOC_MOTION_TORQUE:
            // Target goes directly to current/voltage setpoint
            if (params_.config.torque_mode == SFOC_TORQUE_VOLTAGE) {
                if (params_.motor.phase_resistance <= 0.0f) {
                    outputs_.voltage_q = inputs_.target;
                } else {
                    outputs_.voltage_q = inputs_.target * params_.motor.phase_resistance + outputs_.voltage_bemf;
                }
                outputs_.voltage_q = constrain(outputs_.voltage_q, -params_.limits.voltage_limit, params_.limits.voltage_limit);
            } else {
                outputs_.current_sp = inputs_.target;
            }
            break;

        case SFOC_MOTION_VELOCITY:
            processVelocityControl();
            break;

        case SFOC_MOTION_ANGLE:
            processPositionControl();
            break;

        case SFOC_MOTION_VELOCITY_OL:
            processOpenLoopVelocity();
            break;

        case SFOC_MOTION_ANGLE_OL:
            processOpenLoopAngle();
            break;

        default:
            break;
    }
}

void SimulinkFOC::getOutputs(SimulinkFOC_Outputs_T* outputs) const {
    if (outputs != nullptr) {
        memcpy(outputs, &outputs_, sizeof(SimulinkFOC_Outputs_T));
    }
}

void SimulinkFOC::updateParams(const SimulinkFOC_Params_T* params) {
    if (params != nullptr) {
        memcpy(&params_, params, sizeof(SimulinkFOC_Params_T));
    }
}

void SimulinkFOC::updateGains(const SimulinkFOC_ControlGains_T* gains) {
    if (gains != nullptr) {
        memcpy(&params_.gains, gains, sizeof(SimulinkFOC_ControlGains_T));
    }
}

void SimulinkFOC::updateConfig(const SimulinkFOC_ControlConfig_T* config) {
    if (config != nullptr) {
        memcpy(&params_.config, config, sizeof(SimulinkFOC_ControlConfig_T));
    }
}

void SimulinkFOC::updateSensorCalib(const SimulinkFOC_SensorCalib_T* calib) {
    if (calib != nullptr) {
        memcpy(&params_.sensor, calib, sizeof(SimulinkFOC_SensorCalib_T));
    }
}

void SimulinkFOC::updateLimits(const SimulinkFOC_Limits_T* limits) {
    if (limits != nullptr) {
        memcpy(&params_.limits, limits, sizeof(SimulinkFOC_Limits_T));
    }
}

void SimulinkFOC::getState(SimulinkFOC_State_T* state) const {
    if (state != nullptr) {
        memcpy(state, &state_, sizeof(SimulinkFOC_State_T));
    }
}

bool SimulinkFOC::isInitialized() const {
    return initialized_;
}

// ============================================================================
// Private Implementation - Algorithm Blocks
// ============================================================================

float SimulinkFOC::calculateElectricalAngle(float mechanical_angle) {
    /*
     * Electrical Angle Calculation Block
     *
     * In Simulink, this would be a simple gain and modulo block:
     * electrical_angle = mod(direction * pole_pairs * (mechanical_angle - offset) - zero_angle, 2*pi)
     */
    float angle = params_.sensor.sensor_direction * params_.motor.pole_pairs *
                  (mechanical_angle + params_.sensor.sensor_offset);
    angle = angle - params_.sensor.zero_electric_angle;
    return normalizeAngle(angle);
}

void SimulinkFOC::clarkeTransform(float ia, float ib, float ic, float* i_alpha, float* i_beta) {
    /*
     * Clarke Transform Block (abc to alpha-beta)
     *
     * Standard Clarke transform with common-mode rejection:
     * i_alpha = ia - (ib + ic)/2 = (2*ia - ib - ic)/2
     * i_beta = sqrt(3)/2 * (ib - ic)
     *
     * Simplified for balanced 3-phase:
     * i_alpha = ia
     * i_beta = (1/sqrt(3)) * ia + (2/sqrt(3)) * ib
     */
    float mid = (ia + ib + ic) / 3.0f;
    float a = ia - mid;
    float b = ib - mid;

    *i_alpha = a;
    *i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
}

void SimulinkFOC::inverseClarkeTransform(float u_alpha, float u_beta, float* ua, float* ub, float* uc) {
    /*
     * Inverse Clarke Transform Block (alpha-beta to abc)
     *
     * ua = u_alpha
     * ub = -0.5 * u_alpha + sqrt(3)/2 * u_beta
     * uc = -0.5 * u_alpha - sqrt(3)/2 * u_beta
     */
    *ua = u_alpha;
    *ub = -0.5f * u_alpha + _SQRT3_2 * u_beta;
    *uc = -0.5f * u_alpha - _SQRT3_2 * u_beta;
}

void SimulinkFOC::parkTransform(float i_alpha, float i_beta, float angle, float* id, float* iq) {
    /*
     * Park Transform Block (alpha-beta to d-q)
     *
     * id = i_alpha * cos(angle) + i_beta * sin(angle)
     * iq = i_beta * cos(angle) - i_alpha * sin(angle)
     */
    float sin_a, cos_a;
    _sincos(angle, &sin_a, &cos_a);

    *id = i_alpha * cos_a + i_beta * sin_a;
    *iq = i_beta * cos_a - i_alpha * sin_a;
}

void SimulinkFOC::inverseParkTransform(float ud, float uq, float angle, float* u_alpha, float* u_beta) {
    /*
     * Inverse Park Transform Block (d-q to alpha-beta)
     *
     * u_alpha = ud * cos(angle) - uq * sin(angle)
     * u_beta = ud * sin(angle) + uq * cos(angle)
     */
    float sin_a, cos_a;
    _sincos(angle, &sin_a, &cos_a);

    *u_alpha = ud * cos_a - uq * sin_a;
    *u_beta = ud * sin_a + uq * cos_a;
}

float SimulinkFOC::pidController(float error, float dt, float P, float I, float D,
                                  float ramp, float limit,
                                  float* integral, float* prev_error, float* prev_output) {
    /*
     * PID Controller Block (Tustin/Trapezoidal Discretization)
     *
     * In Simulink, this would be a Discrete PID Controller block
     * with anti-windup and output rate limiting.
     */
    if (dt <= 0.0f) dt = 1e-3f;

    // Proportional term
    float u_p = P * error;

    // Integral term (Tustin integration)
    *integral += I * dt * 0.5f * (error + *prev_error);
    *integral = constrain(*integral, -limit, limit);

    // Derivative term
    float u_d = D * (error - *prev_error) / dt;

    // Sum
    float output = u_p + *integral + u_d;
    output = constrain(output, -limit, limit);

    // Rate limiting
    if (ramp > 0.0f) {
        float d_output = (output - *prev_output) / dt;
        if (d_output > ramp) {
            output = *prev_output + ramp * dt;
        } else if (d_output < -ramp) {
            output = *prev_output - ramp * dt;
        }
    }

    // Store for next iteration
    *prev_error = error;
    *prev_output = output;

    return output;
}

float SimulinkFOC::lowPassFilter(float input, float dt, float Tf, float* prev_output) {
    /*
     * Low-Pass Filter Block (First-Order)
     *
     * In Simulink, this would be a Discrete Transfer Function or
     * Discrete Filter block with cutoff frequency = 1/(2*pi*Tf)
     */
    if (Tf <= 0.0f || dt <= 0.0f) {
        *prev_output = input;
        return input;
    }

    float alpha = Tf / (Tf + dt);
    float output = alpha * (*prev_output) + (1.0f - alpha) * input;
    *prev_output = output;

    return output;
}

void SimulinkFOC::calculateModulation(float uq, float ud, float angle_el) {
    /*
     * Modulation Calculation Block
     *
     * Routes to appropriate modulation algorithm based on configuration.
     */
    switch (params_.config.modulation_type) {
        case SFOC_MOD_SINE_PWM:
        case SFOC_MOD_SPACE_VECTOR:
            sineSpaceVectorPWM(uq, ud, angle_el);
            break;

        case SFOC_MOD_TRAPEZOID_120:
            trapezoid120Modulation(uq, angle_el);
            break;

        case SFOC_MOD_TRAPEZOID_150:
            trapezoid150Modulation(uq, angle_el);
            break;

        default:
            sineSpaceVectorPWM(uq, ud, angle_el);
            break;
    }
}

void SimulinkFOC::sineSpaceVectorPWM(float uq, float ud, float angle_el) {
    /*
     * Sine PWM / Space Vector PWM Modulation Block
     *
     * In Simulink, this would be:
     * 1. Inverse Park Transform subsystem
     * 2. Inverse Clarke Transform subsystem
     * 3. Space Vector centering (optional)
     * 4. Output scaling/centering
     */
    float sin_a, cos_a;
    _sincos(angle_el, &sin_a, &cos_a);

    // Inverse Park transform
    outputs_.Ualpha = cos_a * ud - sin_a * uq;
    outputs_.Ubeta = sin_a * ud + cos_a * uq;

    // Inverse Clarke transform
    float Ua = outputs_.Ualpha;
    float Ub = -0.5f * outputs_.Ualpha + _SQRT3_2 * outputs_.Ubeta;
    float Uc = -0.5f * outputs_.Ualpha - _SQRT3_2 * outputs_.Ubeta;

    float center = params_.limits.voltage_power_supply / 2.0f;

    // Space Vector PWM centering
    if (params_.config.modulation_type == SFOC_MOD_SPACE_VECTOR) {
        float Umin = Ua < Ub ? (Ua < Uc ? Ua : Uc) : (Ub < Uc ? Ub : Uc);
        float Umax = Ua > Ub ? (Ua > Uc ? Ua : Uc) : (Ub > Uc ? Ub : Uc);
        center -= (Umax + Umin) / 2.0f;
    }

    // Apply centering or pull to zero
    if (params_.config.modulation_centered) {
        outputs_.Ua = Ua + center;
        outputs_.Ub = Ub + center;
        outputs_.Uc = Uc + center;
    } else {
        float Umin = Ua < Ub ? (Ua < Uc ? Ua : Uc) : (Ub < Uc ? Ub : Uc);
        outputs_.Ua = Ua - Umin;
        outputs_.Ub = Ub - Umin;
        outputs_.Uc = Uc - Umin;
    }

    // All phases active
    outputs_.phase_state_a = 1;
    outputs_.phase_state_b = 1;
    outputs_.phase_state_c = 1;
}

void SimulinkFOC::trapezoid120Modulation(float uq, float angle_el) {
    /*
     * Trapezoidal 120° Modulation Block
     *
     * In Simulink, this would use a Lookup Table or Multiport Switch
     * to select the appropriate sector and phase values.
     */
    int sector = (int)(6.0f * normalizeAngle(angle_el + _PI_6) / _2PI);
    if (sector >= 6) sector = 5;
    if (sector < 0) sector = 0;

    float center = params_.config.modulation_centered ?
                   params_.limits.voltage_power_supply / 2.0f : uq;

    if (trap_120_map[sector][0] == 0) {
        outputs_.Ua = center;
        outputs_.Ub = trap_120_map[sector][1] * uq + center;
        outputs_.Uc = trap_120_map[sector][2] * uq + center;
        outputs_.phase_state_a = 0;
        outputs_.phase_state_b = 1;
        outputs_.phase_state_c = 1;
    } else if (trap_120_map[sector][1] == 0) {
        outputs_.Ua = trap_120_map[sector][0] * uq + center;
        outputs_.Ub = center;
        outputs_.Uc = trap_120_map[sector][2] * uq + center;
        outputs_.phase_state_a = 1;
        outputs_.phase_state_b = 0;
        outputs_.phase_state_c = 1;
    } else {
        outputs_.Ua = trap_120_map[sector][0] * uq + center;
        outputs_.Ub = trap_120_map[sector][1] * uq + center;
        outputs_.Uc = center;
        outputs_.phase_state_a = 1;
        outputs_.phase_state_b = 1;
        outputs_.phase_state_c = 0;
    }
}

void SimulinkFOC::trapezoid150Modulation(float uq, float angle_el) {
    /*
     * Trapezoidal 150° Modulation Block
     */
    int sector = (int)(12.0f * normalizeAngle(angle_el + _PI_6) / _2PI);
    if (sector >= 12) sector = 11;
    if (sector < 0) sector = 0;

    float center = params_.config.modulation_centered ?
                   params_.limits.voltage_power_supply / 2.0f : uq;

    if (trap_150_map[sector][0] == 0) {
        outputs_.Ua = center;
        outputs_.Ub = trap_150_map[sector][1] * uq + center;
        outputs_.Uc = trap_150_map[sector][2] * uq + center;
        outputs_.phase_state_a = 0;
        outputs_.phase_state_b = 1;
        outputs_.phase_state_c = 1;
    } else if (trap_150_map[sector][1] == 0) {
        outputs_.Ua = trap_150_map[sector][0] * uq + center;
        outputs_.Ub = center;
        outputs_.Uc = trap_150_map[sector][2] * uq + center;
        outputs_.phase_state_a = 1;
        outputs_.phase_state_b = 0;
        outputs_.phase_state_c = 1;
    } else if (trap_150_map[sector][2] == 0) {
        outputs_.Ua = trap_150_map[sector][0] * uq + center;
        outputs_.Ub = trap_150_map[sector][1] * uq + center;
        outputs_.Uc = center;
        outputs_.phase_state_a = 1;
        outputs_.phase_state_b = 1;
        outputs_.phase_state_c = 0;
    } else {
        outputs_.Ua = trap_150_map[sector][0] * uq + center;
        outputs_.Ub = trap_150_map[sector][1] * uq + center;
        outputs_.Uc = trap_150_map[sector][2] * uq + center;
        outputs_.phase_state_a = 1;
        outputs_.phase_state_b = 1;
        outputs_.phase_state_c = 1;
    }
}

void SimulinkFOC::processTorqueControl() {
    /*
     * Voltage-based Torque Control Block
     *
     * Direct voltage control - no current feedback needed.
     */
    // voltage.q is set directly from outer loop
    // voltage.d can include lag compensation if inductance is known
}

void SimulinkFOC::processDCCurrentControl() {
    /*
     * DC Current Control Block
     *
     * Uses single current magnitude for control.
     * In Simulink: Current measurement -> LPF -> PID -> Voltage output
     */
    // Get DC current magnitude from phase currents
    // This is a simplified calculation - real implementation would use
    // proper DC current estimation
    float i_alpha, i_beta;
    clarkeTransform(inputs_.phase_current_a, inputs_.phase_current_b,
                   inputs_.phase_current_c, &i_alpha, &i_beta);

    float id, iq;
    parkTransform(i_alpha, i_beta, outputs_.electrical_angle, &id, &iq);

    // Filter the current
    outputs_.current_q = lowPassFilter(iq, inputs_.dt,
                                       params_.gains.current_q_filter_Tf,
                                       &state_.lpf_current_q_prev);

    // PID control
    float error = outputs_.current_sp - outputs_.current_q;
    outputs_.voltage_q = pidController(error, inputs_.dt,
                                       params_.gains.current_q_P,
                                       params_.gains.current_q_I,
                                       params_.gains.current_q_D,
                                       params_.gains.current_q_ramp,
                                       params_.gains.current_q_limit,
                                       &state_.pid_current_q_integral,
                                       &state_.pid_current_q_prev_error,
                                       &state_.pid_current_q_prev_output);

    // D-axis lag compensation
    if (params_.motor.phase_inductance > 0.0f) {
        outputs_.voltage_d = constrain(
            -outputs_.current_sp * inputs_.shaft_velocity *
            params_.motor.pole_pairs * params_.motor.phase_inductance,
            -params_.limits.voltage_limit, params_.limits.voltage_limit);
    } else {
        outputs_.voltage_d = 0.0f;
    }
}

void SimulinkFOC::processFOCCurrentControl() {
    /*
     * FOC Current Control Block
     *
     * Full DQ current control with independent D and Q axis PIDs.
     * In Simulink: Clarke -> Park -> Dual PID -> Inverse transforms
     */
    // Clarke transform
    float i_alpha, i_beta;
    clarkeTransform(inputs_.phase_current_a, inputs_.phase_current_b,
                   inputs_.phase_current_c, &i_alpha, &i_beta);

    // Park transform
    float id_raw, iq_raw;
    parkTransform(i_alpha, i_beta, outputs_.electrical_angle, &id_raw, &iq_raw);

    // Filter currents
    outputs_.current_q = lowPassFilter(iq_raw, inputs_.dt,
                                       params_.gains.current_q_filter_Tf,
                                       &state_.lpf_current_q_prev);
    outputs_.current_d = lowPassFilter(id_raw, inputs_.dt,
                                       params_.gains.current_d_filter_Tf,
                                       &state_.lpf_current_d_prev);

    // Q-axis current PID
    float error_q = outputs_.current_sp - outputs_.current_q;
    outputs_.voltage_q = pidController(error_q, inputs_.dt,
                                       params_.gains.current_q_P,
                                       params_.gains.current_q_I,
                                       params_.gains.current_q_D,
                                       params_.gains.current_q_ramp,
                                       params_.gains.current_q_limit,
                                       &state_.pid_current_q_integral,
                                       &state_.pid_current_q_prev_error,
                                       &state_.pid_current_q_prev_output);

    // D-axis current PID (target is 0 for PMSM)
    float error_d = 0.0f - outputs_.current_d;
    outputs_.voltage_d = pidController(error_d, inputs_.dt,
                                       params_.gains.current_d_P,
                                       params_.gains.current_d_I,
                                       params_.gains.current_d_D,
                                       params_.gains.current_d_ramp,
                                       params_.gains.current_d_limit,
                                       &state_.pid_current_d_integral,
                                       &state_.pid_current_d_prev_error,
                                       &state_.pid_current_d_prev_output);
}

void SimulinkFOC::processVelocityControl() {
    /*
     * Velocity Control Block
     *
     * Velocity PID that outputs current/voltage setpoint.
     */
    outputs_.shaft_velocity_sp = inputs_.target;

    // Velocity error
    float error = outputs_.shaft_velocity_sp - inputs_.shaft_velocity;

    // Back-EMF estimation
    if (params_.motor.KV_rating > 0.0f) {
        outputs_.voltage_bemf = inputs_.shaft_velocity /
                               (params_.motor.KV_rating * _SQRT3) / _RPM_TO_RADS;
    }

    // Velocity PID
    outputs_.current_sp = pidController(error, inputs_.dt,
                                        params_.gains.velocity_P,
                                        params_.gains.velocity_I,
                                        params_.gains.velocity_D,
                                        params_.gains.velocity_ramp,
                                        params_.gains.velocity_limit,
                                        &state_.pid_velocity_integral,
                                        &state_.pid_velocity_prev_error,
                                        &state_.pid_velocity_prev_output);

    // Convert to voltage if using voltage torque control
    if (params_.config.torque_mode == SFOC_TORQUE_VOLTAGE) {
        if (params_.motor.phase_resistance <= 0.0f) {
            outputs_.voltage_q = outputs_.current_sp;
        } else {
            outputs_.voltage_q = constrain(
                outputs_.current_sp * params_.motor.phase_resistance + outputs_.voltage_bemf,
                -params_.limits.voltage_limit, params_.limits.voltage_limit);
        }

        // D-axis lag compensation
        if (params_.motor.phase_inductance > 0.0f) {
            outputs_.voltage_d = constrain(
                -outputs_.current_sp * inputs_.shaft_velocity *
                params_.motor.pole_pairs * params_.motor.phase_inductance,
                -params_.limits.voltage_limit, params_.limits.voltage_limit);
        } else {
            outputs_.voltage_d = 0.0f;
        }
    }
}

void SimulinkFOC::processPositionControl() {
    /*
     * Position Control Block
     *
     * Position P controller that outputs velocity setpoint,
     * then velocity PID that outputs current/voltage.
     */
    outputs_.shaft_angle_sp = inputs_.target;

    // Position error to velocity setpoint (P controller)
    float angle_error = outputs_.shaft_angle_sp - inputs_.mechanical_angle;
    outputs_.shaft_velocity_sp = inputs_.feed_forward_velocity +
                                 params_.gains.angle_P * angle_error;
    outputs_.shaft_velocity_sp = constrain(outputs_.shaft_velocity_sp,
                                           -params_.limits.velocity_limit,
                                           params_.limits.velocity_limit);

    // Velocity error
    float vel_error = outputs_.shaft_velocity_sp - inputs_.shaft_velocity;

    // Back-EMF estimation
    if (params_.motor.KV_rating > 0.0f) {
        outputs_.voltage_bemf = inputs_.shaft_velocity /
                               (params_.motor.KV_rating * _SQRT3) / _RPM_TO_RADS;
    }

    // Velocity PID
    outputs_.current_sp = pidController(vel_error, inputs_.dt,
                                        params_.gains.velocity_P,
                                        params_.gains.velocity_I,
                                        params_.gains.velocity_D,
                                        params_.gains.velocity_ramp,
                                        params_.gains.velocity_limit,
                                        &state_.pid_velocity_integral,
                                        &state_.pid_velocity_prev_error,
                                        &state_.pid_velocity_prev_output);

    // Convert to voltage if using voltage torque control
    if (params_.config.torque_mode == SFOC_TORQUE_VOLTAGE) {
        if (params_.motor.phase_resistance <= 0.0f) {
            outputs_.voltage_q = outputs_.current_sp;
        } else {
            outputs_.voltage_q = constrain(
                outputs_.current_sp * params_.motor.phase_resistance + outputs_.voltage_bemf,
                -params_.limits.voltage_limit, params_.limits.voltage_limit);
        }

        // D-axis lag compensation
        if (params_.motor.phase_inductance > 0.0f) {
            outputs_.voltage_d = constrain(
                -outputs_.current_sp * inputs_.shaft_velocity *
                params_.motor.pole_pairs * params_.motor.phase_inductance,
                -params_.limits.voltage_limit, params_.limits.voltage_limit);
        } else {
            outputs_.voltage_d = 0.0f;
        }
    }
}

void SimulinkFOC::processOpenLoopVelocity() {
    /*
     * Open-Loop Velocity Control Block
     *
     * Direct angle ramping without feedback.
     */
    outputs_.shaft_velocity_sp = inputs_.target;

    // Integrate velocity to get angle
    state_.open_loop_angle = normalizeAngle(state_.open_loop_angle +
                                            inputs_.target * inputs_.dt);
    outputs_.shaft_angle = state_.open_loop_angle;

    // Calculate electrical angle
    outputs_.electrical_angle = normalizeAngle(
        params_.motor.pole_pairs * state_.open_loop_angle);

    // Use voltage limit or current limit
    float Uq = params_.limits.voltage_limit;
    if (params_.motor.phase_resistance > 0.0f) {
        // Back-EMF estimation
        if (params_.motor.KV_rating > 0.0f) {
            outputs_.voltage_bemf = fabsf(inputs_.target) /
                                   (params_.motor.KV_rating * _SQRT3) / _RPM_TO_RADS;
        }
        Uq = constrain(params_.limits.current_limit * params_.motor.phase_resistance +
                      fabsf(outputs_.voltage_bemf),
                      -params_.limits.voltage_limit, params_.limits.voltage_limit);
        outputs_.current_q = (Uq - fabsf(outputs_.voltage_bemf)) /
                            params_.motor.phase_resistance;
    }

    outputs_.voltage_q = Uq;
    outputs_.voltage_d = 0.0f;

    // Calculate modulated voltages
    calculateModulation(outputs_.voltage_q, outputs_.voltage_d, outputs_.electrical_angle);
}

void SimulinkFOC::processOpenLoopAngle() {
    /*
     * Open-Loop Angle Control Block
     *
     * Direct angle targeting with velocity limiting.
     */
    outputs_.shaft_angle_sp = inputs_.target;

    // Move towards target angle at limited velocity
    float angle_diff = outputs_.shaft_angle_sp - state_.open_loop_angle;
    float max_step = params_.limits.velocity_limit * inputs_.dt;

    if (fabsf(angle_diff) > max_step) {
        state_.open_loop_angle += (angle_diff > 0 ? 1.0f : -1.0f) * max_step;
        outputs_.shaft_velocity_sp = params_.limits.velocity_limit;
    } else {
        state_.open_loop_angle = outputs_.shaft_angle_sp;
        outputs_.shaft_velocity_sp = 0.0f;
    }
    outputs_.shaft_angle = state_.open_loop_angle;

    // Calculate electrical angle (normalize the angle for precision)
    outputs_.electrical_angle = normalizeAngle(
        params_.motor.pole_pairs * normalizeAngle(state_.open_loop_angle));

    // Use voltage limit or current limit
    float Uq = params_.limits.voltage_limit;
    if (params_.motor.phase_resistance > 0.0f) {
        // Back-EMF estimation
        if (params_.motor.KV_rating > 0.0f) {
            outputs_.voltage_bemf = fabsf(outputs_.shaft_velocity_sp) /
                                   (params_.motor.KV_rating * _SQRT3) / _RPM_TO_RADS;
        }
        Uq = constrain(params_.limits.current_limit * params_.motor.phase_resistance +
                      fabsf(outputs_.voltage_bemf),
                      -params_.limits.voltage_limit, params_.limits.voltage_limit);
        outputs_.current_q = (Uq - fabsf(outputs_.voltage_bemf)) /
                            params_.motor.phase_resistance;
    }

    outputs_.voltage_q = Uq;
    outputs_.voltage_d = 0.0f;

    // Calculate modulated voltages
    calculateModulation(outputs_.voltage_q, outputs_.voltage_d, outputs_.electrical_angle);
}

float SimulinkFOC::constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

float SimulinkFOC::normalizeAngle(float angle) {
    float a = fmodf(angle, _2PI);
    return a >= 0.0f ? a : (a + _2PI);
}
