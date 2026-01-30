/**
 * Online Motor Parameter Estimation Example
 *
 * This example demonstrates the use of the MotorParameterEstimator class
 * to estimate motor parameters (Ld, Lq, flux_linkage) online using
 * Recursive Least Squares (RLS) algorithm.
 *
 * The estimator uses the PMSM voltage equations:
 *   Vd = Rs * id - omega_e * Lq * iq
 *   Vq = Rs * iq + omega_e * Ld * id + omega_e * flux_linkage
 *
 * Requirements:
 * - A BLDC motor with known phase resistance
 * - Current sensing (inline or low-side)
 * - Position sensor (encoder or magnetic sensor)
 * - FOC current control mode
 *
 * The motor should be running at sufficient speed for the estimator to work.
 * Minimum recommended speed: ~100 RPM mechanical (depends on pole pairs)
 */
#include <SimpleFOC.h>

// BLDC motor & driver instance
// Adjust pole pairs for your motor
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// Encoder instance
// Adjust pins and PPR for your encoder
Encoder encoder = Encoder(2, 3, 500);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// Current sensor
// Adjust shunt resistance and gain for your current sense
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, A0, A2);

// Motor Parameter Estimator
// Initialize with known phase resistance (measure it beforehand!)
float phase_resistance = 0.5f; // Ohms - measure this value for your motor
MotorParameterEstimator param_estimator = MotorParameterEstimator(phase_resistance);

// Commander for serial control
Commander command = Commander(Serial);
float target_velocity = 0;
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doPrintParams(char* cmd);

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Initialize encoder
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);

  // Driver configuration
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Current sense configuration
  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // Configure the parameter estimator
  // Set initial guesses for parameters (optional, but helps convergence)
  param_estimator.init(
    0.001f,  // Initial Ld guess [H] - typical range: 0.1mH to 10mH
    0.001f,  // Initial Lq guess [H]
    0.01f    // Initial flux linkage guess [Wb] - typical range: 5mWb to 50mWb
  );

  // Estimator configuration (optional - defaults are usually fine)
  param_estimator.forgetting_factor = 0.995f;  // Higher = slower adaptation, more stable
  param_estimator.min_velocity = 5.0f;          // Minimum electrical velocity for estimation [rad/s]
  param_estimator.min_current = 0.1f;           // Minimum current for estimation [A]

  // Link the estimator to the motor
  motor.linkParameterEstimator(&param_estimator);

  // Set known motor parameters
  motor.phase_resistance = phase_resistance;

  // Configure FOC current control (required for parameter estimation)
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity;

  // Current control PID - tune for your motor
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 300;
  motor.PID_current_d.P = 5;
  motor.PID_current_d.I = 300;
  motor.LPF_current_q.Tf = 0.01f;
  motor.LPF_current_d.Tf = 0.01f;

  // Velocity PID
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2.0f;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.01f;

  // Limits
  motor.voltage_limit = 6;
  motor.current_limit = 1;
  motor.velocity_limit = 50;

  motor.useMonitoring(Serial);

  // Initialize motor
  motor.init();
  motor.initFOC();

  // Add serial commands
  command.add('T', doTarget, "target velocity [rad/s]");
  command.add('P', doPrintParams, "print estimated parameters");

  Serial.println(F("Motor Parameter Estimation Example"));
  Serial.println(F("==================================="));
  Serial.println(F("Commands:"));
  Serial.println(F("  T<value> - Set target velocity (rad/s)"));
  Serial.println(F("  P        - Print estimated parameters"));
  Serial.println(F(""));
  Serial.println(F("Instructions:"));
  Serial.println(F("1. Set a target velocity (e.g., T10 for 10 rad/s)"));
  Serial.println(F("2. Wait for the motor to reach steady state"));
  Serial.println(F("3. The estimator will update Ld, Lq, and flux linkage"));
  Serial.println(F("4. Press 'P' to see current estimates"));
  Serial.println(F(""));
  _delay(1000);
}

// Variables for periodic parameter printing
unsigned long last_print_time = 0;
const unsigned long print_interval = 2000; // Print every 2 seconds

void loop() {
  // Main FOC loop
  motor.loopFOC();

  // Motion control
  motor.move(target_velocity);

  // Process serial commands
  command.run();

  // Periodically print estimated parameters
  unsigned long now = millis();
  if (now - last_print_time > print_interval) {
    last_print_time = now;

    // Only print if motor is running and estimator is working
    if (abs(motor.shaft_velocity) > 1.0f) {
      Serial.print(F("Est: Ld="));
      Serial.print(param_estimator.getLd() * 1000.0f, 4);
      Serial.print(F("mH, Lq="));
      Serial.print(param_estimator.getLq() * 1000.0f, 4);
      Serial.print(F("mH, Flux="));
      Serial.print(param_estimator.getFluxLinkage() * 1000.0f, 4);
      Serial.print(F("mWb"));

      if (param_estimator.isConverged()) {
        Serial.print(F(" [CONVERGED]"));
      }
      Serial.println();
    }
  }
}

// Command callback to print parameters
void doPrintParams(char* cmd) {
  MotorParameters_s params = param_estimator.getParameters();

  Serial.println(F("\n=== Estimated Motor Parameters ==="));
  Serial.print(F("Phase Resistance (Rs): "));
  Serial.print(params.Rs, 4);
  Serial.println(F(" Ohm"));

  Serial.print(F("D-axis Inductance (Ld): "));
  Serial.print(params.Ld * 1000.0f, 4);
  Serial.println(F(" mH"));

  Serial.print(F("Q-axis Inductance (Lq): "));
  Serial.print(params.Lq * 1000.0f, 4);
  Serial.println(F(" mH"));

  Serial.print(F("Flux Linkage (lambda_m): "));
  Serial.print(params.flux_linkage * 1000.0f, 4);
  Serial.println(F(" mWb"));

  Serial.print(F("Saliency Ratio (Lq/Ld): "));
  if (params.Ld > 0.0f) {
    Serial.println(params.Lq / params.Ld, 3);
  } else {
    Serial.println(F("N/A"));
  }

  Serial.print(F("Average Inductance: "));
  Serial.print((params.Ld + params.Lq) * 500.0f, 4);
  Serial.println(F(" mH"));

  // Calculate estimated KV rating from flux linkage
  // KV = 1 / (flux_linkage * sqrt(3) * pole_pairs * (pi/30))
  if (params.flux_linkage > 0.0f) {
    float kv_estimated = 30.0f / (_PI * params.flux_linkage * _SQRT3 * motor.pole_pairs);
    Serial.print(F("Estimated KV: "));
    Serial.print(kv_estimated, 1);
    Serial.println(F(" RPM/V"));
  }

  Serial.print(F("\nConverged: "));
  Serial.println(param_estimator.isConverged() ? F("Yes") : F("No"));

  Serial.print(F("Estimation Error (RMS): "));
  Serial.println(param_estimator.getEstimationError(), 4);
  Serial.println(F("===================================\n"));
}
