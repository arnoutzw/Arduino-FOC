# Arduino-FOC API Reference

Comprehensive documentation for all functions, classes, and interfaces in the SimpleFOC library.

---

## Table of Contents

1. [Motor Classes](#motor-classes)
   - [BLDCMotor](#bldcmotor)
   - [StepperMotor](#steppermotor)
   - [HybridStepperMotor](#hybridsteppermotor)
2. [Base Classes](#base-classes)
   - [FOCMotor](#focmotor)
   - [Sensor](#sensor)
   - [CurrentSense](#currentsense)
   - [FOCDriver](#focdriver)
   - [BLDCDriver](#bldcdriver)
   - [StepperDriver](#stepperdriver)
3. [Sensor Implementations](#sensor-implementations)
   - [Encoder](#encoder)
   - [HallSensor](#hallsensor)
   - [MagneticSensorSPI](#magneticsensorspi)
   - [MagneticSensorI2C](#magneticsensori2c)
   - [MagneticSensorAnalog](#magneticsensoranalog)
   - [MagneticSensorPWM](#magneticsensorpwm)
   - [GenericSensor](#genericsensor)
4. [Driver Implementations](#driver-implementations)
   - [BLDCDriver3PWM](#bldcdriver3pwm)
   - [BLDCDriver6PWM](#bldcdriver6pwm)
   - [StepperDriver2PWM](#stepperdriver2pwm)
   - [StepperDriver4PWM](#stepperdriver4pwm)
5. [Current Sense Implementations](#current-sense-implementations)
   - [InlineCurrentSense](#inlinecurrentsense)
   - [LowsideCurrentSense](#lowsidecurrentsense)
   - [GenericCurrentSense](#genericcurrentsense)
6. [Communication Classes](#communication-classes)
   - [Commander](#commander)
   - [StepDirListener](#stepdirlistener)
   - [SimpleFOCDebug](#simplefocdebug)
7. [Control Utilities](#control-utilities)
   - [PIDController](#pidcontroller)
   - [LowPassFilter](#lowpassfilter)
8. [Utility Functions](#utility-functions)
   - [FOC Utils](#foc-utils)
   - [Time Utils](#time-utils)
9. [Data Structures](#data-structures)
10. [Enumerations](#enumerations)
11. [Simulink FOC Integration](#simulink-foc-integration)
    - [SimulinkFOC](#simulinkfoc)
    - [SimulinkFOCAdapter](#simulinkfocadapter)

---

## Motor Classes

### BLDCMotor

The main class for controlling Brushless DC (BLDC) motors using Field Oriented Control.

**Header:** `src/BLDCMotor.h`

#### Constructor

```cpp
BLDCMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET)
```

Creates a new BLDC motor instance.

| Parameter | Type | Description |
|-----------|------|-------------|
| `pp` | `int` | Number of pole pairs of the motor |
| `R` | `float` | Phase resistance in Ohms (optional, for current control) |
| `KV` | `float` | Motor KV rating in RPM/V (optional) |
| `L` | `float` | Phase inductance in Henries (optional, for current control) |

**Example:**
```cpp
BLDCMotor motor = BLDCMotor(7);  // 7 pole pairs
BLDCMotor motor = BLDCMotor(7, 0.5, 120, 0.001);  // With R, KV, L
```

#### Methods

##### linkDriver()

```cpp
void linkDriver(BLDCDriver* driver)
```

Links the motor to a BLDC driver instance.

| Parameter | Type | Description |
|-----------|------|-------------|
| `driver` | `BLDCDriver*` | Pointer to the driver instance |

**Example:**
```cpp
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
motor.linkDriver(&driver);
```

##### init()

```cpp
int init()
```

Initializes the motor hardware. Must be called after linking the driver.

| Returns | Description |
|---------|-------------|
| `int` | 1 on success, 0 on failure |

**Example:**
```cpp
motor.init();
```

##### initFOC()

```cpp
int initFOC(float zero_electric_offset = NOT_SET, Direction sensor_direction = Direction::CW)
```

Initializes the Field Oriented Control algorithm. Aligns the sensor with motor phases.

| Parameter | Type | Description |
|-----------|------|-------------|
| `zero_electric_offset` | `float` | Known zero electric angle (optional, skips calibration) |
| `sensor_direction` | `Direction` | Known sensor direction CW/CCW (optional) |

| Returns | Description |
|---------|-------------|
| `int` | 1 on success, 0 on failure |

**Example:**
```cpp
motor.initFOC();  // Auto-calibration
motor.initFOC(2.5, Direction::CW);  // Skip calibration with known values
```

##### loopFOC()

```cpp
void loopFOC()
```

Main FOC algorithm execution. Must be called as frequently as possible in the main loop. Reads sensor, calculates electrical angle, and sets phase voltages.

**Example:**
```cpp
void loop() {
    motor.loopFOC();
    motor.move(target);
}
```

##### move()

```cpp
void move(float target = NOT_SET)
```

Executes the motion control loop. Sets the target value based on the control mode.

| Parameter | Type | Description |
|-----------|------|-------------|
| `target` | `float` | Target value (angle in rad, velocity in rad/s, or voltage/current) |

**Example:**
```cpp
motor.move(3.14);  // Move to PI radians (angle mode)
motor.move(10);    // 10 rad/s (velocity mode)
```

##### disable()

```cpp
void disable()
```

Disables the motor (sets all PWM outputs to 0).

##### enable()

```cpp
void enable()
```

Enables the motor.

##### setPhaseVoltage()

```cpp
void setPhaseVoltage(float Uq, float Ud, float angle_el)
```

Sets the phase voltages using inverse Park and Clarke transforms.

| Parameter | Type | Description |
|-----------|------|-------------|
| `Uq` | `float` | Q-axis voltage (torque producing) |
| `Ud` | `float` | D-axis voltage (field producing, typically 0) |
| `angle_el` | `float` | Electrical angle in radians |

##### characteriseMotor()

```cpp
int characteriseMotor(float voltage, float correction_factor = 1.0)
```

Estimates motor phase resistance and inductance.

| Parameter | Type | Description |
|-----------|------|-------------|
| `voltage` | `float` | Test voltage to apply |
| `correction_factor` | `float` | Correction factor for inductance (default 1.0) |

| Returns | Description |
|---------|-------------|
| `int` | 1 on success, 0 on failure |

##### enableSimulinkFOC()

```cpp
int enableSimulinkFOC()
```

Enables the Simulink code-generated FOC algorithm.

| Returns | Description |
|---------|-------------|
| `int` | 0 on success, non-zero on failure |

##### disableSimulinkFOC()

```cpp
void disableSimulinkFOC()
```

Disables Simulink FOC and returns to native SimpleFOC algorithm.

##### isSimulinkFOCEnabled()

```cpp
bool isSimulinkFOCEnabled()
```

| Returns | Description |
|---------|-------------|
| `bool` | `true` if Simulink FOC is active |

##### syncSimulinkFOCParams()

```cpp
void syncSimulinkFOCParams()
```

Synchronizes motor parameters to the Simulink FOC component. Call after modifying PID gains or limits.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `driver` | `BLDCDriver*` | Linked driver instance |
| `pole_pairs` | `int` | Number of motor pole pairs |
| `phase_resistance` | `float` | Phase resistance in Ohms |
| `KV_rating` | `float` | Motor KV constant |
| `phase_inductance` | `float` | Phase inductance in Henries |
| `voltage_limit` | `float` | Maximum voltage to apply |
| `current_limit` | `float` | Maximum current limit |
| `velocity_limit` | `float` | Maximum velocity in rad/s |
| `voltage_sensor_align` | `float` | Voltage used during sensor alignment |
| `target` | `float` | Current target value |
| `shaft_angle` | `float` | Current shaft angle in radians |
| `shaft_velocity` | `float` | Current shaft velocity in rad/s |
| `electrical_angle` | `float` | Current electrical angle |
| `Ua, Ub, Uc` | `float` | Phase A, B, C voltages |
| `Ualpha, Ubeta` | `float` | Alpha-beta frame voltages |
| `voltage` | `DQVoltage_s` | D-Q frame voltages |
| `current` | `DQCurrent_s` | D-Q frame currents |
| `current_sp` | `float` | Current setpoint |
| `shaft_velocity_sp` | `float` | Velocity setpoint |
| `shaft_angle_sp` | `float` | Angle setpoint |
| `controller` | `MotionControlType` | Motion control mode |
| `torque_controller` | `TorqueControlType` | Torque control mode |
| `foc_modulation` | `FOCModulationType` | PWM modulation type |
| `modulation_centered` | `int` | Centered modulation flag |
| `sensor_direction` | `Direction` | Sensor direction |
| `zero_electric_angle` | `float` | Calibrated zero angle |
| `sensor_offset` | `float` | Sensor offset |
| `PID_current_q` | `PIDController` | Q-axis current PID |
| `PID_current_d` | `PIDController` | D-axis current PID |
| `PID_velocity` | `PIDController` | Velocity PID controller |
| `P_angle` | `PIDController` | Position P controller |
| `LPF_current_q` | `LowPassFilter` | Q-current low-pass filter |
| `LPF_current_d` | `LowPassFilter` | D-current low-pass filter |
| `LPF_velocity` | `LowPassFilter` | Velocity low-pass filter |
| `LPF_angle` | `LowPassFilter` | Angle low-pass filter |
| `motor_status` | `FOCMotorStatus` | Current motor status |
| `enabled` | `int` | Motor enabled flag |

---

### StepperMotor

Class for controlling stepper motors using Field Oriented Control.

**Header:** `src/StepperMotor.h`

#### Constructor

```cpp
StepperMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET)
```

Creates a new stepper motor instance.

| Parameter | Type | Description |
|-----------|------|-------------|
| `pp` | `int` | Number of pole pairs (typically 50 for 200-step motor) |
| `R` | `float` | Phase resistance in Ohms (optional) |
| `KV` | `float` | Motor KV rating (optional) |
| `L` | `float` | Phase inductance in Henries (optional) |

**Example:**
```cpp
StepperMotor motor = StepperMotor(50);  // 50 pole pairs (200-step motor)
```

#### Methods

##### linkDriver()

```cpp
void linkDriver(StepperDriver* driver)
```

Links the motor to a stepper driver instance.

| Parameter | Type | Description |
|-----------|------|-------------|
| `driver` | `StepperDriver*` | Pointer to the stepper driver |

All other methods are identical to [BLDCMotor](#bldcmotor):
- `init()`, `initFOC()`, `loopFOC()`, `move()`, `disable()`, `enable()`
- `setPhaseVoltage()`, `characteriseMotor()`
- Simulink FOC methods

#### Properties

Same as [BLDCMotor](#bldcmotor) properties.

---

### HybridStepperMotor

Class for controlling hybrid stepper motors using BLDC drivers.

**Header:** `src/HybridStepperMotor.h`

Combines stepper motor control with BLDC driver hardware, allowing hybrid stepper motors to be controlled with 3-phase drivers.

#### Constructor

```cpp
HybridStepperMotor(int pp, float R = NOT_SET, float KV = NOT_SET, float L = NOT_SET)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pp` | `int` | Number of pole pairs |
| `R` | `float` | Phase resistance (optional) |
| `KV` | `float` | Motor KV rating (optional) |
| `L` | `float` | Phase inductance (optional) |

#### Methods

##### linkDriver()

```cpp
void linkDriver(BLDCDriver* driver)
```

Links to a BLDC driver (uses phases A and B).

All other methods identical to [BLDCMotor](#bldcmotor).

---

## Base Classes

### FOCMotor

Abstract base class for all FOC-controlled motors.

**Header:** `src/common/base_classes/FOCMotor.h`

#### Abstract Methods (must be implemented by subclasses)

```cpp
virtual int init() = 0;
virtual void disable() = 0;
virtual void enable() = 0;
virtual int initFOC() = 0;
virtual void loopFOC() = 0;
virtual void move(float target = NOT_SET) = 0;
virtual void setPhaseVoltage(float Uq, float Ud, float angle_el) = 0;
```

#### Implemented Methods

##### linkSensor()

```cpp
void linkSensor(Sensor* sensor)
```

Links a position sensor to the motor.

| Parameter | Type | Description |
|-----------|------|-------------|
| `sensor` | `Sensor*` | Pointer to sensor instance |

##### linkCurrentSense()

```cpp
void linkCurrentSense(CurrentSense* current_sense)
```

Links a current sensing instance to the motor.

| Parameter | Type | Description |
|-----------|------|-------------|
| `current_sense` | `CurrentSense*` | Pointer to current sense instance |

##### shaftAngle()

```cpp
float shaftAngle()
```

Returns the current shaft angle in radians, accounting for sensor direction and offset.

| Returns | Description |
|---------|-------------|
| `float` | Shaft angle in radians |

##### shaftVelocity()

```cpp
float shaftVelocity()
```

Returns the current shaft velocity in rad/s.

| Returns | Description |
|---------|-------------|
| `float` | Shaft velocity in rad/s |

##### electricalAngle()

```cpp
float electricalAngle()
```

Calculates the current electrical angle.

| Returns | Description |
|---------|-------------|
| `float` | Electrical angle in radians |

##### useMonitoring()

```cpp
void useMonitoring(Print &serial)
```

Enables real-time monitoring output.

| Parameter | Type | Description |
|-----------|------|-------------|
| `serial` | `Print&` | Serial port for output |

##### monitor()

```cpp
void monitor()
```

Outputs monitoring data to the configured serial port.

---

### Sensor

Abstract base class for all position sensors.

**Header:** `src/common/base_classes/Sensor.h`

#### Abstract Methods

##### getSensorAngle()

```cpp
virtual float getSensorAngle() = 0
```

Pure virtual method to read the raw sensor angle.

| Returns | Description |
|---------|-------------|
| `float` | Sensor angle in radians [0, 2π) |

#### Virtual Methods (with default implementations)

##### init()

```cpp
virtual void init()
```

Initializes the sensor hardware. Default implementation does nothing.

##### update()

```cpp
virtual void update()
```

Updates the sensor reading. Call this for interrupt-safe sensor reads.

##### getMechanicalAngle()

```cpp
virtual float getMechanicalAngle()
```

Returns the mechanical angle (single rotation, 0 to 2π).

| Returns | Description |
|---------|-------------|
| `float` | Mechanical angle in radians |

##### getAngle()

```cpp
virtual float getAngle()
```

Returns the full angle including multiple rotations.

| Returns | Description |
|---------|-------------|
| `float` | Full angle in radians (can exceed 2π) |

##### getPreciseAngle()

```cpp
virtual double getPreciseAngle()
```

Returns high-precision angle value.

| Returns | Description |
|---------|-------------|
| `double` | Precise angle in radians |

##### getVelocity()

```cpp
virtual float getVelocity()
```

Returns the angular velocity.

| Returns | Description |
|---------|-------------|
| `float` | Angular velocity in rad/s |

##### getFullRotations()

```cpp
virtual int32_t getFullRotations()
```

Returns the number of complete rotations.

| Returns | Description |
|---------|-------------|
| `int32_t` | Number of full rotations |

##### needsSearch()

```cpp
virtual int needsSearch()
```

Indicates if the sensor needs an index search.

| Returns | Description |
|---------|-------------|
| `int` | 1 if search needed, 0 otherwise |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `min_elapsed_time` | `float` | Minimum time between velocity calculations (default 0.0001s) |

---

### CurrentSense

Abstract base class for current sensing implementations.

**Header:** `src/common/base_classes/CurrentSense.h`

#### Abstract Methods

##### init()

```cpp
virtual int init() = 0
```

Initializes the current sensing hardware.

| Returns | Description |
|---------|-------------|
| `int` | 1 on success, 0 on failure |

##### getPhaseCurrents()

```cpp
virtual PhaseCurrent_s getPhaseCurrents() = 0
```

Reads and returns the phase currents.

| Returns | Description |
|---------|-------------|
| `PhaseCurrent_s` | Structure containing a, b, c phase currents |

#### Virtual Methods

##### driverAlign()

```cpp
virtual int driverAlign(float align_voltage, bool modulation_centered = false)
```

Aligns the current sense with the motor driver phases.

| Parameter | Type | Description |
|-----------|------|-------------|
| `align_voltage` | `float` | Voltage to apply during alignment |
| `modulation_centered` | `bool` | Use centered modulation |

| Returns | Description |
|---------|-------------|
| `int` | 0=failure, 1=success, 2=pins reconfigured, 3=gains inverted, 4=both |

##### getDCCurrent()

```cpp
virtual float getDCCurrent(float angle_el = 0)
```

Returns the DC current magnitude.

| Parameter | Type | Description |
|-----------|------|-------------|
| `angle_el` | `float` | Electrical angle (optional) |

| Returns | Description |
|---------|-------------|
| `float` | DC current in Amps |

##### enable() / disable()

```cpp
virtual void enable()
virtual void disable()
```

Enable or disable current sensing.

#### Implemented Methods

##### linkDriver()

```cpp
void linkDriver(FOCDriver *driver)
```

Links current sensing to a driver for synchronization.

##### getFOCCurrents()

```cpp
DQCurrent_s getFOCCurrents(float angle_el)
```

Returns D-Q frame currents using Clarke-Park transform.

| Parameter | Type | Description |
|-----------|------|-------------|
| `angle_el` | `float` | Electrical angle in radians |

| Returns | Description |
|---------|-------------|
| `DQCurrent_s` | Structure with d and q currents |

##### getABCurrents()

```cpp
ABCurrent_s getABCurrents(PhaseCurrent_s current)
```

Performs Clarke transform (abc → αβ).

| Parameter | Type | Description |
|-----------|------|-------------|
| `current` | `PhaseCurrent_s` | Phase currents |

| Returns | Description |
|---------|-------------|
| `ABCurrent_s` | Alpha-beta currents |

##### getDQCurrents()

```cpp
DQCurrent_s getDQCurrents(ABCurrent_s current, float angle_el)
```

Performs Park transform (αβ → dq).

| Parameter | Type | Description |
|-----------|------|-------------|
| `current` | `ABCurrent_s` | Alpha-beta currents |
| `angle_el` | `float` | Electrical angle |

| Returns | Description |
|---------|-------------|
| `DQCurrent_s` | D-Q currents |

##### alignBLDCDriver()

```cpp
int alignBLDCDriver(float align_voltage, BLDCDriver* driver, bool modulation_centered)
```

Aligns current sense with BLDC driver.

##### alignStepperDriver()

```cpp
int alignStepperDriver(float align_voltage, StepperDriver* driver, bool modulation_centered)
```

Aligns current sense with stepper driver.

##### alignHybridDriver()

```cpp
int alignHybridDriver(float align_voltage, BLDCDriver* driver, bool modulation_centered)
```

Aligns current sense with hybrid stepper driver.

##### readAverageCurrents()

```cpp
PhaseCurrent_s readAverageCurrents(int N = 100)
```

Reads and averages N current samples.

| Parameter | Type | Description |
|-----------|------|-------------|
| `N` | `int` | Number of samples to average |

| Returns | Description |
|---------|-------------|
| `PhaseCurrent_s` | Averaged phase currents |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `skip_align` | `bool` | Skip alignment during initFOC |
| `initialized` | `bool` | Initialization status |
| `driver` | `FOCDriver*` | Linked driver |
| `driver_type` | `DriverType` | Type of linked driver |
| `gain_a, gain_b, gain_c` | `float` | Phase gains |
| `offset_ia, offset_ib, offset_ic` | `float` | Zero current offsets |
| `pinA, pinB, pinC` | `int` | ADC pin numbers |

---

### FOCDriver

Abstract base class for all motor drivers.

**Header:** `src/common/base_classes/FOCDriver.h`

#### Abstract Methods

```cpp
virtual int init() = 0;
virtual void enable() = 0;
virtual void disable() = 0;
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `voltage_power_supply` | `float` | Power supply voltage |
| `voltage_limit` | `float` | Maximum output voltage |
| `pwm_frequency` | `long` | PWM frequency in Hz |
| `initialized` | `bool` | Initialization status |
| `enable_active_high` | `bool` | Enable pin polarity |
| `dead_zone` | `float` | Dead time for complementary PWM |

---

### BLDCDriver

Abstract base class for 3-phase BLDC drivers.

**Header:** `src/common/base_classes/BLDCDriver.h`

Extends `FOCDriver`.

#### Abstract Methods

##### setPwm()

```cpp
virtual void setPwm(float Ua, float Ub, float Uc) = 0
```

Sets PWM duty cycles for all three phases.

| Parameter | Type | Description |
|-----------|------|-------------|
| `Ua` | `float` | Phase A voltage |
| `Ub` | `float` | Phase B voltage |
| `Uc` | `float` | Phase C voltage |

##### setPhaseState()

```cpp
virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) = 0
```

Sets the state of each phase (on/off/high-Z).

| Parameter | Type | Description |
|-----------|------|-------------|
| `sa, sb, sc` | `PhaseState` | Phase states |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `dc_a, dc_b, dc_c` | `float` | Duty cycles [0, 1] |

---

### StepperDriver

Abstract base class for 2-phase stepper drivers.

**Header:** `src/common/base_classes/StepperDriver.h`

Extends `FOCDriver`.

#### Abstract Methods

##### setPwm()

```cpp
virtual void setPwm(float Ua, float Ub) = 0
```

Sets PWM for both phases.

| Parameter | Type | Description |
|-----------|------|-------------|
| `Ua` | `float` | Phase A voltage |
| `Ub` | `float` | Phase B voltage |

##### setPhaseState()

```cpp
virtual void setPhaseState(PhaseState sa, PhaseState sb) = 0
```

Sets the state of each phase.

---

## Sensor Implementations

### Encoder

Quadrature encoder sensor implementation.

**Header:** `src/sensors/Encoder.h`

#### Constructor

```cpp
Encoder(int encA, int encB, float ppr, int index = 0)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `encA` | `int` | Channel A pin |
| `encB` | `int` | Channel B pin |
| `ppr` | `float` | Pulses per revolution |
| `index` | `int` | Index pin (optional, 0 to disable) |

**Example:**
```cpp
Encoder encoder = Encoder(2, 3, 2048);  // 2048 PPR encoder
Encoder encoder = Encoder(2, 3, 2048, 4);  // With index on pin 4
```

#### Methods

##### init()

```cpp
void init()
```

Initializes encoder pins and state.

##### enableInterrupts()

```cpp
void enableInterrupts(void (*doA)() = nullptr, void (*doB)() = nullptr, void (*doIndex)() = nullptr)
```

Enables hardware interrupts for encoder channels.

| Parameter | Type | Description |
|-----------|------|-------------|
| `doA` | `void (*)()` | Channel A interrupt handler |
| `doB` | `void (*)()` | Channel B interrupt handler |
| `doIndex` | `void (*)()` | Index interrupt handler |

**Example:**
```cpp
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

encoder.init();
encoder.enableInterrupts(doA, doB);
```

##### handleA() / handleB() / handleIndex()

```cpp
void handleA()
void handleB()
void handleIndex()
```

Interrupt handlers for each encoder channel. Call from ISR.

##### getSensorAngle()

```cpp
float getSensorAngle()
```

Returns current angle from encoder count.

##### getVelocity()

```cpp
float getVelocity()
```

Returns velocity calculated from encoder pulses.

##### needsSearch()

```cpp
int needsSearch()
```

Returns 1 if index pulse hasn't been found yet.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pinA, pinB` | `int` | Encoder channel pins |
| `index_pin` | `int` | Index pin |
| `cpr` | `float` | Counts per revolution (4 × PPR) |
| `quadrature` | `Quadrature` | Quadrature mode (ON/OFF) |
| `pullup` | `Pullup` | Pullup resistor mode |

---

### HallSensor

Hall effect sensor implementation for BLDC motors.

**Header:** `src/sensors/HallSensor.h`

#### Constructor

```cpp
HallSensor(int hallA, int hallB, int hallC, int pp)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `hallA` | `int` | Hall sensor A pin |
| `hallB` | `int` | Hall sensor B pin |
| `hallC` | `int` | Hall sensor C pin |
| `pp` | `int` | Number of pole pairs |

**Example:**
```cpp
HallSensor halls = HallSensor(5, 6, 7, 7);  // 7 pole pairs
```

#### Methods

##### init()

```cpp
void init()
```

Initializes hall sensor pins.

##### enableInterrupts()

```cpp
void enableInterrupts(void (*doA)() = nullptr, void (*doB)() = nullptr, void (*doC)() = nullptr)
```

Enables interrupts on hall sensor transitions.

##### handleA() / handleB() / handleC()

```cpp
void handleA()
void handleB()
void handleC()
```

Interrupt handlers for hall sensors.

##### update()

```cpp
void update()
```

Interrupt-safe update function.

##### getSensorAngle()

```cpp
float getSensorAngle()
```

Returns electrical angle from hall state.

##### getVelocity()

```cpp
float getVelocity()
```

Returns velocity from hall transitions.

##### attachSectorCallback()

```cpp
void attachSectorCallback(void (*onSectorChange)(int sector))
```

Attaches a callback for sector changes.

| Parameter | Type | Description |
|-----------|------|-------------|
| `onSectorChange` | `void (*)(int)` | Callback function |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pinA, pinB, pinC` | `int` | Hall sensor pins |
| `pullup` | `Pullup` | Pullup mode |
| `hall_state` | `volatile int8_t` | Current hall state (1-6) |
| `electric_sector` | `volatile int8_t` | Current electrical sector |
| `electric_rotations` | `volatile long` | Electrical rotation count |
| `velocity_max` | `float` | Maximum valid velocity |

---

### MagneticSensorSPI

SPI-based magnetic angle sensor (AS5047, AS5048A, etc.).

**Header:** `src/sensors/MagneticSensorSPI.h`

#### Constructors

```cpp
MagneticSensorSPI(int cs, int bit_resolution, int angle_register)
MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs = -1)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `cs` | `int` | Chip select pin |
| `bit_resolution` | `int` | Sensor bit resolution (e.g., 14) |
| `angle_register` | `int` | Angle register address |
| `config` | `MagneticSensorSPIConfig_s` | Configuration structure |

**Example:**
```cpp
// Manual configuration
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);

// Using predefined config
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, 10);
```

#### Methods

##### init()

```cpp
void init(SPIClass* _spi = &SPI)
```

Initializes the SPI interface.

| Parameter | Type | Description |
|-----------|------|-------------|
| `_spi` | `SPIClass*` | SPI instance (default: SPI) |

##### getSensorAngle()

```cpp
float getSensorAngle()
```

Reads and returns the angle from the sensor.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `spi_mode` | `int` | SPI mode (0-3) |
| `clock_speed` | `long` | SPI clock speed in Hz |

#### Predefined Configurations

```cpp
extern MagneticSensorSPIConfig_s AS5147_SPI;
extern MagneticSensorSPIConfig_s AS5048_SPI;
extern MagneticSensorSPIConfig_s AS5047_SPI;
extern MagneticSensorSPIConfig_s MA730_SPI;
```

---

### MagneticSensorI2C

I2C-based magnetic angle sensor (AS5600, etc.).

**Header:** `src/sensors/MagneticSensorI2C.h`

#### Constructors

```cpp
MagneticSensorI2C(uint8_t chip_address, int bit_resolution, uint8_t angle_register_msb, int msb_bits)
MagneticSensorI2C(MagneticSensorI2CConfig_s config)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `chip_address` | `uint8_t` | I2C device address |
| `bit_resolution` | `int` | Sensor resolution |
| `angle_register_msb` | `uint8_t` | MSB register address |
| `msb_bits` | `int` | Number of bits in MSB register |
| `config` | `MagneticSensorI2CConfig_s` | Configuration structure |

**Example:**
```cpp
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0x0E, 4);
MagneticSensorI2C sensor = MagneticSensorI2C::AS5600();  // Factory method
```

#### Static Factory Methods

##### AS5600()

```cpp
static MagneticSensorI2C AS5600()
```

Creates an AS5600-configured sensor instance.

#### Methods

##### init()

```cpp
void init(TwoWire* _wire = &Wire)
```

Initializes the I2C interface.

| Parameter | Type | Description |
|-----------|------|-------------|
| `_wire` | `TwoWire*` | I2C instance (default: Wire) |

##### getSensorAngle()

```cpp
float getSensorAngle()
```

Reads and returns the angle.

##### checkBus()

```cpp
int checkBus(byte sda_pin = SDA, byte scl_pin = SCL)
```

Checks I2C bus for stuck conditions and attempts recovery.

| Returns | Description |
|---------|-------------|
| `int` | Number of clock cycles needed, or error code |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `currWireError` | `uint8_t` | Last I2C error code |

---

### MagneticSensorAnalog

Analog output magnetic sensor.

**Header:** `src/sensors/MagneticSensorAnalog.h`

#### Constructor

```cpp
MagneticSensorAnalog(uint8_t pinAnalog, int min_raw_count = 0, int max_raw_count = 0)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pinAnalog` | `uint8_t` | Analog input pin |
| `min_raw_count` | `int` | Minimum ADC value |
| `max_raw_count` | `int` | Maximum ADC value |

**Example:**
```cpp
MagneticSensorAnalog sensor = MagneticSensorAnalog(A0, 14, 1010);
```

#### Methods

##### init()

```cpp
void init()
```

Initializes the analog pin.

##### getSensorAngle()

```cpp
float getSensorAngle()
```

Reads ADC and converts to angle.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pinAnalog` | `int` | Analog pin number |
| `raw_count` | `int` | Last raw ADC reading |

---

### MagneticSensorPWM

PWM output magnetic sensor.

**Header:** `src/sensors/MagneticSensorPWM.h`

#### Constructors

```cpp
MagneticSensorPWM(uint8_t pinPWM, int min = 0, int max = 0)
MagneticSensorPWM(uint8_t pinPWM, int freqHz, int total_pwm_clocks, int min_pwm_clocks, int max_pwm_clocks)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pinPWM` | `uint8_t` | PWM input pin |
| `min` | `int` | Minimum pulse width (µs) |
| `max` | `int` | Maximum pulse width (µs) |
| `freqHz` | `int` | PWM frequency |
| `total_pwm_clocks` | `int` | Total PWM period in clocks |
| `min_pwm_clocks` | `int` | Min value in clocks |
| `max_pwm_clocks` | `int` | Max value in clocks |

**Example:**
```cpp
MagneticSensorPWM sensor = MagneticSensorPWM(3, 4, 904);
MagneticSensorPWM sensor = MagneticSensorPWM(3, 920, 4351, 128, 4223);  // AS5600 PWM
```

#### Methods

##### init()

```cpp
void init()
```

Initializes the PWM input.

##### update()

```cpp
void update()
```

Interrupt-safe update for reading pulse width.

##### getSensorAngle()

```cpp
float getSensorAngle()
```

Returns angle from pulse width.

##### handlePWM()

```cpp
void handlePWM()
```

PWM edge interrupt handler.

##### enableInterrupt()

```cpp
void enableInterrupt(void (*doPWM)())
```

Enables interrupt-based reading.

| Parameter | Type | Description |
|-----------|------|-------------|
| `doPWM` | `void (*)()` | Interrupt handler function |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pinPWM` | `int` | PWM input pin |
| `pulse_length_us` | `unsigned long` | Last pulse length |
| `timeout_us` | `unsigned int` | Timeout for pulse reading |

---

### GenericSensor

Custom sensor using callback functions.

**Header:** `src/sensors/GenericSensor.h`

#### Constructor

```cpp
GenericSensor(float (*readCallback)() = nullptr, void (*initCallback)() = nullptr)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `readCallback` | `float (*)()` | Function returning angle |
| `initCallback` | `void (*)()` | Initialization function |

**Example:**
```cpp
float readMySensor() {
    return myCustomAngle;
}
void initMySensor() {
    // Custom init
}
GenericSensor sensor = GenericSensor(readMySensor, initMySensor);
```

#### Methods

##### init()

```cpp
void init()
```

Calls the init callback.

##### getSensorAngle()

```cpp
float getSensorAngle()
```

Calls the read callback and returns the angle.

---

## Driver Implementations

### BLDCDriver3PWM

3-PWM BLDC driver with optional enable pins.

**Header:** `src/drivers/BLDCDriver3PWM.h`

#### Constructor

```cpp
BLDCDriver3PWM(int phA, int phB, int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `phA` | `int` | Phase A PWM pin |
| `phB` | `int` | Phase B PWM pin |
| `phC` | `int` | Phase C PWM pin |
| `en1` | `int` | Enable pin 1 (optional) |
| `en2` | `int` | Enable pin 2 (optional) |
| `en3` | `int` | Enable pin 3 (optional) |

**Example:**
```cpp
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11);  // PWM only
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);  // With single enable
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8, 7, 6);  // Per-phase enable
```

#### Methods

##### init()

```cpp
int init()
```

Initializes PWM pins and timer configuration.

| Returns | Description |
|---------|-------------|
| `int` | 1 on success, 0 on failure |

##### enable() / disable()

```cpp
void enable()
void disable()
```

Enable or disable the driver outputs.

##### setPwm()

```cpp
void setPwm(float Ua, float Ub, float Uc)
```

Sets PWM duty cycles for all phases.

| Parameter | Type | Description |
|-----------|------|-------------|
| `Ua, Ub, Uc` | `float` | Phase voltages (0 to voltage_limit) |

##### setPhaseState()

```cpp
void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc)
```

Sets individual phase states.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pwmA, pwmB, pwmC` | `int` | PWM pin numbers |
| `enableA_pin, enableB_pin, enableC_pin` | `int` | Enable pin numbers |

Inherited from `FOCDriver`:
- `voltage_power_supply`, `voltage_limit`, `pwm_frequency`, `dead_zone`

---

### BLDCDriver6PWM

6-PWM BLDC driver with complementary outputs and dead-time.

**Header:** `src/drivers/BLDCDriver6PWM.h`

#### Constructor

```cpp
BLDCDriver6PWM(int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en = NOT_SET)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `phA_h` | `int` | Phase A high-side PWM |
| `phA_l` | `int` | Phase A low-side PWM |
| `phB_h` | `int` | Phase B high-side PWM |
| `phB_l` | `int` | Phase B low-side PWM |
| `phC_h` | `int` | Phase C high-side PWM |
| `phC_l` | `int` | Phase C low-side PWM |
| `en` | `int` | Enable pin (optional) |

**Example:**
```cpp
BLDCDriver6PWM driver = BLDCDriver6PWM(5, 6, 9, 10, 3, 11, 8);
```

#### Methods

Same as [BLDCDriver3PWM](#bldcdriver3pwm):
- `init()`, `enable()`, `disable()`, `setPwm()`, `setPhaseState()`

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pwmA_h, pwmA_l` | `int` | Phase A high/low pins |
| `pwmB_h, pwmB_l` | `int` | Phase B high/low pins |
| `pwmC_h, pwmC_l` | `int` | Phase C high/low pins |
| `enable_pin` | `int` | Enable pin |
| `dead_zone` | `float` | Dead time (0-1) |
| `phase_state[3]` | `PhaseState` | Current phase states |

---

### StepperDriver2PWM

2-PWM stepper driver with direction pins.

**Header:** `src/drivers/StepperDriver2PWM.h`

#### Constructors

```cpp
StepperDriver2PWM(int pwm1, int dir1, int pwm2, int dir2, int en1 = NOT_SET, int en2 = NOT_SET)
StepperDriver2PWM(int pwm1, int* in1, int pwm2, int* in2, int en1 = NOT_SET, int en2 = NOT_SET)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pwm1` | `int` | Phase 1 PWM pin |
| `dir1` | `int` | Phase 1 direction pin |
| `pwm2` | `int` | Phase 2 PWM pin |
| `dir2` | `int` | Phase 2 direction pin |
| `in1` | `int*` | Array of 2 direction pins for phase 1 |
| `in2` | `int*` | Array of 2 direction pins for phase 2 |
| `en1, en2` | `int` | Enable pins |

**Example:**
```cpp
StepperDriver2PWM driver = StepperDriver2PWM(5, 4, 6, 7);
```

#### Methods

##### init()

```cpp
int init()
```

Initializes driver hardware.

##### enable() / disable()

```cpp
void enable()
void disable()
```

##### setPwm()

```cpp
void setPwm(float Ua, float Ub)
```

Sets PWM for both phases.

##### setPhaseState()

```cpp
void setPhaseState(PhaseState sa, PhaseState sb)
```

Sets phase states.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pwm1, pwm2` | `int` | PWM pins |
| `dir1a, dir1b` | `int` | Direction pins phase 1 |
| `dir2a, dir2b` | `int` | Direction pins phase 2 |
| `enable_pin1, enable_pin2` | `int` | Enable pins |

---

### StepperDriver4PWM

4-PWM stepper driver (H-bridge per phase).

**Header:** `src/drivers/StepperDriver4PWM.h`

#### Constructor

```cpp
StepperDriver4PWM(int ph1A, int ph1B, int ph2A, int ph2B, int en1 = NOT_SET, int en2 = NOT_SET)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `ph1A` | `int` | Phase 1A PWM |
| `ph1B` | `int` | Phase 1B PWM |
| `ph2A` | `int` | Phase 2A PWM |
| `ph2B` | `int` | Phase 2B PWM |
| `en1, en2` | `int` | Enable pins |

**Example:**
```cpp
StepperDriver4PWM driver = StepperDriver4PWM(5, 6, 9, 10);
```

#### Methods

Same as [StepperDriver2PWM](#stepperdriver2pwm).

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pwm1A, pwm1B` | `int` | Phase 1 PWM pins |
| `pwm2A, pwm2B` | `int` | Phase 2 PWM pins |
| `enable_pin1, enable_pin2` | `int` | Enable pins |

---

## Current Sense Implementations

### InlineCurrentSense

Inline (series) current sensing using shunt resistors.

**Header:** `src/current_sense/InlineCurrentSense.h`

#### Constructors

```cpp
InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET)
InlineCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `shunt_resistor` | `float` | Shunt resistance in Ohms |
| `gain` | `float` | Amplifier gain |
| `mVpA` | `float` | Millivolts per Amp (combined shunt × gain) |
| `pinA, pinB, pinC` | `int` | ADC pins for each phase |

**Example:**
```cpp
// Shunt + gain method
InlineCurrentSense current = InlineCurrentSense(0.01, 50, A0, A1, A2);

// mV/A method
InlineCurrentSense current = InlineCurrentSense(66, A0, A1, A2);  // 66mV/A
```

#### Methods

##### init()

```cpp
int init()
```

Initializes ADC and calibrates zero offsets.

| Returns | Description |
|---------|-------------|
| `int` | 1 on success, 0 on failure |

##### getPhaseCurrents()

```cpp
PhaseCurrent_s getPhaseCurrents()
```

Reads and returns phase currents.

| Returns | Description |
|---------|-------------|
| `PhaseCurrent_s` | Phase currents in Amps |

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `shunt_resistor` | `float` | Shunt resistance |
| `amp_gain` | `float` | Amplifier gain |
| `volts_to_amps_ratio` | `float` | Conversion factor |

---

### LowsideCurrentSense

Low-side current sensing (shunts between motor and ground).

**Header:** `src/current_sense/LowsideCurrentSense.h`

#### Constructors

```cpp
LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET)
LowsideCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET)
```

Same parameters as [InlineCurrentSense](#inlinecurrentsense).

**Note:** Low-side sensing requires synchronized ADC sampling with PWM. The driver must trigger ADC reads at specific points in the PWM cycle.

#### Methods

Same as [InlineCurrentSense](#inlinecurrentsense).

---

### GenericCurrentSense

Custom current sensing using callbacks.

**Header:** `src/current_sense/GenericCurrentSense.h`

#### Constructor

```cpp
GenericCurrentSense(PhaseCurrent_s (*readCallback)() = nullptr, void (*initCallback)() = nullptr)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `readCallback` | `PhaseCurrent_s (*)()` | Function returning phase currents |
| `initCallback` | `void (*)()` | Initialization function |

**Example:**
```cpp
PhaseCurrent_s readCurrents() {
    return {readPhaseA(), readPhaseB(), readPhaseC()};
}
GenericCurrentSense current = GenericCurrentSense(readCurrents, nullptr);
```

#### Methods

##### init()

```cpp
int init()
```

Calls init callback if provided.

##### getPhaseCurrents()

```cpp
PhaseCurrent_s getPhaseCurrents()
```

Calls read callback.

##### driverAlign()

```cpp
int driverAlign(float align_voltage, bool modulation_centered = false)
```

Returns 1 (no auto-alignment for generic sensing).

---

## Communication Classes

### Commander

Serial command interface for motor control and tuning.

**Header:** `src/communication/Commander.h`

#### Constructors

```cpp
Commander(Stream &serial, char eol = '\n', bool echo = false)
Commander(char eol = '\n', bool echo = false)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `serial` | `Stream&` | Serial port reference |
| `eol` | `char` | End-of-line character |
| `echo` | `bool` | Echo received characters |

**Example:**
```cpp
Commander command = Commander(Serial);
```

#### Methods

##### add()

```cpp
void add(char id, CommandCallback onCommand, const char* label = nullptr)
```

Registers a command callback.

| Parameter | Type | Description |
|-----------|------|-------------|
| `id` | `char` | Single character command ID |
| `onCommand` | `CommandCallback` | Callback function `void(char*)` |
| `label` | `const char*` | Human-readable label |

**Example:**
```cpp
void onMotor(char* cmd) { command.motor(&motor, cmd); }
command.add('M', onMotor, "motor");
```

##### run()

```cpp
void run()
void run(Stream &serial, char eol = '\n')
void run(char* user_input)
```

Processes incoming commands.

**Example:**
```cpp
void loop() {
    command.run();
}
```

##### motor()

```cpp
void motor(FOCMotor* motor, char* user_cmd)
```

Built-in motor command handler.

| Command | Description |
|---------|-------------|
| `?` | Print help |
| `E` | Enable/disable (E0/E1) |
| `D` | Motion downsample |
| `C` | Motion control mode |
| `T` | Torque control mode |
| `V` | Velocity PID |
| `A` | Angle P controller |
| `L` | Limits (voltage, current, velocity) |
| `R` | Phase resistance |
| `I` | Current PID (Q and D axis) |
| `F` | Low-pass filter time constants |
| `S` | Sensor offset and direction |
| `W` | PWM modulation |
| `M` | Monitoring variables |

##### pid()

```cpp
void pid(PIDController* pid, char* user_cmd)
```

PID parameter command handler.

| Command | Description |
|---------|-------------|
| `P` | Proportional gain |
| `I` | Integral gain |
| `D` | Derivative gain |
| `R` | Output ramp |
| `L` | Output limit |

##### lpf()

```cpp
void lpf(LowPassFilter* lpf, char* user_cmd)
```

Low-pass filter command handler.

| Command | Description |
|---------|-------------|
| `F` | Filter time constant |

##### scalar()

```cpp
void scalar(float* value, char* user_cmd)
```

Generic scalar value handler.

##### target()

```cpp
void target(FOCMotor* motor, char* user_cmd, char* separator = nullptr)
```

Sets motor target value.

##### motion()

```cpp
void motion(FOCMotor* motor, char* user_cmd, char* separator = nullptr)
```

Motion control command (target + limits).

##### isSentinel()

```cpp
bool isSentinel(char ch)
```

Checks if character is command delimiter.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `verbose` | `VerboseMode` | Output verbosity level |
| `decimal_places` | `uint8_t` | Float decimal precision |
| `com_port` | `Stream*` | Current serial port |
| `eol` | `char` | End-of-line character |
| `echo` | `bool` | Echo mode |

---

### StepDirListener

Step/direction interface for CNC-style control.

**Header:** `src/communication/StepDirListener.h`

#### Constructor

```cpp
StepDirListener(int pinStep, int pinDir, float counter_to_value = 1.0f/200.0f)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `pinStep` | `int` | Step pulse input pin |
| `pinDir` | `int` | Direction input pin |
| `counter_to_value` | `float` | Step-to-angle conversion (default: 200 steps/rev) |

**Example:**
```cpp
StepDirListener stepdir = StepDirListener(2, 3, _2PI/200.0);  // 200 steps = 2π
```

#### Methods

##### init()

```cpp
void init()
```

Initializes step/dir pins.

##### enableInterrupt()

```cpp
void enableInterrupt(void (*handleStep)())
```

Enables interrupt for step pulses.

| Parameter | Type | Description |
|-----------|------|-------------|
| `handleStep` | `void (*)()` | ISR function |

**Example:**
```cpp
void onStep() { stepdir.handle(); }
stepdir.enableInterrupt(onStep);
```

##### handle()

```cpp
void handle()
```

Step interrupt handler. Call from ISR.

##### getValue()

```cpp
float getValue()
```

Returns current position value.

| Returns | Description |
|---------|-------------|
| `float` | Position in configured units |

##### attach()

```cpp
void attach(float* variable)
```

Attaches a variable for automatic updates.

| Parameter | Type | Description |
|-----------|------|-------------|
| `variable` | `float*` | Pointer to target variable |

**Example:**
```cpp
stepdir.attach(&motor.target);  // Auto-update motor target
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `pin_step` | `int` | Step pin |
| `pin_dir` | `int` | Direction pin |
| `count` | `long` | Step counter |
| `polarity` | `decltype(RISING)` | Step edge polarity |

---

### SimpleFOCDebug

Debug output utility class.

**Header:** `src/communication/SimpleFOCDebug.h`

#### Static Methods

##### enable()

```cpp
static void enable(Print* debugPrint = &Serial)
```

Enables debug output to specified print target.

| Parameter | Type | Description |
|-----------|------|-------------|
| `debugPrint` | `Print*` | Output target (default: Serial) |

**Example:**
```cpp
SimpleFOCDebug::enable(&Serial);
```

##### println()

```cpp
static void println(const char* msg)
static void println(const char* msg, float val)
static void println(const char* msg, int val)
static void println()
```

Prints debug messages with optional values.

##### print()

```cpp
static void print(const char* msg)
static void print(int val)
static void print(float val)
```

Prints without newline.

#### Macros

```cpp
SIMPLEFOC_DEBUG("message")
SIMPLEFOC_DEBUG("value:", 123.456f)
```

Debug macro using flash strings. Disabled when `SIMPLEFOC_DISABLE_DEBUG` is defined.

---

## Control Utilities

### PIDController

PID controller with output ramping and limits.

**Header:** `src/common/pid.h`

#### Constructor

```cpp
PIDController(float P, float I, float D, float ramp, float limit)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `P` | `float` | Proportional gain |
| `I` | `float` | Integral gain |
| `D` | `float` | Derivative gain |
| `ramp` | `float` | Maximum output change per second (0 = disabled) |
| `limit` | `float` | Maximum output magnitude |

**Example:**
```cpp
PIDController pid = PIDController(0.5, 10, 0, 10000, 12);
```

#### Methods

##### operator()

```cpp
float operator()(float error)
```

Calculates PID output for given error.

| Parameter | Type | Description |
|-----------|------|-------------|
| `error` | `float` | Error value (setpoint - measured) |

| Returns | Description |
|---------|-------------|
| `float` | Control output |

**Example:**
```cpp
float output = pid(target - measured);
```

##### reset()

```cpp
void reset()
```

Resets integrator and derivative state.

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `P` | `float` | Proportional gain |
| `I` | `float` | Integral gain |
| `D` | `float` | Derivative gain |
| `output_ramp` | `float` | Output ramp rate |
| `limit` | `float` | Output limit |
| `error_prev` | `float` | Previous error (internal) |
| `output_prev` | `float` | Previous output (internal) |
| `integral_prev` | `float` | Integral accumulator (internal) |
| `timestamp_prev` | `unsigned long` | Previous timestamp (internal) |

---

### LowPassFilter

First-order low-pass filter.

**Header:** `src/common/lowpass_filter.h`

#### Constructor

```cpp
LowPassFilter(float Tf)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `Tf` | `float` | Filter time constant in seconds |

**Example:**
```cpp
LowPassFilter filter = LowPassFilter(0.01);  // 10ms time constant
```

#### Methods

##### operator()

```cpp
float operator()(float x)
```

Filters input value.

| Parameter | Type | Description |
|-----------|------|-------------|
| `x` | `float` | Input value |

| Returns | Description |
|---------|-------------|
| `float` | Filtered output |

**Example:**
```cpp
float filtered = filter(raw_value);
```

#### Properties

| Property | Type | Description |
|----------|------|-------------|
| `Tf` | `float` | Time constant in seconds |
| `y_prev` | `float` | Previous output (internal) |
| `timestamp_prev` | `unsigned long` | Previous timestamp (internal) |

---

## Utility Functions

### FOC Utils

**Header:** `src/common/foc_utils.h`

#### Mathematical Functions

##### _sin()

```cpp
float _sin(float a)
```

Fast sine approximation using lookup table.

| Parameter | Type | Description |
|-----------|------|-------------|
| `a` | `float` | Angle in radians [0, 2π] |

| Returns | Description |
|---------|-------------|
| `float` | Sine value [-1, 1] |

**Performance:** ~40µs on Arduino UNO

##### _cos()

```cpp
float _cos(float a)
```

Fast cosine approximation.

| Parameter | Type | Description |
|-----------|------|-------------|
| `a` | `float` | Angle in radians [0, 2π] |

| Returns | Description |
|---------|-------------|
| `float` | Cosine value [-1, 1] |

**Performance:** ~50µs on Arduino UNO

##### _sincos()

```cpp
void _sincos(float a, float* s, float* c)
```

Calculates both sine and cosine in one call.

| Parameter | Type | Description |
|-----------|------|-------------|
| `a` | `float` | Angle in radians |
| `s` | `float*` | Output sine value |
| `c` | `float*` | Output cosine value |

##### _atan2()

```cpp
float _atan2(float y, float x)
```

Fast atan2 approximation.

| Parameter | Type | Description |
|-----------|------|-------------|
| `y` | `float` | Y coordinate |
| `x` | `float` | X coordinate |

| Returns | Description |
|---------|-------------|
| `float` | Angle in radians |

##### _normalizeAngle()

```cpp
float _normalizeAngle(float angle)
```

Normalizes angle to [0, 2π] range.

| Parameter | Type | Description |
|-----------|------|-------------|
| `angle` | `float` | Input angle |

| Returns | Description |
|---------|-------------|
| `float` | Normalized angle |

##### _electricalAngle()

```cpp
float _electricalAngle(float shaft_angle, int pole_pairs)
```

Calculates electrical angle from mechanical angle.

| Parameter | Type | Description |
|-----------|------|-------------|
| `shaft_angle` | `float` | Mechanical shaft angle |
| `pole_pairs` | `int` | Number of pole pairs |

| Returns | Description |
|---------|-------------|
| `float` | Electrical angle [0, 2π] |

##### _sqrtApprox()

```cpp
float _sqrtApprox(float value)
```

Fast square root approximation using inverse square root.

| Parameter | Type | Description |
|-----------|------|-------------|
| `value` | `float` | Input value |

| Returns | Description |
|---------|-------------|
| `float` | Approximate square root |

#### Macros

| Macro | Description |
|-------|-------------|
| `_sign(a)` | Returns -1, 0, or 1 based on sign |
| `_round(x)` | Rounds float to nearest integer |
| `_constrain(amt, low, high)` | Constrains value to range |
| `_sqrt(a)` | Alias for `_sqrtApprox` |
| `_isset(a)` | Checks if value is set (not NOT_SET) |
| `_swap(a, b)` | Swaps two values |
| `_powtwo(x)` | Returns 2^x |

#### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `_PI` | 3.14159... | Pi |
| `_2PI` | 6.28318... | 2π |
| `_PI_2` | 1.57079... | π/2 |
| `_PI_3` | 1.04719... | π/3 |
| `_PI_6` | 0.52359... | π/6 |
| `_3PI_2` | 4.71238... | 3π/2 |
| `_SQRT3` | 1.73205... | √3 |
| `_1_SQRT3` | 0.57735... | 1/√3 |
| `_2_SQRT3` | 1.15470... | 2/√3 |
| `_SQRT3_2` | 0.86602... | √3/2 |
| `_SQRT2` | 1.41421... | √2 |
| `_120_D2R` | 2.09439... | 120° in radians |
| `_RPM_TO_RADS` | 0.10471... | RPM to rad/s conversion |
| `NOT_SET` | -12345.0 | Unset value marker |
| `_HIGH_IMPEDANCE` | 0 | High-Z state |
| `_ACTIVE` | 1 | Active state |
| `_NC` | NOT_SET as int | Not connected marker |

---

### Time Utils

**Header:** `src/common/time_utils.h`

##### _delay()

```cpp
void _delay(unsigned long ms)
```

Platform-independent delay function.

| Parameter | Type | Description |
|-----------|------|-------------|
| `ms` | `unsigned long` | Delay in milliseconds |

##### _micros()

```cpp
unsigned long _micros()
```

Platform-independent microsecond timer.

| Returns | Description |
|---------|-------------|
| `unsigned long` | Microseconds since startup |

---

## Data Structures

### DQCurrent_s

D-Q axis current values.

```cpp
struct DQCurrent_s {
    float d;  // Direct axis current
    float q;  // Quadrature axis current
};
```

### PhaseCurrent_s

Three-phase current values.

```cpp
struct PhaseCurrent_s {
    float a;  // Phase A current
    float b;  // Phase B current
    float c;  // Phase C current
};
```

### DQVoltage_s

D-Q axis voltage values.

```cpp
struct DQVoltage_s {
    float d;  // Direct axis voltage
    float q;  // Quadrature axis voltage
};
```

### ABCurrent_s

Alpha-beta (Clarke transform) current values.

```cpp
struct ABCurrent_s {
    float alpha;  // Alpha axis current
    float beta;   // Beta axis current
};
```

### MagneticSensorSPIConfig_s

SPI sensor configuration.

```cpp
struct MagneticSensorSPIConfig_s {
    int bit_resolution;   // Sensor resolution in bits
    int angle_register;   // Angle register address
    int data_start_bit;   // Starting bit position
    int command_rw_bit;   // Read/write bit position
    int command_parity_bit; // Parity bit position
    int spi_mode;         // SPI mode (0-3)
};
```

### MagneticSensorI2CConfig_s

I2C sensor configuration.

```cpp
struct MagneticSensorI2CConfig_s {
    int chip_address;       // I2C address
    int bit_resolution;     // Sensor resolution
    int angle_register;     // MSB register address
    int data_start_bit;     // Starting bit position
};
```

---

## Enumerations

### MotionControlType

Motion control loop selection.

```cpp
enum MotionControlType : uint8_t {
    torque            = 0x00,  // Torque/current control
    velocity          = 0x01,  // Velocity control
    angle             = 0x02,  // Position control
    velocity_openloop = 0x03,  // Open-loop velocity
    angle_openloop    = 0x04   // Open-loop position
};
```

### TorqueControlType

Torque control method selection.

```cpp
enum TorqueControlType : uint8_t {
    voltage     = 0x00,  // Voltage mode (no current sensing)
    dc_current  = 0x01,  // DC current control
    foc_current = 0x02   // FOC current control (d-q)
};
```

### FOCModulationType

PWM modulation strategy.

```cpp
enum FOCModulationType : uint8_t {
    SinePWM       = 0x00,  // Sinusoidal PWM
    SpaceVectorPWM = 0x01,  // Space Vector PWM
    Trapezoid_120 = 0x02,  // Trapezoidal 120°
    Trapezoid_150 = 0x03   // Trapezoidal 150°
};
```

### FOCMotorStatus

Motor state machine status.

```cpp
enum FOCMotorStatus : uint8_t {
    motor_uninitialized = 0x00,  // Not initialized
    motor_initializing  = 0x01,  // Initializing
    motor_uncalibrated  = 0x02,  // Not calibrated
    motor_calibrating   = 0x03,  // Calibrating
    motor_ready         = 0x04,  // Ready for operation
    motor_error         = 0x05,  // Error state
    motor_calib_failed  = 0x06,  // Calibration failed
    motor_init_failed   = 0x07   // Initialization failed
};
```

### Direction

Rotation direction.

```cpp
enum Direction : int8_t {
    CW      = 1,   // Clockwise
    CCW     = -1,  // Counter-clockwise
    UNKNOWN = 0    // Unknown/unset
};
```

### Pullup

Internal pullup resistor configuration.

```cpp
enum Pullup : uint8_t {
    USE_INTERN = 0x00,  // Use internal pullup
    USE_EXTERN = 0x01   // Use external pullup
};
```

### PhaseState

Phase output state.

```cpp
enum PhaseState : uint8_t {
    PHASE_OFF = 0,  // Phase disabled
    PHASE_ON  = 1,  // Phase active
    PHASE_HI  = 2,  // Phase high (for 6-PWM)
    PHASE_LO  = 3   // Phase low (for 6-PWM)
};
```

### DriverType

Motor driver type.

```cpp
enum DriverType : uint8_t {
    UnknownDriver = 0,
    BLDC          = 1,
    Stepper       = 2,
    Hybrid        = 3
};
```

### Quadrature

Encoder quadrature mode.

```cpp
enum Quadrature : uint8_t {
    ON  = 0x00,  // Quadrature decoding enabled
    OFF = 0x01   // Quadrature decoding disabled
};
```

### VerboseMode

Commander output verbosity.

```cpp
enum VerboseMode : uint8_t {
    nothing          = 0x00,  // No output
    on_request       = 0x01,  // Output only when requested
    user_friendly    = 0x02,  // Human-readable output
    machine_readable = 0x03   // Machine-parseable output
};
```

---

## Simulink FOC Integration

### SimulinkFOC

Simulink code-generated FOC algorithm interface.

**Header:** `src/common/simulink_foc/SimulinkFOC.h`

#### Constructor

```cpp
SimulinkFOC()
```

Creates a new Simulink FOC instance.

#### Methods

##### init()

```cpp
int init(const SimulinkFOC_Params_T* params)
```

Initializes the FOC algorithm with parameters.

| Parameter | Type | Description |
|-----------|------|-------------|
| `params` | `SimulinkFOC_Params_T*` | Complete parameter set |

| Returns | Description |
|---------|-------------|
| `int` | 0 on success, non-zero on failure |

##### reset()

```cpp
void reset()
```

Resets algorithm state (integrators, filters).

##### setInputs()

```cpp
void setInputs(const SimulinkFOC_Inputs_T* inputs)
```

Sets algorithm inputs for current step.

| Parameter | Type | Description |
|-----------|------|-------------|
| `inputs` | `SimulinkFOC_Inputs_T*` | Input values |

##### step()

```cpp
void step()
```

Executes one complete algorithm step (inner + outer loop).

##### runInnerLoop()

```cpp
void runInnerLoop()
```

Executes inner FOC loop only (current/voltage control).

##### runOuterLoop()

```cpp
void runOuterLoop()
```

Executes outer control loop only (velocity/position).

##### getOutputs()

```cpp
void getOutputs(SimulinkFOC_Outputs_T* outputs)
```

Retrieves algorithm outputs.

| Parameter | Type | Description |
|-----------|------|-------------|
| `outputs` | `SimulinkFOC_Outputs_T*` | Output structure to fill |

##### updateParams()

```cpp
void updateParams(const SimulinkFOC_Params_T* params)
```

Updates all parameters.

##### updateGains()

```cpp
void updateGains(const SimulinkFOC_ControlGains_T* gains)
```

Updates controller gains only.

##### updateConfig()

```cpp
void updateConfig(const SimulinkFOC_ControlConfig_T* config)
```

Updates control configuration.

##### updateSensorCalib()

```cpp
void updateSensorCalib(const SimulinkFOC_SensorCalib_T* calib)
```

Updates sensor calibration values.

##### updateLimits()

```cpp
void updateLimits(const SimulinkFOC_Limits_T* limits)
```

Updates control limits.

##### getState()

```cpp
void getState(SimulinkFOC_State_T* state)
```

Retrieves internal algorithm state.

##### isInitialized()

```cpp
bool isInitialized()
```

Returns initialization status.

##### Output Getters

```cpp
float getUa() const;   // Phase A voltage
float getUb() const;   // Phase B voltage
float getUc() const;   // Phase C voltage
float getElectricalAngle() const;
float getVoltageQ() const;
float getVoltageD() const;
float getCurrentQ() const;
float getCurrentD() const;
uint8_t getPhaseStateA() const;
uint8_t getPhaseStateB() const;
uint8_t getPhaseStateC() const;
```

---

### SimulinkFOCAdapter

Adapter bridging SimpleFOC motor with Simulink FOC algorithm.

**Header:** `src/common/simulink_foc/SimulinkFOCAdapter.h`

#### Constructor

```cpp
SimulinkFOCAdapter()
```

Creates a new adapter instance.

#### Methods

##### init()

```cpp
int init(FOCMotor* motor, BLDCDriver* driver, Sensor* sensor, CurrentSense* current_sense)
```

Initializes adapter with motor components.

| Parameter | Type | Description |
|-----------|------|-------------|
| `motor` | `FOCMotor*` | Motor instance |
| `driver` | `BLDCDriver*` | Driver instance |
| `sensor` | `Sensor*` | Position sensor (can be null) |
| `current_sense` | `CurrentSense*` | Current sensing (can be null) |

| Returns | Description |
|---------|-------------|
| `int` | 0 on success, non-zero on failure |

##### syncParameters()

```cpp
void syncParameters()
```

Synchronizes motor parameters to Simulink component. Call after changing PID gains or limits.

##### runLoopFOC()

```cpp
void runLoopFOC()
```

Executes inner FOC loop. Replacement for motor's `loopFOC()`.

##### runMove()

```cpp
void runMove(float target)
```

Executes motion control loop. Replacement for motor's `move()`.

| Parameter | Type | Description |
|-----------|------|-------------|
| `target` | `float` | Control target |

##### reset()

```cpp
void reset()
```

Resets algorithm state.

##### isInitialized()

```cpp
bool isInitialized() const
```

Returns initialization status.

##### getFOC()

```cpp
SimulinkFOC& getFOC()
const SimulinkFOC& getFOC() const
```

Returns reference to underlying SimulinkFOC instance.

##### Voltage Getters

```cpp
float getUa() const;  // Phase A voltage
float getUb() const;  // Phase B voltage
float getUc() const;  // Phase C voltage
```

##### Phase State Getters

```cpp
uint8_t getPhaseStateA() const;
uint8_t getPhaseStateB() const;
uint8_t getPhaseStateC() const;
```

---

### Simulink FOC Types

**Header:** `src/common/simulink_foc/SimulinkFOCTypes.h`

#### SimulinkFOC_MotorParams_T

Motor physical parameters.

```cpp
struct SimulinkFOC_MotorParams_T {
    float phase_resistance;   // Phase resistance (Ohms)
    float phase_inductance;   // Phase inductance (H)
    float pole_pairs;         // Number of pole pairs
    float KV_rating;          // Motor KV rating
};
```

#### SimulinkFOC_SensorCalib_T

Sensor calibration values.

```cpp
struct SimulinkFOC_SensorCalib_T {
    float zero_electric_angle;  // Zero angle offset
    int8_t sensor_direction;    // Direction (CW=1, CCW=-1)
    float sensor_offset;        // Additional offset
};
```

#### SimulinkFOC_ControlGains_T

Control loop gains.

```cpp
struct SimulinkFOC_ControlGains_T {
    // Q-axis current PID
    float current_q_P, current_q_I, current_q_D;
    float current_q_ramp, current_q_limit;

    // D-axis current PID
    float current_d_P, current_d_I, current_d_D;
    float current_d_ramp, current_d_limit;

    // Velocity PID
    float velocity_P, velocity_I, velocity_D;
    float velocity_ramp, velocity_limit;

    // Position P controller
    float angle_P, angle_limit;

    // Low-pass filter time constants
    float current_q_filter_Tf, current_d_filter_Tf;
    float velocity_filter_Tf, angle_filter_Tf;
};
```

#### SimulinkFOC_Limits_T

Control limits.

```cpp
struct SimulinkFOC_Limits_T {
    float voltage_limit;         // Max voltage
    float current_limit;         // Max current
    float velocity_limit;        // Max velocity
    float voltage_power_supply;  // Supply voltage
};
```

#### SimulinkFOC_ControlConfig_T

Control configuration.

```cpp
struct SimulinkFOC_ControlConfig_T {
    uint8_t torque_mode;       // Torque control type
    uint8_t motion_mode;       // Motion control type
    uint8_t modulation_type;   // PWM modulation type
    uint8_t modulation_centered;  // Centered modulation
    uint16_t motion_downsample;   // Outer loop divider
};
```

#### SimulinkFOC_Params_T

Complete parameter set.

```cpp
struct SimulinkFOC_Params_T {
    SimulinkFOC_MotorParams_T motor;
    SimulinkFOC_SensorCalib_T sensor;
    SimulinkFOC_ControlGains_T gains;
    SimulinkFOC_Limits_T limits;
    SimulinkFOC_ControlConfig_T config;
};
```

#### SimulinkFOC_Inputs_T

Algorithm inputs.

```cpp
struct SimulinkFOC_Inputs_T {
    float mechanical_angle;    // Shaft angle (rad)
    float shaft_velocity;      // Shaft velocity (rad/s)
    float phase_current_a;     // Phase A current (A)
    float phase_current_b;     // Phase B current (A)
    float phase_current_c;     // Phase C current (A)
    float target;              // Control target
    float feed_forward_velocity;  // FF velocity
    float dt;                  // Time step (s)
    uint8_t enabled;           // Enable flag
    uint8_t current_sense_available;  // Current sense present
};
```

#### SimulinkFOC_Outputs_T

Algorithm outputs.

```cpp
struct SimulinkFOC_Outputs_T {
    float Ua, Ub, Uc;         // Phase voltages
    float Ualpha, Ubeta;      // Alpha-beta voltages
    float electrical_angle;    // Electrical angle
    float voltage_q, voltage_d;  // D-Q voltages
    float current_q, current_d;  // D-Q currents
    float current_sp;          // Current setpoint
    float shaft_velocity_sp;   // Velocity setpoint
    float shaft_angle_sp;      // Angle setpoint
    float shaft_angle;         // Current angle
    float voltage_bemf;        // Back-EMF voltage
    uint8_t phase_state_a;     // Phase A state
    uint8_t phase_state_b;     // Phase B state
    uint8_t phase_state_c;     // Phase C state
};
```

#### SimulinkFOC_State_T

Internal algorithm state.

```cpp
struct SimulinkFOC_State_T {
    float current_q_integral;
    float current_d_integral;
    float velocity_integral;
    float angle_error;
    float filtered_velocity;
    float filtered_current_q;
    float filtered_current_d;
};
```

#### Simulink FOC Constants

```cpp
// Torque modes
#define SFOC_TORQUE_VOLTAGE      0
#define SFOC_TORQUE_DC_CURRENT   1
#define SFOC_TORQUE_FOC_CURRENT  2

// Motion modes
#define SFOC_MOTION_TORQUE       0
#define SFOC_MOTION_VELOCITY     1
#define SFOC_MOTION_ANGLE        2
#define SFOC_MOTION_VELOCITY_OL  3
#define SFOC_MOTION_ANGLE_OL     4

// Modulation types
#define SFOC_MOD_SINE_PWM        0
#define SFOC_MOD_SPACE_VECTOR    1
#define SFOC_MOD_TRAPEZOID_120   2
#define SFOC_MOD_TRAPEZOID_150   3

// Phase states
#define SFOC_PHASE_OFF           0
#define SFOC_PHASE_ON            1
```

---

## Quick Reference

### Minimal BLDC Setup

```cpp
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
Encoder encoder = Encoder(2, 3, 2048);

void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

void setup() {
    encoder.init();
    encoder.enableInterrupts(doA, doB);
    motor.linkSensor(&encoder);

    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::velocity;
    motor.init();
    motor.initFOC();
}

void loop() {
    motor.loopFOC();
    motor.move(10);  // 10 rad/s
}
```

### Current Sensing Setup

```cpp
InlineCurrentSense current = InlineCurrentSense(0.01, 50, A0, A1, A2);

void setup() {
    // ... driver and sensor setup ...
    current.init();
    motor.linkCurrentSense(&current);
    motor.torque_controller = TorqueControlType::foc_current;
    motor.init();
    motor.initFOC();
}
```

### Commander Setup

```cpp
Commander command = Commander(Serial);
void onMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
    Serial.begin(115200);
    command.add('M', onMotor, "motor");
}

void loop() {
    motor.loopFOC();
    motor.move();
    command.run();
}
```

---

*Documentation generated for Arduino-FOC library*
