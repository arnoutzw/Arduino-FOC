#ifndef SIMULINK_FOC_CONFIG_H
#define SIMULINK_FOC_CONFIG_H

/**
 * Simulink FOC Configuration
 *
 * This header contains configuration options for the Simulink code-generated
 * FOC algorithm integration.
 *
 * Define SIMPLEFOC_USE_SIMULINK_FOC before including SimpleFOC.h to enable
 * the Simulink FOC algorithm by default, or call enableSimulinkFOC() on
 * motor instances to switch at runtime.
 */

/**
 * Enable Simulink FOC by default
 * Uncomment to use Simulink-generated algorithm as the default
 */
// #define SIMPLEFOC_USE_SIMULINK_FOC

/**
 * Allow runtime switching between native and Simulink FOC
 * When enabled, motors have enableSimulinkFOC()/disableSimulinkFOC() methods
 */
#define SIMPLEFOC_SIMULINK_RUNTIME_SWITCH 1

/**
 * Include debugging/diagnostic features for Simulink FOC
 */
#define SIMPLEFOC_SIMULINK_DEBUG 0

/**
 * Simulink model sample time (for reference)
 * The actual step time should be provided in the inputs.dt field
 */
#define SIMPLEFOC_SIMULINK_SAMPLE_TIME_US 100  // 100us = 10kHz

#endif // SIMULINK_FOC_CONFIG_H
