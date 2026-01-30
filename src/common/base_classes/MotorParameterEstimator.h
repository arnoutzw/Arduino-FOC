#ifndef MOTOR_PARAMETER_ESTIMATOR_H
#define MOTOR_PARAMETER_ESTIMATOR_H

#include "Arduino.h"
#include "../foc_utils.h"
#include "../time_utils.h"
#include "../lowpass_filter.h"

// Default configuration for the RLS estimator
#define DEF_RLS_FORGETTING_FACTOR 0.995f    // Forgetting factor (0.95-0.999)
#define DEF_RLS_INITIAL_COVARIANCE 1000.0f  // Initial covariance diagonal value
#define DEF_RLS_MIN_VELOCITY 5.0f           // Minimum velocity for estimation [rad/s electrical]
#define DEF_RLS_MIN_CURRENT 0.1f            // Minimum current for estimation [A]
#define DEF_RLS_ESTIMATE_FILTER_TF 0.01f    // Low-pass filter time constant for estimates

/**
 * Structure holding the estimated motor parameters
 */
struct MotorParameters_s {
    float Ld;           //!< d-axis inductance [H]
    float Lq;           //!< q-axis inductance [H]
    float flux_linkage; //!< Permanent magnet flux linkage [Wb]
    float Rs;           //!< Phase resistance [Ohm] (optional, can be provided)
};

/**
 * Structure for storing RLS covariance matrix elements
 * We estimate 3 parameters: Ld, Lq, flux_linkage
 * The 3x3 covariance matrix is stored as individual elements for efficiency
 */
struct RLSCovariance_s {
    // Upper triangular + diagonal (symmetric matrix)
    float P11, P12, P13;
    float      P22, P23;
    float           P33;
};

/**
 * @class MotorParameterEstimator
 * @brief Online recursive least squares (RLS) estimator for PMSM motor parameters
 *
 * This class implements an online RLS estimator to identify the motor parameters
 * (Ld, Lq, flux_linkage) based on the PMSM voltage equations:
 *
 * Vd = Rs * id + Ld * (did/dt) - ωe * Lq * iq
 * Vq = Rs * iq + Lq * (diq/dt) + ωe * Ld * id + ωe * λm
 *
 * At steady state (or with negligible dynamics):
 * Vd = Rs * id - ωe * Lq * iq
 * Vq = Rs * iq + ωe * Ld * id + ωe * λm
 *
 * The estimator uses these equations to recursively estimate the parameters
 * using incoming voltage and current measurements.
 */
class MotorParameterEstimator {
public:
    /**
     * Constructor
     * @param phase_resistance Known or estimated phase resistance [Ohm]
     */
    MotorParameterEstimator(float phase_resistance = NOT_SET);

    /**
     * Initialize the estimator with initial parameter guesses
     * @param Ld_init Initial d-axis inductance guess [H]
     * @param Lq_init Initial q-axis inductance guess [H]
     * @param flux_init Initial flux linkage guess [Wb]
     */
    void init(float Ld_init = 0.001f, float Lq_init = 0.001f, float flux_init = 0.01f);

    /**
     * Reset the estimator to initial state
     */
    void reset();

    /**
     * Update the parameter estimates with new measurements
     * This should be called in the FOC loop with current measurements
     *
     * @param Vd Applied d-axis voltage [V]
     * @param Vq Applied q-axis voltage [V]
     * @param id Measured d-axis current [A]
     * @param iq Measured q-axis current [A]
     * @param omega_e Electrical angular velocity [rad/s]
     * @return true if update was performed, false if skipped (insufficient excitation)
     */
    bool update(float Vd, float Vq, float id, float iq, float omega_e);

    /**
     * Get the current parameter estimates
     * @return Structure containing Ld, Lq, flux_linkage, Rs
     */
    MotorParameters_s getParameters();

    /**
     * Get the estimated d-axis inductance
     * @return Ld in Henries
     */
    float getLd();

    /**
     * Get the estimated q-axis inductance
     * @return Lq in Henries
     */
    float getLq();

    /**
     * Get the estimated flux linkage
     * @return Flux linkage in Webers
     */
    float getFluxLinkage();

    /**
     * Check if the estimator has converged
     * @return true if estimates are considered stable
     */
    bool isConverged();

    /**
     * Get the estimation error (RMS of prediction errors)
     * @return RMS prediction error
     */
    float getEstimationError();

    // Configuration parameters
    float forgetting_factor;    //!< RLS forgetting factor (0.95-0.999)
    float min_velocity;         //!< Minimum velocity for estimation [rad/s electrical]
    float min_current;          //!< Minimum current magnitude for estimation [A]
    float Rs;                   //!< Phase resistance [Ohm]

    // Enable/disable estimation
    bool enabled;               //!< Enable/disable the estimator

protected:
    // Parameter estimates
    float Ld_est;               //!< Estimated d-axis inductance
    float Lq_est;               //!< Estimated q-axis inductance
    float flux_est;             //!< Estimated flux linkage

    // Low-pass filters for smoothing estimates
    LowPassFilter LPF_Ld;       //!< Filter for Ld estimate
    LowPassFilter LPF_Lq;       //!< Filter for Lq estimate
    LowPassFilter LPF_flux;     //!< Filter for flux estimate

    // RLS covariance matrix (3x3 symmetric)
    RLSCovariance_s P;

    // Convergence tracking
    float error_sum;            //!< Sum of squared prediction errors
    unsigned long update_count; //!< Number of updates performed
    float initial_covariance;   //!< Initial covariance value

    // Previous values for derivative estimation
    float id_prev;
    float iq_prev;
    unsigned long timestamp_prev;
    bool first_update;

    /**
     * Internal RLS update for Ld and flux_linkage from q-axis equation
     * Vq - Rs*iq = ωe*Ld*id + ωe*λm
     */
    void updateFromQAxis(float Vq, float id, float iq, float omega_e);

    /**
     * Internal RLS update for Lq from d-axis equation
     * Vd - Rs*id = -ωe*Lq*iq
     */
    void updateFromDAxis(float Vd, float id, float iq, float omega_e);
};

#endif // MOTOR_PARAMETER_ESTIMATOR_H
