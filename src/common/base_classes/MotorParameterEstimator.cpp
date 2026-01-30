#include "MotorParameterEstimator.h"

MotorParameterEstimator::MotorParameterEstimator(float phase_resistance)
    : LPF_Ld(DEF_RLS_ESTIMATE_FILTER_TF)
    , LPF_Lq(DEF_RLS_ESTIMATE_FILTER_TF)
    , LPF_flux(DEF_RLS_ESTIMATE_FILTER_TF)
{
    Rs = phase_resistance;
    forgetting_factor = DEF_RLS_FORGETTING_FACTOR;
    min_velocity = DEF_RLS_MIN_VELOCITY;
    min_current = DEF_RLS_MIN_CURRENT;
    initial_covariance = DEF_RLS_INITIAL_COVARIANCE;
    enabled = true;
    first_update = true;

    // Initialize with default values
    init();
}

void MotorParameterEstimator::init(float Ld_init, float Lq_init, float flux_init) {
    // Set initial parameter estimates
    Ld_est = Ld_init;
    Lq_est = Lq_init;
    flux_est = flux_init;

    // Initialize covariance matrix as diagonal
    // P = initial_covariance * I
    P.P11 = initial_covariance;
    P.P12 = 0.0f;
    P.P13 = 0.0f;
    P.P22 = initial_covariance;
    P.P23 = 0.0f;
    P.P33 = initial_covariance;

    // Reset convergence tracking
    error_sum = 0.0f;
    update_count = 0;

    // Reset previous values
    id_prev = 0.0f;
    iq_prev = 0.0f;
    timestamp_prev = 0;
    first_update = true;
}

void MotorParameterEstimator::reset() {
    init(Ld_est, Lq_est, flux_est);
}

bool MotorParameterEstimator::update(float Vd, float Vq, float id, float iq, float omega_e) {
    if (!enabled) return false;

    // Check if phase resistance is set
    if (!_isset(Rs)) return false;

    // Get absolute values for threshold checks
    float abs_omega = omega_e > 0 ? omega_e : -omega_e;
    float current_magnitude = _sqrt(id * id + iq * iq);

    // Skip update if insufficient excitation
    // We need some velocity to observe the back-EMF and coupling terms
    if (abs_omega < min_velocity) return false;
    if (current_magnitude < min_current) return false;

    // Update from both d-axis and q-axis equations
    updateFromDAxis(Vd, id, iq, omega_e);
    updateFromQAxis(Vq, id, iq, omega_e);

    // Store previous values
    id_prev = id;
    iq_prev = iq;
    timestamp_prev = _micros();
    first_update = false;

    update_count++;
    return true;
}

void MotorParameterEstimator::updateFromQAxis(float Vq, float id, float iq, float omega_e) {
    // Q-axis equation (steady state):
    // Vq = Rs * iq + ωe * Ld * id + ωe * λm
    //
    // Rearranged for RLS:
    // y = Vq - Rs * iq
    // y = [ωe * id, ωe] * [Ld, λm]^T
    // y = φ^T * θ
    //
    // Where:
    // φ = [ωe * id, ωe]^T  (regressor vector)
    // θ = [Ld, λm]^T       (parameters to estimate)

    // Compute output (measurement)
    float y = Vq - Rs * iq;

    // Compute regressor
    float phi1 = omega_e * id;  // coefficient for Ld
    float phi2 = omega_e;       // coefficient for λm

    // Prediction with current estimates
    float y_pred = phi1 * Ld_est + phi2 * flux_est;

    // Prediction error
    float e = y - y_pred;

    // Track estimation error
    error_sum += e * e;

    // RLS update (using only P11, P12, P22 submatrix for Ld and λm)
    // For this 2-parameter problem embedded in our 3-parameter structure:
    // We use P11 for Ld variance, P33 for λm variance, P13 for covariance

    // K = P * φ / (λ + φ^T * P * φ)
    // where λ is the forgetting factor

    // P * φ (using P11, P13 for Ld row and P13, P33 for flux row)
    float Pphi1 = P.P11 * phi1 + P.P13 * phi2;
    float Pphi2 = P.P13 * phi1 + P.P33 * phi2;

    // φ^T * P * φ + λ
    float denom = forgetting_factor + phi1 * Pphi1 + phi2 * Pphi2;

    // Avoid division by zero
    if (denom < 1e-10f) return;

    // Kalman gain K = P * φ / denom
    float K1 = Pphi1 / denom;
    float K2 = Pphi2 / denom;

    // Parameter update: θ = θ + K * e
    float Ld_update = Ld_est + K1 * e;
    float flux_update = flux_est + K2 * e;

    // Constrain estimates to physically reasonable values
    if (Ld_update > 0.0f && Ld_update < 1.0f) {
        Ld_est = LPF_Ld(Ld_update);
    }
    if (flux_update > 0.0f && flux_update < 10.0f) {
        flux_est = LPF_flux(flux_update);
    }

    // Covariance update: P = (P - K * φ^T * P) / λ
    // For P11: P11_new = (P11 - K1 * phi1 * P11 - K1 * phi2 * P13) / λ
    // For P13: P13_new = (P13 - K1 * phi1 * P13 - K1 * phi2 * P33) / λ
    // For P33: P33_new = (P33 - K2 * phi1 * P13 - K2 * phi2 * P33) / λ

    float inv_lambda = 1.0f / forgetting_factor;

    float P11_new = (P.P11 - K1 * (phi1 * P.P11 + phi2 * P.P13)) * inv_lambda;
    float P13_new = (P.P13 - K1 * (phi1 * P.P13 + phi2 * P.P33)) * inv_lambda;
    float P33_new = (P.P33 - K2 * (phi1 * P.P13 + phi2 * P.P33)) * inv_lambda;

    // Update covariance (with bounds to prevent numerical issues)
    float max_cov = initial_covariance * 10.0f;
    float min_cov = 1e-10f;

    P.P11 = _constrain(P11_new, min_cov, max_cov);
    P.P13 = _constrain(P13_new, -max_cov, max_cov);
    P.P33 = _constrain(P33_new, min_cov, max_cov);
}

void MotorParameterEstimator::updateFromDAxis(float Vd, float id, float iq, float omega_e) {
    // D-axis equation (steady state):
    // Vd = Rs * id - ωe * Lq * iq
    //
    // Rearranged for RLS:
    // y = Vd - Rs * id
    // y = [-ωe * iq] * [Lq]
    // y = φ * θ
    //
    // This is a single-parameter estimation for Lq

    // Compute output (measurement)
    float y = Vd - Rs * id;

    // Compute regressor
    float phi = -omega_e * iq;  // coefficient for Lq

    // Skip if regressor is too small
    if (phi * phi < 1e-10f) return;

    // Prediction with current estimate
    float y_pred = phi * Lq_est;

    // Prediction error
    float e = y - y_pred;

    // Track estimation error
    error_sum += e * e;

    // Simple RLS for single parameter using P22
    // K = P22 * φ / (λ + φ * P22 * φ)
    float denom = forgetting_factor + phi * P.P22 * phi;

    if (denom < 1e-10f) return;

    float K = P.P22 * phi / denom;

    // Parameter update: Lq = Lq + K * e
    float Lq_update = Lq_est + K * e;

    // Constrain to physically reasonable values
    if (Lq_update > 0.0f && Lq_update < 1.0f) {
        Lq_est = LPF_Lq(Lq_update);
    }

    // Covariance update: P22 = (P22 - K * φ * P22) / λ
    float P22_new = (P.P22 - K * phi * P.P22) / forgetting_factor;

    float max_cov = initial_covariance * 10.0f;
    float min_cov = 1e-10f;
    P.P22 = _constrain(P22_new, min_cov, max_cov);
}

MotorParameters_s MotorParameterEstimator::getParameters() {
    MotorParameters_s params;
    params.Ld = Ld_est;
    params.Lq = Lq_est;
    params.flux_linkage = flux_est;
    params.Rs = Rs;
    return params;
}

float MotorParameterEstimator::getLd() {
    return Ld_est;
}

float MotorParameterEstimator::getLq() {
    return Lq_est;
}

float MotorParameterEstimator::getFluxLinkage() {
    return flux_est;
}

bool MotorParameterEstimator::isConverged() {
    // Consider converged if:
    // 1. We have enough updates
    // 2. The covariance has reduced significantly from initial value
    if (update_count < 100) return false;

    // Check if diagonal elements of P have reduced
    float cov_threshold = initial_covariance * 0.01f;
    return (P.P11 < cov_threshold && P.P22 < cov_threshold && P.P33 < cov_threshold);
}

float MotorParameterEstimator::getEstimationError() {
    if (update_count == 0) return 0.0f;
    return _sqrt(error_sum / (float)update_count);
}
