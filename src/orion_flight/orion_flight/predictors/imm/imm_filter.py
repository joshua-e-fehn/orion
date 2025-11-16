"""Interacting Multiple Model (IMM) filter implementation."""

import numpy as np
from typing import List, Tuple, Dict

from ..cv.cv_model import CVModel
from ..ca.ca_model import CAModel
from ..common.types import PredictorOutput, ensure_covariance_valid


class IMMFilter:
    """
    Interacting Multiple Model (IMM) filter combining CV and CA models.
    
    The IMM filter adaptively blends predictions from multiple models
    (Constant Velocity and Constant Acceleration) based on their likelihood
    given recent measurements.
    
    Key components:
    1. Model transition probability matrix
    2. Model mixing
    3. Parallel model filtering
    4. Likelihood computation
    5. Mode probability update
    6. Estimate combination
    """
    
    def __init__(
        self,
        cv_model: CVModel,
        ca_model: CAModel,
        transition_matrix: np.ndarray = None,
        initial_mode_probabilities: np.ndarray = None
    ):
        """
        Initialize IMM filter.
        
        Args:
            cv_model: Constant Velocity model
            ca_model: Constant Acceleration model (will be set to IMM mode automatically)
            transition_matrix: 2x2 model transition probability matrix
                              [[P(CV->CV), P(CV->CA)],
                               [P(CA->CV), P(CA->CA)]]
                              Default: 95% stay, 5% switch
            initial_mode_probabilities: Initial mode probabilities [P(CV), P(CA)]
                                       Default: [0.5, 0.5]
        """
        self.cv_model = cv_model
        self.ca_model = ca_model
        
        # CRITICAL: Ensure CA model is in IMM mode (6D measurements for fair likelihood comparison)
        if hasattr(ca_model, 'use_acceleration_measurements'):
            if ca_model.use_acceleration_measurements:
                raise ValueError(
                    "CA model must be initialized with use_acceleration_measurements=False for IMM. "
                    "This ensures 6D innovations (pos+vel) for fair likelihood comparison with CV model."
                )
        
        self.models = [cv_model, ca_model]
        self.n_models = 2
        
        # Model transition probability matrix
        if transition_matrix is None:
            # Default: 95% probability of staying in current mode
            self.T = np.array([
                [0.95, 0.05],  # From CV: 95% stay CV, 5% switch to CA
                [0.05, 0.95]   # From CA: 5% switch to CV, 95% stay CA
            ])
        else:
            self.T = transition_matrix
            # Validate transition matrix
            assert self.T.shape == (2, 2), "Transition matrix must be 2x2"
            assert np.allclose(self.T.sum(axis=1), 1.0), "Transition matrix rows must sum to 1"
        
        # Mode probabilities
        if initial_mode_probabilities is None:
            self.mu = np.array([0.5, 0.5])  # Equal initial probabilities
        else:
            self.mu = initial_mode_probabilities
            assert len(self.mu) == 2, "Must have 2 mode probabilities"
            assert np.isclose(self.mu.sum(), 1.0), "Mode probabilities must sum to 1"
        
        # Mixing probabilities (computed during mixing step)
        self.mixing_probs = np.zeros((2, 2))
        
        # Likelihood of each model
        self.likelihoods = np.ones(2)
        
        self.initialized = False
    
    def _compute_mixing_probabilities(self):
        """
        Compute mixing probabilities for interaction.
        
        Mixing probability μ_{i|j} = probability that mode i was active at k-1
        given that mode j is active at k.
        
        μ_{i|j} = (T_{ij} * μ_i) / c_j
        where c_j = Σ_i (T_{ij} * μ_i) is normalization constant
        """
        # Predicted mode probabilities (before measurement)
        c = np.zeros(self.n_models)
        for j in range(self.n_models):
            c[j] = sum(self.T[i, j] * self.mu[i] for i in range(self.n_models))
        
        # Mixing probabilities
        for i in range(self.n_models):
            for j in range(self.n_models):
                if c[j] > 1e-10:
                    self.mixing_probs[i, j] = (self.T[i, j] * self.mu[i]) / c[j]
                else:
                    self.mixing_probs[i, j] = 1.0 / self.n_models
        
        return c
    
    def _mix_states(self):
        """
        Mix state estimates from different models before prediction.
        
        This step creates mixed initial conditions for each model filter
        based on mixing probabilities.
        
        For simplicity in this implementation, we don't actually mix the
        internal states but let each model maintain its own estimate.
        Full IMM would mix here, but for prediction-oriented use, we can
        skip this and just maintain separate filters.
        """
        # In a full IMM implementation, we would:
        # 1. Compute mixed state: x̂^{0j}(k-1|k-1) = Σ_i μ_{i|j} * x̂^i(k-1|k-1)
        # 2. Compute mixed covariance with spread term
        # 
        # For our prediction-oriented approach, we skip mixing and maintain
        # separate CV and CA state estimates, which is simpler and adequate
        # when we already have good measurements.
        pass
    
    def _compute_likelihoods(self, innovations: List[np.ndarray], 
                            innovation_covariances: List[np.ndarray]) -> np.ndarray:
        """
        Compute likelihood of each model given the measurement innovation.
        
        Likelihood L_j = N(y_j; 0, S_j) where:
        - y_j is innovation (measurement residual) for model j
        - S_j is innovation covariance for model j
        
        CRITICAL: All innovations must have the same dimension for fair comparison!
        
        Args:
            innovations: List of innovation vectors [y_cv, y_ca] - MUST be same dimension!
            innovation_covariances: List of innovation covariance matrices [S_cv, S_ca]
        
        Returns:
            likelihoods: Array of likelihoods [L_cv, L_ca]
        """
        # Verify dimensions match
        assert len(innovations[0]) == len(innovations[1]), \
            f"Innovation dimension mismatch: CV={len(innovations[0])}, CA={len(innovations[1])}"
        
        likelihoods = np.zeros(self.n_models)
        
        for j in range(self.n_models):
            y = innovations[j]
            S = innovation_covariances[j]
            
            # Ensure S is valid
            S = ensure_covariance_valid(S)
            
            # Compute Gaussian likelihood
            # L = (1/sqrt((2π)^n * |S|)) * exp(-0.5 * y^T * S^{-1} * y)
            try:
                # For numerical stability, use log-likelihood
                n = len(y)
                sign, logdet = np.linalg.slogdet(S)
                if sign <= 0:
                    # Invalid covariance
                    likelihoods[j] = 1e-10
                    continue
                
                S_inv = np.linalg.inv(S)
                mahalanobis = y.T @ S_inv @ y
                
                # Log-likelihood
                log_likelihood = -0.5 * (n * np.log(2 * np.pi) + logdet + mahalanobis)
                
                # Convert back to likelihood
                likelihoods[j] = np.exp(log_likelihood)
                
                # Prevent numerical underflow
                if likelihoods[j] < 1e-100:
                    likelihoods[j] = 1e-100
                    
            except (np.linalg.LinAlgError, ValueError):
                # If computation fails, use small likelihood
                likelihoods[j] = 1e-10
        
        return likelihoods
    
    def _update_mode_probabilities(self, c: np.ndarray, likelihoods: np.ndarray):
        """
        Update mode probabilities based on likelihoods.
        
        μ_j(k) = (L_j * c_j) / Λ
        where Λ = Σ_j (L_j * c_j) is normalization constant
        
        Args:
            c: Predicted mode probabilities
            likelihoods: Model likelihoods
        """
        # Compute unnormalized probabilities
        unnormalized = likelihoods * c
        
        # Normalize
        total = unnormalized.sum()
        if total > 1e-10:
            self.mu = unnormalized / total
        else:
            # If all likelihoods are near zero, keep previous probabilities
            # or reset to uniform
            self.mu = np.ones(self.n_models) / self.n_models
        
        # Ensure probabilities are valid
        self.mu = np.clip(self.mu, 1e-10, 1.0)
        self.mu = self.mu / self.mu.sum()
    
    def update(self, position: np.ndarray, velocity: np.ndarray, dt: float,
               acceleration: np.ndarray = None):
        """
        Update IMM filter with new measurement.
        
        Steps:
        1. Compute mixing probabilities
        2. Mix state estimates (skipped in simplified version)
        3. Update each model filter
        4. Compute likelihoods
        5. Update mode probabilities
        
        Args:
            position: Measured position [x, y, z]
            velocity: Measured velocity [vx, vy, vz]
            dt: Time since last update (seconds)
            acceleration: Measured acceleration [ax, ay, az] (optional, used for CA initialization only)
        
        Note:
            Both models use 6D measurements (position + velocity) to ensure
            innovation dimensions match for fair likelihood comparison.
        """
        if not self.initialized:
            # Initialize both models
            self.cv_model.initialize(position, velocity)
            self.ca_model.initialize(position, velocity, acceleration)
            self.initialized = True
            return
        
        # Validate dt
        if dt < 1e-6:
            dt = 1e-6  # Prevent numerical issues
        
        # Step 1: Compute mixing probabilities
        c = self._compute_mixing_probabilities()
        
        # Step 2: Mix states (skipped in simplified implementation)
        self._mix_states()
        
        # Step 3: Update each model filter
        # CRITICAL: Both models MUST produce same-dimension innovations (6D: pos+vel)
        # Store innovations and covariances before update
        innovations = []
        innovation_covariances = []
        
        # Update CV model (6D innovation: pos + vel)
        self.cv_model.update(position, velocity, dt)
        cv_innov = self.cv_model.get_innovation()
        cv_S = self.cv_model.R  # 6x6 innovation covariance
        innovations.append(cv_innov)
        innovation_covariances.append(cv_S)
        
        # Update CA model (6D innovation: pos + vel ONLY - acceleration not measured)
        # Note: CA model still estimates acceleration internally, but measurement
        # vector only includes position and velocity for fair likelihood comparison
        self.ca_model.update(position, velocity, dt, acceleration)
        ca_innov = self.ca_model.get_innovation()
        ca_S = self.ca_model.R  # 6x6 innovation covariance (pos+vel only)
        innovations.append(ca_innov)
        innovation_covariances.append(ca_S)
        
        # Verify innovation dimensions match (both should be 6D)
        assert len(cv_innov) == len(ca_innov) == 6, \
            f"Innovation dimension mismatch: CV={len(cv_innov)}, CA={len(ca_innov)}, expected 6"
        assert cv_S.shape == ca_S.shape == (6, 6), \
            f"Innovation covariance shape mismatch: CV={cv_S.shape}, CA={ca_S.shape}, expected (6,6)"
        
        # Step 4: Compute likelihoods (now valid since innovations are same dimension)
        self.likelihoods = self._compute_likelihoods(innovations, innovation_covariances)
        
        # Step 5: Update mode probabilities
        self._update_mode_probabilities(c, self.likelihoods)
    
    def predict(self, horizon: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Predict fused state at future time using IMM combination.
        
        Args:
            horizon: Prediction time horizon (seconds)
        
        Returns:
            predicted_position: Fused [x, y, z]
            predicted_velocity: Fused [vx, vy, vz]
            predicted_acceleration: Fused [ax, ay, az]
            full_covariance: Fused 6x6 covariance matrix for [pos, vel] with cross-correlations
        """
        # Get predictions from each model
        cv_pos, cv_vel, cv_cov = self.cv_model.predict(horizon)
        ca_pos, ca_vel, ca_acc, ca_cov = self.ca_model.predict(horizon)
        
        # Fuse predictions using mode probabilities
        # x̂ = Σ_j μ_j * x̂_j
        fused_pos = self.mu[0] * cv_pos + self.mu[1] * ca_pos
        fused_vel = self.mu[0] * cv_vel + self.mu[1] * ca_vel
        fused_acc = self.mu[1] * ca_acc  # Only CA has acceleration
        
        # Fuse full 6x6 covariances with spread term
        # P = Σ_j μ_j * (P_j + (x̂_j - x̂)(x̂_j - x̂)^T)
        # This includes position-velocity cross-correlations!
        fused_cov = np.zeros((6, 6))
        
        # Build full state vectors for spread computation
        cv_state = np.concatenate([cv_pos, cv_vel])
        ca_state = np.concatenate([ca_pos, ca_vel])
        fused_state = np.concatenate([fused_pos, fused_vel])
        
        for j, (state_j, cov_j) in enumerate([
            (cv_state, cv_cov),
            (ca_state, ca_cov)
        ]):
            # Compute spread term: (x̂_j - x̂)(x̂_j - x̂)^T
            state_diff = state_j - fused_state
            spread = np.outer(state_diff, state_diff)
            
            # Fuse: P += μ_j * (P_j + spread)
            fused_cov += self.mu[j] * (cov_j + spread)
        
        # Ensure covariance is valid (symmetric, positive semi-definite)
        fused_cov = ensure_covariance_valid(fused_cov)
        
        return fused_pos, fused_vel, fused_acc, fused_cov
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get current fused state estimate (prediction with horizon = 0).
        
        Returns:
            position: Fused [x, y, z]
            velocity: Fused [vx, vy, vz]
            acceleration: Fused [ax, ay, az]
            full_covariance: Fused 6x6 covariance matrix for [pos, vel] with cross-correlations
        """
        return self.predict(0.0)
    
    def get_mode_probabilities(self) -> np.ndarray:
        """
        Get current mode probabilities.
        
        Returns:
            Array [P(CV), P(CA)]
        """
        return self.mu.copy()
    
    def get_model_predictions(self, horizon: float) -> Dict:
        """
        Get individual model predictions (for diagnostics/visualization).
        
        Args:
            horizon: Prediction time horizon (seconds)
        
        Returns:
            Dictionary with 'cv' and 'ca' predictions
        """
        cv_pos, cv_vel, cv_cov = self.cv_model.predict(horizon)
        ca_pos, ca_vel, ca_acc, ca_cov = self.ca_model.predict(horizon)
        
        # Extract diagonal blocks for backward compatibility with visualization
        cv_pos_cov = cv_cov[0:3, 0:3]
        cv_vel_cov = cv_cov[3:6, 3:6]
        ca_pos_cov = ca_cov[0:3, 0:3]
        ca_vel_cov = ca_cov[3:6, 3:6]
        
        return {
            'cv': {
                'position': cv_pos,
                'velocity': cv_vel,
                'acceleration': np.zeros(3),
                'position_covariance': cv_pos_cov,
                'velocity_covariance': cv_vel_cov,
                'full_covariance': cv_cov,  # 6x6 with cross-correlations
                'probability': self.mu[0]
            },
            'ca': {
                'position': ca_pos,
                'velocity': ca_vel,
                'acceleration': ca_acc,
                'position_covariance': ca_pos_cov,
                'velocity_covariance': ca_vel_cov,
                'full_covariance': ca_cov,  # 6x6 with cross-correlations
                'probability': self.mu[1]
            }
        }
    
    def reset(self):
        """Reset IMM filter to initial state."""
        self.cv_model.reset()
        self.ca_model.reset()
        self.mu = np.array([0.5, 0.5])
        self.mixing_probs = np.zeros((2, 2))
        self.likelihoods = np.ones(2)
        self.initialized = False
