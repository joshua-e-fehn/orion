"""Abstract base class for all target predictors."""

from abc import ABC, abstractmethod
from typing import List
import numpy as np

from .types import PredictorInput, PredictorOutput


class PredictorBase(ABC):
    """
    Abstract base class that all target predictors must implement.
    
    This defines the common interface for CV, CA, IMM, and future predictors.
    """
    
    def __init__(self, predictor_type: str):
        """
        Initialize base predictor.
        
        Args:
            predictor_type: String identifier for this predictor (e.g., 'cv', 'ca', 'imm')
        """
        self.predictor_type = predictor_type
        self.initialized = False
        self.last_update_time = None
    
    @abstractmethod
    def update(self, measurement: PredictorInput) -> None:
        """
        Update internal state estimate with new measurement.
        
        This is the "correction" step in Kalman filtering terminology.
        
        Args:
            measurement: New target measurement data
        """
        pass
    
    @abstractmethod
    def predict(self, horizon: float) -> PredictorOutput:
        """
        Predict target state at future time.
        
        This is the "prediction" step in Kalman filtering terminology.
        
        Args:
            horizon: Time horizon for prediction (seconds from current time)
        
        Returns:
            PredictorOutput with predicted state and uncertainty
        """
        pass
    
    @abstractmethod
    def predict_multiple(self, horizons: List[float]) -> List[PredictorOutput]:
        """
        Predict target state at multiple future times.
        
        Args:
            horizons: List of time horizons (seconds from current time)
        
        Returns:
            List of PredictorOutput, one for each horizon
        """
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset predictor to initial uninitialized state."""
        pass
    
    @abstractmethod
    def get_state_estimate(self) -> PredictorOutput:
        """
        Get current state estimate (prediction with horizon = 0).
        
        Returns:
            PredictorOutput with current filtered state
        """
        pass
    
    def is_initialized(self) -> bool:
        """Check if predictor has been initialized with at least one measurement."""
        return self.initialized
    
    def get_predictor_type(self) -> str:
        """Get the type identifier of this predictor."""
        return self.predictor_type
    
    def _calculate_dt(self, current_time: float) -> float:
        """
        Calculate time delta since last update.
        
        Args:
            current_time: Current timestamp (seconds)
        
        Returns:
            Time delta in seconds, or default value if first update
        """
        if self.last_update_time is None:
            return 0.1  # Default dt for first update
        
        dt = current_time - self.last_update_time
        
        # Sanity check
        if dt < 0:
            raise ValueError(f"Negative dt: {dt} (time went backwards?)")
        if dt > 1.0:
            # Large gap - may want to reset or use max dt
            return 1.0
        if dt < 1e-6:
            # Too small - avoid numerical issues
            return 1e-6
        
        return dt
