"""Abstract base class for all guidance planners."""

from abc import ABC, abstractmethod
from rclpy.node import Node

from .types import PlannerInput, PlannerOutput


class PlannerBase(Node, ABC):
    """
    Abstract base class that all guidance planners must implement.
    
    This defines the common interface for PP, PN, LPN, GPN, FRPN, MPC planners.
    Inherits from rclpy.Node to provide ROS2 functionality.
    """
    
    def __init__(self, node_name: str):
        """
        Initialize base planner.
        
        Args:
            node_name: ROS2 node name
        """
        super().__init__(node_name)
        self.planner_initialized = False
    
    @abstractmethod
    def compute_guidance(self, planner_input: PlannerInput) -> PlannerOutput:
        """
        Compute guidance command based on current state.
        
        This is the core method that implements the guidance law.
        
        Args:
            planner_input: Current interceptor and target states
        
        Returns:
            PlannerOutput with commanded acceleration and derived setpoints
        """
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """
        Reset planner to initial state.
        
        Clears any internal state, history, or integrators.
        """
        pass
    
    @abstractmethod
    def is_converged(self) -> bool:
        """
        Check if interception criteria are met.
        
        Returns:
            True if intercept complete (within threshold distance/time)
        """
        pass
    
    @abstractmethod
    def get_planner_type(self) -> str:
        """
        Get the type identifier of this planner.
        
        Returns:
            String identifier ('pp', 'pn', 'lpn', 'frpn', 'mpc', etc.)
        """
        pass
    
    def is_initialized(self) -> bool:
        """
        Check if planner has been initialized.
        
        Returns:
            True if planner is ready to compute guidance
        """
        return self.planner_initialized
    
    def set_initialized(self, initialized: bool) -> None:
        """
        Set initialization state.
        
        Args:
            initialized: Initialization state
        """
        self.planner_initialized = initialized
