"""Visualization utilities for target predictions."""

from typing import List, Tuple
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from .types import PredictorOutput, ned_to_enu, rotate_covariance_ned_to_enu


def create_prediction_markers(
    predictions: List[Tuple[float, PredictorOutput]],
    frame_id: str = 'map',
    namespace: str = 'predictor',
    scale: float = 1.0
) -> MarkerArray:
    """
    Create RViz markers for predicted trajectory.
    
    Args:
        predictions: List of (horizon, prediction_output) tuples
        frame_id: ROS frame ID for markers
        namespace: Marker namespace
        scale: Scaling factor for marker sizes
    
    Returns:
        MarkerArray with trajectory path and uncertainty ellipsoids
    """
    markers = MarkerArray()
    marker_id = 0
    
    # Sort predictions by horizon
    predictions_sorted = sorted(predictions, key=lambda x: x[0])
    
    # Create path line strip
    path_marker = Marker()
    path_marker.header.frame_id = frame_id
    path_marker.header.stamp.sec = 0
    path_marker.header.stamp.nanosec = 0
    path_marker.ns = f"{namespace}_path"
    path_marker.id = marker_id
    marker_id += 1
    path_marker.type = Marker.LINE_STRIP
    path_marker.action = Marker.ADD
    path_marker.scale.x = 0.05 * scale  # Line width
    path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green
    path_marker.pose.orientation.w = 1.0
    
    for horizon, pred in predictions_sorted:
        if not pred.is_valid:
            continue
        
        # Convert NED to ENU for visualization
        pos_enu = ned_to_enu(pred.predicted_position)
        
        point = Point()
        point.x = float(pos_enu[0])
        point.y = float(pos_enu[1])
        point.z = float(pos_enu[2])
        path_marker.points.append(point)
    
    markers.markers.append(path_marker)
    
    # Create uncertainty ellipsoids at each prediction point
    for horizon, pred in predictions_sorted:
        if not pred.is_valid:
            continue
        
        # Convert position to ENU
        pos_enu = ned_to_enu(pred.predicted_position)
        
        # Convert covariance to ENU
        cov_enu = rotate_covariance_ned_to_enu(pred.position_covariance)
        
        # Create ellipsoid marker
        ellipsoid = create_uncertainty_ellipsoid(
            position=pos_enu,
            covariance=cov_enu,
            frame_id=frame_id,
            namespace=f"{namespace}_uncertainty",
            marker_id=marker_id,
            scale=scale,
            horizon=horizon
        )
        marker_id += 1
        markers.markers.append(ellipsoid)
        
        # Create position sphere
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp.sec = 0
        sphere.header.stamp.nanosec = 0
        sphere.ns = f"{namespace}_points"
        sphere.id = marker_id
        marker_id += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(pos_enu[0])
        sphere.pose.position.y = float(pos_enu[1])
        sphere.pose.position.z = float(pos_enu[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.2 * scale
        sphere.scale.y = 0.2 * scale
        sphere.scale.z = 0.2 * scale
        
        # Color based on horizon (green -> yellow -> red)
        max_horizon = predictions_sorted[-1][0] if predictions_sorted else 1.0
        t = horizon / max_horizon
        sphere.color = ColorRGBA(
            r=float(t),
            g=float(1.0 - t * 0.5),
            b=0.0,
            a=0.9
        )
        
        markers.markers.append(sphere)
        
        # Create velocity arrow (always display)
        vel_enu = ned_to_enu(pred.predicted_velocity)
        vel_norm = np.linalg.norm(vel_enu)
        
        arrow = Marker()
        arrow.header.frame_id = frame_id
        arrow.header.stamp.sec = 0
        arrow.header.stamp.nanosec = 0
        arrow.ns = f"{namespace}_velocity"
        arrow.id = marker_id
        marker_id += 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        
        # Arrow from current position pointing in velocity direction
        start_point = Point()
        start_point.x = float(pos_enu[0])
        start_point.y = float(pos_enu[1])
        start_point.z = float(pos_enu[2])
        
        # Scale arrow length by velocity magnitude (but cap it)
        # Use minimum length for very small velocities
        if vel_norm > 0.01:
            arrow_length = min(vel_norm * 0.5, 2.0) * scale
            vel_unit = vel_enu / vel_norm
        else:
            # Default to small arrow pointing in X direction if velocity is zero
            arrow_length = 0.3 * scale
            vel_unit = np.array([1.0, 0.0, 0.0])
        
        end_point = Point()
        end_point.x = float(pos_enu[0] + vel_unit[0] * arrow_length)
        end_point.y = float(pos_enu[1] + vel_unit[1] * arrow_length)
        end_point.z = float(pos_enu[2] + vel_unit[2] * arrow_length)
        
        arrow.points.append(start_point)
        arrow.points.append(end_point)
        
        # Arrow shaft and head dimensions (smaller arrows)
        arrow.scale.x = 0.05 * scale  # Shaft diameter (reduced from 0.1)
        arrow.scale.y = 0.10 * scale  # Head diameter (reduced from 0.2)
        arrow.scale.z = 0.15 * scale  # Head length (reduced from 0.3)
        
        # Color based on velocity magnitude (blue -> cyan -> white)
        vel_color_factor = min(vel_norm / 5.0, 1.0)  # Normalize to max 5 m/s
        arrow.color = ColorRGBA(
            r=float(vel_color_factor),
            g=float(0.5 + vel_color_factor * 0.5),
            b=1.0,
            a=0.9
        )
        
        markers.markers.append(arrow)
        
        # Create text label with horizon
        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp.sec = 0
        text.header.stamp.nanosec = 0
        text.ns = f"{namespace}_labels"
        text.id = marker_id
        marker_id += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(pos_enu[0])
        text.pose.position.y = float(pos_enu[1])
        text.pose.position.z = float(pos_enu[2] + 0.5)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.3 * scale
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f"{horizon:.1f}s"
        
        markers.markers.append(text)
    
    return markers


def create_uncertainty_ellipsoid(
    position: np.ndarray,
    covariance: np.ndarray,
    frame_id: str,
    namespace: str,
    marker_id: int,
    scale: float = 1.0,
    horizon: float = 0.0,
    sigma_level: float = 2.0
) -> Marker:
    """
    Create an ellipsoid marker representing uncertainty.
    
    Args:
        position: Center position [x, y, z] in ENU
        covariance: 3x3 covariance matrix
        frame_id: ROS frame ID
        namespace: Marker namespace
        marker_id: Unique marker ID
        scale: Scaling factor
        horizon: Prediction horizon (for color coding)
        sigma_level: Sigma level for ellipsoid (1=68%, 2=95%, 3=99.7%)
    
    Returns:
        Marker representing uncertainty ellipsoid
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp.sec = 0
    marker.header.stamp.nanosec = 0
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    
    # Position
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    
    # Compute eigenvalues and eigenvectors for orientation and scale
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    eigenvalues = np.maximum(eigenvalues, 1e-6)  # Prevent negative/zero
    
    # Scale by sigma level
    scales = sigma_level * np.sqrt(eigenvalues)
    
    # Set scale (semi-axes of ellipsoid)
    marker.scale.x = float(scales[0] * scale)
    marker.scale.y = float(scales[1] * scale)
    marker.scale.z = float(scales[2] * scale)
    
    # Set orientation from eigenvectors
    # Convert rotation matrix to quaternion
    quat = rotation_matrix_to_quaternion(eigenvectors)
    marker.pose.orientation.x = float(quat[0])
    marker.pose.orientation.y = float(quat[1])
    marker.pose.orientation.z = float(quat[2])
    marker.pose.orientation.w = float(quat[3])
    
    # Color based on uncertainty magnitude (trace of covariance)
    uncertainty_magnitude = np.trace(covariance)
    
    if uncertainty_magnitude < 1.0:
        # Low uncertainty - green
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)
    elif uncertainty_magnitude < 10.0:
        # Medium uncertainty - yellow
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.3)
    else:
        # High uncertainty - red
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.3)
    
    return marker


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """
    Convert 3x3 rotation matrix to quaternion [x, y, z, w].
    
    Args:
        R: 3x3 rotation matrix
    
    Returns:
        Quaternion [x, y, z, w]
    """
    trace = np.trace(R)
    
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    
    return np.array([x, y, z, w])


def delete_all_markers(namespace: str, frame_id: str = 'map') -> MarkerArray:
    """
    Create a MarkerArray that deletes all markers in a namespace.
    
    Args:
        namespace: Marker namespace to delete
        frame_id: Frame ID for markers
    
    Returns:
        MarkerArray with delete action
    """
    markers = MarkerArray()
    
    delete_marker = Marker()
    delete_marker.header.frame_id = frame_id
    delete_marker.ns = namespace
    delete_marker.action = Marker.DELETEALL
    
    markers.markers.append(delete_marker)
    
    return markers
