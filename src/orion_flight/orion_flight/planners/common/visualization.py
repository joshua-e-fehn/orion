"""Visualization utilities for planner framework."""

import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from .utils import ned_to_enu


def create_guidance_markers(interceptor_pos: np.ndarray,
                            target_pos: np.ndarray,
                            acceleration: np.ndarray,
                            frame_id: str = 'map',
                            namespace: str = 'planner',
                            scale_accel: float = 0.5) -> MarkerArray:
    """
    Create visualization markers for guidance.
    
    Args:
        interceptor_pos: Interceptor position in NED
        target_pos: Target position in NED
        acceleration: Commanded acceleration in NED
        frame_id: RViz frame ID
        namespace: Marker namespace
        scale_accel: Scale factor for acceleration arrow visualization
    
    Returns:
        MarkerArray with guidance visualization
    """
    markers = MarkerArray()
    
    # Convert to ENU for RViz
    interceptor_enu = ned_to_enu(interceptor_pos)
    target_enu = ned_to_enu(target_pos)
    
    # Marker 0: Line-of-sight line (interceptor to target)
    los_marker = Marker()
    los_marker.header.frame_id = frame_id
    los_marker.header.stamp.sec = 0
    los_marker.header.stamp.nanosec = 0
    los_marker.ns = namespace
    los_marker.id = 0
    los_marker.type = Marker.LINE_STRIP
    los_marker.action = Marker.ADD
    
    los_marker.points = [
        Point(x=interceptor_enu[0], y=interceptor_enu[1], z=interceptor_enu[2]),
        Point(x=target_enu[0], y=target_enu[1], z=target_enu[2])
    ]
    
    los_marker.scale.x = 0.02  # Line width
    los_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)  # Green
    
    markers.markers.append(los_marker)
    
    # Marker 1: Commanded acceleration arrow
    accel_enu = ned_to_enu(acceleration)
    accel_end = interceptor_enu + accel_enu * scale_accel
    
    accel_marker = Marker()
    accel_marker.header.frame_id = frame_id
    accel_marker.header.stamp.sec = 0
    accel_marker.header.stamp.nanosec = 0
    accel_marker.ns = namespace
    accel_marker.id = 1
    accel_marker.type = Marker.ARROW
    accel_marker.action = Marker.ADD
    
    accel_marker.points = [
        Point(x=interceptor_enu[0], y=interceptor_enu[1], z=interceptor_enu[2]),
        Point(x=accel_end[0], y=accel_end[1], z=accel_end[2])
    ]
    
    accel_marker.scale = Vector3(x=0.05, y=0.1, z=0.1)  # Shaft/head diameter
    accel_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)  # Red
    
    markers.markers.append(accel_marker)
    
    # Marker 2: Interceptor position sphere
    interceptor_marker = Marker()
    interceptor_marker.header.frame_id = frame_id
    interceptor_marker.header.stamp.sec = 0
    interceptor_marker.header.stamp.nanosec = 0
    interceptor_marker.ns = namespace
    interceptor_marker.id = 2
    interceptor_marker.type = Marker.SPHERE
    interceptor_marker.action = Marker.ADD
    
    interceptor_marker.pose.position = Point(x=interceptor_enu[0], 
                                             y=interceptor_enu[1], 
                                             z=interceptor_enu[2])
    interceptor_marker.pose.orientation.w = 1.0
    
    interceptor_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
    interceptor_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)  # Blue
    
    markers.markers.append(interceptor_marker)
    
    # Marker 3: Target position sphere
    target_marker = Marker()
    target_marker.header.frame_id = frame_id
    target_marker.header.stamp.sec = 0
    target_marker.header.stamp.nanosec = 0
    target_marker.ns = namespace
    target_marker.id = 3
    target_marker.type = Marker.SPHERE
    target_marker.action = Marker.ADD
    
    target_marker.pose.position = Point(x=target_enu[0], 
                                        y=target_enu[1], 
                                        z=target_enu[2])
    target_marker.pose.orientation.w = 1.0
    
    target_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
    target_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)  # Red
    
    markers.markers.append(target_marker)
    
    return markers


def create_intercept_point_marker(intercept_point: np.ndarray,
                                  frame_id: str = 'map',
                                  namespace: str = 'planner',
                                  marker_id: int = 10) -> Marker:
    """
    Create marker for predicted intercept point.
    
    Args:
        intercept_point: Predicted intercept position in NED
        frame_id: RViz frame ID
        namespace: Marker namespace
        marker_id: Unique marker ID
    
    Returns:
        Marker for intercept point
    """
    intercept_enu = ned_to_enu(intercept_point)
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp.sec = 0
    marker.header.stamp.nanosec = 0
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    
    marker.pose.position = Point(x=intercept_enu[0], 
                                y=intercept_enu[1], 
                                z=intercept_enu[2])
    marker.pose.orientation.w = 1.0
    
    marker.scale = Vector3(x=0.4, y=0.4, z=0.4)
    marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)  # Blue, semi-transparent
    
    return marker


def create_trajectory_marker(positions: list,
                             frame_id: str = 'map',
                             namespace: str = 'planner',
                             marker_id: int = 20,
                             color: tuple = (1.0, 1.0, 0.0, 0.8)) -> Marker:
    """
    Create marker for trajectory path.
    
    Args:
        positions: List of positions in NED
        frame_id: RViz frame ID
        namespace: Marker namespace
        marker_id: Unique marker ID
        color: RGBA color tuple
    
    Returns:
        Marker for trajectory
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp.sec = 0
    marker.header.stamp.nanosec = 0
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    
    # Convert positions to ENU and add to marker
    for pos_ned in positions:
        pos_enu = ned_to_enu(pos_ned)
        marker.points.append(Point(x=pos_enu[0], y=pos_enu[1], z=pos_enu[2]))
    
    marker.scale.x = 0.03  # Line width
    marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
    
    return marker


def create_uncertainty_ellipsoid_marker(position: np.ndarray,
                                       covariance: np.ndarray,
                                       frame_id: str = 'map',
                                       namespace: str = 'planner',
                                       marker_id: int = 30,
                                       n_sigma: float = 2.0) -> Marker:
    """
    Create marker for uncertainty ellipsoid.
    
    Args:
        position: Center position in NED
        covariance: 3x3 position covariance matrix
        frame_id: RViz frame ID
        namespace: Marker namespace
        marker_id: Unique marker ID
        n_sigma: Number of standard deviations for ellipsoid size
    
    Returns:
        Marker for uncertainty ellipsoid
    """
    pos_enu = ned_to_enu(position)
    
    # Compute eigenvalues and eigenvectors of covariance
    eigenvalues, eigenvectors = np.linalg.eig(covariance)
    
    # Sort by eigenvalue magnitude
    idx = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[idx]
    eigenvectors = eigenvectors[:, idx]
    
    # Convert eigenvalues to standard deviations (axis lengths)
    axis_lengths = n_sigma * np.sqrt(np.abs(eigenvalues))
    
    # Convert rotation from NED to ENU
    # This is a simplified conversion; proper rotation transformation needed
    # For now, just use the eigenvalue magnitudes for sphere scaling
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp.sec = 0
    marker.header.stamp.nanosec = 0
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    
    marker.pose.position = Point(x=pos_enu[0], y=pos_enu[1], z=pos_enu[2])
    marker.pose.orientation.w = 1.0  # Simplified: no rotation
    
    # Scale by uncertainty (simplified: use mean of axis lengths)
    mean_scale = float(np.mean(axis_lengths))
    marker.scale = Vector3(x=mean_scale, y=mean_scale, z=mean_scale)
    
    marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.3)  # Cyan, very transparent
    
    return marker


def create_text_marker(position: np.ndarray,
                       text: str,
                       frame_id: str = 'map',
                       namespace: str = 'planner',
                       marker_id: int = 40,
                       height: float = 0.3) -> Marker:
    """
    Create text marker for labeling.
    
    Args:
        position: Text position in NED
        text: Text content
        frame_id: RViz frame ID
        namespace: Marker namespace
        marker_id: Unique marker ID
        height: Text height
    
    Returns:
        Text marker
    """
    pos_enu = ned_to_enu(position)
    
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp.sec = 0
    marker.header.stamp.nanosec = 0
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    
    marker.pose.position = Point(x=pos_enu[0], y=pos_enu[1], z=pos_enu[2] + 0.5)
    marker.pose.orientation.w = 1.0
    
    marker.text = text
    marker.scale.z = height  # Text height
    marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
    
    return marker


def delete_all_markers(namespace: str = 'planner') -> MarkerArray:
    """
    Create marker array to delete all markers in namespace.
    
    Args:
        namespace: Marker namespace to delete
    
    Returns:
        MarkerArray with delete actions
    """
    markers = MarkerArray()
    
    delete_marker = Marker()
    delete_marker.ns = namespace
    delete_marker.action = Marker.DELETEALL
    
    markers.markers.append(delete_marker)
    
    return markers
