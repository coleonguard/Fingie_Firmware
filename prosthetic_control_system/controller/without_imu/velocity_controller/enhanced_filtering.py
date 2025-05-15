#!/usr/bin/env python3
"""
Enhanced filtering utilities for velocity-based proximity controller.

This module provides enhanced filtering functions for smoothing sensor readings
and reducing jerkiness in the velocity-based controller.
"""

def apply_enhanced_distance_filtering(raw_distance, finger, controller):
    """
    Apply enhanced filtering to distance measurements.
    
    This filtering uses a combination of:
    1. History-based median filtering to reject outliers
    2. Heavy exponential filtering for smooth transitions
    3. Consecutive similar readings check to reduce noise-based velocity changes
    
    Args:
        raw_distance: Raw distance reading from sensor
        finger: Finger name (for state tracking)
        controller: Controller instance with filtering state
        
    Returns:
        Filtered distance value
    """
    # Add to distance history
    if finger in controller.last_distance_readings:
        controller.last_distance_readings[finger].append(raw_distance)
        # Keep history limited to specified size
        if len(controller.last_distance_readings[finger]) > controller.distance_history_size:
            controller.last_distance_readings[finger] = controller.last_distance_readings[finger][-controller.distance_history_size:]
    
    # Apply median filter first to reject outliers
    median_distance = raw_distance
    if finger in controller.last_distance_readings and len(controller.last_distance_readings[finger]) >= 3:
        sorted_distances = sorted(controller.last_distance_readings[finger])
        median_distance = sorted_distances[len(sorted_distances) // 2]
    
    # Apply heavy exponential filtering for smooth transition
    if finger in controller.filtered_distances:
        # Very strong filtering (small alpha = stronger filtering)
        controller.filtered_distances[finger] = ((1.0 - controller.distance_filter_alpha) * 
                                          controller.filtered_distances[finger] + 
                                          controller.distance_filter_alpha * median_distance)
        filtered_distance = controller.filtered_distances[finger]
    else:
        # Initialize if not present
        controller.filtered_distances[finger] = median_distance
        filtered_distance = median_distance
        
    # Check for consecutive similar readings to reduce noise-based velocity changes
    if finger in controller.consecutive_readings:
        # If reading is similar to filtered value (within 10%)
        if abs(raw_distance - filtered_distance) < (filtered_distance * 0.1):
            controller.consecutive_readings[finger] += 1
        else:
            # Reset counter for large changes
            controller.consecutive_readings[finger] = 0
            
    return filtered_distance

def get_filtered_sensor_reading(finger, primary_sensor, controller):
    """
    Get the best available and filtered proximity reading for a finger.
    
    Args:
        finger: Finger name (e.g., "Thumb")
        primary_sensor: Primary sensor for this finger (e.g., "T1")
        controller: Controller instance
        
    Returns:
        Tuple of (filtered_distance, sensor_name) with best available reading
    """
    # Get status of primary sensor
    primary_status = controller.status.get(primary_sensor, "BAD")
    raw_distance = None
    sensor_used = None
    
    # If primary sensor is ok, use it
    if primary_status == "OK" and primary_sensor in controller.filtered_values:
        raw_distance = controller.filtered_values[primary_sensor]
        sensor_used = primary_sensor
    
    # Otherwise use fallbacks
    else:
        # Find best available alternative - same finger first
        for sensor_name in controller.filtered_values:
            # Check if this sensor starts with the same letter (same finger)
            if sensor_name.startswith(primary_sensor[0]) and sensor_name != primary_sensor:
                raw_distance = controller.filtered_values[sensor_name]
                sensor_used = sensor_name
                break
        
        # If no same-finger sensor, try neighbors
        if raw_distance is None and primary_sensor in controller.FALLBACK:
            for nb in controller.FALLBACK[primary_sensor]:
                if nb in controller.filtered_values:
                    raw_distance = controller.filtered_values[nb]
                    sensor_used = nb
                    break
    
    # If no valid reading found, use default value
    if raw_distance is None:
        raw_distance = 100.0  # 100mm = far away
        sensor_used = None
        
    # Apply enhanced filtering
    filtered_distance = apply_enhanced_distance_filtering(raw_distance, finger, controller)
            
    return filtered_distance, sensor_used