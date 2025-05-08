#!/usr/bin/env python3
"""
Component tests for the finger state machine (C-01, C-02).

Tests:
- C-01: Finger θ monotonic - Check that finger angle increases monotonically with decreasing distance
- C-02: dI/dt contact trigger - Check that a current spike triggers contact state transition
"""

import sys
import os
import unittest

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import components to test
from controller.state_machines import FingerFSM, FingerState

class TestFingerFSM(unittest.TestCase):
    """Test cases for the finger state machine"""
    
    def test_finger_monotonic_position(self):
        """Test that finger position increases monotonically with decreasing distance (C-01)"""
        # Create finger FSM
        fsm = FingerFSM("Test", 
                        approach_threshold=40.0,
                        contact_threshold=5.0,
                        max_position=100.0)
        
        # Test sequence of decreasing distances - but not crossing contact threshold
        # Using 40, 30, 20, 10, 6 to avoid the contact state at 5mm
        distances = [40, 30, 20, 10, 6]
        positions = []
        states = []
        
        # Run the sequence
        for i, distance in enumerate(distances):
            # First reset to ensure consistent starting state
            fsm.reset()
            
            # Run two updates to ensure we get into the right state
            fsm.update(distance, 0.0, 0.0)
            state, position, torque = fsm.update(distance, 0.0, 0.0)
            positions.append(position)
            states.append(state)
        
        # Check positions within PROPORTIONAL state are strictly increasing
        for i in range(1, len(positions)):
            if states[i] == FingerState.PROPORTIONAL and states[i-1] == FingerState.PROPORTIONAL:
                self.assertGreater(positions[i], positions[i-1],
                                f"Position not monotonically increasing: {positions}")
        
        # Check that the first position is 0 (distance > approach_threshold)
        self.assertEqual(positions[0], 0.0,
                       f"First position should be 0 but was {positions[0]}")
        
        # Check that the last position is close to max_position (as distance approaches contact_threshold)
        last_dist = distances[-1]
        expected_position = fsm.max_position * (fsm.approach_threshold - last_dist) / (fsm.approach_threshold - fsm.contact_threshold)
        self.assertAlmostEqual(positions[-1], expected_position, 1, 
                            f"Last position should be ~{expected_position:.1f} but was {positions[-1]:.1f}")
    
    def test_current_derivative_contact_trigger(self):
        """Test that current derivative spike triggers contact state transition (C-02)"""
        # Create finger FSM
        fsm = FingerFSM("Test",
                        approach_threshold=40.0,
                        contact_threshold=5.0,
                        slip_detection_threshold=-0.05)
        
        # First, we need to ensure we start in IDLE state
        fsm.reset()
        
        # Then put FSM into APPROACH state with first update
        distance = 30.0  # Between approach_threshold (40.0) and contact_threshold (5.0)
        fsm.update(distance, 0.0, 0.0)
        
        # Second update should move to PROPORTIONAL state
        state, _, _ = fsm.update(distance, 0.0, 0.0)
        self.assertEqual(state, FingerState.PROPORTIONAL,
                       f"Should be in PROPORTIONAL state with distance {distance}")
        
        # Now simulate a current spike with negative derivative (indicating impact)
        state, _, _ = fsm.update(distance, 0.2, -0.1)  # Current derivative below threshold
        
        # Should transition to CONTACT state based on current derivative
        self.assertEqual(state, FingerState.CONTACT,
                       "Should transition to CONTACT state based on current derivative")

if __name__ == "__main__":
    unittest.main()