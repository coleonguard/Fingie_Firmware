#!/usr/bin/env python3
"""
Component test for the hand state machine (C-03).

Tests:
- C-03: Hand FSM release - Tests the complete release sequence (lowering → impact → stationary → release)
"""

import sys
import os
import unittest
import time
from unittest.mock import MagicMock

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import components to test
from controller.state_machines import HandFSM, HandState, FingerState

class TestHandFSM(unittest.TestCase):
    """Test cases for the hand state machine"""
    
    def test_hand_release_sequence(self):
        """Test the complete release sequence (C-03)"""
        # Create hand FSM with standard settings
        hand_fsm = HandFSM(release_current_threshold=0.2, required_fingers_grasping=2)
        
        # Create mocked finger states and currents
        finger_states = {
            "Thumb": FingerState.CONTACT,
            "Index": FingerState.CONTACT,
            "Middle": FingerState.CONTACT,
            "Ring": FingerState.PROPORTIONAL,
            "Pinky": FingerState.PROPORTIONAL
        }
        
        finger_currents = {
            "Thumb": 0.3,
            "Index": 0.3,
            "Middle": 0.3,
            "Ring": 0.1,
            "Pinky": 0.1
        }
        
        # Initialize IMU-related variables
        is_lowering = False
        is_impact_detected = False
        is_stationary = False
        time_since_impact = 0.0
        
        # Step 1: Start in IDLE state
        self.assertEqual(hand_fsm.state, HandState.IDLE, 
                       "Initial state should be IDLE")
        
        # Step 2: Transition to REACH due to fingers in PROPORTIONAL/CONTACT
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.REACH, 
                       "Should transition to REACH with fingers in PROPORTIONAL/CONTACT")
        
        # Step 3: Transition to GRASP once enough fingers are in CONTACT
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.GRASP, 
                       "Should transition to GRASP with enough fingers in CONTACT")
        
        # Step 4: Transition to LOWER when lowering is detected
        is_lowering = True
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.LOWER, 
                       "Should transition to LOWER when lowering detected")
        
        # Step 5: Stay in LOWER when impact detected but not stationary yet
        is_impact_detected = True
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.LOWER, 
                       "Should stay in LOWER when impact detected but not stationary")
        
        # Step 6: No release yet because currents are still high
        is_stationary = True
        time_since_impact = 0.5  # More than the stationary window (0.3s)
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.LOWER, 
                       "Should stay in LOWER when currents are high")
        
        # Step 7: Transition to RELEASE when currents are low
        finger_currents = {finger: 0.1 for finger in finger_currents}  # All currents low
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.RELEASE, 
                       "Should transition to RELEASE with low currents + stationary")
        
        # Step 8: Always transition to RETRACT after RELEASE
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.RETRACT, 
                       "Should transition to RETRACT after RELEASE")
        
        # Step 9: Stay in RETRACT until fingers are open
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.RETRACT, 
                       "Should stay in RETRACT until fingers are open")
        
        # Step 10: Transition back to IDLE when fingers are open
        finger_states = {finger: FingerState.APPROACH for finger in finger_states}
        state = hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary, time_since_impact
        )
        self.assertEqual(state, HandState.IDLE, 
                       "Should transition to IDLE when fingers are open")
    
    def test_edge_cases(self):
        """Test edge cases that might cause glitches"""
        # Create hand FSM with standard settings
        hand_fsm = HandFSM(release_current_threshold=0.2, required_fingers_grasping=2)
        
        # Initial conditions
        finger_states = {
            "Thumb": FingerState.CONTACT,
            "Index": FingerState.CONTACT,
            "Middle": FingerState.CONTACT,
            "Ring": FingerState.PROPORTIONAL,
            "Pinky": FingerState.PROPORTIONAL
        }
        
        finger_currents = {
            "Thumb": 0.3,
            "Index": 0.3,
            "Middle": 0.3,
            "Ring": 0.1,
            "Pinky": 0.1
        }
        
        # Get to GRASP state
        hand_fsm.update(finger_states, finger_currents, False, False, False, 0.0)
        hand_fsm.update(finger_states, finger_currents, False, False, False, 0.0)
        self.assertEqual(hand_fsm.state, HandState.GRASP)
        
        # Edge case 1: Brief lowering followed by non-lowering - should not remain in LOWER
        hand_fsm.update(finger_states, finger_currents, True, False, False, 0.0)
        self.assertEqual(hand_fsm.state, HandState.LOWER)
        
        hand_fsm.update(finger_states, finger_currents, False, False, False, 0.0)
        self.assertEqual(hand_fsm.state, HandState.GRASP, 
                      "Should return to GRASP after brief lowering")
        
        # Edge case 2: Impact without lowering - should not trigger release
        hand_fsm.update(finger_states, finger_currents, False, True, True, 0.5)
        finger_currents = {finger: 0.1 for finger in finger_currents}
        hand_fsm.update(finger_states, finger_currents, False, True, True, 0.5)
        
        self.assertNotEqual(hand_fsm.state, HandState.RELEASE, 
                         "Should not release without lowering first")
        
        # Edge case 3: Lost grasp during lowering - should go back to GRASP
        hand_fsm.state = HandState.LOWER
        finger_states = {
            "Thumb": FingerState.PROPORTIONAL,
            "Index": FingerState.PROPORTIONAL,
            "Middle": FingerState.PROPORTIONAL,
            "Ring": FingerState.PROPORTIONAL,
            "Pinky": FingerState.PROPORTIONAL
        }
        
        hand_fsm.update(finger_states, finger_currents, True, False, False, 0.0)
        self.assertEqual(hand_fsm.state, HandState.GRASP, 
                      "Should return to GRASP if contact lost during lowering")

if __name__ == "__main__":
    unittest.main()