# Hand motor interface and control package

import os
import sys
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("hand")

# Add the ability-hand-api to the Python path
try:
    # Find the project root (parent directory of prosthetic_control_system)
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
    
    # Path to the ability-hand-api Python package
    ability_hand_path = os.path.join(project_root, 'ability-hand-api', 'python')
    
    # Add to path if it exists
    if os.path.isdir(ability_hand_path):
        logger.info(f"Adding Ability Hand API to Python path: {ability_hand_path}")
        sys.path.insert(0, ability_hand_path)
    else:
        logger.warning(f"Ability Hand API not found at {ability_hand_path}")
        logger.warning("Using simulated mode for Ability Hand")
except Exception as e:
    logger.error(f"Error configuring Ability Hand API path: {e}")
    logger.warning("Using simulated mode for Ability Hand")