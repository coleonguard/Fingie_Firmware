#!/usr/bin/env python3
# Launcher script for prosthetic hand controller

import sys
import os

# Add the parent directory to the path so we can import the controller module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import and run the controller
from prosthetic_rpi.controller.main import main

if __name__ == "__main__":
    sys.exit(main())