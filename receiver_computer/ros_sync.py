#!/usr/bin/env python3
import rospy
from std_msgs_stamped.msg import BoolStamped
import subprocess
import signal
import sys
import os
import tkinter as tk
from threading import Thread

# ------------------------------------------------------------------------------
# Trial Descriptions
# Using a dictionary to store the single-sentence descriptions for each trial type.
# You can add or modify as needed for more detail.
# ------------------------------------------------------------------------------

TRIAL_DESCRIPTIONS = {
    # --------------------------------------------------------------------------
    # Static Trials
    # --------------------------------------------------------------------------
    "FlatHand3s": "Hold your hand flat on the table for 3 seconds without movement.",
    "AllFingerPinch3s": "Gently pinch all fingers together for 3 seconds and remain still.",

    # --------------------------------------------------------------------------
    # Prehensile Trials
    # --------------------------------------------------------------------------
    # Abducted (Power)
    "LargeCylinder": "Grasp the large cylinder firmly, lift it 6 inches, hold for 3 seconds, and replace.",
    "SmallCylinder": "Grip the small cylinder, lift it 6 inches, hold for 3 seconds, and set it down.",
    "Disk": "Grasp the disk securely, lift it 6 inches, hold for 3 seconds, and place it back.",
    "Ring": "Hold the ring with your fingers, lift it 6 inches, maintain position for 3 seconds, then replace.",

    # Adducted (Power)
    "Cylindrical": "Wrap your fingers around the cylindrical object, lift it 6 inches, hold for 3 seconds, then set it down.",
    "Palmar": "Grasp the object with your palm, lift it 6 inches, hold for 3 seconds, and release.",
    # "FixedHook": "Use a hook grip to lift the object 6 inches, hold for 3 seconds, and return it to the table.",

    # Precision
    "Key": "Hold the object like a key, lift it 6 inches, hold for 3 seconds, and return it.",
    # "PalmarPrecision": "Use your fingertips to grasp the object, lift 6 inches, hold for 3 seconds, and replace.",
    # "DiskPrecision": "Grip the disk with your fingertips, lift it 6 inches, hold for 3 seconds, and place it back.",
    # "ParallelPrecision": "Pinch the object between thumb and fingers, lift it 6 inches, hold for 3 seconds, and set it down.",

    # --------------------------------------------------------------------------
    # Non-Prehensile Trials
    # --------------------------------------------------------------------------
    "FlatPalmPress": "Press your palm flat against the table for 3 seconds, then return to rest.",
    "FingerPress": "Press your fingers against the table for 3 seconds, then return to rest.",

    # --------------------------------------------------------------------------
    # Neutral (Resting) Trials
    # --------------------------------------------------------------------------
    "MovingArmOnly": "Move your hand randomly for 3 seconds, keeping your fingers and wrist mostly still.",
    "HandFlat": "Rest your hand flat on the table for 3 seconds while allowing minor arm movement.",
    "FingersCurled": "Keep your fingers curled into a fist on the table for 3 seconds while allowing minor arm movement.",
    # "MinimalFinger": "Minimize finger movement while keeping your hand mostly still for 3 seconds."  # Commented out for reference
}

# ------------------------------------------------------------------------------
# Updated trial configuration based on the new experimental protocol
# ------------------------------------------------------------------------------
# 1) Static Trials:
#    - 2 tasks: FlatHand3s, AllFingerPinch3s
#    - We'll repeat each task 3 times for demonstration (adjust as needed).
static_trials = []
static_tasks = ["FlatHand3s", "AllFingerPinch3s"]
static_reps = range(1, 2)  # e.g., 1 repeats

for task in static_tasks:
    for r in static_reps:
        static_trials.append(f"Static/{task}_{r}")

# 2) Prehensile Trials:
#    - Abducted (Power): LargeCylinder, SmallCylinder, Disk, Ring
#    - Adducted (Power): Cylindrical, Palmar (# 'FixedHook' commented out as example)
#    - Precision: Key (# 'PalmarPrecision', 'DiskPrecision', 'ParallelPrecision' commented out)
#    - Each grasp has 15 repeats at 3 force levels (light, normal, hard) => 45 total
#      (Here, we're just using 5 repeats each for example.)
prehensile_trials = []
abducted_power = ["LargeCylinder", "SmallCylinder", "Disk", "Ring"]
adducted_power = ["Cylindrical", "Palmar"]  # ,"FixedHook",
precision_grips = ["Key"]  # , "PalmarPrecision", "DiskPrecision", "ParallelPrecision"
forces = ["light", "normal", "hard"]
prehensile_reps = range(1, 6)  # 5 repeats

# Abducted
for grasp in abducted_power:
    for force in forces:
        for r in prehensile_reps:
            prehensile_trials.append(f"Prehensile/Abducted_{grasp}_{force}_{r}")

# Adducted
for grasp in adducted_power:
    for force in forces:
        for r in prehensile_reps:
            prehensile_trials.append(f"Prehensile/Adducted_{grasp}_{force}_{r}")

# Precision
for grasp in precision_grips:
    for force in forces:
        for r in prehensile_reps:
            prehensile_trials.append(f"Prehensile/Precision_{grasp}_{force}_{r}")

# 3) Non-Prehensile Trials:
#    - Movement Types: FlatPalmPress, FingerPress
#    - 45 repeats each (15 for each force: light, normal, hard) (just 5 each for example)
non_prehensile_trials = []
non_prehensile = ["FlatPalmPress", "FingerPress"]
non_prehensile_reps = range(1, 6)  # 5 repeats

for movement in non_prehensile:
    for force in forces:
        for r in non_prehensile_reps:
            non_prehensile_trials.append(f"NonPrehensile/{movement}_{force}_{r}")

# 4) Neutral (Resting) Trials:
#    - Movements: HandFlat, MovingArmOnly, FingersCurled
#    - 15 repeats each (just 5 each for example)
neutral_trials = []
neutral_movements = ["HandFlat", "MovingArmOnly", "FingersCurled"]  # 'MinimalFinger' removed
neutral_reps = range(1, 6)  # 5 repeats

for movement in neutral_movements:
    for r in neutral_reps:
        neutral_trials.append(f"Neutral/{movement}_{r}")

# Combine everything into one list
trial_list = static_trials + prehensile_trials + non_prehensile_trials + neutral_trials

# ------------------------------------------------------------------------------
# Global variables and setup
# ------------------------------------------------------------------------------
current_trial_index = 0
current_trial = None
last_sync = False
recording_process = None
trial_in_progress = False
retrying_trial = False
subject_name = ""

# Base data directory (adjust path as needed)
BASE_DIR = os.path.expanduser("~/Desktop/YourStudy/data/")

# ------------------------------------------------------------------------------
# Simple Tkinter-based GUI to display the trial status and description
# ------------------------------------------------------------------------------
class TrialGUI:
    def __init__(self, total_trials):
        self.root = tk.Tk()
        self.root.title("Trial Status")
        self.label_status = tk.Label(self.root, text="Waiting...", font=("Helvetica", 24))
        self.label_status.pack(pady=20)

        self.label_trial = tk.Label(self.root, text="Trial 0", font=("Helvetica", 18))
        self.label_trial.pack(pady=10)

        # New label for the single-sentence description
        self.label_description = tk.Label(self.root, text="", font=("Helvetica", 14), wraplength=600, justify="center")
        self.label_description.pack(pady=10)

        self.total_trials = total_trials
        self.current_trial = 0

    def update_status(self, force, trial_number, in_progress, description):
        # Update the text for which trial we are on
        self.current_trial = trial_number
        display_text = f"Trial {trial_number}/{self.total_trials}"
        if force:
            display_text = f"{force.capitalize()} Force, {display_text}"

        self.label_trial.config(text=display_text)
        self.label_description.config(text=description)

        # Update background color and status label
        if in_progress:
            self.root.config(bg="green")
            self.label_status.config(text="Trial In Progress", bg="green", fg="white")
        else:
            self.root.config(bg="red")
            self.label_status.config(text="Trial Ended", bg="red", fg="white")

    def start(self):
        self.root.mainloop()

# ------------------------------------------------------------------------------
# Thread function to run the GUI
# ------------------------------------------------------------------------------
def run_gui():
    global trial_gui
    trial_gui.start()

# ------------------------------------------------------------------------------
# Helper print functions
# ------------------------------------------------------------------------------
def print_next_trial(trial_name, retry=False):
    if retry:
        print(f'Next Trial: "{trial_name} (retry)"')
    else:
        print(f'Next Trial: "{trial_name}"')

def print_in_progress(trial_name):
    print(f'Trial In Progress: "{trial_name}"')

# ------------------------------------------------------------------------------
# Function to extract a short description of the trial from TRIAL_DESCRIPTIONS
# ------------------------------------------------------------------------------
def get_trial_description(trial_name):
    """
    Parses the trial_name to determine a relevant single-sentence description
    from TRIAL_DESCRIPTIONS. If none found, returns a default string.
    """
    try:
        # trial_name might look like: 'Prehensile/Abducted_LargeCylinder_light_1'
        main_part = trial_name.split('/')[1]  # e.g. 'Abducted_LargeCylinder_light_1'
        sub_parts = main_part.split('_')

        # We'll check each part to see if it matches a key in TRIAL_DESCRIPTIONS
        for part in sub_parts:
            if part in TRIAL_DESCRIPTIONS:
                return TRIAL_DESCRIPTIONS[part]

        # If none of the sub_parts matched, it might be a static or neutral movement
        # e.g., 'FlatHand3s_1'
        if '_' in main_part:
            task_name = main_part.rsplit('_', 1)[0]  # 'FlatHand3s'
            if task_name in TRIAL_DESCRIPTIONS:
                return TRIAL_DESCRIPTIONS[task_name]

    except Exception:
        pass

    # If we can't find anything in the dictionary, default:
    return "No specific description found for this trial."

# ------------------------------------------------------------------------------
# ROS callback for /sync topic
# ------------------------------------------------------------------------------
def sync_callback(msg):
    global last_sync, trial_in_progress, recording_process
    global current_trial, retrying_trial, current_trial_index, trial_gui

    current_sync = msg.data  # True/False from BoolStamped

    # Rising edge: start trial
    if current_sync and not last_sync:
        trial_in_progress = True
        print_in_progress(current_trial)

        # Determine force for display
        trial_name_parts = current_trial.split('/')
        sub_parts = trial_name_parts[1].split('_')
        if len(sub_parts) >= 3:
            force = sub_parts[-2]
        else:
            force = ""

        # Get a description for the current trial
        description = get_trial_description(current_trial)

        # Update GUI
        trial_gui.update_status(force, current_trial_index + 1, in_progress=True, description=description)

        # Create a safe filename by replacing '/'
        bag_name = current_trial.replace("/", "_") + ".bag"
        bag_path = os.path.join(BASE_DIR, subject_name, bag_name)

        # Start recording
        cmd = ["rosbag", "record", "-a", "-O", bag_path]
        recording_process = subprocess.Popen(cmd)

    # Falling edge: end trial
    if not current_sync and last_sync:
        # End trial
        if recording_process is not None:
            # Stop rosbag record
            recording_process.send_signal(signal.SIGINT)
            recording_process.wait()
            recording_process = None

        trial_in_progress = False
        trial_gui.update_status("", current_trial_index + 1, in_progress=False, description="Trial has ended.")

        # Prompt user to decide if we retry or move to next
        retrying_trial = False
        user_input = input("Press 'n' to retry the previous trial, or press Enter to proceed: ").strip().lower()
        if user_input == 'n':
            retrying_trial = True
            print_next_trial(current_trial, retry=True)
        else:
            current_trial_index += 1
            if current_trial_index < len(trial_list):
                current_trial = trial_list[current_trial_index]
                print_next_trial(current_trial)
            else:
                print("All trials completed. Exiting.")
                rospy.signal_shutdown("All trials done.")
                sys.exit(0)

    last_sync = current_sync

# ------------------------------------------------------------------------------
# Main entry point
# ------------------------------------------------------------------------------
def main():
    global current_trial, subject_name, trial_gui

    # Prompt for subject name at start
    subject_name = input("Enter subject name: ").strip()
    if not subject_name:
        print("Subject name cannot be empty. Exiting.")
        sys.exit(1)

    # Create directory structure for this subject
    subject_dir = os.path.join(BASE_DIR, subject_name)
    os.makedirs(subject_dir, exist_ok=True)

    rospy.init_node('trial_manager', anonymous=True)

    current_trial = trial_list[0]
    print_next_trial(current_trial)

    # Initialize GUI
    trial_gui = TrialGUI(len(trial_list))
    gui_thread = Thread(target=run_gui)
    gui_thread.daemon = True
    gui_thread.start()

    # Subscribe to /sync topic with BoolStamped type
    rospy.Subscriber('/sync', BoolStamped, sync_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
