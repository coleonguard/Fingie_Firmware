#!/usr/bin/env python3
import rospy
from std_msgs_stamped.msg import BoolStamped
import subprocess
import signal
import sys
import os
import tkinter as tk
from threading import Thread

# Generate the full trial list based on your protocol
objects = ["prismatic_power", "palmar_prismatic", "adducted_prismatic", "cylindrical_power", "key_pinch", "precision_pinch"]
orientations = ["vertical", "horizontal"]
forces = ["light", "normal", "hard"]
reps = range(1, 6)  # 1 through 5

trial_list = []
for obj in objects:
    for orientation in orientations:
        for force in forces:
            for r in reps:
                trial_name = f"{obj}/{orientation}_{force}_{r}"
                trial_list.append(trial_name)

current_trial_index = 0
current_trial = None
last_sync = False
recording_process = None
trial_in_progress = False
retrying_trial = False
subject_name = ""

# Base data directory
BASE_DIR = os.path.expanduser("~/Desktop/Cole/infinity_gauntlet_data/")

# GUI Class
class TrialGUI:
    def __init__(self, total_trials):
        self.root = tk.Tk()
        self.root.title("Trial Status")
        self.label_status = tk.Label(self.root, text="Waiting...", font=("Helvetica", 24))
        self.label_status.pack(pady=20)

        self.label_trial = tk.Label(self.root, text="Trial 0", font=("Helvetica", 18))
        self.label_trial.pack(pady=10)

        self.total_trials = total_trials
        self.current_trial = 0

    def update_status(self, force, trial_number, in_progress):
        self.current_trial = trial_number
        self.label_trial.config(text=f"{force.capitalize()} Grasp, Trial {trial_number}/{self.total_trials}")

        if in_progress:
            self.root.config(bg="green")
            self.label_status.config(text="Trial In Progress", bg="green", fg="white")
        else:
            self.root.config(bg="red")
            self.label_status.config(text="Trial Ended", bg="red", fg="white")

    def start(self):
        self.root.mainloop()

# Function to update GUI in a separate thread
def run_gui():
    global trial_gui
    trial_gui.start()

def print_next_trial(trial_name, retry=False):
    if retry:
        print(f'Next Trial: "{trial_name} (retry)"')
    else:
        print(f'Next Trial: "{trial_name}"')

def print_in_progress(trial_name):
    print(f'Trial In Progress: "{trial_name}"')

def sync_callback(msg):
    global last_sync, trial_in_progress, recording_process, current_trial, retrying_trial, current_trial_index, trial_gui

    current_sync = msg.data  # Extract the data field from BoolStamped

    # Rising edge: 0 -> 1
    if current_sync and not last_sync:
        trial_in_progress = True
        print_in_progress(current_trial)

        # Update GUI
        trial_name_parts = current_trial.split('/')
        force = trial_name_parts[1].split('_')[1]
        trial_gui.update_status(force, current_trial_index + 1, in_progress=True)

        # Create a safe filename by replacing '/'
        bag_name = current_trial.replace("/", "_") + ".bag"
        # Full path under subject directory
        bag_path = os.path.join(BASE_DIR, subject_name, bag_name)

        # Start recording
        cmd = ["rosbag", "record", "-a", "-O", bag_path]
        recording_process = subprocess.Popen(cmd)

    # Falling edge: 1 -> 0
    if not current_sync and last_sync:
        # End trial
        if recording_process is not None:
            # Stop rosbag record
            recording_process.send_signal(signal.SIGINT)
            recording_process.wait()
            recording_process = None

        trial_in_progress = False
        trial_gui.update_status("", current_trial_index + 1, in_progress=False)

        # Automatically proceed to next trial unless retrying
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
