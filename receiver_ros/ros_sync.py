#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import subprocess
import signal
import sys
import os

# Generate the full trial list based on your protocol
objects = ["prismatic_power", "palmar", "adducted_thumb", "cylindrical_power", "key", "precision_pinch"]
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

def print_next_trial(trial_name, retry=False):
    if retry:
        print(f'Next Trial: "{trial_name} (retry)"')
    else:
        print(f'Next Trial: "{trial_name}"')

def print_in_progress(trial_name):
    print(f'Trial In Progress: "{trial_name}"')

def sync_callback(msg):
    global last_sync, trial_in_progress, recording_process, current_trial, retrying_trial, current_trial_index

    current_sync = msg.data

    # Rising edge: 0 -> 1
    if current_sync and not last_sync:
        trial_in_progress = True
        print_in_progress(current_trial)

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

        # Prompt for success/failure
        user_input = input("Was the trial successful? (y/n): ").strip().lower()
        if user_input == 'y':
            # Success: move to next trial
            retrying_trial = False
            current_trial_index += 1
            if current_trial_index < len(trial_list):
                current_trial = trial_list[current_trial_index]
                print_next_trial(current_trial)
            else:
                # No more trials
                print("All trials completed. Exiting.")
                rospy.signal_shutdown("All trials done.")
                sys.exit(0)
        else:
            # Failure: retry same trial
            retrying_trial = True
            print_next_trial(current_trial, retry=True)

    last_sync = current_sync

def main():
    global current_trial, subject_name

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

    # Subscribe to /sync topic
    rospy.Subscriber('/sync', Bool, sync_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
