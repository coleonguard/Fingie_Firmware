import rospy
from std_msgs.msg import String
import time
from signal import signal, SIGINT

# Initialize ROS node
rospy.init_node('test_ros_publisher')

# Define publishers for each sensor channel under the 'fingie_sensors' namespace
pub_proximity = rospy.Publisher('/fingie_sensors/proximity', String, queue_size=10)
pub_pressure = rospy.Publisher('/fingie_sensors/pressure', String, queue_size=10)
pub_emg = rospy.Publisher('/fingie_sensors/emg', String, queue_size=10)
pub_imu = rospy.Publisher('/fingie_sensors/imu', String, queue_size=10)

# Define a 2-second rate for publishing
rate = rospy.Rate(0.5)  # 0.5 Hz (every 2 seconds)

# Handle graceful exit
exit_flag = False
def exit_gracefully(signal_received, frame):
    global exit_flag
    exit_flag = True
signal(SIGINT, exit_gracefully)

# Main loop to publish test messages
while not rospy.is_shutdown() and not exit_flag:
    # Simulated data for each sensor channel
    proximity_data = " ".join(str(i) for i in range(1, 19))  # 18 integers
    pressure_data = " ".join(f"{i*0.1:.1f}" for i in range(1, 22))  # 21 floats
    emg_data = f"{int(time.time())} {int(time.time())+5} " + " ".join(str(i*10) for i in range(1, 9)) + " 1"  # EMG format
    imu_data = " ".join(f"{i*10.5:.1f}" for i in range(1, 10))  # 9 DoF values

    # Publish data to each topic
    pub_proximity.publish(proximity_data)
    pub_pressure.publish(pressure_data)
    pub_emg.publish(emg_data)
    pub_imu.publish(imu_data)

    # Log to confirm publishing
    rospy.loginfo(f"Published proximity data: {proximity_data}")
    rospy.loginfo(f"Published pressure data: {pressure_data}")
    rospy.loginfo(f"Published emg data: {emg_data}")
    rospy.loginfo(f"Published imu data: {imu_data}")

    # Wait 2 seconds before publishing again
    rate.sleep()

# Confirm script exit
if exit_flag:
    print("Exited gracefully.")
