import subprocess
import signal
import time
import os
import re
import csv
from datetime import datetime
import numpy as np
import random

def start_rosbag_recording(topics, output_directory, file_name):
    # Ensure the output directory exists
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)

    # Full path for the output file
    output_file = os.path.join(output_directory, file_name)

    # Start the rosbag record command as a subprocess
    command = ["rosbag", "record", "-O", output_file] + topics
    print("AUTO: command", command)
    process = subprocess.Popen(command)
    return process

def stop_rosbag_recording(process):
    # Terminate the rosbag recording process
    process.send_signal(signal.SIGINT)

def launch_ros_file(package,launchfile):

    try:
        # Replace 'your_launch_file.launch' with the name of your launch file
        # Include the full path if the file is not in the current directory
        subprocess.run(["roslaunch", package, launchfile])
    except subprocess.CalledProcessError as e:
        print("Failed to launch ROS launch file:", e)

def calibrate(targetFile, bagFile, topics):
    command = "source ~/kalibr_workspace/devel/setup.bash && " \
              "rosrun kalibr kalibr_calibrate_cameras --target {} --models pinhole-radtan pinhole-radtan --topics {} --bag {} --bag-freq 10.0 --show-extraction".format(
                  targetFile, ' '.join(topics), bagFile)
    subprocess.run(command, shell=True, executable='/bin/bash')

def extract_reprojection_error(filePath):
    # Resolve the path
    resolvedPath = os.path.expanduser(filePath)

    # Regular expression to match the reprojection error line and capture the values
    regex = r"reprojection error: \[.*\] \+- \[(\d+\.\d+), (\d+\.\d+)\]"

    # Read the file and search for the reprojection error
    with open(resolvedPath, 'r') as file:
        content = file.read()
        matches = re.findall(regex, content)

        # Assuming you want to extract the values from the last occurrence of reprojection error
        if matches:
            return matches[-1]  # Returns a tuple with the two values
        else:
            return None

def fullCalibration(waypoints,updateWaypoints=True,
                    executeTrajectory=True,
                    recordBag=True,
                    runCalibration=True
                    ):
    d_time = 10000.0
    if updateWaypoints:
        # Save waypoints to txt
        filePath = os.path.expanduser("~/catkin_workspace/src/vincent_scripts/src/waypoints.txt")
        with open(filePath, 'w', newline='') as file:
            writer = csv.writer(file)
            for waypoint in waypoints:
                writer.writerow(waypoint)
    start_time = time.time()
    if executeTrajectory and recordBag:
        bagsDir = "~/Capstone/Bags/"
        bagFileName = "moustafa3.bag"

        dataRecording = start_rosbag_recording(topics=["/G435i/infra1/image_rect_raw",
                                                    "/G435i/infra2/image_rect_raw"],
                                            output_directory = os.path.expanduser(bagsDir),
                                            file_name=bagFileName)
        print("AUTO: recording started")
        launch_ros_file(package="vincent_scripts",launchfile="movevince.launch")
        print("AUTO: Phase 1 done")

        time.sleep(1)
        stop_rosbag_recording(dataRecording)
        time.sleep(1)
        print("AUTO: Recording finished")
    
    if executeTrajectory == True and recordBag == False:
        print("Warning! No bag recording")
        launch_ros_file(package="vincent_scripts",launchfile="movevince.launch")
        d_time = time.time() - start_time
        print("Execution time:",d_time,"s")
        print("AUTO: Phase 1 done")
    
    if runCalibration:
        targetFile = "~/Capstone/Boards/6x6.yaml"
        for i in range(2,0,-1):
            print("Calibration starts in...",i)
            time.sleep(1)

        calibrate(targetFile=os.path.expanduser(targetFile),
                bagFile=os.path.expanduser("~/Capstone/Bags/"+bagFileName),
                topics=["/G435i/infra1/image_rect_raw","/G435i/infra2/image_rect_raw"])

        print("Calibration done")
        d_time = time.time() - start_time
        print("Total calibration time:",d_time,"s")

        for i in range(2,0,-1):
            print("Report ready in...",i)
            time.sleep(1)

        # Return re-projection error
        error_values = extract_reprojection_error(os.path.expanduser("~/Capstone/Bags/" + bagFileName.split(".")[0] + "-results-cam.txt"))
        if error_values:
            print("Reprojection Error Values:", error_values)
        else:
            print("Reprojection error not found.")
    if error_values and d_time < 3000:
        # Define the file path for the CSV file
        resultsFilePath = os.path.expanduser("~/catkin_workspace/src/vincent_scripts/src/capstone_single_results_log.csv")

        # Get the current date and time
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Write the data to the CSV file
        with open(resultsFilePath, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([current_time, waypoints, float(error_values[0]), d_time])

        return float(error_values[0]), d_time 

    print("Trial done!")

defaultPoints = [[0.45,-0.15,0.466,87.7,17.7,84.2],[0.6,-0.05,0.466,84.9,-32.1,98.2],[0.599,-0.115,0.469,93.2,27.9,83.9],[0.5,-0.02,0.443,92.7,-20,99]]  # Example waypoints
crossPaths = [
    [0.561,0.455,0.423,90.2,-2.3,91.6],
    [0.532,-0.436,0.416,89.5,-2.6,89.9],
    [0.608,0.2,0.669,95.6,-4,86.5],
    [0.621,0.056,0.174,88.7,-7.9,87.8],
    [1.0,0.025,0.319,90.1,-4.2,92.8],
    [0.577,0.001,0.434,90,0,90]
]
#[[0.45,-0.15,0.466,87.7,17.7,84.2],[0.6,-0.05,0.466,84.9,-32.1,98.2],[0.599,-0.115,0.469,93.2,27.9,83.9],[0.5,-0.02,0.443,92.7,-20,99]]

fullCalibration(waypoints=defaultPoints,updateWaypoints=True,
                executeTrajectory=True,
                recordBag=True,
                runCalibration=True)