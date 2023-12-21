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
        bagFileName = "moustafa2.bag"

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
        error_values = extract_reprojection_error(os.path.expanduser("~/Capstone/Bags/moustafa2-results-cam.txt"))
        if error_values:
            print("Reprojection Error Values:", error_values)
        else:
            print("Reprojection error not found.")
    if error_values and d_time < 3000:
        # Define the file path for the CSV file
        resultsFilePath = os.path.expanduser("~/catkin_workspace/src/vincent_scripts/src/calibration_results.csv")

        # Get the current date and time
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Write the data to the CSV file
        with open(resultsFilePath, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([current_time, waypoints, float(error_values[0]), d_time])

        return float(error_values[0]), d_time 

    print("Trial done! Score:",str(error_values[0] * 2000 + d_time))

#waypoints = [[0.45,-0.15,0.466,87.7,17.7,84.2],[0.6,-0.05,0.466,84.9,-32.1,98.2],[0.599,-0.115,0.469,93.2,27.9,83.9],[0.5,-0.02,0.443,92.7,-20,99]]  # Example waypoints
#fullCalibration(waypoints=waypoints,updateWaypoints=True,
#                executeTrajectory=True,
#                recordBag=True,
#                runCalibration=True)

class RobotControl:
    def __init__(self):
        pass

    def move_and_calibrate(self, waypoints):
        return self.call_robot_control_function(waypoints)

    def call_robot_control_function(self, waypoints):         
        reprojection_error, total_time = fullCalibration(waypoints=waypoints,updateWaypoints=True,executeTrajectory=True,recordBag=True,runCalibration=True)        
        print(waypoints)
        for i in range(2,0,-1):
            print("New trial starts in...",i)
            time.sleep(1)
        return reprojection_error, total_time

class Optimizer:
    def __init__(self, robot_control, coord_bounds, initial_guess, max_trials=300, convergence_threshold=5, population_size=10, mutation_rate=0.1):
        self.robot_control = robot_control
        self.coord_bounds = coord_bounds
        self.population_size = max(population_size, 2)
        self.population = [self.create_random_individual() for _ in range(self.population_size)]
        self.population[0] = self.ensure_waypoint_structure(initial_guess)
        self.max_trials = max_trials
        self.convergence_threshold = convergence_threshold
        self.mutation_rate = mutation_rate

    def create_random_individual(self):
        return [[np.random.uniform(low, high) for (low, high) in self.coord_bounds.values()] for _ in range(4)]

    def ensure_waypoint_structure(self, waypoints):
        structured_waypoints = []
        for waypoint in waypoints:
            if isinstance(waypoint, list) and len(waypoint) == 6:
                structured_waypoints.append(waypoint)
            else:
                structured_waypoints.append([0, 0, 0, 0, 0, 0])
        return structured_waypoints

    def objective_function(self, waypoints):
        reprojection_error, total_time = self.robot_control.move_and_calibrate(waypoints)
        return reprojection_error * 2000 + total_time

    def local_search(self, initial_candidate):
        k_max = 5
        current_candidate = [waypoint.copy() for waypoint in initial_candidate]

        for k in range(1, k_max + 1):
            candidate = self.shake(current_candidate, k)
            candidate = self.local_improvement(candidate)

            if self.objective_function(candidate) < self.objective_function(current_candidate):
                current_candidate = candidate
                k = 1

        return current_candidate

    def shake(self, candidate, k):
        shaken_candidate = [waypoint.copy() for waypoint in candidate]
        for _ in range(k):
            waypoint_index = random.randint(0, len(candidate) - 1)
            coordinate_index = random.randint(0, len(candidate[waypoint_index]) - 1)
            low, high = self.coord_bounds[list(self.coord_bounds.keys())[coordinate_index]]
            shaken_candidate[waypoint_index][coordinate_index] = np.random.uniform(low, high)
        return shaken_candidate

    def local_improvement(self, candidate):
        improved_candidate = [waypoint.copy() for waypoint in candidate]
        for waypoint in improved_candidate:
            for i in range(len(waypoint)):
                low, high = self.coord_bounds[list(self.coord_bounds.keys())[i]]
                tweak = np.random.uniform(-0.1, 0.1) * (high - low)
                waypoint[i] = min(max(waypoint[i] + tweak, low), high)
        return improved_candidate
    
    def crossover(self, parent1, parent2):
        child = []
        for i in range(4):
            crossover_point = random.randint(1, 5)
            child_waypoint = parent1[i][:crossover_point] + parent2[i][crossover_point:]
            child.append(child_waypoint)
        return child


    def select_parents(self):
        if len(self.population) >= 2:
            return random.sample(self.population, 2)
        else:
            return [self.population[0], self.create_random_individual()]

    def generate_new_candidate(self):
        parent1, parent2 = self.select_parents()
        child = self.crossover(parent1, parent2)
        return child

    def optimize(self):
        best_score = float('inf')
        best_waypoints = None

        for _ in range(self.max_trials):
            candidate = self.generate_new_candidate()
            candidate = self.local_search(candidate)
            score = self.objective_function(candidate)

            if score < best_score:
                improvement = best_score - score
                if improvement < self.convergence_threshold:
                    break
                best_score = score
                best_waypoints = candidate

            self.population.append(candidate)
            self.population.sort(key=lambda ind: self.objective_function(ind))
            self.population = self.population[:len(self.population)//2]

        return best_waypoints, best_score
    

print('hello')
# replace with your actual bounds
x_pos_min, x_pos_max = 0.5,0.75
y_pos_min, y_pos_max = -0.25,0.25
z_pos_min, z_pos_max = 0.24,0.47
w_yaw_min, w_yaw_max = 85,95
p_pitch_min, p_pitch_max = -30,30
r_roll_min, r_roll_max = 70,110
coord_bounds = {'x_pos': (x_pos_min, x_pos_max), 'y_pos': (y_pos_min, y_pos_max), 'z_pos': (z_pos_min, z_pos_max), 'w_yaw': (w_yaw_min, w_yaw_max), 'p_pitch': (p_pitch_min, p_pitch_max), 'r_roll': (r_roll_min, r_roll_max)}
initial_guess = [[0.5,-0.15,0.466,87.7,17.7,84.2],[0.6,-0.05,0.466,84.9,-32.1,98.2],[0.599,-0.115,0.469,93.2,27.9,83.9],[0.5,-0.02,0.443,92.7,-20,99]]

# actual optimization
max_trials = 50
robot_control = RobotControl()
optimizer = Optimizer(robot_control, coord_bounds, initial_guess, max_trials)
best_coordinates, best_score = optimizer.optimize()

print("Best Score:", best_score)
print("Best Coordinates:", best_coordinates)