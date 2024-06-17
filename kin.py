import numpy as np
import matplotlib.pyplot as plt
import rospy
from sr_robot_commander.sr_hand_commander import SrHandCommander

rospy.init_node("robot_commander_examples", anonymous=True)
hand_commander = SrHandCommander(name="right_hand")

print("Robot name: ", hand_commander.get_robot_name())
print("Group name: ", hand_commander.get_group_name())
print("Planning frame: ", hand_commander.get_planning_frame())

joints_position = hand_commander.get_joints_position()
joints_velocity = hand_commander.get_joints_velocity()

print("Hand joint positions\n" + str(joints_position) + "\n")
print("Hand joint velocities\n" + str(joints_velocity) + "\n")

current_state = hand_commander.get_current_state()

current_state = hand_commander.get_current_state_bounded()

#hand_commander.plan_to_named_target("open")
#hand_commander.move_to_named_target("open")

#hand_commander.plan_to_named_target("pack")
#hand_commander.move_to_named_target("pack")

#trajectory = [{'name': 'open', 'interpolate_time': 4.0}, {'name': 'pack', 'interpolate_time': 4.0, 'pause_time': 2}, {'name': 'open', 'interpolate_time': 4.0}]

trajectory = [{'name': 'open', 'interpolate_time': 4.0}]

hand_commander.run_named_trajectory(trajectory)

# Kalman filter initialization steps

# Define the state transition matrix (constant velocity model)
dt = 1  # Time step
A = np.array([[1, dt],
              [0, 1]])

# Define the observation matrix
H = np.array([[1, 0]])

# Define the covariance of the process noise
Q = np.array([[0.1, 0],
              [0, 0.1]])

# Define the covariance of the observation noise
R = np.array([[1]])

# Initial state estimate
x_hat = np.array([[0],
                  [0]])

# Initial error covariance
P = np.array([[1, 0],
              [0, 1]])

# Simulate real-time motion
# true_position = 0

# Function to simulate real-time measurements
# def get_measurement():
#     global true_position
#     true_position += 1  # Simulate constant velocity motion
#     return true_position + np.random.randn()  # Simulate noisy measurement
threshold = 0.05

with open('streams_m.txt', 'r') as file:
    # Read the entire contents of the file
    # file_contents = file.read()
    # Print or do whatever you want with the contents
    # print(file_contents)
    # Read the first line of the file
    #first_line = file.readline()
    # Print or do whatever you want with the first line
    #print(first_line)
    #step = 20
    tag = 0
    
    
    # init for the current measurement
    measurements_current = {}
    filtered_positions_current = {}
    
    x_hat_minus_current = {}
    P_minus_current = {}
    # Update
    K_current = {}
    x_hat_current = {}
    P_current = {}
    
    # init for the previous measurement
    measurements_previous = {}
    filtered_positions_previous = {}
    
    x_hat_minus_previous = {}
    P_minus_previous = {}
    # Update
    K_previous = {}
    x_hat_previous = {}
    P_previous = {}
    
    # initialize all vectors
    for i in range(22): # number of angles supported by LeapMotion 
        measurements_current[i] = []
        filtered_positions_current[i] = []
        
        x_hat_minus_current[i] = []
        P_minus_current[i] = []
        # Update
        K_current[i] = []
        x_hat_current[i] = []
        P_current[i] = []
        
        measurements_previous[i] = []
        filtered_positions_previous[i] = []
        
        x_hat_minus_previous[i] = []
        P_minus_previous[i] = []
        # Update
        K_previous[i] = []
        x_hat_previous[i] = []
        P_previous[i] = []
    
    for line in file: 
        for lineno, line in enumerate(file): # check enumerate (it doesn't capture the first line)
            # print (line)
            tmp = line.split()
            tmp_float = [float(x) for x in tmp]
            # print(tmp_float[0])
            if lineno > 0:
                current = tmp_float # current flow of data
                tag = 0

                l_diff = []
                for i in range(22): # number of angles # change to 22
            
                    # current angle
                    z1 = current[i]

                    # Prediction
                    if lineno == 1:
                        x_hat_minus_current[i].append(np.dot(A, x_hat))
                        P_minus_current[i].append(np.dot(A, np.dot(P, A.T)) + Q)
                    else: 
                        x_hat_minus_current[i].append(np.dot(A, x_hat_current[i][lineno - 2]))
                        P_minus_current[i].append(np.dot(A, np.dot(P_current[i][lineno -  2], A.T)) + Q)
                                        
                    # Update
                    K_current[i].append(np.dot(P_minus_current[i][lineno - 1], np.dot(H.T, np.linalg.inv(np.dot(H, np.dot(P_minus_current[i][lineno - 1], H.T)) + R))))
                    x_hat_current[i].append(x_hat_minus_current[i][lineno - 1] + np.dot(K_current[i][lineno - 1], (z1 - np.dot(H, x_hat_minus_current[i][lineno - 1]))))
                    P_current[i].append(np.dot((np.eye(2) - np.dot(K_current[i][lineno - 1], H)), P_minus_current[i][lineno - 1]))

                    filtered_positions_current[i].append(x_hat_current[i][lineno - 1][0])

                    measurements_current[i].append(z1)
                    
                    # previous angle
                    z2 = prev[i]
                    

                    # Prediction
                    if lineno == 1:
                        x_hat_minus_previous[i].append(np.dot(A, x_hat))
                        P_minus_previous[i].append(np.dot(A, np.dot(P, A.T)) + Q)
                    else: 
                        x_hat_minus_previous[i].append(np.dot(A, x_hat_previous[i][lineno - 2]))
                        P_minus_previous[i].append(np.dot(A, np.dot(P_previous[i][lineno -  2], A.T)) + Q)

                    # Update
                    K_previous[i].append(np.dot(P_minus_previous[i][lineno - 1], np.dot(H.T, np.linalg.inv(np.dot(H, np.dot(P_minus_previous[i][lineno - 1], H.T)) + R))))
                    x_hat_previous[i].append(x_hat_minus_previous[i][lineno - 1] + np.dot(K_previous[i][lineno - 1], (z2 - np.dot(H, x_hat_minus_previous[i][lineno - 1]))))
                    P_previous[i].append(np.dot((np.eye(2) - np.dot(K_previous[i][lineno - 1], H)), P_minus_previous[i][lineno - 1]))

                    filtered_positions_previous[i].append(x_hat_previous[i][lineno - 1][0])

                    measurements_previous[i].append(z2)

                    if (filtered_positions_current[i][lineno-1] - filtered_positions_previous[i][lineno-1] > threshold) | (filtered_positions_current[i][lineno-1] - filtered_positions_previous[i][lineno-1] < -threshold):
                        tag = 1
                if tag==1:
                    print ('instruct movement \n')
                else: 
                    print('no change \n')
                    
            prev = tmp_float  # previous flow of data
            print (tag)
            

            if tag == 1:
                if (filtered_positions_current[2][lineno -1]<-0.349):
                    filtered_positions_current[2][lineno -1] = -0.348
                if (filtered_positions_current[2][lineno -1]>0.349):
                    filtered_positions_current[2][lineno -1] = 0.348
                if (filtered_positions_current[3][lineno -1]<-0.262):
                    filtered_positions_current[3][lineno -1] = -0.261
                if (filtered_positions_current[3][lineno -1]>1.571):
                    filtered_positions_current[3][lineno -1] = 1.570
                if (filtered_positions_current[4][lineno -1]<-0):
                    filtered_positions_current[4][lineno -1] = 0.001
                if (filtered_positions_current[4][lineno -1]>1.571):
                    filtered_positions_current[4][lineno -1] = 1.570
                if (filtered_positions_current[5][lineno -1]<0):
                    filtered_positions_current[5][lineno -1] = 0.001
                if (filtered_positions_current[5][lineno -1]>1.571):
                    filtered_positions_current[5][lineno -1] = 1.570
                if (filtered_positions_current[6][lineno -1]<-0.349):
                    filtered_positions_current[6][lineno -1] = -0.348
                if (filtered_positions_current[6][lineno -1]>0.349):
                    filtered_positions_current[6][lineno -1] = 0.348
                if (filtered_positions_current[7][lineno -1]<-0.262):
                    filtered_positions_current[7][lineno -1] = -0.261
                if (filtered_positions_current[7][lineno -1]>1.571):
                    filtered_positions_current[7][lineno -1] = 1.570
                if (filtered_positions_current[8][lineno -1]<0):
                    filtered_positions_current[8][lineno -1] = 0.001
                if (filtered_positions_current[8][lineno -1]>1.571):
                    filtered_positions_current[8][lineno -1] = 1.570
                if (filtered_positions_current[9][lineno -1]<0):
                    filtered_positions_current[9][lineno -1] = 0.001
                if (filtered_positions_current[9][lineno -1]>1.571):
                    filtered_positions_current[9][lineno -1] = 1.570
                if (filtered_positions_current[10][lineno -1]<-0.349):
                    filtered_positions_current[10][lineno -1] = -0.348
                if (filtered_positions_current[10][lineno -1]>0.349):
                    filtered_positions_current[10][lineno -1] = 0.348
                if (filtered_positions_current[11][lineno -1]<-0.262):
                    filtered_positions_current[11][lineno -1] = -0.261
                if (filtered_positions_current[11][lineno -1]>1.571):
                    filtered_positions_current[11][lineno -1] = 1.570
                if (filtered_positions_current[12][lineno -1]<0):
                    filtered_positions_current[12][lineno -1] = 0.001
                if (filtered_positions_current[12][lineno -1]>1.571):
                    filtered_positions_current[12][lineno -1] = 1.570
                if (filtered_positions_current[13][lineno -1]<0):
                    filtered_positions_current[13][lineno -1] = 0.001
                if (filtered_positions_current[13][lineno -1]>1.571):
                    filtered_positions_current[13][lineno -1] = 1.570
                if (filtered_positions_current[14][lineno -1]<-0.349):
                    filtered_positions_current[14][lineno -1] = -0.348
                if (filtered_positions_current[14][lineno -1]>0.349):
                    filtered_positions_current[14][lineno -1] = 0.348
                if (filtered_positions_current[15][lineno -1]<-0.262):
                    filtered_positions_current[15][lineno -1] = -0.261
                if (filtered_positions_current[15][lineno -1]>1.571):
                    filtered_positions_current[15][lineno -1] = 1.570
                if (filtered_positions_current[16][lineno -1]<0):
                    filtered_positions_current[16][lineno -1] = 0.001
                if (filtered_positions_current[16][lineno -1]>1.571):
                    filtered_positions_current[16][lineno -1] = 1.570
                if (filtered_positions_current[17][lineno -1]<0):
                    filtered_positions_current[17][lineno -1] = 0.001
                if (filtered_positions_current[17][lineno -1]>1.571):
                    filtered_positions_current[17][lineno -1] = 1.570
                if (filtered_positions_current[18][lineno -1]<0):
                    filtered_positions_current[18][lineno -1] = 0.001
                if (filtered_positions_current[18][lineno -1]>1.222):
                    filtered_positions_current[18][lineno -1] = 1.221
                if (filtered_positions_current[19][lineno -1]<-1.047):
                    filtered_positions_current[19][lineno -1] = -1.046
                if (filtered_positions_current[19][lineno -1]>1.047):
                    filtered_positions_current[19][lineno -1] = 1.046
                if (filtered_positions_current[20][lineno -1]<-0.698):
                    filtered_positions_current[20][lineno -1] = -0.697
                if (filtered_positions_current[20][lineno -1]>0.698):
                    filtered_positions_current[20][lineno -1] = 0.697
                if (filtered_positions_current[21][lineno -1]<-0.262):
                    filtered_positions_current[21][lineno -1] = -0.261
                if (filtered_positions_current[21][lineno -1]>1.571):
                    filtered_positions_current[21][lineno -1] = 1.570
                # print(filtered_positions_current)
                    
                joint_states = {'rh_FFJ1':filtered_positions_current[5][lineno -1], 'rh_FFJ2':filtered_positions_current[4][lineno -1], 'rh_FFJ3':filtered_positions_current[3][lineno -1], 'rh_FFJ4':filtered_positions_current[2][lineno -1],
                'rh_MFJ1':filtered_positions_current[9][lineno -1], 'rh_MFJ2':filtered_positions_current[8][lineno -1], 'rh_MFJ3':filtered_positions_current[7][lineno -1], 'rh_MFJ4':filtered_positions_current[6][lineno -1],
                'rh_RFJ1':filtered_positions_current[13][lineno -1], 'rh_RFJ2':filtered_positions_current[12][lineno -1], 'rh_RFJ3':filtered_positions_current[11][lineno -1], 'rh_RFJ4':filtered_positions_current[10][lineno -1],
                'rh_LFJ1':filtered_positions_current[17][lineno -1], 'rh_LFJ2':filtered_positions_current[16][lineno -1], 'rh_LFJ3':filtered_positions_current[15][lineno -1], 'rh_LFJ4':filtered_positions_current[14][lineno -1],  'rh_LFJ5':0.0,
                'rh_THJ1':filtered_positions_current[21][lineno -1], 'rh_THJ2':filtered_positions_current[20][lineno -1], 'rh_THJ3':0.00, 'rh_THJ4':filtered_positions_current[18][lineno -1],  'rh_THJ5':filtered_positions_current[19][lineno -1], 
                'rh_WRJ1':0.0, 'rh_WRJ2':0.0}
                
                hand_commander.move_to_joint_value_target_unsafe(joint_states, time = 0.05, wait=True, angle_degrees=False)	


            # Plot filtered position in real time
#             plt.clf()
#             plt.plot(range(len(measurements_current)), measurements_current, 'b-', label='Measurements')
#             plt.plot(range(len(filtered_positions_current)), filtered_positions_current, 'r-', label='Filtered Position')
#             plt.xlabel('Time')
#             plt.ylabel('Position')
#             plt.title('Kalman Filter Motion Prediction (Real Time)')
#             plt.legend()
#             plt.pause(0.1)  # Pause to update plot in real time


plt.show()
