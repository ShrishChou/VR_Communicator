import pybullet as p
import pybullet_data
import numpy as np

# Initialize simulation
physicsClient = p.connect(p.GUI)  # Or p.DIRECT for headless
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

robot_urdf_path = "/output.urdf"  # Make sure this path is correct
robot_start_pos = [0, 0, 0]
robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
robot_id = p.loadURDF(robot_urdf_path,
                   basePosition=robot_start_pos,
                   baseOrientation=robot_start_orientation,
                   useFixedBase=True)  # Fix the base of the robot
# Define IK parameters
num_joints = p.getNumJoints(robot_id)
end_effector_index=7
# Target position and orientation (Y-down)
target_pos = [3, 3, 3]  # x,y,z in meters
target_orientation = p.getQuaternionFromEuler([0, np.pi, 0])  # Pitch=π for Y-down

# Calculate IK
joint_angles = p.calculateInverseKinematics(
    robot_id,
    end_effector_index,
    target_pos,
    target_orientation,
    maxNumIterations=100
)

print(f"Joint angles (radians): {joint_angles}")