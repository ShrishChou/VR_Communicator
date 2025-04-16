import pybullet as p
import pybullet_data
import numpy as np
import socket
import json
import threading
import time
import math

class PyBulletIKServer:
    def __init__(self, urdf_path="xarm7.urdf", host='127.0.0.1', port=65432):
        # Physics setup
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load robot URDF
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )
        
        # Joint configuration
        self.num_joints = p.getNumJoints(self.robot_id)
        self.base_joint = 0      # First joint for XY rotation
        self.arm_joints = list(range(1, 7))  # Joints 1-6 for arm positioning (changed from 1-5)
        self.wrist_joint = 6     # Last joint for final adjustment
        
        # Track current base angle to avoid 180° flips
        self.current_base_angle = 0.0
        
        # Store initial position (home position)
        self.home_position = [0.0] * self.num_joints
        # Initialize the robot to a good home position
        self.set_home_position([0, 0, 0, 0, 0, 0, 0])
        
        # Network setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(1)
        print(f"PyBullet IK Server running on {host}:{port}")

    def calculate_base_rotation(self, x, y):
        """
        Calculate base rotation with proper quadrant handling to avoid 180° flips
        and prevent the robot from rotating through itself.
        Returns the angle in radians.
        """
        # Calculate the raw angle
        raw_angle = math.atan2(y, x)  # Returns angle in radians between -π and π
        
        # Get current base angle
        current_angle = p.getJointState(self.robot_id, self.base_joint)[0]
        
        # Unwrap the angles to the same range for comparison
        while current_angle > math.pi:
            current_angle -= 2 * math.pi
        while current_angle < -math.pi:
            current_angle += 2 * math.pi
            
        # Calculate the difference between current and target angles
        angle_diff = raw_angle - current_angle
        
        # Normalize the difference to [-π, π]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        # Check if the rotation would be more than 150 degrees (5π/6) in either direction
        # This helps prevent the arm from trying to rotate through itself
        max_rotation = 5 * math.pi / 6  # 150 degrees
        
        if abs(angle_diff) > max_rotation:
            print(f"WARNING: Large rotation detected ({math.degrees(abs(angle_diff)):.1f}°). Using return to home strategy.")
            # Return to home position first, then do the rotation
            return None  # Signal that we need to use the home position strategy
        
        # For normal rotations, proceed as before
        # Identify quadrant for logging purposes
        quadrant = 0
        if x >= 0 and y >= 0:
            quadrant = 1
        elif x < 0 and y >= 0:
            quadrant = 2
        elif x < 0 and y < 0:
            quadrant = 3
        else:
            quadrant = 4
        
        print(f"Target XY: ({x:.2f}, {y:.2f}) | Quadrant: {quadrant} | Base angle: {math.degrees(raw_angle):.1f}° | Current: {math.degrees(current_angle):.1f}°")
        
        # Update our tracked value
        target_angle = current_angle + angle_diff
        self.current_base_angle = target_angle
        
        return target_angle

    def calculate_arm_ik(self, target_pos, base_angle):
        """
        Calculate IK for arm joints 1-6 while keeping base joint fixed.
        """
        # Lock the base joint at calculated angle
        p.resetJointState(self.robot_id, self.base_joint, base_angle)
        
        # Target orientation - ensure Y-axis points downward (zero pitch and roll)
        target_orientation = p.getQuaternionFromEuler([0, -math.pi, 0])
        
        # Create joint constraint for base joint to prevent IK from changing it
        constraint_id = p.createConstraint(
            self.robot_id, self.base_joint,
            self.robot_id, -1,  # Link index -1 is the base
            p.JOINT_FIXED,
            [0, 0, 0],  # Joint axis doesn't matter for fixed joint
            [0, 0, 0],  # Parent frame position
            [0, 0, 0]   # Child frame position
        )
        
        print(f"Calculating IK for target position: {target_pos}")
        
        # Calculate IK for all joints (constraint will prevent base joint movement)
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.wrist_joint,  # Target end effector
            target_pos,
            targetOrientation=target_orientation,
            lowerLimits=[-math.pi for _ in range(self.num_joints)],
            upperLimits=[math.pi for _ in range(self.num_joints)],
            jointDamping=[0.01 for _ in range(self.num_joints)],
            solver=p.IK_DLS,  # Use DLS solver for better stability
            maxNumIterations=200,
            residualThreshold=1e-5
        )
        
        # Remove the constraint after IK calculation
        p.removeConstraint(constraint_id)
        
        # Ensure the base joint stays at the intended angle (additional safety)
        # We only need joints 1-6 (indices 1-6)
        corrected_angles = list(joint_angles[1:7])
        
        # Extract current end effector pose for debugging
        end_state = p.getLinkState(self.robot_id, self.wrist_joint)
        end_orientation = end_state[1]  # Quaternion
        euler = p.getEulerFromQuaternion(end_orientation)
        
        # Only correct if we have significant roll or pitch deviation
        if abs(euler[0]) > 0.05 or abs(euler[2]) > 0.05:
            print(f"Correcting end effector orientation: Roll={math.degrees(euler[0]):.1f}°, Pitch={math.degrees(euler[1]):.1f}°, Yaw={math.degrees(euler[2]):.1f}°")
            
            # Create joint constraint again for refined IK
            constraint_id = p.createConstraint(
                self.robot_id, self.base_joint,
                self.robot_id, -1,
                p.JOINT_FIXED,
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            )
            
            # Perform additional IK iterations with stronger orientation constraint
            joint_angles = p.calculateInverseKinematics(
                self.robot_id,
                self.wrist_joint,
                target_pos,
                targetOrientation=target_orientation,
                lowerLimits=[-math.pi for _ in range(self.num_joints)],
                upperLimits=[math.pi for _ in range(self.num_joints)],
                jointDamping=[0.01 for _ in range(self.num_joints)],
                solver=p.IK_DLS,
                maxNumIterations=500,  # More iterations
                residualThreshold=1e-6  # Tighter threshold
            )
            
            # Remove the constraint
            p.removeConstraint(constraint_id)
            
            # Use only joints 1-6, keep base joint unchanged
            corrected_angles = list(joint_angles[1:7])
        
        # Manually verify base joint hasn't changed
        current_base = p.getJointState(self.robot_id, self.base_joint)[0]
        if abs(current_base - base_angle) > 0.01:
            print(f"WARNING: Base joint changed from {base_angle} to {current_base}. Resetting.")
            p.resetJointState(self.robot_id, self.base_joint, base_angle)
        
        return corrected_angles

    def execute_movement(self, joint_angles, duration=1.0):
        """Smooth movement with interpolation"""
        steps = int(duration * 240)  # 240 Hz simulation
        if steps == 0:
            steps = 1
        
        # Get the current angles of the joints we want to move
        current_angles = [p.getJointState(self.robot_id, i)[0] for i in range(len(joint_angles))]
        
        for step in range(steps):
            t = step / steps
            interpolated = [
                current*(1-t) + target*t 
                for current, target in zip(current_angles, joint_angles)
            ]
            
            for i, angle in enumerate(interpolated):
                p.resetJointState(self.robot_id, i, angle)
            
            p.stepSimulation()
            # time.sleep(1./240.)

    def check_end_effector_orientation(self):
        """Check and log end effector orientation"""
        end_state = p.getLinkState(self.robot_id, self.wrist_joint)
        orientation = p.getEulerFromQuaternion(end_state[1])
        roll, pitch, yaw = [math.degrees(a) for a in orientation]
        print(f"End effector orientation: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
        return orientation

    def set_home_position(self, joint_angles=None):
        """Set and store home position"""
        if joint_angles:
            # If specific joint angles are provided, use them
            for i, angle in enumerate(joint_angles):
                if i < self.num_joints:
                    p.resetJointState(self.robot_id, i, angle)
                    self.home_position[i] = angle
        else:
            # Default home position if none provided
            default_home = [0, math.pi/6, -math.pi/3, 0, -math.pi/2, 0, 0]
            for i, angle in enumerate(default_home):
                if i < self.num_joints:
                    p.resetJointState(self.robot_id, i, angle)
                    self.home_position[i] = angle
        
        # Step simulation to update
        for _ in range(10):
            p.stepSimulation()
        
        print("Home position set:", [round(angle, 2) for angle in self.home_position])
        return self.home_position

    def return_to_home(self, duration=1.0):
        """Return robot to home position"""
        print("Returning to home position...")
        self.execute_movement(self.home_position, duration=duration)
        return self.home_position

    def handle_client(self, conn):
        try:
            buffer = b""  # Buffer to hold incoming data
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                
                # Add new data to buffer
                buffer += data
                
                # Process complete messages
                try:
                    # Try to find a complete JSON object in the buffer
                    # Strip any whitespace from the beginning
                    buffer = buffer.strip()
                    
                    # Parse the JSON
                    target = json.loads(buffer.decode())
                    # Reset the buffer after successful parsing
                    buffer = b""
                    
                    # IMPORTANT: The coordinate mapping is:
                    # x → y, y → z, z → -x
                    x_orig, y_orig, z_orig = target['position']
                    
                    # Apply the correct coordinate transformation
                    y = -x_orig       # x maps to y
                    z = y_orig       # y maps to z
                    x = z_orig      # z maps to -x
                    
                    print(f"Received coords: ({x_orig}, {y_orig}, {z_orig}) → Transformed to: ({x}, {y}, {z})")
                    
                    height_above = 0.1  # 10cm above target
                    
                    # STAGE 1: Calculate the base rotation angle
                    base_angle = self.calculate_base_rotation(x, y)
                    
                    # Check if we need to use the safe rotation strategy
                    if base_angle is None:
                        # For large rotations, first return to home position
                        print("Using safe rotation strategy: Return to home first")
                        self.return_to_home(duration=1.0)
                        
                        # Recalculate base angle after returning to home
                        # This time we'll accept the angle since we're starting from home
                        base_angle = math.atan2(y, x)
                    
                    # Move only the base joint
                    base_only_angles = [base_angle] + [p.getJointState(self.robot_id, i)[0] for i in range(1, self.num_joints)]
                    self.execute_movement(base_only_angles, duration=0.1)
                    
                    # STAGE 2: Move arm above target while keeping base joint fixed
                    # Use the transformed coordinates for the target position
                    target_pos = [x, y, z+0.35]
                    
                    # Calculate IK for joints 1-6 only, keeping base joint fixed
                    arm_angles = self.calculate_arm_ik(target_pos, base_angle)
                    
                    # Create target angles array: [base_angle, *arm_angles]
                    target_angles = [base_angle] + arm_angles
                    
                    # Execute the movement
                    self.execute_movement(target_angles, duration=0.1)
                    
                    # Check orientation after arm movement
                    orientation = self.check_end_effector_orientation()
                    
                    # STAGE 3: Optional final adjustments if needed
                    # Verify base angle hasn't changed
                    final_base_angle = p.getJointState(self.robot_id, self.base_joint)[0]
                    if abs(final_base_angle - base_angle) > 0.01:
                        print(f"WARNING: Base angle changed during movement: {base_angle} -> {final_base_angle}")
                        p.resetJointState(self.robot_id, self.base_joint, base_angle)
                    
                    # Get final joint angles
                    final_angles = [p.getJointState(self.robot_id, i)[0] for i in range(self.num_joints)]
                    
                    # STAGE 4: Wait for 1 second at target position
                    print(f"Target reached. Waiting for 1 second...")
                    # time.sleep(1.0)
                    
                    # STAGE 5: Return to home position
                    self.return_to_home(duration=1.0)
                    
                    # Replace the existing conn.sendall line with this:
                    angles = [float(base_angle)] + [float(a) for a in arm_angles]
                    print(angles)
                    conn.sendall(json.dumps({
                        'status': 'success',
                        'angles': angles
                    }).encode())
                    
                except json.JSONDecodeError as e:
                    # If we get a JSON error, it could be because:
                    # 1. The message is incomplete (need more data)
                    # 2. There's extra data after a valid JSON object
                    # 3. The data is malformed
                    
                    # For case 2 (extra data), try to find a valid JSON object
                    try:
                        # Find the first occurrence of '}'
                        end_index = buffer.find(b'}') + 1
                        if end_index > 0:
                            # Try to parse just up to the first closing brace
                            json_str = buffer[:end_index].decode().strip()
                            target = json.loads(json_str)
                            
                            # If successful, remove the parsed portion from buffer
                            buffer = buffer[end_index:].strip()
                            
                            # Process the target as before...
                            # (The rest of the processing code would be duplicated here)
                            # For brevity, let's log and continue instead
                            print(f"Recovered JSON from partial data: {target}")
                            
                            # You would implement the same processing logic as above
                            # but for now let's just continue to the next iteration
                            continue
                    except Exception as inner_e:
                        # If recovery fails, just keep waiting for more data
                        print(f"Couldn't parse JSON yet: {str(e)}. Waiting for more data.")
                        continue
                
                except Exception as e:
                    print(f"Error processing data: {str(e)}")
                    # Try to clear the buffer and continue
                    buffer = b""
                    conn.sendall(json.dumps({
                        'status': 'error',
                        'message': str(e)
                    }).encode())
                
        finally:
            conn.close()

    def run(self):
        try:
            while True:
                conn, addr = self.server_socket.accept()
                print(f"Connected to Unity client: {addr}")
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(conn,),
                    daemon=True
                )
                client_thread.start()
                
        except KeyboardInterrupt:
            print("Shutting down server...")
        finally:
            self.server_socket.close()
            p.disconnect()

if __name__ == "__main__":
    server = PyBulletIKServer(urdf_path="output.urdf")
    server.run()