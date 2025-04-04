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
        self.arm_joints = list(range(1, 6))  # Joints 1-5 for arm positioning
        self.wrist_joint = 6     # Last joint for final adjustment
        
        # Network setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host, port))
        self.server_socket.listen(1)
        print(f"PyBullet IK Server running on {host}:{port}")

    def calculate_base_rotation(self, x, y):
        """Improved base joint angle calculation with quadrant handling"""
        new_angle = math.atan2(y, x)  # Returns angle in radians between -π and π
        new_angle_deg = math.degrees(new_angle) + 180  # Convert to 0-360 range
        
        # Determine quadrant and adjust angle if needed
        quad = 0
        if x >= 0 and y >= 0:
            quad = 1
        elif x < 0 and y >= 0:
            quad = 2
            new_angle_deg = 280  # Special case for quadrant 2
        elif x < 0 and y < 0:
            quad = 3
        else:
            quad = 4
        
        print(f"Target XY: ({x:.2f}, {y:.2f}) | Quadrant: {quad} | Base angle: {new_angle_deg:.1f}°")
        return math.radians(new_angle_deg)  # Convert back to radians for PyBullet

    def calculate_arm_ik(self, target_pos, base_angle):
        """Calculate IK for arm joints only with fixed base"""
        # Lock the base joint at calculated angle
        p.resetJointState(self.robot_id, self.base_joint, base_angle)
        
        # Target orientation (Y-down)
        target_orientation = p.getQuaternionFromEuler([0, -math.pi, 0])
        
        # Calculate IK for arm joints only
        joint_angles = p.calculateInverseKinematics(
            self.robot_id,
            self.wrist_joint,  # Target wrist position
            target_pos,
            targetOrientation=target_orientation,
            lowerLimits=[-math.pi]*self.num_joints,
            upperLimits=[math.pi]*self.num_joints,
            residualThreshold=1e-5,
            maxNumIterations=200
        )
        
        return joint_angles[1:6]  # Return only arm joints (1-5)

    def execute_movement(self, joint_angles, duration=1.0):
        """Smooth movement with interpolation"""
        steps = int(duration * 240)  # 240 Hz simulation
        if steps == 0:
            steps = 1
        
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
            time.sleep(1./240.)

    def handle_client(self, conn):
        try:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                
                try:
                    target = json.loads(data.decode())
                    x, y, z = target['position']
                    height_above = 0.1  # 10cm above target
                    
                    # STAGE 1: Rotate base using improved angle calculation
                    base_angle = self.calculate_base_rotation(x, y)
                    self.execute_movement(
                        [base_angle] + [0]*(self.num_joints-1),
                        duration=0.5
                    )
                    
                    # STAGE 2: Move arm above target
                    target_pos = [-x, z, y + height_above]
                    arm_angles = self.calculate_arm_ik(target_pos, base_angle)
                    
                    # Combine base + arm angles (keep wrist at current angle)
                    current_angles = [p.getJointState(self.robot_id, i)[0] for i in range(self.num_joints)]
                    target_angles = [
                        base_angle,
                        *arm_angles,
                        current_angles[self.wrist_joint]
                    ]
                    self.execute_movement(target_angles, duration=1.0)
                    
                    # STAGE 3: Optional wrist adjustment
                    if 'rotation' in target:
                        current_angles = [p.getJointState(self.robot_id, i)[0] for i in range(self.num_joints)]
                        target_angles = [
                            current_angles[self.base_joint],
                            *[current_angles[i] for i in self.arm_joints],
                            target['rotation']
                        ]
                        self.execute_movement(target_angles, duration=0.3)
                    
                    # Get final state
                    final_angles = [p.getJointState(self.robot_id, i)[0] for i in range(self.num_joints)]
                    
                    # Send response
                    conn.sendall(json.dumps({
                        'status': 'success',
                        'base_angle': float(base_angle),
                        'arm_angles': [float(a) for a in arm_angles],
                        'final_angles': [float(a) for a in final_angles],
                        'position': target_pos
                    }).encode())
                    
                except Exception as e:
                    print(f"Error: {str(e)}")
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