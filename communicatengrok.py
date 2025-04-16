import pybullet as p
import pybullet_data
import numpy as np
import socket
import json
import threading
import time
import math
import traceback
import datetime

class PyBulletIKServer:
    def __init__(self, urdf_path="xarm7.urdf", host='127.0.0.1', port=65432):
        # Add debug level
        self.debug_level = 2  # 0=minimal, 1=normal, 2=verbose
        self.keepalive_interval = 10 
        self.log_debug(f"Initializing PyBullet IK Server with debug level {self.debug_level}", 0)
        
        # Physics setup
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load robot URDF
        self.log_debug(f"Loading robot URDF from {urdf_path}", 1)
        self.robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )
        
        # Joint configuration
        self.num_joints = p.getNumJoints(self.robot_id)
        self.log_debug(f"Robot has {self.num_joints} joints", 1)
        self.base_joint = 0      # First joint for XY rotation
        self.arm_joints = list(range(1, 7))  # Joints 1-6 for arm positioning
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
        self.server_socket.bind(('0.0.0.0', port))
        self.server_socket.listen(1)
        self.log_debug(f"PyBullet IK Server running on {host}:{port}", 0)
        
        # Track connection and request statistics
        self.active_connections = 0
        self.total_connections = 0
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.start_time = datetime.datetime.now()

    def log_debug(self, message, level=1):
        """Log a debug message with timestamp if debug level is high enough"""
        if level <= self.debug_level:
            timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] {'  '*level}{message}")
        
    def calculate_base_rotation(self, x, y):
        """
        Calculate base rotation with proper quadrant handling to avoid 180° flips
        and prevent the robot from rotating through itself.
        Returns the angle in radians.
        """
        self.log_debug(f"Calculating base rotation for x={x:.4f}, y={y:.4f}", 1)
        
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
            self.log_debug(f"WARNING: Large rotation detected ({math.degrees(abs(angle_diff)):.1f}°). Using return to home strategy.", 0)
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
        
        self.log_debug(f"Target XY: ({x:.2f}, {y:.2f}) | Quadrant: {quadrant} | Base angle: {math.degrees(raw_angle):.1f}° | Current: {math.degrees(current_angle):.1f}°", 0)
        
        # Update our tracked value
        target_angle = current_angle + angle_diff
        self.current_base_angle = target_angle
        
        return target_angle

    def calculate_arm_ik(self, target_pos, base_angle):
        """
        Calculate IK for arm joints 1-6 while keeping base joint fixed.
        """
        self.log_debug(f"Calculating IK for target position: {target_pos}", 1)
        
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
            self.log_debug(f"Correcting end effector orientation: Roll={math.degrees(euler[0]):.1f}°, Pitch={math.degrees(euler[1]):.1f}°, Yaw={math.degrees(euler[2]):.1f}°", 1)
            
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
            self.log_debug(f"WARNING: Base joint changed from {base_angle} to {current_base}. Resetting.", 1)
            p.resetJointState(self.robot_id, self.base_joint, base_angle)
        
        return corrected_angles
    def parse_json_messages(self, data_buffer):
        """
        Parse potentially multiple JSON objects from a buffer.
        Returns a list of parsed objects and any remaining data.
        """
        self.log_debug(f"Parsing JSON messages from buffer ({len(data_buffer)} bytes)", 2)
        
        messages = []
        remaining_buffer = data_buffer
        
        # Keep trying to parse JSON objects until we can't find any more
        while remaining_buffer:
            # Strip any leading whitespace
            remaining_buffer = remaining_buffer.strip()
            
            if not remaining_buffer:
                break
            
            # Try to find a complete JSON object
            try:
                # Look for newline or other message delimiter
                delimiter_index = remaining_buffer.find(b'\n')
                
                if delimiter_index >= 0:
                    # If we found a delimiter, parse just this chunk
                    json_data = remaining_buffer[:delimiter_index].strip()
                    if json_data:  # Only process if we have actual data
                        message = json.loads(json_data.decode())
                        messages.append(message)
                    
                    # Move past this message
                    remaining_buffer = remaining_buffer[delimiter_index+1:]
                    continue
                
                # If no delimiter, try to parse the entire buffer
                message = json.loads(remaining_buffer.decode())
                messages.append(message)
                remaining_buffer = b""
                
            except json.JSONDecodeError:
                # If we can't parse the entire buffer, try to find where a valid JSON object might end
                try:
                    # Look for closing brace of an object
                    end_index = remaining_buffer.find(b'}') + 1
                    if end_index > 0:
                        # Try to parse just up to the closing brace
                        json_data = remaining_buffer[:end_index]
                        message = json.loads(json_data.decode())
                        messages.append(message)
                        
                        # Keep the rest for later
                        remaining_buffer = remaining_buffer[end_index:]
                    else:
                        # If no closing brace, this is partial data
                        self.log_debug("Incomplete JSON data, waiting for more", 2)
                        break
                except Exception:
                    # If we still can't parse, keep the buffer for more data
                    self.log_debug("Failed to parse JSON, waiting for more data", 2)
                    break
        
        return messages, remaining_buffer

# Then update the handle_client method to use this new function:
    def execute_movement(self, joint_angles, duration=1.0):
        """Smooth movement with interpolation"""
        self.log_debug(f"Executing movement to angles: {[round(a, 2) for a in joint_angles]}, duration: {duration}s", 2)
        
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
        self.log_debug(f"End effector orientation: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°", 1)
        return orientation

    def set_home_position(self, joint_angles=None):
        """Set and store home position"""
        self.log_debug("Setting home position", 1)
        
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
        
        self.log_debug("Home position set: " + str([round(angle, 2) for angle in self.home_position]), 1)
        return self.home_position

    def return_to_home(self, duration=1.0):
        """Return robot to home position"""
        self.log_debug("Returning to home position...", 1)
        self.execute_movement(self.home_position, duration=duration)
        return self.home_position

    def handle_client(self, conn, addr):
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        # Linux specific (adjust for Windows if needed)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 5)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        """Handle client connection and process requests"""
        client_id = self.total_connections
        self.active_connections += 1
        self.total_connections += 1
        
        self.log_debug(f"Client #{client_id} connected from {addr}. Active connections: {self.active_connections}", 0)
        
        # Set a timeout to detect stalled connections
        conn.settimeout(30.0)  # 30 second timeout
        
        request_count = 0
        success_count = 0
        
        try:
            buffer = b""  # Buffer to hold incoming data
            while True:
                try:
                    # Try to receive data with timeout
                    
                    self.log_debug(f"Client #{client_id}: Waiting for data...", 2)
                    data = conn.recv(4096)
                    
                    if not data:
                        self.log_debug(f"Client #{client_id}: Connection closed by client (empty data received)", 0)
                        break
                    
                    # Log the raw data received
                    self.log_debug(f"Client #{client_id}: Received {len(data)} bytes", 1)
                    if self.debug_level >= 2:
                        self.log_debug(f"Raw data: {data}", 2)
                    
                    # Add new data to buffer
                    buffer += data
                    
                    # Process complete messages
                    messages, buffer = self.parse_json_messages(buffer)
                    
                    for target in messages:
                        request_count += 1
                        self.total_requests += 1
                        
                        self.log_debug(f"Client #{client_id}: Request #{request_count} - Processing JSON: {target}", 1)
                        
                        # Check if we have a valid position in the request
                        if 'position' not in target or len(target['position']) != 3:
                            raise ValueError("Invalid position data in request")
                        
                        # IMPORTANT: The coordinate mapping is:
                        # x → y, y → z, z → -x
                        x_orig, y_orig, z_orig = target['position']
                        
                        # Apply the correct coordinate transformation
                        y = -x_orig       # x maps to y
                        z = y_orig       # y maps to z
                        x = z_orig      # z maps to -x
                        print(x,y,z,"position")
                        self.log_debug(f"Client #{client_id}: Received coords: ({x_orig}, {y_orig}, {z_orig}) → Transformed to: ({x}, {y}, {z})", 0)
                        
                        height_above = 0.04  # 10cm above target
                        
                        # STAGE 1: Calculate the base rotation angle
                        base_angle = self.calculate_base_rotation(x, y)
                        
                        # Check if we need to use the safe rotation strategy
                        if base_angle is None:
                            # For large rotations, first return to home position
                            self.log_debug("Using safe rotation strategy: Return to home first", 1)
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
                            self.log_debug(f"WARNING: Base angle changed during movement: {base_angle} -> {final_base_angle}", 1)
                            p.resetJointState(self.robot_id, self.base_joint, base_angle)
                        
                        # Get final joint angles
                        final_angles = [p.getJointState(self.robot_id, i)[0] for i in range(self.num_joints)]
                        
                        # STAGE 4: Wait for 1 second at target position
                        self.log_debug(f"Target reached. Waiting for 1 second...", 1)
                        # time.sleep(1.0)
                        
                        # STAGE 5: Return to home position
                        self.return_to_home(duration=0.1)
                        
                        # Send response with angles
                        angles = [float(base_angle)] + [float(a) for a in arm_angles]
                        response = {
                            'status': 'success',
                            'request_id': request_count,
                            'angles': angles
                        }
                        
                        response_json = json.dumps(response)
                        response_bytes = response_json.encode()
                        
                        self.log_debug(f"Client #{client_id}: Sending response ({len(response_bytes)} bytes): {response_json}", 1)
                        conn.sendall(response_bytes)
                        
                        # Explicitly flush the TCP buffer (though sendall should do this already)
                        try:
                            # Try to flush socket buffer (TCP_NODELAY option)
                            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
                        except Exception as e:
                            self.log_debug(f"Warning: Could not flush socket buffer: {str(e)}", 1)
                        
                        # Record successful request
                        success_count += 1
                        self.successful_requests += 1
                        
                        self.log_debug(f"Client #{client_id}: Request #{request_count} completed successfully", 0)
                    
                except socket.timeout:
                    self.log_debug(f"Client #{client_id}: Socket timeout while waiting for data", 1)
                    # Check if we have partial data in the buffer
                    if buffer:
                        self.log_debug(f"Client #{client_id}: Buffer has {len(buffer)} bytes of incomplete data", 1)
                    continue
                    
        except Exception as e:
            self.log_debug(f"Client #{client_id}: Error receiving data: {str(e)}", 0)
    def run(self):
        """Run the server"""
        self.log_debug("Server starting, waiting for connections...", 0)
        
        try:
            while True:
                # Accept new connections
                conn, addr = self.server_socket.accept()
                
                # Start a new thread to handle this client
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(conn, addr),
                    daemon=True
                )
                client_thread.start()
                
                # Periodically log statistics
                uptime = datetime.datetime.now() - self.start_time
                hours, remainder = divmod(uptime.total_seconds(), 3600)
                minutes, seconds = divmod(remainder, 60)
                
                self.log_debug(f"Server statistics - Uptime: {int(hours)}h {int(minutes)}m {int(seconds)}s | "
                      f"Connections: {self.total_connections} | "
                      f"Requests: {self.total_requests} | "
                      f"Success: {self.successful_requests} | "
                      f"Failed: {self.failed_requests}", 0)
                
        except KeyboardInterrupt:
            self.log_debug("Shutting down server (keyboard interrupt)...", 0)
        except Exception as e:
            self.log_debug(f"Server error: {str(e)}", 0)
            if self.debug_level >= 1:
                self.log_debug(f"Stack trace: {traceback.format_exc()}", 1)
        finally:
            # Clean up
            self.server_socket.close()
            p.disconnect()
            self.log_debug("Server shutdown complete", 0)

if __name__ == "__main__":
    server = PyBulletIKServer(urdf_path="output.urdf")
    server.run()