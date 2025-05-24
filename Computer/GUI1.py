import tkinter as tk
from tkinter import ttk, messagebox
import socket
import time
import threading
import queue


class KUKAControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("KUKA Rehabilitation Robot Control")
        self.root.geometry("600x700")
        self.root.configure(bg='#f0f0f0')

        # Network configuration
        self.robot_ip = "172.31.1.147"
        self.robot_port = 30300
        self.torque_port = 30001
        self.pc_port = 30333
        self.motion_port = 30002

        # Communication variables
        self.socket = None
        self.torque_socket = None
        self.counter = 0
        self.motion_active = False
        self.receiving_torque = False

        # Queues for thread communication
        self.torque_queue = queue.Queue()

        # Create GUI elements
        self.create_widgets()

        # Start torque monitoring
        self.start_torque_monitoring()

    def create_widgets(self):
        # Title
        title_label = tk.Label(self.root, text="KUKA iiwa Rehabilitation Control",
                               font=("Arial", 16, "bold"), bg='#f0f0f0')
        title_label.pack(pady=10)

        # Connection frame
        conn_frame = tk.LabelFrame(self.root, text="Connection", font=("Arial", 12, "bold"))
        conn_frame.pack(fill="x", padx=20, pady=5)

        self.connect_btn = tk.Button(conn_frame, text="Connect to Robot",
                                     command=self.connect_robot, bg='#4CAF50', fg='white',
                                     font=("Arial", 10, "bold"))
        self.connect_btn.pack(side=tk.LEFT, padx=10, pady=10)

        self.status_label = tk.Label(conn_frame, text="Status: Disconnected",
                                     font=("Arial", 10), bg='#f0f0f0')
        self.status_label.pack(side=tk.LEFT, padx=10)

        # Control frame
        control_frame = tk.LabelFrame(self.root, text="Motion Control", font=("Arial", 12, "bold"))
        control_frame.pack(fill="x", padx=20, pady=10)

        # Start motion button
        self.start_btn = tk.Button(control_frame, text="START MOTION",
                                   command=self.start_motion, bg='#2196F3', fg='white',
                                   font=("Arial", 14, "bold"), height=2, width=15)
        self.start_btn.pack(side=tk.LEFT, padx=10, pady=10)

        # Emergency stop button
        self.stop_btn = tk.Button(control_frame, text="EMERGENCY STOP",
                                  command=self.emergency_stop, bg='#f44336', fg='white',
                                  font=("Arial", 14, "bold"), height=2, width=15)
        self.stop_btn.pack(side=tk.RIGHT, padx=10, pady=10)

        # Stiffness settings frame
        stiffness_frame = tk.LabelFrame(self.root, text="Impedance Settings",
                                        font=("Arial", 12, "bold"))
        stiffness_frame.pack(fill="x", padx=20, pady=10)

        # Create stiffness controls
        stiffness_controls = [
            ("X Stiffness (N/m):", "stiffness_x", 300, 100, 1000),
            ("Y Stiffness (N/m):", "stiffness_y", 300, 100, 1000),
            ("Z Stiffness (N/m):", "stiffness_z", 300, 100, 1000),
            ("Rotation Stiffness (Nm/rad):", "stiffness_rot", 200, 50, 500)
        ]

        self.stiffness_vars = {}

        for i, (label, var_name, default, min_val, max_val) in enumerate(stiffness_controls):
            row = i // 2
            col = i % 2

            frame = tk.Frame(stiffness_frame)
            frame.grid(row=row, column=col, padx=10, pady=5, sticky="w")

            tk.Label(frame, text=label, font=("Arial", 9)).pack(anchor="w")

            var = tk.DoubleVar(value=default)
            self.stiffness_vars[var_name] = var

            scale = tk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                             variable=var, length=150, command=self.update_stiffness)
            scale.pack()

            entry = tk.Entry(frame, textvariable=var, width=8, font=("Arial", 9))
            entry.pack()

        # Apply stiffness button
        apply_btn = tk.Button(stiffness_frame, text="Apply Stiffness Settings",
                              command=self.apply_stiffness, bg='#FF9800', fg='white',
                              font=("Arial", 10, "bold"))
        apply_btn.grid(row=2, column=0, columnspan=2, pady=10)

        # Torque monitoring frame
        torque_frame = tk.LabelFrame(self.root, text="Real-time Torque Monitoring",
                                     font=("Arial", 12, "bold"))
        torque_frame.pack(fill="both", expand=True, padx=20, pady=10)

        # Torque display
        self.torque_text = tk.Text(torque_frame, height=8, font=("Courier", 9))
        self.torque_text.pack(fill="both", expand=True, padx=5, pady=5)

        scrollbar = tk.Scrollbar(torque_frame, command=self.torque_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.torque_text.config(yscrollcommand=scrollbar.set)

        # Initially disable control buttons
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='disabled')

    def connect_robot(self):
        """Connect to the robot and get initial state"""
        try:
            # Create UDP socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind(("0.0.0.0", self.pc_port))

            robot_address = (self.robot_ip, self.robot_port)

            # Get robot state
            current_time = int(time.time() * 1000)
            msg = f"{current_time};1;Get_State;true".encode("utf-8")
            self.socket.sendto(msg, robot_address)

            # Receive state response
            self.socket.settimeout(5.0)  # 5 second timeout
            receive_packet, _ = self.socket.recvfrom(508)

            # Parse response to get counter
            state_message = receive_packet.decode("utf-8").split(";")
            self.counter = int(state_message[2])

            # Start the application
            self.start_application(robot_address)

            # Update UI
            self.status_label.config(text="Status: Connected", fg='green')
            self.connect_btn.config(text="Connected", state='disabled', bg='gray')
            self.start_btn.config(state='normal')
            self.stop_btn.config(state='normal')

            self.log_torque("Successfully connected to robot")

        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect to robot: {str(e)}")
            self.log_torque(f"Connection failed: {str(e)}")

    def start_application(self, robot_address):
        """Send App_Start command to robot"""
        try:
            # Send rising edge for App_Start (false -> true)
            current_time = int(time.time() * 1000)
            msg = f"{current_time};{self.counter};App_Start;false".encode("utf-8")
            self.socket.sendto(msg, robot_address)

            self.counter += 1
            current_time = int(time.time() * 1000)
            msg = f"{current_time};{self.counter};App_Start;true".encode("utf-8")
            self.socket.sendto(msg, robot_address)

            self.log_torque("Application started on robot")

        except Exception as e:
            self.log_torque(f"Failed to start application: {str(e)}")

    def start_motion(self):
        """Start the rehabilitation motion"""
        if not self.motion_active:
            try:
                # Send START_MOTION command to robot using existing socket
                if self.socket:
                    robot_address = (self.robot_ip, self.motion_port)
                    current_time = int(time.time() * 1000)
                    self.counter += 1

                    # Send motion start command
                    msg = f"{current_time};{self.counter};START_MOTION;true".encode("utf-8")
                    self.socket.sendto(msg, robot_address)

                    self.motion_active = True
                    self.start_btn.config(state='disabled', text="MOTION ACTIVE")
                    self.log_torque("MOTION STARTED - Command sent to robot")
                else:
                    raise Exception("Not connected to robot")

            except Exception as e:
                messagebox.showerror("Motion Error", f"Failed to start motion: {str(e)}")
                self.log_torque(f"Motion start failed: {str(e)}")

    def emergency_stop(self):
        """Emergency stop the robot motion"""
        try:
            # Send emergency stop command
            if self.socket:
                robot_address = (self.robot_ip, self.robot_port)
                current_time = int(time.time() * 1000)
                self.counter += 1
                msg = f"{current_time};{self.counter};Emergency_Stop;true".encode("utf-8")
                self.socket.sendto(msg, robot_address)

            self.motion_active = False
            self.start_btn.config(state='normal', text="START MOTION")
            self.log_torque("EMERGENCY STOP ACTIVATED")

            messagebox.showwarning("Emergency Stop", "Robot motion has been stopped immediately!")

        except Exception as e:
            self.log_torque(f"Emergency stop failed: {str(e)}")

    def update_stiffness(self, value=None):
        """Called when stiffness sliders are moved"""
        pass  # Real-time update if needed

    def apply_stiffness(self):
        """Apply new stiffness settings to the robot"""
        try:
            stiffness_x = self.stiffness_vars['stiffness_x'].get()
            stiffness_y = self.stiffness_vars['stiffness_y'].get()
            stiffness_z = self.stiffness_vars['stiffness_z'].get()
            stiffness_rot = self.stiffness_vars['stiffness_rot'].get()

            # In a full implementation, send these values to the robot
            # Example: self.send_stiffness_update(stiffness_x, stiffness_y, stiffness_z, stiffness_rot)

            self.log_torque(
                f"Stiffness updated: X={stiffness_x}, Y={stiffness_y}, Z={stiffness_z}, Rot={stiffness_rot}")
            messagebox.showinfo("Settings Applied", "Stiffness settings have been updated")

        except Exception as e:
            messagebox.showerror("Settings Error", f"Failed to apply settings: {str(e)}")

    def start_torque_monitoring(self):
        """Start the torque data monitoring thread"""
        self.torque_thread = threading.Thread(target=self.receive_torque_data, daemon=True)
        self.torque_thread.start()

        # Start GUI update timer
        self.root.after(100, self.update_torque_display)

    def receive_torque_data(self):
        """Receive torque data from robot (runs in separate thread)"""
        try:
            self.torque_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.torque_socket.bind(("0.0.0.0", self.torque_port))
            self.receiving_torque = True

            while self.receiving_torque:
                try:
                    data, addr = self.torque_socket.recvfrom(1024)
                    torque_data = data.decode("utf-8")

                    # Parse torque data
                    parts = torque_data.split(';')
                    if len(parts) >= 7:  # Expecting status + 7 joint torques
                        status = parts[0] if len(parts) > 7 else "UNKNOWN"
                        torques = parts[-7:] if len(parts) > 7 else parts

                        # Format for display
                        formatted_data = f"[{time.strftime('%H:%M:%S')}] Status: {status}"
                        for i, torque in enumerate(torques):
                            try:
                                torque_val = float(torque)
                                formatted_data += f" | J{i + 1}: {torque_val:6.3f}"
                            except ValueError:
                                pass

                        # Add to queue for GUI update
                        self.torque_queue.put(formatted_data)

                except socket.timeout:
                    continue
                except Exception as e:
                    if self.receiving_torque:  # Only log if we're still supposed to be receiving
                        self.torque_queue.put(f"Torque reception error: {str(e)}")

        except Exception as e:
            self.torque_queue.put(f"Failed to start torque monitoring: {str(e)}")

    def update_torque_display(self):
        """Update the torque display (runs in main GUI thread)"""
        try:
            while not self.torque_queue.empty():
                torque_data = self.torque_queue.get_nowait()
                self.log_torque(torque_data)
        except queue.Empty:
            pass

        # Schedule next update
        self.root.after(100, self.update_torque_display)

    def log_torque(self, message):
        """Add message to torque display"""
        self.torque_text.insert(tk.END, message + "\n")
        self.torque_text.see(tk.END)

        # Keep only last 100 lines
        lines = self.torque_text.get("1.0", tk.END).split("\n")
        if len(lines) > 100:
            self.torque_text.delete("1.0", f"{len(lines) - 100}.0")

    def on_closing(self):
        """Clean up when closing the application"""
        self.receiving_torque = False

        if self.socket:
            self.socket.close()
        if self.torque_socket:
            self.torque_socket.close()

        self.root.destroy()


def main():
    root = tk.Tk()
    app = KUKAControlGUI(root)

    # Handle window closing
    root.protocol("WM_DELETE_WINDOW", app.on_closing)

    root.mainloop()


if __name__ == "__main__":
    main()