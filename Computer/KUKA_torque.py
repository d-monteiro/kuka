import socket
import time

# PART 1: Sending "Get_State" and "App_Start" commands
# Create a UDP socket
my_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the PC's port
my_socket.bind(("0.0.0.0", 30333))

# Robot address (adjust IP accordingly)
robot_address = ("172.31.1.147", 30300)

# Get robot state
current_time = int(time.time() * 1000)  # Current time in milliseconds
msg = f"{current_time};1;Get_State;true".encode("utf-8")
my_socket.sendto(msg, robot_address)
print(msg)
# Receive answer state message
receive_data = bytearray(508)
receive_packet, _ = my_socket.recvfrom(508)
print("msg received")
# Extract counter
state_message = receive_packet.decode("utf-8").split(";")
counter = int(state_message[2])
print(counter)
# Start application by sending a rising edge (false -> true) for App_Start
# First message (false)
current_time = int(time.time() * 1000)
msg = f"{current_time};{counter};App_Start;false".encode("utf-8")
my_socket.sendto(msg, robot_address)
print(msg)
# Second message (true)
counter += 1
current_time = int(time.time() * 1000)
msg = f"{current_time};{counter};App_Start;true".encode("utf-8")
my_socket.sendto(msg, robot_address)
print(msg)

my_socket.close()
print("Init Socket closed.")
# PART 2: Receiving and saving torque values
print("Finished sending commands. Waiting for torque values...")

# Create a UDP socket
torque_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the PC's port
torque_socket.bind(("0.0.0.0", 30001))

# Robot address (adjust IP accordingly)
robot_address = ("172.31.1.147", 30001)

# Open a file to save the torque values
with open("torque_values.txt", "a") as file:
    try:
        while True:
            # Receive torque values from the robot
            receive_data, addr = torque_socket.recvfrom(1024) # changed from 508 to 1024 to not overflow the buffer
            torque_values = receive_data.decode("utf-8")

            # Print and save the received torque values
            print(f"Received: {torque_values}")
            file.write(torque_values + "\n")
            file.flush()  # Ensure data is written immediately

            # Control the reception rate (optional, e.g., 10 Hz)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Terminating program...")
    finally:
        # Close the socket
        torque_socket.close()
        print("Torque Socket closed.")
