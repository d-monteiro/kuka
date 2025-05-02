import matplotlib.pyplot as plt

# File containing the data
filename = "torque_values.txt"

# Read the file and process the data
data = []
with open(filename, "r") as file:
    for line in file:
        # Split the line into values and convert them to floats
        values = list(map(float, line.strip().split(";")))
        data.append(values)

# Transpose the data to group by columns
# Each column represents one of the 7 values across all rows
data = list(zip(*data))

# Plot each column
plt.figure(figsize=(10, 6))
for i, column in enumerate(data):
    plt.plot(column, label=f"Axis {i+1}")

# Add labels, legend, and grid
plt.title("Torque Data Plot")
plt.xlabel("Time")
plt.ylabel("Torque Values")
plt.legend()
plt.grid(True)

# Show the plot
plt.tight_layout()
plt.show()
