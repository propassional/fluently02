import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('trj_smooth.csv')

# Create a figure and a set of subplots
fig, axes = plt.subplots(nrows=6, ncols=1, figsize=(10, 12), sharex=True)  # 6 rows, 1 column
joint_limits=np.array([360, 360, 540, 380, 360, 450])/2.005
# print(joint_limits)
# exit()
joint_limits=np.deg2rad(joint_limits)


# Adjust the spacing between subplots
plt.subplots_adjust(hspace=0.5)

# Iterate through the columns and plot each in a separate subplot
for i, column in enumerate(df.columns):
     # Clip the values to the joint limits
    df[column] = np.clip(df[column], -joint_limits[i], joint_limits[i])

    # axes[i].plot(np.diff(df[column]))
    axes[i].plot(df[column])
    axes[i].set_ylabel(f'Joint {i+1}')  # Label each subplot with the joint number
    axes[i].grid(True)
    axes[i].set_title(f'Joint {i+1} Trajectory')

    # Add horizontal lines for upper and lower limits
    axes[i].axhline(y=joint_limits[i], color='r', linestyle='--', label='Upper Limit')
    axes[i].axhline(y=-joint_limits[i], color='r', linestyle='--', label='Lower Limit')
    
    # Add legend to the first subplot only
    if i == 0:
        axes[i].legend()

# Set common labels for the entire figure
fig.suptitle('Joint Trajectories Over Time')
axes[-1].set_xlabel('Row Index')  # Label only the last subplot's x-axis

# Show the plot
plt.show()
