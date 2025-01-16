import pandas as pd
import matplotlib.pyplot as plt

# Load PID log data
def load_pid_log(file_path):
    """Load PID data from a CSV file into a Pandas DataFrame."""
    columns = ["Timestamp", "Error", "P_Term", "I_Term", "D_Term", "Control_Output"]
    return pd.read_csv(file_path, names=columns, skiprows=1)

# Load pose log data
def load_pose_log(file_path):
    """Load pose data from a CSV file into a Pandas DataFrame."""
    columns = ["Timestamp", "X", "Y", "Z", "Orientation_Z", "Orientation_W"]
    return pd.read_csv(file_path, names=columns, skiprows=1)

# Plot the PID data
def plot_pid_data(df, filename):
    """Plot the PID components and control output."""
    plt.figure(figsize=(12, 8))

    # Plot error over time
    plt.subplot(2, 2, 1)
    plt.plot(df["Timestamp"], df["Error"], label="Error", color="blue")
    plt.title("Error Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.grid()
    plt.legend()

    # Plot proportional, integral, and derivative terms
    plt.subplot(2, 2, 2)
    plt.plot(df["Timestamp"], df["P_Term"], label="P_Term", color="green")
    plt.plot(df["Timestamp"], df["I_Term"], label="I_Term", color="orange")
    plt.plot(df["Timestamp"], df["D_Term"], label="D_Term", color="red")
    plt.title("PID Terms Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("PID Term Values")
    plt.grid()
    plt.legend()

    # Plot control output
    plt.subplot(2, 1, 2)
    plt.plot(df["Timestamp"], df["Control_Output"], label="Control Output", color="purple")
    plt.title("Control Output Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Output")
    plt.grid()
    plt.legend()

    # Adjust layout and save the plot
    plt.tight_layout()
    plt.savefig(filename)

# Plot the robot's pose data
def plot_pose_data(df):
    """Plot the robot's trajectory (X vs. Y) and orientation over time."""
    plt.figure(figsize=(12, 8))

    # Plot X vs. Y trajectory
    plt.subplot(2, 1, 1)
    plt.plot(df["X"], df["Y"], label="Trajectory", color="blue")
    plt.title("Robot Trajectory (X vs. Y)")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.grid()
    plt.legend()

    # Plot orientation (Z and W components of quaternion)
    plt.subplot(2, 1, 2)
    plt.plot(df["Timestamp"], df["Orientation_Z"], label="Orientation Z", color="red")
    plt.plot(df["Timestamp"], df["Orientation_W"], label="Orientation W", color="orange")
    plt.title("Orientation Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Orientation (Quaternion)")
    plt.grid()
    plt.legend()

    # Adjust layout and save the plot
    plt.tight_layout()
    plt.savefig("outputs/pose_plot.png")


def main():
    # File paths to the logs
    pid_log_file_path = "outputs/lin_pid_log.csv"
    pid_ang_log_file_path = "outputs/ang_pid_log.csv"
    pose_log_file_path = "outputs/pose_log.csv"

    # Load and plot the PID data
    pid_df = load_pid_log(pid_log_file_path)
    plot_pid_data(pid_df, "outputs/lin_pid_plot.png")

    pid_ang_df = load_pid_log(pid_ang_log_file_path)
    plot_pid_data(pid_ang_df, "outputs/ang_pid_plot.png")

    # Load and plot the pose data
    pose_df = load_pose_log(pose_log_file_path)
    plot_pose_data(pose_df)

if __name__ == "__main__":
    main()
