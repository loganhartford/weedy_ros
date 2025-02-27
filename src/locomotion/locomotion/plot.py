import pandas as pd
import matplotlib.pyplot as plt

def load_pid_log(file_path):
    columns = ["Timestamp", "Error", "P_Term", "I_Term", "D_Term", "Control_Output"]
    return pd.read_csv(file_path, names=columns, skiprows=1)

def load_pose_log(file_path):
    columns = ["Timestamp", "X", "Y", "Z", "Orientation_Z", "Orientation_W"]
    return pd.read_csv(file_path, names=columns, skiprows=1)

def plot_pid_data(df, filename):
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 2, 1)
    plt.plot(df["Timestamp"], df["Error"], label="Error", color="blue")
    plt.title("Error Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.grid()
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(df["Timestamp"], df["P_Term"], label="P_Term", color="green")
    plt.plot(df["Timestamp"], df["I_Term"], label="I_Term", color="orange")
    plt.plot(df["Timestamp"], df["D_Term"], label="D_Term", color="red")
    plt.title("PID Terms Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("PID Term Values")
    plt.grid()
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(df["Timestamp"], df["Control_Output"], label="Control Output", color="purple")
    plt.title("Control Output Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Output")
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.savefig(filename)

def plot_pose_data(df):
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(df["Y"], df["X"], label="Trajectory", color="blue")
    plt.title("Robot Trajectory (X vs. Y)")
    plt.xlabel("Y Position (m)")
    plt.ylabel("X Position (m)")
    plt.grid()
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(df["Timestamp"], df["Orientation_Z"], label="Orientation Z", color="red")
    plt.plot(df["Timestamp"], df["Orientation_W"], label="Orientation W", color="orange")
    plt.title("Orientation Over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Orientation (Quaternion)")
    plt.grid()
    plt.legend()

    plt.tight_layout()
    plt.savefig("outputs/pose_plot.png")


def main():
    pid_log_file_path = "outputs/lin_pid_log.csv"
    pid_ang_log_file_path = "outputs/ang_pid_log.csv"
    pose_log_file_path = "outputs/odom_log.csv"

    pid_df = load_pid_log(pid_log_file_path)
    plot_pid_data(pid_df, "outputs/lin_pid_plot.png")

    pid_ang_df = load_pid_log(pid_ang_log_file_path)
    plot_pid_data(pid_ang_df, "outputs/ang_pid_plot.png")

    pose_df = load_pose_log(pose_log_file_path)
    plot_pose_data(pose_df)

if __name__ == "__main__":
    main()
