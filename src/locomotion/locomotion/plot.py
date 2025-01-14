import pandas as pd
import matplotlib.pyplot as plt

# Load the PID log data
def load_pid_log(file_path):
    """Load PID data from a CSV file into a Pandas DataFrame."""
    columns = ["Timestamp", "Error", "P_Term", "I_Term", "D_Term", "Control_Output"]
    return pd.read_csv(file_path, names=columns, skiprows=1)

# Plot the PID data
def plot_pid_data(df):
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

    # Adjust layout and show the plot
    plt.tight_layout()
    plt.savefig("pid_plot.png")

def main():
    # File path to the PID log
    log_file_path = "pid_log.csv"

    # Load and plot the data
    df = load_pid_log(log_file_path)
    plot_pid_data(df)

if __name__ == "__main__":
    main()
