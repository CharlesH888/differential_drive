from config import Config
from plot import Plot

# Starting point of the code
def main():
    # Get targets from config file
    targets = Config.target_locations
    #Initialize Plot so it can grab necessary value from config file
    plot = Plot()
    # Starts the plotting / bot maneuver process
    targets_met = plot.plot_path(targets)
    return targets_met


if __name__ == "__main__":
    main()