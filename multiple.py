import pathlib
import threading
import logging

import drone  # Import the Drone class from drone.py

# List of drone URIs. Uncomment or add more URIs as needed.
URIS = [
    # "radio://0/80/2M/E7E7E7E701",
    # "radio://0/80/2M/E7E7E7E702",
    "radio://0/80/2M/E7E7E7E703",
]


def control_drone(uri, drone_id):
    """
    Function to control a single drone. Initializes the drone, executes flight commands,
    and ensures safe landing.

    :param uri: The URI of the drone to control.
    :param drone_id: The identifier for the drone.
    """
    # Define the path for the drone's log file
    log_path = pathlib.Path(f"log_{drone_id}.txt").resolve()

    # Initialize the Drone object with URI and ID, and associate it with a log file
    d = drone.Drone(drone_uri=uri, drone_id=drone_id, log_file=log_path)

    try:
        # Execute the takeoff command
        d.take_off()

        # Uncomment and add additional flight commands as needed
        # Example:
        # d.move_linear()
        # d.perform_custom_trajectory(waypoints=[(0.5, 0, 0), (0, 0.5, 0), (-0.5, 0, 0), (0, -0.5, 0)])
        # d.set_velocity(vx=0.2, vy=0.0, vz=0.0, duration=5)

    except Exception as e:
        # Log any exceptions that occur during drone control
        logging.error(f"Error controlling drone {drone_id} ({uri}): {e}")

    finally:
        # Ensure the drone lands safely in case of any errors
        d.land()


if __name__ == "__main__":
    # Configure logging to capture errors and write them to 'multiple_errors.log'
    logging.basicConfig(
        filename="multiple_errors.log",
        filemode="w",
        level=logging.ERROR,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )

    threads = []  # List to keep track of all threads

    # Iterate over the list of URIs and create a thread for each drone
    for drone_id, uri in enumerate(URIS):
        # Create a new thread targeting the `control_drone` function
        thread = threading.Thread(target=control_drone, args=(uri, drone_id))
        threads.append(thread)  # Add the thread to the list
        thread.start()  # Start the thread

    # Wait for all threads to complete their execution
    for thread in threads:
        thread.join()
