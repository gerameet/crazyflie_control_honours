import math
import logging
import sys
import time
import pathlib
from threading import Event

# Colorama library for colored console output
import colorama
from colorama import Fore, Style

# Initialize colorama
colorama.init()

# Crazyflie Python library imports
import cflib.crtp  # Crazy RealTime Protocol
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig  # Logging configuration
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie  # Synchronized Crazyflie
from cflib.utils import uri_helper  # URI helper functions
from cflib.positioning.motion_commander import MotionCommander  # Movement control

# Default flight parameters and colors for console output
DEFAULT_HEIGHT = 0.40  # Default flight height in meters
COLORS = [Fore.BLUE, Fore.RED, Fore.GREEN, Fore.BLACK]  # Colors for different drones

class Drone:
    """
    A class to represent a drone and control its flight using the Crazyflie API.
    """

    def __init__(self, drone_uri="radio://0/80/2M/E7E7E7E7E7", drone_id=0, log_file=None):
        """
        Initialize the drone with its URI, ID, and an optional log file.

        :param drone_uri: The URI of the drone to connect to.
        :param drone_id: An identifier for the drone (used for console output).
        :param log_file: A pathlib.Path object to a log file for logging data.
        """
        # Set the drone's URI
        self.uri = uri_helper.uri_from_env(default=drone_uri)
        # Event to check if the deck is attached
        self.deck_attached_event = Event()
        # Drone ID for identification
        self.id = drone_id
        # Log file for recording data
        self.log_file = log_file

        # Configure logging
        logging.basicConfig(level=logging.ERROR)
        # Initialize the low-level drivers
        cflib.crtp.init_drivers()

        # Create a LogConfig object for logging drone's state
        self.log_config = LogConfig(name="Stabilizer", period_in_ms=10)
        # Add variables to the log configuration
        self.log_config.add_variable("stateEstimate.x", "float")
        self.log_config.add_variable("stateEstimate.y", "float")
        self.log_config.add_variable("stateEstimate.z", "float")
        self.log_config.add_variable("stabilizer.roll", "float")
        self.log_config.add_variable("stabilizer.pitch", "float")
        self.log_config.add_variable("stabilizer.yaw", "float")

        # Placeholder for SyncCrazyflie object (set in main)
        self.scf = None

        # Initialize drone state variables
        self.x = None
        self.y = None
        self.z = None
        self.roll = None
        self.pitch = None
        self.yaw = None

    def _log_position_callback(self, timestamp, data, logconf):
        """
        Callback function for logging position data asynchronously.

        :param timestamp: The time at which the data was logged.
        :param data: A dictionary containing the logged variables.
        :param logconf: The LogConfig object.
        """
        # Create a log message with color formatting
        log_message = (
            COLORS[self.id]
            + f"[{self.id}] [{timestamp}][{logconf.name}]: {data}"
            + Style.RESET_ALL
        )

        if self.log_file:
            # If a log file is specified, write the log message to the file
            with open(self.log_file, "a") as outfile:
                outfile.write(log_message + "\n")
        else:
            # Otherwise, print the log message to the console
            print(log_message)

        # Update the drone's state variables with the new data
        self.x = data["stateEstimate.x"]
        self.y = data["stateEstimate.y"]
        self.z = data["stateEstimate.z"]
        self.roll = data["stabilizer.roll"]
        self.pitch = data["stabilizer.pitch"]
        self.yaw = data["stabilizer.yaw"]

    def log_async(self):
        """
        Start asynchronous logging of the drone's state variables.
        """
        # Add the log configuration to the drone
        self.scf.cf.log.add_config(self.log_config)
        # Set the callback function for received log data
        self.log_config.data_received_cb.add_callback(self._log_position_callback)
        # Start logging
        self.log_config.start()
        # Allow logging to run for a period of time
        time.sleep(1)
        # Stop logging
        self.log_config.stop()

    def deck_attachment_callback(self, name, value_str):
        """
        Callback function to check if the Flow deck is correctly attached.

        :param name: The parameter name.
        :param value_str: The value of the parameter as a string.
        """
        # Convert the string value to an integer
        value = int(value_str)
        if value:
            # If the deck is attached, set the event
            self.deck_attached_event.set()
            print(
                COLORS[self.id] + f"[{self.id}] Deck is attached!" + Style.RESET_ALL
            )
        else:
            # If the deck is not attached, inform the user
            print(
                COLORS[self.id] + f"[{self.id}] Deck is NOT attached!" + Style.RESET_ALL
            )

    def take_off(self):
        """
        Command the drone to take off and hover at the default height.
        """
        # Use the MotionCommander for high-level movement commands
        with MotionCommander(self.scf, default_height=DEFAULT_HEIGHT) as mc:
            # Wait for the drone to stabilize
            time.sleep(3)
            # Stop any movement
            mc.land()

    def move_linear(self):
        """
        Command the drone to move forward and then back to the starting position.
        """
        with MotionCommander(self.scf, default_height=DEFAULT_HEIGHT) as mc:
            # Wait for the drone to stabilize
            time.sleep(3)
            # Move forward by 0.5 meters
            mc.forward(0.5)
            time.sleep(3)
            # Turn around
            mc.turn_left(180)
            time.sleep(3)
            # Move forward (which is effectively moving back to the start)
            mc.forward(0.5)
            time.sleep(3)
            # Stop any movement
            mc.stop()

    def move_square(self):
        """
        Command the drone to fly in a square pattern.
        """
        with MotionCommander(self.scf, default_height=DEFAULT_HEIGHT) as mc:
            # Wait for the drone to stabilize
            time.sleep(3)

            for i in range(4):
                # Move forward by 0.75 meters
                mc.forward(0.75)
                time.sleep(3)
                # Turn left by 90 degrees
                mc.turn_left(90)
                time.sleep(3)

            # Wait briefly before stopping
            time.sleep(1)
            # Stop any movement
            mc.stop()

    def move_fancy_square(self):
        """
        Command the drone to fly in a square pattern with vertical movement.
        """
        with MotionCommander(self.scf, default_height=DEFAULT_HEIGHT) as mc:
            # Wait for the drone to stabilize
            time.sleep(3)

            for i in range(4):
                # Move forward by 0.75 meters
                mc.forward(0.75)
                time.sleep(3)
                if i % 2 == 0:
                    # On even iterations, ascend by 0.35 meters
                    mc.up(0.35)
                    time.sleep(3)
                else:
                    # On odd iterations, descend by 0.35 meters
                    mc.down(0.35)
                    time.sleep(3)
                # Turn left by 90 degrees
                mc.turn_left(90)
                time.sleep(3)

            # Wait briefly before stopping
            time.sleep(2)
            # Stop any movement
            mc.stop()

    def move_circle(self):
        """
        Command the drone to fly in a circular pattern.
        """
        with MotionCommander(self.scf, default_height=DEFAULT_HEIGHT) as mc:
            # Wait for the drone to stabilize
            time.sleep(3)

            # Fly in a circle to the left with a radius of 0.75 meters
            mc.circle_left(0.75)
            time.sleep(2)
            # Ascend by 0.5 meters
            mc.up(0.5)
            time.sleep(2)
            # Fly in a circle to the right with a radius of 0.75 meters
            mc.circle_right(0.75)
            time.sleep(2)
            # Descend by 0.5 meters
            mc.down(0.5)

            # Wait briefly before stopping
            time.sleep(3)
            # Stop any movement
            mc.stop()

    def perform_shm(self, amplitude, time_period, total_time, phi_0=0):
        """
        Command the drone to perform simple harmonic motion along the x-axis.

        :param amplitude: The amplitude of the motion in meters.
        :param time_period: The time period of the motion in seconds.
        :param total_time: The total duration to perform the motion in seconds.
        :param phi_0: The initial phase angle in radians.
        """
        # Calculate the angular frequency
        omega = 2 * math.pi / time_period

        # Calculate initial wait time based on the phase offset
        initial_wait_time = phi_0 / omega
        if initial_wait_time > 0:
            time.sleep(initial_wait_time)

        with MotionCommander(self.scf, default_height=0.3) as mc:
            # Record the start time of the motion
            start_time = time.time()
            while True:
                # Calculate elapsed time
                elapsed_time = time.time() - start_time
                if elapsed_time >= total_time:
                    break

                # Calculate position and velocity based on SHM equations
                x_position = amplitude * math.sin(omega * elapsed_time + phi_0)
                x_velocity = amplitude * omega * math.cos(omega * elapsed_time + phi_0)

                # Use the velocity to command the drone's movement
                if x_velocity >= 0:
                    mc.start_forward(x_velocity)
                else:
                    mc.start_back(-x_velocity)

                # If a log file is specified, log the position
                if self.log_file is not None:
                    with open(self.log_file, "a") as outfile:
                        outfile.write(f"position: ({x_position}, 0.0, z)\n")

                # Sleep to maintain control update rate
                time.sleep(0.1)

            # Stop any movement
            mc.stop()

    def perform_custom_trajectory(self, waypoints):
        """
        Command the drone to follow a custom trajectory defined by waypoints.

        :param waypoints: A list of tuples representing relative movements (dx, dy, dz).
        """
        with MotionCommander(self.scf, default_height=DEFAULT_HEIGHT) as mc:
            for dx, dy, dz in waypoints:
                # Move the drone by the specified distances
                mc.move_distance(dx, dy, dz)
                time.sleep(0.1)
            mc.stop()

    def go_to_position(self, x, y, z=None):
        """
        Command the drone to move to the specified absolute position (x, y, z).

        :param x: Target x coordinate in meters.
        :param y: Target y coordinate in meters.
        :param z: Target z coordinate in meters. If None, maintains current z.
        """
        # Ensure current position estimates are available
        if self.x is None or self.y is None or self.z is None:
            print(
                COLORS[self.id]
                + f"[{self.id}] Position estimates not available. Please start logging."
                + Style.RESET_ALL
            )
            return

        # Calculate relative distances
        dx = x - self.x
        dy = y - self.y
        dz = (z - self.z) if z is not None else 0

        with MotionCommander(self.scf, default_height=self.z or DEFAULT_HEIGHT) as mc:
            # Move to the target position
            mc.move_distance(dx, dy, dz)
            # Hover at the target position
            time.sleep(3)
            mc.stop()

    def set_velocity(self, vx, vy, vz=0, duration=1):
        """
        Command the drone to move with specified velocities for a certain duration.

        :param vx: Velocity in x direction (m/s).
        :param vy: Velocity in y direction (m/s).
        :param vz: Velocity in z direction (m/s), defaults to 0.
        :param duration: Duration in seconds to maintain the velocity.
        """
        with MotionCommander(self.scf, default_height=DEFAULT_HEIGHT) as mc:
            # Start moving with the specified velocities
            mc.start_linear_motion(vx, vy, vz)
            # Maintain the motion for the specified duration
            time.sleep(duration)
            mc.stop()

    def main(self, flight_function, log=False, check_deck=True, **kwargs):
        """
        Main function to establish connection and execute flight commands.

        :param flight_function: The flight function to execute.
        :param log: A boolean indicating whether to start logging.
        :param check_deck: A boolean indicating whether to check for the deck attachment.
        :param kwargs: Additional arguments to pass to the flight function.
        """
        # Use SyncCrazyflie for thread-safe communication with the drone
        with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            # Set the SyncCrazyflie object
            self.scf = scf

            # Add a callback to check if the Flow deck is attached
            scf.cf.param.add_update_callback(
                group="deck", name="bcFlow2", cb=self.deck_attachment_callback
            )

            # Wait for parameters to be updated
            time.sleep(1)

            # If required, check if the deck is attached
            if check_deck and not self.deck_attached_event.wait(timeout=5):
                print(
                    COLORS[self.id]
                    + f"[{self.id}] No flow deck detected!"
                    + Style.RESET_ALL
                )
                sys.exit(1)

            # Start logging if requested
            if log:
                self.log_async()

            # Execute the provided flight function with any additional arguments
            if flight_function is not None:
                flight_function(**kwargs)

if __name__ == "__main__":
    # Create an instance of the Drone class
    drone = Drone()

    # Examples of how to use the Drone class methods
    # To take off and hover
    # drone.main(drone.take_off)

    # To move in a linear path
    # drone.main(drone.move_linear)

    # To fly in a square pattern
    # drone.main(drone.move_square)

    # To fly in a fancy square pattern with vertical movement
    # drone.main(drone.move_fancy_square)

    # To fly in a circular pattern
    # drone.main(drone.move_circle)

    # To perform simple harmonic motion
    # drone.main(drone.perform_shm, amplitude=1.0, time_period=2.0)
