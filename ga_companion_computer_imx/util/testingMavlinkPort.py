import time
from threading import Timer
from datetime import datetime
import pytz
import logging
import threading
from pymavlink import mavutil,mavwp

# Function to get GPS coordinates of all navigation waypoints in the mission
def get_mission_coordinates(device):
    # Create a mavlink connection object
    mav = mavutil.mavlink_connection(device)
    print("Connected")
    # Wait for the first heartbeat message
    mav.wait_heartbeat()
    print("Waiting for heart beat done")

    coordinates = []

    # Get home position for NED coordinates
    msg = mav.recv_match(type='HOME_POSITION', blocking=True)
    if msg is None:
        print("Home position not available.")
        return

    home_lat = msg.latitude / 1e7
    home_lon = msg.longitude / 1e7
    home_alt = msg.altitude

    # Loop through all mission items
    for i in range(mav.mission_count()):
        # Request the mission item
        mav.mav.mission_request_int_send(mav.target_system, mav.target_component, i)
        msg = mav.recv_match(type='MISSION_ITEM_INT', blocking=True)
        if msg is not None:
            # Check if the mission item is a navigation command
            if msg.command == pymavlink.mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                # Get the GPS coordinates of the waypoint
                lat = msg.x / 1e7
                lon = msg.y / 1e7
                alt = msg.z

                # Convert GPS coordinates to NED
                d_lat = lat - home_lat
                d_lon = lon - home_lon
                d_alt = home_alt - alt  # altitude in NED is positive upwards
                R = 6371000  # Earth's radius in meters
                x = d_lon * math.pi / 180 * R * math.cos(home_lat * math.pi / 180)
                y = d_lat * math.pi / 180 * R
                z = d_alt

                # Add the NED coordinates to the list
                coordinates.append((x, y, z))
            elif msg.command == pymavlink.mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                print(f"Skipping takeoff command at index {i}.")
            elif msg.command == pymavlink.mavutil.mavlink.MAV_CMD_NAV_LAND:
                print(f"Skipping land command at index {i}.")
            elif msg.command == pymavlink.mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
                print(f"Skipping RTL command at index {i}.")
            else:
                print(f"Skipping non-navigation command at index {i} with command value {msg.command}.")
        else:
            print(f"Failed to retrieve mission item at index {i}.")
    
    return coordinates
get_mission_coordinates("/dev/ttymxc6")