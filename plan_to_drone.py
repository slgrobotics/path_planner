#!/usr/bin/env python3

# ===============================================================================
# @brief Upload a QGroundControl .plan mission file to a MAVSDK-compatible drone.
#
# This script connects to a MAVSDK drone via UDP, imports a QGroundControl mission
#   plan, and uploads the mission to the drone.
# View the mission in QGroundControl's Plan View after running this script -
#   use "Download" button.
#
# Install dependencies with:
#   pip3 install mavsdk --break-system-packages
#
# Usage:
#   python3 plan_to_drone.py <path_to_plan_file>
#
# Created by: GitHub Copilot
# Date: 2026-05-04
# ===============================================================================

import asyncio
import sys
from mavsdk import System
from mavsdk.mission_raw import MissionRawError

async def run_mission_upload():

    # 1. Check if a filename was provided in the terminal
    if len(sys.argv) < 2:
        print("Usage: ./plan_to_drone.py <path_to_plan_file>")
        return

    plan_path = sys.argv[1]  # This captures, for example, '../front-east/paths/front-east-feel_mission.plan'

    # Connect to the drone (adjust address as needed)

    # listen to drone heartbeat (if enabled there), local IP used:
    #drone_addr = "udpin://0.0.0.0:14550"
    #drone_addr = "udpin://0.0.0.0:14540"
    #drone_addr = "udpin://0.0.0.0:14030"
    #drone_addr = "udpin://0.0.0.0:13280"

    # send heartbeat to drone, drone IP required:
    #drone_addr = "udpout://navio:18570"  # instance #0
    drone_addr = "udpout://navio:14580"   # instance #1
    #drone_addr = "udpout://navio:14280"  # instance #2
    #drone_addr = "udpout://navio:13030"  # instance #3

    print(f"IP: connecting: {drone_addr}")

    drone = System()
    await drone.connect(system_address=drone_addr)

    try:
        print("Waiting for drone to connect...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print("OK: Connected!")
                break

        print(f"IP: Importing mission from: {plan_path}")
        # Import the .plan file into MAVSDK raw mission items
        mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission(plan_path)
        
        # 3. Upload the mission to the drone
        print(f"IP Uploading {len(mission_import_data.mission_items)} items...")
        await drone.mission_raw.upload_mission(mission_import_data.mission_items)
        
        print("Done! Check QGroundControl Plan View to see the mission.")

        print("OK: Mission upload complete.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # 2. "Disconnect" logic
        # In MAVSDK-Python, simply letting the script end or 
        # deleting the object closes the connection.
        print("IP: Closing connection...")
        drone._stop_mavsdk_server()
        del drone 
        print("Done")

if __name__ == "__main__":
    try:
        asyncio.run(run_mission_upload())
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Cleaning up and exiting...")
        # Any final cleanup happens here automatically as the loop closes
        sys.exit(0)
