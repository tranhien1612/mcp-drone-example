import threading
import asyncio
import time
from pymavlink import mavutil

class DroneController:
    def __init__(self):
        self.master = None
        self.connection_str = None
        self.disconnect_flag = False
        self.drone_info = {"status": "No active connection"}
        self.thread = None

    # -----------------------
    # Connection Management
    # -----------------------
    async def _connect_drone(self, connection_str: str):
        """
        Connect to a MAVLink drone and keep telemetry updated.
        """
        self.connection_str = connection_str
        self.drone_info = {
            # "connection_id": connection_str,
            "status": f"Connecting to {connection_str}..."
        }
        self.disconnect_flag = False

        while not self.disconnect_flag:
            try:
                print(f"[INFO] Connecting to drone at {connection_str} ...")
                master = mavutil.mavlink_connection(connection_str)
                self.master = master
                master.wait_heartbeat(timeout=15)

                self.drone_info["status"] = (
                    f"Connected [{master.target_system}:{master.target_component}]"
                )
                print(f"[INFO] {self.drone_info['status']}")

                while not self.disconnect_flag:
                    msg = master.recv_match(blocking=False)
                    if not msg:
                        await asyncio.sleep(0.1)
                        continue

                    mtype = msg.get_type()

                    # GPS and altitude
                    if mtype == "GLOBAL_POSITION_INT":
                        self.drone_info["latitude"] = msg.lat / 1e7
                        self.drone_info["longitude"] = msg.lon / 1e7
                        self.drone_info["altitude_m"] = msg.relative_alt / 1000.0

                    # Battery info
                    elif mtype == "SYS_STATUS":
                        self.drone_info["battery_voltage_V"] = msg.voltage_battery / 1000.0
                        self.drone_info["battery_remaining_%"] = msg.battery_remaining

                    # Mode, system status
                    elif mtype == "HEARTBEAT":
                        self.drone_info["mode"] = mavutil.mode_string_v10(msg)
                        self.drone_info["system_status"] = msg.system_status
                        self.drone_info["type"] = msg.type
                        self.drone_info["system_id"] = master.target_system
                        self.drone_info["component_id"] = master.target_component

                    elif mtype == "ATTITUDE":
                        # Angles are in radians; convert to degrees
                        self.drone_info["imu_roll_deg"] = msg.roll * 57.2958
                        self.drone_info["imu_pitch_deg"] = msg.pitch * 57.2958
                        self.drone_info["imu_yaw_deg"] = msg.yaw * 57.2958
                        self.drone_info["angular_velocity"] = {
                            "rollspeed_rad_s": msg.rollspeed,
                            "pitchspeed_rad_s": msg.pitchspeed,
                            "yawspeed_rad_s": msg.yawspeed,
                        }

                    elif mtype == "RAW_IMU":
                        self.drone_info["acceleration_raw"] = {
                            "x": msg.xacc / 1000.0,  # m/s^2 approx.
                            "y": msg.yacc / 1000.0,
                            "z": msg.zacc / 1000.0,
                        }
                        self.drone_info["gyro_raw"] = {
                            "x": msg.xgyro / 1000.0,  # rad/s approx.
                            "y": msg.ygyro / 1000.0,
                            "z": msg.zgyro / 1000.0,
                        }
                        self.drone_info["mag_raw"] = {
                            "x": msg.xmag,
                            "y": msg.ymag,
                            "z": msg.zmag,
                        }

                    self.drone_info["status"] = "Connected"

            except Exception as e:
                if self.disconnect_flag:
                    break
                self.drone_info["status"] = f"Disconnected: {e}"
                print(f"[ERROR] {connection_str} disconnected: {e}")
                await asyncio.sleep(5)

        print(f"[INFO] Connection closed for {connection_str}")
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None
        self.connection_str = None
        self.drone_info.clear()
        self.drone_info.update({"status": "No active connection"})

    def connect(self, connection_ip: str, port: int):
        """Start MAVLink connection in a background thread."""
        connection_str = f"udp:{connection_ip}:{port}"
        if self.thread and self.thread.is_alive():
            print("[WARN] Connection already active.")
            return "Connection already active."

        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._connect_drone(connection_str))
            loop.close()

        self.thread = threading.Thread(target=_run, daemon=True)
        self.thread.start()
        print(f"[INFO] Started connection thread for {connection_str}")
        return f"Connecting to {connection_str}"

    def disconnect(self) -> str:
        """Stop the MAVLink connection."""
        if not self.master:
            self.drone_info.update({"status": "No active connection"})
            return "No active connection to disconnect."

        self.disconnect_flag = True
        print("[INFO] Disconnect requested.")
        return "Disconnecting drone..."

    # -----------------------
    # Drone Commands
    # -----------------------
    def change_mode(self, mode_name: str) -> str:
        """
        Change the drone's flight mode (e.g., GUIDED, LOITER, RTL, STABILIZE, AUTO).
        Validates that the mode exists before attempting to change.
        """
        if not self.master:
            return "No active drone connection."

        try:
            # Fetch available mode mapping from the connected autopilot
            mode_mapping = self.master.mode_mapping()
            if not mode_mapping:
                return "Unable to retrieve mode mapping from the drone."

            # Check if requested mode exists
            if mode_name.upper() not in mode_mapping:
                available_modes = ", ".join(mode_mapping.keys())
                return (
                    f"Invalid mode '{mode_name}'. "
                    f"Available modes: {available_modes}"
                )

            print(f"[INFO] Changing flight mode to {mode_name.upper()}...")
            self.master.set_mode(mode_name.upper())

            self.drone_info["mode"] = mode_name.upper()
            self.drone_info["status"] = f"Mode changed to {mode_name.upper()}"
            return f"Flight mode changed to {mode_name.upper()}."

        except Exception as e:
            return f"Failed to change flight mode: {e}"

    def arm(self) -> str:
        """Arm the drone motors."""
        if not self.master:
            return "No active drone connection."

        try:
            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            self.drone_info["status"] = "Motors armed"
            print("[INFO] Motors armed.")
            return "Drone motors armed."

        except Exception as e:
            return f"Arming failed: {e}"
    
    def disarm(self) -> str:
        """DisArm the drone motors."""
        if not self.master:
            return "No active drone connection."

        try:
            self.master.arducopter_disarm()
            self.master.motors_disarmed_wait()
            self.drone_info["status"] = "Motors disarmed"
            print("[INFO] Motors disarmed.")
            return "Drone motors disarmed."

        except Exception as e:
            return f"Arming failed: {e}"
        
    def takeoff(self, altitude: float) -> str:
        """Command the drone to take off."""
        if not self.master:
            return "No active drone connection."

        try:
            print(f"[INFO] Taking off to {altitude}m...")
            self.master.set_mode("GUIDED")
            time.sleep(1)

            self.master.arducopter_arm()
            self.master.motors_armed_wait()
            print("[INFO] Motors armed.")
            time.sleep(2)

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, altitude
            )

            self.drone_info["status"] = f"Taking off to {altitude}m"
            timeout = time.time() + 30

            while time.time() < timeout:
                alt = self.drone_info.get("altitude_m", 0)
                if alt and alt >= altitude * 0.95:
                    print(f"[INFO] Target altitude {altitude}m reached.")
                    self.drone_info["status"] = f"Hovering at {altitude}m"
                    return f"Takeoff complete. Altitude: {alt:.1f}m"
                time.sleep(0.5)

            return f"Timeout: Drone did not reach {altitude}m."

        except Exception as e:
            return f"Takeoff failed: {e}"

    def land(self) -> str:
        """Command the drone to land."""
        if not self.master:
            return "No active drone connection."

        try:
            print("[INFO] Landing initiated...")
            self.master.set_mode("LAND")
            self.drone_info["status"] = "Landing..."
            time.sleep(1)

            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0, 0, 0, 0, 0, 0, 0, 0
            )

            start_alt = self.drone_info.get("altitude_m", None)
            timeout = time.time() + 40

            while time.time() < timeout:
                alt = self.drone_info.get("altitude_m", 0)
                if alt is not None and alt < 0.5:
                    print("[INFO] Landing complete.")
                    self.drone_info["status"] = "Landed"
                    return "Drone landed successfully."
                if start_alt and alt < start_alt * 0.2:
                    print("[INFO] Altitude reduced significantly, nearing ground.")
                time.sleep(0.5)

            return "Timeout: Drone may not have fully landed."

        except Exception as e:
            return f"Landing failed: {e}"

    def rtl(self) -> str:
        """Command the drone to Return to Launch (RTL)."""
        if not self.master:
            return "No active drone connection."

        try:
            print("[INFO] RTL initiated...")
            self.master.set_mode("RTL")
            self.drone_info["status"] = "Returning to Launch"
            return "RTL command sent."

        except Exception as e:
            return f"RTL failed: {e}"
    
    def goto_location(self, latitude: float, longitude: float, altitude: float) -> str:
        """
        Command the drone to fly to a specified GPS coordinate (lat, lon, alt in meters).
        Requires the drone to be in GUIDED mode and armed.
        """
        if not self.master:
            return "No active drone connection."

        try:
            # Ensure drone is in GUIDED mode
            print("[INFO] Switching to GUIDED mode...")
            self.master.set_mode("GUIDED")
            time.sleep(1)

            # Ensure drone is armed
            if not self.drone_info.get("status", "").startswith("Motors armed"):
                print("[INFO] Arming motors...")
                self.master.arducopter_arm()
                self.master.motors_armed_wait()
                print("[INFO] Motors armed successfully.")

            print(f"[INFO] Navigating to ({int(latitude * 1e7)}, {int(longitude * 1e7)}, {altitude})...")

            # Send MAVLink command
            self.master.mav.set_position_target_global_int_send(
                0, #int(time.time() * 1000),
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # Type mask
                int(latitude * 1e7),
                int(longitude * 1e7),
                altitude,
                0, 0, 0,  # Velocity (ignored)
                0, 0, 0,  # Accel (ignored)
                0, 0      # Yaw, yaw_rate
            )

            return f"Drone navigating to ({latitude:.6f}, {longitude:.6f}, {altitude:.1f}m)"

        except Exception as e:
            return f"Goto command failed: {e}"

    # -----------------------
    # Info Accessors
    # -----------------------
    def get_info(self) -> dict:
        """Return latest telemetry info."""
        return self.drone_info

'''
# Arm the drone
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')

# Disarm the drone
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
master.motors_disarmed_wait()
print('Disarmed!')

# Change Mode
mode = 'STABILIZE'
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)


'''
