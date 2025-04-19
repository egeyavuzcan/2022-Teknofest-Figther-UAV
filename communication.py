import logging
import json
from datetime import datetime
from typing import Tuple, Optional
import requests
from dronekit import Vehicle, connect

class Communication:
    """
    Handles communication with the UAV and the remote server via REST API.
    """

    def __init__(self, team_number: int = 199, server_url: str = "http://10.0.0.15:64559"):
        """
        Initialize communication client.

        Args:
            team_number: Teknofest team number.
            server_url: Base URL for server API endpoints.
        """
        self.team_number = team_number
        self.server_url = server_url.rstrip("/")
        self.session = requests.Session()
        self.log_file = open("log.txt", "a")
        logging.basicConfig(
            level=logging.INFO,
            filename="log.txt",
            format="%(asctime)s - %(levelname)s - %(message)s",
        )
        logging.info("=== Start of Communication Log ===")

    def close(self) -> None:
        """
        Close log file and HTTP session.
        """
        self.session.close()
        self.log_file.close()

    def draw_hud(self, width: int, height: int) -> Tuple[int, int, int, int]:
        """
        Calculate HUD overlay positions based on frame dimensions.

        Returns:
            Tuple of (x_right, y_right, x_left, y_left).
        """
        x_right = width // 4
        y_right = height // 10
        x_left = x_right * 3
        y_left = y_right * 9
        return x_right, y_right, x_left, y_left

    def connect_drone(self, connection_str: str = "127.0.0.1:14570", baud: int = 57600) -> Vehicle:
        """
        Connect to UAV via DroneKit.

        Args:
            connection_str: Connection string for UAV (e.g., SITL or serial port).
            baud: Baud rate for serial connection.

        Returns:
            Connected Vehicle instance.
        """
        logging.info(f"Connecting to UAV at {connection_str} with baud {baud}")
        vehicle = connect(connection_str, baud=baud, wait_ready=True)
        return vehicle

    def login(self, username: str, password: str) -> requests.Response:
        """
        Authenticate with remote server.

        Args:
            username: Team username.
            password: Team password.

        Returns:
            HTTP Response from server.
        """
        payload = {"username": username, "password": password}
        url = f"{self.server_url}/api/login"
        logging.info(f"Logging in to {url}")
        response = self.session.post(url, json=payload)
        response.raise_for_status()
        return response

    def send_kamikaze(self, start_time: datetime, end_time: datetime, qr_text: str) -> requests.Response:
        """
        Send kamikaze mission information to server.

        Args:
            start_time: Mission start datetime.
            end_time: Mission end datetime.
            qr_text: Detected QR code text.

        Returns:
            HTTP Response from server.
        """
        payload = {
            "kamikazeStartTime": {"hours": start_time.hour, "minutes": start_time.minute, "seconds": start_time.second, "milliseconds": int(start_time.microsecond / 1000)},
            "kamikazeEndTime":   {"hours": end_time.hour,   "minutes": end_time.minute,   "seconds": end_time.second,   "milliseconds": int(end_time.microsecond / 1000)},
            "qrText": qr_text,
        }
        url = f"{self.server_url}/api/kamikaze-info"
        logging.info(f"Sending kamikaze payload: {json.dumps(payload)}")
        response = self.session.post(url, json=payload)
        response.raise_for_status()
        return response

    def send_lock_info(self, start_time: datetime, end_time: datetime, autolock: bool = False) -> requests.Response:
        """
        Send lock/tracking information to server.

        Args:
            start_time: Tracking lock start datetime.
            end_time: Tracking lock end datetime.
            autolock: Indicates if lock was autonomous.

        Returns:
            HTTP Response from server.
        """
        payload = {
            "startTime": {"hours": start_time.hour, "minutes": start_time.minute, "seconds": start_time.second, "milliseconds": int(start_time.microsecond / 1000)},
            "endTime":   {"hours": end_time.hour,   "minutes": end_time.minute,   "seconds": end_time.second,   "milliseconds": int(end_time.microsecond / 1000)},
            "autolock": autolock,
        }
        url = f"{self.server_url}/api/lock-info"
        logging.info(f"Sending lock payload: {json.dumps(payload)}")
        response = self.session.post(url, json=payload)
        response.raise_for_status()
        return response

    def get_coordinates(self) -> dict:
        """
        Get QR code coordinates from server.

        Returns:
            Dictionary containing coordinates.
        """
        url = f"{self.server_url}/api/qr-coordinate"
        logging.info(f"Getting coordinates from {url}")
        response = self.session.get(url)
        response.raise_for_status()
        return response.json()

    def send_telemetry(self, vehicle: Vehicle, lock_status: bool, msg: str) -> requests.Response:
        """
        Send telemetry data to server.

        Args:
            vehicle: UAV vehicle instance.
            lock_status: Indicates if UAV is locked onto target.
            msg: Message to be sent.

        Returns:
            HTTP Response from server.
        """
        battery_percent = int(((vehicle.battery.voltage - 14) * 100) / 2.8)
        center_x, center_y, width, height = msg  # target bounding box
        telemetry_payload = {
            "team_number":       self.team_number,
            "uav_latitude":      vehicle.location.global_frame.lat,
            "uav_longitude":     vehicle.location.global_frame.lon,
            "uav_altitude":      vehicle.location.global_frame.alt,
            "uav_pitch":         int(vehicle.attitude.pitch * 57.2958),
            "uav_yaw":           int(vehicle.attitude.yaw * 57.2958),
            "uav_roll":          int(vehicle.attitude.roll * 57.2958),
            "uav_speed":         int(vehicle.velocity[0]),
            "uav_battery":       battery_percent,
            "uav_autonomous":    lock_status,
            "uav_locked":        False,
            "target_center_x":   center_x,
            "target_center_y":   center_y,
            "target_width":      width,
            "target_height":     height,
            "gps_timestamp":     None,
        }
        url = f"{self.server_url}/api/telemetry"
        logging.info(f"Sending telemetry payload: {json.dumps(telemetry_payload)}")
        response = self.session.post(url, json=telemetry_payload)
        response.raise_for_status()
        return response

    def get_server_time(self) -> Tuple[int, int, int, int]:
        """
        Get current time from server.

        Returns:
            Tuple of (hours, minutes, seconds, milliseconds).
        """
        url = f"{self.server_url}/api/server-time"
        logging.info(f"Getting time from {url}")
        response = self.session.get(url)
        response.raise_for_status()
        data = response.json()
        return int(data["hours"]), int(data["minutes"]), int(data["seconds"]), int(data["milliseconds"])

    def run(
        self,
        vehicle: Vehicle,
        msg: str,
        hours_value: int,
        minutes_values: int,
        seconds_value: int,
        milisecs_value: int,
        hours_current: int,
        minutes_current: int,
        seconds_current: int,
        milisecs_current: int,
        continuity_of_lock: bool,
        flag_for_start_time: bool,
    ) -> Tuple[requests.Response, bool, bool, int, int, int, int]:
        """
        Run communication loop.

        Args:
            vehicle: UAV vehicle instance.
            msg: Message to be sent.
            hours_value: Hours value.
            minutes_values: Minutes value.
            seconds_value: Seconds value.
            milisecs_value: Milliseconds value.
            hours_current: Current hours value.
            minutes_current: Current minutes value.
            seconds_current: Current seconds value.
            milisecs_current: Current milliseconds value.
            continuity_of_lock: Indicates if UAV is locked onto target.
            flag_for_start_time: Indicates if start time has been set.

        Returns:
            Tuple of (HTTP Response, continuity_of_lock, flag_for_start_time, hours_value, minutes_values, seconds_value, milisecs_value).
        """
        a1, a2, a3, a4 = self.get_server_time()
        utc = [a1, a2, a3, a4]
        xflag = 0
        if continuity_of_lock:
            xflag = 1
        returned_telemetry = self.send_telemetry(vehicle, xflag, msg)
        logging.info(f"Received telemetry response: {returned_telemetry.json()}")
        return returned_telemetry, continuity_of_lock, flag_for_start_time, hours_value, minutes_values, seconds_value, milisecs_value
