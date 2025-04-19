from typing import Optional, List
from fastapi import FastAPI
from pydantic import BaseModel
import datetime

app = FastAPI()

class TimeStamp(BaseModel):
    """Represents a timestamp with hours, minutes, seconds, and milliseconds."""
    hours: int
    minutes: int
    seconds: int
    milliseconds: int

class Telemetry(BaseModel):
    """UAV telemetry data."""
    team_number: int
    uav_latitude: float
    uav_longitude: float
    uav_altitude: float
    uav_pitch: int
    uav_yaw: int
    uav_roll: int
    uav_speed: int
    uav_battery: int
    uav_autonomous: bool
    uav_locked: bool
    target_center_x: int
    target_center_y: int
    target_width: int
    target_height: int
    gps_timestamp: Optional[TimeStamp] = None

class Coordinate(BaseModel):
    """QR code coordinates."""
    qr_latitude: float
    qr_longitude: float

class LockInfo(BaseModel):
    """Lock (tracking) information with start and end timestamps."""
    start_time: Optional[TimeStamp] = None
    end_time: Optional[TimeStamp] = None
    autolock: bool

def get_current_time() -> TimeStamp:
    """Get the current server time as a TimeStamp."""
    now = datetime.datetime.now()
    return TimeStamp(
        hours=now.hour,
        minutes=now.minute,
        seconds=now.second,
        milliseconds=int(now.microsecond / 1000)
    )

@app.get("/")
def read_root() -> dict:
    """Root endpoint providing API documentation paths."""
    return {"docs": "/docs", "redocs": "/redoc", "owner": "@e"}

@app.get("/api/server-time")
def server_time() -> TimeStamp:
    """Endpoint to get server time."""
    return get_current_time()

@app.post("/api/kamikaze-info")
def post_kamikaze_info() -> int:
    """Endpoint to receive kamikaze information."""
    return 0

@app.get("/api/qr-coordinate")
def get_qr_coordinate() -> Coordinate:
    """Endpoint to retrieve QR code coordinates."""
    return Coordinate(qr_latitude=41.123456, qr_longitude=26.654987)

@app.post("/api/telemetry")
async def post_telemetry(data: Telemetry) -> dict:
    """Endpoint to receive telemetry and respond with sample position data."""
    sample_positions: List[dict] = [
        {
            "team_number": 1,
            "uav_latitude": 40.231998,
            "uav_longitude": 29.0037,
            "uav_altitude": 500,
            "uav_pitch": 5,
            "uav_yaw": 256,
            "uav_roll": 0,
            "time_difference": 93,
        },
        {
            "team_number": 2,
            "uav_latitude": 40.23126,
            "uav_longitude": 29.003631,
            "uav_altitude": 190,
            "uav_pitch": 5,
            "uav_yaw": 256,
            "uav_roll": 0,
            "time_difference": 74,
        },
        {
            "team_number": 3,
            "uav_latitude": 40.243071,
            "uav_longitude": 29.003746,
            "uav_altitude": 222.3,
            "uav_pitch": 5,
            "uav_yaw": 256,
            "uav_roll": 0,
            "time_difference": 43,
        }
    ]
    return {"server_time": get_current_time(), "positions": sample_positions}

@app.post("/api/login")
def login(data: Telemetry) -> int:
    """Endpoint for team login."""
    return 0

@app.post("/api/lock-info")
def post_lock_info(info: LockInfo) -> LockInfo:
    """Endpoint to receive lock information."""
    return info
