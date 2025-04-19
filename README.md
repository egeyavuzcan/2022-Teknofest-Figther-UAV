# Fighter UAV – Teknofest 2022

## Overview
This project implements a UAV control and tracking system for Teknofest 2022. It includes:

- **Server**: REST API (FastAPI) handling telemetry, QR coordinates, lock and kamikaze info.
- **Client**: `communication.py` wraps HTTP calls and DroneKit connection.
- **Detection & Tracking**: YOLO-based detection + GOTURN tracker in `DetectAndTrack.py`.
- **Utils**: helper modules (`videoUtils.py`, `DetectAndTrackUtils.py`).
- **Examples**: `usage_example.py` demonstrates server & client usage.

## Structure
```
2022-Teknofest-Figther-UAV/
├── server.py
├── communication.py
├── DetectAndTrack.py
├── DetectAndTrackUtils.py
├── videoUtils.py
├── usage_example.py
├── README.md
├── requirements.txt
└── .gitignore
```

## Algorithm Flow
1. **Frame Acquisition**: capture from camera, video, or SITL via `videoUtils`.
2. **Object Detection**: YOLO detector produces bounding boxes.
3. **Tracker Initialization**: GOTURN tracker seeded on initial detection.
4. **Continuous Tracking**: tracker updates object location; periodic verification via YOLO.
5. **QR Code Reading**: optional QR detection to extract mission data.
6. **Communication Loop**: send telemetry, lock status, and kamikaze info to server via `Communication` client.

## Installation
### Prerequisites
- Python 3.8+

### Install dependencies
```bash
pip install -r requirements.txt
```

## Usage
### Start Server
```bash
uvicorn server:app --reload --host 0.0.0.0 --port 8000
```
Visit API docs at [http://localhost:8000/docs](http://localhost:8000/docs).

### Run Client Example
```bash
python usage_example.py --run-client
```

### Run Detection & Tracking
```bash
python mainDetect.py --video input.mp4
```

## Dependencies
See [requirements.txt](requirements.txt) for core Python packages.
Additional setup:
- Download YOLO config/weights in `yolo/model/`.
- Place GOTURN prototxt and caffemodel in `goturn/nets/`.

