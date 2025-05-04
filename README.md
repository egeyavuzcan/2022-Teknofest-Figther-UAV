# 🛩️ Fighter UAV – Teknofest 2022

## 📖 Overview
This project implements a modular UAV control and tracking system for Teknofest 2022, structured as a Python package for easier development and extensibility.

- **🛠️ detection/**: Object detection and related scripts (e.g., `DetectAndTrack.py`, `mainDetect.py`, `main_Detect_Track.py`)
- **🎯 tracking/**: Object tracking modules and scripts (e.g., `mainGoturnTracker.py`)
- **🧰 utils/**: Helper utilities (e.g., `DetectAndTrackUtils.py`, `videoUtils.py`)
- **🌐 server/**: Server-side code and communication module (`server.py`, `communication.py`)
- **📄 usage_example.py**: Example usage of server & client.

You can now import modules from these packages in your scripts:
```python
from detection.DetectAndTrack import ...
from tracking.mainGoturnTracker import ...
from utils.videoUtils import ...
from server.server import ...
from server.communication import ...
```


## 🔄 Algorithm Flow
1. **Frame Acquisition**: capture from camera, video, or SITL via `videoUtils`.
2. **Object Detection**: YOLO detector produces bounding boxes.
3. **Tracker Initialization**: GOTURN tracker seeded on initial detection.
4. **Continuous Tracking**: tracker updates object location; periodic verification via YOLO.
5. **QR Code Reading**: optional QR detection to extract mission data.
6. **Communication Loop**: send telemetry, lock status, and kamikaze info to server via `Communication` client.

## 🛠️ Installation
### 📋 Prerequisites
- Python 3.8+

### 📦 Install dependencies
```bash
pip install -r requirements.txt
```

## 🚀 Usage
### 🌐 Start Server
```bash
uvicorn server:app --reload --host 0.0.0.0 --port 8000
```
Visit API docs at [http://localhost:8000/docs](http://localhost:8000/docs).

### 🤖 Run Client Example
```bash
python usage_example.py --run-client
```

### 🛠️ Run Detection & Tracking
```bash
python mainDetect.py --video input.mp4
```

## 🧩 Dependencies
See [requirements.txt](requirements.txt) for core Python packages.
Additional setup:
- Download YOLO config/weights in `yolo/model/`.
- Place GOTURN prototxt and caffemodel in `goturn/nets/`.

