"""
Simple usage example for server and communication client.

Usage:
    # Start server: run FastAPI app
    python usage_example.py --run-server

    # Run client usage
    python usage_example.py --run-client
"""

import argparse
import threading


def run_server() -> None:
    """Run the FastAPI server using Uvicorn."""
    import uvicorn
    from server import app

    uvicorn.run(app, host="127.0.0.1", port=8000)


def client_usage() -> None:
    """Demonstrate usage of the Communication client."""
    from communication import Communication
    from datetime import datetime

    # Initialize client
    client = Communication(
        team_number=199,
        server_url="http://127.0.0.1:8000"
    )

    # Login
    client.login(username="sufaiprojeekibi", password="61aynprs23")

    # Fetch server time
    server_time = client.get_time_from_server()
    print(f"Server time: {server_time}")

    # Fetch QR coordinates
    coords = client.get_coordinates()
    print(f"QR coordinates: {coords}")

    # Connect to UAV (SITL or real drone)
    vehicle = client.connect_drone(connection_str="127.0.0.1:14570", baud=57600)

    # Send telemetry
    telemetry_response = client.send_telemetry(
        vehicle=vehicle,
        lock_status=False,
        msg=(0, 0, 0, 0)  # placeholder message tuple
    )
    print(f"Telemetry response: {telemetry_response.status_code}")

    # Send lock info example
    now = datetime.now()
    lock_response = client.send_lock_info(
        start_time=now,
        end_time=now,
        autolock=False
    )
    print(f"Lock info response: {lock_response.status_code}")

    # Send kamikaze info example
    kamikaze_response = client.send_kamikaze(
        start_time=now,
        end_time=now,
        qr_text="TEST_QR"
    )
    print(f"Kamikaze info response: {kamikaze_response.status_code}")

    client.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-server", action="store_true")
    parser.add_argument("--run-client", action="store_true")
    args = parser.parse_args()

    if args.run_server:
        run_server()
    if args.run_client:
        client_usage()


if __name__ == "__main__":
    main()
