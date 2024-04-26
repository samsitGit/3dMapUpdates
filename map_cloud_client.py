import requests
import datetime

# Base URL of the Flask server
base_url = 'http://127.0.0.1:5000'

def upload(file_path, client_id):
    """
    Upload a .pcd file to the server.
    """
    with open(file_path, 'rb') as file:
        files = {'file': (file_path, file, 'application/octet-stream')}
        response = requests.post(f"{base_url}/upload", files=files)
        if response.status_code == 201:
            print(f"Upload successful: {response.json()}")
        else:
            print(f"Failed to upload: {response.text}")

def fetch(client_id='default_client', dev=False, last_time=None):
    """
    Fetch updates from the server.
    If dev is True, last_time can be specified manually to simulate different client states.
    """
    if dev:
        if last_time is None:
            last_time = datetime.datetime.min.isoformat()  # Simulate a new client with no data
        params = {"client_id": client_id, "last_time": last_time}
    else:
        # Automatically set the last fetch time to a reasonable default or based on internal tracking
        last_fetch_time = datetime.datetime.now() - datetime.timedelta(minutes=5)  # Example: fetch last 5 minutes data
        params = {"client_id": client_id, "last_time": last_fetch_time.isoformat()}

    response = requests.get(f"{base_url}/fetch", params=params)
    if response.status_code == 200:
        print(f"Fetch successful: {response.json()}")
    else:
        print(f"Failed to fetch: {response.status_code} {response.text}")

# Example usage
upload('1/1.pcd', 'client1')
upload('1/100.pcd', 'client1')
upload('1/200.pcd', 'client1')

# Fetch for a new client
fetch(client_id='client1', dev=True, last_time=datetime.datetime.min.isoformat())

upload('1/300.pcd', 'client1')
upload('1/400.pcd', 'client1')

# Fetch with a specified previous state for testing
fetch(client_id='client1', dev=True, last_time="2024-04-10T12:00:00")
