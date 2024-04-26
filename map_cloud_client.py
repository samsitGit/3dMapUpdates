import requests
import datetime

# URL of the Flask server
base_url = 'http://127.0.0.1:5000'

# Test upload
response = requests.post(f"{base_url}/upload", json={
    "points": [[1, 2, 3], [4, 5, 6], [7, 8, 9]],
    "metadata": {
        "client_id": "client123",
        "location": "Some location",
        "time": str(datetime.datetime.now())
    }
})
print("Upload response:", response.text)

# Simulate waiting for new data
import time
time.sleep(1)

# Test fetch
response = requests.get(f"{base_url}/fetch", params={
    "client_id": "client123",
    "last_time": str(datetime.datetime.now() - datetime.timedelta(minutes=1))
})
print("Fetch response:", response.text)
