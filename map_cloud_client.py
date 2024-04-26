import requests

# URL of the Flask server
base_url = 'http://127.0.0.1:5000'

# Specify the path to the .pcd file
file_path = '1/100.pcd'

# Test upload
with open(file_path, 'rb') as file:
    files = {'file': (file_path, file, 'application/octet-stream')}
    response = requests.post(f"{base_url}/upload", files=files)
print("Upload response:", response.text)

# Test fetch and save the fetched file
response = requests.get(f"{base_url}/fetch", stream=True)
if response.status_code == 200:
    filename = response.headers.get('Content-Disposition').split("filename=")[1].strip('"')
    with open(filename, 'wb') as f:
        for chunk in response.iter_content(chunk_size=8192):
            f.write(chunk)
    print(f"File downloaded and saved as {filename}.")
else:
    print("Failed to fetch file:", response.status_code, response.text)