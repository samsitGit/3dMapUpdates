import requests
import random

# Base URL of the Flask server
base_url = 'http://127.0.0.1:5000'

def generate_unique_id():
    """Generate a random 64-bit integer."""
    return random.getrandbits(64)

def get_points_from_pcd_without_metadata(file_path):
    """
    Extracts points from a .pcd file, skipping the header metadata.
    """
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('DATA'):
                break
        points = [list(map(float, line.strip().split())) for line in file]
    return points

def generate_full_delete_tensor(points):
    """
    Generates a list of indices from 0 to len(points)-1.
    """
    return list(range(len(points)))

def upload_changes(additions, deletions=[]):
    """
    Sends additions with unique IDs and deletions to the server.
    """
    # Generate unique IDs for new points
    points_with_ids = [{"id": generate_unique_id(), "coordinates": point} for point in additions]
    data = {
        "additions": points_with_ids,
        "deletions": deletions
    }
    response = requests.post(f"{base_url}/upload", json=data)
    print("Upload response:", response.text)
    return [point['id'] for point in points_with_ids]  # Return generated IDs for future reference

def fetch_current_point_cloud():
    """
    Fetches the current state of the point cloud from the server and saves it to a .pcd file.
    """
    response = requests.get(f"{base_url}/fetch")
    if response.status_code == 200:
        points_with_ids = response.json()['points']
        save_points_to_pcd(points_with_ids, 'output_point_cloud.pcd')
        print("Fetched point cloud saved as 'output_point_cloud.pcd'")
        return [point['id'] for point in points_with_ids]  # Return generated IDs for future reference
    else:
        print("Failed to fetch point cloud:", response.text)
        return []

def save_points_to_pcd(points, file_path):
    """
    Saves a list of points to a .pcd file.
    """
    with open(file_path, 'w') as file:
        file.write("VERSION .7\n")
        file.write("FIELDS x y z\n")
        file.write("SIZE 4 4 4\n")
        file.write("TYPE F F F\n")
        file.write("COUNT 1 1 1\n")
        file.write(f"WIDTH {len(points)}\n")
        file.write("HEIGHT 1\n")
        file.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        file.write(f"POINTS {len(points)}\n")
        file.write("DATA ascii\n")
        for point in points:
            coord = point['coordinates']
            file.write(f"{' '.join(map(str, coord))}\n")

upload_changes(get_points_from_pcd_without_metadata('1/200.pcd'))
ids = upload_changes(get_points_from_pcd_without_metadata('1/300.pcd'))
upload_changes(get_points_from_pcd_without_metadata('1/400.pcd'))
# deleting 300th
upload_changes([],ids)
fetch_current_point_cloud()

# # delete all
# ids = fetch_current_point_cloud()
# upload_changes([],ids)
# fetch_current_point_cloud()