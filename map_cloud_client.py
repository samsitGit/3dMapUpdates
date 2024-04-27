import requests

# Base URL of the Flask server
base_url = 'http://127.0.0.1:5000'

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
    Sends additions and deletions to the server, handles both IDs and commit hashes.
    """
    data = {
        "additions": additions,
        "deletions": deletions
    }
    response = requests.post(f"{base_url}/upload", json=data)
    if response.status_code == 200:
        response_data = response.json()
        print("Upload response:", response_data)
        return response_data['ids'], response_data['commit_hash']
    else:
        print("Failed to upload:", response.text)
        return [], None

def fetch_updates(last_known_hash):
    """
    Fetches updates from the server and saves additions and deletions to separate .pcd files.
    Includes handling of the latest commit hash.
    """

    response = requests.get(f"{base_url}/fetch", params={"last_hash": last_known_hash})
    if response.status_code == 200:
        updates = response.json()

        # send additions to a file for visualizing
        if 'additions' in updates:
            save_points_to_pcd(updates['additions'], 'output_point_cloud.pcd')
        
        print(f"Updates fetched and saved")
        return updates
    else:
        print("Failed to fetch updates:", response.text)
        return None

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

# # adding frame 200, 300, 400
# ids1, commit_hash1 = upload_changes(get_points_from_pcd_without_metadata('1/200.pcd'))
# ids2, commit_hash2 = upload_changes(get_points_from_pcd_without_metadata('1/300.pcd'))
# ids3, commit_hash3 = upload_changes(get_points_from_pcd_without_metadata('1/400.pcd'))
# # deleting 300th
# upload_changes([], ids2)

# print(commit_hash1, commit_hash2, commit_hash3)

# try fetching from commit hash 1, 2, 3 and so on
results = fetch_updates('179b85ab24f35786b399851d06dd52555cb2b2d8991dda1236698ba528094c50')
commit_hash = results['latest_commit_hash']