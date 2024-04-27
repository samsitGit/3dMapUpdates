from flask import Flask, request, jsonify
import open3d as o3d
import numpy as np

app = Flask(__name__)

# This will hold our point cloud in memory
current_point_cloud = o3d.geometry.PointCloud()

@app.route('/upload', methods=['POST'])
def upload():
    data = request.json
    global current_point_cloud

    # Handle additions
    if 'additions' in data and len(data['additions']) > 0:
        new_points = o3d.utility.Vector3dVector(data['additions'])
        current_point_cloud.points.extend(new_points)

    # Handle deletions
    if 'deletions' in data and len(data['deletions']) > 0:
        current_points = np.asarray(current_point_cloud.points)
        points_to_delete = np.array(data['additions'])[data['deletions']]
        # Create a mask to keep points not in the deletions
        mask = np.ones(len(current_points), dtype=bool)
        for pt in points_to_delete:
            # Find points in the current cloud that match the deletion points
            matches = np.all(current_points == pt, axis=1)
            mask &= ~matches
        # Apply mask to keep only the points not deleted
        current_point_cloud.points = o3d.utility.Vector3dVector(current_points[mask])

    return jsonify({"message": "Changes applied successfully"}), 200

@app.route('/fetch', methods=['GET'])
def fetch():
    points = np.asarray(current_point_cloud.points).tolist()
    return jsonify({"points": points}), 200

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False) # set to false to avoid restarting during test
