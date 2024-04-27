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
    additions = np.array(data['additions'])
    if len(additions) > 0:
        new_points = o3d.utility.Vector3dVector(additions)
        current_point_cloud.points.extend(new_points)

    # Handle deletions
    deletions = data['deletions']
    if len(deletions) > 0:
        current_point_cloud.points = o3d.utility.Vector3dVector(np.delete(np.asarray(current_point_cloud.points), deletions, axis=0))

    return jsonify({"message": "Changes applied successfully"}), 200

@app.route('/fetch', methods=['GET'])
def fetch():
    return jsonify({"points": np.asarray(current_point_cloud.points).tolist()}), 200

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False) # set to false to avoid restarting during test