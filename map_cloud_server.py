from flask import Flask, request, jsonify
import open3d as o3d
import json
import datetime

app = Flask(__name__)

# Storage for point clouds and metadata
point_clouds = []
last_updated = {}

def process_point_cloud(data):
    # Convert list of points to Open3D point cloud
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(data["points"])
    return pc

@app.route('/upload', methods=['POST'])
def upload():
    data = request.json
    timestamp = datetime.datetime.now()

    pc = process_point_cloud(data)
    point_clouds.append({"timestamp": timestamp, "point_cloud": pc, "metadata": data["metadata"]})
    last_updated[data["metadata"]["client_id"]] = timestamp

    return jsonify({"message": "Data uploaded successfully", "timestamp": str(timestamp)})

@app.route('/fetch', methods=['GET'])
def fetch():
    client_id = request.args.get('client_id')
    last_time = datetime.datetime.fromisoformat(request.args.get('last_time'))

    updates = []
    for entry in point_clouds:
        if entry["timestamp"] > last_time:
            updates.append({
                "timestamp": entry["timestamp"],
                "points": entry["point_cloud"].points,
                # Add other necessary details
            })

    return jsonify({"updates": updates})

if __name__ == '__main__':
    app.run(debug=True)
