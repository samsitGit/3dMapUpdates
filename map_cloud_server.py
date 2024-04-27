from flask import Flask, request, jsonify
import datetime
import random

app = Flask(__name__)

# This will hold our point cloud with unique IDs
point_cloud = {}

def generate_unique_id():
    """Generate a random 64-bit integer."""
    return random.getrandbits(64)

@app.route('/upload', methods=['POST'])
def upload():
    data = request.json
    global point_cloud
    ids = []

    # Handle additions
    point_upload_time = datetime.datetime.utcnow().isoformat()
    for point in data['additions']:
        point_id = generate_unique_id()
        point_cloud[point_id] = {"coordinates": point, "upload_time": point_upload_time}
        ids.append(point_id)

    # Handle deletions
    if 'deletions' in data:
        for point_id in data['deletions']:
            if point_id in point_cloud:
                del point_cloud[point_id]

    return jsonify({"message": "Changes applied successfully", "ids": ids}), 200

@app.route('/fetch', methods=['GET'])
def fetch():
    points = [{"id": point_id, "coordinates": data["coordinates"], "upload_time": data["upload_time"]} 
              for point_id, data in point_cloud.items()]
    return jsonify({"points": points}), 200

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False) # set to false to avoid restarting during test