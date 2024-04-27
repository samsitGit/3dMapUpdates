from flask import Flask, request, jsonify

app = Flask(__name__)

# This will hold our point cloud with unique IDs
point_cloud = {}

@app.route('/upload', methods=['POST'])
def upload():
    data = request.json
    global point_cloud

    # Handle additions
    for item in data['additions']:
        point_id = item['id']
        point_coordinates = item['coordinates']
        point_cloud[point_id] = point_coordinates  # Store point with unique ID

    # Handle deletions
    if 'deletions' in data:
        for point_id in data['deletions']:
            if point_id in point_cloud:
                del point_cloud[point_id]  # Delete the point by ID

    return jsonify({"message": "Changes applied successfully"}), 200

@app.route('/fetch', methods=['GET'])
def fetch():
    points = [{"id": point_id, "coordinates": coordinates} for point_id, coordinates in point_cloud.items()]
    return jsonify({"points": points}), 200

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False) # set to false to avoid restarting during test