from flask import Flask, request, jsonify, send_from_directory
import open3d as o3d
import os
from werkzeug.utils import secure_filename
import datetime

app = Flask(__name__)

# Directory to store uploaded files
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['MAX_CONTENT_LENGTH'] = 1 * 1024 * 1024  # 1 MB limit

# Simulating a database
uploads = []

@app.route('/upload', methods=['POST'])
def upload():
    if 'file' not in request.files:
        return jsonify({"error": "No file part"}), 400
    file = request.files['file']
    if file.filename == '':
        return jsonify({"error": "No selected file"}), 400
    if file and file.filename.endswith('.pcd'):
        filename = secure_filename(file.filename)
        file_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(file_path)
        # Save metadata with timestamp
        uploads.append({"timestamp": datetime.datetime.now(), "filename": filename})
        return jsonify({"message": "File uploaded successfully", "filename": filename}), 201
    else:
        return jsonify({"error": "Unsupported file format"}), 400

@app.route('/fetch', methods=['GET'])
def fetch():
    client_id = request.args.get('client_id')
    last_time = request.args.get('last_time')
    if last_time:
        last_time = datetime.datetime.fromisoformat(last_time)
    else:
        last_time = datetime.datetime.min

    new_uploads = [u for u in uploads if u["timestamp"] > last_time]
    files = []
    for upload in new_uploads:
        path = os.path.join(app.config['UPLOAD_FOLDER'], upload['filename'])
        files.append(path)

    return jsonify({"files": files})

if __name__ == '__main__':
    app.run(debug=True)
