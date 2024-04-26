from flask import Flask, request, jsonify, send_from_directory
import open3d as o3d
import os
from werkzeug.utils import secure_filename

app = Flask(__name__)

# Directory to store uploaded files
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER
app.config['MAX_CONTENT_LENGTH'] = 1 * 1024 * 1024  # 1 MB limit

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
        return jsonify({"message": "File uploaded successfully", "filename": filename}), 201
    else:
        return jsonify({"error": "Unsupported file format"}), 400

@app.route('/fetch', methods=['GET'])
def fetch():
    # Example of returning a static file (latest pcd)
    # In practice, you should handle selecting and sending the appropriate file
    pcd_files = os.listdir(app.config['UPLOAD_FOLDER'])
    pcd_files.sort(reverse=True)  # Assuming newer files have later names
    if pcd_files:
        return send_from_directory(app.config['UPLOAD_FOLDER'], pcd_files[0])
    else:
        return jsonify({"error": "No data available"}), 404

if __name__ == '__main__':
    app.run(debug=True)