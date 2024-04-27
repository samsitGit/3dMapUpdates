from flask import Flask, request, jsonify
import datetime
import hashlib
import json
import random

app = Flask(__name__)

# Store the commits
commits = []

def generate_unique_id():
    """Generate a random 64-bit integer."""
    return random.getrandbits(64)

def generate_hash(data):
    """ Generate a unique hash for each commit based on the data and the current time. """
    hash_data = json.dumps(data, sort_keys=True) + datetime.datetime.utcnow().isoformat()
    return hashlib.sha256(hash_data.encode()).hexdigest()

@app.route('/upload', methods=['POST'])
def upload():
    data = request.json
    ids = [generate_unique_id() for _ in data.get('additions', [])]
    commit_hash = generate_hash({"additions": ids, "deletions": data.get('deletions', [])})
    commit = {
        "timestamp": datetime.datetime.utcnow().isoformat(),
        "additions": [{"id": id, "coordinates": point} for id, point in zip(ids, data.get('additions', []))],
        "deletions": data.get('deletions', []),
        "hash": commit_hash
    }
    commits.append(commit)
    return jsonify({"message": "Commit created", "ids": ids, "commit_hash": commit_hash}), 200

@app.route('/fetch', methods=['GET'])
def fetch():
    last_hash = request.args.get('last_hash')
    index = next((i for i, commit in enumerate(commits) if commit['hash'] == last_hash), -1)

    # Aggregate changes from the commit after the last known to the latest
    additions = []
    deletions = []
    latest_hash = commits[-1]['hash'] if commits else None
    
    for commit in commits[index + 1:]:
        # Aggregate additions and deletions
        additions.extend(commit['additions'])
        deletions.extend(commit['deletions'])

    # Optimize: remove redundant changes
    final_additions = [add for add in additions if add['id'] not in deletions]
    final_deletions = [del_id for del_id in deletions if del_id not in [add['id'] for add in additions]]

    return jsonify({
        "additions": final_additions, 
        "deletions": final_deletions, 
        "latest_commit_hash": latest_hash
    }), 200

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False) # set to false to avoid restarting during test