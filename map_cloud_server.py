from flask import Flask, request, jsonify
from square_root_decomposition import *
import datetime
import hashlib
import json
import random

app = Flask(__name__)

# Store commits in blocks
blocks = []
block_size = 8
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

    # Add commit to the latest block or create a new block if necessary
    if not blocks or len(blocks[-1].commits) >= block_size:
        blocks.append(CommitBlock())
    blocks[-1].add_commit(commit)

    return jsonify({"message": "Commit created", "ids": ids, "commit_hash": commit_hash}), 200

@app.route('/fetch', methods=['GET'])
def fetch():
    last_hash = request.args.get('last_hash')
    all_additions = {}
    all_deletions = set()

    # Find the block index for the last known hash
    found = False
    for block in blocks:
        for commit in block.commits:
            if commit['hash'] == last_hash:
                found = True
            if found:
                for addition in commit['additions']:
                    if addition['id'] not in all_deletions:
                        all_additions[addition['id']] = addition['coordinates']
                for deletion in commit['deletions']:
                    if deletion in all_additions:
                        del all_additions[deletion]
                    all_deletions.add(deletion)

    latest_hash = blocks[-1].commits[-1]['hash'] if blocks and blocks[-1].commits else None

    return jsonify({
        "additions": list(all_additions.values()),
        "deletions": list(all_deletions),
        "latest_commit_hash": latest_hash
    }), 200

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False) # set to false to avoid restarting during test