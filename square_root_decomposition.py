import math

class CommitBlock:
    def __init__(self):
        self.commits = []
        self.aggregated_additions = {}
        self.aggregated_deletions = set()

    def add_commit(self, commit):
        # Add the new commit to the block
        self.commits.append(commit)
        
        # Update the aggregated data
        for addition in commit['additions']:
            if addition['id'] not in self.aggregated_deletions:
                self.aggregated_additions[addition['id']] = addition['coordinates']
        
        for deletion in commit['deletions']:
            if deletion in self.aggregated_additions:
                del self.aggregated_additions[deletion]
            self.aggregated_deletions.add(deletion)

    def get_aggregated_results(self):
        # Returns the current state of aggregated additions and deletions
        return list(self.aggregated_additions.values()), list(self.aggregated_deletions)
