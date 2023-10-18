# INPUT: [INSERT TASK SUMMARY]
TASK_FAILURE_PROMPT = \
"""SUMMARY OF PREVIOUS FAILED ATTEMPTS:
[INSERT TASK SUMMARY]

PROBLEM RESOLUTION:
Can you suggest what was wrong with the plans for the trajectories, and suggest specific changes that would be appropriate?
Then, replan and retry the task by continuing with INITIAL PLANNING 1.
"""
