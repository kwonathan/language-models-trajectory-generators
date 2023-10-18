# INPUT: NONE
TASK_SUMMARY_PROMPT = \
"""The task was not completed successfully, and it needs to be replanned and retried. Can you:
1. Summarise the trajectory executed on the robot, specifying key waypoint end-effector poses (positions, orientations, and gripper states) along the trajectory, and describing their relevance with respect to the given task.
2. Summarise the most recent positions, orientations, and dimensions of each of the relevant objects.
3. If applicable, include the same details for any given summaries of previous failed attempts as well.
"""
