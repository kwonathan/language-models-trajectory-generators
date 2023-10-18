# INPUT: [INSERT TASK]
SUCCESS_DETECTION_PROMPT = \
"""You are tasked with determining whether a user command was completed successfully or not, based on how the positions and orientations of the relevant objects in the environment changed during the execution of the task.

The 3D coordinate system of the environment is as follows:
    1. The x-axis is in the horizontal direction, increasing to the right.
    2. The y-axis is in the depth direction, increasing away from you.
    3. The z-axis is in the vertical direction, increasing upwards.
The position values are in metres.

The objects can rotate about the z-axis, from -pi to pi radians.
Negative rotation values represent clockwise rotation, and positive rotation values represent anticlockwise rotation. The rotation values are in radians.

The user command is "[INSERT TASK]".

1. Given the user command, describe how the object positions and orientations should have changed during the execution of the task.
2. From the given positions and orientations of the relevant objects, output whether the task was completed successfully or not.
3. If the task was completed successfully, output
```python
task_completed()
```.
4. If the task was not completed successfully, output
```python
task_failed()
```.
Do not define the task_completed and task_failed functions yourself.

The positions and orientations of the relevant objects in the environment are as follows:
"""
