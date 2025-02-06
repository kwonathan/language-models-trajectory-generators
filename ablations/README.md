# Prompt Ablations

Here, we provide brief descriptions of each of the prompt ablations used in our work.
All the ablated prompts used for the ablation studies can be found in this folder.
The diffs between the ablated prompts and the full main prompt can be examined by clicking on the commit message.

## Main Prompt Ablations

##### `collisions_phrase.py`:

This ablated prompt studies the collision avoidance capabilities of the LLM, and the effect of removing a specific phrase which we noticed during our investigation was being used frequently by the LLM for its internal reasoning ("clear objects and the tabletop"). The results can be seen in Figure 5 in the main paper (column "Remove prompt to clear objects and the tabletop to avoid collisions").

##### `collisions_section.py`:

This ablated prompt also studies the collision avoidance capabilities of the LLM, and the effect of removing the entire collision avoidance prompt. The results can be seen in Figure 5 in the main paper (column "Remove prompt for entire collision avoidance section").

##### `define_annotate_functions.py`:

This ablated prompt studies the coding capabilities of the LLM, and the effect of removing various guides when generating the code for the robot to execute. The results can be seen in Figure 5 in the main paper (column "Remove prompt to define reusable as well as specific functions, and annotate them").

##### `number_trajectory_variable.py`:

This ablated prompt also studies the coding capabilities of the LLM, and the effect of removing instructions to number each sub-trajectory step variable. The results can be seen in Figure 5 in the main paper (column "Remove prompt to name each trajectory variable with a number for smooth motion").

##### `generation_execution_step_by_step.py`:

This ablated prompt studies the step-by-step reasoning capabilities of the LLM, and the effect of removing instructions to generate and execute the code in small chunks so that any output can be taken into account when generating further code. The results can be seen in Figure 5 in the main paper (column "Remove prompt to break down the trajectory generation and execution into steps").

##### `lower_gripper.py`:

This ablated prompt also studies the step-by-step reasoning capabilities of the LLM, and the effect of removing instructions to include in its plan when to lower the gripper to make contact with an object. The results can be seen in Figure 5 in the main paper (column "Remove prompt to plan when to lower the gripper to make contact with the object").

##### `trajectory_step_by_step.py`:

This ablated prompt also studies the step-by-step reasoning capabilities of the LLM, and the effect of removing instructions to break down the trajectory into a sequence of sub-trajectory steps. The results can be seen in Figure 5 in the main paper (column "Remove prompt to break down the trajectory into steps").

##### `object_approach.py`:

This ablated prompt studies the object interaction capabilities of the LLM, and the effect of removing instructions to describe how best to approach the object. The results can be seen in Figure 5 in the main paper (column "Remove prompt to describe how best to approach the object").

##### `object_part.py`:

This ablated prompt also studies the object interaction capabilities of the LLM, and the effect of removing instructions to describe which part of the object would be most suitable for interaction. The results can be seen in Figure 5 in the main paper (column "Remove prompt to describe the part of the object most suitable for interaction").

##### `section_headings.py`:

This ablated prompt studies whether removing section headings of the long system prompt would affect performance, and the results can be seen in Figure 5 in the main paper (column "Remove prompt section headings").

##### `trajectory_chaining.py`:

This ablated prompt studies whether removing instructions to chain sub-trajectory steps such that the start point of sub-trajectory 2 is the same as the end point of sub-trajectory 1 and so on would affect performance. The results can be seen in Figure 5 in the main paper (column "Remove prompt to chain the trajectory for smooth motion").

##### `trajectory_shape.py`:

This ablated prompt studies the trajectory reasoning capabilities of the LLM, and the effect of removing instructions to describe the shape of the motion trajectory as internal reasoning. The results can be seen in Figure 5 in the main paper (column "Remove prompt to describe the shape of the motion trajectory").

## Action Output Prompt Ablations

##### `binary_output_gripper.py`:

This prompt investigates the optimal way for the LLM to control the gripper open or close action (as binary values or explicit functions). The results can be seen in Figure 8 E in the main paper.

##### `numbers_output_trajectory.py`:

This prompt investigates the optimal way for the LLM to output the sequence of end-effector poses (as a list of numerical values or as code for trajectory generation). The results can be seen in Figures 8 C and 8 D in the main paper.
