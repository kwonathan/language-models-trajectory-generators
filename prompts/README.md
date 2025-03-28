# Prompts

Here, we provide brief descriptions of each of the prompts used in our work.
All the prompts used for the main system can be found in this folder.

##### `main_prompt.py`:

This is the main prompt for the trajectory generation, and instructs the LLM to generate Python code which outputs a sequence of poses for the robot end-effector to follow, to complete the given task. It contains details such as available functions which the LLM can call, the environment set-up, and guidance on collision avoidance, velocity control, code generation and planning.

##### `print_output_prompt.py`:

This prompt is used if the LLM calls the `print` function while it is generating the code for the trajectory, and the printed outputs are provided directly to the LLM. The `print` function is also used in the `detect_object` API call, and this prompt will be used to provide the relevant object information automatically back to the LLM for further reasoning and trajectory generation.

##### `error_correction_prompt.py`:

This prompt is used if there is an error in the code generated by the LLM. If the LLM outputs multiple blocks of code, the block number where the error occurred and the error message are provided directly to the LLM, and it is instructed to correct the error.

##### `success_detection_prompt.py`:

Upon completing the given task, this prompt is provided to another instance of the LLM to determine if the task was completed successfully or not. The poses of the relevant objects over the duration of the task execution are appended to the end of this prompt.

##### `task_summary_prompt.py`:

If the LLM predicts that the task was not completed successfully, this prompt is used to output a summary of both the generated trajectory and the poses of the relevant objects, for the most recent and any previous failed episodes. This information is later added so that the same sequence of actions is avoided by the next instantiation of the LLM for subsequent attempts at the task.

##### `task_failure_prompt.py`:

This prompt, along with the `task_summary` information generated by the LLM, is appended to the end of the `main_prompt`, and this combined prompt is given to another instance of the LLM to generate an alternative plan and sequence of end-effector poses.
