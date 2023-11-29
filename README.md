# Language Models as Zero-Shot Trajectory Generators

## Teyun Kwon, Norman Di Palo, Edward Johns

[The Robot Learning Lab](https://www.robot-learning.uk/), Department of Computing, Imperial College London

[[arXiv](https://arxiv.org/abs/2310.11604)] [[project page](https://www.robot-learning.uk/language-models-trajectory-generators)]

![example tasks](./assets/tasks.png)

In this work, we investigate if an LLM (GPT-4) can directly predict a dense sequence of end-effector poses for manipulation skills, when given access to only object detection and segmentation vision models, and without any in-context examples, motion primitives, or external trajectory optimisers, with only a single task-agnostic prompt.

This repository currently contains the full prompts, and the prompts used for ablation studies, along with an example LLM output.

## Getting Started

Clone the repository and initialise XMem submodule:
```
git clone --recurse-submodules https://github.com/kwonathan/language-models-trajectory-generators
```

Create a folder in `XMem` called `saves`. Download the `XMem.pth` model from the following website into the newly created folder:
```
https://github.com/hkchengrex/XMem/releases/tag/v1.0
```

Install the LangSAM model from the following website:
```
https://github.com/luca-medeiros/lang-segment-anything
```

Finally, create the `./images/trajectory` folder.

## Running the Code

The following command will start the simulator to run the system:
```
python main.py --robot franka
```
(The Sawyer robot will be added soon!)

## Citation

Please consider citing our work if you found it useful!
```bibtex
@misc{kwon2023language,
      title={Language Models as Zero-Shot Trajectory Generators}, 
      author={Teyun Kwon and Norman Di Palo and Edward Johns},
      year={2023},
      eprint={2310.11604},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
