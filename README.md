# T-REG
Make a T-Rex walk!
Please see this [document](https://hackmd.io/Q8IJhiZXRCK4nl1kpiCvUA) for a general overview of the project.

This Repo contains two subdirectories:

## Repo 1: T-REG-RL
Playground for using reinforcement learning (via the unity-ml-agents package) to make the T-REX walk

### T-REG-RL Get Started
- Install [Unity Hub](https://unity.com/download) (includes signing up to Unity personal)
- Shallow clone the `ml-agents` repo `git clone --depth=1 --branch release_20 https://github.com/Unity-Technologies/ml-agents.git`
- Create and activate a conda environment (e.g., python 3.10) using the command: `conda create -n treg python=3.10.8`
- Follow [this instructions](https://github.com/Unity-Technologies/ml-agents/blob/release_20/docs/Installation.md) to complete installation. See the instructions summary below:

```
pip install torch -f https://download.pytorch.org/whl/torch_stable.html
pip install -e ./ml-agents-envs
pip install -e ./ml-agents
pip install protobuf==3.20
```

- Download or clone this repo (e.g., `git clone git@github.com:alan-turing-institute/T-REG.git`)

To run a sample simulation, follow the instructions below
- Navigate to the T-REG repo and run the simulation. Use the command below:
```
cd T-REG
mlagents-learn config/ppo/TReg.yaml
```

## Repo 2: T-REG-HMI
Have users control the T-Rex limbs
