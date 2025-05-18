# Control Strategies for Pursuit-Evasion Under Occlusion

## Overview
[Project link](https://existentialrobotics.org/VisibilityControl/)

## Models (Optional)
The MPT models included here are pretrained from the work described in [Motion Planning Transformer](https://arxiv.org/abs/2106.02791). These models will be placed under `pursuer/planner/models` during the setup process.

The Motion Planning Transformers code base is currently for reviewing purpose alone. Please do not distribute.

## Setup
1. Ensure Anaconda (or Miniconda) is installed on your system and available in your terminal.
2. From the root of this repository, make the setup script executable with:
   `chmod +x setup.sh`
3. Run: 
4. `./setup.sh`

This creates a new Conda environment named `erl_vc`, installs the necessary packages and downloads the models.

## Usage
After running `setup.sh`, activate the environment:
`conda activate erl_vc`

You can then run:
`python main.py`
to start the simulation.

## Notes
- If `gdown` is missing, the script installs it automatically to facilitate model downloads.
- If `requirements.txt` is not present, the script skips dependency installation.
- The environment can be removed by running:
  conda remove --name erl_vc --all
- Path planning and CARLA-ROS integration is still under migrating from old code base and not available at this time.

