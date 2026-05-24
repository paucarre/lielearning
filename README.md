# Introduction

This repository provides simple examples of using Lie Groups and Lie Algebras in robotics. 

It serves as supporting material for the video **[An informal introduction to Lie Groups for robotics](https://www.youtube.com/watch?v=xYMKfsnJ85c)**.

The repository contains two scripts:
- One for **pose interpolation**
- One for **pose averaging**

Both are visualized and animated using [Genesis](https://github.com/Genesis-Embodied-AI/genesis-world).  
The Lie group computations are handled by [PyPose](https://pypose.org/).


> **Note**: The scripts may run slowly because Genesis builds physics kernels during initialization (these are not actually used in this project).

> **Note**: The [urdf/drones](urdf/drones) folder contains [Genesis URDFs for drones](https://github.com/Genesis-Embodied-AI/genesis-world/tree/c81777a251e792a93d756aaa3204426486710abf/genesis/assets/urdf/drones)

# Setup

Install **UV** (if not already installed):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Install `git-lfs` (if not already installed):
```bash
sudo apt update && sudo apt install git-lfs
```

Create and set up the virtual environment:

```bash
uv sync
```

Activate the virtual environment:

```bash
source .venv/bin/activate
```

Pull `git-lfs`:
```bash (if not already pulled)
git lfs pull
```

## Pose Interpolation

This example corresponds to the first problem presented in the video.

You can interpolate between an initial and final pose, generating as many intermediate poses as desired.

**Usage:**

```bash
source .venv/bin/activate
python pose_interpolation.py --num_poses 6 --duration 8
```

You should see an animation similar to:

![Pose Interpolation](media/interpolation.gif)

## Pose Averaging

This example corresponds to the second problem presented in the video.

You can run as many optimization iterations as you like. The result of each iteration is displayed.

For the provided initial poses, **2 iterations** are usually sufficient. Feel free to experiment with different initial poses and iteration counts.

The input proposals appear in **orange**. The solutions begin in black during the first iteration and gradually converge (remaining black) in subsequent iterations.

**Usage:**

```bash
source .venv/bin/activate
python pose_averaging.py --optimization_iterations 2
```

You should see something similar to:

![Pose Averaging](media/averaging.png)

Zooming in on the averaged pose reveals the slight improvements between the first and second iterations:

![Pose Averaging](media/2_solutions.png)

Running with **3 optimization iterations** shows almost no visible improvement from the second to the third iteration:

![Pose Averaging](media/3_solutions.png)
