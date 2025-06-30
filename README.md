# Image segmentation in isaac-sim based on SAM2

## Contents
- [Installation](#Installation)
- [Getting started](#Gettingstarted)
    - [chekpoints and yaml](#Downloadthecheckpoints)
	- [rosrun](#rosrun)
- [Examples](#Examples)
- [Citing SAM2](#CitingSAM2)
### Installation

SAM2 needs to be installed first before use. The code requires python>=3.10, as well as torch>=2.5.1 and torchvision>=0.20.1. For more information about SAM2 click [here](https://github.com/facebookresearch/sam2?tab=readme-ov-file).

```
git clone https://github.com/zyy1108/Isaac-sam2.git && cd Isaac-sam2

pip install -e .

cd /src

git clone https://github.com/facebookresearch/sam2.git && cd sam2

pip install -e .

```

### Getting started
#### Download the checkpoints
First, we need to download a model checkpoint. All the model checkpoints can be downloaded by running:
```
cd checkpoints && \

./download_ckpts.sh && \

cd ..
```
or individually from [SAM2](https://github.com/facebookresearch/sam2?tab=readme-ov-file).

After downloading, please modify the checkpoints path in the `<segment.py>` according to the local checkpoints and make sure that the `yaml` corresponds to the download checkpoints.
#### rosrun
Make sure that the ROS node information in the `<segment.py>` has been modified to what you need.

```
cd Isaac-sim

catkin-make

source devel/setup.bash

rosrun img_seg segment.py
```

### Examples
Some examples:

#### official:






#### wareroom:








### Citing SAM2
We use the Segment Anything Model 2 (SAM 2) for image/video segmentation tasks.
```
@article{ravi2024sam2,
  title={SAM 2: Segment Anything in Images and Videos},
  author={Ravi, Nikhila and Gabeur, Valentin and Hu, Yuan-Ting and Hu, Ronghang and Ryali, Chaitanya and Ma, Tengyu and Khedr, Haitham and R{\"a}dle, Roman and Rolland, Chloe and Gustafson, Laura and Mintun, Eric and Pan, Junting and Alwala, Kalyan Vasudev and Carion, Nicolas and Wu, Chao-Yuan and Girshick, Ross and Doll{\'a}r, Piotr and Feichtenhofer, Christoph},
  journal={arXiv preprint arXiv:2408.00714},
  url={https://arxiv.org/abs/2408.00714},
  year={2024}
}
```

