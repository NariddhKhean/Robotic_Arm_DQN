# Robotic_Arm_DQN

> _Work in Progress (including the README.md file)_

Trains a _[Kuka LBR iiwa](https://www.kuka.com/en-au/products/robotics-systems/industrial-robots/lbr-iiwa)_ robotic arm through the reinforcement learning algorithm, deep Q-network, within the [PyBullet](https://pybullet.org/wordpress/) Simulation Environment.

![](media/environment.jpg)

> Yet to exhibit intelligent motion. Steps forward:
> - Upgrade DQN learning algorithm to [Double-DQN](https://arxiv.org/abs/1509.06461), [Duelling-DQN](https://arxiv.org/abs/1511.06581), or both.
> - Upgrade CNN to [CapsNet](https://arxiv.org/abs/1710.09829).
> - Tile joint state before the concatenate, as done in [this](https://arxiv.org/abs/1603.02199) paper.

## Getting Started

I had some difficultly getting PyBullet up and running on my Windows machine, but here are the steps I took to make it work:
1. Download built tools, found [here](https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=BuildTools&rel=15). During the installation process, be sure to check:
    - Windows 10 SDK (10.0...)
    - Visuals C++ tools for CMake
    - C++/CLI Support
    - VC++ 2015.3 v14.00 (v40) toolset for desktop
2. According to [this](https://stackoverflow.com/questions/14372706/visual-studio-cant-build-due-to-rc-exe) article, copy the `rc.exe` and `rcdll.dll` files from the `C:/Program Files (x86)/Windows Kits/8.1/bin/x86` directory to `C:/Program Files (x86)/Microsoft Visual Studio 14.0/VC/bin`.
3. Upgrade python set up tools using `pip install --upgrade setuptools`.
4. Install PyBullet using `pip install pybullet`.

> Note: Please take these steps with a grain of salt. What may work for me might not for you. Also, I am sure I have a few of the dependencies already installed on my machine, so this is not a complete setup guide. Please refer to the documentation.

With PyBullet installed, the best place to get started is through the [PyBullet Quick Start Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.e27vav9dy7v6).

## `urdf_utils.py`

A collection of utility functions that can be used to test `*.urdf` robots. The original use case was to assure that the development of novel _[Kuka LBR iiwa](https://www.kuka.com/en-au/products/robotics-systems/industrial-robots/lbr-iiwa)_ end effectors were working as intended in the simulation environment.

### Functions

The functions currently available in the `urdf_utils.py` file are demonstrated using the _Kuka LBR iiwa_ robot arm. Full examples of how to use the functions are shown below.

> Note: these functions have only been tested with the `kuka_iiwa/model.urdf` file and a handful of other `*.urdf` files.

### `find_joint_count()`
Counts the number of joints in robot, as defined in the `*.urdf` file.

| Arguments | Description | Default |
| --------- | ----------- | ------- |
| `ID`      | An integer representing a robot, as return from the `p.loadURDF()` method. | |

#### Example
```python
import pybullet as p
import pybullet_data

import urdf_utils

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka = p.loadURDF('kuka_iiwa/model.urdf', [0, 0, 0], useFixedBase=True)

kuka_joint_count = urdf_utils.find_joint_count(ID=kuka)

print(kuka_joint_count)
```

Returns:
```shell
7
```

### `test_movement_extents()`
Iterates through each joint and rotates them to the extent of its movement. Returns a list of tuples representing the minimum and maximum extents of rotation for each joint.

| Arguments     | Description | Default |
| ------------- | ----------- | ------- |
| `ID`          | An integer representing a robot, as return from the `p.loadURDF()` method. | |
| `joint_count` | An integer representing the number of joints as defined in the `*.urdf` file. Can be calculated with the `find_joint_count()` function. | |
| `speed`       | A number representing the rate at which each link rotates. | 0.2 |
| `precision`   | An integer representing the number of decimal places to which the extents will be calculated to. | 5 |
| `delay`       | A number that artificially delays the simulation steps with `time.sleep(delay)`. | 0.001 |

#### Example
```python
import pybullet as p
import pybullet_data

import urdf_utils

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka = p.loadURDF('kuka_iiwa/model.urdf', [0, 0, 0], useFixedBase=True)

kuka_joint_count = urdf_utils.find_joint_count(ID=kuka)

kuka_extents = urdf_utils.test_movement_extents(
    ID=kuka,
    joint_count=kuka_joint_count,
    speed=0.5,
    precision=7,
    delay=0.002
)

print(kuka_extents)
```

Returns:
```shell
[
    (-169.9999682239179, 169.9999530056643),
    (-119.9999695404722, 119.9999487814800),
    (-169.9999682080224, 169.9999595191940),
    (-119.9999589600828, 119.9999484192288),
    (-169.9999686172800, 169.9999593911397),
    (-119.9999536575531, 119.9999615657730),
    (-174.9999600597941, 174.9999496216755)
]
```

> #### More to come (when needed)...