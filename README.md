# Multiverse

# Multiverse-Framework

A powerful framework for robotics and 3D scene representation leveraging OpenUSD.

---

## Features

- Modular robotics control and simulation  
- Integration with OpenUSD for advanced 3D scene handling  
- Flexible architecture for extension and customization  

---

## Installation Guide

Follow these steps to install the Multiverse-Framework and its dependencies, including OpenUSD.

### Prerequisites

Make sure you have the following installed:

- Python 3.8+ and pip  
- Git  
- CMake (3.18 or higher)  
- A C++ compiler (gcc/g++ recommended)  
- Python development headers (e.g., `python3-dev` on Ubuntu)  
- Additional dependencies like Boost, TBB, etc., required by OpenUSD (see below)  

---

### Step 0: Create a virtual environment (recommended)

It is highly recommended to use virtual environment for Multiverse Framework.

```bash
python3 -m venv venv
source path/to/venv/bin/activate
echo 'alias activate_venv='source path/to/venv/bin/activate' >> ~/.bashrc 
```

### Step 1: Clone the repository

```bash
git clone https://github.com/mitsav01/Multiverse-Framework.git --recursive
cd Multiverse-Framework
```

### Step 2: Install required Python packages

```bash
pip install -r requirements.txt
```

### Step 3: Install OpenUSD

Although, it is given in setup.sh file to clone and install OpenUSD repo. I've cloned and installed repo by myself.

```bash
git clone https://github.com/PixarAnimationStudios/OpenUSD.git 
mkdir OpenUSD/install
chmod -R +x OpenUSD
```
To integrate Multiverse with OpenUSD, we need to specify OpenUSD's source and install directory. I did it by specifying them in `.bashrc` file.

```bash
echo 'export USD_SRC_DIR=path/to/OpenUSD' >> ~/.bashrc 
echo 'export USD_INSTALL_DIR=path/to/OpenUSD/install' >> ~/.bashrc
```
### Step 4: Running Installation script

Now, after following this steps.
We can go to any individual package of Multiverse and execute ```install.sh```
I did it with ```Multiverse-Knowledge```.

```bash
cd path/to/Multiverse/Multiverse-Knowledge
./install.sh
```
The installation of ```OpenUSD``` will take really long (~15-20 minutes). At the end of installation, we'll have an output showing that Installation finished successfully and we'll get a message to add some stuffs to ```PATH``` and ```PYTHONPATH``` variable. 

```bash
echo 'export PATH=$PATH:path/to/OpenUSD/install/bin' >> ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:path/to/OpenUSD/install/lib/python' >> ~/.bashrc
```
### Step 5: Building workspace

It is also important to build and source `ros_ws` or `ros2_ws` everytime, we use this framework.
We can build the `ros2_ws` with below command:

```bash
cd /path/to/Multiverse/Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/ros_ws/multiverse_ws2
colcon build --symlink-install
source install/setup.bash
````
In order to make it easy, we can add following line to `~/.bashrc`:

```bash
source /path/to/Multiverse/Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/ros_ws/multiverse_ws2/install/setup.bash
```

### Step 6: Verifying Installation

After successful execution of ```install.sh```, we can verify the installation of 'Multiverse' framework using below command:

```bash
cd /path/to/Multiverse/Multiverse-Launch/bin
multiverse_launch
```
You'll see four different windows showing an example of 'Table with ball and Square' in simulation. 
Note: In order to save the time to write whole command to launch `multiverse_launch`, we can use following command.

```bash
echo 'alias multiverse_launch='bash /path/to/Multiverse/Multiverse-Launch/bin/multiverse_launch'' >> ~/.bashrc
```
Everytime, we can launch 'Multiverse' just by writing `multiverse_launch` in the terminal.


## Citation

If you find this software helpful, please cite it as follows:

```bibtex
@software{Multiverse,
  author = {Giang Nguyen, Michael Beetz},
  title = {Multiverse},
  month = {December},
  year = {2024},
  url = {https://github.com/Multiverse-Framework/Multiverse},
  doi = {https://doi.org/10.5281/zenodo.14035537}
}
```
