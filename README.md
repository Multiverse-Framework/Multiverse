# 🌌 Multiverse

Decentralized Simulation Framework designed to integrate multiple advanced physics engines along with various photo-realistic graphics engines to simulate everything, everywhere, all at once.

https://github.com/user-attachments/assets/2690373a-e4fa-46be-befc-d1139e13e5d5

## 🛠 Installation

> Make sure to use a **virtual environment**. You can choose from [conda](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html), [virtualenv](https://virtualenv.pypa.io/en/latest/), or [virtualenvwrapper](https://virtualenvwrapper.readthedocs.io/en/latest/).

---

### 1. Clone the Repository

Clone the repository with all submodules (shallow clone):

```bash
git clone --recurse-submodules https://github.com/Multiverse-Framework/Multiverse --depth 1
cd Multiverse
```

> Alternatively, if you already cloned the repo without `--recurse-submodules`, run:
>
> ```bash
> git submodule update --init --recursive
> ```

---

### 2. Install Multiverse Launch and Multiverse Utilities

```bash
pip install -r ./Multiverse-Launch/requirements.txt
pip install -r ./Multiverse-Utilities/requirements.txt
```

---

### 3. Install Simulators Connector

> Choose the connector depending on the simulator you're using. Default: **MuJoCo**.

```bash
pip install -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_simulators_connector/src/mujoco_connector/requirements.txt
```

---

### 4. Install ROS Connector

```bash
pip install -r ./Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/requirements.txt
source /opt/ros/$ROS_DISTRO/setup.bash
cd ./Multiverse-Launch/src/multiverse_connectors/multiverse_ros_connector/ros_ws
```

* For **ROS 1**:

  ```bash
  cd multiverse_ws
  catkin build
  ```

* For **ROS 2**:

  ```bash
  cd multiverse_ws2
  colcon build --symlink-install
  ```

---

### 5. (Optional) Install Multiverse Parser

```bash
pip install -r ./Multiverse-Parser/requirements.txt
cd Multiverse-Parser
./setup.sh
```

## 🧪 Test

To test the framework, run the default example:

```bash
./Multiverse-Launch/bin/multiverse_launch
```

This will spawn **4 instances of the MuJoCo simulator**. Each instance runs independently but remains interconnected with the others in real time — demonstrating the decentralized and modular nature of the framework.

---

## 📚 Citation

If you use this framework in your research, please cite:

```bibtex
@software{Multiverse,
  author = {Giang Nguyen and Michael Beetz},
  title = {Multiverse},
  month = {December},
  year = {2024},
  url = {https://github.com/Multiverse-Framework/Multiverse},
  doi = {10.5281/zenodo.14035537}
}
```
