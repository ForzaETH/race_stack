# Docker Structure and Guidelines
**Note:** Only installation through Docker is currently supported and maintained.

## Prerequisites
If using [Visual Studio Code](https://code.visualstudio.com/), install the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension (and the [WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl) extension on Windows).

### Ubuntu
Although the stack should be installable on most Linux distributions, the provided instructions are intended for Ubuntu.

Download [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and follow the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/).

Install the required packages (git, wget, make):
```bash
sudo apt update
sudo apt install git wget make
```

### MacOS
Download [Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/) and the MacOS package manager [Homebrew](https://brew.sh/):
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```
Homebrew installs the Xcode Command Line Tools, which include the required packages (git, make).

### Windows
Install Ubuntu through the [Windows Subsystem for Linux](https://apps.microsoft.com/detail/9pdxgncfsczv?ocid=webpdpshare) (WSL2).

Download and set up Docker Desktop for [Windows](https://docs.docker.com/desktop/setup/install/windows-install/). Enable WSL integration by accessing the Docker dashboard and navigating to **Settings > Resources > WSL Integration** and selecting Ubuntu, and then clicking on **Apply & Restart**.

Using a WSL terminal instance, install the required packages (git, make):
```bash
sudo apt update
sudo apt install git make
```
Clone the repository inside the WSL home directory (`~/`), not under `/mnt/c/`, as mounts from the Windows drive are slow and break file permissions.

### Jetson
Install [Docker Engine](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/setup_docker.html) following the Nvidia documentation and follow the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/). Check that the Nvidia runtime is available:
```bash
docker info | grep -i runtime  # should list nvidia
```

Install the required packages (git, wget, make):
```bash
sudo apt update
sudo apt install git wget make
```

## Installation steps

Clone the repository on the ros2-humble branch, preferably inside a dedicated folder:
```bash
git clone -b ros2-humble https://github.com/ForzaETH/race_stack.git
cd race_stack
```

Run the setup:
```bash
make setup
```
This creates the `race_stack_deps`, `race_stack_maps` and `race_stack_bags` folders next to the repository, holding the external repositories, maps and bags.

If using Visual Studio Code with Dev Containers, open the folder in the application and type `CTRL + SHIFT + P` and select `Dev Containers: Rebuild and Reopen in Container`. On newer versions of Docker and Docker Compose you might need to disable the Docker Buildx bake filesystem entitlement check. Make sure to restart Visual Studio Code so it picks up the new environment variable:
```bash
echo 'export BUILDX_BAKE_ENTITLEMENTS_FS=0' >> ~/.profile
echo 'export BUILDX_BAKE_ENTITLEMENTS_FS=0' >> ~/.bashrc
```

If not using Visual Studio Code, the containers can be built with:
```bash
make launch
```

## Structure
The structure of the deployed containers is defined in the [docker-compose.yaml](../docker-compose.yaml). Four containers are deployed:

- The main race stack container;
- The Zenoh router allowing node communication;
- The Foxglove bridge to enable visualization over websockets with Foxglove / Lichtblick;
- The noVNC container, for RViz and RQT visualizations in sessions without a display, such as on MacOS or over SSH without X forwarding.

VIO (visual-inertial odometry) is built into the main race stack container only if `USE_VIO=true` is set in the `.env` file during setup. It is recommended to leave this set to false unless VIO is needed, as the install process can be long.

## GUI Forwarding
The race stack requires GUI forwarding for some functionalities. This is being deprecated as it can be fragile, in favour of Foxglove / Lichtblick connecting to `ws://<host>:8765`.

The display is selected each time the container is opened, based on the session opening it:

| Session | GUI applications open on |
|---|---|
| Linux, locally | The host display |
| SSH with X forwarding | The display of the connecting machine |
| SSH without X forwarding, from any OS | noVNC |
| MacOS, locally | noVNC |

The selection is shared by all sessions of a user on that machine and the last one opened applies to new container terminals. Terminals that are already open keep their display.

When connected to a car from Linux, use X forwarding and update the display access of the container:
```bash
ssh -X <username>@<car_ip>
cd <race_stack folder>
make x11
```
When using Visual Studio Code, set `ForwardX11 yes` for the car in `~/.ssh/config` so the display is forwarded when opening the container; if it is not picked up, run `make x11` as above.

noVNC is available at [http://localhost:20501](http://localhost:20501). Visual Studio Code forwards port 20501 when connected to a car, so its applications are shown at the same address. Without Visual Studio Code, forward the port with `ssh -L 20501:localhost:20501 <username>@<car_ip>`.
