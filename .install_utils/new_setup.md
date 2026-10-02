# Installation

The ForzaETH race stack is designed to be used in Linux-based environment; in order to provide the same experience on as many devices as possible, it is containerised with [Docker](https://www.docker.com/). 
## Dependencies
### Ubuntu

Download [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and also follow the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/). 

Although any code editor can be used with the stack, we recommend installing [Visual Studio Code](https://code.visualstudio.com/docs/setup/linux), as well as the [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) and [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extensions.

Then install the required packages (git, git-credential manager, make):
```bash
sudo apt update
sudo apt install git wget make 

wget https://github.com/git-ecosystem/git-credential-manager/releases/download/v2.7.0
/gcm-linux-x64-2.7.0.deb
sudo dpkg -i gcm-linux-x64-2.7.0.deb
git-credential-manager configure
git config --global credential.credentialStore secretservice
```

### MacOS

If not installed already, download the MacOS package manager [Homebrew](https://brew.sh/):
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Download [Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/), [Visual Studio Code](https://code.visualstudio.com/docs/setup/mac) as well as the [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) and [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extensions.

Then install the required packages (git, git credential manager, make, openssh, docker-cli):
```bash
brew update
brew install git make
brew install --cask git-credential-manager
brew install docker
```

## Windows (oh no...)

On Windows first install Ubuntu [Windows Subsystem for Linux](https://apps.microsoft.com/detail/9pdxgncfsczv?ocid=webpdpshare) (WSL2). 

Download and setup Docker Desktop [Windows](https://docs.docker.com/desktop/setup/install/windows-install/). Enable WSL integration by accessing the Docker dashboard and navigating to **Settings > Resources > WSL Integration** and selecting Ubuntu, and then clicking on **Apply & Restart**.

Download [Visual Studio Code](https://code.visualstudio.com/docs/setup/windows) as well as the [Docker](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker),  [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) and [WSL](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-wsl) extensions.

From here, open the Terminal app on MacOS and Ubuntu or the previously downloaded Ubuntu app for Windows (do not use PowerShell or Windows Terminal). You will need to setup a username and password.

Install [Git for Windows](https://gitforwindows.org/) and download the required packages in **the WSL / Ubuntu terminal**:
```bash
sudo apt update
sudo apt install git make
git config --global credential.helper "/mnt/c/Program\ Files/Git/mingw64/bin/git-credential-manager.exe"
```
## Stack installation

In terminal (WSL terminal for Windows), clone the repository:
```bash
git clone -b revamp https://github.com/ForzaETH/race_stack.git
```

Navigate to the the folder and run the setup:
```bash
cd race_stack
make setup
```

Open the the folder in Visual Studio Code
```bash
code .
```

and type `CTRL + SHIFT + P` (or `CMD + SHIFT + P` on MacOS) and select *Dev Containers: Rebuild Container*.

That's it! You're ready to go.
