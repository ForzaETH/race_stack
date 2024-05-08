GUI applications in remote containers are a bit of a pain in the neck. Which makes GUI applications in remote VSCode devcontainers are even more of a pain.

## GUI applications in remote Docker containers
If you want to launch a GUI application from a remote docker container, as for example during [mapping](https://git.ee.ethz.ch/pbl/research/f1tenth2/race_stack/-/tree/master/base_system/pbl_f110_system?ref_type=heads#mapping), a couple of specific steps need to be taken. 

1. Connect to a car via SSH, enabling X forwarding with the `-X` flag: 
```bash
ssh -X <username>@<car_ip>
```

2.  Move to the ForzaETH race stack directory, and run the `xauth_setup.sh` script:
```bash
cd <racestack_directory>
source .devcontainer/.install_utils/xauth_setup.sh
```
You should get at least the first line of the following output:
```
non-network local connections being added to access control list
xhost:  must be on local machine to add or remove hosts.
```

3. run the container with the appropriate script. If had you never run the container, running `main_dock.sh` should be fine, otherwise consider attaching to the already built container with `main_attach_dock.sh`.
```bash
.docker_utils/main_dock.sh
```

4. Enjoy a terminal with GUI forwarding!


## GUI applications in remote VSCode devcontainers
Due to the complicatedness of how a VSCode container is spun up, connecting to GUI applications requires a bit more involvement and a secondary SSH connection, to which we can relay the X forwarding.
Due to this reason, both a terminal **and** VSCode need to be opened in the car. 

1. Connect to a car via SSH, enabling X forwarding with the `-X` flag: 
```bash
ssh -X <username>@<car_ip>
```

2.  Move to the ForzaETH race stack directory, and run the `xauth_setup.sh` script:
```bash
cd <racestack_directory>
source .devcontainer/.install_utils/xauth_setup.sh
```
You should get at least the first line of the following output:
```
non-network local connections being added to access control list
xhost:  must be on local machine to add or remove hosts.
```

3. Memorize the `DISPLAY` number in this SSH-connected terminal, after printing it to screen:
```bash
echo $DISPLAY
```

an example output could be 
```
localhost:10.0
```


4. open up the devcontainer on the car, first by opening up VSCode, then connecting to the car with the remote connection button to the bottom left (Connect to Host...), then open the race stack folder, and reopen in the devcontainer.

5. In the devcontainer terminal where you want to use the GUI application, export now the `DISPLAY` variable number. For example:
```bash
export DISPLAY=localhost:10.0
```
**Note**: use the full name as from the output of point 3.
 
6. Enjoy a terminal with GUI forwarding!

