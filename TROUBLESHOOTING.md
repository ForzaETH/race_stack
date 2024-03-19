# TROUBLESHOOTING

## `catkin clean` fails in the docker cointainer setup
As the `build`, `devel`, and `logs` foder are mounted from outside, trying to run `catkin clean` will fail. This however is not a problem, as only the deletion of the actul folders is prevented. The packages are anyway actually cleaned by the command `catkin clean`. 

## Running `./main_dock.sh` returns error `Error response from daemon: Conflict. The container name "/forzaeth_devcontainer" is already in use by container ...`
The container is already running, if you want to attach to it you can use the [`./main_attach_dock.sh`](.docker_utils/main_attach_dock.sh) script. If you want to completely remove it and restart it you can do it by removing it with the command `docker rm forzaeth_devcontainer` and then running the [`./main_dock.sh`](.docker_utils/main_dock.sh) script.
This error might also happen when trying to run the `.devcontainer`, in that case the removing solution explained above is still valid.

## Running `./sec_dock.sh` returns error `Error response from daemon: Container ... is not running`
The container is not running, if already run you can restart it with the [`./main_attach.sh`](.docker_utils/main_attach_dock.sh) otherwise you can start it with the [`./main_dock.sh`](.docker_utils/main_dock.sh) script.

