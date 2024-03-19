## Instructions


**Note**: Throughout this README, NUC2 refers to a generic physical car.

### Car connection

The WAN can then be used to connect to the car. For instance, to connect to NUC2:
`ssh <car_username>@<car_fixed_IP>`

NOTE: 
It only works if the car is on.

### SSH connections - basic info

To connect to the car you will use the Secure Shell Protocol (SSH) with the homonym UNIX command `ssh`.
It will create a secure and encrypted connection to the machine. 

To automatically login into the selected host, you can copy one of your public ssh keys with the single-liner:

    ssh-copy-id -i <path to your public key> <car_username>@<car_fixed_IP>
For instance, if you created the ssh key in the default location and you want to automatically authenticate to the user `<car_username>` in NUC2, you should use the following command:

    ssh-copy-id -i ~/.ssh/id_rsa.pub <car_username>@<car_fixed_IP>
After this, running the command `ssh <car_username>@<car_fixed_IP>` will automatically login into the user without the need of a password.

### X11 forwarding
The X Window System (X11, or simply X) is the windowing system that basically create and manages the graphical interfaces on most Ubuntu installations. 
For the work on F1TENTH, we will need to forward X from the car onto the PC you are using, as it is needed for mapping.
Doing that is very simple, as it only consists in adding the flag `-X` to the `ssh` command. Adding the additional flag `-C` for compression is suggested:

    ssh -XC <car_username>@<car_fixed_IP>

### SSH and VSCode
VSCode provides you with very easy integration with ssh connections. 
If you want to access the filesystem of the car with VSCode, so to be able to modify the files on the car with the capabilities of VSCode, you'll first need to install the Remote-SSH extension, with the extensions window. 
Then a green symbol will appear in the lower left corner. You can click that to start an ssh connection to a specified server. 
You can also save preferred hosts by clicking on `Open SSH Configuration File...` and then saving the hosts in a way similar to this:

    Host racecrew@NUC2
      ForwardX11 yes
      Compression yes
      HostName <car_fixed_IP>
      User <car_username>

    ...

After saving such a config file, you can also run `ssh racecrew@NUC2` and it will connect you to the car with the specified settings (e.g. X11 forwarding enabled)

### Bash Aliases
You can add the following to your bash configuration file for convenience. There are also other aliases for Pit Starter convenience, take a look on the [Pit Usage Guide](./PitUsage.md).
```bash
# aliases to get into cars faster
alias sshnuc2="ssh -XC <car_username>@<car_fixed_IP>"
alias sshnuc3="ssh -XC <another_car_username>@<another_car_fixed_IP>"
...
```

---
[Go back to the checklists index](./README.md)