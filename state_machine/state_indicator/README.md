## Overview

The state indicator contains two parts: a .ino file which is flashed on the microcontroller (MCU) board and a ROS node that runs parallelly on the onboard computer (OBC). The ROS node handles all heavy computations and publishes the needed topics in a lightweight format to the MCU, which in turn commands the LED ring based on this information.

Two modes for visualizing data on the state indicator are available.
- Mode 1 (default): STATE_MODE for visualization of the current state from the state machine
    - Red: Follow the gap controller
    - Green: GB optimizer
    - Blue: Trailing
    - Blinking RGB: Overtaking


- Mode 2: SPEED_STEERING_MODE for visualization of telemetry data
    - LEDs on front: Visualization of the current speed. Green corresponds to slow, while red corresponds to high speed (parameter range: 0 - 7 m/s) 
    - LEDs on sides: Vizualization of the steering. Left steering corresponds to an activation of the LEDs on the left side and vice versa. The more intensive the blue light, the larger the steering angle (parameter range: -0.3 - 0 rad and 0 - 0.3 rad).

At any time the mode can be switched by typing *1* or *2* in the running terminal.



## Hardware setup

### Mounting
The mounting system consists of two parts: a socket that is mounted on an even surface on the racecar, as well as the housing part that protects the MCU and is the foundation for the sombrero hat. The sombrero can be fixed with double sided tape to the housing part.

On the following drive, the STL files of these parts are available for 3D printing:
`https://drive.google.com/drive/folders/1xYb2RIYk_Wn1eqLJk18Y0YzSIoeKPCN8`

### Electronics
The centerpiece of the state indicator is the MCU, which processes the data from the OBC. The board is connected to the OBC per USB cable and is wired with the LED ring on which the data is visualized. The utilized MCU is an **Arduino Micro** which offers a micro USB port for communication via rosserial with the OBC, as well as *pulse width modulation (PWM)* pins for communication with the LED ring. Three cables transmitting **GND**, **5V VCC** and **Data in (PWM)** are soldered on the corresponding pins on the MCU as well as on the LED ring.

## Software setup

### Setup on the microcontroller (MCU)
Plug in the MCU to your computer and flash the provided `.ino` file on it. Additionally, the `Adafruit_Neopixel` library needs to be installed on the MCU.


### Setup on the onboard computer (OBC)

Connect the MCU with the OBC via a micro USB cable.  
To attach the MCU permanently, add the following udev rule:
```
$ sudo nano /etc/udev/rules.d/99-state-indicator.rules
```

Then define the rule as follows: 
`KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8037", MODE="0666", GROUP="dialout", SYMLINK+="state_indicator"`

Execute these commands to reload and apply the rule:
```
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```

Now you should find the port `/dev/state_indicator`



## Usage
To start up the state indicator, launch the following file:
```
$ roslaunch stack_master state_indicator.launch
```