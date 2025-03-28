# Firmware for Liquids Projects

This Repository contains the Firmware for Liquids Projects, that consists of the main C++ Application, the [can_houbolt](https://github.com/SpaceTeam/can_houbolt) submodule and the **STRHAL submodule**. The main application interfaces to the [LL Server](https://github.com/SpaceTeam/llserver_ecui_houbolt) via FDCAN and the **can_houbolt** interface imported as a submodule in Core/Inc. To avoid using the HAL and gain more insight and control over the hardware, a custom hardware abstraction layer using only CMSIS and LL, **STRHAL**, was written and imported as a submodule in Drivers/STM32xxxx/STRHAL, where each supported STM32 variant contains the **STRHAL** repo with the respective branch.

## Table of Contents  
1. [Run](#run)
2. [Structure](#application-structure)
3. [Can_houbolt Protocol](#can_houbolt-protocol)

## Run 
This repository is currently both a cmake project and an [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) Project. While opening the project with said IDE is still supported,
going forward cmake is preferred. `CMakePresets.json` should contain the necessary data so your IDE of choice can open the project. Both VSCode and CLion have been tested to work with the project.

You can also build it manually from the command line using the presets:
```sh
cmake --preset "debug" -B build/debug
cmake --build build/debug
```

When running the cmake build, `ninja` is used as the build system by default (for better Windows compatibility), so make sure to have it installed.

### VSCode
The workspace contains some recommended extensions, one of which is the official VSCode extension by ST. Others are for CMake and C/C++ support.

You will need to configure:
- The path to the `STM32CubeCLT` directory. This is necessary so the default run configuration (for flashing + debugging) works. You can also change it in the settings.
  The `STM32CubeCLT` package can be downloaded from the ST website.
- The cmake configure preset. You most likely want to choose `debug` for development. You can later change it using the `Select Configure Preset` command in the command palette.

You should be prompted for both things when you first open the workspace.

### CLion
CLion already has C/C++/CMake support out of the box, so you don't need to install any additional plugins.
You will need to configure:
- The CMake preset to use. You should be prompted for this when you first open the project. Otherwise, go to Settings > Build, Execution, Deployment > CMake. Enable the profiles
that are annotated with `preset` that you need. _Disable_ the default CMake profile, it does not work.
- A GDB configuration. The easiest way is to go to Run > New Embedded Configuration and select `STM32CubeIDE ST-LINK GDB Server`. You will need to specify the path to the `STM32CubeIDE` executable.
  The `STM32CubeIDE` package can be downloaded from the ST website. You can use the default settings for the other options. \
  The configuration can also be created manually by going to the Run/Debug Configurations and choosing "Embedded GDB Server". You will then however need to enter most of the settings manually.

## Application Structure

UML DIAGRAM

## can_houbolt Protocol
The FDCAN communication assumes network with a single master ([LL Server](https://github.com/SpaceTeam/llserver_ecui_houbolt)) and multiple nodes (STM32 MCUs flashed with this repo's software), that are defined by an ID. The communication between nodes and to/from the master follows the protocol defined in [can_houbolt](https://github.com/SpaceTeam/can_houbolt). It includes the basic message structure, message types, channel types, channel variables and possible commands. Basically it defines the structure and possible values of the data bytes inside the FDCAN frame.

Generally, a message consists of the channel id in the first byte, the command id in the second byte, and some defined message after. All possible structs are inside the can_houbolt repository and are very straightforward. Thus, this section will focus on the implemented individual channel commands and variables and how they are implemented. IMPORTANT: as the can_houbolt protocol was made before this software was even begun to be written the channel names of the protocol do not match the application channel names. They are related by the `CHANNEL_TYPE` given to each application channel as these match the protocol channel names. In the following section, the application channel names are used. Also note, that, because of the master-to-node structure of the network, and the design choice to always respond to a master command, each command exists as a request and a response, where the response command ID is the request command ID incremented by 1. As, apart from the GENERIC_RES_DATA message, no other message is sent without a request, only request commands will be listed here and the repective response is discussed under that section aswell.

### Generic Channel
This channel only exists once per board and, in the C++ application, is also the class that the board objects extends. It is of CHANNEL_TYPE_NODE_GENERIC and handles all board specific variables and commands.
#### Variables
- GENERIC_REFRESH_DIVIDER
The generic refresh divider determines by how much the timer frequency of the GENERIC_RES_DATA command, that sends the `DataMsg_t` to the master, is divided. This timer has a freqency of 10kHz. By default, this generic divider is 100, thus, data is sent to the master with 100Hz. 

#### Commands
- GENERIC_REQ_NODE_INFO
This command is sent by the LL_Server on startup to get info on all the connected nodes. The node responds with a `NodeInfoMsg_t` containing the firmware version, a channel mask, where each enabled channel is marked by a 1-bit, and array of CHANNEL_TYPE, where each enabled channel writes the respected CHANNEL_TYPE.

- GENERIC_REQ_DATA
This command is rarely used as the node automatically sends the GENERIC_RES_DATA. As mentioned, the node responds with the `DataMsg_t` containing, again, a channel mask, and a byte array, where every enabled channel appends it's data bytes.

- GENERIC_REQ_RESET_ALL_SETTINGS
This commands resets the Config Sector of the Flash and the Flash object's Config Array. This means, that every config register now is 0x0000, in the Flash memory aswell as in the Flash object.

### ADC Channel

### Digital In Channel

### Digital Out Channel

### IMU Channel

### Pressure Control Channel

### Pyro Channel

### Servo Channel
This channel, unsurprisingly, is of CHANNEL_TYPE_SERVO. On startup, it loads the position start- and endpoints from Flash.
#### Variables
- SERVO_TARGET_POSITION
The servo target position determines where the servos position is by setting the PWM Duty cycle accordingly. It expects a 16-bit integer, as the 16-bit range is then mapped using start- and endposition to the appropriate duty cycle. In more detail, a change in this target poition turns on the PWM timer and triggers the `SERVO::MOVIN` state, that waits until the servo position feedback shows, that it is within acceptable range of the target, or until a timeout is reached, to transition to `SERVO::IDLE`, where the PWM timer is turnt off again.

- SERVO_POSITION_STARTPOINT
The servo startpoint determines the startpoint of the PWM refrence. It expects a 16-bit integer, as the 16-bit range is then mapped according to pre-defined PWM range values. Setting it to 30.000 would, for example, half the PWM range. It also triggers the `SERVO::CALIB` state, where the target position is set to 0 and the servo waits until the feedback change is within an acceptable range for a defined number of time steps. Then the startpoint for the ADC range is set to the actual feedback value and the new PWM and ADC startpoints are saved inside external Flash memory.

- SERVO_POSITION_ENDPOINT
Thie servo endpoint is exactly like the startpoint, except it makes the servo position itself to the endpoint that is 65535. 

- SERVO_SENSOR_REFRESH_DIVIDER
The servo refresh divider determines how much the generic `getSensorData()` call is divided. This means, that this servo channel only sends data and sets the corresponding channelmask bit once the number of calls from the generic channel to send sensor data is divided by the servo refresh divider. In other words, the frequency of servo sensor data is determined by the timer frequency (10kHz) divided by the generic refresh divider and further divided by the servo refresh divider.

#### Commands
- SERVO_REQ_RESET_SETTINGS
This command fills the Flash config with the default values for all servo related registers.
