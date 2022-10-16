# National Taiwan University Racing Team Software Development Log
###### tags: `development_log` `NTURT`
##### Group: electrical system group
##### Person in charge: 羅紀翔
##### Authors: 羅紀翔 劉宇彤 黃柏瑞
##### Subsystem: RPI
##### Subsystem number: RP5
##### Software name: can_parser
##### Repository: [github](https://github.com/NTURacingTeam/nturt_can_parser)
##### Started designing date: 2022/1/18
##### Current version: 1.0
##### Last modified date: 2022/10/6

---

## Engineering goal:

Parse can signal to allow bidirectional can sinal transfer between ros and other software/hardware on the can line.

## Program structure:

TODO

## Included libraries:

- boost-array
- yaml-cpp

## Testing environment:

- ros noetic
- docker virtual environment from [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker) with image `ros_matlab`, `ros_rpi` based on ubuntu20.04

##### Testing hardware:

- asus tuf gaming a15 FA506II-0031A 4800H
- raspberry pi 3B+

##### Operating system:

- ubuntu 20.04
- raspbian 32-bit

##### Compiler(intepreter) version:

- gcc 9.4.0 (Ubuntu 9.4.0-1ubuntu1~20.04.1)

---

## Testing result of 1.0:

### Receiving can signal from front box

Register to can parser from nodes `nturt_torque_controller`, `nturt_state_controller`, registered and data received successfully.

## Todos in 1.0:

- check for transmitting can signal
