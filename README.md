# Drone simulator based on Strathclyde 6DOF

# Installation guide - Requirements

## Eigen installation
The Eigen library is required. 
As the [documentation](https://eigen.tuxfamily.org/dox/GettingStarted.html#title0) mentions, it is enough to place the Eigen headers in a standard header path. 
### Mac OS
Install Eigen trough **brew**:
```
brew install eigen
```
### Other Unix environments
Make sure Eigen headers are present in one of your gcc header search paths.
Run `gcc -v -E -` to list the standard search paths.

### Windows
TODO: Write Windows documentation

# Installation guide - Install

Run `sh setup.sh` from the root folder of the project.

# Usage

## Start the simulator 
Issue `./6dof` to start the simulation.
Once started, the simulator will listen for the PX4 autopilot handshake messages.

## Start PX4
1. The PX4 autopilot must be downloaded from the [PX4 Repo](https://github.com/PX4/PX4-Autopilot).
2. Start the autopilot via the command `make px4_sitl none_standard_vtol`. This will build the autopilot and start it with the default standard_vtol parameters.

## Takeoff
Once PX4 displays a ready state, one can interact with the commander via the `commander` controls from the PX4 console.

To **takeoff** issue `commander takeoff`. This will issue actuation controls to the simulator and the drone will start the VTOL takeoff sequence.

## Flight Reporting
The flight logs produced by PX4 can be found in the cloned PX4 repo, under the `build/px4_standard_vtol` folder.

To analyse the flight logs (.ulg), one can upload them to the [PX4 Flight Review](https://logs.px4.io/) web application.

# Debugging

## Inspect MAVLink messages
To display the MAVLink messages passed to the autopilot, uncomment the relevant defines in `Interfaces/DroneStateEncoder.h`.

## Inspect received actuator controls
To display the received actuator controls from the autopilot, uncomment the relevant define in `Drone.cc`.