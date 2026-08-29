# Flight Debrief
A system for recording MAVLink flight telemetry, extracting a trajectory, and replaying it as PX4 Offboard position setpoints. A Raspberry Pi Zero 2W can record from and control a Pixhawk over UART and "replay" the trajectory.

> [!CAUTION]
> This is experimental software developed for educational purpose. It can send commands to a real drone and may cause injury, property damage, or loss of the vehicle. 
> 
> Test all change in simulation first. Verify all the behaviors with reliable methods. Operate ONLY in a SAFE, LEGAL, and sufficiently open area. 
>
> The physical replay system has not yet been validated in propeller-on flight for few reasons listed above. Use this software entirely at your own risk...
> 
> If you found any problems, please let me know by creating issues.

## Overview
The project provides four small C programs:


| Program | Purpose |
| --- | --- |
| `flight-debrief` | Record complete MAVLink frames from UART or UDP |
| `tlog-validator` | Check frame integrity and report decoded message counts |
| `flight-replay` | Extract and print the recorded local-position trajectory |
| `sitl-position-hold` | Replay the trajectory through PX4 Offboard control |


Recordings use a project-specific `.tlog` format. Each record contains an 8-byte receive timestamp followed by one complete MAVLink frame. Note that this is not the standard PX4 Telemetry log.

## Hardware 
- Holybro X500 v2
- Pixhawk 6X with PX4
- Raspberry Pi Zero 2W

See [Known Working Configuration][doc/CONFIGURATION.md] for the verified PX4, UART, UDP, and macOS SITL settings.

## Repository Layout

```text
main/
  include/          Public headers
  src/common/       MAVLink parser, decoder, and tlog reader
  src/recorder/     UART and UDP telemetry recorder
  src/validator/    Recording validation and summary
  src/replay/       Trajectory extraction, resampling, and validation
  src/control/      PX4 transport, commands, and replay state machine
doc/                Configuration and development notes
tools/              Log transfer and cleanup scripts
```

The MAVLink parser and decoder are implemented manually as a learning exercise. The controller uses official MAVLink C headers for encoding messages.

## Build
Build the project using Makefile:

```
cd main
make
```

Note that the `control` target requires generated MAVLink C headers. Modify the makefile or use:
```sh
make MAVLINK_GENERATED_DIR=/path/to/generated/mavlink control
```

## Usage
Run the following cmmands from `main/`

Record from PX4 SITL over UDP:
```
./flight-debrief udp 14550
```

Record from Pixhawk connected to the Raspberry Pi: 
```
./flight-debrief serial /dev/serial0
```

Validate and inspect a recording:
```sh
./tlog-validator out/<session>/telemetry.tlog
./flight-replay out/<session>/telemetry.tlog
```


Replay in SITL, optionally printing the difference between commanded and reported local positions:
```sh
./sitl-position-hold out/<session>/telemetry.tlog
./sitl-position-hold out/<session>/telemetry.tlog --diff
```

The hardware transport uses the same controller over UART:
```sh
./sitl-position-hold serial /dev/serial0 out/<session>/telemetry.tlog
```

> [!WARNING]
> The hardware command can arm and move a real drone. DO NOT  run it with propellers installed until the complete configuration and safety behavior have been independently verified.
> PLEASE BE SAFE!

## About Replay and Safety

## Verification

## Reporting Issues
If you find a problem, please, report it by opening a GitHub issue including the relevant platform, PX4 version, connection type (UDP or serial), and log output.

## License
This project is available under the [MIT License](LICENSE.md).
