# Known Working Configuration

Last verified: 2026-08-18

This file records the configuration that has already worked in this project. Planned Offboard and physical-replay settings are not included as working settings yet.

## Hardware

- Drone frame: Holybro X500 v2
- Flight controller: Pixhawk 6X running PX4
- Companion computer: Raspberry Pi Zero 2 W
- Pi operating system: Raspberry Pi OS Lite, 64-bit
- Pi hostname used by the log-copy script: `fd-hyu.local`
- Previously observed Pi address: `192.168.0.79`

## Pixhawk TELEM2

PX4 parameters used for the Pi connection:

```text
MAV_1_CONFIG = TELEM 2
MAV_1_MODE = Onboard
MAV_1_RATE = 0
SER_TEL2_BAUD = 115200
UXRCE_DDS_CFG = Disabled
```

UART settings:

```text
115200 baud
8 data bits
no parity
1 stop bit
no RTS/CTS hardware flow control
```

The Pi opens the port as `/dev/serial0`. On the tested Pi, `/dev/serial0` points to `/dev/ttyS0`.

Start the recorder on the Pi:

```sh
cd ~/repo/flight-debrief/mavlink
./flight-debrief serial /dev/serial0
```

The recorder waits for at least one byte, has no serial read timeout, and stops cleanly with `Ctrl+C`.

## Log output

The recorder creates a directory under:

```text
mavlink/out/log_YYYYMMDD_HH_MM_SS/
```

The raw MAVLink log is:

```text
telemetry.tlog
```

Each stored record contains an 8-byte receive timestamp followed by one complete MAVLink frame.

Copy Pi logs to the Mac from the repository root:

```sh
./pull_pi_logs.sh
```

Default source and destination:

```text
hyu@fd-hyu.local:~/repo/flight-debrief/mavlink/out/
./downloaded-logs/
```

`PI_HOST` and `PI_LOG_DIR` can override the default Pi host and directory.

## PX4 SITL and Gazebo on Apple Silicon

The working setup uses a local PX4 branch named `macos-sitl`, based on the current PX4 `origin/main`. PX4 `v1.17.0` was not used because it had compatibility problems with the installed macOS packages.

Installed Homebrew packages:

```sh
brew install osrf/simulation/gz-harmonic
brew install qt@5 opencv@4
```

Run from the `PX4-Autopilot` repository:

```sh
source .venv/bin/activate
export CMAKE_PREFIX_PATH="$(brew --prefix qt@5):$(brew --prefix opencv@4)"
ulimit -n 10240
make px4_sitl gz_x500
```

If Gazebo requires X11:

```sh
open -a XQuartz
```

Missing GStreamer and Java warnings can be ignored for the `gz_x500` target. GStreamer is used by the video plugin, and Java is used by jMAVSim.

## SITL UDP recording

The working MAVLink UDP connection was:

```text
PX4 local/source port: 18570
recorder receive port: 14550
```

Start the recorder from the `mavlink` directory:

```sh
./flight-debrief udp 14550
```

The recorder only receives on this connection. It does not currently send commands to PX4.

## Known working test log

Test file:

```text
mavlink/out/log_20260818_18_03_57/telemetry.tlog
```

This log contains a takeoff and landing. The highest recorded point is approximately:

```text
time: 14.08 seconds
LOCAL_POSITION_NED z: -2.3735 meters
```

Because this is NED coordinates, a negative Z value is above the starting point.

Validator result:

```text
Records read:    8443
Valid frames:    8443
Invalid CRC:        0
Unknown CRC:        0
Decode failures:    0
```

The log contains `1121` decoded `LOCAL_POSITION_NED` samples and lasts approximately `22.4` seconds.

## Not verified yet

The following are planned but must not be treated as working configuration yet:

- Gazebo `/world/<world>/set_pose` replay
- PX4 Offboard mode entry from `flight-replay`
- Sending position setpoints to SITL
- Sending position setpoints to the real Pixhawk
- Offboard-loss, abort, and pilot-takeover settings for physical replay
