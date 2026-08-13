### PX4 TELEM2 MAVLink Configuration

Configure the Pixhawk 6X through QGroundControl:
- `MAV_1_CONFIG = TELEM 2`
- `MAV_1_MODE = Onboard`
- `MAV_1_RATE = 0`
- `SER_TEL2_BAUD = 115200`
- `UXRCE_DDS_CFG = Disabled`

The UART uses `115200 8N1`:
- 115200 bits per second
- 8 data bits
- No parity
- 1 stop bit

2:50 PM
