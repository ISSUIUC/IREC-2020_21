# pressureTestDAQ

Data acquisition (DAQ) firmware written for the hybrid engine's high pressure N2 test.

### Notable Functionality:
  - Implements a FIFO buffer to push data bandwidth as high as possible (can be higher with further optimization tbh).
  - Uses an interrupt to handle incoming bytes from UART instead of polling the Serial buffer.
  - Can trigger the solenoid and the ball valve servos through telemetry.
  - Simple python GUI for real-time plotting

### LED Indicators
Uses LEDs onboard TARS-MK1 to communicate system state. Thea meaning of which are as follows:

  - **WHITE** == System and SD card file initialization successful
  - **ORANGE** == Ball Valve is **OPEN**
  - **RED** == Vent valve is **OPEN**
  - **BLUE** == Data is being recorded to SD card.
  - **ONBOARD LED** (blinking) == Something went wrong :'(
