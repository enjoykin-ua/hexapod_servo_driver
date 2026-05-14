# contrib/

Off-firmware data that lives in this repo for convenience but is **not** used
by the firmware itself. The firmware sees 18 PWM outputs and pulse-µs values
— nothing in this directory is compiled into the .uf2.

## Files

### `servo_mapping.yaml`

Authoritative mapping from Servo2040 output index (0..17) to hexapod joint name
plus per-servo calibration (pulse_min, pulse_max, pulse_zero, direction).

- **Phase 7:** placeholder values (default ranges 500–2500 µs, direction +1 for all)
- **Phase 9:** copied into `hexapod_ws/src/hexapod_hardware/config/` and consumed
  by the `ros2_control` HardwareInterface for joint-rad → pulse-µs conversion
- **Phase 10:** real calibration values overwrite the placeholders here AND in
  hexapod_hardware (kept in sync manually for now; a sync helper script may
  follow if the duplication becomes painful)

See the file header for the full pin convention + `direction` semantics.
