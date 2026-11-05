# CANopenNode drive integration

This development branch keeps three separate layers:

1. `canopen`: CANopenNode, SDO transfers, NMT and heartbeat monitoring.
2. `cia402`: drive states, profile-position motion, standard homing, zeroing and digital inputs.
3. `tmcm1240`: Trinamic current conversion, six-point ramps, StallGuard registers, device diagnostics and bounded stall seeking.

Application YAML owns mechanical units, calibration, travel distances, open/close sequences, MQTT entities and the decision to establish a home reference. No application wiring or calibration is included here.

`TMCM1240` derives from `CiA402Drive`. Both use the same CANopenNode SDO client; there is no second CANopen implementation or per-motor worker task.

`seek_stall(distance, margin)` uses drive counts and confirms motion followed by two early-stop samples, correct target and StallGuard readbacks, zero load, and no driver fault. Reaching the requested endpoint is failure for this operation. It never zeroes position. `move_until_stop` also accepts normal target arrival. Application code must wait for completion and check the result before issuing the next command.

`zero_position()` uses standard homing method 35 and verifies position zero. A failed request, timeout, stop or lost heartbeat cancels the active sequence. `get_reference_epoch()` changes on node loss/reset so application code can invalidate its reference even if the node recovers quickly.

Configuration is volatile. Saving is a separate explicit `save_configuration()` operation which requires the drive to be disabled. There is no automatic movement or homing in the driver.

Register meanings and StallGuard emergency 0x3230 follow the [TMCM-1240 firmware manual](https://www.analog.com/media/en/dsp-documentation/software-manuals/TMCM-1240-CANopen_firmware_manual_fw3.25_rev1.04.pdf). That emergency is tolerated only during an explicit stall-seeking operation and never substitutes for the readback checks.

These extensions are on the integration branch only; they do not enlarge the submitted upstream PRs. Previous hardware testing of the legacy application does not establish hardware validation of this port.
