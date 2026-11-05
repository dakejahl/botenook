# Status bits in the DroneCAN flow quality byte

Idea, 2026-11-05. Not built.

`com.hex.equipment.flow.Measurement` carries one `uint8 quality`, and the ARK Flow nodes put raw SQUAL there. The flight controller cannot see the chip's illumination mode or the PAA3905 challenging-surface flag. The low bits of the quality byte can carry both. That needs no new message and leaves the data type signature unchanged. It replaces the companion message in dakejahl/DSDL#2.

The mode bits also answer the problem behind PX4/PX4-Autopilot#28625, that the message drops the mode. They do it without rescaling quality.

## Encoding

| Bits | Content |
|---|---|
| 7:3 | SQUAL with the low 3 bits cleared |
| 2:1 | Mode, numbered as `sensor_optical_flow`: 0 unknown, 1 bright, 2 low light, 3 super low light |
| 0 | Challenging surface. Always 0 on the PAW3902 |

Quality moves in steps of 8. Four bits would give steps of 16 and collapse the 60 to 85 band, where tracking falls off, to 64 and 80.

## Rules

1. **Zero stays zero.** ArduPilot fuses any quality above 0 ([AP_NavEKF3_Measurements.cpp:205](https://github.com/ArduPilot/ardupilot/blob/802db19de1/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp#L205)). `EKF2_OF_QMIN` defaults to 1, and [VehicleOpticalFlow.cpp:118](https://github.com/PX4/PX4-Autopilot/blob/53434189b5/src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp#L118) reads 0 as a gap. Status bits are set only when the masked SQUAL is nonzero. Frames below SQUAL 8 therefore go out as 0.
2. **Pack after the node averages and strip before the FC averages.** Both sides average quality across frames ([VehicleOpticalFlow.cpp:209](https://github.com/PX4/PX4-Autopilot/blob/53434189b5/src/modules/sensors/vehicle_optical_flow/VehicleOpticalFlow.cpp#L209)). The node packs in the DroneCAN flow publisher after its 50 Hz aggregation. The FC strips in [flow.cpp:72](https://github.com/PX4/PX4-Autopilot/blob/53434189b5/src/drivers/uavcan/sensors/flow.cpp#L72). A message covers about 2.5 chip frames, so it carries the latest mode and sets the challenging-surface bit if any frame flagged it.
3. **Decode only for senders known to pack.** Here Flow and every shipped ARK Flow build send a plain quality byte, so a blind decoder would flag a challenging surface on about half their frames. Old ARK builds report the same node name, `org.ark.can-flow`, so the gate is name plus software version. The FC already gates Vertiq ESC status bits on vendor through `NodeInfoPublisher` ([esc.cpp:226](https://github.com/PX4/PX4-Autopilot/blob/53434189b5/src/drivers/uavcan/actuators/esc.cpp#L226)). Every other sender passes through unchanged.

Shipped flight controllers read a packing node up to 7 counts high, under 3 % of the weighting ramp. They reject the frames below SQUAL 8 that they fuse today. All of those are below the bright-mode floor of 25.

## Diagnostics

Per-sample delivery isn't needed for the rest of DSDL#2. The node shouldn't publish an unhealthy chip frame at all. The motion bit means nothing on an aggregated sample. Chip health, discard rate and mode-change count go in `NodeStatus.vendor_specific_status_code`, which has 16 bits at about 1 Hz and shows in the DroneCAN GUI tool. That also shows on the bus when the node enters low light.

## Plumbing

- Node: `vehicle_optical_flow` gets mode and challenging-surface fields.
- FC: `flow.cpp` decodes into `sensor_optical_flow.mode` and a new challenging-surface field.

## Open

- Agreement from Hex, which owns the namespace, and from Tridge, recorded as a comment on DTID 20200.
- andyp1per's ArduPilot quality PR masking and decoding from the start. ArduPilot uses quality only as nonzero today.
- The node software version that starts packing.
- The `vendor_specific_status_code` bit layout.
- Whether the challenging-surface flag earns a fusion role. That is step 3.1 of the [RFC](improvement_rfc.md) and waits on the concrete data from step 0.5.
