# DroneCAN node management over MAVLink

This note describes how a ground control station (GCS) can discover DroneCAN nodes, read their status, and edit their parameters over a MAVLink telemetry link. It records what PX4 and QGroundControl (QGC) provide today, the changes that each side needs, and the features that this design excludes.

Status: backlog. Written on 2026-08-28 during the review of [mavlink/qgroundcontrol#14983](https://github.com/mavlink/qgroundcontrol/pull/14983) and [PX4/PX4-Autopilot#28435](https://github.com/PX4/PX4-Autopilot/pull/28435), which move autopilot parameter download to MAVLink FTP. Firmware statements are verified against PX4 `main` (`src/modules/mavlink/mavlink_parameters.cpp` and `src/drivers/uavcan/uavcan_main.cpp`). GCS statements are verified against QGC `master`.

## Goal

A GCS connected over an ordinary telemetry link can do the following for every DroneCAN node on the bus:

- See that the node exists and whether it's healthy.
- Read the node's name, hardware version, software version, and unique ID.
- Read and write the node's parameters.

The design doesn't tunnel CAN frames over MAVLink, and it adds no traffic to the link while the DroneCAN page is closed.

## Terminology

The MAVLink messages and the PX4 driver use the name UAVCAN. This note keeps those names for messages, topics, and code, and uses DroneCAN for the user-facing feature.

## Current behavior

### PX4 bridges node parameters to MAVLink parameters

When PX4 is built with the uavcan driver, `CONFIG_MAVLINK_UAVCAN_PARAMETERS` is enabled by default. The MAVLink module forwards a `PARAM_REQUEST_LIST`, `PARAM_REQUEST_READ`, or `PARAM_SET` message to the uavcan driver when the message's `target_component` meets all of the following conditions:

- The value is in the range 1 to 126.
- The value isn't the autopilot's component ID.
- The value isn't the component ID of a MAVLink camera that PX4 has observed.

The driver treats `target_component` as the node ID. It sends each reply as a `PARAM_VALUE` message whose sender component ID is the node ID, with the node's own `param_index` and `param_count`. After a `PARAM_SET`, the driver marks the node's parameters as dirty and issues the save opcode itself, so the write persists without further action from the GCS.

A request with `target_component` set to `MAV_COMP_ID_ALL` does two things: the autopilot streams all of its own parameters, and the driver lists the parameters of every active node. No `target_component` value means "all nodes except the autopilot".

### PX4 publishes node status internally

The uavcan driver publishes a `dronecan_node_status` uORB topic for each node every 100 ms from its `NodeStatusMonitor`. The topic carries the node ID, uptime, health, mode, and vendor-specific status code. These are the same fields as the MAVLink `UAVCAN_NODE_STATUS` message (ID 310).

The driver's `NodeInfoRetriever` stores the name, hardware version, software version, and unique ID of every node that it has seen. Nothing publishes that information.

### QGC shows node parameters only as a side effect

When QGC sends the initial `PARAM_REQUEST_LIST` to `MAV_COMP_ID_ALL`, the node `PARAM_VALUE` messages arrive under the node component IDs, and the parameter editor shows a component tab for each node. QGC doesn't know the node's name or health, and it can't detect a node by any other means.

After #14983, QGC downloads the autopilot's parameters over MAVLink FTP when the parameter hash check misses, so the request that used to list the nodes isn't sent. When the hash check hits, QGC loads the autopilot's parameters from its cache and never sent the request. As a result, node parameters aren't visible on PX4 until something requests them for each node.

### Nothing on the link discovers nodes

The following gaps prevent a GCS from discovering nodes:

- DroneCAN nodes don't send a MAVLink `HEARTBEAT`.
- PX4 doesn't stream `UAVCAN_NODE_STATUS` or `UAVCAN_NODE_INFO`.
- QGC doesn't handle either message.

ArduPilot doesn't send these messages either. ArduPilot tunnels CAN frames with `CAN_FRAME`, `CANFD_FRAME`, and `CAN_FILTER_MODIFY` (`AP_MAVLinkCAN`), and Mission Planner speaks DroneCAN itself. This design doesn't go that far.

On current firmware, the only way to list the nodes without streaming the autopilot's parameters is to send `PARAM_REQUEST_LIST` to `MAV_COMP_ID_ALL` and then immediately send `PARAM_SET` for `_HASH_CHECK` to the autopilot. The second message sets `_send_all_index` to -1, which stops the autopilot's stream after a few leaked values. This workaround returns node IDs only.

## Proposed changes

The GCS drives everything. Node status and node info are requested with `MAV_CMD_REQUEST_MESSAGE` while the DroneCAN page is open, not enabled as streams, and node parameters are listed with `PARAM_REQUEST_LIST` addressed to each node. The autopilot's own parameters stay on the MAVLink FTP path from #28435 and are never streamed again.

### PX4

1. Add a `UAVCAN_NODE_STATUS` stream class in `src/modules/mavlink/streams/` that isn't in any default profile. Implement `request_message()` so that `MAV_CMD_REQUEST_MESSAGE` with `param1` = 310 and `param2` = node ID sends that node's status, and `param2` = 0 sends one message per online node. Read the data from the `dronecan_node_status` instances and set the sender component ID to the node ID. Neither `UAVCAN_NODE_STATUS` nor `UAVCAN_NODE_INFO` has a node ID field, so the sender component ID identifies the node, which matches the parameter bridge. `param2` is the index ID by specification. When a requested stream isn't configured, `handle_request_message_command` configures it at rate 0 and triggers it once, so no profile changes are needed.
2. Add a `dronecan_node_info` uORB topic. Fill it from a `NodeInfoRetriever::Listener` implementation in the uavcan driver (`handleNodeInfoRetrieved`).
3. Add a `UAVCAN_NODE_INFO` stream class with the same `request_message()` contract as the status stream. Don't use `MAV_CMD_UAVCAN_GET_NODE_INFO` (5200): nothing implements it, and MAVLink has replaced per-message request commands with `MAV_CMD_REQUEST_MESSAGE`.
4. Apply the camera exclusion. Node IDs 1 to 126 overlap MAVLink component IDs; for example, `MAV_COMP_ID_CAMERA` is 100. The parameter bridge skips observed cameras, and the streams must skip them too. The GCS must not treat a component that sends a `HEARTBEAT` as a node.

### QGC

1. Add a `DroneCANNodeManager` class to `Vehicle` that runs only while the DroneCAN page is open. On open, it requests `UAVCAN_NODE_INFO` for all nodes once and then requests `UAVCAN_NODE_STATUS` for all nodes every 1 to 2 seconds. When a status arrives from a component ID that has no info yet, it requests `UAVCAN_NODE_INFO` for that node. On close, it stops requesting. The manager keeps a model keyed by component ID with the node's name, hardware version, software version, unique ID, health, mode, uptime, last-seen time, and parameter load state, and it removes a node that misses several status polls. Ten nodes polled at 1 Hz cost about 200 bytes per second, which a SiK link absorbs.
2. Load parameters for each node. When a node is discovered, call `ParameterManager::refreshAllParameters(nodeComponentId)`. `_startParameterDownload` takes the FTP branch only for `MAV_COMP_ID_ALL` and the autopilot, so any other component ID goes down the `PARAM_REQUEST_LIST` branch with that node as `target_component`, and PX4's bridge lists that node only. Component tracking, index retries over `PARAM_REQUEST_READ`, and `PARAM_SET` already work for each component ID because the bridge handles all three messages.
3. Add a DroneCAN page to vehicle setup. The page lists nodes with health and mode, shows the `GetNodeInfo` fields, and embeds the existing `ParameterEditor` filtered to the node's component ID. Move the "UAVCAN nodes" wording from the reset-all-parameters warning to this page.

## Out of scope

The following features aren't part of this design:

- CAN frame tunnelling with `CAN_FRAME` and `CANFD_FRAME`.
- Firmware update over the link. The PX4 driver updates nodes from the SD card.
- Enumeration and dynamic node ID allocation.
- Parity with the DroneCAN GUI tool.

## Open questions

- **Should node parameters be served as a packed FTP file, such as `@PARAM/param.pck?node=N`?** No. The driver fetches one parameter per bus round trip, and an FTP read must return synchronously. The `PARAM_VALUE` stream is the right transport for the tens of parameters that a node has.
- **How does the GCS reboot a node or force a save?** The driver has `reset_node` and the save opcode, but no MAVLink entry point calls them. Saving is automatic after a set. A reboot after a node ID change needs a command. Decide after the page exists.
- **Does PX4 need to receive `UAVCAN_NODE_STATUS`?** No. PX4 only sends it.
- **Should the page keep node parameters after it closes?** Yes. They live in `ParameterManager` like any other component's parameters. Only the status and info polling stops.
