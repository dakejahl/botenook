# MAVLink stream catalog and profiles

Status: **proposal**. Written 2026-08-30 as a companion to [`module_architecture.md`](module_architecture.md), revised the same day: the per-instance file is the source of truth and `MAV_n_MODE` goes away. Firmware statements are against PX4 `main` at `436cc71c89`.

Audience: an engineer or agent designing stream configuration for the new MAVLink module, the MAVSDK tests, and any later QGC page that edits rates.

## Takeaways

One JSON file per instance is the configured stream set. It is not an overlay on `MAV_n_MODE`. `MAV_n_MODE` goes away.

`SET_MESSAGE_INTERVAL` / `GET_MESSAGE_INTERVAL` and `mavlink stream` stay as the runtime API. They do not write the file. The file changes only when something writes it over MAVLink FTP (GCS Save). A completed write is applied immediately; no reboot.

Factory defaults are complete snapshot files baked into ROMFS (today's Normal / Onboard / Iridium tables, generated from the same declarations as the catalog). Reset copies that snapshot over the instance file and applies it. First boot with no SD file uses the ROMFS snapshot; if an SD is present, copy it out so there is always a file to edit.

Two objects:

| Object | Role | On the vehicle | Who writes it |
|---|---|---|---|
| **Catalog** | What this firmware *can* stream, plus named factory snapshots | ROMFS, `streams.json.xz`, CRC'd component metadata | The build |
| **Profile** | What *this instance* is configured to send | `/fs/microsd/etc/mavlink/{i}.json` (SITL: posix path) | GCS Save via FTP, a test, or a human with a text editor. Reset restores the factory snapshot |

The catalog is discovery. The profile is the instance table. Ship both.

Do not xz the profile. Component-metadata `.xz` is a **host** step (Python `lzma` at build); the FMU serves that blob and the GCS decompresses it. The FMU does not run liblzma. A profile is a few kilobytes of JSON, must be parsed on `wq:mavlink`, must be editable on the SD card, and is rewritten on Save. Compressing it adds a decompressor to the hot config path for no flash win. The catalog stays `.xz` like `parameters.json.xz`.

## What exists today

`MAV_n_MODE` is a C++ switch in `configure_streams_to_default()` and a per-instance param (`module.yaml` defaults `[Normal, Onboard, Normal]`). It also selects non-stream personality: Iridium (HIGH_LATENCY2, TX gate, no param send), Gimbal (restricted RX). HEARTBEAT 1 Hz and STATUSTEXT 20 Hz are forced on for every non-Iridium mode.

`SET_MESSAGE_INTERVAL` maps a msg id to a stream name. Interval `< 0` stops, `0` restores the mode default, `> 0` is microseconds. HEARTBEAT is rejected. That restore-default has to mean "re-read this stream from the profile file" once MODE is gone.

QGC `MAVLinkStreamConfig` is three hardcoded high-rate sets plus restore-defaults, all via `SET_MESSAGE_INTERVAL`. It has no catalog and does not persist.

## Catalog

Add `COMP_METADATA_TYPE_STREAMS = 6` in mavlink (development.xml first). Serve `mftp://etc/extras/streams.json.xz`. Generate it from the same declarations the firmware uses to register streams.

The catalog does not describe live rates. It describes capability and factory snapshots.

### Schema (draft)

```json
{
  "version": 1,
  "streams": [
    {
      "name": "ATTITUDE",
      "msgId": 30,
      "class": "bestEffort",
      "minHz": 5,
      "maxHz": 50,
      "sizeBytes": 28,
      "setIntervalAllowed": true
    },
    {
      "name": "HEARTBEAT",
      "msgId": 0,
      "class": "latencySensitive",
      "minHz": 1,
      "maxHz": 1,
      "sizeBytes": 9,
      "setIntervalAllowed": false
    }
  ],
  "factoryProfiles": {
    "normal": {
      "personality": "normal",
      "streams": { "HEARTBEAT": 1, "STATUSTEXT": 20, "ATTITUDE": 15 }
    },
    "onboard": {
      "personality": "onboard",
      "streams": { "HEARTBEAT": 1, "ATTITUDE": 30, "HIGHRES_IMU": 50 }
    },
    "iridium": {
      "personality": "iridium",
      "streams": { "HIGH_LATENCY2": 0.015 }
    }
  },
  "instanceFactory": ["normal", "onboard", "normal"]
}
```

`instanceFactory` replaces `module.yaml`'s `MAV_n_MODE` default `[0, 2, 0]`. Board config still decides that telem1 is Normal-shaped and telem2 is Onboard-shaped; it does it by naming a snapshot, not a param the user is expected to keep.

Rules:

- `minHz` is the token-bucket floor. `0` means the stream may be off.
- `maxHz` clamps `SET_MESSAGE_INTERVAL` and profile rates.
- `class` is `bestEffort` or `latencySensitive`. Periodic streams are not `reliable`.
- `setIntervalAllowed` defaults true.
- Absent from `streams` = not compiled in. `SET_MESSAGE_INTERVAL` for that msg id NAKs.
- Each `factoryProfiles` entry is a **complete** profile (same schema as the instance file). Names not listed are off.

## Profile

One file per instance. Complete. Omitted stream = off.

```json
{
  "version": 1,
  "personality": "normal",
  "streams": {
    "HEARTBEAT": 1,
    "STATUSTEXT": 20,
    "ATTITUDE": 10,
    "OPTICAL_FLOW_RAD": 15
  }
}
```

- `personality` is the non-stream leftover of MODE: `normal` (default), `iridium`, `gimbal`, and any other link behavior that is not a rate. Stream rates do not imply Iridium TX-gating.
- Rate `0` disables. `-1` unlimited. Unknown or compiled-out names: log and ignore, keep the rest. Invalid JSON: keep the live table, log, do not stop the instance.
- HEARTBEAT: honor `setIntervalAllowed`. If false and the key is present, still allow it in the **file** (factory snapshots include it) but NAK `SET_MESSAGE_INTERVAL` at runtime. Revisit if tests show that split is more confusing than dropping the NAK.

Path: `/fs/microsd/etc/mavlink/{i}.json`. SITL: the posix equivalent under the rootfs. ROMFS factory: `/etc/extras/mavlink/{i}.json` generated from `instanceFactory`.

Load:

1. If the SD file exists and parses, use it.
2. Else use the ROMFS factory file for `{i}`. If an SD is mounted, copy factory → SD so a user always has a file to tweak.
3. No SD (some boards): run from ROMFS only. Save via FTP can still write SD if it appears later, or refuse Save with a logged error.

Parse on `wq:mavlink`. Post the resulting table to `mavlink_{i}_tx`. Never parse on the TX thread.

### Reset

Reset = copy the ROMFS factory file for `{i}` over the SD file (or delete the SD file and copy), then apply immediately. Surfaces: GCS button (FTP put of the factory snapshot, or a small `MAV_CMD` that does the copy on the vehicle), nsh `mavlink profile reset -i {i}`. Factory bytes also come from the catalog's `factoryProfiles` so a GCS that does not want a vehicle-side command can FTP-put that JSON itself.

### Save

GCS Save = FTP put of the complete JSON to the instance path. Firmware does not dump the live table unless a later command exists; v1 is FTP-only. Runtime `SET_MESSAGE_INTERVAL` is not Save. A GCS that wants Save to include this session's interval tweaks must put those rates in the JSON it writes.

### Apply on write

FTP already lives in the module. On a **completed** write (session close / last `kCmdWriteFile` that finishes the file), if the path is an instance profile:

1. Read and parse on `wq:mavlink`.
2. If parse fails, keep the live table and log. Do not apply a truncated file mid-burst.
3. If parse succeeds, replace the instance table and apply (enable, retune, disable). No reboot.

A human who edits the SD file and wants apply without FTP can reboot, or nsh `mavlink profile reload -i {i}`. Do not poll `stat()` in the TX loop.

## Runtime versus file

```
boot / reset / FTP Save / reload:
    file → instance table → TX
SET_MESSAGE_INTERVAL / mavlink stream:
    live table only
GET_MESSAGE_INTERVAL:
    live table
```

`SET_MESSAGE_INTERVAL` with interval `0` (today: restore mode default) re-reads that stream from the **file**, not from a MODE table. If the file does not list it, the stream goes off.

## Token bucket

The file is configured Hz. The TX path still applies `MAV_n_RATE`, floors from the catalog, and opportunistic throttle. A file that asks for 200 Hz ATTITUDE on a 1200 B/s link does not bypass the cap.

## QGC

v1: `SET_MESSAGE_INTERVAL` still works (runtime). Save/Reset can wait for a page: fetch catalog, edit, FTP-put, vehicle applies. `MAVLinkStreamConfig`'s hardcoded sets become "apply this factory snapshot now" (runtime) versus Save (file).

Do not block the firmware rewrite on that page. MAVSDK tests FTP-put.

## MAVSDK / SITL tests

Agent-facing gates. `mavlink_direct` plus `ftp` until typed APIs exist. Use the SITL instance that the test runner already talks to.

1. **Factory boot.** No SD profile. HEARTBEAT ≈ 1 Hz, ATTITUDE ≈ factory snapshot for instance 0 (today's Normal: 15 Hz).
2. **Runtime only.** `SET_MESSAGE_INTERVAL` ATTITUDE to 5 Hz. Count drops. FTP-get the profile file (or confirm none / factory copy). File contents unchanged. Reboot. Count is factory again.
3. **Save applies now.** FTP-put a complete profile with ATTITUDE 5 Hz. Count becomes 5 Hz **without reboot**. Reboot. Still 5 Hz.
4. **Omitted is off.** Put a profile that lists HEARTBEAT and not HIGHRES_IMU. HIGHRES_IMU count is 0. `GET_MESSAGE_INTERVAL` agrees.
5. **Corrupt put is refused.** FTP-put truncated JSON. Live rates unchanged. Log/error observable.
6. **Reset.** After (3), reset (FTP-put factory snapshot or `MAV_CMD`). Count returns to factory. File on disk matches factory.
7. **Catalog.** FTP-get `streams.json.xz`, decompress on the test host. ATTITUDE present with `minHz`. A KConfig-stripped stream absent. `factoryProfiles.normal` lists ATTITUDE 15.
8. **Compiled-out NAK.** `SET_MESSAGE_INTERVAL` for an absent msg id → COMMAND_ACK denied.
9. **Token bucket.** Sum `sizeBytes * hz` vs `MAV_n_RATE`; best-effort does not exceed the cap, and drops toward floors during a reliable transfer.

`ComponentMetadata` in MAVSDK needs `MetadataType::Streams` (MAVSDK-Proto, then `~/code/jake/MAVSDK`). Until then, FTP-get the catalog by path.

## mavlink.xml

`COMP_METADATA_TYPE_STREAMS = 6` in development.xml with the schema next to `component_metadata/*.schema.json`. Firmware can serve the file before QGC knows the enum. Tests FTP-get by path.

## Out of scope

- QGC editor in the first firmware PR.
- Auto-persisting `SET_MESSAGE_INTERVAL`.
- Per-stream parameters (`MAV_ATTITUDE_HZ`).
- RX handler catalog.
- xz on the instance profile.

## Open questions

1. Forced streams (HEARTBEAT, STATUSTEXT) — catalog flags versus the TX path ignoring a file that omits them.
2. Whether personality stays a string in the file or a separate small param. File keeps one source of truth; a param is easier to miss when Reset runs.
3. `MAV_CMD` for Reset/Reload in v1 versus FTP-put only. Tests can FTP-put factory JSON either way. A command is nicer on boards where the GCS should not round-trip the snapshot.
4. JSON parser on constrained FMUs. If it hurts, a line format is a fallback, not a second official schema.
