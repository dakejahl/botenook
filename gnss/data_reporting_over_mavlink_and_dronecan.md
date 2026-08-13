# GNSS: data reporting over MAVLink and DroneCAN

Status: **idea-stage**, written 2026-08-13. The problem picture (§1) is receipted; the message family (§3–§6) is a proposal, not a design.

Goal in one line: a GNSS message family that can express **everything a modern multi-band, multi-constellation receiver knows**, so a GCS can show full status, an operator gets something actionable, and a receiver can be debugged remotely over the link instead of with the vendor's tool on a bench.

Related work:

- [mavlink/rfcs#30](https://github.com/mavlink/rfcs/pull/30) — split `GNSS_INTEGRITY` into receiver-level integrity plus a per-band `GNSS_BANDS`. Vendor-agnostic, Septentrio + u-blox + NovAtel field mapping. The concrete prior art for §4 and §5.
- [mavlink/mavlink#2146](https://github.com/mavlink/mavlink/pull/2146) — `GNSS`, a `GPS_RAW_INT` / `GPS2_RAW` replacement with an instance id. Marked *talking point only*.
- `GNSS_INTEGRITY` (id 441) already sits in `development.xml`.
- [`selection_fusion_and_heading.md`](selection_fusion_and_heading.md) — the consumer side. Better per-receiver reporting is what a health-gated selector needs to select *on*.

---

## 1. What is wrong today

The GNSS telemetry surface is a pile of accreted GPS-era messages. Concretely:

**`GPS_RAW_INT` / `GPS2_RAW` cap out at two receivers.** There is no instance field, just two hardcoded message ids. Three receivers is not expressible. This is what #2146 fixes.

**`GPS_RAW_INT.satellites_visible` is not satellites visible.** PX4 fills it from `sensor_gps.satellites_used` (`GPS_RAW_INT.hpp`). The field name has been wrong for the whole life of the message, and every GCS that renders "sats" is rendering the used count.

**`GPS_STATUS` cannot describe a modern receiver at all.** It is five parallel 20-element arrays, and it is the only per-satellite message there is:

- **20 satellites, hard.** A receiver tracking 35 does not fit. There is no offset or chunk field, so the remainder is not expressible — PX4 sends `min(count, 20)` and drops the rest (`GPS_STATUS.hpp`).
- **The dropped satellites are not a random sample.** u-blox emits `NAV-SAT` grouped by `gnssId` ascending, and PX4's ubx driver stores it in arrival order, so the truncation lands on the tail: GLONASS first, then BeiDou. A healthy unit on a clear sky reports `GLONASS 0/0`. Measured on the ARK SAM/DAN bench.
- **`satellite_prn` is a `uint8_t` squash, not a unique id.** PX4's ubx driver folds `gnssId:svId` into the legacy NAV-SVINFO numbering. 65 is both BeiDou B37 and GLONASS R1. 255 is the unmapped sentinel for all of NavIC, GLONASS with an unknown slot, and BeiDou past B37 — many satellites, one id.
- **No band or signal field.** A dual-band unit cannot be told apart from an L1-only one. An ARK DAN board with a dead L5 path still reports a full healthy sky.
- **`satellites_visible` in the same message reports the untruncated count**, so the message quietly contradicts its own arrays.

**Nothing carries RF or integrity detail.** Jamming and spoofing are receiver-level booleans at best. Which band, at what frequency, mitigated or not, what the AGC and noise floor are doing — none of it reaches the link, so diagnosing a GNSS problem in the field means recovering the vehicle and attaching u-center or RxTools.

**DroneCAN receivers are second-class.** `uavcan.equipment.gnss.Auxiliary` is thinner still. Whatever a serial receiver can report, a CAN node should be able to report identically.

There is one piece of good news: **`GPS_STATUS` is nearly unused.** QGC does not reference it anywhere in its source, and ArduPilot never sends it. Superseding it costs almost nothing.

---

## 2. Shape of the proposal

Four messages, one base plus three optional. All keyed by the same `id` instance field so a GCS can assemble a per-receiver view, and all mirrored 1:1 in DroneCAN.

| Message | Carries | Rate | Optional |
|---|---|---|---|
| `GNSS` | fix, position, velocity, accuracies, yaw | 5–10 Hz | no — the base stream |
| `GNSS_INTEGRITY` | receiver-level health, jamming/spoofing/auth summary, RAIM, antenna, uptime | 1 Hz | yes |
| `GNSS_BANDS` | per-RF-band interference: frequency, jammed, mitigated | 1 Hz | yes |
| `GNSS_SATS` | per-satellite: constellation, svid, signal, C/N0, elevation, azimuth, flags | ≤1 Hz, or on demand | yes |

The split is by **rate and by audience**. `GNSS` is what the vehicle flies on and what a GCS always needs. The other three are diagnostics: a GCS asks for them with `MAV_CMD_SET_MESSAGE_INTERVAL` when a status page is open, or a log records them at a low rate for post-flight. On a 57600 telemetry link that difference decides whether the family is usable at all.

```
GNSS            5-10 Hz   fix + PVT + accuracy          always on
  │
  ├─ GNSS_INTEGRITY  1 Hz  receiver health & threats    on demand / logged
  ├─ GNSS_BANDS      1 Hz  which frequency is affected  on demand / logged
  └─ GNSS_SATS      <=1 Hz which satellites, on which   on demand / logged
                           signal, at what C/N0
```

---

## 3. `GNSS` — the base message

Take #2146 as written. It is `GPS_RAW_INT` with an `id` instance field, floats instead of scaled integers for the accuracies, and a datum enum. Nothing here needs re-litigating.

Two things to settle:

- **`satellites_visible` must mean visible**, and a separate `satellites_used` must exist. Getting this wrong once has already cost the ecosystem a decade of mislabelled GCS displays. If only one can be afforded, carry `satellites_used` under its real name and let `GNSS_SATS` supply the rest.
- **Message id 442 is claimed twice** — by `GNSS` in #2146 and by `GNSS_BANDS` in RFC #30. Needs resolving before either lands.

---

## 4. `GNSS_INTEGRITY` — receiver-level health

RFC #30's version, unmodified. It is well researched: every field is mapped to a concrete Septentrio and u-blox source, vendor-specific 0–10 abstract scales are removed, and `raim_hfom`/`raim_vfom` are correctly renamed to `raim_hpl`/`raim_vpl` (protection levels are integrity bounds, not accuracy estimates).

Worth supporting explicitly: **`cpu_load`, `up_time`, `corrections_age`**. The RFC asks whether they earn their place. They do — they are exactly the fields that turn "the GPS is being weird" into a diagnosis without recovering the vehicle. `up_time` in particular catches a receiver that is silently resetting.

---

## 5. `GNSS_BANDS` — per-band interference

RFC #30's version. Minimal variant: `frequency`, `band_jamming_state`, `band_mitigation_state`, arrays indexed by band with a `band_count`.

The one thing worth pushing on: **the interference characteristics from Alternative 1 (`interference_bandwidth` in kHz, `interference_power` in dBm) should be in the base message, not an alternative.** They are in standard units, Septentrio populates them today, and they are the difference between "L1 is jammed" and "L1 is jammed by a 200 kHz carrier at -70 dBm" — the second tells an operator whether to fly away from it or land. u-blox not populating them costs two invalid-marked fields.

The RFC's array-vs-repeated-message analysis lands correctly on arrays (32 vs 57 bytes/sec for three bands), with the caveat that a large `GNSS_MAX_BANDS` wastes payload on unused entries. See §6 — `GNSS_SATS` has the same problem an order of magnitude worse, and its answer should be reused here.

---

## 6. `GNSS_SATS` — per-satellite detail

The new one, and the one with no prior art. It supersedes `GPS_STATUS`.

**Requirement 1: a unique satellite identity.** Carry `constellation` (enum) and `svid` (constellation-local) as separate fields. `(constellation, svid)` is unique by construction and needs no lookup table. This is what the 8-bit PRN squash cannot do.

**Requirement 2: a signal identity.** Carry which signal the row describes — L1C/A, L2C, L5, E1, E5b, B1I, B2a. A multi-band receiver reports one row per satellite *per signal*, which is the only way a GCS or a production test can tell an L1-only unit from a dual-band one whose L5 path is dead.

**Requirement 3: express more than 20.** A receiver tracking 40 satellites on 2–3 signals produces 80–120 rows. This is the flaw that motivated the whole exercise, so the fix has to be explicit rather than incidental:

- Pack rows into a fixed array sized to fit one MAVLink payload (~25 rows at 8 bytes each).
- Carry **`total_count`** (rows across the whole set) and **`start_index`** (where this message's array begins).
- A receiver sends `ceil(total_count / array_size)` messages per cycle; a consumer knows when it has the whole set and can drop a partial one.

Merging on the satellite key alone would also work — the rows are self-describing — but an explicit offset means a GCS can tell a complete snapshot from a torn one, and can age out satellites that dropped from view. Self-describing rows plus no framing is how `GPS_STATUS` ended up ambiguous.

Sketch, ~8 bytes per row:

| Field | Type | Notes |
|---|---|---|
| `id` | `uint8_t` | receiver instance |
| `total_count` | `uint8_t` | rows in the full set, across all messages this cycle |
| `start_index` | `uint8_t` | index of the first row in this message |
| `count` | `uint8_t` | rows in this message |
| `constellation[]` | `uint8_t` | GPS / SBAS / Galileo / BeiDou / QZSS / GLONASS / NavIC |
| `svid[]` | `uint8_t` | constellation-local; with `constellation`, unique |
| `signal[]` | `uint8_t` | L1C/A, L2C, L5, E1, E5b, B1I, B2a, … ; 0 = unspecified |
| `cn0[]` | `uint8_t` | dB-Hz, 0 = not tracking |
| `elevation[]` | `int8_t` | deg, −90..90 |
| `azimuth[]` | `uint8_t` | deg/2, 0..179 |
| `flags[]` | `uint8_t` | bitmask: used in solution, healthy, ephemeris available, corrections applied, authenticated |

`elevation` and `azimuth` are there for a sky plot — the one GCS GNSS view that makes a bad antenna placement obvious at a glance.

**Rate.** This is the expensive one: 40 satellites is ~2 messages of ~215 bytes, so 1 Hz costs ~430 B/s — more than a 5 Hz `GNSS` stream. It should default to **off** and be requested. Post-flight logging at 0.2 Hz is the other real use.

**Open questions:**

- Is the signal-level row the right granularity, or should it be one row per satellite with a band bitmask? The bitmask is far cheaper (no row multiplication) but loses per-signal C/N0, which is the measurement that actually shows a dead band.
- `GNSS_MAX_SATS` sizing, and whether the padding waste the RFC identifies for `GNSS_BANDS` argues for the same `start_index` chunking there.
- Does the `used` flag need to distinguish "used for position" from "used for time only"?

---

## 7. DroneCAN

Every message above needs a DroneCAN equivalent, or CAN receivers stay second-class and PX4 has to maintain two divergent notions of what a GPS can report. This matters directly for ARK's own CAN GPS nodes.

Two things make this easier than the MAVLink side:

- DroneCAN multi-frame transfers handle large payloads natively, so `GNSS_SATS` does not need `start_index` chunking on CAN — though keeping the field aligned with MAVLink is probably worth more than the bytes saved.
- The flight controller is the translator either way. If the DroneCAN and MAVLink structures are 1:1, the driver is a field copy and there is nowhere for a semantic gap to hide.

Bus load is the constraint to watch: `GNSS_SATS` at 1 Hz per node on a shared bus with two receivers is real traffic. Same answer — default off, request it.

Sequencing: settle the MAVLink definitions first, then mirror. Doing both at once means renaming twice.

---

## 8. Why this is worth doing

- **The GCS can show a real GNSS page.** Sky plot, per-band interference, per-constellation tracking, receiver health — instead of a satellite count and a fix type.
- **The operator gets an action.** "L1 jammed at 1575.42 MHz, mitigated, L5 clean" is a decision. "GPS glitch" is not.
- **Remote debugging.** A receiver misconfigured to a single constellation, an antenna with no L5 coverage, a receiver quietly resetting, a spoofing event — all diagnosable from a log or a live link, without the vendor's tool and a bench.
- **Production test gets the same data.** The ARK GNSS benches currently work around `GPS_STATUS`'s 20-satellite cap by shelling into the flight controller and reading `satellite_info` over NSH. That is a workaround for a protocol gap, and it only exists because the bench happens to own both ends.
- **It ends the GPS-era naming.** `GPS_RAW_INT` with a `satellites_visible` field that holds the used count, and a `satellite_prn` that is not a PRN, are the kind of thing that costs every downstream implementer an afternoon, forever.

---

## 9. Next steps

1. Land the id-442 collision resolution between #2146 and RFC #30.
2. Get `GNSS_SATS` in front of the RFC #30 author — it is the same vendor-agnostic exercise, one layer down, and the Septentrio/u-blox mapping work should be done the same way.
3. Prototype against a u-blox F9/X20 and a Septentrio Mosaic in PX4 to find out what the drivers can actually populate before the fields are frozen.
4. Mirror to DroneCAN once the MAVLink definitions settle.
