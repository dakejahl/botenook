# GNSS: data reporting over MAVLink and DroneCAN

Status: **proposal with draft XML**, written 2026-08-13, revised 2026-08-21 after checking RFC #30 against the u-blox F9 HPG 1.51 and X20 HPG 2.10 interface descriptions and the PX4/QGC consumers. The problem picture (§1) is receipted; the message family (§3–§6) is a proposal with concrete XML, not a landed design. §10 lists where it deliberately departs from RFC #30.

Goal in one line: a GNSS message family that can express **everything a modern multi-band, multi-constellation receiver knows**, so a GCS can show full status, an operator gets something actionable, and a receiver can be debugged remotely over the link instead of with the vendor's tool on a bench.

Related work:

- [mavlink/rfcs#30](https://github.com/mavlink/rfcs/pull/30) — split `GNSS_INTEGRITY` into receiver-level integrity plus a per-frequency `GNSS_BANDS`. Vendor-agnostic, Septentrio + u-blox + NovAtel field mapping. The concrete prior art for §4 and §5.
- [mavlink/mavlink#2146](https://github.com/mavlink/mavlink/pull/2146) — `GNSS`, a `GPS_RAW_INT` / `GPS2_RAW` replacement with an instance id. Now carries `satellites_used` under its real name.
- `GNSS_INTEGRITY` (id 441) already sits in `development.xml`.
- [PX4-GPSDrivers#231](https://github.com/PX4/PX4-GPSDrivers/pull/231) — u-blox driver reads the receiver-level jamming state from `UBX-SEC-SIG`; the per-frequency groups that feed §5 are parsed as far as the header and carry a TODO for the rest.
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

**Nothing carries RF or integrity detail.** Jamming and spoofing are receiver-level booleans at best. Which frequency, mitigated or not, what the AGC and noise floor are doing — none of it reaches the link, so diagnosing a GNSS problem in the field means recovering the vehicle and attaching u-center or RxTools.

**What does exist is compiled out.** `GNSS_INTEGRITY` lives in `development.xml`; PX4 builds hardware targets against `common`, so only SITL and the `mavlink-dev` board variants stream it. QGC has a complete GPS resilience indicator wired to it (`GPSResilienceIndicator.qml`) that no stock vehicle has ever lit.

**DroneCAN receivers are second-class.** `uavcan.equipment.gnss.Auxiliary` is thinner still. PX4 smuggles `jamming_state` and `spoofing_state` between its own nodes by packing them into `Fix2.ecef_position_velocity.position_xyz_mm[2]`, which cannot grow to carry arrays. Whatever a serial receiver can report, a CAN node should be able to report identically.

There is one piece of good news: **`GPS_STATUS` is nearly unused.** QGC does not reference it anywhere in its source, and ArduPilot never sends it. Superseding it costs almost nothing.

---

## 2. Shape of the proposal

Four messages, one base plus three optional. All keyed by the same `id` instance field so a GCS can assemble a per-receiver view, and all mirrored 1:1 in DroneCAN.

| Message | Carries | Rate | Optional |
|---|---|---|---|
| `GNSS` | fix, position, velocity, accuracies, yaw | 5–10 Hz | no — the base stream |
| `GNSS_INTEGRITY` | receiver-level health, jamming/spoofing/auth summary, RAIM, antenna, uptime | 1 Hz | yes |
| `GNSS_BANDS` | per-center-frequency interference: frequency, jammed, mitigated, bandwidth, power | 1 Hz | yes |
| `GNSS_SATS` | per-signal: constellation, svid, signal, C/N0, elevation, azimuth, flags | ≤1 Hz, or on demand | yes |

The split is by **rate and by audience**. `GNSS` is what the vehicle flies on and what a GCS always needs. The other three are diagnostics: a GCS asks for them with `MAV_CMD_SET_MESSAGE_INTERVAL` when a status page is open, or a log records them at a low rate for post-flight. On a 57600 telemetry link that difference decides whether the family is usable at all.

```
GNSS            5-10 Hz   fix + PVT + accuracy          always on
  │
  ├─ GNSS_INTEGRITY  1 Hz  receiver health & threats    on demand / logged
  ├─ GNSS_BANDS      1 Hz  which frequency is affected  on demand / logged
  └─ GNSS_SATS      <=1 Hz which satellites, on which   on demand / logged
                           signal, at what C/N0
```

Message ids: `GNSS` keeps 442 from #2146. `GNSS_BANDS` moves from the 442 in RFC #30 to 443, and `GNSS_SATS` takes 444. The collision has to be settled in one place before either PR lands; this doc assumes #2146 wins because it is the base stream.

---

## 3. `GNSS` — the base message

Take #2146 as written. It is `GPS_RAW_INT` with an `id` instance field, floats instead of scaled integers for the accuracies, a datum enum, and `satellites_used` named for what it holds. The visible count is not in it and should not be: `GNSS_SATS.total_count` supplies it at the diagnostic rate.

---

## 4. `GNSS_INTEGRITY` — receiver-level health

RFC #30's version with the enum fixes in §10. Every field is mapped to a concrete Septentrio and u-blox source, vendor-specific 0–10 abstract scales are removed, and `raim_hfom`/`raim_vfom` are correctly renamed to `raim_hpl`/`raim_vpl` (protection levels are integrity bounds, not accuracy estimates).

Worth supporting explicitly: **`cpu_load`, `up_time`, `corrections_age`**. The RFC asks whether they earn their place. They do — they are exactly the fields that turn "the GPS is being weird" into a diagnosis without recovering the vehicle. `up_time` in particular catches a receiver that is silently resetting.

```xml
<message id="441" name="GNSS_INTEGRITY">
    <description>Receiver-level integrity and resilience status: jamming and spoofing summary, signal authentication, RAIM, antenna and system health. Per-frequency RF diagnostics are in GNSS_BANDS.</description>
    <field type="uint8_t" name="id" instance="true">GNSS receiver id. Must match instance ids of other messages from same receiver.</field>
    <field type="uint32_t" name="system_errors" enum="GNSS_SYSTEM_ERROR_FLAGS">Bitmask of errors in the GNSS system. Vendors set only the bits they can detect.</field>
    <field type="uint8_t" name="antenna_state" enum="GNSS_ANTENNA_STATE">Status of the main antenna supervisor.</field>
    <field type="uint8_t" name="antenna_power" enum="GNSS_ANTENNA_POWER">Power state of the main antenna.</field>
    <field type="uint8_t" name="cpu_load" units="%" invalid="UINT8_MAX">Receiver CPU load.</field>
    <field type="uint32_t" name="up_time" units="s" invalid="UINT32_MAX">Time since receiver startup or last reset.</field>
    <field type="uint16_t" name="corrections_age" units="cs" invalid="UINT16_MAX">Age of the most recently applied differential corrections. A receiver that only reports an age range reports the upper edge of the range.</field>
    <field type="uint8_t" name="authentication_state" enum="GNSS_AUTHENTICATION_STATE">Navigation message authentication state (e.g. Galileo OSNMA).</field>
    <field type="uint8_t" name="jamming_state" enum="GNSS_JAMMING_STATE">Receiver-level jamming state. Worst case across GNSS_BANDS entries where the receiver does not compute its own summary.</field>
    <field type="uint8_t" name="spoofing_state" enum="GNSS_SPOOFING_STATE">Receiver-level spoofing state.</field>
    <field type="uint8_t" name="raim_state" enum="GNSS_RAIM_STATE">Status of position-domain RAIM processing.</field>
    <field type="uint16_t" name="raim_hpl" units="cm" invalid="UINT16_MAX">Horizontal Protection Level: statistical upper bound on the horizontal position error.</field>
    <field type="uint16_t" name="raim_vpl" units="cm" invalid="UINT16_MAX">Vertical Protection Level: statistical upper bound on the vertical position error.</field>
</message>
```

```xml
<enum name="GNSS_JAMMING_STATE">
    <description>Jamming state. Values are ordered by severity so a consumer can take the maximum across receivers or frequencies.</description>
    <entry value="0" name="GNSS_JAMMING_STATE_UNKNOWN"><description>No jamming information available.</description></entry>
    <entry value="1" name="GNSS_JAMMING_STATE_SPECTRUM_CLEAN"><description>No jamming indicators detected.</description></entry>
    <entry value="2" name="GNSS_JAMMING_STATE_MITIGATED"><description>Jamming detected and actively mitigated by the receiver.</description></entry>
    <entry value="3" name="GNSS_JAMMING_STATE_DETECTED"><description>Jamming detected and not mitigated.</description></entry>
</enum>
<enum name="GNSS_SPOOFING_STATE">
    <description>Spoofing state. Values are ordered by severity.</description>
    <entry value="0" name="GNSS_SPOOFING_STATE_UNKNOWN"><description>No spoofing information available.</description></entry>
    <entry value="1" name="GNSS_SPOOFING_STATE_SPECTRUM_CLEAN"><description>No spoofing indicators detected.</description></entry>
    <entry value="2" name="GNSS_SPOOFING_STATE_DETECTED"><description>Spoofing indicators detected.</description></entry>
    <entry value="3" name="GNSS_SPOOFING_STATE_AFFECTED"><description>The receiver indicates its measurements or PVT may be affected by non-authentic signals.</description></entry>
</enum>
<enum name="GNSS_AUTHENTICATION_STATE">
    <description>Navigation message authentication state.</description>
    <entry value="0" name="GNSS_AUTHENTICATION_STATE_UNKNOWN"><description>No authentication information available.</description></entry>
    <entry value="1" name="GNSS_AUTHENTICATION_STATE_INITIALIZING"><description>Authentication is initializing (key material, time synchronisation).</description></entry>
    <entry value="2" name="GNSS_AUTHENTICATION_STATE_ERROR"><description>Authentication failed to initialize.</description></entry>
    <entry value="3" name="GNSS_AUTHENTICATION_STATE_AUTHENTICATING"><description>Authentication is running; the current solution has not been verified.</description></entry>
    <entry value="4" name="GNSS_AUTHENTICATION_STATE_DISABLED"><description>Authentication is disabled on the receiver.</description></entry>
    <entry value="5" name="GNSS_AUTHENTICATION_STATE_AUTHENTICATED"><description>Authentication is running and the current solution is verified against authenticated navigation data.</description></entry>
</enum>
```

`GNSS_SYSTEM_ERROR_FLAGS`, `GNSS_ANTENNA_STATE`, `GNSS_ANTENNA_POWER` and `GNSS_RAIM_STATE` as in RFC #30.

---

## 5. `GNSS_BANDS` — per-frequency interference

RFC #30's message with the interference characteristics from its Alternative 1 promoted into the base, and the unit of reporting stated plainly: **entries are center frequencies the receiver reports on, not RF front ends.** u-blox `UBX-SEC-SIG` lists one entry per center frequency with at least one in-use signal — an X20 with every constellation enabled reports up to eight (1575.42, 1561.098, 1602, 1227.6, 1246, 1268.52, 1278.75, 1176.45 MHz), three of which share the L1 front end. Septentrio `RFStatus.RFBand` lists interference detections, each with a frequency and bandwidth, and lists nothing when the spectrum is clean. Both are frequency lists; neither is a front-end list. The GCS derives "L1 / L2 / L5" from the frequency for display and keeps the exact value in the detail view.

`interference_bandwidth` (kHz) and `interference_power` (dBm) are in standard units, Septentrio populates them today, and they are the difference between "L1 is jammed" and "L1 is jammed by a 200 kHz carrier at -70 dBm" — the second tells an operator whether to fly away from it or land. u-blox not populating them costs two invalid-marked arrays.

Array size is 8, sized to the u-blox worst case. Entries past `count` are zero-filled so MAVLink 2 payload truncation drops them; validity is governed by `count`, not by per-entry invalid markers, because a 0xFF marker in an unused entry defeats truncation. Three entries cost 74 payload bytes before truncation, an empty message a handful.

```xml
<message id="443" name="GNSS_BANDS">
    <description>Per-frequency RF interference diagnostics for a GNSS receiver. One entry per center frequency the receiver reports on; the set is dynamic and may be empty. Several signals may share a center frequency and several entries may share an RF front end. Receiver-level summary states are in GNSS_INTEGRITY.</description>
    <field type="uint8_t" name="id" instance="true">GNSS receiver id. Must match instance ids of other messages from same receiver.</field>
    <field type="uint8_t" name="count">Number of valid entries in the arrays below. Entries past count are zero.</field>
    <field type="uint32_t[8]" name="frequency" units="Hz">Center frequency of each entry.</field>
    <field type="uint8_t[8]" name="jamming_state" enum="GNSS_JAMMING_STATE">Jamming state at each frequency.</field>
    <field type="uint8_t[8]" name="mitigation_state" enum="GNSS_JAMMING_MITIGATION_STATE">Mitigation applied at each frequency.</field>
    <field type="uint16_t[8]" name="interference_bandwidth" units="kHz" invalid="[UINT16_MAX]">Bandwidth of the detected interference at each frequency. 0 for pulsed interference.</field>
    <field type="int8_t[8]" name="interference_power" units="dBm" invalid="[INT8_MIN]">Estimated interference power at each frequency.</field>
</message>
<enum name="GNSS_JAMMING_MITIGATION_STATE">
    <description>Per-frequency jamming mitigation state.</description>
    <entry value="0" name="GNSS_JAMMING_MITIGATION_UNKNOWN"><description>Not reported by this receiver.</description></entry>
    <entry value="1" name="GNSS_JAMMING_MITIGATION_NOT_MITIGATED"><description>Interference present, no mitigation applied.</description></entry>
    <entry value="2" name="GNSS_JAMMING_MITIGATION_CANCELLED"><description>Interference cancelled autonomously by the receiver.</description></entry>
    <entry value="3" name="GNSS_JAMMING_MITIGATION_SUPPRESSED"><description>Frequency suppressed by an operator-configured notch filter.</description></entry>
</enum>
```

Vendor mapping, as far as it is known today:

| Field | Septentrio | u-blox |
|---|---|---|
| `count`, `frequency` | `RFStatus.N`, `RFBand.Frequency` | `SEC-SIG.jamNumCentFreqs`, `jamStateCentFreq.centFreq` × 1000 |
| `jamming_state` | `RFBand.Info.Mode` | `jamStateCentFreq.jammed` → CLEAN / DETECTED |
| `mitigation_state` | `RFBand.Info.Mode` bits 0–3 | `MON-RF.cwSuppression` of the front end covering the frequency (block ↔ frequency via `rfBlockGnssBand` on HPG 2.10); RFC #30 lists this as unavailable |
| `interference_bandwidth`, `interference_power` | `RFBand.Bandwidth`, `RFBand.Power` | invalid |

---

## 6. `GNSS_SATS` — per-signal detail

The new one, and the one with no prior art. It supersedes `GPS_STATUS`.

**Requirement 1: a unique satellite identity.** Carry `constellation` (enum) and `svid` (constellation-local) as separate fields. `(constellation, svid)` is unique by construction and needs no lookup table. This is what the 8-bit PRN squash cannot do.

**Requirement 2: a signal identity.** Carry which signal the row describes — L1C/A, L2C, L5, E1, E5b, B1I, B2a. A multi-band receiver reports one row per satellite *per signal*, which is the only way a GCS or a production test can tell an L1-only unit from a dual-band one whose L5 path is dead. u-blox provides exactly this in `UBX-NAV-SIG` (per-signal C/N0, quality, pseudorange/carrier used, health, OSNMA `authStatus`); elevation and azimuth come from `UBX-NAV-SAT` per satellite, so a row is a join on `(gnssId, svId)`. That settles the earlier open question: the per-signal row is available, no band bitmask compromise is needed.

**Requirement 3: express more than 20.** A receiver tracking 40 satellites on 2–3 signals produces 80–120 rows. This is the flaw that motivated the whole exercise, so the fix has to be explicit rather than incidental:

- Pack rows into a fixed array sized to fit one MAVLink payload: 25 rows at 8 bytes each.
- Carry **`total_count`** (rows across the whole set) and **`start_index`** (where this message's array begins).
- A receiver sends `ceil(total_count / 25)` messages per cycle; a consumer knows when it has the whole set and can drop a partial one.

Merging on the satellite key alone would also work — the rows are self-describing — but an explicit offset means a GCS can tell a complete snapshot from a torn one, and can age out satellites that dropped from view. Self-describing rows plus no framing is how `GPS_STATUS` ended up ambiguous.

```xml
<message id="444" name="GNSS_SATS">
    <description>Per-signal satellite tracking for a GNSS receiver. A receiver with more rows than fit in one message sends several per cycle; total_count and start_index let a consumer assemble a complete set or discard a torn one. Supersedes GPS_STATUS.</description>
    <field type="uint8_t" name="id" instance="true">GNSS receiver id. Must match instance ids of other messages from same receiver.</field>
    <field type="uint8_t" name="total_count">Rows in the full set this cycle, across all messages.</field>
    <field type="uint8_t" name="start_index">Index of the first row of this message within the full set.</field>
    <field type="uint8_t" name="count">Valid rows in this message. Rows past count are zero.</field>
    <field type="uint8_t[25]" name="constellation" enum="GNSS_CONSTELLATION">Constellation of each row.</field>
    <field type="uint8_t[25]" name="svid">Constellation-local satellite id: PRN for GPS, SBAS, Galileo, BeiDou, QZSS and NavIC; orbital slot for GLONASS.</field>
    <field type="uint8_t[25]" name="signal" enum="GNSS_SIGNAL">Signal each row describes.</field>
    <field type="uint8_t[25]" name="cn0" units="dBHz">Carrier-to-noise density. 0 if not tracked.</field>
    <field type="int8_t[25]" name="elevation" units="deg" invalid="[INT8_MIN]">Elevation, -90..90.</field>
    <field type="uint16_t[25]" name="azimuth" units="deg" invalid="[UINT16_MAX]">Azimuth, 0..359.</field>
    <field type="uint8_t[25]" name="flags" enum="GNSS_SAT_FLAGS" display="bitmask">Per-row status flags.</field>
</message>
<enum name="GNSS_CONSTELLATION">
    <description>GNSS constellation. Numbering follows the u-blox gnssId convention.</description>
    <entry value="0" name="GNSS_CONSTELLATION_GPS"/>
    <entry value="1" name="GNSS_CONSTELLATION_SBAS"/>
    <entry value="2" name="GNSS_CONSTELLATION_GALILEO"/>
    <entry value="3" name="GNSS_CONSTELLATION_BEIDOU"/>
    <entry value="4" name="GNSS_CONSTELLATION_IMES"/>
    <entry value="5" name="GNSS_CONSTELLATION_QZSS"/>
    <entry value="6" name="GNSS_CONSTELLATION_GLONASS"/>
    <entry value="7" name="GNSS_CONSTELLATION_NAVIC"/>
</enum>
<enum name="GNSS_SIGNAL">
    <description>GNSS signal component.</description>
    <entry value="0" name="GNSS_SIGNAL_UNSPECIFIED"><description>Receiver does not report per-signal detail.</description></entry>
    <entry value="1" name="GNSS_SIGNAL_GPS_L1CA"/>
    <entry value="2" name="GNSS_SIGNAL_GPS_L1C"/>
    <entry value="3" name="GNSS_SIGNAL_GPS_L2C"/>
    <entry value="4" name="GNSS_SIGNAL_GPS_L5"/>
    <entry value="5" name="GNSS_SIGNAL_SBAS_L1CA"/>
    <entry value="6" name="GNSS_SIGNAL_GAL_E1"/>
    <entry value="7" name="GNSS_SIGNAL_GAL_E5A"/>
    <entry value="8" name="GNSS_SIGNAL_GAL_E5B"/>
    <entry value="9" name="GNSS_SIGNAL_GAL_E6"/>
    <entry value="10" name="GNSS_SIGNAL_BDS_B1I"/>
    <entry value="11" name="GNSS_SIGNAL_BDS_B1C"/>
    <entry value="12" name="GNSS_SIGNAL_BDS_B2I"/>
    <entry value="13" name="GNSS_SIGNAL_BDS_B2A"/>
    <entry value="14" name="GNSS_SIGNAL_BDS_B3I"/>
    <entry value="15" name="GNSS_SIGNAL_QZSS_L1CA"/>
    <entry value="16" name="GNSS_SIGNAL_QZSS_L1S"/>
    <entry value="17" name="GNSS_SIGNAL_QZSS_L1CB"/>
    <entry value="18" name="GNSS_SIGNAL_QZSS_L2C"/>
    <entry value="19" name="GNSS_SIGNAL_QZSS_L5"/>
    <entry value="20" name="GNSS_SIGNAL_GLO_L1OF"/>
    <entry value="21" name="GNSS_SIGNAL_GLO_L2OF"/>
    <entry value="22" name="GNSS_SIGNAL_NAVIC_L5"/>
</enum>
<enum name="GNSS_SAT_FLAGS" bitmask="true">
    <description>Per-signal status flags.</description>
    <entry value="1" name="GNSS_SAT_FLAGS_USED"><description>Used in the navigation solution.</description></entry>
    <entry value="2" name="GNSS_SAT_FLAGS_HEALTHY"><description>Signal flagged healthy by the constellation.</description></entry>
    <entry value="4" name="GNSS_SAT_FLAGS_EPHEMERIS"><description>Ephemeris available.</description></entry>
    <entry value="8" name="GNSS_SAT_FLAGS_CORRECTIONS"><description>Differential corrections applied to this signal.</description></entry>
    <entry value="16" name="GNSS_SAT_FLAGS_AUTHENTICATED"><description>Navigation data for this signal authenticated (e.g. OSNMA).</description></entry>
</enum>
```

`elevation` and `azimuth` are there for a sky plot — the one GCS GNSS view that makes a bad antenna placement obvious at a glance.

**Rate.** This is the expensive one: 40 satellites on two signals is ~4 messages of ~215 bytes, so 1 Hz costs ~860 B/s — more than a 5 Hz `GNSS` stream. It should default to **off** and be requested. Post-flight logging at 0.2 Hz is the other real use. On the receiver side `NAV-SIG` is 8 + 16 B per row at 1 Hz on the serial link and has to be stream-parsed the way the ubx driver already parses `NAV-SAT`.

**Open questions:**

- Does the `USED` flag need to distinguish "used for position" from "used for time only"?
- `GNSS_SIGNAL` is the u-blox signal list; Septentrio and NovAtel need checking for signals it cannot name.

---

## 7. DroneCAN

Every message above needs a DroneCAN equivalent, or CAN receivers stay second-class and PX4 has to maintain two divergent notions of what a GPS can report. This matters directly for ARK's own CAN GPS nodes, and the `Fix2` field-packing hack in §1 cannot be extended to carry any of the arrays.

Two things make this easier than the MAVLink side:

- DroneCAN multi-frame transfers handle large payloads natively, so `GNSS_SATS` does not need `start_index` chunking on CAN — though keeping the field aligned with MAVLink is probably worth more than the bytes saved.
- The flight controller is the translator either way. If the DroneCAN and MAVLink structures are 1:1, the driver is a field copy and there is nowhere for a semantic gap to hide.

Bus load is the constraint to watch: `GNSS_SATS` at 1 Hz per node on a shared bus with two receivers is real traffic. Same answer — default off, request it.

Sequencing: settle the MAVLink definitions first, then mirror. Doing both at once means renaming twice.

---

## 8. Why this is worth doing

- **The GCS can show a real GNSS page.** Sky plot, per-frequency interference, per-constellation tracking, receiver health — instead of a satellite count and a fix type.
- **The operator gets an action.** "1575.42 MHz jammed, mitigated; L5 clean" is a decision. "GPS glitch" is not. Amber for mitigated, red for detected-and-not-mitigated is the whole UI.
- **Remote debugging.** A receiver misconfigured to a single constellation, an antenna with no L5 coverage, a receiver quietly resetting, a spoofing event — all diagnosable from a log or a live link, without the vendor's tool and a bench.
- **Bring-up.** A USB3 hub next to the receiver shows up as L1 and L2 jammed on the ground with nothing else wrong. That is the case where an operator can act before the EKF ever sees it, and the answer to the objection that per-band data arrives too late to matter in flight.
- **Production test gets the same data.** The ARK GNSS benches currently work around `GPS_STATUS`'s 20-satellite cap by shelling into the flight controller and reading `satellite_info` over NSH. That is a workaround for a protocol gap, and it only exists because the bench happens to own both ends.
- **It ends the GPS-era naming.** `GPS_RAW_INT` with a `satellites_visible` field that holds the used count, and a `satellite_prn` that is not a PRN, are the kind of thing that costs every downstream implementer an afternoon, forever.

---

## 9. Next steps

1. Land the id-442 collision resolution between #2146 and RFC #30 (this doc assumes 442 → `GNSS`, 443 → `GNSS_BANDS`, 444 → `GNSS_SATS`).
2. Get §10 in front of the RFC #30 author; the enum numbering and the authentication result are the two that change consumer code.
3. Get `GNSS_SATS` in front of the RFC #30 author — it is the same vendor-agnostic exercise, one layer down, and the Septentrio/u-blox mapping work should be done the same way.
4. Prototype against a u-blox F9/X20 and a Septentrio Mosaic in PX4 to find out what the drivers can actually populate before the fields are frozen. The u-blox side starts from PX4-GPSDrivers#231: the `SEC-SIG` repeated group, a 1 Hz `sensor_gnss_bands` uORB topic rather than growing `sensor_gps`, and `NAV-SIG` stream parsing.
5. Get `GNSS_INTEGRITY` (and the rest) into `common.xml`, or nothing above reaches a stock flight controller.
6. Mirror to DroneCAN once the MAVLink definitions settle.

---

## 10. Where this departs from RFC #30

Checked against the u-blox F9 HPG 1.51 and X20 HPG 2.10 interface descriptions, PX4's consumers, and QGC.

- **Enum numbering.** The RFC renumbers `GNSS_JAMMING_STATE` to 2 = DETECTED, 3 = MITIGATED, which inverts severity relative to the existing values and to its own spoofing enum (2 = DETECTED, 3 = AFFECTED). PX4's `sensor_gps.msg` constants, the ubx driver mapping, and QGC's indicator colour (`max(spoofing, jamming)`) all assume 3 is worst. §4 keeps the numbers and renames only.
- **Authentication result.** The RFC redefines `OK` as "operating normally" and routes authentication failures to `spoofing_state`. u-blox reports the result separately (`NAV-PVT.nmaFixStatus`, `authTime`, `NAV-SIG.authStatus`) and its ICD states that an unverified fix does not imply spoofing, so the one thing OSNMA exists to answer had no field. §4 adds `AUTHENTICATED` and renames `OK` to `AUTHENTICATING`.
- **Unit of reporting.** The RFC's `GNSS_BANDS` is described as "sent once per RF front-end / frequency band" and sized for three entries. Both vendors report frequency lists, and u-blox reports up to eight. §5 says so and sizes the array to 8.
- **u-blox mitigation.** The RFC marks `band_mitigation_state` unavailable on u-blox. `MON-RF.cwSuppression` reads as exactly that — the CW suppression level in effect per front end — and the front end covering a frequency is known.
- **RAIM on u-blox.** The RFC maps `raim_state` to `UBX-TIM-TP.raim`, which is the timing-RAIM flag for the time pulse, not position integrity. u-blox has no position RAIM state or protection level; the fields stay invalid.
- **`corrections_age` on u-blox.** `NAV-PVT.lastCorrectionAge` is a 4-bit bin (0–1 s … ≥120 s). §4 states that a receiver reporting a range reports the upper edge, so the centisecond unit does not imply precision the receiver lacks.
- **Interference characteristics** move from Alternative 1 into the base message (§5).
- **Per-signal authentication and spoofing** are not speculative on u-blox: `NAV-SIG` carries `authStatus` per signal and X20 HPG 2.10 refers to a per-signal `spfState`. `GNSS_SATS` carries the authentication bit; per-signal spoofing is left out until a second vendor has it.
- **`system_errors` on u-blox** can draw on `MON-SYS` (`errorCount`, `warnCount`), not only the antenna status.
- **Message ids.** 442 collides with #2146; §2 moves `GNSS_BANDS` to 443.
