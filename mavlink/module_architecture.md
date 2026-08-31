# MAVLink module architecture

Status: **prerequisite reading**, not a design. Written 2026-08-30 against PX4 `main` at `436cc71c89` (`src/modules/mavlink/`), revised the same day. Read this before you plan the new module. Treat the target sketch as requirements to evaluate, not as decided internals. Stream catalog and user profiles are in [`streams_metadata.md`](streams_metadata.md).

Audience: an engineer or agent who is about to write a design for a **new** MAVLink module. This note records how the current module works, which of its behaviors are the GCS contract versus defects, the rewrite constraints that are already decided, and the questions the design still has to answer.

## Takeaways

The current module already has two threads per instance. Both poll: the receiver on `poll(..., 10 ms)`, the other on `px4_usleep`. That is not an event-driven RX/TX split.

`MavlinkReceiver` parses inbound frames and owns the request/response services (parameters, missions, FTP, log download, timesync). Those services send on the receiver thread. The other thread sleeps, then walks every configured stream. There is no outbound mailbox. A full UART buffer drops the current packet in `send_start()`; the caller usually still believes it sent.

The instance rate parameter is `MAV_n_RATE` (CLI `-r`), in bytes per second. It does not meter bytes on the wire. It recomputes a stream-interval multiplier (`_rate_mult`) from a predicted bandwidth sum. Services never read that multiplier. They burst against `get_free_tx_buf()`. The class named `MavlinkRateLimiter` is unused on the telemetry path.

Reliability lives inside each protocol (mission retries, ULog ACKs, command-sender retries, FTP sequence cache). It is not a TX-layer property of a message. Best-effort telemetry, bulk transfers, and latency-sensitive replies share one packet buffer and one `write()`.

Write a new module. Keep the GCS-visible contract and the intended feature set. Do not keep the threading, the god-object receiver, the feed-forward rate multiplier, or the service internals that produce today's bursts and silent drops. QGC (`~/code/jake/qgroundcontrol`) is the human GCS contract. MAVSDK (`~/code/jake/MAVSDK`) is the automated, headless contract and the interface the test agents run against in PX4 SITL. Extend MAVSDK when a needed test surface is missing. KConfig must be able to compile out services, TX streams, and RX handlers independently.

Thread names are `mavlink_{i}_rx` and `mavlink_{i}_tx`. Blocking work runs on one dedicated `wq:mavlink`. Three instances are six hot threads plus that one work queue. A work queue per instance is a regression.

Stream sets: one JSON file per instance is the source of truth. `MAV_n_MODE` goes away. Factory snapshots live in ROMFS; Reset restores them. See [`streams_metadata.md`](streams_metadata.md).

## Decided for the rewrite

These are constraints, not open questions.

**Greenfield module.** New code, new layout. The existing `src/modules/mavlink/` tree is the behavior and defect reference. It is not a template to transcribe. Shipping (replace in place, parallel KConfig, migration) is still a design question; the source strategy is not.

**Holistic services.** Every microservice is open to scrutiny. Fix the design defects (process-wide singletons, RX-thread filesystem, unlocked TIMESYNC send, 256 KB log bursts, param list that does not throttle Normal-mode telemetry). Do not port a state machine unchanged because it happens to work with QGC today. Re-derive the service from the on-wire contract and the intended feature.

**Faithful GCS contract, not bit-identical internals.** No regressions in intended supported functionality. Accidental behavior (silent `send_start()` drops, TIMESYNC racing the TX mutex, param throttle only in `LOW_BANDWIDTH`) is in scope to change. If QGC, MAVSDK, or a documented PX4 GCS path depends on a sequence, preserve that sequence. Verify against QGC source for human GCS behavior, and against MAVSDK for everything an agent can assert in SITL.

**Three TX classes**, all going through one TX path that respects `MAV_n_RATE`:

| Class | Loss | Delay | Role |
|---|---|---|---|
| Latency-sensitive | Avoid | Keep short, even during a bulk transfer | Jump the queue, still consume tokens. TIMESYNC replies, PING replies, COMMAND_ACK. Membership beyond those is a design call. |
| Reliable | None | May wait for tokens | Queue until sent. Parameter download, mission transfer, FTP, log download, and similar. |
| Best-effort | Allowed | N/A | Telemetry streams. The scheduler may lower the rate when the link is busy, not below a per-stream floor (for example HEARTBEAT ≥ 1 Hz, ATTITUDE ≥ 5 Hz). |

A two-class bucket either delays TIMESYNC behind PARAM_VALUE or lets PING starve a reliable transfer. The third class exists so neither happens.

**Opportunistic best-effort throttle.** While a reliable transfer is active, lower best-effort rates toward their floors so the transfer gets bandwidth. Links have real bandwidth constraints; a 57600 baud SiK radio is not USB.

**KConfig granularity.** Compile-time enable/disable for (1) each microservice, (2) each TX stream, (3) each RX handler. Disabled means not compiled, not "configured at 0 Hz." A request for a compiled-out stream or service must fail cleanly on the wire (`COMMAND_ACK` unsupported / equivalent), not crash or stall the GCS.

**Thread names and count.** Per-instance hot paths are `mavlink_{i}_rx` and `mavlink_{i}_tx` (`{i}` is the instance id, same numbering as `MAV_n_*`). Blocking work runs on one dedicated `wq:mavlink` (new `wq_configurations` entry, same pattern as `wq:uavcan`). Not `lp_default` (FTP `fread` would stall other low-priority work) and not a queue per instance. See [Threading](#threading).

**SITL + MAVSDK is the iteration loop.** Design and implementation are driven by orchestration, implementation, reviewer, and test agents. The test agents speak MAVSDK against PX4 SITL, headless. QGC compatibility remains required; it is not the automated gate. See [Testing](#testing).

**Stream sets.** One complete JSON file per instance is the configured set. `MAV_n_MODE` goes away. ROMFS holds factory snapshots (today's Normal/Onboard/… tables); Reset copies that snapshot back. `SET_MESSAGE_INTERVAL` is live-only. GCS Save is FTP put of the file, applied immediately. Catalog is component metadata for discovery. Details: [`streams_metadata.md`](streams_metadata.md).

## Target architecture

This is the requested shape, restated as requirements a design must evaluate. It is not an implementation plan.

1. Split RX processing from TX processing. Each side is event-driven and stays on the hot path: parse or dequeue-and-write. Mailboxes (queues) are the interface. Threads are named `mavlink_{i}_rx` and `mavlink_{i}_tx`.
2. Services do not sit inside the receiver's parse loop. Filesystem and dataman run on `wq:mavlink`. Latency-sensitive replies go RX mailbox → TX mailbox and never wait on that queue. Hot threads never join or block on service work.
3. Every byte that leaves the instance goes through one TX path that respects `MAV_n_RATE`.
4. That path classifies each frame as latency-sensitive, reliable, or best-effort, per the table above.
5. A token bucket meters actual bytes: tokens refill at `MAV_n_RATE` bytes per second, a burst depth allows short clumps, reliable traffic waits for tokens rather than dropping, latency-sensitive traffic is dequeued first and still debits the bucket, and best-effort traffic consumes leftover tokens or is dropped.
6. KConfig selects which services, TX streams, and RX handlers exist in the binary. A full-featured board default matches today's intended feature set so QGC and MAVSDK do not regress. Constrained boards cut at this grain instead of one `CONSTRAINED_FLASH` blob.
7. Stream membership and rates come from the per-instance profile file (factory snapshot if none). `SET_MESSAGE_INTERVAL` mutates the live table only. See [`streams_metadata.md`](streams_metadata.md).

## Threading

Per-instance RX and TX, plus one dedicated `wq:mavlink` for the board.

### What exists today

Each instance is already two threads. Three instances are six, plus an extra `mavlink_shell` task if someone opens the nsh bridge.

| Today | Name | Role |
|---|---|---|
| TX / main | `mavlink_if{i}` | Sleep, walk streams |
| RX | `mavlink_rcv_if{i}` | `poll` one fd, parse, also TX of services |

A count of one thread per instance misses the receiver pthread. Three instances are already six.

### What not to do

Nine threads (RX + TX + work queue, times three instances) buys nothing the six hot threads do not already isolate, and it adds three stacks plus three schedulable entities that will sit idle or contend on the filesystem. PX4's own pattern is one work queue and many `WorkItem`s (`wq:uavcan`, per-tty `wq:ttyS*`).

Do not put all instances on one TX thread. Serial is opened `O_RDWR` without `O_NONBLOCK`; USB and RTS/CTS can block; UDP has a 10 ms `SO_SNDTIMEO`. A flow-controlled SiK UART that blocks in `write()` would stall USB HEARTBEAT and Ethernet on the same thread. Token buckets cannot run while the thread is in the kernel. Thread priority cannot help another instance on the same thread. Independent hardware endpoints get independent TX threads. That is why PX4 already split `wq:SPI0`..`SPI6` and `wq:ttyS0`..`ttyACM0`.

Do not run FTP `fread` on `wq:lp_default`. That queue is already the catch-all low-priority pool.

### Default

Event-driven means `poll` on the fd and the mailbox, not `usleep` then walk a list. Per-instance `poll` on one fd is that.

- `mavlink_{i}_rx` — high priority relative to TX. Parse, push inbound mailbox, never FS, never wait on the work queue.
- `mavlink_{i}_tx` — mid. Pull outbound mailbox, apply the instance token bucket (latency-sensitive, then reliable, then best-effort), write. If the write still blocks, it blocks **this** instance only. Prefer `POLLOUT` or a non-blocking write so this thread sleeps on the fd instead of spinning.
- `wq:mavlink` — low, one for the board. FTP, log download, dataman, param enumeration. Completes by posting frames into the instance TX mailbox. Add it next to `wq:uavcan` in `WorkQueueManager.hpp`.

Priority bands are RX > TX > WQ. Do not use thread priority *between instances* as QoS (telem1 TX above USB TX). That fights the token bucket. Intra-link QoS is the three TX classes plus floors. Inter-link isolation is separate threads.

Hot threads never wait for the work queue. The queue posts; TX dequeues. That is how you avoid priority inversion without priority inheritance. Inversion in the old module is RX holding `lock_send()` across `fread` while TX wants HEARTBEAT — the new split exists to make that wait structurally impossible.

### Cost versus today

| | Threads (3 instances) | Isolation |
|---|---|---|
| Today | 6 (+ optional shell) | Per instance, but RX does TX and FS |
| Rewrite | 6 hot + `wq:mavlink` | Per instance on the wire; FS cannot stall HEARTBEAT |
| WQ per instance | 9 | Same isolation, more stacks — do not |

Six plus one is the number to design for. Nine is not.

## Compatibility

Preserve intended functionality and the GCS-visible contract. Do not preserve internals.

**Intended** means: a GCS can connect, see HEARTBEAT, download and set parameters, transfer missions, run FTP (including component metadata and the param-file path), stream or download logs, send commands and get acks, receive STATUSTEXT and events, configure stream rates with `SET_MESSAGE_INTERVAL` / `REQUEST_MESSAGE`, and use the modes PX4 documents (Normal, Onboard, Iridium, Gimbal, …). HIL, forwarding, signing, shell, and serial passthrough stay if they are supported features of the board's KConfig, not because the old classes existed.

**Not intended** means: burst sizes, which thread ran `fread`, `_rate_mult` floors, unlocked TIMESYNC, two-slot forwarding, process-wide ULog and command-sender singletons, `send_start()` dropping a packet the caller counts as sent. Those are defects or accidents. A QGC check that only passes because of an accident must be called out in the design and either kept as a documented compatibility wart or fixed on both sides.

**QGC** (`~/code/jake/qgroundcontrol`) is the human GCS contract. During design, read the consumer before locking a service's on-wire behavior. Starting points:

| Contract | QGC |
|---|---|
| Heartbeat, initial connect, commands | `src/Vehicle/`, `src/Utilities/StateMachine/` (including `RequestMessageState`) |
| Parameters: `_HASH_CHECK`, `PARAM_REQUEST_LIST`, FTP fallback | `src/FactSystem/ParameterManager.cc` |
| FTP | `src/MAVLink/MAVLinkFTP.cc` |
| Stream rates | `src/MAVLink/MAVLinkStreamConfig.cc` |
| Missions | `src/MissionManager/` |
| Events, STATUSTEXT | `src/MAVLink/LibEvents/`, `src/MAVLink/StatusTextHandler.cc` |
| Onboard log download, console | `src/AnalyzeView/OnboardLogs/`, `src/AnalyzeView/MAVLinkConsole/` |
| Signing | `src/MAVLink/Signing/` |

**MAVSDK** (`~/code/jake/MAVSDK`) is the automated contract. Test agents run it against PX4 SITL. A sequence that QGC needs and MAVSDK cannot yet assert is a MAVSDK gap to fill in `~/code/jake/MAVSDK` (proto + plugin), not a reason to skip the test. A compiled-out stream that a GCS `SET_MESSAGE_INTERVAL`s still has to NAK in a way QGC already handles; the same NAK must be observable from MAVSDK (`mavlink_direct` COMMAND_ACK, or a dedicated API).

Documented companion-computer paths are also consumers. When QGC and MAVSDK disagree about a sequence, flag it in the design; do not pick silently.

## Testing

The rewrite is iterated by agents: orchestration, implementation, review, test. The test loop is PX4 SITL plus MAVSDK, headless, so a run does not need QGC or a radio.

`test/mavsdk_tests/` in PX4 today is a flight-behavior suite (takeoff, mission, RTL). It is not a MAVLink protocol suite. The new module needs its own tests that assert on-wire behavior: HEARTBEAT rate, param hash and list and FTP, mission upload/download, FTP burst, ULog streaming, events sequence, `SET_MESSAGE_INTERVAL` / `GET_MESSAGE_INTERVAL`, profile file as source of truth (Save applies without reboot, interval does not persist, Reset restores factory), token-bucket respect for `MAV_n_RATE`, opportunistic throttle during a reliable transfer, and NAK of a compiled-out stream. Stream-config cases are listed in [`streams_metadata.md`](streams_metadata.md).

MAVSDK plugins that already reach those surfaces:

| Surface | Plugin |
|---|---|
| Arbitrary TX/RX, rate counts, `SET_MESSAGE_INTERVAL` as COMMAND_LONG | `mavlink_direct` (`send_message`, `subscribe_message`) |
| Parameters | `param` |
| FTP, component metadata files | `ftp`, `component_metadata` |
| Missions | `mission`, `mission_raw` |
| Onboard logs, ULog stream | `log_files`, `log_streaming` |
| Events, shell, telemetry high-level | `events`, `shell`, `telemetry` |

`ComponentMetadata::MetadataType` today is Parameter, Events, Actuators. A streams catalog needs a proto addition in MAVSDK-Proto and a plugin update. `mavlink_direct` can still download the JSON over FTP and parse it in the test until that exists.

Put new MAVSDK APIs in `~/code/jake/MAVSDK` (fork workflow: no write access on upstream; PRs go through `dakejahl`). Prefer extending `mavlink_direct` or `component_metadata` over a one-off PX4 test harness that speaks raw UDP.

SITL is the right link: UDP, no UART flow control, fast iteration. Keep a short list of tests that only mean something on a bandwidth-capped serial instance (token bucket vs `MAV_n_RATE`, opportunistic throttle) and run those with `MAV_n_RATE` set low on the SITL UDP port, or on hardware later. Do not wait on hardware to gate the agent loop.

## Current architecture

Path: `src/modules/mavlink/`. One `Mavlink` object per instance (up to `MAVLINK_COMM_NUM_BUFFERS`, 4 or 6). Typical boards start three instances from `MAV_n_CONFIG`. This section is the defect catalog and the behavior reference. Copying it is the failure mode the rewrite exists to avoid.

### Threads

Each `mavlink start` spawns a PX4 task, then that task starts a receiver pthread.

| Thread | Name | How it waits | What it does |
|---|---|---|---|
| TX / main | `mavlink_if{i}` | `px4_usleep(_main_loop_delay)` | Recompute `_rate_mult`, drain uORB-driven side paths, walk `_streams`, ULog, events, one forwarded message |
| RX | `mavlink_rcv_if{i}` | `poll(..., 10 ms)` | Read UART/UDP, parse, `handle_message()`, then every ≥10 ms run mission/param/FTP/log-handler `send()` |

`_main_loop_delay` is `(10000 * 1000) / _datarate` microseconds, clamped to 1500–10000 µs (about 666 Hz to 100 Hz). At the default telem1 rate of 1200 B/s it is about 8.3 ms. The sleep is a CPU tradeoff, not a stream timer.

There is no work-queue item and no outbound queue of MAVLink frames. The live "queues" are the stream list, uORB subscriptions, a two-message forwarding ring buffer, and each service's internal state.

RX priority is `SCHED_PRIORITY_MAX - 80`. TX is `SCHED_PRIORITY_DEFAULT`. Stacks are small (RX about 4 KB plus net, TX about 3 KB plus net). Constrained boards already feel instance count. A rewrite that adds a thread per service, or a work queue per instance, will not fit. The default ([Threading](#threading)) keeps the hot-path count at two per instance.

### Send path

There is no `Mavlink::send_message()`. Callers use the generated C helpers (`mavlink_msg_*_send_struct`). Those call `MAVLINK_START_UART_SEND` / `MAVLINK_SEND_UART_BYTES` / `MAVLINK_END_UART_SEND`, bound in `mavlink_bridge_header.h` to `send_start()`, `send_bytes()`, `send_finish()`.

`send_start()` takes the per-channel recursive mutex (`lock_send()`). If the packet is larger than `get_free_tx_buf()`, it sets `_tx_buffer_low` and counts a TX error. `send_finish()` then returns without writing. The helper still returns to the caller as if the send ran. Most telemetry `send()` methods do not check free space first, so this is a silent drop.

The on-wire buffer is `_buf[MAVLINK_MAX_PACKET_LEN]` — one in-flight packet, not a queue. Serial `write()` is blocking (`O_RDWR | O_NOCTTY`, not `O_NONBLOCK`). NuttX callers that care use `ioctl(FIONSPACE)` via `get_free_tx_buf()`. UDP pretends the free size is 1500 bytes, or 15000 on POSIX, so burst gates that trust this value are not a rate limit.

The same mutex also protects the MAVLink C library's per-channel `mavlink_status_t` (TX sequence, parse state, flags). RX takes it around `mavlink_parse_char()` for each byte, then drops it before `handle_message()` because some handlers busy-wait on the TX thread (`configure_stream_threadsafe()`). TIMESYNC replies and `REQUEST_EVENT` send from the RX thread without that lock.

### How `MAV_n_RATE` is applied

There is no parameter named `MAV_RATE`. `module.yaml` defines `MAV_n_RATE` as "Maximum MAVLink sending rate for instance n" in B/s. The startup script passes it as `-r`. USB ACM forces 100000. A value of 0 becomes `baudrate / 20` (half of 8N1 theoretical). Cap is 10 MB/s.

That number is `_datarate`. `update_rate_mult()` (TX thread, every loop) does the following:

1. Sum `get_size_avg() * 1e6 / interval` over streams, split into `const_rate` versus variable.
2. If ULog is active, subtract its current fraction (started at 70% of `_datarate`).
3. `bandwidth_mult = (_datarate * ulog_inv - const_rate) / variable_rate`.
4. If parameters are sending **and** the instance mode is `LOW_BANDWIDTH`, cap that at 0.25.
5. Fold in radio software throttle (`RADIO_STATUS.txbuf` → `_radio_status_mult`) and TX-error congestion.
6. `_rate_mult = constrain(min(bandwidth_mult, hardware_mult), 0.05, 1.0)`.

`MavlinkStream::update()` divides `_interval` by `_rate_mult` unless `const_rate()` is true. Lower multiplier means a longer period, not a drop of a due message.

Consequences that the parameter text does not state:

- `_rate_mult` never exceeds 1, so headroom in `MAV_n_RATE` does not raise configured stream Hz.
- The 5% floor means radio-critical never silences ATTITUDE. Only the parameter sender honors `radio_status_critical()`.
- `get_size_avg()` is never overridden; it is `get_size()`. Streams that return 0 when idle (STATUSTEXT, COMMAND_LONG) are free in the budget. Unlimited (`interval < 0`) and manual (`interval == 0`) streams contribute 0.
- The budget is a **prediction** from configured intervals and sizes. It does not observe bytes actually written this tick.
- Services, COMMAND_ACK, forwarding, one-shot `REQUEST_MESSAGE`, TIMESYNC replies, shell, and serial passthrough are invisible to it.

`const_rate()` is the existing "do not throttle" bit. HEARTBEAT, HIGH_LATENCY2, PING, CAMERA_TRIGGER, CAMERA_IMAGE_CAPTURED, ADSB_VEHICLE, UTM_GLOBAL_POSITION, and the uAvionix streams set it. That is all-or-nothing, not a floor. HEARTBEAT is configured at 1 Hz and `SET_MESSAGE_INTERVAL` cannot change it.

`MavlinkRateLimiter` is a `dt >= interval` gate. Its only user is `MavlinkMissionManager` for 1 Hz `MISSION_CURRENT` / `MISSION_ITEM_REACHED`.

### What the receiver owns

`MavlinkReceiver` is a god object. It holds FTP, log download, mission, parameters, timesync, inbound STATUSTEXT, plus on the order of forty uORB publications, HIL sensor objects, RTCM assembly, and a large `handle_message()` switch.

After the switch, every inbound frame is also handed to mission, parameters (after boot), FTP, log handler, timesync, and `Mavlink::handle_message()` (signing and forwarding).

Every ≥10 ms, still on that thread and under one `lock_send()`:

- `MavlinkMissionManager::send()`
- `MavlinkParametersManager::send()` (not Iridium)
- `MavlinkFTP::send()`
- `MavlinkLogHandler::send()`

Those `send()` methods can do dataman I/O, `fread`, and `::read` of files while the TX thread is waiting for the same mutex to emit HEARTBEAT.

Handlers that send immediately on the RX thread include PING reply, TIMESYNC reply, PARAM_VALUE / PARAM_ERROR for a single request, MISSION_* protocol frames, FTP `_reply()`, EVENT for `REQUEST_EVENT`, and one-shot `request_message()`. TIMESYNC and PING are why the third TX class exists: they were nailed to the parse thread because the only other option was the sleepy stream walker.

### Services, in isolation

None of these read `_rate_mult`. "Burst" below is what one 10 ms tick can push. Each row is a candidate to redesign, not a spec to reimplement.

| Service | Thread | Burst / pacing | Reliability | Shares the rate budget? |
|---|---|---|---|---|
| Parameters | RX `send()` + immediate single-param | 3 frames/tick on serial, 20 on USB/UDP; 8 Hz in low-bandwidth mode | No PARAM_VALUE retry. GCS re-requests. Hash (`_HASH_CHECK`) can skip a list | UART space + `radio_status_critical`. Stream `_rate_mult` cut to 0.25 only in `LOW_BANDWIDTH` |
| Missions | RX | One item per GCS request; retry 250 ms | 5 s protocol timeout; resend REQUEST/COUNT; duplicate-request tolerance; CRC32 | No, except 1 Hz CURRENT |
| FTP | RX immediate reply + `send()` burst | Fill UART; pause every 35 KB (`burst_complete`) | Seq + last-reply cache for small ACKs. Burst itself is unacked | `get_free_tx_buf` only |
| Log download (`LOG_DATA`) | RX | Up to 256 KB per 10 ms tick, UART-gated | None. GCS re-requests ranges | `get_free_tx_buf` only |
| ULog streaming | TX `handle_update()` | Up to 70% of `_datarate`, 100 ms window | `LOGGING_DATA_ACKED`: 50 ms × 50 tries, then abort | Own cap; remainder left to streams. Does not check UART space |
| Timesync | RX reply; TX stream for requests | Immediate reply | None | No |
| Events | TX `update()`; RX `REQUEST_EVENT` | Drain buffer while UART has space | Sequence + GCS replay | UART space on TX path; RX request loop has no cap |
| Command sender | TX `COMMAND_LONG` stream (unlimited) | Drain `vehicle_command` while UART has space | 3 retries, 500 ms, list of 3, process-wide singleton | `get_size()` is 0, so invisible to `_rate_mult` |
| STATUSTEXT | TX stream at 20 Hz | Drain `mavlink_log` while UART has space | None. Stale >5 s dropped. Needs a GCS heartbeat | Scaled by `_rate_mult` (not `const_rate`) |
| Shell / serial passthrough | TX, one or more SERIAL_CONTROL per loop | UART-gated | None | No |
| Forwarding | RX `pass_message` → TX `resend_message` | **One** forwarded frame per TX loop | Two-message ring. Overflow is a perf count | No |

Parameter sending sets `_sending_parameters` for 2 s after the last PARAM_VALUE. That flag is the only explicit service-to-stream coupling, and it only fires in low-bandwidth mode. A Normal-mode telem1 instance at 1200 B/s does **not** slow ATTITUDE during a param list.

### Protocol-layer reliability today

These are on-wire behaviors QGC may depend on. Re-derive each service from the contract and from QGC; do not stack a TX-layer retry on top of a protocol retry without choosing a layer.

- Mission: 5 s give-up, 250 ms resend, duplicate `MISSION_REQUEST` accepted.
- FTP: sequence; if the GCS repeats the last request, resend the cached small reply. Burst download is GCS-paced with `burst_complete`.
- ULog: acked records wait; unacked `LOGGING_DATA` does not.
- Events: ring buffer with sequence; GCS `REQUEST_EVENT` replays.
- Commands: `MavlinkCommandSender`, 3 × 500 ms, per channel.
- Parameters: GCS-driven. PX4 does not retransmit PARAM_VALUE. QGC can skip the list via hash and, on recent work, download via MAVLink FTP.
- STATUSTEXT / forwarding / LOG_DATA / FTP burst: drop is loss.

A TX-layer reliable queue is a new contract. If both the queue and the protocol retry, you double-send on timeout.

### Compile-time cuts today

`src/modules/mavlink/Kconfig` offers four switches: `MODULES_MAVLINK` (the whole module), `MAVLINK_DIALECT`, `MAVLINK_UAVCAN_PARAMETERS`, `USER_MAVLINK`. Individual services, TX streams, and RX handlers are not selectable.

`CONSTRAINED_FLASH` is a board-level `#ifdef` that drops a blob of streams and some debug RX handlers. That is the opposite of the required grain: you cannot keep missions and drop DEBUG, or keep HEARTBEAT/ATTITUDE and drop onboard log download, without editing C++.

## Comparison

| Requirement | Today |
|---|---|
| Greenfield module | One accumulating tree. Receiver, services, streams, and UART write are one component. |
| Fast RX at the event level | `poll` is event-ish for bytes. Then the same thread runs a large handler switch and, every 10 ms, services that can block on filesystem and dataman. Parse holds `lock_send()` per byte. |
| Fast TX at the event level | Sleep, then poll every stream. A due ATTITUDE waits up to `_main_loop_delay`. A due HEARTBEAT waits on `lock_send()` if the receiver is in FTP/`fread`. |
| Mailboxes as the interface | None for MAVLink frames. Forwarding is a two-slot ring. `configure_stream_threadsafe()` is a busy-wait mailbox of one stream name. |
| Independent, redesigned services | Owned by `MavlinkReceiver` or `Mavlink`, constructed with `Mavlink &`, calling `get_channel()` / `lock_send()` / `get_free_tx_buf()` directly. Mission, ULog, and command-sender also have process-wide singletons. |
| TX respects `MAV_n_RATE` | Only variable-rate streams, via predicted interval scaling, with a 5% floor. Everything else bypasses. |
| Latency-sensitive / reliable / best-effort | No. Same `_buf`, same `write()`. Latency-sensitive replies were glued to the RX thread. `const_rate()` and a few UART-space checks are the only other distinctions. |
| Best-effort min rate | No. `const_rate()` never scales. Other streams scale uniformly down to 5% of configured Hz. |
| Opportunistic telemetry throttle during param/FTP/log | ULog reserves 70% of the predicted budget. Param list reserves 75% of streams **only** in low-bandwidth mode. FTP and log download reserve nothing and can fill the UART. |
| KConfig per service / stream / handler | Whole module, dialect, UAVCAN params, userspace. Streams drop as a `CONSTRAINED_FLASH` blob. |
| `wq:mavlink` | No work queue. FS runs on the RX thread under `lock_send()`. |
| User stream sets | `MAV_n_MODE` C++ tables, `mavlink stream` CLI, `SET_MESSAGE_INTERVAL` (not persistent). |
| Automated protocol tests | Flight-level `test/mavsdk_tests/`. No MAVLink-module suite. |

### RX is also a second TX path

This is the design defect that started the rewrite. Protocol responses are latency-sensitive, so they were nailed to the parse thread. Bulk `send()` was then nailed to the same 10 ms tick so a param list would make progress without a third thread. Parse, filesystem, and telemetry contend for one recursive mutex and one UART.

The mailbox split plus the latency-sensitive class is how TIMESYNC and PING leave the parse thread without waiting behind `fread` or a param list.

### Rate control is feed-forward interval scaling, not a token bucket

A token bucket meters **actual** bytes and allows a bounded burst. Today's `_rate_mult` meters **configured** stream Hz. A 20-PARAM_VALUE burst on USB, a 35 KB FTP chunk, or 256 KB of `LOG_DATA` does not debit the stream scheduler. The UART either absorbs it (`FIONSPACE` / blocking `write`) or `send_start()` drops whoever called next, which may be ATTITUDE or TIMESYNC.

Radio software throttle (`MAV_n_RADIO_CTL`) only lengthens stream intervals. It does not pause FTP.

### Best-effort drop happens in the wrong place

When the OS TX buffer is full, `send_start()` drops **this** packet. HEARTBEAT checks free space and skips. ATTITUDE generally does not. A reliable protocol frame that does not check free space is dropped the same way, and only the protocol retry (if any) recovers.

Reliable traffic needs a queue in front of `write()`, with send-complete (or UART-space) as the consumer event. Best-effort should still be allowed to drop at that queue, not inside `send_start()` after the C library has already assigned a sequence number. Latency-sensitive frames use the same queue with jump-the-line dequeue, and still debit tokens.

### `const_rate()` is not a minimum rate

HEARTBEAT at 1 Hz matches the example floor, but ATTITUDE is a variable stream: in Normal mode it is configured at 15 Hz and can be scaled to 0.75 Hz. There is no way for a stream to say "configured 15 Hz, never below 5 Hz."

Uniform `_rate_mult` also cannot prefer a latency-sensitive HEARTBEAT over DEBUG while still leaving ATTITUDE at its floor: every variable stream is multiplied by the same number.

## What the current tree is for

The existing module is evidence, not scaffolding.

- Use it to list intended features, default stream rates per mode, and on-wire sequences QGC already speaks.
- Use it to list defects the new module must not reproduce (this note).
- Do not transplant `MavlinkReceiver`, `_rate_mult`, `MavlinkRateLimiter`, process-wide singletons, or a service that does FS I/O under `lock_send()`.
- The MAVLink C library remains the codec if that is the cheapest correct choice; it must not remain the place that writes the UART, and its per-channel global `mavlink_status_t` is open to replacement.
- `RADIO_STATUS.txbuf` and UART `FIONSPACE` remain useful inputs to a token bucket. ULog's 70% reservation is evidence that a declared share can work; generalize it rather than special-casing ULog.

## Constraints a rewrite must preserve

- **GCS contract**, verified in QGC source for each service you implement, and asserted in MAVSDK tests against SITL. Changing on-wire behavior that QGC or MAVSDK depends on is a regression even if the firmware is cleaner.
- **Multiple instances.** USB, telem1, and companion Ethernet run at once. Forwarding, signing-key reload, mission `_transfer_in_progress`, ULog singleton, and `MavlinkCommandSender` already couple them — those couplings are defects to redesign, not a requirement to keep a singleton. A per-instance TX queue must not globally stall USB because telem1 is in a param list.
- **Link kinds.** SiK at 57600 / 1200 B/s, onboard 921600, USB, UDP, Iridium (`HIGH_LATENCY2`, TX often disabled). One architecture, different bucket rates and floors. KConfig may omit Iridium or gimbal services on boards that never use them.
- **HIL.** `HIL_ACTUATOR_CONTROLS` at 200 Hz on links with `_datarate > 5000`, when the HIL stream and handlers are compiled in.
- **Lockstep SITL.** `px4_usleep` follows sim time; `configure_stream_threadsafe()` already uses wall `usleep` for that reason. Event waits must not stall the lockstep clock.
- **Constrained flash/RAM.** Fine-grained KConfig replaces `CONSTRAINED_FLASH` blobs. Extra threads, extra queues of `mavlink_message_t` (payload up to 280 bytes plus header), and a worker per service are not free. Budget 6 hot threads plus one shared WQ at three instances, not 9.
- **Filesystem.** FTP, log download, and dataman cannot run on a high-priority parse thread. They also cannot hold the UART mutex.
- **Component metadata / param FTP.** Recent GCS work downloads autopilot parameters as a file. That is FTP bulk on the same instance as HEARTBEAT. It is the motivating case for opportunistic telemetry throttle, and it is already on the wire. Confirm the current QGC path in `ParameterManager.cc` before designing the param service.

## Questions a design must answer

These are not decided here. Traffic classes, greenfield, holistic services, QGC+MAVSDK contract, KConfig grain, thread names, `wq:mavlink`, and per-instance RX/TX are.

1. **Token units.** Bytes are the honest unit for `MAV_n_RATE`. Message counts are easier and wrong for FTP versus HEARTBEAT. Burst depth must be small enough that a 57600 baud radio is not given 256 KB. Latency-sensitive frames still debit the same bucket; decide whether they can run the bucket negative for one packet.
2. **Class membership.** TIMESYNC reply, PING reply, and COMMAND_ACK are latency-sensitive. HEARTBEAT might be latency-sensitive, best-effort-with-floor, or both. STATUSTEXT, events, COMMAND_LONG, forwarding, and ULog-acked records each need a call. Operators treat STATUSTEXT as reliable; today it is a 20 Hz best-effort stream that needs a GCS.
3. **Who defines floors.** Hard-coded in the stream, a parameter, the catalog in [`streams_metadata.md`](streams_metadata.md), or derived from `SET_MESSAGE_INTERVAL`. HEARTBEAT is special today (`SET_MESSAGE_INTERVAL` rejected); that special case is open to drop if QGC and MAVSDK do not depend on the rejection.
4. **Who retries.** TX-layer retransmit versus protocol retries. Pick one layer per message family, after reading how QGC and MAVSDK recover loss for that family.
5. **Opportunistic throttle policy.** During param/FTP, scale all variable streams toward their floors, park a declared set, or give reliable traffic a reserved token fraction (the ULog 70% pattern).
6. **Interaction with `RADIO_STATUS` and HW flow control.** Today they only affect `_rate_mult` / RTS/CTS. A token bucket should fold `txbuf` into refill or into "stop dequeuing best-effort." Latency-sensitive should still get through.
7. **`write()` completion as an event.** Serial is blocking today. Per-instance TX wants `POLLOUT` or a non-blocking write so *that* instance's TX thread can sleep on the fd instead of spinning or blocking the CPU. UDP `SO_SNDTIMEO` is already 10 ms to avoid an IOB stall; that timeout is a third form of "drop."
8. **Library state.** Split TX seq from RX parse state, keep a mutex, or stop using the C convenience send path. This is a prerequisite for actually being event-driven; the current comments already say the mutex is a stopgap.
9. **KConfig shape.** One symbol per stream/handler versus groups (for example "gimbal", "HIL", "debug"). Disabled-stream NAK behavior that QGC and MAVSDK already handle. Default `defconfig` for a full FMU versus a constrained board, so the full default is QGC-complete.
10. **How the new module ships.** Replace `MODULES_MAVLINK` in place, live beside it behind a new symbol, or board-by-board cutover. The source is greenfield either way.
11. **Gimbal / Iridium / uAvionix.** They already strip services and streams. Prefer KConfig plus a small mode table over a parallel module personality that grows its own worker.
12. **MAVSDK gaps.** Which protocol tests need a new proto/plugin versus `mavlink_direct` plus FTP. Streams catalog type is one such gap (`ComponentMetadata::MetadataType` has no streams value).

Stream catalog, profile file, factory snapshots, and Save/Reset are in [`streams_metadata.md`](streams_metadata.md), not here.

## Out of scope for this note

- Message definitions, dialects, or GCS UI (except as compatibility constraints).
- Whether param download should be FTP-only. Decide that in the param-service design, against `ParameterManager.cc`.
- DroneCAN parameter bridging (`CONFIG_MAVLINK_UAVCAN_PARAMETERS`); it rides the param service. Redesign it with that service, or KConfig it off.
- A PR plan, class list, or migration order.
- Stream catalog schema, profile file format, and GCS UI. Those live in [`streams_metadata.md`](streams_metadata.md).

## How to use this note

Write a design for a new module. Start from the decided constraints and the remaining questions. Use this note for "how the old module works" and "what not to copy" instead of re-deriving it from `mavlink_receiver.cpp`. For each service and each on-wire sequence, open the matching QGC code before locking behavior, and add a MAVSDK SITL test that an agent can run headless. Threading is `mavlink_{i}_rx` / `mavlink_{i}_tx` plus `wq:mavlink`. Stream sets are [`streams_metadata.md`](streams_metadata.md). If a design claim contradicts a statement here, verify against the PX4 tree; this snapshot is `436cc71c89` and the old module moves.
