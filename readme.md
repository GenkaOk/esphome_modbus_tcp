# ESPHome Modbus TCP

An external component for ESPHome that talks Modbus TCP over plain sockets — no external
serial bridge, no `uart` proxy. It implements the same model as the built-in
[`modbus_controller`](https://esphome.io/components/modbus_controller.html): a bus object
(`modbustcp`) plus one or more polling controllers (`modbustcp_controller`) that expose
`sensor`, `binary_sensor`, `switch`, `number` and `text_sensor` entities.

- Target: ESP32, Arduino or ESP-IDF framework
- Reference builds: ESPHome 2026.6.5, Arduino and ESP-IDF

## Installation

```yaml
external_components:
  - source: github://creepystefan/esphome_modbus_tcp
    components: [modbustcp, modbustcp_controller]
```

`refresh` defaults to `1d`; add `refresh: 0s` while you are iterating so ESPHome re-pulls the
component on every compile.

> **`host` must be a literal IPv4 address.** Host names (`my-gateway.local`, DNS names) are
> rejected at config validation — Modbus gateways are addressed by IP here on purpose.

## Bus: `modbustcp`

```yaml
modbustcp:
  - id: modbus_tcp_id       # your id
    host: 192.168.178.46    # required, IPv4 literal
    port: 502               # optional, 502 is the default
    send_wait_time: 250ms   # optional, 250ms is the default
```

| Option           | Default  | Meaning                                                                 |
| ---------------- | -------- | ----------------------------------------------------------------------- |
| `host`           | —        | IPv4 address of the Modbus TCP server (required)                         |
| `port`           | `502`    | TCP port                                                                 |
| `send_wait_time` | `250ms`  | How long a command waits for its response before the next one is queued |

Several buses may be configured (`MULTI_CONF`), and several controllers may share one bus.

## Controller: `modbustcp_controller`

One controller = one Modbus **unit id** (slave address). Put one controller per device.

```yaml
modbustcp_controller:
  - id: modbus_tcp_controller_id
    modbustcp_id: modbus_tcp_id     # required
    address: 10                     # unit id, 0x01 by default
    update_interval: 5s             # polling interval, 60s by default
```

| Option                     | Default | Meaning                                                                                   |
| -------------------------- | ------- | ----------------------------------------------------------------------------------------- |
| `modbustcp_id`             | —       | Bus id (required)                                                                          |
| `address`                  | `0x01`  | Modbus unit id                                                                             |
| `update_interval`          | `60s`   | Polling interval                                                                           |
| `command_throttle`         | `0ms`   | Minimum gap between two commands on this controller                                        |
| `max_cmd_retries`          | `4`     | Sends before a command is dropped and the device is marked offline                         |
| `offline_skip_updates`     | `0`     | While offline, skip this many polling cycles per range to stop flooding the channel        |
| `allow_duplicate_commands` | `false` | Queue identical commands instead of merging them                                           |

Automations:

```yaml
modbustcp_controller:
  - id: modbus_tcp_controller_id
    modbustcp_id: modbus_tcp_id
    address: 10
    on_command_sent:
      - logger.log: { format: "sent FC 0x%X to register 0x%X", args: [function_code, address] }
    on_online:
      - logger.log: "device back online"
    on_offline:
      - logger.log: "device offline"
```

`on_command_sent`, `on_online` and `on_offline` all receive `function_code` and `address`
(both `int`).

## Entities

All entity platforms share this base:

| Option           | Default    | Meaning                                                                        |
| ---------------- | ---------- | ------------------------------------------------------------------------------ |
| `address`        | —          | Register address (required unless `custom_command` is used)                     |
| `register_type`  | see below  | `read` (input registers, FC 04), `holding` (FC 03), `coil` (FC 01/05), `discrete_input` (FC 02) |
| `value_type`     | `U_WORD`   | `RAW`, `U_WORD`, `S_WORD`, `U_DWORD`, `S_DWORD`, `U_QWORD`, `S_QWORD`, `FP32` and their `_R` (word-swapped) variants |
| `register_count` | from `value_type` | Number of 16-bit registers in the range                                  |
| `offset` / `byte_offset` | `0` | Skip bytes inside the response (mutually exclusive)                          |
| `bitmask`        | `0xFFFFFFFF` | Mask off bits (and shift down) — this is how a `binary_sensor` picks one bit |
| `skip_updates`   | `0`        | Poll this register only every N-th cycle                                        |
| `force_new_range`| `false`    | Never merge this entity into a neighbouring read range                          |
| `response_size`  | `0`        | Force the response size in bytes (`2` for `text_sensor`)                        |
| `lambda`         | —          | `(item, x, data) -> optional<float>` (`optional<std::string>` for text sensors) to post-process the value |

`register_type` is required for `switch`, `binary_sensor` and `text_sensor`; `number` defaults
to `holding`.

### Sensor

```yaml
sensor:
  - platform: modbustcp_controller
    modbustcp_controller_id: modbus_tcp_controller_id
    name: YEAR
    address: 31
    register_type: read
    value_type: U_WORD
```

### Binary sensor

```yaml
binary_sensor:
  - platform: modbustcp_controller
    modbustcp_controller_id: modbus_tcp_controller_id
    name: "Fault"
    address: 30
    register_type: read
    bitmask: 0x04          # bit 2 of the register
```

### Switch

```yaml
switch:
  - platform: modbustcp_controller
    modbustcp_controller_id: modbus_tcp_controller_id
    name: "Enable load test mode"
    register_type: coil
    address: 745
    bitmask: 1
    # use_write_multiple: true  # for holding registers, FC 16 instead of FC 06
```

### Number

```yaml
number:
  - platform: modbustcp_controller
    modbustcp_controller_id: modbus_tcp_controller_id
    name: "Target"
    address: 48
    register_type: holding    # default
    value_type: U_WORD
    min_value: 0
    max_value: 1000
    step: 1
```

### Text sensor

```yaml
text_sensor:
  - platform: modbustcp_controller
    modbustcp_controller_id: modbus_tcp_controller_id
    name: "Device model"
    address: 300
    register_type: read
    register_count: 8
    response_size: 16
    raw_encode: ANSI         # NONE, HEXBYTES, COMMA or ANSI
```

## Connection handling

- The TCP connection is opened from `loop()` and re-opened **at most once every 5 seconds**.
  The backoff starts at the moment the connection is dropped, so a server that accepts and
  immediately closes (for example a gateway still holding a stale connection) cannot turn
  into a connect/disconnect storm.
- No connection attempt is made while the network is down — the component waits until WiFi
  or Ethernet reports an IP.
- On every disconnect the receive buffer and the pending-response marker are cleared, so a
  half-received frame from the dead connection cannot be glued onto the first response of the
  new one.
- Responses are matched against the transaction id of the last request. Late or foreign
  responses are dropped, not decoded.

## Troubleshooting

| Log message | Meaning |
| ----------- | ------- |
| `Disconnected, will retry...` / `Reconnecting...` | Link lost, next attempt follows 5 s later |
| `Cannot send, not connected` | A poll fired while the socket was down — sensor values stay unchanged, the device is marked offline after `max_cmd_retries` |
| `Ignoring Modbus response with unexpected transaction id 0x....` | Late or multiplexed response, dropped by design |
| `Implausible Modbus TCP frame length ... discarding N buffered bytes` | Stream lost framing (usually right after a reconnect), buffer reset |
| `Modbus response received but no command is queued, ignoring` | Stale frame after a reconnect, ignored |
| `Modbus device=N set offline` | No response after `max_cmd_retries` attempts; raise `offline_skip_updates` if the device is permanently absent |

If the ESP cannot get on WiFi at all after a reboot, check `reset_reason` and the WiFi
disconnect reason in the log first — a reconnect storm used to be one contributor, but it is
not the only possible cause.

## Known limitations

- One Modbus unit id per controller; one TCP connection per bus.
- `host` accepts IPv4 literals only.
- No TLS.
- Units are always addressed as unit id `address`; broadcast (unit id 0) is not supported.
- **Server mode is not implemented yet.** `server_registers` exists in the controller schema,
  but the TCP transport never dispatches incoming requests to it, and config validation for
  that key currently fails with `NameError: name 'modbus' is not defined`. Do not use it.
- **`custom_command` / `register_type: custom` is not usable yet.** Raw commands go through
  `send_raw()`, which does not record a transaction id, so the response is rejected by the
  transaction-id check and the command times out.

## Alternative: upstream `modbus` over a serial-over-TCP bridge

If you prefer ESPHome's built-in `modbus`/`modbus_controller` and only need the wire to be
TCP, use a third-party `tcp_uart` bridge component that presents the remote serial link as a
UART:

```yaml
external_components:
  - source: github://Bascht74/esphome-tcp-uart
    components: [tcp_uart]

tcp_uart:
  - id: remote_serial
    host: "192.168.178.46"
    port: 502
    protocol: modbus

modbus:
  - id: modbus_bus
    uart_id: remote_serial

modbus_controller:
  - id: modbus_device
    modbus_id: modbus_bus
    address: 10
```

This is a different project (`Bascht74/esphome-tcp-uart`), not part of this repository.

## Links

- [Public Modbus TCP test server](https://modbus.pult.online/en.html)
- [Modbus TCP protocol description](https://ipc2u.de/artikel/wissenswertes/detaillierte-beschreibung-des-modbus-tcp-protokolls-mit-befehlsbeispielen/)
- [Modbus TCP master component (Gucioo)](https://github.com/Gucioo/esphome_modbus_tcp_master)

## License

Apache-2.0 — see [LICENSE](LICENSE).
