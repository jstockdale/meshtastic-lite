# meshtastic-lite

A clean-room, header-only C/C++ implementation of the Meshtastic LoRa mesh protocol. Designed for embedded systems that want to interoperate with the Meshtastic network without pulling in the full Meshtastic firmware, Arduino, RadioLib, Nanopb, or FreeRTOS dependencies.

## What this is

A standalone protocol library that handles everything between raw LoRa bytes and decoded Meshtastic messages:

- **Packet parsing** — 16-byte Meshtastic header (to, from, id, flags, channel, hop_limit, hop_start)
- **AES-256-CTR encryption/decryption** — channel-based PSK with proper IV construction (`packetId || fromNode || counter`)
- **PKI direct messages** — x25519 key exchange + AES-256-CCM for authenticated DMs (v2.5+ protocol)
- **Channel management** — PSK expansion (1-byte shortcuts, 16/32-byte keys), channel hash computation, multi-channel decrypt
- **Protobuf decoding** — TEXT_MESSAGE, POSITION, NODEINFO, TELEMETRY, ROUTING (no Nanopb dependency)
- **Radio configuration** — frequency calculation with DJB2 hash, all modem presets (LongFast through ShortTurbo), all regions (US/EU/CN/JP/ANZ/KR/TW/RU/IN/NZ/TH/UA/MY/SG/PH)
- **CSMA/CA** — contention window sizing and slot time calculation per the Meshtastic MAC protocol
- **TX frame building** — encrypted packet construction with proper header, ready to send over any LoRa radio

## What this is not

- Not a radio driver — you provide the SPI/LoRa interface
- Not a mesh router — no packet forwarding, flooding, or NodeDB management
- Not a complete Meshtastic node — no MQTT, no BLE, no phone app interface

## Usage

### Header-only — just include it

```cpp
#define MESH_CRYPTO_USE_MBEDTLS 1  // or 0 for OpenSSL
#include "meshtastic.h"
```

That single include pulls in all sub-headers. The entire library is ~2300 lines across 8 header files.

### Initialize a session

```cpp
MeshSession session;
session.init(REGION_US, MODEM_LONG_FAST, ROLE_CLIENT, my_node_num);
session.channels.addDefaultChannel();  // default PSK on channel 0

// Get radio parameters
MeshRadioConfig rc = session.radioConfig();
// rc.frequency_mhz = 906.875  (US/LongFast default)
// rc.bandwidth_khz = 250.0
// rc.spreading_factor = 11
// rc.coding_rate = 5 (4/5)
// rc.sync_word = 0x2B
// rc.tx_power_dbm = 30
// rc.preamble_length = 16
```

### Receive and decode

```cpp
// After your radio receives raw bytes...
uint8_t raw[256];
uint8_t len = radio_receive(raw);
float rssi = radio_get_rssi();
float snr = radio_get_snr();

MeshRxResult result;
if (session.processRx(raw, len, rssi, snr, &result)) {
    printf("From: !%08x  Channel: %d\n", result.packet.from, result.channel_idx);

    switch (result.data.portnum) {
        case PORT_TEXT_MESSAGE:
            printf("Text: %.*s\n", result.data.payload_len, result.data.payload);
            break;
        case PORT_POSITION: {
            MeshPosition pos;
            meshDecodePosition(result.data.payload, result.data.payload_len, &pos);
            printf("Pos: %.6f, %.6f alt:%d\n", pos.latitude(), pos.longitude(), pos.altitude);
            break;
        }
        case PORT_NODEINFO: {
            MeshUser user;
            meshDecodeUser(result.data.payload, result.data.payload_len, &user);
            printf("Node: %s (%s)\n", user.long_name, user.short_name);
            break;
        }
        case PORT_TELEMETRY: {
            MeshTelemetry tel;
            meshDecodeTelemetry(result.data.payload, result.data.payload_len, &tel);
            if (tel.has_device_metrics)
                printf("Battery: %d%%  Voltage: %.1fV\n",
                       tel.device_metrics.battery_level,
                       tel.device_metrics.voltage);
            break;
        }
    }
}
```

### Transmit

```cpp
// Build an encrypted text message
uint8_t frame[256];
size_t frame_len = session.buildTextTx(
    0,                    // channel index
    MESH_ADDR_BROADCAST,  // destination (0xFFFFFFFF = broadcast)
    "Hello from meshtastic-lite!",
    false,                // want_ack
    frame);

// CSMA/CA backoff
uint32_t delay_ms = meshTxDelayMs(session.preset);
delay(delay_ms);

// Send via your radio
radio_transmit(frame, frame_len);
```

### Frequency slot override

By default, the frequency slot is computed from the DJB2 hash of the channel name. You can override this for private channels or specific community configurations:

```cpp
// Use explicit frequency slot 45 (e.g., for a local community)
session.init(REGION_US, MODEM_MEDIUM_FAST, ROLE_CLIENT, my_node_num, 45);

// Or use the default hash-based slot (-1)
session.init(REGION_US, MODEM_MEDIUM_FAST, ROLE_CLIENT, my_node_num);
```

### Custom PSK

```cpp
// Add a channel with a custom 256-bit PSK
uint8_t my_psk[32] = { /* your key */ };
session.channels.addChannel("MyChannel", my_psk, 32);
```

## Crypto backends

The library supports two crypto backends, selected at compile time:

| Define | Backend | Use case |
|---|---|---|
| `MESH_CRYPTO_USE_MBEDTLS 1` | mbedtls | ESP32, embedded (hardware AES acceleration) |
| `MESH_CRYPTO_USE_MBEDTLS 0` | OpenSSL | Linux, macOS, testing |

## Files

```
src/
├── meshtastic.h          — Single include (pulls in all sub-headers)
├── meshtastic_config.h   — Region definitions, modem presets, constants
├── meshtastic_packet.h   — Packet header parsing, frame construction
├── meshtastic_crypto.h   — AES-256-CTR encryption/decryption
├── meshtastic_pki.h      — x25519 + AES-CCM for PKI direct messages
├── meshtastic_channel.h  — Channel table, PSK expansion, hash computation
├── meshtastic_pb.h       — Protobuf decoder (TEXT, POSITION, NODEINFO, TELEMETRY)
└── meshtastic_radio.h    — Frequency calc, CSMA/CA, session management, TX builder

examples/
├── test_meshtastic.cpp      — Protocol verification tests
├── meshtastic_task.cpp      — ESP32-P4 FreeRTOS integration (RX/TX tasks)
└── meshtastic_task.h        — C API for the ESP32 integration
```

## Protocol verification

The implementation has been verified against the Meshtastic firmware source and documentation:

| Feature | Reference | Status |
|---|---|---|
| AES-CTR nonce layout | `[packetId(8B LE)][fromNode(4B LE)][counter(4B LE)]` | ✓ Matches CryptoEngine.cpp |
| Packet header | 16 bytes packed: to, from, id, flags, channel, hop, relay | ✓ Matches RadioInterface.h |
| Channel hash | `XOR(name_bytes) ^ XOR(expanded_key_bytes)` | ✓ Matches Channels.cpp |
| PSK expansion | Single-byte index → well-known key, 16/32-byte direct | ✓ Matches Channels.cpp |
| Default PSK | `0xd4f1bb3a20290759f0bcffabcf4e6901` ("AQ==" expanded) | ✓ Matches Channels.h |
| DJB2 frequency hash | `hash = ((hash << 5) + hash) + c` | ✓ Matches RadioInterface.cpp |
| Frequency formula | `freqStart + (bw/2000) + (slot × bw/1000)` | ✓ Matches RadioInterface.cpp |
| Sync word | 0x2B, preamble 16 symbols | ✓ Matches RadioLibInterface.h |
| CSMA/CA | `random(0, 2^CWsize) × slotTime` | ✓ Matches RadioInterface.cpp |
| PKI key exchange | x25519 ECDH → AES-256-CCM with 4-byte random nonce | ✓ Matches CryptoEngine.cpp |

## Tested with

- **ESP32-P4** (RISC-V, 360 MHz) — SX1262 via SPI, hardware AES via mbedtls
- **Meshtastic firmware 2.5+** — interop confirmed on MediumFast/US with default and custom channels

The library was developed for the [ADS-B Scope](https://github.com/offx1/adsb-scope) project's "Meshy" feature, which runs a Meshtastic receiver alongside an ADS-B receiver on the LilyGo T-Display-P4.

## ESP32 integration example

The `examples/` directory includes a complete FreeRTOS integration (`meshtastic_task.cpp/h`) that provides:

- Two FreeRTOS tasks: RX (50ms poll) and TX (queue-driven with CSMA/CA)
- 1000-message PSRAM ring buffer with packet ID deduplication
- 512-node tracking table with LRU eviction
- C API: `meshy_start()`, `meshy_stop()`, `meshy_send_text()`, `meshy_recv()`, `meshy_get_nodes()`
- Hardware AES via mbedtls (`MESH_CRYPTO_USE_MBEDTLS=1`)

This integration is designed for the LilyGo T-Display-P4's `Cpp_Bus_Driver` SX1262 API but can be adapted to any SX126x driver.

## Known limitations

- **No mesh routing** — receives and transmits, but doesn't relay packets for other nodes
- **No NodeDB persistence** — node table is in RAM only, lost on reboot
- **Channel encryption only** — no message authentication (inherent to AES-CTR, not a library limitation)
- **PKI requires v2.5+** — DM encryption only works with nodes running Meshtastic 2.5 or later
- **No admin messages** — session-based admin RPC is not implemented

## License

BSD 3-Clause. See individual file headers.

This is a clean-room implementation based on publicly available Meshtastic documentation and protocol specifications. It does not contain any code from the Meshtastic firmware repository.

## Acknowledgments

- [Meshtastic](https://meshtastic.org/) — the protocol specification, documentation, and the community that makes mesh networking accessible
- [Off by One](https://offx1.com) — development and testing
