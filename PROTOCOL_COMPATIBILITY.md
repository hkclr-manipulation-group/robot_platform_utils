# UDP protocol compatibility

The current wire format is protocol V1. Its existing byte layout is frozen.

Handshake negotiation is an optional authenticated suffix. New clients first
send the suffix (`major=1`, `minor=0`, append-only capability); if a legacy
server does not respond, they retry with the original byte-identical V1
handshake. A new server only appends its handshake response suffix when the
request contained one, so old clients continue receiving their original
response layout.

## V1 evolution rules

1. Never insert, remove, reorder, or resize an existing wire field.
2. Never change an enum's underlying type or reuse an existing enum value.
3. New optional data must be appended after the complete existing message.
   Fields that logically belong to an arm or gripper still go in the message
   trailer; do not insert them into the repeated arm/gripper blocks.
4. A missing trailing field uses a documented default value.
5. Readers ignore authenticated trailing bytes they do not understand.
6. Writers must include the complete trailer in the existing message HMAC.
7. A required or incompatible change creates a new major protocol codec; it
   must not silently change the V1 layout.

Use `cpp/include/protocol_extensions.h` for new optional scalar or
trivially-copyable fields. Each trailer entry has an `EXT1` marker, a stable
`uint16_t` field ID, a `uint16_t` value length, and the value bytes. Field IDs
are permanent and must not be reused with another meaning or type.

This gives old readers built from the compatibility release forward
compatibility with append-only messages, and gives new readers backward
compatibility with messages that do not contain newer trailing fields.

## Release policy

Keep explicit V1 encode/decode tests and test both directions in CI:

- current SDK against current rt_control;
- previous released SDK against current rt_control;
- current SDK against previous released rt_control;
- authenticated unknown trailing fields are accepted;
- truncated and unauthenticated fields are rejected.

When a V2 envelope is introduced, rt_control must keep the V1 decoder during
the migration period and select the response codec from the version negotiated
during handshake.

## FlatBuffers migration (phase 1)

Phase 1 keeps the existing UDP + HMAC envelope and only migrates the
`SdkHandshakeReq` / `SdkHandshakeRes` payload when both sides advertise the
`kFlatBuffersPayload` capability.

Wire layout for FlatBuffers handshake messages:

1. Fixed prefix: `magic`, `MessageType`, `client_id` / `request_client_id`
2. Marker: `0x42454642` (`"FBFB"`)
3. FlatBuffer root table (`schema/handshake.fbs`)
4. Existing trailing `security_hmac`

Legacy V1 handshake remains byte-identical and is still attempted after a
short FlatBuffers probe or when the peer does not understand the FB payload.

New optional fields should be added to the `.fbs` schema first. Command,
config, and state messages remain on the frozen V1 codec until a later phase.
