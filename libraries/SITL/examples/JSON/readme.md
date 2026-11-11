# JSON Simulation Format

The JSON SITL backend allows software to easily interface with ArduPilot using a standard JSON interface.

To launch the JSON backend run SITL with ```--model JSON:127.0.0.1``` where 127.0.0.1 is replaced with the IP the physics backend is running at. For example: ```sim_vehicle.py -v ArduCopter -f octaquad --model JSON:127.0.0.1 --map --console```

Connection to SITL is made via a UDP link. The physics backend should listen for incoming messages on port 9002 it should then reply to the IP and port the messages were received from. This removes the need to configure a target
IP and port for SITL in the physics backend. SITL will send a output message every 10 seconds allowing the physics backend to auto detect.

## SITL output

Data is output from SITL in a binary format:

```text
    uint16 magic = 18458
    uint16 frame_rate
    uint32 frame_count
    uint16 pwm[16]
```

The magic value is a constant of 18458, this is used to confirm the packet is from ArduPilot and is used for protocol versioning.

The number of output channels may be increased to 32 by setting the parameter
SERVO_32_ENABLE = 1. The SITL output packet is then

```text
    uint16 magic = 29569
    uint16 frame_rate
    uint32 frame_count
    uint16 pwm[32]
```

and uses a magic value 29569.

The frame rate represents the time step the simulation should take, this can be changed with the SIM_RATE_HZ ArduPilot parameter. The physics backend is free to ignore this value, a maximum time step size would typically be set. The SIM_RATE_HZ should value be kept above the vehicle loop rate, by default this 400hz on copter and quadplanes and 50 hz on plane and rover.

The frame_count will increment for each output frame sent by ArduPilot, this count can be used to detect lost or duplicate frames. This count will be reset when SITL is re-started allowing the physics backend to reset the vehicle. If not input data is received after 10 seconds ArduPilot will re-send the output frame without incrementing the counter. This allows the physics model to be restarted and re-connect. Note that this may fill up the input buffer of the physics backend after some time.

PWM is a array of 16 (or 32) servo values in micro seconds, typically in the 1000 to 2000 range as set by the servo output functions.

## SITL input

Data is received from the physics backend in a plain text JSON format. The data must contain the following fields:

```text
    timestamp (s) physics time
    imu:
        gyro(roll, pitch, yaw) (radians/sec) body frame
        accel_body(x, y, z) (m/s^2) body frame
    position(north, east, down) (m) earth frame
    velocity(north, east, down) (m/s) earth frame
```

It is possible to send the attitude in a euler format using ```attitude``` or as a quaternion with ```quaternion```, one of these fields must be received. If both are received the quaternion attitude will be used.

```text
    attitude(roll, pitch yaw) (radians)
    quaternion(q1, q2, q3, q4)
```

This is a example input frame, it should be terminated with a newline ("\n"). Over UDP the end of a datagram also ends a packet, so the newline may be left out there:

```json
{"timestamp":2500,"imu":{"gyro":[0,0,0],"accel_body":[0,0,0]},"position":[0,0,0],"attitude":[0,0,0],"velocity":[0,0,0]}
```

The order of fields is not important, but a field must not be given twice. A packet must be on a single
line: a newline inside a packet ends it, and the packet is rejected.
Note that the timestamp is the absolute physics time, not the timestep.

It is possible to send optional fields to provide data for additional sensors, in most cases this will require setting the relevant sensor type param to the SITL driver.

rangefinder distances corresponding to driver instances:

```text
    rng_1 (m)
    rng_2 (m)
    rng_3 (m)
    rng_4 (m)
    rng_5 (m)
    rng_6 (m)
```

A rangefinder with no reading, out of range for example, can send infinity or NaN; SITL then reports
no data for it. JSON has no way to write these, so the common ones are accepted for any number:
`null` (NaN, as written by most JSON encoders including rosbridge), `NaN`, `Infinity`, `-Infinity`
(Python's `json` module) and `nan`, `-nan`, `inf`, `-inf` (C's `printf`). A packet is rejected if
infinity or NaN appears in the vehicle state: timestamp, latitude, longitude, altitude, imu,
position, attitude, quaternion or velocity.

## Apparent wind

```text
    windvane:
        direction (radians) clockwise relative to the front, i.e. 0 = head to wind
        speed (m/s)
```

for example:

```json
 "windvane":{"direction":0,"speed":0}
```

## Wind Vector

3D wind can be provided in m/s NED frame, for example:

```json
"velocity_wind":[3.2,0.0,-0.7]
```

## Airspeed

```text
    airspeed (m/s)
```

## RC Input

The controller can optionally provide R/C input data, up to 12
channels.

```json
"rc":{"rc_1":1500,"rc_2":1500,"rc_3":1000,"rc_4":1500,"rc_5":1000,"rc_6":1000,"rc_7":1000,"rc_8":1500,"rc_9":1500,"rc_10":1500}
```

## Battery

The controller can provide battery voltage and current. Voltage in
Volts, current in Amps:

```json
"battery":{"voltage":50.39,"current":64.01}
```

## Compact packets for slow links

Over a local UDP socket packet size does not matter, but over a serial
or radio link a 400Hz stream of full precision JSON does not fit. A
packet with the mandatory fields written by Python's `json.dumps` is
about 400 bytes; the options below bring it to about 150 bytes while
keeping it plain JSON. They can be used together or separately, and
packets using them are still accepted over UDP.

### Precision

Send each value only to the precision that matters. These rounding
errors are below the resolution of typical real sensors:

| Field | Decimals | Max error |
| --- | --- | --- |
| timestamp (s) | 6 | 0.5 us |
| gyro (rad/s), attitude (rad) | 4 | 5e-5 |
| quaternion | 5 | 5e-6 |
| accel_body (m/s^2), position (m), velocity (m/s), rangefinders (m) | 3 | 0.5e-3 |
| latitude, longitude (deg) | 7 | about 1cm |
| altitude, airspeed, wind, battery | 2 | 5e-3 |

In Python, `json.dumps(round(x, 4))` writes the shortest form of the
rounded value, e.g. `0.0123`.

Rounding also makes packets cheaper to parse on a microcontroller.
Numbers with at most 7 significant digits (15 for latitude, longitude,
altitude, position and timestamp) are converted without calling strtof
or strtod; on a Cortex-M4 that is about 300 instructions per number,
against about 3000 for a 17 digit number from `json.dumps`.

### Short keys

Every field can also be sent under a short key in the root object.
If both the full name and the short key are present the full name is
used.

| Short key | Field | Short key | Field |
| --- | --- | --- | --- |
| `t` | timestamp | `r1` .. `r6` | rng_1 .. rng_6 |
| `g` | imu: gyro | `vw` | velocity_wind |
| `a` | imu: accel_body | `wd` | windvane: direction |
| `p` | position | `ws` | windvane: speed |
| `v` | velocity | `as` | airspeed |
| `e` | attitude | `c1` .. `c12` | rc: rc_1 .. rc_12 |
| `q` | quaternion | `bv` | battery: voltage |
| `lat`, `lon`, `alt` | latitude, longitude, altitude | `bc` | battery: current |

For example, 148 bytes instead of about 400:

```json
{"t":12.0025,"g":[0.0123,-0.2311,0.0457],"a":[0.312,-0.088,-9.803],"p":[123.456,-45.678,-10.5],"v":[1.234,-0.567,0.012],"e":[0.0123,-0.0456,1.5708]}
```

### Sending slow fields less often

The rangefinders, wind, windvane, airspeed, RC, battery and the
`no_time_sync` / `no_lockstep` flags keep their last value for 0.5s of
physics time after they were last received. They can be sent at a
lower rate than the vehicle state, for example every 8th packet at
400Hz. The vehicle state fields (timestamp, imu, position, velocity,
attitude, quaternion, latitude, longitude, altitude) are never held
and must be in every packet that needs them.

### Checksum

A UDP datagram has its own checksum but a serial link does not, and a
single flipped bit in a digit gives a different number that is still
valid JSON. A packet may end with `*` and four hex digits after the
closing brace:

```text
{"t":12.0025,...}*1A2B
```

The value is the CRC-16/CCITT-FALSE (polynomial 0x1021, initial value
0xFFFF) of every byte before the `*`. Packets with a wrong checksum
are dropped. In Python:

```python
import binascii, json

def encode(fields):
    body = json.dumps(fields, separators=(',', ':')).encode('ascii')
    return b'%s*%04X\n' % (body, binascii.crc_hqx(body, 0xFFFF))
```

## Debugging

When first connecting you will see a message reporting what fields were successfully received. If any of the mandatory fields are missing SITL rejects the packet and waits for a complete one, however it will run without the optional fields. This message can be used to double check SITL is receiving everything being sent by the physics backend.

For example:

```text
JSON received:
        timestamp
        gyro
        accel_body
        position
        attitude
        velocity
        rng_1
```

### Rejected packets

SITL uses a packet only if all of it is valid; nothing from a rejected packet is used. It then sends the
same servo frame again and waits for the next packet, so simulated time stands still until a valid one
arrives. Rejections are reported at most once a second, with a count of all rejected packets so far:

```text
JSON: rejected sensor packet: duplicate key, 3 rejected
```

| Message | Cause |
| ------- | ----- |
| `rejected sensor packet: <reason>` | The packet is not valid JSON or does not match the fields above, see below |
| `sensor packet missing: <fields>` | A mandatory field is missing |
| `sensor packet with infinity or NaN in: <fields>` | The vehicle state holds infinity or NaN; only sensors may send them |
| `sensor datagram too large` | The datagram is over 4kB |

Common reasons for `rejected sensor packet`:

| Reason | Cause |
| ------ | ----- |
| `wrong type of value for field` | e.g. `true` where a number is expected, or `null` for a boolean |
| `string where a field value was expected` | e.g. `"5"` where a number is expected |
| `array field has the wrong number of elements` | e.g. 2 values for `velocity` |
| `array where a single value was expected`, `number where an array was expected` | |
| `duplicate key` | The same field given twice, or a field given both by its full name and inside a repeated object |
| `line ended inside the packet` | A newline in the middle of the packet: packets must be on one line, e.g. not pretty printed |
| `invalid number`, `leading zero in number` | e.g. `1.`, `1e`, `01`; write `1.0`, `1e0`, `1` |
| `invalid value`, `invalid literal` | e.g. `.5`, `+1`, `True`, `'text'`, or a spelling of infinity or NaN not listed above such as `Inf` |
| `CRC mismatch`, `invalid CRC` | The CRC suffix does not match the packet, or is not 4 hex digits |
| `unexpected data after the packet` | Anything but whitespace or a CRC after the closing `}` |
| `packet too long` | Over 4096 bytes |
| `nesting too deep` | Objects and arrays nested more than 8 deep |
