-- Mighty serial packet streamer.
--
-- Packet format:
--   0xDF | packet_type | payload_length | payload | checksum
--
-- packet_type:
--   0x01 = pose
--   0x02 = state
--
-- payload_length is one byte, so payloads must be <= 255 bytes.
-- checksum is the 8-bit sum of packet_type, payload_length, and payload bytes.
--
-- Binary payloads are little-endian.
--
-- Pose payload, type 0x01, 24 bytes:
--   uint64 timestamp_ns
--   uint8  valid
--   uint8  confidence * 255
--   int16  position x, meters * 1000
--   int16  position y, meters * 1000
--   int16  position z, meters * 1000
--   int16  quaternion x * 32767
--   int16  quaternion y * 32767
--   int16  quaternion z * 32767
--   int16  quaternion w * 32767
--
-- State payload, type 0x02, 10 bytes:
--   uint64 timestamp_ns
--   uint8  vio_state enum: 0 unknown, 1 off, 2 initializing,
--                         3 tracking, 4 degraded, 5 lost
--   uint8  flags: bit0 recording, bit1 preview_mode,
--                 bit2 initializing, bit3 arm_connected

local START_BYTE = 0xDF
local TYPE_POSE = 0x01
local TYPE_STATE = 0x02

local BAUD = 115200

local CONFIDENCE_SCALE = 255
local POSITION_SCALE = 1000
local QUATERNION_SCALE = 32767

local INT16_MIN = -32768
local INT16_MAX = 32767

-- Change these to control output frequency.
-- frequency_hz = 1000 / period_ms
local POSE_PERIOD_MS = 20    -- 50 Hz
local STATE_PERIOD_MS = 200  -- 5 Hz

local pose_elapsed = 0
local state_elapsed = 0

local blink = {
  idle = { { true, 100 }, { false, 900 } },
  recording = { { true, 120 }, { false, 120 }, { true, 120 }, { false, 120 }, { true, 120 }, { false, 400 } },
  starting = { { true, 100 }, { false, 100 } },
  tracking = { { true, 1000 } },
  degraded = { { true, 50 }, { false, 150 }, { true, 50 }, { false, 750 } },
  lost = { { true, 200 }, { false, 200 } },
}

local vio_blink = {
  off = "idle",
  tracking = "tracking",
  degraded = "degraded",
  lost = "lost",
}

local mode = "starting"
local step = 1
local led_elapsed = 0
local recording_started_by_button = false

local function value_or_zero(value)
  if value == nil then
    return 0
  end
  return value
end

local function round(value)
  if value >= 0 then
    return math.floor(value + 0.5)
  end
  return -math.floor(-value + 0.5)
end

local function clamp(value, min_value, max_value)
  if value < min_value then
    return min_value
  end
  if value > max_value then
    return max_value
  end
  return value
end

local function pack_u8(value)
  return string.char(clamp(math.floor(value), 0, 255))
end

local function pack_u16_le(value)
  value = clamp(math.floor(value), 0, 65535)

  local b0 = value % 256
  local b1 = math.floor(value / 256) % 256

  return string.char(b0, b1)
end

local function pack_i16_le(value)
  value = clamp(math.floor(value), INT16_MIN, INT16_MAX)
  if value < 0 then
    value = value + 65536
  end

  return pack_u16_le(value)
end

local function pack_u64_decimal_le(value)
  local bytes = { 0, 0, 0, 0, 0, 0, 0, 0 }
  local text = tostring(value_or_zero(value))

  for i = 1, #text do
    local c = string.byte(text, i)
    if c >= 48 and c <= 57 then
      local carry = c - 48

      for j = 1, 8 do
        local next_value = bytes[j] * 10 + carry
        bytes[j] = next_value % 256
        carry = math.floor(next_value / 256)
      end
    end
  end

  return string.char(
    bytes[1], bytes[2], bytes[3], bytes[4],
    bytes[5], bytes[6], bytes[7], bytes[8]
  )
end

local function scaled_i16(value, scale)
  return clamp(round(value_or_zero(value) * scale), INT16_MIN, INT16_MAX)
end

local function state_id(state)
  if state == "off" then
    return 1
  elseif state == "initializing" then
    return 2
  elseif state == "tracking" then
    return 3
  elseif state == "degraded" then
    return 4
  elseif state == "lost" then
    return 5
  end

  return 0
end

local function checksum8(packet_type, payload)
  local sum = packet_type + #payload

  for i = 1, #payload do
    sum = sum + string.byte(payload, i)
  end

  return sum % 256
end

local function send_packet(packet_type, payload)
  local length = #payload

  if length > 255 then
    print("serial packet payload too long", packet_type, length)
    return false
  end

  local checksum = checksum8(packet_type, payload)

  Serial.writeBytes(
    string.char(START_BYTE, packet_type, length) ..
    payload ..
    string.char(checksum)
  )

  return true
end

local function send_pose_packet()
  local pose = Mighty.pose()
  local confidence = clamp(round(value_or_zero(pose.confidence) * CONFIDENCE_SCALE), 0, CONFIDENCE_SCALE)

  -- Payload binary:
  -- timestamp_ns,valid,confidence,x,y,z,qx,qy,qz,qw
  local payload =
    pack_u64_decimal_le(pose.timestamp_ns) ..
    pack_u8(pose.valid and 1 or 0) ..
    pack_u8(confidence) ..
    pack_i16_le(scaled_i16(pose.position.x, POSITION_SCALE)) ..
    pack_i16_le(scaled_i16(pose.position.y, POSITION_SCALE)) ..
    pack_i16_le(scaled_i16(pose.position.z, POSITION_SCALE)) ..
    pack_i16_le(scaled_i16(pose.orientation_xyzw.x, QUATERNION_SCALE)) ..
    pack_i16_le(scaled_i16(pose.orientation_xyzw.y, QUATERNION_SCALE)) ..
    pack_i16_le(scaled_i16(pose.orientation_xyzw.z, QUATERNION_SCALE)) ..
    pack_i16_le(scaled_i16(pose.orientation_xyzw.w, QUATERNION_SCALE))

  send_packet(TYPE_POSE, payload)
end

local function send_state_packet()
  local flags = 0
  if Mighty.recording() then
    flags = flags + 1
  end
  if Mighty.preview_mode() then
    flags = flags + 2
  end
  if Mighty.initializing() then
    flags = flags + 4
  end
  if Mighty.arm_connected() then
    flags = flags + 8
  end

  -- Payload binary:
  -- timestamp_ns,vio_state,flags
  local payload =
    pack_u64_decimal_le(Mighty.time_ns()) ..
    pack_u8(state_id(Mighty.vio_state())) ..
    pack_u8(flags)

  send_packet(TYPE_STATE, payload)
end

local function current_mode()
  if not Mighty.arm_connected() then return "starting" end
  if Mighty.recording() then return "recording" end
  if Mighty.preview_mode() then return "idle" end
  if Mighty.initializing() then return "starting" end
  return vio_blink[Mighty.vio_state()] or "starting"
end

local function show()
  LED.write(blink[mode][step][1])
end

local function handle_button()
  if Button.held(1200) and Mighty.preview_mode() and not Mighty.recording() then
    local ok = Mighty.start_recording()
    recording_started_by_button = ok
  end

  if not Button.released() then return end

  if recording_started_by_button then
    recording_started_by_button = false
    return
  end

  if Mighty.recording() then
    Mighty.stop_recording()
  elseif Button.held_ms() < 1200 then
    if Mighty.preview_mode() then
      Mighty.start_vio()
    else
      Mighty.stop_vio()
    end
  end
end

local function update_led(delta_ms)
  local next_mode = current_mode()
  if next_mode ~= mode then
    mode, step, led_elapsed = next_mode, 1, 0
    show()
    return
  end

  local pattern = blink[mode]
  led_elapsed = led_elapsed + delta_ms
  if led_elapsed < pattern[step][2] then return end

  led_elapsed = 0
  step = step % #pattern + 1
  show()
end

local function update_serial(delta_ms)
  pose_elapsed = pose_elapsed + delta_ms
  state_elapsed = state_elapsed + delta_ms

  if pose_elapsed >= POSE_PERIOD_MS then
    pose_elapsed = 0
    send_pose_packet()
  end

  if state_elapsed >= STATE_PERIOD_MS then
    state_elapsed = 0
    send_state_packet()
  end
end

function setup()
  LED.begin()
  Button.begin()
  LED.off()
  Serial.begin(BAUD)
  print("mcu led serial ready")
end

function loop(dt_ms)
  local delta_ms = dt_ms or 1

  handle_button()
  update_serial(delta_ms)
  update_led(delta_ms)
end
