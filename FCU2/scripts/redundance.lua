-- This script is a test of param set and get

local count = 0
local last_value = 0
local last_rc8_input = 0

local switch_high = 2
local switch_low = 0
local standby_function = 76

local scripting_rc_1 = rc:find_channel_for_option(300)


-- for fast param acess it is better to get a param object,
-- this saves the code searching for the param by name every time
local JSON_MASTER = Parameter()
if not JSON_MASTER:init('SIM_JSON_MASTER') then
  gcs:send_text(6, 'get JSON_MASTER failed')
end

local SYSID_THISMAV = Parameter()
if not SYSID_THISMAV:init('SYSID_THISMAV') then
  gcs:send_text(6, 'get SYSID_THISMAV failed')
end


local sysid = SYSID_THISMAV:get()

-- this allows this example to catch the otherwise fatal error
-- not recommend if error is possible/expected, use separate construction and init

-- local user_param = Parameter('SCR_USER1')
-- is equivalent to:
-- local user_param = Parameter()
-- assert(user_param:init('SCR_USER1'), 'No parameter: SCR_USER1')
gcs:send_text(6, 'LUA: hello')

function update() -- this is the loop which periodically runs

  -- get and print all the scripting parameters
  local value = JSON_MASTER:get()
  if value then
      if value ~= last_value then
          gcs:send_text(6, string.format('LUA: SIM_JSON_MASTER: %i',value))
          last_value = value
      end
  else
    gcs:send_text(6, 'LUA: get SIM_JSON_MASTER failed')
  end

  rc8_input = rc:get_pwm(8)
  -- standby switch enable == high

  if rc8_input ~= last_rc8_input then
      if sysid == 2 then
          if rc8_input > 1500 then
              if not JSON_MASTER:set(0) then
                  gcs:send_text(6, string.format('LUA: failed to set JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_high)
                  gcs:send_text(6, string.format('LUA: set JSON_MASTER to 0'))
              end
          else
              if not JSON_MASTER:set(1) then
                  gcs:send_text(6, string.format('LUA: failed to set JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_low)
                  gcs:send_text(6, string.format('LUA: set JSON_MASTER to 1'))
              end
          end
      end
      if sysid == 1 then
          if rc8_input > 1500 then
              if not JSON_MASTER:set(0) then
                  gcs:send_text(6, string.format('LUA: failed to set JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_low)
                  gcs:send_text(6, string.format('LUA: set JSON_MASTER to 0'))
              end
          else
              if not JSON_MASTER:set(1) then
                  gcs:send_text(6, string.format('LUA: failed to set JSON_MASTER'))
              else
                  rc:run_aux_function(standby_function, switch_high)
                  gcs:send_text(6, string.format('LUA: set JSON_MASTER to 1'))
              end
          end
      end
      last_rc8_input = rc8_input
  end

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
