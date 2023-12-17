-- This is complete script of payload drop. lat_des and long_des are the drop location. accurate within 1-2 meters

local SCRIPT_NAME     = 'payload_demo.lua'
local RUN_INTERVAL_MS = 100

local MAV_SEVERITY_INFO  = 6
local lat_des =  1.9134376*100000000
local long_des =  7.2912096*100000000
local pwm = 1900
local timeout = 3000
local epsilon = 70

-- wrapper for gcs:send_text()
local function gcs_msg(severity, txt)
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

-- ! setup/initialization logic
local last_posit = nil

function update()
    if not ahrs:healthy() then return update, RUN_INTERVAL_MS end
    if arming:is_armed() then
        local current_posit = ahrs:get_position()
        local lat =  current_posit:lat()
        local long = current_posit:lng()

        if last_posit == nil then
            last_posit = current_posit
            return update, RUN_INTERVAL_MS
        end
        
        if lat == nil and long == nil then return update, RUN_INTERVAL_MS end

        if (lat < lat_des + epsilon and lat > lat_des - epsilon and long < long_des + epsilon and long > long_des - epsilon) then
            gcs:send_text(3, string.format("PAYLOAD RELEASED"))
            SRV_Channels:set_output_pwm_chan_timeout(9, pwm, timeout)
        end

        last_posit = current_posit
        gcs:send_text(6, string.format(" Lat:%.7f Long:%.7f ", lat, long))
        return update, RUN_INTERVAL_MS
    end
    gcs:send_text(6, string.format("prachit"))
    return update, RUN_INTERVAL_MS
end

gcs_msg(MAV_SEVERITY_INFO, 'Initialized.')

return update()