-- This script was used to find lat and long as read by ahrs, which were later used to find the drop location

local SCRIPT_NAME     = 'location_demo.lua'
local RUN_INTERVAL_MS = 1000

local MAV_SEVERITY_INFO      = 6

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

        last_posit = current_posit
        gcs:send_text(6, string.format(" Lat:%.7f Long:%.7f ", lat, long))
        return update, RUN_INTERVAL_MS
    end
    gcs:send_text(6, string.format("prachit"))
    return update, RUN_INTERVAL_MS
end


gcs_msg(MAV_SEVERITY_INFO, 'Initialized.')

return update()