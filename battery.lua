local SCRIPT_NAME     = 'batt_data_logger.lua'
local MAV_SEVERITY_INFO  = 6
local RUN_INTERVAL_MS = 3000
local Filename = 'logs/batt.log'
local voltage = 0
local current = 0
local capacity = 0
local left = 100

-- wrapper for gcs:send_text()
local function gcs_msg(severity, txt)
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

local file = io.open(Filename, "a")

function update()

    -- fixed this line to work properly (YouTube demo has a logic error)
    if (not battery:healthy(0)) then 
        gcs:send_text(4,"battery monitoring not started")
        return update, RUN_INTERVAL_MS
    end
    
    voltage = battery:voltage(0)
    capacity = battery:consumed_mah(0)
    current = battery:current_amps(0)
    left =    battery:capacity_remaining_pct(0)

    if voltage == nil or current == nil or capacity == nil then
        gcs:send_text(4,"nil value received")
        return update, RUN_INTERVAL_MS end
    
    if left < 20 then
        gcs:send_text(2,"LESS THAN 20PERCENT LEFT")
        return update, RUN_INTERVAL_MS end
        

    file:write(string.format("Voltage = %fV  Current = %f  Capacity = %fmah",voltage,current,capacity))
    gcs:send_text(MAV_SEVERITY_INFO,string.format("Voltage = %fV  Current = %fA  Capacity = %fmah",voltage,current,capacity))
    return update, RUN_INTERVAL_MS
end

gcs_msg(MAV_SEVERITY_INFO, 'Initialized.')

return update()