
function update() -- this is the loop which periodically runs
  local pwm = 1900
  local pwm2 = 1000
  local timeout = 5000
  SRV_Channels:set_output_pwm_chan_timeout(9, pwm, timeout)
  gcs:send_text(6, string.format("payload releasing"))
  --SRV_Channels:set_output_pwm_chan_timeout(9, pwm2, timeout)
  --gcs:send_text(6, string.format("baack to normal"))
  return update, 100 -- reschedules the loop at 50Hz
end
gcs:send_text(6, string.format("HELLO PRACHIT"))
return update()