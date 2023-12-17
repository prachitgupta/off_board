function update () -- periodic function that will be called
    local roll = ahrs:get_roll() -- fetch the current position of the vehicle
    r = string.format("%.2f",roll)
    gcs:send_text(6, r)           -- fetch the home position of the vehicle
    return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
  end
  
  return update, 1000   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded