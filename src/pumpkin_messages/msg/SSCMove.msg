#Command sent to SSC to a single PIN
#This is intended to be used inside the SSCMoveList msg.
#channel: Number of channel (PIN)
uint8 channel
#pulse: The value of the pulse sent
uint16 pulse
#speed: The value of the speed (in us)
#(set it to 0 to not send the speed)
uint16 speed
