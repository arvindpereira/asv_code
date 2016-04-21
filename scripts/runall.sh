# !bin/sh
nohup ../gpsReader/readGPS /dev/ttyUSB2 -q &
nohup ../roboduck/roboduck -q &
nohup ../RoboduckServer/dbex -q &
nohup ../xbowReader/xbowReader &
nohup ../3dmgReader/m3dmg 2 &
nohup ../eKalmanFilter/kalman &
	nohup ../rudderControl/rudder &
