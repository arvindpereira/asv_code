# !bin/sh
nohup ../gpsReader/readGPS -b 4800 /dev/ttyUSB0 &
nohup ../3dmgReader/m3dmg 2 &
#../imuReader/readIMU &
nohup ../roboduck/roboduck -q &
nohup ../RoboduckServer/dbex -q &
nohup ../eKalmanFilter/kalman &
