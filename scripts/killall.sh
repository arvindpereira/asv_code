# !bin sh
sudo killall -9 m3dmg
sudo killall -9 kalman
sudo killall -9 readGPS
sudo killall -9 roboduck
sudo killall -9 dbex
sudo killall -9 xbowReader
sudo ipcrm -M 0x4d2
sudo ipcrm -M 0x162e
