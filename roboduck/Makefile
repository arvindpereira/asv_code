roboduck : pid.o roboduck.o ax1500.o guidance.o utm.o qboatModels.o
	g++ -o roboduck roboduck.o pid.o ax1500.o guidance.o utm.o qboatModels.o -L ../common -lm -lmstimer -I . -I ../common

roboduck.o : roboduck.cpp ms_timer.o
	g++ -c roboduck.cpp -I . -I ../common

pid.o : pid.cpp pid.h
	g++ -c pid.cpp -I . -I ../common

#ms_timer.o : ms_timer.cpp
#	g++ -c ms_timer.cpp -I . -I ../common

ax1500.o : ax1500.cpp
	g++ -c ax1500.cpp -I . -I ../common

guidance.o : guidance.cpp
	g++ -c guidance.cpp -I . -I ../common

utm.o : utm.c Utm.h Constants.h
	g++ -c utm.c -I . -I ../common

qboatModels.o : qboatModels.cpp qboatModels.h
	g++ -c qboatModels.cpp -I . -I ../common

clean : 
	rm -f *.o roboduck
