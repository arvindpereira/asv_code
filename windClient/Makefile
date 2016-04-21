windClient: Socket.o ClientComm.o ClientSocket.o byteorder.o windClient.o CommProtC.o
	g++ -o windClient Socket.o ClientComm.o ClientSocket.o byteorder.o windClient.o CommProtC.o -lpthread -I. -I ../common -L../common -lshm -lmstimer

ClientComm.o : ClientComm.cpp ClientComm.h
	g++ -c ClientComm.cpp -I . -I ../common/

ClientSocket.o : ClientSocket.cpp ClientSocket.h
	g++ -c ClientSocket.cpp -I .

Socket.o : Socket.cpp SocketException.h Socket.h
	g++ -c Socket.cpp -I .

byteorder.o : byteorder.cpp
	g++ -c byteorder.cpp -I .

circularbuf.o : circularbuf.cpp circularbuf.h
	g++ -c circularbuf.cpp -I .

windClient.o : windClient.cpp
	g++ -c windClient.cpp -I . -I../common/

CommProtC.o : CommProtC.cpp CommProtC.h
	g++ -c CommProtC.cpp -I .

clean:
	rm -f windClient
	rm -f *.o
	rm -f *.*~
