CC = g++
CFLAGS = -Wall -O3

evolution: evolution.o
	$(CC) $(CFLAGS) evolution.o -o evolution
  
evolution.o: evolution.cpp
	$(CC) $(CFLAGS) -c evolution.cpp

clean:
	rm *.o evolution
