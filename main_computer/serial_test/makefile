CFLAGS = -O 
CC = g++ 
SRC = main.cpp serialUtil.cpp 
OBJ = $(SRC:.cpp = .o) 
mainDriveDriver: $(OBJ) 
	$(CC) $(CFLAGS) -o mainDriveDriver $(OBJ) 
clean: 
	rm -f core *.o 