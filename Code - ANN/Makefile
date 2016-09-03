CC = g++ -std=c++11

INCLUDES = -I./ -Ibullet/ -Iutils/ -Isrc -Isrc/strategies
LIBRARIES = -lOpenCL -lGL -lGLU -lstdc++ -lc -lm -lglut -lGLU -pthread $(filter-out -lippicv, $(shell pkg-config --libs opencv))

SRC := $(shell find src/ -name '*.cpp')
FILE_NAMES_SRC = $(SRC:.cpp=.o)

UTILS := $(wildcard utils/*.cpp)
FILE_NAMES_UTILS = $(UTILS:.cpp=.o)

SRCBULLET = bullet/
SOURCES := $(shell find $(SRCBULLET) -name '*.cpp')
FILE_NAMES_BULLET = $(SOURCES:.cpp=.o)

EXEC = DQN


.cpp.o: $(SRC) $(UTILS) $(SOURCES)
	$(CC) -c $(INCLUDES) $(LIBRARIES) -Wall -ffast-math -o $@ $< -w

all: $(EXEC)
	@echo Executando... Aguarde.

run:
	./$(EXEC)

$(EXEC): $(FILE_NAMES_SRC) $(FILE_NAMES_BULLET) $(FILE_NAMES_UTILS)
	@$(CC) -Wall -fexceptions -g -m64 -fno-rtti -w -o $(EXEC) $(FILE_NAMES_SRC) $(FILE_NAMES_BULLET) $(FILE_NAMES_UTILS) $(INCLUDES) $(LIBRARIES)

clean:
	rm $(EXEC) $(FILE_NAMES_SRC) $(FILE_NAMES_BULLET) $(FILE_NAMES_UTILS)

clean-src: 
	rm $(EXEC) $(FILE_NAMES_SRC) $(FILE_NAMES_UTILS)




