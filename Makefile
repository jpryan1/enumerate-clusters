AFLAGS =  -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo
LFLAGS = -std=c++11 `pkg-config --libs lib/glfw3.pc` 
CFLAGS = -std=c++11 -c -o
FAST_FLAG =

recompile:
	rm build/src/* ;
	make compile ;

configure:
	@cd src/nauty && \
	echo "Configuring nauty..." && \
	./configure &> ../../docs/nauty/config.log  && \
	echo "Making nauty library..." && \
	make &>../../docs/nauty/make.log nauty.a && \
	echo "Cleaning up..." && \
	cd ../.. && \
	cp src/nauty/nauty.a build/lib/ && \
	rm -rf src/nauty/ && \
	echo "Compiling glew..." && \
	gcc -c -o build/lib/glew.o src/glew.c  ;
	
build/src/sphere.o: src/sphere.cpp src/sphere.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/sphere.o src/sphere.cpp 
	
build/src/edge.o: src/edge.cpp src/edge.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/edge.o src/edge.cpp 

build/src/main.o: src/main.cpp src/animation.h src/Configuration.h src/Bank.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/main.o src/main.cpp 

build/src/animation.o: src/animation.cpp src/animation.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/animation.o src/animation.cpp 

build/src/Config.o: src/Configuration.cpp src/Configuration.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/Config.o src/Configuration.cpp

build/src/canonize.o: src/canonize.cpp src/Configuration.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/canonize.o src/canonize.cpp

build/src/project.o: src/project.cpp src/Configuration.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/project.o src/project.cpp

build/src/walk.o: src/walk.cpp src/Configuration.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/walk.o src/walk.cpp

build/src/dimension.o: src/dimension.cpp src/Configuration.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/dimension.o src/dimension.cpp

build/src/Bank.o: src/Bank.cpp src/Configuration.h src/Bank.h
	g++ $(FAST_FLAG) $(CFLAGS) build/src/Bank.o src/Bank.cpp

build/src/Timer.o: src/Timer.cpp src/Timer.h 
	g++ $(FAST_FLAG) $(CFLAGS) build/src/Timer.o src/Timer.cpp


compile: build/src/animation.o build/src/main.o build/src/Config.o build/src/walk.o build/src/dimension.o build/src/canonize.o build/src/project.o build/src/Bank.o build/src/sphere.o build/src/edge.o build/lib/glew.o build/src/Timer.o
	g++ $(LFLAGS) $(AFLAGS) $(FAST_FLAG) -o build/enumerate_clusters build/src/animation.o build/src/main.o build/src/Config.o build/src/walk.o build/src/dimension.o build/src/canonize.o build/src/project.o build/src/Bank.o build/src/sphere.o build/src/edge.o build/lib/glew.o build/src/Timer.o build/lib/nauty.a

run:
	./build/enumerate_clusters

animate:
	./build/enumerate_clusters 1

fast_compile:
	make compile FAST_FLAG=-O2

fast_recompile:
	make recompile FAST_FLAG=-O2
