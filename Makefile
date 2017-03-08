FLAGS = `pkg-config --libs lib/glfw3.pc` -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo
OBJS = 


configure:
	@cd src/nauty && \
	echo "Configuring nauty..." && \
	./configure &> ../../docs/nauty/config.log  && \
	echo "Making nauty library..." && \
	make &>../../docs/nauty/make.log nauty.a && \
	echo "Cleaning up..." && \
	cd ../.. && \
	cp src/nauty/nauty.a build/src/ && \
	rm -rf src/nauty/ && \
	echo "Compiling glew..." && \
	gcc -c -o build/src/glew.o src/glew.c  ;
	
build/src/sphere.o: src/sphere.cpp src/sphere.h
	g++ -c -o build/src/sphere.o src/sphere.cpp 

build/src/main.o: src/main.cpp src/animation.h src/Configuration.h src/Bank.h
	g++ -std=c++11 -c -o build/src/main.o src/main.cpp 

build/src/animation.o: src/animation.cpp src/animation.h
	g++ -c -o build/src/animation.o src/animation.cpp 

build/src/Configuration.o: src/Configuration.cpp src/Configuration.h
	g++ -c -o build/src/Configuration.o src/Configuration.cpp

build/src/Bank.o: src/Bank.cpp src/Configuration.h src/Bank.h
	g++ -c -o build/src/Bank.o src/Bank.cpp

compile: build/src/animation.o build/src/main.o build/src/Configuration.o build/src/Bank.o build/src/sphere.o build/src/glew.o
	g++ $(FLAGS) -o build/enumerate_clusters build/src/animation.o build/src/main.o build/src/Configuration.o build/src/Bank.o build/src/sphere.o build/src/glew.o build/src/nauty.a

run:
	./build/enumerate_clusters

animate:
	./build/enumerate_clusters 1
