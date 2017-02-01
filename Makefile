FLAGS =
compile:
	g++ $(FLAGS) src/*.cpp src/*.a -o build/enumerate_clusters
run:
	./build/enumerate_clusters