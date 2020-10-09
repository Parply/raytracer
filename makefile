raytracer: raytracer.cpp
	g++ raytracer.cpp -o raytracer -Wall -Wextra -Ofast -fopenmp -march=native $(gdlib-config --cflags --libs)
