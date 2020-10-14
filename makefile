raytracer: raytracer.cpp
	g++ raytracer.cpp -o raytracer -Wall -Wextra -Ofast -fopenmp -march=native -lgd
	
raytracer_cuda: raytracer_cuda.cu
	nvcc -x cu raytracer_cuda.cu -O3 -lgd -Xcompiler '-Wall -Wextra -Ofast -fopenmp -march=native'
	





