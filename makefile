all: raytracer raytracer_cuda

raytracer: raytracer.cpp
	g++ raytracer.cpp -o raytracer -Wall -Wextra -Ofast -fopenmp -march=native -lgd
	
raytracer_cuda: raytracer_cuda.cu
	nvcc -x cu raytracer_cuda.cu -o raytracer_cuda -O3 -lgd -Xcompiler '-I ~/cuda-samples/Common -fopenmp -Wall -Wextra -Ofast -march=native'
	





