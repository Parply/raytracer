raytracer: raytracer.cpp
	g++ raytracer.cpp -o raytracer -Wall -Wextra -Ofast -fopenmp -march=native -lgd
	./raytracer
	ffmpeg -start_number 0 -i trace%d.png -vcodec mpeg4 tracevideo.avi




