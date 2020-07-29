compile:
g++ -g test.cpp CameraDevice.cpp -o device `pkg-config opencv --cflags --libs` -std=c++11 -lMVSDK 
g++ -g CameraDevice.cpp -o device `pkg-config opencv --cflags --libs` -std=c++11 -lMVSDK 

