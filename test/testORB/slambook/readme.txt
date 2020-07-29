make by pkg-config:
g++ feature_extraction.cpp -o feature_extract `pkg-config opencv --cflags --libs` -std=c++11

or make by cmake :
mkdir build
cd build
cmake ..
make 


run :
./feature_extract logo/logo-cap3.jpg logo_cap-4.png
