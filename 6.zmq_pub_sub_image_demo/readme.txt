sudo apt install libopencv-dev
sudo apt install protobuf
sudo apt install zmq

g++ -o sub_exec sub.cpp image.pb.cc `pkg-config --cflags --libs opencv4 protobuf` -lzmq

g++ -o pub_exec pub.cpp image.pb.cc `pkg-config --cflags --libs opencv4 protobuf` -lzmq