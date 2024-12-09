#ifndef MY_LIBRARY_H
#define MY_LIBRARY_H


void helloWorld();
void printinfo();

#define REGISTER_TENSORRT_PLUGIN()                                                                           \
    void printinfo() {std::cout << "hello " << std::endl;}


#endif // MY_LIBRARY_H




