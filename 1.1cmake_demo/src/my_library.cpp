#include "my_library.h"
#include <iostream>

extern std::string name;

void printName(){
    name = "Alice";
    std::cout << "global variable 'name' is: " << name << std::endl;
}


void helloWorld() {
    std::cout << "Hello, World from the dynamic library!" << std::endl;

}

REGISTER_TENSORRT_PLUGIN();
