#include <iostream>
#include "my_library.h"
#include <string>

std::string name = "wangzijin";
extern void printName();

int main() {
    helloWorld();
    printinfo();
    printName();
    return 0;
}

