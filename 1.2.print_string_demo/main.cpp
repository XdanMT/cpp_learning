#include <iostream>
#include "method.h"
#include <string>


int main(){
	int a = 1;
	std::cout << "a: " << a << std::endl;
	std::string name = "wzj";
	PrintName(name);
	PrintName("hello world");

	int * p = new int;
	*p = 10;
	std::cout << "p: " << p << std::endl;

	
	return 0;
}

