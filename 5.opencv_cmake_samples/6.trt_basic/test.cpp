#include <iostream>

class Dims32
{
public:
    //! The maximum rank (number of dimensions) supported for a tensor.
    static constexpr int MAX_DIMS{8};
    //! The rank (number of dimensions).
    int nbDims;
    //! The extent of each dimension.
    int d[MAX_DIMS];
};


int main(){
    Dims32 tmp{4, {1, 2, 3, 4}};
    std::cout << "nbDims: " << tmp.nbDims << std::endl;
    for(int i = 0; i < tmp.nbDims; i++){
        std::cout << "d[" << i << "]: " << tmp.d[i] << std::endl;
    }
    std::cout << sizeof(float) << std::endl;
    std::cout << sizeof(int) << std::endl;
    std::cout << sizeof(double) << std::endl;
    return 0;
}