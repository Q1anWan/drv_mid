#include "BitArray.hpp"
#include <iostream>
#include <cstring>


int main() {
    uint8_t* buf_ptr = (uint8_t*)malloc(32);
    memset(buf_ptr, 0, 32);

    void* bitarr_ptr = (void*)malloc(sizeof(BitArray));
    BitArray* arr = new (bitarr_ptr) BitArray(buf_ptr, 32, 12);

    arr->operator[](0) = 123;
    arr->operator[](1) = 321;
    arr->operator[](2) = 114;

    std::cout << "Initial values:" << std::endl;
    for (auto elem : *arr) {
        std::cout << elem << std::endl;
    }
    std::cout << std::endl;

    arr->memset(1234);

    (*arr)[0] = 12;
    (*arr)[1] = 34;
    (*arr)[2] = 56;
    (*arr)[3] = 78;

    std::cout << "After memset and setting new values:" << std::endl;
    for (auto elem : *arr) {
        std::cout << elem << std::endl;
    }
    std::cout << std::endl;

    for (auto elem : *arr) {
        elem++;
        elem*=2;
    }

    std::cout << "After for in range right value operation:" << std::endl;
    for (auto elem : *arr) {
        std::cout << elem << std::endl;
    }
    std::cout << std::endl;

    // Test increment and decrement operators
    ++(*arr)[0];
    (*arr)[1]++;
    --(*arr)[2];
    (*arr)[3]--;

    std::cout << "After increment and decrement operations:" << std::endl;
    for (auto elem : *arr) {
        std::cout << elem << std::endl;
    }
    std::cout << std::endl;

    // Test addition and subtraction assignment operators
    (*arr)[0] += 100;
    (*arr)[1] -= 50;

    std::cout << "After addition and subtraction assignment operations:" << std::endl;
    for (auto elem : *arr) {
        std::cout << elem << std::endl;
    }
    std::cout << std::endl;

    // Test overflow
    (*arr)[0] = 4096+1;  // Should be masked to fit within bitWidth
    (*arr)[100] = 0;     // Should handle overflow

    std::cout << "After overflow and underflow tests:" << std::endl;
    for (auto elem : *arr) {
        std::cout << elem << std::endl;
    }
    std::cout << std::endl;

    // Clean up memory
    free(buf_ptr);
    free(bitarr_ptr);

    return 0;
}