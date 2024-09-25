#pragma once
#ifndef BITARRAY_HPP
#define BITARRAY_HPP

#include <cstdint>

/**
 * @brief BitArray class for managing an array of fixed-width bit elements.
 */
class BitArray {
private:
    uint8_t* data;         // Pointer to the underlying data buffer.
    uint32_t bufferSize;   // Size of the data buffer in bytes.
    uint32_t bitWidth;     // Number of bits for each element.
    uint32_t numElements;  // Maximum number of elements that can fit in the buffer.
    uint32_t mask;         // Mask used to ensure value fits within bitWidth.

public:
    /**
     * @brief Constructs a BitArray object.
     * 
     * @param inputData The pointer to the external data buffer.
     * @param bufferSizeInBytes Size of the data buffer in bytes.
     * @param bitWidth Number of bits used to represent each element.
     */
    BitArray(uint8_t* inputData, uint32_t bufferSizeInBytes, uint32_t bitWidth);

    /**
     * @brief Gets the maximum number of elements that can be stored in the BitArray.
     * 
     * @return Maximum number of elements.
     */
    uint32_t getMaxElements() const;

    /**
     * @brief Returns the internal data buffer and optionally its size.
     * 
     * @param buffer_length Optional pointer to store the size of the buffer.
     * @return Pointer to the data buffer.
     */
    uint8_t* getDataBuffer(uint32_t* buffer_length = nullptr);

    /**
     * @brief Sets a specific element at the given index.
     * 
     * @param index Index of the element to set.
     * @param value The value to set at the given index.
     */
    void set(uint32_t index, uint32_t value);

    /**
     * @brief Gets the value of a specific element at the given index.
     * 
     * @param index Index of the element to get.
     * @return Value of the element at the given index.
     */
    uint32_t get(uint32_t index) const;

    /**
     * @brief Copies data from another BitArray of the same bit width and buffer size.
     * 
     * @param other The BitArray to copy from.
     * @return Reference to the current BitArray object.
     */
    BitArray& operator=(const BitArray& other);

    /**
     * @brief Sets all elements in the array to a specific value.
     * 
     * @param value The value to set for all elements.
     */
    void memset(uint32_t value);

    /**
     * @brief Proxy class for accessing and modifying individual elements.
     */
    class Proxy {
    private:
        BitArray& bitArray;
        uint32_t index;

    public:
        Proxy(BitArray& arr, uint32_t idx);
        Proxy& operator=(uint32_t value);
        operator uint32_t() const;
        Proxy& operator+=(uint32_t value);
        Proxy& operator-=(uint32_t value);
        Proxy& operator*=(uint32_t value);
        Proxy& operator/=(uint32_t value);
        Proxy& operator%=(uint32_t value);
        Proxy& operator++();
        Proxy operator++(int);
        Proxy& operator--();
        Proxy operator--(int);
    };

    Proxy operator[](uint32_t index);  // Non-const operator[]
    uint32_t operator[](uint32_t index) const;  // Const operator[]

    /**
     * @brief Iterator class for traversing the BitArray.
     */
    class Iterator {
    private:
        BitArray& bitArray;
        uint32_t index;

    public:
        Iterator(BitArray& arr, uint32_t startIndex);
        Proxy operator*();  // Return Proxy for modification
        Iterator& operator++();
        bool operator!=(const Iterator& other) const;
    };

    Iterator begin();
    Iterator end();
};

#endif // BITARRAY_HPP