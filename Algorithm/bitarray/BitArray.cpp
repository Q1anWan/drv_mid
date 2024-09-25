/*
    *  BitArray.cpp
    *  Version 1.0
    *  Created on: 2024.9.25
    *  qianwan.jin
*/
#include "BitArray.hpp"
#include <cstring>
/**
 * @brief Constructs a BitArray object with a given buffer and bit width.
 *
 * @param inputData Pointer to the input data buffer.
 * @param bufferSizeInBytes Size of the buffer in bytes.
 * @param bitWidth Width of each bit element.
 */
BitArray::BitArray(uint8_t* inputData, uint32_t bufferSizeInBytes, uint32_t bitWidth)
        : data(inputData), bufferSize(bufferSizeInBytes), bitWidth(bitWidth), mask((1U << bitWidth) - 1) {
    if (bitWidth > 32 || bitWidth <= 0) {
        // Handle error: Bit width must be between 1 and 32.
        return;
    }

    numElements = (bufferSize * 8) / bitWidth;
    if (numElements > UINT32_MAX) {
        numElements = UINT32_MAX;
    }
}

/**
 * @brief Gets the maximum number of elements that can be stored in the BitArray.
 *
 * @return uint32_t Maximum number of elements.
 */
uint32_t BitArray::getMaxElements() const {
    return numElements;
}

/**
 * @brief Gets the data buffer and optionally its length.
 *
 * @param buffer_length Pointer to store the length of the buffer.
 * @return uint8_t* Pointer to the data buffer.
 */
uint8_t* BitArray::getDataBuffer(uint32_t* buffer_length) {
    if (buffer_length != nullptr) {
        *buffer_length = bufferSize;
    }
    return data;
}

/**
 * @brief Sets the value at a specified index in the BitArray.
 *
 * @param index Index at which to set the value.
 * @param value Value to set.
 */
void BitArray::set(uint32_t index, uint32_t value) {
    if (index >= numElements) {
        // Handle error: Index out of range.
        return;
    }

    value &= mask;

    uint32_t bitPos = index * bitWidth;
    uint32_t byteIndex = bitPos / 8;
    uint8_t bitOffset = bitPos % 8;

    uint8_t bitsRemaining = bitWidth;

    for (uint8_t i = 0; bitsRemaining > 0; ++i) {
        uint8_t bitsInCurrentByte = (bitsRemaining < (8 - bitOffset)) ? bitsRemaining : (8 - bitOffset);
        uint32_t currentMask = ((1U << bitsInCurrentByte) - 1) << bitOffset;
        data[byteIndex + i] = (data[byteIndex + i] & ~currentMask) | ((value << bitOffset) & currentMask);
        bitsRemaining -= bitsInCurrentByte;
        value >>= bitsInCurrentByte;
        bitOffset = 0;
    }
}

/**
 * @brief Gets the value at a specified index in the BitArray.
 *
 * @param index Index from which to get the value.
 * @return uint32_t Value at the specified index.
 */
uint32_t BitArray::get(uint32_t index) const {
    if (index >= numElements) {
        return 0;
    }

    uint32_t bitPos = index * bitWidth;
    uint32_t byteIndex = bitPos / 8;
    uint8_t bitOffset = bitPos % 8;

    uint32_t value = 0;
    uint8_t bitsRemaining = bitWidth;

    for (uint8_t i = 0; bitsRemaining > 0; ++i) {
        uint8_t bitsInCurrentByte = (bitsRemaining < (8 - bitOffset)) ? bitsRemaining : (8 - bitOffset);
        uint32_t currentMask = (1U << bitsInCurrentByte) - 1;
        value |= ((data[byteIndex + i] >> bitOffset) & currentMask) << (bitWidth - bitsRemaining);
        bitsRemaining -= bitsInCurrentByte;
        bitOffset = 0;
    }

    return value & mask;
}

/**
 * @brief Assignment operator for BitArray.
 *
 * @param other The other BitArray to assign from.
 * @return BitArray& Reference to this BitArray.
 */
BitArray& BitArray::operator=(const BitArray& other) {
    if (this != &other) {
        if (bufferSize != other.bufferSize || bitWidth != other.bitWidth) {
            return *this;
        }
        memcpy(data, other.data, bufferSize);
    }
    return *this;
}

/**
 * @brief Sets all elements in the BitArray to a specified value.
 *
 * @param value Value to set all elements to.
 */
void BitArray::memset(uint32_t value) {
    for (uint32_t i = 0; i < numElements; ++i) {
        set(i, value);
    }
}

/**
 * @brief Proxy constructor.
 *
 * @param arr Reference to the BitArray.
 * @param idx Index in the BitArray.
 */
BitArray::Proxy::Proxy(BitArray& arr, uint32_t idx) : bitArray(arr), index(idx) {}

/**
 * @brief Assignment operator for Proxy.
 *
 * @param value Value to assign.
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator=(uint32_t value) {
    bitArray.set(index, value);
    return *this;
}

/**
 * @brief Conversion operator for Proxy.
 *
 * @return uint32_t Value at the proxy's index.
 */
BitArray::Proxy::operator uint32_t() const {
    return bitArray.get(index);
}

/**
 * @brief Addition assignment operator for Proxy.
 *
 * @param value Value to add.
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator+=(uint32_t value) {
    uint32_t currentValue = bitArray.get(index);
    bitArray.set(index, currentValue + value);
    return *this;
}

/**
 * @brief Subtraction assignment operator for Proxy.
 *
 * @param value Value to subtract.
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator-=(uint32_t value) {
    uint32_t currentValue = bitArray.get(index);
    bitArray.set(index, currentValue - value);
    return *this;
}

/**
 * @brief Multiplication assignment operator for Proxy.
 *
 * @param value Value to multiply.
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator*=(uint32_t value) {
    uint32_t currentValue = bitArray.get(index);
    bitArray.set(index, currentValue * value);
    return *this;
}

/**
 * @brief Division assignment operator for Proxy.
 *
 * @param value Value to divide.
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator/=(uint32_t value) {
    if (value == 0) {
        return *this;
    }
    uint32_t currentValue = bitArray.get(index);
    bitArray.set(index, currentValue / value);
    return *this;
}

/**
 * @brief Modulus assignment operator for Proxy.
 *
 * @param value Value to modulus.
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator%=(uint32_t value) {
    if (value == 0) {
        return *this;
    }
    uint32_t currentValue = bitArray.get(index);
    bitArray.set(index, currentValue % value);
    return *this;
}

/**
 * @brief Pre-increment operator for Proxy.
 *
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator++() {
    uint32_t value = bitArray.get(index);
    bitArray.set(index, ++value);
    return *this;
}

/**
 * @brief Post-increment operator for Proxy.
 *
 * @param int Dummy parameter to differentiate from pre-increment.
 * @return Proxy Copy of the Proxy before increment.
 */
BitArray::Proxy BitArray::Proxy::operator++(int) {
    Proxy temp = *this;
    ++(*this);
    return temp;
}

/**
 * @brief Pre-decrement operator for Proxy.
 *
 * @return Proxy& Reference to this Proxy.
 */
BitArray::Proxy& BitArray::Proxy::operator--() {
    uint32_t value = bitArray.get(index);
    bitArray.set(index, --value);
    return *this;
}

/**
 * @brief Post-decrement operator for Proxy.
 *
 * @param int Dummy parameter to differentiate from pre-decrement.
 * @return Proxy Copy of the Proxy before decrement.
 */
BitArray::Proxy BitArray::Proxy::operator--(int) {
    Proxy temp = *this;
    --(*this);
    return temp;
}

/**
 * @brief Indexing operator for BitArray.
 *
 * @param index Index to access.
 * @return Proxy Proxy object for the specified index.
 */
BitArray::Proxy BitArray::operator[](uint32_t index) {
    return Proxy(*this, index);
}

/**
 * @brief Const indexing operator for BitArray.
 *
 * @param index Index to access.
 * @return uint32_t Value at the specified index.
 */
uint32_t BitArray::operator[](uint32_t index) const {
    return get(index);
}

/**
 * @brief Iterator constructor.
 *
 * @param arr Reference to the BitArray.
 * @param startIndex Starting index for the iterator.
 */
BitArray::Iterator::Iterator(BitArray& arr, uint32_t startIndex) : bitArray(arr), index(startIndex) {}

/**
 * @brief Dereference operator for Iterator.
 *
 * @return Proxy Proxy object for the current iterator index.
 */
BitArray::Proxy BitArray::Iterator::operator*() {
    return bitArray[index];
}

/**
 * @brief Pre-increment operator for Iterator.
 *
 * @return Iterator& Reference to this Iterator.
 */
BitArray::Iterator& BitArray::Iterator::operator++() {
    ++index;
    return *this;
}

/**
 * @brief Inequality operator for Iterator.
 *
 * @param other The other Iterator to compare with.
 * @return bool True if the iterators are not equal, false otherwise.
 */
bool BitArray::Iterator::operator!=(const Iterator& other) const {
    return index != other.index;
}

/**
 * @brief Gets an iterator to the beginning of the BitArray.
 *
 * @return Iterator Iterator to the beginning.
 */
BitArray::Iterator BitArray::begin() {
    return Iterator(*this, 0);
}

/**
 * @brief Gets an iterator to the end of the BitArray.
 *
 * @return Iterator Iterator to the end.
 */
BitArray::Iterator BitArray::end() {
    return Iterator(*this, numElements);
}