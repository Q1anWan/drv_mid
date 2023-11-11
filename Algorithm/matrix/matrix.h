/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   A matrix calculate lib.
 *          Use last arm_math lib to get better performance
 ******************************************************************************
 * Original code (C)
 * @author  Spoon Guan
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * https://github.com/SJTU-RoboMaster-Team/Matrix_and_Robotics_on_STM32
 * MIT License
 * All rights reserved.
 * ******************************************************************************
 * Modified code (C)
 * @note    Modified operators
 * @date    2023/11/11
 * @author  qianwan.Jin
 * @version 1.0
 * @stepper 0.0
 * *****************************************************************************
 */

/**
 * Arm matrix storage as mat(i,j)=mat_data[i*cols+j]
 * [a11,a12,a13,a21,a22,a23,a31,a32,a33]
 */


#ifndef MATRIX_H
#define MATRIX_H

#include <cstdint>

#include "arm_math.h"

namespace matrixf {
// Matrix class
    template<uint32_t _rows, uint32_t _cols>
    class Matrixf {
    protected:
        // data
        float data_[_rows * _cols];

    public:
        // arm matrix instance
        arm_matrix_instance_f32 arm_mat_;

        // Constructor without input data
        Matrixf() {
            arm_mat_.numCols = _cols;
            arm_mat_.numRows = _rows;
            arm_mat_.pData = data_;
        }

        Matrixf(const float* data) : Matrixf() {
            memcpy(this->data_, data, _rows * _cols * sizeof(float));
        }

        // Copy constructor
        Matrixf(const Matrixf<_rows, _cols> &mat) : Matrixf() {
            memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
        }

        // Destructor
        ~Matrixf() = default;

        /*  Operators about elements    */
        // Row size
        uint32_t rows() { return _rows; }

        // Column size
        uint32_t cols() { return _cols; }

        // Column size
        uint32_t size() { return _rows * _cols; }

        // Overload the function call operator for element access
        float &operator()(const uint32_t &row, const uint32_t &col) {
            return data_[row * _cols + col];
        }

        // Overload the function call operator for getting specific row vector
        Matrixf<1, _cols> row(const uint32_t &row) {
            Matrixf<1, _cols> res;
            memcpy(&res(0, 0), data_ + row * _cols, _cols * sizeof(float));
            return res;
        }

        // Overload the function call operator for getting specific col vector
        Matrixf<_rows, 1> col(const uint32_t &col) {
            Matrixf<_rows, 1> res;
            for (uint32_t i = 0; i < _rows; i++) {
                res(i, 0) = data_[i * _cols + col];
            }
            return res;
        }

//        // Overload the << operator for matrix assignment
//        template<typename... Args>
//        Matrixf &operator<<(Args... args) {
//            if (sizeof...(args) != _rows * _cols) { return *this; }
//
//            float values[] = {static_cast<float>(args)...};
//            for (uint32_t i = 0; i < _rows; ++i) {
//                for (uint32_t j = 0; j < _cols; ++j) {
//                    (*this)(i, j) = values[i * _cols + j];
//                }
//            }
//            return *this;
//        }

        // Set values for a specific row
        void setRow(const uint32_t &row, const Matrixf<1, _cols> &newRow) {
            memcpy(data_ + row * _cols, &newRow(0, 0), _cols * sizeof(float));
        }

        // Set values for a specific column
        void setCol(const uint32_t &col, const Matrixf<_rows, 1> &newCol) {
            for (uint32_t i = 0; i < _rows; ++i) {
                (*this)(i, col) = newCol(i, 0);
            }
        }

        /*Operators about operations*/
        Matrixf<_rows, _cols> &operator=(const Matrixf<_rows, _cols> mat) {
            memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
            return *this;
        }

        Matrixf<_rows, _cols> &operator+=(const Matrixf<_rows, _cols> mat) {
            arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
            return *this;
        }

        Matrixf<_rows, _cols> &operator-=(const Matrixf<_rows, _cols> mat) {
            arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
            return *this;
        }

        Matrixf<_rows, _cols> &operator*=(const float &val) {
            arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
            return *this;
        }

        Matrixf<_rows, _cols> &operator*=(const Matrixf<_rows, _cols> &mat) {
            arm_mat_mult_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
            return *this;
        }

        Matrixf<_rows, _cols> &operator/=(const float &val) {
            arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
            return *this;
        }

        Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols> &mat) {
            Matrixf<_rows, _cols> res;
            arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
            return res;
        }

        Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols> &mat) {
            Matrixf<_rows, _cols> res;
            arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
            return res;
        }

        Matrixf<_rows, _cols> operator*(const float &val) {
            Matrixf<_rows, _cols> res;
            arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
            return res;
        }

        friend Matrixf<_rows, _cols> operator*(const float &val,
                                               const Matrixf<_rows, _cols> &mat) {
            Matrixf<_rows, _cols> res;
            arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
            return res;
        }

        Matrixf<_rows, _cols> operator/(const float &val) {
            Matrixf<_rows, _cols> res;
            arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
            return res;
        }

        // Matrix multiplication
        template<uint32_t cols>
        friend Matrixf<_rows, cols> operator*(const Matrixf<_rows, _cols> &mat1,
                                              const Matrixf<_cols, cols> &mat2) {
            Matrixf<_rows, cols> res;
            arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
            return res;
        }

        // Transpose
        Matrixf<_cols, _rows> transpose() {
            Matrixf<_cols, _rows> res;
            arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
            return res;
        }

        // Trace
        float trace() {
            float res = 0;
            for (uint32_t i = 0; i < fmin(_rows, _cols); i++) {
                res += (*this)(i, i);
            }
            return res;
        }

        // Inverse
        Matrixf<_cols, _rows> inverse() {
            Matrixf<_cols, _rows> res(*this);
            arm_mat_inverse_f32(res, res);
            return res;
        }

        // Norm Frobenius-Norm
        float norm() { return _sqrtf((this->transpose() * *this)(0, 0)); }

        // normalized
        Matrixf<_cols, _rows> normalized() {
            Matrixf<_cols, _rows> res;
            arm_mat_scale_f32(&arm_mat_, 1.0f / this->norm(), &res);
            return res;
        }
    };



/* Matrix functions*/
// Special Matrices
// Zero matrix
    template<uint32_t _rows, uint32_t _cols>
    Matrixf<_rows, _cols> Zeros() {
        float data[_rows * _cols] = {0};
        return Matrixf<_rows, _cols>(data);
    }

// Ones matrix
    template<uint32_t _rows, uint32_t _cols>
    Matrixf<_rows, _cols> Ones() {
        float data[_rows * _cols] = {0};
        for (uint32_t i = 0; i < _rows * _cols; i++) {
            data[i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

// Identity matrix
    template<uint32_t _rows, uint32_t _cols>
    Matrixf<_rows, _cols> Eye() {
        float data[_rows * _cols] = {0};
        for (uint32_t i = 0; i < fmin(_rows, _cols); i++) {
            data[i * _cols + i] = 1;
        }
        return Matrixf<_rows, _cols>(data);
    }

// Diagonal matrix
    template<uint32_t _rows, uint32_t _cols>
    Matrixf<_rows, _cols> Diag(Matrixf<_rows, 1> vec) {
        Matrixf<_rows, _cols> res = matrixf::Zeros<_rows, _cols>();
        for (uint32_t i = 0; i < fmin(_rows, _cols); i++) {
            res[i][i] = vec[i][0];
        }
        return res;
    }

}  // namespace matrixf

namespace vector3f {
// hat of vector
    matrixf::Matrixf<3, 3> hat(matrixf::Matrixf<3, 1> vec);

// cross product
    matrixf::Matrixf<3, 1> cross(const matrixf::Matrixf<3, 1> &vec1, const matrixf::Matrixf<3, 1> &vec2);
}  // namespace vector3f

#endif  // MATRIX_H
