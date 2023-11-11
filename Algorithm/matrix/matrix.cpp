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

#include "matrix.h"

// hat of vector
matrixf::Matrixf<3, 3> vector3f::hat(matrixf::Matrixf<3, 1> vec) {
        float hat[9] = {0, -vec(2, 0), vec(1, 0),
                        vec(2, 0), 0, -vec(0, 0),
                        -vec(1, 0), vec(0, 0), 0};
        return {hat};
    }

// cross product
matrixf::Matrixf<3, 1> vector3f::cross(const matrixf::Matrixf<3, 1> &vec1, const matrixf::Matrixf<3, 1> &vec2) {
        return vector3f::hat(vec1) * vec2;
    }
