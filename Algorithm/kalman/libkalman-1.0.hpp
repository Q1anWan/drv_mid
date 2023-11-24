/*
 * @Description: A base class of kalman filter 
 * @Author: qianwan
 * @Date: 2023-11-14 22:41:02
 * @LastEditTime: 2023-11-24 20:14:04
 * @LastEditors: qianwan
 */
#pragma once
#ifndef LIB_KALMAN_
#define LIB_KALMAN_

#include "../Eigen/Dense"

namespace KalmanA {

    template<typename Scalar, uint32_t Xsize, uint32_t Usize, uint32_t Zsize>
    class cKalmanA {
    protected:

        Eigen::Vector<Scalar, Xsize> _vecXhat;
        Eigen::Vector<Scalar, Usize> _vecUk;
        Eigen::Vector<Scalar, Zsize> _vecZk;

        /*Middle matrix*/
        Eigen::Matrix<Scalar, Xsize, Xsize> _matPk;
        Eigen::Matrix<Scalar, Xsize, Zsize> _matK;

        /*Const matrix*/
        Eigen::Matrix<Scalar, Xsize, Xsize> _matFk;
        Eigen::Matrix<Scalar, Xsize, Usize> _matBk;
        Eigen::Matrix<Scalar, Xsize, Xsize> _matQk;
        Eigen::Matrix<Scalar, Zsize, Xsize> _matHk;
        Eigen::Matrix<Scalar, Zsize, Zsize> _matRk;


    public:
        cKalmanA() :
                _vecXhat(Eigen::Vector<Scalar, Xsize>::Zero()),
                _vecUk(Eigen::Vector<Scalar, Usize>::Zero()),
                _vecZk(Eigen::Vector<Scalar, Zsize>::Zero()),

                /*Middle matrix*/
                _matPk(Eigen::Matrix<Scalar, Xsize, Xsize>::Zero()),
                _matK(Eigen::Matrix<Scalar, Xsize, Zsize>::Zero()),

                /*Const matrix*/
                _matFk(Eigen::Matrix<Scalar, Xsize, Xsize>::Zero()),
                _matBk(Eigen::Matrix<Scalar, Xsize, Usize>::Zero()),
                _matQk(Eigen::Matrix<Scalar, Xsize, Xsize>::Zero()),
                _matHk(Eigen::Matrix<Scalar, Zsize, Xsize>::Zero()),
                _matRk(Eigen::Matrix<Scalar, Zsize, Zsize>::Zero()) {};

        cKalmanA(Eigen::Matrix<Scalar, Xsize, Xsize> &matFk,
                 Eigen::Matrix<Scalar, Xsize, Usize> &matBk,
                 Eigen::Matrix<Scalar, Xsize, Xsize> &matQk,
                 Eigen::Matrix<Scalar, Zsize, Xsize> &matHk,
                 Eigen::Matrix<Scalar, Zsize, Zsize> &matRk
        ) :
                _vecXhat(Eigen::Vector<Scalar, Xsize>::Zero()),
                _matPk(Eigen::Matrix<Scalar, Xsize, Xsize>::Zero()),
                _matK(Eigen::Matrix<Scalar, Xsize, Zsize>::Zero()),
                _matFk(matFk), _matBk(matBk), _matQk(matQk), _matHk(matHk), _matRk(matRk) {}

        void Reset() {
            _vecXhat = Eigen::Vector<Scalar, Xsize>::Zero();
            _vecUk = Eigen::Vector<Scalar, Usize>::Zero();
            _vecZk = Eigen::Vector<Scalar, Zsize>::Zero();

            /*Middle matrix*/
            _matPk = Eigen::Matrix<Scalar, Xsize, Xsize>::Zero();
            _matK = Eigen::Matrix<Scalar, Xsize, Zsize>::Zero();

            /*Const matrix*/
            _matFk = Eigen::Matrix<Scalar, Xsize, Xsize>::Zero();
            _matBk = Eigen::Matrix<Scalar, Xsize, Usize>::Zero();
            _matQk = Eigen::Matrix<Scalar, Xsize, Xsize>::Zero();
            _matHk = Eigen::Matrix<Scalar, Zsize, Xsize>::Zero();
            _matRk = Eigen::Matrix<Scalar, Zsize, Zsize>::Zero();
        }

    };
};

#endif