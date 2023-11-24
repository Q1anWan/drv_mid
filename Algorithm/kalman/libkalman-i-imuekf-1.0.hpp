/*
 * @Description: An instance of expanded kalman filter of AHRS
 * @Author: qianwan
 * @Date: 2023-11-14 22:41:02
 * @LastEditTime: 2023-11-24 20:35:16
 * @LastEditors: qianwan
 * @original Author: Hongxi Wang
 */
/**
 * EKF::cEKF myekf(10, 0.001, 10000000, 0.9996);
*/
#pragma once
#ifndef LIB_KALMAN_IMUEKF_
#define LIB_KALMAN_IMUEKF_

#include "../Eigen/Dense"
#include "libkalman-1.0.hpp"

#define EKF_SCALAR float
namespace EKF {

class cEKF : public KalmanA::cKalmanA<EKF_SCALAR, 6, 1, 3> {
protected:
    EKF_SCALAR _quaternion[4];
    EKF_SCALAR _gyrobias[3];
    EKF_SCALAR _q1;                     // process_noise_quaternion
    EKF_SCALAR _q2;                     // process_noise_gyroscope
    EKF_SCALAR _r;                      // process_noise_accelerometer
    EKF_SCALAR _lambda_inv;             // fading coefficient inverse
    EKF_SCALAR _chi2threshold;          // Chi square testing threshold
    EKF_SCALAR _adaptive_gain_scale;    // Chi square adaptive scale
    EKF_SCALAR _orientation_cosine[3];  // Cosine of each axis
    EKF_SCALAR _gyro[3];
    EKF_SCALAR _accel[3];

    Eigen::Vector<EKF_SCALAR, 1> _chiSquare;
    Eigen::Vector<EKF_SCALAR, 3> _vec_chi;
    Eigen::Matrix<EKF_SCALAR, 3, 3> _mat_chi;
    Eigen::Vector<EKF_SCALAR, 6> _vec_measure_correct;

    EKF_SCALAR _accel_norm;
    EKF_SCALAR _gyro_norm;

    uint8_t _stable;
    uint32_t _chi_square_err_cnt;
    uint8_t _chi_square_stable;
    uint8_t _chi_square_stable_once;

public:
    cEKF(EKF_SCALAR process_noise_quaternion,
         EKF_SCALAR process_noise_gyroscope,
         EKF_SCALAR process_noise_accelerometer,
         EKF_SCALAR fading_coefficient) : KalmanA::cKalmanA<EKF_SCALAR, 6, 1, 3>(),
                                          _q1(process_noise_quaternion),
                                          _q2(process_noise_gyroscope),
                                          _r(process_noise_accelerometer),
                                          _lambda_inv(1.0f / fading_coefficient),
                                          _stable(0),
                                          _chi_square_err_cnt(0),
                                          _chi_square_stable_once(0),
                                          _chi2threshold(1e-8) {
        _gyrobias[0] = 0.0f;
        _gyrobias[1] = 0.0f;
        _gyrobias[2] = 0.0f;
        _vecXhat << 1, 0, 0, 0, 0, 0;
        _matFk = Eigen::Matrix<EKF_SCALAR, 6, 6>::Identity();
        _matPk << 100000, 0.1, 0.1, 0.1, 0.1, 0.1,
            0.1, 100000, 0.1, 0.1, 0.1, 0.1,
            0.1, 0.1, 100000, 0.1, 0.1, 0.1,
            0.1, 0.1, 0.1, 100000, 0.1, 0.1,
            0.1, 0.1, 0.1, 0.1, 100, 0.1,
            0.1, 0.1, 0.1, 0.1, 0.1, 100;
    }

    void ResetEKF() {
        KalmanA::cKalmanA<EKF_SCALAR, 6, 1, 3>::Reset();
        _vecXhat << 1, 0, 0, 0, 0, 0;
        _matFk = Eigen::Matrix<EKF_SCALAR, 6, 6>::Identity();
        _matPk << 100000, 0.1, 0.1, 0.1, 0.1, 0.1,
            0.1, 100000, 0.1, 0.1, 0.1, 0.1,
            0.1, 0.1, 100000, 0.1, 0.1, 0.1,
            0.1, 0.1, 0.1, 100000, 0.1, 0.1,
            0.1, 0.1, 0.1, 0.1, 100, 0.1,
            0.1, 0.1, 0.1, 0.1, 0.1, 100;
        _stable = 0;
        _chi_square_err_cnt = 0;
        _chi_square_stable_once = 0;
        _gyrobias[0] = 0.0f;
        _gyrobias[1] = 0.0f;
        _gyrobias[2] = 0.0f;
    }

    void GetQuaternion(float *qbuf) {
        memcpy(qbuf, _quaternion, sizeof(_quaternion));
    }


    uint8_t
    UpdateQuaternion(EKF_SCALAR accelx, EKF_SCALAR accely, EKF_SCALAR accelz, EKF_SCALAR gyrox, EKF_SCALAR gyroy,
                     EKF_SCALAR gyroz, EKF_SCALAR dt) {
        uint8_t skip_update_P = 0;

        EKF_SCALAR half_gx_dt, half_gy_dt, half_gz_dt;

        EKF_SCALAR norm_inverse;
        EKF_SCALAR tmp_value[4];

        _gyro[0] = gyrox - _gyrobias[0];
        _gyro[1] = gyroy - _gyrobias[1];
        _gyro[2] = gyroz - _gyrobias[2];

        /**Prepare Data**/
        half_gx_dt = 0.5f * _gyro[0] * dt;
        half_gy_dt = 0.5f * _gyro[1] * dt;
        half_gz_dt = 0.5f * _gyro[2] * dt;

        // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
        // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
        _matFk << 1, -half_gx_dt, -half_gy_dt, -half_gz_dt, 0, 0,
            half_gx_dt, 1, half_gz_dt, -half_gy_dt, 0, 0,
            half_gy_dt, -half_gz_dt, 1, half_gx_dt, 0, 0,
            half_gz_dt, half_gy_dt, -half_gx_dt, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

        /*Normalize*/
        _accel_norm = sqrt(accelx * accelx + accely * accely + accelz * accelz);
        _gyro_norm = sqrt(_gyro[0] * _gyro[0] + _gyro[1] * _gyro[1] + _gyro[2] * _gyro[2]);
        norm_inverse = 1.0f / _accel_norm;
        // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
        // 稍后在最后的姿态更新部分会利用StableFlag来确定
        _stable = (_gyro_norm < 0.3) && (fabs(_accel_norm - 9.8) < 0.5);
        // set Q R,过程噪声和观测噪声矩阵
        _matQk(0, 0) = _q1 * dt;
        _matQk(1, 1) = _q1 * dt;
        _matQk(2, 2) = _q1 * dt;
        _matQk(3, 3) = _q1 * dt;
        _matQk(4, 4) = _q2 * dt;
        _matQk(5, 5) = _q2 * dt;
        _matRk(0, 0) = _r;
        _matRk(1, 1) = _r;
        _matRk(2, 2) = _r;

        /**EKF**/
        /*Step-0 Get measurement data*/
        _vecZk << accelx * norm_inverse, accely * norm_inverse, accelz * norm_inverse;

        /*Step-1 predict xhat*/
        // xhat|k = F|k·xhat`|k-1 + B|k·u|k
        _vecXhat = _matFk * _vecXhat;
        // 更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
        tmp_value[0] = _vecXhat(0) * dt * 0.5f;
        tmp_value[1] = _vecXhat(1) * dt * 0.5f;
        tmp_value[2] = _vecXhat(2) * dt * 0.5f;
        tmp_value[3] = _vecXhat(3) * dt * 0.5f;

        _matFk(0, 4) = tmp_value[1];
        _matFk(0, 5) = tmp_value[2];
        _matFk(1, 4) = -tmp_value[0];
        _matFk(1, 5) = tmp_value[3];
        _matFk(2, 4) = -tmp_value[3];
        _matFk(2, 5) = -tmp_value[0];
        _matFk(3, 4) = tmp_value[2];
        _matFk(3, 5) = -tmp_value[1];
        // 并对零漂的方差进行限制,防止过度收敛并限幅防止发散
        _matPk(4, 4) *= _lambda_inv;
        _matPk(5, 5) *= _lambda_inv;
        // 限幅,防止发散
        if (_matPk(4, 4) > 10000) {
            _matPk(4, 4) = 10000;
        }
        if (_matPk(5, 5) > 10000) {
            _matPk(5, 5) = 10000;
        }
        // Normalize x_hat
        norm_inverse = 1.0f /
                       sqrt(_vecXhat(0) * _vecXhat(0) + _vecXhat(1) * _vecXhat(1) + _vecXhat(2) * _vecXhat(2) +
                            _vecXhat(3) * _vecXhat(3));
        _vecXhat *= norm_inverse;

        /*Step-2 predict P*/
        // P|k = F|k·P`|k-1·FT|k + Q|k
        _matPk = _matFk * _matPk * _matFk.transpose() + _matQk;
        // 在工作点处计算观测函数h(x)的Jacobi矩阵H
        tmp_value[0] = _vecXhat(0) * 2.0f;
        tmp_value[1] = _vecXhat(1) * 2.0f;
        tmp_value[2] = _vecXhat(2) * 2.0f;
        tmp_value[3] = _vecXhat(3) * 2.0f;

        _matHk <<
                (-tmp_value[2]), tmp_value[3], (-tmp_value[0]), tmp_value[1], 0, 0,
                tmp_value[1], tmp_value[0], tmp_value[3], tmp_value[2], 0, 0,
                tmp_value[0], (-tmp_value[1]), (-tmp_value[2]), tmp_value[3], 0, 0;

        /*Step-3 Update K*/
        // K = P|k·HT|k/(H|k·P|k·HT|k+R|k)
        // ChiSquare vector and matrix
        //  V = z(k) - h(xhat)
        _vec_chi(0) = 2.0f * (_vecXhat(1) * _vecXhat(3) - _vecXhat(0) * _vecXhat(2));
        _vec_chi(1) = 2.0f * (_vecXhat(0) * _vecXhat(1) + _vecXhat(2) * _vecXhat(3));
        _vec_chi(2) = _vecXhat(0) * _vecXhat(0) - _vecXhat(1) * _vecXhat(1) - _vecXhat(2) * _vecXhat(2) +
                      _vecXhat(3) * _vecXhat(3);
        // 计算预测值和各个轴的方向余弦
        _orientation_cosine[0] = acos(abs(_vec_chi(0)));
        _orientation_cosine[1] = acos(abs(_vec_chi(1)));
        _orientation_cosine[2] = acos(abs(_vec_chi(2)));
        // ChiSquare vector and matrix
        //  V = z(k) - h(xhat)
        _vec_chi = _vecZk - _vec_chi;
        // A=(H|k·P|k·HT|k+R|k)^-1
        _mat_chi = (_matHk * _matPk * _matHk.transpose() + _matRk).inverse();
        // ChiSquare = VT·A·V
        _chiSquare = _vec_chi.transpose() * _mat_chi * _vec_chi;
        EKF_SCALAR chi_val = _chiSquare(0);
        // Through chi square, decide method to fusion data
        _chi_square_stable_once = _chi_square_stable;
        _chi_square_stable = (chi_val < 0.5 * _chi2threshold);
        // Once converged and rk is big
        if ((_chi_square_stable == 0) && _chi_square_stable_once) {
            _stable ? _chi_square_err_cnt++ : _chi_square_err_cnt = 0;
            if (_chi_square_err_cnt > 50) {
                // Filter is divergence
                _chi_square_stable_once = 0;
                _chi_square_err_cnt = 0;
                return 0x01;
            }
            skip_update_P = 1;  // Filter only update by predict. Measurement won't be used to correct xhat and P
        } else                  // if divergent or rk is not that big/acceptable,use adaptive gain
        {
            if (chi_val > 0.1f * _chi2threshold && _chi_square_stable) {
                _adaptive_gain_scale = (_chi2threshold - chi_val) / (0.9f * _chi2threshold);
            } else {
                // divergent need to rest
                _adaptive_gain_scale = 1;
            }
        }
        if (skip_update_P == 0) {
            // Measurement value will be used to correct xhat and P
            // Calculate K Xhat`|k P`|k
            _matK = _matPk * _matHk.transpose() * _mat_chi * _adaptive_gain_scale;
            _matK(4, 0) *= _orientation_cosine[0] * 0.6366197723675813430755350534;
            _matK(4, 1) *= _orientation_cosine[0] * 0.6366197723675813430755350534;
            _matK(4, 2) *= _orientation_cosine[0] * 0.6366197723675813430755350534;
            _matK(5, 0) *= _orientation_cosine[1] * 0.6366197723675813430755350534;
            _matK(5, 1) *= _orientation_cosine[1] * 0.6366197723675813430755350534;
            _matK(5, 2) *= _orientation_cosine[1] * 0.6366197723675813430755350534;
            // 计算修正值
            _vec_measure_correct = _matK * (_vecZk - _matHk * _vecXhat);
            // 零漂修正限幅,一般不会有过大的漂移
            if (_vec_measure_correct(4) > 1e-2f * dt) {
                _vec_measure_correct(4) = 1e-2f * dt;
            } else if (_vec_measure_correct(4) < -1e-2f * dt) {
                _vec_measure_correct(4) = -1e-2f * dt;
            }
            if (_vec_measure_correct(5) > 1e-2f * dt) {
                _vec_measure_correct(5) = 1e-2f * dt;
            } else if (_vec_measure_correct(5) < -1e-2f * dt) {
                _vec_measure_correct(5) = -1e-2f * dt;
            }
            // Do not correct yaw data
            _vec_measure_correct(3) = 0;
            _vecXhat += _vec_measure_correct;

            /*Step-5 Update P*/
            // P`|k = P|k - K·H|k·P|k
            _matPk = _matPk - _matK * _matHk * _matPk;
        }
        // suppress filter excessive convergence
        if (_matPk(0, 0) < 0) {
            _matPk(0, 0) = 0;
        }
        if (_matPk(1, 1) < 0) {
            _matPk(1, 1) = 0;
        }
        if (_matPk(2, 2) < 0) {
            _matPk(2, 2) = 0;
        }
        if (_matPk(3, 3) < 0) {
            _matPk(3, 3) = 0;
        }
        if (_matPk(4, 4) < 0) {
            _matPk(4, 4) = 0;
        }
        if (_matPk(5, 5) < 0) {
            _matPk(5, 5) = 0;
        }

        _quaternion[0] = _vecXhat(0);
        _quaternion[1] = _vecXhat(1);
        _quaternion[2] = _vecXhat(2);
        _quaternion[3] = _vecXhat(3);
        _gyrobias[0] = _vecXhat(4);
        _gyrobias[1] = _vecXhat(5);
        _gyrobias[2] = 0;
        
        return 0;
    }
};
}  // namespace EKF

#endif