<!--
 * @Description: markdown file
 * @Author: qianwan
 * @Date: 2023-11-24 20:22:53
 * @LastEditTime: 2023-11-24 20:34:16
 * @LastEditors: qianwan
-->
## Expanded kalman filter of AHRS    

Original code by [Hongxi Wang](https://github.com/WangHongxi2001/kalman-filter-C-implementation.git "GitHub").
Use [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page "eigen") library.


Known questions:
1. In arm-clang environment, Eigen martixs cannot work normally. Shown as matrixs cannot be assigned correctly.
2. The previous quesition causes twice times the error on Yaw axis than normal.