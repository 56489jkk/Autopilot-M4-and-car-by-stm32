/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-09     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATRIX_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATRIX_H_

#include<math.h>

//函数功能：矩阵加法
void MatrixAdd( float* fMatrixA,float* fMatrixB,float* Result,
        unsigned int m,unsigned int n );
//函数功能：矩阵减法
void MatrixSub( float* fMatrixA,float* fMatrixB,float* Result,
        unsigned int m,unsigned int n );
//函数功能：矩阵乘法
void MatrixMultiply( float* fMatrixA,unsigned int uRowA,unsigned int uColA,
float* fMatrixB,unsigned int uRowB,unsigned int uColB,float* MatrixResult );
//函数功能：矩阵转置
void MatrixTranspose(float* fMatrixA,unsigned int m,unsigned n,float* fMatrixB);
void MatrixProduct(float* A, int m, int n, float* B, int k, float* C);
//函数功能：单位矩阵生成
void MatrixE(float* fMatrixA,unsigned int n);
//函数功能：2阶矩阵行列式的值
double MatrixDet2(float* fMatrixA);
//函数功能：2阶矩阵求逆
int MatrixInverse2(float* fMatrixA,float* fMatrixB);
//函数功能：矩阵求逆
int MatrixInverse(float* fMatrixA,int n,float ep);
void UD(float * A,int  n,float * U,float * D);
//函数功能：求矩阵范数
float Norm(float*fMatrixA,int iRow,int iCol);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_MATH_CL_MATRIX_H_ */
