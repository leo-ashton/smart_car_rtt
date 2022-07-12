/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_transform_matrix.h
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 11-Jul-2022 16:35:23
 */

#ifndef GET_TRANSFORM_MATRIX_H
#define GET_TRANSFORM_MATRIX_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void get_transform_matrix(const float corner_coordinates[8],
                                 float transform_matrix[9]);

extern void get_transform_matrix_initialize(void);

extern void get_transform_matrix_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for get_transform_matrix.h
 *
 * [EOF]
 */
