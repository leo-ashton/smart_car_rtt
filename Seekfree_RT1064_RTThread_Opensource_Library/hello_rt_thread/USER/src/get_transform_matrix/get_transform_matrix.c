/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: get_transform_matrix.c
 *
 * MATLAB Coder version            : 5.4
 * C/C++ source code generated on  : 11-Jul-2022 16:35:23
 */

/* Include Files */
#include "get_transform_matrix.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static float maximum(const float x[4]);

static float minimum(const float x[4]);

/* Function Definitions */
/*
 * Arguments    : const float x[4]
 * Return Type  : float
 */
static float maximum(const float x[4])
{
  float ex;
  int idx;
  int k;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      float f;
      f = x[k - 1];
      if (ex < f) {
        ex = f;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const float x[4]
 * Return Type  : float
 */
static float minimum(const float x[4])
{
  float ex;
  int idx;
  int k;
  if (!rtIsNaNF(x[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!rtIsNaNF(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 5; k++) {
      float f;
      f = x[k - 1];
      if (ex > f) {
        ex = f;
      }
    }
  }
  return ex;
}

/*
 * Arguments    : const float corner_coordinates[8]
 *                float transform_matrix[9]
 * Return Type  : void
 */
void get_transform_matrix(const float corner_coordinates[8],
                          float transform_matrix[9])
{
  float A[64];
  float b_transform_matrix[9];
  float B[8];
  float b_corner_coordinates[4];
  float c_corner_coordinates[4];
  float h;
  float w;
  int a;
  int i;
  int j;
  int jA;
  int jp1j;
  int k;
  int temp_tmp;
  signed char ipiv[8];
  /* 四个原顶点 */
  b_corner_coordinates[0] = corner_coordinates[0];
  c_corner_coordinates[0] = corner_coordinates[0];
  b_corner_coordinates[1] = corner_coordinates[2];
  c_corner_coordinates[1] = corner_coordinates[2];
  b_corner_coordinates[2] = corner_coordinates[4];
  c_corner_coordinates[2] = corner_coordinates[4];
  b_corner_coordinates[3] = corner_coordinates[6];
  c_corner_coordinates[3] = corner_coordinates[6];
  w = roundf(maximum(b_corner_coordinates) - minimum(c_corner_coordinates));
  b_corner_coordinates[0] = corner_coordinates[1];
  c_corner_coordinates[0] = corner_coordinates[1];
  b_corner_coordinates[1] = corner_coordinates[3];
  c_corner_coordinates[1] = corner_coordinates[3];
  b_corner_coordinates[2] = corner_coordinates[5];
  c_corner_coordinates[2] = corner_coordinates[5];
  b_corner_coordinates[3] = corner_coordinates[7];
  c_corner_coordinates[3] = corner_coordinates[7];
  h = roundf(maximum(b_corner_coordinates) - minimum(c_corner_coordinates));
  B[0] = 1.0F;
  B[1] = 1.0F;
  B[2] = h;
  B[3] = 1.0F;
  B[4] = h;
  B[5] = w;
  B[6] = 1.0F;
  B[7] = w;
  /* 变换后的四个顶点，方程右边的值 */
  /* 联立解方程组，方程的系数 */
  /* 求解变换矩阵的行列式 */
  A[0] = corner_coordinates[1];
  A[8] = corner_coordinates[0];
  A[16] = 1.0F;
  A[24] = 0.0F;
  A[32] = 0.0F;
  A[40] = 0.0F;
  A[48] = -corner_coordinates[1];
  A[56] = -corner_coordinates[0];
  A[1] = 0.0F;
  A[9] = 0.0F;
  A[17] = 0.0F;
  A[25] = corner_coordinates[1];
  A[33] = corner_coordinates[0];
  A[41] = 1.0F;
  A[49] = -corner_coordinates[1];
  A[57] = -corner_coordinates[0];
  A[2] = corner_coordinates[3];
  A[10] = corner_coordinates[2];
  A[18] = 1.0F;
  A[26] = 0.0F;
  A[34] = 0.0F;
  A[42] = 0.0F;
  A[50] = -h * corner_coordinates[3];
  A[58] = -h * corner_coordinates[2];
  A[3] = 0.0F;
  A[11] = 0.0F;
  A[19] = 0.0F;
  A[27] = corner_coordinates[3];
  A[35] = corner_coordinates[2];
  A[43] = 1.0F;
  A[51] = -corner_coordinates[3];
  A[59] = -corner_coordinates[2];
  A[4] = corner_coordinates[5];
  A[12] = corner_coordinates[4];
  A[20] = 1.0F;
  A[28] = 0.0F;
  A[36] = 0.0F;
  A[44] = 0.0F;
  A[52] = -h * corner_coordinates[5];
  A[60] = -h * corner_coordinates[4];
  A[5] = 0.0F;
  A[13] = 0.0F;
  A[21] = 0.0F;
  A[29] = corner_coordinates[5];
  A[37] = corner_coordinates[4];
  A[45] = 1.0F;
  A[53] = -w * corner_coordinates[5];
  A[61] = -w * corner_coordinates[4];
  A[6] = corner_coordinates[7];
  A[14] = corner_coordinates[6];
  A[22] = 1.0F;
  A[30] = 0.0F;
  A[38] = 0.0F;
  A[46] = 0.0F;
  A[54] = -corner_coordinates[7];
  A[62] = -corner_coordinates[6];
  A[7] = 0.0F;
  A[15] = 0.0F;
  A[23] = 0.0F;
  A[31] = corner_coordinates[7];
  A[39] = corner_coordinates[6];
  A[47] = 1.0F;
  A[55] = -w * corner_coordinates[7];
  A[63] = -w * corner_coordinates[6];
  for (i = 0; i < 8; i++) {
    ipiv[i] = (signed char)(i + 1);
  }
  for (j = 0; j < 7; j++) {
    int b_tmp;
    int mmj_tmp;
    signed char i1;
    mmj_tmp = 6 - j;
    b_tmp = j * 9;
    jp1j = b_tmp + 2;
    jA = 8 - j;
    a = 0;
    w = fabsf(A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      h = fabsf(A[(b_tmp + k) - 1]);
      if (h > w) {
        a = k - 1;
        w = h;
      }
    }
    if (A[b_tmp + a] != 0.0F) {
      if (a != 0) {
        a += j;
        ipiv[j] = (signed char)(a + 1);
        for (k = 0; k < 8; k++) {
          jA = k << 3;
          temp_tmp = j + jA;
          w = A[temp_tmp];
          jA += a;
          A[temp_tmp] = A[jA];
          A[jA] = w;
        }
      }
      i = (b_tmp - j) + 8;
      for (a = jp1j; a <= i; a++) {
        A[a - 1] /= A[b_tmp];
      }
    }
    jA = b_tmp;
    for (a = 0; a <= mmj_tmp; a++) {
      w = A[(b_tmp + (a << 3)) + 8];
      if (w != 0.0F) {
        i = jA + 10;
        jp1j = (jA - j) + 16;
        for (temp_tmp = i; temp_tmp <= jp1j; temp_tmp++) {
          A[temp_tmp - 1] += A[((b_tmp + temp_tmp) - jA) - 9] * -w;
        }
      }
      jA += 8;
    }
    i1 = ipiv[j];
    if (i1 != j + 1) {
      w = B[j];
      B[j] = B[i1 - 1];
      B[i1 - 1] = w;
    }
  }
  for (k = 0; k < 8; k++) {
    jA = k << 3;
    if (B[k] != 0.0F) {
      i = k + 2;
      for (a = i; a < 9; a++) {
        B[a - 1] -= B[k] * A[(a + jA) - 1];
      }
    }
  }
  for (k = 7; k >= 0; k--) {
    jA = k << 3;
    w = B[k];
    if (w != 0.0F) {
      w /= A[k + jA];
      B[k] = w;
      for (a = 0; a < k; a++) {
        B[a] -= B[k] * A[a + jA];
      }
    }
  }
  for (i = 0; i < 8; i++) {
    transform_matrix[i] = B[i];
  }
  transform_matrix[8] = 1.0F;
  i = 0;
  jp1j = 0;
  for (jA = 0; jA < 9; jA++) {
    b_transform_matrix[jp1j + 3 * i] = transform_matrix[jA];
    i++;
    if (i > 2) {
      i = 0;
      jp1j++;
    }
  }
  transform_matrix[0] = b_transform_matrix[0];
  transform_matrix[3] = b_transform_matrix[3];
  transform_matrix[6] = b_transform_matrix[6];
  transform_matrix[1] = b_transform_matrix[1];
  transform_matrix[4] = b_transform_matrix[4];
  transform_matrix[7] = b_transform_matrix[7];
  transform_matrix[2] = b_transform_matrix[2];
  transform_matrix[5] = b_transform_matrix[5];
  transform_matrix[8] = b_transform_matrix[8];
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void get_transform_matrix_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void get_transform_matrix_terminate(void)
{
}

/*
 * File trailer for get_transform_matrix.c
 *
 * [EOF]
 */
