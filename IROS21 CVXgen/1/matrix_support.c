/* Produced by CVXGEN, 2021-01-15 09:44:47 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.tau[0]*params.Fd[0])-rhs[1]*(-params.tau[0]*params.Fd[1])-rhs[2]*(-params.tau[0]*params.Fd[2]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.tau[0]*params.Fd[0]);
  lhs[1] = -rhs[0]*(-params.tau[0]*params.Fd[1]);
  lhs[2] = -rhs[0]*(-params.tau[0]*params.Fd[2]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2);
  lhs[1] = rhs[1]*(2);
  lhs[2] = rhs[2]*(2);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
}
void fillh(void) {
  work.h[0] = params.Td[0];
}
void fillb(void) {
}
void pre_ops(void) {
}
