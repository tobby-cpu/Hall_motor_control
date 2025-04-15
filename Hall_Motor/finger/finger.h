#ifndef MATH_UTILS_H
#define MATH_UTILS_H
/******************************************************************************/
#include "stdio.h"
#include "stm32f10x.h"
//***************************************************************************************************************************************************//
void matrix_multiply(double *A, double *B, double *C, int m, int n, int p) ;
void compute_JT(double JT[3][3], double h1, double h2, double AF, double beta, double gamma, double rho, double FH, double px, double py);
void compute_tao(double T[1][4], double F0, double judge_1, double judge_2, double k1, double delta1, double F_01, double k2, double delta2, 
double F_02, double theta2, double theta2_0, double k3, double F_03);
int inverse_3x3(double *A, double *invA);
double calculation_beta(double alpha, double AC, double AF, double CE, double EF, double zeta, double epsilon);
double calculation_gamma(double FG, double FH, double GI, double HI, double psy, double phi, double beta);
int main_calculate(double F_01, double F_02, double F_03, double alpha, double beta, double gamma, double rho);
/******************************************************************************/
#endif
