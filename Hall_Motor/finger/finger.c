#include "stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include "MyProject.h"
#include "user.h"
#include "finger.h"

#define PI 3.14159265358979323846
#define rtIsNaN(x) (isnan(x))
#define rtIsInf(x) (isinf(x))
#define rtNaN (NAN)
#define rtInf (INFINITY)
#define MATRIX_ELEMENT(m, i, j) m[i][j]

double JT[3][3], h1, h2, px, py, udge_1,judge_1,judge_2;     
const double PI_OVER_2;
double JT[3][3];
double tao[1][4];
int i, j, k, m;  

double delta0, delta1, delta2, theta2;
double JA[4][3];
typedef struct {
    double result1[3];
    double result2[3];
    double result3[3];
    double result4[3];
} JA_Result;
double T[1][4];
double k1 = 1;
double k2 = 1;
double k3 = 1;
double F0 = 5;
double theta2_0 = 2.3562;

double temp[1][3];
double invJT[3][3];
double F[1][3];
double beta_o;
double gamma_o;
JA_Result results;
//***************************************************************************************************************************************************//

JA_Result res;
double a_tmp, a_tmp_tmp, a_tmp_tmp_tmp, a_tmp_tmp_tmp_tmp, b_a_tmp, b_a_tmp_tmp, b_a_tmp_tmp_tmp, b_reslut1_1_tmp, b_reslut1_1_tmp_tmp;
double c_a_tmp, c_a_tmp_tmp, c_reslut1_1_tmp, c_reslut1_1_tmp_tmp, d_a_tmp, d_a_tmp_tmp, d_reslut1_1_tmp, d_reslut1_1_tmp_tmp, e_a_tmp;
double e_a_tmp_tmp, e_reslut1_1_tmp, e_reslut1_1_tmp_tmp, f_a_tmp, f_a_tmp_tmp, f_reslut1_1_tmp, f_reslut1_1_tmp_tmp, g_a_tmp, g_a_tmp_tmp;
double g_reslut1_1_tmp, g_reslut1_1_tmp_tmp, h_a_tmp, h_a_tmp_tmp, h_reslut1_1_tmp, h_reslut1_1_tmp_tmp, i_a_tmp, i_a_tmp_tmp, i_reslut1_1_tmp;
double j_a_tmp, j_a_tmp_tmp, j_reslut1_1_tmp, k_a_tmp, k_a_tmp_tmp, k_reslut1_1_tmp, l_a_tmp, l_a_tmp_tmp, l_reslut1_1_tmp, m_a_tmp, m_a_tmp_tmp;
double m_reslut1_1_tmp, n_reslut1_1_tmp, o_reslut1_1_tmp, p_reslut1_1_tmp, q_reslut1_1_tmp, r_reslut1_1_tmp, reslut1_1_tmp, reslut1_1_tmp_tmp;
double reslut1_1_tmp_tmp_tmp, reslut1_1_tmp_tmp_tmp_tmp, result1_1, result1_2, result1_3, result2_1, result2_2, result2_3, result3_1, result3_2, result3_3;
double result4_1, result4_2, result4_3;
/****************************************************************************************************************************************************/

double zeta = 0.7854;
double  epsilon = 3.1416;
double  lambda = 0;
double  AF = 45;
double  AC = 14.2891;
double  AB = 5.8710;
double  EF = 11.5000;
double  DE = 24.4060;
double  CD = 15.2790;
double  DK = 2.8000;
double  BK = 28.6700;
double  phi = 156.5722;
double  FH = 28;
double  FG = 5.2818;
double  psy = 0.7854;
double  HI = 11;
double  GI = 32.4708;
double  CE = 39.6850;
double  F_0 = 0;
double  F_1 = 0;
double  F_2 = 0;
//***************************************************************************************************************************************************//
double calculate_delta0(double alpha, double beta, double gamma) {
      double a_tmp;
      double a_tmp_tmp;
      double b_a_tmp;
      double b_a_tmp_tmp;
      double b_delta0_tmp;
      double c_a_tmp;
      double c_delta0_tmp;
      double d_a_tmp;
      double d_delta0_tmp;
      double delta0_tmp;
      double e_a_tmp;
      double e_delta0_tmp;
      double f_a_tmp;
      double f_delta0_tmp;
      double g_a_tmp;
      double h_a_tmp;
      double i_a_tmp;

      a_tmp = alpha - epsilon;
      b_a_tmp = cos(a_tmp);
      c_a_tmp = AC * AC;
      d_a_tmp = AF * AF;
      e_a_tmp = sqrt((c_a_tmp + 2.0 * b_a_tmp * AC * AF) + d_a_tmp);
      f_a_tmp = EF * EF;
      g_a_tmp = beta + zeta;
      b_a_tmp *= 2.0 * AC * AF;
      a_tmp_tmp = asin(AC * sin(a_tmp) / e_a_tmp);
      a_tmp = 2.0 * EF * cos(g_a_tmp + a_tmp_tmp) * e_a_tmp;
      b_a_tmp_tmp = (c_a_tmp + d_a_tmp) + f_a_tmp;
      h_a_tmp = sqrt((b_a_tmp_tmp + b_a_tmp) + a_tmp);
      i_a_tmp = DE - h_a_tmp;
      delta0_tmp = DE * DE;
      g_a_tmp = cos(g_a_tmp);
      b_delta0_tmp = 2.0 * AF * EF * g_a_tmp;
      c_delta0_tmp = 2.0 * AB * AC * cos((lambda - epsilon) + PI * 1.5);
      d_delta0_tmp = 2.0 * d_a_tmp;
      e_delta0_tmp = sqrt((d_a_tmp + 2.0 * g_a_tmp * AF * EF) + f_a_tmp);
      e_delta0_tmp *= 2.0 * AB * cos(((alpha - lambda) - PI_OVER_2) + acos((d_delta0_tmp + 2.0 * EF * g_a_tmp * AF) /(2.0 * AF * e_delta0_tmp)));
      f_delta0_tmp = (AB * AB + d_a_tmp) + delta0_tmp;
      d_delta0_tmp = DE * ((((((d_delta0_tmp + 2.0 * f_a_tmp) + b_delta0_tmp) + c_delta0_tmp) - e_delta0_tmp) + b_a_tmp) + a_tmp);
      a_tmp = (f_delta0_tmp + f_a_tmp) + b_delta0_tmp;
      g_a_tmp = d_delta0_tmp / h_a_tmp;
      return BK - sqrt((((((f_delta0_tmp + DK * DK) + f_a_tmp) - 2.0 * DK * cos(PI_OVER_2 - acos(((((((((d_a_tmp - c_a_tmp) + delta0_tmp) + f_a_tmp) + i_a_tmp * i_a_tmp) +
      b_delta0_tmp) + c_delta0_tmp) - g_a_tmp) - e_delta0_tmp) /(2.0 * i_a_tmp * sqrt((a_tmp - d_delta0_tmp / sqrt((b_a_tmp_tmp + 2.0 * EF * cos((a_tmp_tmp + beta) +zeta) *
      e_a_tmp) + b_a_tmp)) - e_delta0_tmp)))) * sqrt((a_tmp - g_a_tmp) - e_delta0_tmp)) + b_delta0_tmp) - g_a_tmp) - e_delta0_tmp);
}

double calculate_delta1(double alpha, double beta, double gamma) {
    return CD + DE - sqrt(AC * AC + AF * AF + EF * EF + 2 * AC * AF * cos(alpha - epsilon) +
        2 * EF * cos(beta + zeta + asin((AC * sin(alpha - epsilon)) /
        sqrt(AC * AC + 2 * cos(alpha - epsilon) * AC * AF + AF * AF))) *
        sqrt(AC * AC + 2 * cos(alpha - epsilon) * AC * AF + AF * AF));
}


double calculate_delta2(double alpha, double beta, double gamma) {
    double delta2_1 = FG * FG + FH * FH + HI * HI;
    double delta2_2 = cos(gamma + psy + asin((FG * sin(beta - phi)) / sqrt(FG * FG + FH * FH + 2 * FG * FH * cos(beta - phi))));
    double delta2_3 = sqrt(FG * FG + 2 * cos(beta - phi) * FG * FH + FH * FH);
    return sqrt(delta2_1 + 2 * HI * delta2_2 * delta2_3 + 2 * FG * FH * cos(beta - phi)) - GI;
}

// ?? theta2
double calculate_theta2(double alpha, double beta, double gamma) {
    return PI - zeta - beta;
}



// ????
static double rt_powd_snf(double u0, double u1) {
    double d;
    double d1;
    double y;
    if (rtIsNaN(u0) || rtIsNaN(u1)) {
        y = rtNaN;
    } else {
        d = fabs(u0);
        d1 = fabs(u1);
        if (rtIsInf(u1)) {
            if (d == 1.0) {
                y = 1.0;
            } else if (d > 1.0) {
                if (u1 > 0.0) {
                    y = rtInf;
                } else {
                    y = 0.0;
                }
            } else if (u1 > 0.0) {
                y = 0.0;
            } else {
                y = rtInf;
            }
        } else if (d1 == 0.0) {
            y = 1.0;
        } else if (d1 == 1.0) {
            if (u1 > 0.0) {
                y = u0;
            } else {
                y = 1.0 / u0;
            }
        } else if (u1 == 2.0) {
            y = u0 * u0;
        } else if ((u1 == 0.5) && (u0 >= 0.0)) {
            y = sqrt(u0);
        } else if ((u0 < 0.0) && (u1 > floor(u1))) {
            y = rtNaN;
        } else {
            y = pow(u0, u1);
        }
    }
    return y;
}



// ??????? JA
JA_Result calculate_JA(double alpha, double beta, double gamma){
      reslut1_1_tmp_tmp = alpha - epsilon;
      b_reslut1_1_tmp_tmp = sin(reslut1_1_tmp_tmp);
      reslut1_1_tmp_tmp = cos(reslut1_1_tmp_tmp);
      c_reslut1_1_tmp_tmp = beta + zeta;
      d_reslut1_1_tmp_tmp = cos(c_reslut1_1_tmp_tmp);
      a_tmp = AC * AC;
      b_a_tmp = AF * AF;
      a_tmp_tmp = (a_tmp + 2.0 * reslut1_1_tmp_tmp * AC * AF) + b_a_tmp;
      c_a_tmp = sqrt(a_tmp_tmp);
      b_a_tmp_tmp = asin(AC * b_reslut1_1_tmp_tmp / c_a_tmp);
      a_tmp_tmp_tmp = EF * EF;
      c_a_tmp_tmp = (a_tmp + b_a_tmp) + a_tmp_tmp_tmp;
      a_tmp_tmp_tmp_tmp = 2.0 * AC * AF;
      d_a_tmp_tmp = a_tmp_tmp_tmp_tmp * reslut1_1_tmp_tmp;
      e_a_tmp_tmp = c_reslut1_1_tmp_tmp + b_a_tmp_tmp;
      f_a_tmp_tmp = cos(e_a_tmp_tmp);
      g_a_tmp_tmp = 2.0 * EF * f_a_tmp_tmp * c_a_tmp;
      h_a_tmp_tmp = sqrt((c_a_tmp_tmp + d_a_tmp_tmp) + g_a_tmp_tmp);
      d_a_tmp = DE - h_a_tmp_tmp;
      b_a_tmp_tmp = 2.0 * EF * cos((b_a_tmp_tmp + beta) + zeta) * c_a_tmp;
      c_a_tmp_tmp = (c_a_tmp_tmp + b_a_tmp_tmp) + d_a_tmp_tmp;
      e_a_tmp = sqrt(c_a_tmp_tmp);
      f_a_tmp = 2.0 * DE - 2.0 * e_a_tmp;
      g_a_tmp = DE - e_a_tmp;
      h_a_tmp = 2.0 * b_a_tmp;
      i_a_tmp_tmp = (b_a_tmp + 2.0 * d_reslut1_1_tmp_tmp * AF * EF) + a_tmp_tmp_tmp;
      i_a_tmp = sqrt(i_a_tmp_tmp);
      j_a_tmp_tmp = 2.0 * AF * EF;
      j_a_tmp = j_a_tmp_tmp * d_reslut1_1_tmp_tmp;
      b_a_tmp_tmp_tmp = h_a_tmp + 2.0 * EF * d_reslut1_1_tmp_tmp * AF;
      k_a_tmp_tmp = ((alpha - lambda) - 1.5707963267948966) +
                    acos(b_a_tmp_tmp_tmp / (2.0 * AF * i_a_tmp));
      l_a_tmp_tmp = cos(k_a_tmp_tmp);
      k_a_tmp = 2.0 * AB * l_a_tmp_tmp * i_a_tmp;
      m_a_tmp_tmp = 2.0 * AB * AC * cos((lambda - epsilon) + 4.71238898038469);
      h_a_tmp = (h_a_tmp + 2.0 * a_tmp_tmp_tmp) + j_a_tmp;
      l_a_tmp = DE * DE;
      m_a_tmp =
          DE * ((((h_a_tmp - k_a_tmp) + m_a_tmp_tmp) + b_a_tmp_tmp) + d_a_tmp_tmp) /
          e_a_tmp;
      g_a_tmp =
          (((((((g_a_tmp * g_a_tmp - a_tmp) + b_a_tmp) + l_a_tmp) + a_tmp_tmp_tmp) -
             m_a_tmp) +
            j_a_tmp) -
           k_a_tmp) +
          m_a_tmp_tmp;
      reslut1_1_tmp = sin(k_a_tmp_tmp);
      b_reslut1_1_tmp = a_tmp * AF;
      c_reslut1_1_tmp = rt_powd_snf(a_tmp_tmp, 1.5);
      d_reslut1_1_tmp_tmp = sin(e_a_tmp_tmp);
      d_reslut1_1_tmp = 2.0 * EF * d_reslut1_1_tmp_tmp;
      e_reslut1_1_tmp =
          a_tmp_tmp_tmp_tmp * EF * f_a_tmp_tmp * b_reslut1_1_tmp_tmp / c_a_tmp;
      f_a_tmp_tmp =
          AC * cos(alpha - epsilon) /
          sqrt((AC * AC + 2.0 * cos(alpha - epsilon) * AC * AF) + AF * AF);
      e_reslut1_1_tmp_tmp = 2.0 * AB * reslut1_1_tmp;
      f_reslut1_1_tmp = e_reslut1_1_tmp_tmp * i_a_tmp;
      k_a_tmp_tmp = a_tmp_tmp_tmp_tmp * sin(alpha - epsilon);
      g_reslut1_1_tmp =
          (((((2.0 * (AF * AF) + 2.0 * (EF * EF)) +
              2.0 * AF * EF * cos(beta + zeta)) +
             m_a_tmp_tmp) -
            2.0 * AB *
                cos(((alpha - lambda) - 1.5707963267948966) +
                    acos((2.0 * (AF * AF) + 2.0 * EF * cos(beta + zeta) * AF) /
                         (2.0 * AF *
                          sqrt((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) +
                               EF * EF)))) *
                sqrt((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) + EF * EF)) +
           2.0 * AC * AF * cos(alpha - epsilon)) +
          2.0 * EF *
              cos((beta + zeta) +
                  asin(AC * sin(alpha - epsilon) /
                       sqrt((AC * AC + 2.0 * cos(alpha - epsilon) * AC * AF) +
                            AF * AF))) *
              sqrt((AC * AC + 2.0 * cos(alpha - epsilon) * AC * AF) + AF * AF);
      f_reslut1_1_tmp_tmp = rt_powd_snf(c_a_tmp_tmp, 1.5);
      h_reslut1_1_tmp = 2.0 * f_reslut1_1_tmp_tmp;
      reslut1_1_tmp_tmp_tmp_tmp = (AB * AB + b_a_tmp) + l_a_tmp;
      b_a_tmp_tmp = reslut1_1_tmp_tmp_tmp_tmp + a_tmp_tmp_tmp;
      g_reslut1_1_tmp_tmp = ((b_a_tmp_tmp - m_a_tmp) + j_a_tmp) - k_a_tmp;
      i_reslut1_1_tmp = sqrt(g_reslut1_1_tmp_tmp);
      j_reslut1_1_tmp = DE * g_reslut1_1_tmp / e_a_tmp;
      reslut1_1_tmp_tmp_tmp = 2.0 * h_a_tmp_tmp;
      h_reslut1_1_tmp_tmp = 2.0 * DE - reslut1_1_tmp_tmp_tmp;
      k_reslut1_1_tmp = h_reslut1_1_tmp_tmp * i_reslut1_1_tmp;
      l_reslut1_1_tmp =
          2.0 * h_reslut1_1_tmp_tmp * rt_powd_snf(g_reslut1_1_tmp_tmp, 1.5);
      m_reslut1_1_tmp = sqrt(((b_a_tmp_tmp + j_a_tmp) - j_reslut1_1_tmp) - k_a_tmp);
      e_a_tmp_tmp = b_reslut1_1_tmp_tmp * b_reslut1_1_tmp_tmp;
      h_reslut1_1_tmp_tmp = (((((((-a_tmp + b_a_tmp) + l_a_tmp) + a_tmp_tmp_tmp) +
                                d_a_tmp * d_a_tmp) +
                               j_a_tmp) +
                              m_a_tmp_tmp) -
                             j_reslut1_1_tmp) -
                            k_a_tmp;
      j_reslut1_1_tmp =
          1.5707963267948966 - acos(h_reslut1_1_tmp_tmp / k_reslut1_1_tmp);
      n_reslut1_1_tmp =
          (k_a_tmp_tmp +
           d_reslut1_1_tmp *
               (f_a_tmp_tmp + b_reslut1_1_tmp *
                                  (b_reslut1_1_tmp_tmp * b_reslut1_1_tmp_tmp) /
                                  c_reslut1_1_tmp) *
               c_a_tmp /
               sqrt(1.0 - a_tmp * (b_reslut1_1_tmp_tmp * b_reslut1_1_tmp_tmp) /
                              a_tmp_tmp)) +
          e_reslut1_1_tmp;
      o_reslut1_1_tmp =
          DE *
          (((-2.0 * AB *
                 sin(((alpha - lambda) - 1.5707963267948966) +
                     acos((2.0 * (AF * AF) + 2.0 * EF * cos(beta + zeta) * AF) /
                          (2.0 * AF *
                           sqrt((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) +
                                EF * EF)))) *
                 sqrt((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) + EF * EF) +
             k_a_tmp_tmp) +
            d_reslut1_1_tmp *
                (f_a_tmp_tmp + b_reslut1_1_tmp *
                                   (b_reslut1_1_tmp_tmp * b_reslut1_1_tmp_tmp) /
                                   c_reslut1_1_tmp) *
                c_a_tmp /
                sqrt(1.0 - a_tmp * (b_reslut1_1_tmp_tmp * b_reslut1_1_tmp_tmp) /
                               a_tmp_tmp)) +
           e_reslut1_1_tmp) /
          e_a_tmp;
      f_a_tmp_tmp =
          DE *
          ((k_a_tmp_tmp +
            d_reslut1_1_tmp *
                (f_a_tmp_tmp + b_reslut1_1_tmp *
                                   (b_reslut1_1_tmp_tmp * b_reslut1_1_tmp_tmp) /
                                   c_reslut1_1_tmp) *
                c_a_tmp /
                sqrt(1.0 - a_tmp * (b_reslut1_1_tmp_tmp * b_reslut1_1_tmp_tmp) /
                               a_tmp_tmp)) +
           e_reslut1_1_tmp) *
          g_reslut1_1_tmp / h_reslut1_1_tmp;
      b_a_tmp_tmp = (o_reslut1_1_tmp + f_reslut1_1_tmp) - f_a_tmp_tmp;
      p_reslut1_1_tmp = f_a_tmp * f_a_tmp;
      q_reslut1_1_tmp = DK * cos(j_reslut1_1_tmp);
      r_reslut1_1_tmp = p_reslut1_1_tmp * e_a_tmp * i_reslut1_1_tmp;
      j_reslut1_1_tmp = 2.0 * DK * sin(j_reslut1_1_tmp);
      p_reslut1_1_tmp *= g_reslut1_1_tmp_tmp;
      k_a_tmp_tmp =
          2.0 * sqrt((((((reslut1_1_tmp_tmp_tmp_tmp + DK * DK) + a_tmp_tmp_tmp) -
                        m_a_tmp) +
                       j_a_tmp) -
                      k_a_tmp) -
                     2.0 * DK *
                         cos(1.5707963267948966 -
                             acos(g_a_tmp / (f_a_tmp * i_reslut1_1_tmp))) *
                         i_reslut1_1_tmp);
      result1_1 =
          ((((-(DE * (((-2.0 * AB * reslut1_1_tmp * i_a_tmp +
                        a_tmp_tmp_tmp_tmp * b_reslut1_1_tmp_tmp) +
                       d_reslut1_1_tmp *
                           (AC * reslut1_1_tmp_tmp / c_a_tmp +
                            b_reslut1_1_tmp * e_a_tmp_tmp / c_reslut1_1_tmp) *
                           c_a_tmp / sqrt(1.0 - a_tmp * e_a_tmp_tmp / a_tmp_tmp)) +
                      e_reslut1_1_tmp)) /
                  e_a_tmp -
              f_reslut1_1_tmp) +
             DE * n_reslut1_1_tmp *
                 ((((h_a_tmp + m_a_tmp_tmp) - k_a_tmp) + d_a_tmp_tmp) +
                  g_a_tmp_tmp) /
                 h_reslut1_1_tmp) +
            q_reslut1_1_tmp * b_a_tmp_tmp / i_reslut1_1_tmp) +
           j_reslut1_1_tmp *
               ((-(((o_reslut1_1_tmp + d_a_tmp * n_reslut1_1_tmp / e_a_tmp) +
                    f_reslut1_1_tmp) -
                   f_a_tmp_tmp) /
                     k_reslut1_1_tmp +
                 b_a_tmp_tmp * h_reslut1_1_tmp_tmp / l_reslut1_1_tmp) +
                n_reslut1_1_tmp * h_reslut1_1_tmp_tmp / r_reslut1_1_tmp) *
               m_reslut1_1_tmp / sqrt(1.0 - g_a_tmp * g_a_tmp / p_reslut1_1_tmp)) /
          k_a_tmp_tmp;
      b_a_tmp_tmp = sin(c_reslut1_1_tmp_tmp);
      reslut1_1_tmp = j_a_tmp_tmp * b_a_tmp_tmp;
      m_a_tmp = 4.0 * b_a_tmp * i_a_tmp_tmp;
      f_reslut1_1_tmp = 2.0 * AB * AF * EF * l_a_tmp_tmp * b_a_tmp_tmp / i_a_tmp;
      c_a_tmp_tmp = EF * sin(beta + zeta);
      h_reslut1_1_tmp =
          2.0 * AB *
          sin(((alpha - lambda) - 1.5707963267948966) +
              acos(
                  (2.0 * (AF * AF) + 2.0 * EF * cos(beta + zeta) * AF) /
                  (2.0 * AF *
                   sqrt((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) + EF * EF)))) *
          (c_a_tmp_tmp /
               sqrt((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) + EF * EF) -
           c_a_tmp_tmp * (2.0 * (AF * AF) + 2.0 * EF * cos(beta + zeta) * AF) /
               (2.0 *
                rt_powd_snf((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) + EF * EF,
                            1.5))) *
          sqrt((AF * AF + 2.0 * cos(beta + zeta) * AF * EF) + EF * EF);
      e_reslut1_1_tmp =
          2.0 * AF * EF * sin(beta + zeta) +
          2.0 * EF *
              sin((beta + zeta) +
                  asin(AC * sin(alpha - epsilon) /
                       sqrt((AC * AC + 2.0 * cos(alpha - epsilon) * AC * AF) +
                            AF * AF))) *
              sqrt((AC * AC + 2.0 * cos(alpha - epsilon) * AC * AF) + AF * AF);
      l_a_tmp = DE * EF * d_reslut1_1_tmp_tmp * c_a_tmp * g_reslut1_1_tmp /
                f_reslut1_1_tmp_tmp;
      o_reslut1_1_tmp = -2.0 * AF * EF * sin(beta + zeta);
      reslut1_1_tmp_tmp_tmp_tmp = d_reslut1_1_tmp * c_a_tmp;
      g_reslut1_1_tmp_tmp =
          h_reslut1_1_tmp / sqrt(1.0 - b_a_tmp_tmp_tmp * b_a_tmp_tmp_tmp / m_a_tmp);
      result1_2 =
          ((((((reslut1_1_tmp -
                DE *
                    (((reslut1_1_tmp + reslut1_1_tmp_tmp_tmp_tmp) -
                      e_reslut1_1_tmp_tmp *
                          (EF * b_a_tmp_tmp / i_a_tmp -
                           EF * sin(beta + zeta) * b_a_tmp_tmp_tmp /
                               (2.0 * rt_powd_snf(i_a_tmp_tmp, 1.5))) *
                          i_a_tmp /
                          sqrt(1.0 - b_a_tmp_tmp_tmp * b_a_tmp_tmp_tmp / m_a_tmp)) -
                     f_reslut1_1_tmp) /
                    e_a_tmp) +
               q_reslut1_1_tmp *
                   ((((-2.0 * AF * EF * b_a_tmp_tmp +
                       DE *
                           ((e_reslut1_1_tmp -
                             h_reslut1_1_tmp /
                                 sqrt(1.0 - b_a_tmp_tmp_tmp * b_a_tmp_tmp_tmp /
                                                m_a_tmp)) -
                            f_reslut1_1_tmp) /
                           e_a_tmp) +
                      g_reslut1_1_tmp_tmp) -
                     l_a_tmp) +
                    f_reslut1_1_tmp) /
                   i_reslut1_1_tmp) -
              g_reslut1_1_tmp_tmp) +
             j_reslut1_1_tmp *
                 ((-(((((o_reslut1_1_tmp +
                         DE *
                             ((e_reslut1_1_tmp -
                               h_reslut1_1_tmp /
                                   sqrt(1.0 - b_a_tmp_tmp_tmp * b_a_tmp_tmp_tmp /
                                                  m_a_tmp)) -
                              f_reslut1_1_tmp) /
                             e_a_tmp) +
                        h_reslut1_1_tmp /
                            sqrt(1.0 -
                                 b_a_tmp_tmp_tmp * b_a_tmp_tmp_tmp / m_a_tmp)) +
                       d_reslut1_1_tmp * d_a_tmp * c_a_tmp / e_a_tmp) -
                      l_a_tmp) +
                     f_reslut1_1_tmp) /
                       k_reslut1_1_tmp +
                   ((((o_reslut1_1_tmp +
                       DE *
                           ((e_reslut1_1_tmp -
                             h_reslut1_1_tmp /
                                 sqrt(1.0 - b_a_tmp_tmp_tmp * b_a_tmp_tmp_tmp /
                                                m_a_tmp)) -
                            f_reslut1_1_tmp) /
                           e_a_tmp) +
                      h_reslut1_1_tmp /
                          sqrt(1.0 - b_a_tmp_tmp_tmp * b_a_tmp_tmp_tmp / m_a_tmp)) -
                     l_a_tmp) +
                    f_reslut1_1_tmp) *
                       h_reslut1_1_tmp_tmp / l_reslut1_1_tmp) +
                  reslut1_1_tmp_tmp_tmp_tmp * h_reslut1_1_tmp_tmp /
                      r_reslut1_1_tmp) *
                 m_reslut1_1_tmp /
                 sqrt(1.0 - g_a_tmp * g_a_tmp / p_reslut1_1_tmp)) +
            l_a_tmp) -
           f_reslut1_1_tmp) /
          k_a_tmp_tmp;
      result1_3 = 0.0;

      result2_1 = n_reslut1_1_tmp / reslut1_1_tmp_tmp_tmp;

      result2_2 = EF * d_reslut1_1_tmp_tmp * c_a_tmp / h_a_tmp_tmp;

      result2_3 = 0.0;

      result3_1 = 0.0;

      c_a_tmp_tmp = beta - phi;
      reslut1_1_tmp_tmp_tmp_tmp = sin(c_a_tmp_tmp);
      g_reslut1_1_tmp_tmp = 2.0 * FG * FH;
      e_a_tmp_tmp = cos(c_a_tmp_tmp);
      h_reslut1_1_tmp = FG * FG;
      c_a_tmp_tmp = FH * FH;
      b_a_tmp_tmp = h_reslut1_1_tmp + c_a_tmp_tmp;
      k_a_tmp_tmp = g_reslut1_1_tmp_tmp * e_a_tmp_tmp;
      e_reslut1_1_tmp = b_a_tmp_tmp + k_a_tmp_tmp;
      f_a_tmp_tmp = sqrt(e_reslut1_1_tmp);
      l_a_tmp = FG * reslut1_1_tmp_tmp_tmp_tmp;
      o_reslut1_1_tmp = gamma + psy;
      f_reslut1_1_tmp = o_reslut1_1_tmp + asin(l_a_tmp / f_a_tmp_tmp);
      m_a_tmp = sqrt((h_reslut1_1_tmp + 2.0 * e_a_tmp_tmp * FG * FH) + c_a_tmp_tmp);
      reslut1_1_tmp = sin(f_reslut1_1_tmp);
      b_a_tmp_tmp =
          sqrt(((b_a_tmp_tmp + HI * HI) +
                2.0 * HI * cos(o_reslut1_1_tmp + asin(l_a_tmp / m_a_tmp)) *
                    f_a_tmp_tmp) +
               k_a_tmp_tmp);
      c_a_tmp_tmp = reslut1_1_tmp_tmp_tmp_tmp * reslut1_1_tmp_tmp_tmp_tmp;

      result3_2 =
          -((g_reslut1_1_tmp_tmp * reslut1_1_tmp_tmp_tmp_tmp +
             2.0 * HI * reslut1_1_tmp *
                 (FG * e_a_tmp_tmp / f_a_tmp_tmp +
                  h_reslut1_1_tmp * FH * c_a_tmp_tmp /
                      rt_powd_snf(e_reslut1_1_tmp, 1.5)) *
                 m_a_tmp /
                 sqrt(-(h_reslut1_1_tmp * c_a_tmp_tmp) / e_reslut1_1_tmp + 1.0)) +
            g_reslut1_1_tmp_tmp * HI * reslut1_1_tmp_tmp_tmp_tmp *
                cos(f_reslut1_1_tmp) / f_a_tmp_tmp) /
          (2.0 * b_a_tmp_tmp);

      result3_3 = -(HI * reslut1_1_tmp * m_a_tmp) / b_a_tmp_tmp;
      result4_1 = 0.0;
      result4_2 = -1.0;
      result4_3 = 0.0;

      res.result1[0] = result1_1;
      res.result1[1] = result1_2;
      res.result1[2] = result1_3;
      res.result2[0] = result2_1;
      res.result2[1] = result2_2;
      res.result2[2] = result2_3;
      res.result3[0] = result3_1;
      res.result3[1] = result3_2;
      res.result3[2] = result3_3;

      res.result4[0] = result4_1;
      res.result4[1] = result4_2;
      res.result4[2] = result4_3;

      return res;
}

// ??JT??
void compute_JT(double JT[3][3], double h1, double h2, double AF, double beta, double gamma, double rho, double FH, double px, double py) {
    JT[0][0] = h1;
    JT[0][1] = 0;
    JT[0][2] = 0;
    JT[1][0] = AF * cos(beta) + h2;
    JT[1][1] = h2;
    JT[1][2] = 0;
    JT[2][0] = AF * cos(beta + gamma - rho) + FH * cos(gamma - rho) + px * cos(rho) - py * sin(rho);
    JT[2][1] = FH * cos(gamma - rho) + px * cos(rho) - py * sin(rho);
    JT[2][2] = px * cos(rho) - py * sin(rho);
}

// ?? tao??
void compute_tao(double T[1][4], double F0, double judge_1, double judge_2, double k1, double delta1, double F_01, double k2, double delta2, double F_02, double theta2, double theta2_0, double k3, double F_03) {
    T[0][0] = F0;
    if (judge_1 == 1) {
        T[0][1] = 0;
    }else {
        T[0][1] = -(k1 * delta1 + F_01);
    }
    if (judge_2 == 1) {
        T[0][2] = 0;
    }else {
        T[0][2] = -(k2 * delta2 + F_02);
    }
    T[0][3] = (theta2 - theta2_0) * (k3 + F_03);

}


void matrix_multiply(double *A, double *B, double *C, int m, int n, int p) {
    for ( i = 0; i < m; i++) {
        for ( j = 0; j < p; j++) {
            C[i*p + j] = 0.0;
            for ( k = 0; k < n; k++) {
                C[i*p + j] += A[i*n + k] * B[k*p + j];
            }
        }
    }
}


int inverse_3x3(double *A, double *invA) {
    
    double det = A[0]*(A[4]*A[8] - A[5]*A[7]) -
                 A[1]*(A[3]*A[8] - A[5]*A[6]) +
                 A[2]*(A[3]*A[7] - A[4]*A[6]);

    if (det == 0.0) {
        printf("False\r\n");
        return 0; 
    }

   
    invA[0] = (A[4]*A[8] - A[5]*A[7]) / det;
    invA[1] = (A[2]*A[7] - A[1]*A[8]) / det;
    invA[2] = (A[1]*A[5] - A[2]*A[4]) / det;

    invA[3] = (A[5]*A[6] - A[3]*A[8]) / det;
    invA[4] = (A[0]*A[8] - A[2]*A[6]) / det;
    invA[5] = (A[2]*A[3] - A[0]*A[5]) / det;

    invA[6] = (A[3]*A[7] - A[4]*A[6]) / det;
    invA[7] = (A[1]*A[6] - A[0]*A[7]) / det;
    invA[8] = (A[0]*A[4] - A[1]*A[3]) / det;

    return 1; 
}

double calculation_beta(double alpha, double AC, double AF, double CE, double EF, double zeta, double epsilon) {
    double CF, EFC, AFC, beta;

    CF = sqrt(AC * AC + AF * AF - 2 * AC * AF * cos(PI - epsilon + alpha));

    EFC = acos((EF * EF + CF * CF - CE * CE) / (2 * EF * CF));
    AFC = acos((AF * AF + CF * CF - AC * AC) / (2 * AF * CF));

    beta = PI - (EFC - AFC) - zeta;

    return beta;
}

double calculation_gamma(double FG, double FH, double GI, double HI, double psy, double phi, double beta) {
    double GH, GHI, GHF, gamma;

    GH = sqrt(FG * FG + FH * FH - 2 * FG * FH * cos(PI - phi + beta));
    GHI = acos((GH * GH + HI * HI - GI * GI) / (2 * GH * HI));
    GHF = acos((GH * GH + FH * FH - FG * FG) / (2 * GH * FH));
    gamma = PI - (GHI - GHF) - psy;

    return gamma;
}

int main_calculate(double F_01, double F_02, double F_03, double alpha, double beta, double gamma, double rho)
{
	
	//delta0, delta1, delta2, theta2
    delta0 = calculate_delta0(alpha, beta, gamma);
    delta1 = calculate_delta1(alpha, beta, gamma);
    delta2 = calculate_delta2(alpha, beta, gamma);
    theta2 = calculate_theta2(alpha, beta, gamma);
    printf("delta0 = %f\r\n", delta0);
    printf("delta1 = %f\r\n", delta1);
    printf("delta2 = %f\r\n", delta2);
    printf("theta2 = %f\r\n", theta2);

    results = calculate_JA(alpha, beta, gamma);
    h1 = 1.0;
    h2 = 2.0;
    px = 16.5000;
    py = -1.5000;
    judge_1 = 1; 
    judge_2 = 1 ; 

    printf("\n JT matrix is :\r\n");
    compute_JT(JT, h1, h2, AF, beta, gamma, rho, FH, px, py);
    for ( i = 0; i < 3; i++) {
        for ( j = 0; j < 3; j++) {
            printf("%f ", JT[i][j]);
        }
        printf("\r\n");
    }

    printf("\n");
    beta_o = calculation_beta(alpha, AC, AF, CE, EF, zeta, epsilon);
    printf("the result of beta_0 is : %f\r\n", beta_o);
    if (beta > beta_o) {
        printf("judge_1 = 0");
        judge_1 = 0;
    }else {
        printf("judge_1 = 1");
        judge_1 = 1;
    }
    printf("\r\n");

    gamma_o = calculation_gamma(FG, FH, GI, HI, psy, phi, beta);
    printf("the result of gamma_o is : %f\r\n", gamma_o);
    if (gamma < gamma_o) {
        printf("judge_2 = 0");
        judge_2 = 0;
    }else {
        printf("judge_2 = 1");
        judge_2 = 1;
    }
    printf("\r\n");

    compute_tao(T, F0, judge_1, judge_2, k1, delta1, F_01, k2, delta2, F_02, theta2, theta2_0, k3, F_03);
    printf("\n matrix tao(T) is :\r\n");
    for ( j = 0; j < 4; j++) {
        printf("%f ", T[0][j]);
    }
    printf("\r\n");

    JA[0][0] = results.result1[0];
    JA[0][1] = results.result1[1];
    JA[0][2] = results.result1[2];
    JA[1][0] = results.result2[0];
    JA[1][1] = results.result2[1];
    JA[1][2] = results.result2[2];
    JA[2][0] = results.result3[0];
    JA[2][1] = results.result3[1];
    JA[2][2] = results.result3[2];
    JA[3][0] = results.result4[0];
    JA[3][1] = results.result4[1];
    JA[3][2] = results.result4[2];

    printf("\n matrix JA is :\r\n");
    for ( i = 0; i < 4; i++) {
        for ( j = 0; j < 3; j++) {
            printf("%f ", JA[i][j]);
        }
        printf("\r\n");
    }
  
    matrix_multiply(&T[0][0], &JA[0][0], &temp[0][0], 1, 4, 3);
   
    if (!inverse_3x3(&JT[0][0], &invJT[0][0])) {
        return -1; 
    }
   
    matrix_multiply(&temp[0][0], &invJT[0][0], &F[0][0], 1, 3, 3);
    printf("\n result matrix F:\r\n");
    printf("[ %8.4f, %8.4f, %8.4f ]\r\n", F[0][0], F[0][1], F[0][2]);
		return 0;
}	

