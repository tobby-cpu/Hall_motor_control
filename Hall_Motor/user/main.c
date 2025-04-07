
#include "stm32f10x.h"
#include <string.h>
#include <stdlib.h>
#include "MyProject.h"
#include "user.h"
/******************************************************************************/
//Dalian University of Technology Yunxin Zhao (Add an angle control loop and three-motor coordinated control to the existing closed-loop control system.)
/******************************************************************************/
#define LED_blink    GPIOC->ODR^=(1<<13)
#define PI 3.14159265358979323846
#define rtIsNaN(x) (isnan(x))
#define rtIsInf(x) (isinf(x))
#define rtNaN (NAN)
#define rtInf (INFINITY)
#define MATRIX_ELEMENT(m, i, j) m[i][j]


double JT[3][3], h1, h2, rho, px, py, udge_1,judge_1,judge_2;     //是耦合，0是自适应 judge_2 = 1是耦合，0是自适应
const double PI_OVER_2;

// 定义输入变量
double alpha, beta, gamma;

// 定义常量
double zeta, epsilon, lambda, AF, AC, AB, EF, DE, CD, DK, BK, phi, FH, FG, psy, HI, GI, CE;
double F_0, F_1, F_2;
double JT[3][3];
double tao[1][4];
int i, j, k, m;  
// 定义输出变量
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
double F_01 = 1;
double F_02 = 1;
double F_03 = 1;
double theta2_0 = 2.3562;

   // 中间结果和最终结果
double temp[1][3];
double invJT[3][3];
double F[1][3];

double beta_o;
double gamma_o;

JA_Result results;
/******************************************************************************/
//声明函数
void commander_run(void);
double calculate_delta0(void);
double calculate_delta1(void);
double calculate_delta2(void);
double calculate_theta2(void);
static double rt_powd_snf(double u0, double u1);
JA_Result calculate_JA(void);
void matrix_multiply(double *A, double *B, double *C, int m, int n, int p) ;
void compute_JT(double JT[3][3], double h1, double h2, double AF, double beta, double gamma, double rho, double FH, double px, double py);
void compute_tao(double T[1][4], double F0, double judge_1, double judge_2, double k1, double delta1, double F_01, double k2, double delta2, 
	double F_02, double theta2, double theta2_0, double k3, double F_03);
int inverse_3x3(double *A, double *invA);
double calculation_beta(double alpha, double AC, double AF, double CE, double EF, double zeta, double epsilon);
double calculation_gamma(double FG, double FH, double GI, double HI, double psy, double phi, double beta);
/******************************************************************************/
MOTORController M1,M2;

long timecntr_pre=0;
long time_cntr=0;
/******************************************************************************/
//us计时，每71.5分钟溢出循环一次
uint32_t timecount(void)
{
	uint32_t  diff,now_us;
	
	now_us = _micros();    //0xFFFFFFFF=4294967295 us=71.5分钟
	if(now_us>=timecntr_pre)diff = now_us - timecntr_pre;   //us
	else
		diff = 0xFFFFFFFF - timecntr_pre + now_us;
	timecntr_pre = now_us;
	
	return diff;
}
/******************************************************************************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA,GPIOB,GPIOC,AFIO;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         //PC13是LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  //速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //对选中管脚初始化
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);                 //上电点亮LED
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_9;          //使能,PB9是motor1,PB3是motor2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_9);                  //低电平解除,Motor_init()中使能
}
/******************************************************************************/
int main(void)
{
	GPIO_Config();
	uart_init(115200);
	strcpy(M1.str, "M1");
	strcpy(M2.str, "M2");
	TIM2_PWM_Init();    //M1_PWM
	TIM3_PWM_Init();    //M2_PWM
	systick_CountInit();      
	printf("Double HALL F1\r\n");
	
	delay_ms(1000);               //Wait for the system to stabilize
	//InlineCurrentSense(&M1,0.001,50,ADC_Channel_3,ADC_Channel_4,NOT_SET);    //SimpleMotor//采样电阻阻值，运放倍数，A相，B相，C相
	//InlineCurrentSense(&M2,0.001,50,ADC_Channel_5,ADC_Channel_9,NOT_SET);
	//InlineCurrentSense_Init(&M1); //ADC初始化和偏置电压校准
	//InlineCurrentSense_Init(&M2);
	configureADC();
	LPF_init(&M1);                //LPF参数初始化，可进入函数初始化参数
	LPF_init(&M2);
	PID_init(&M1);                //PID参数初始化，可进入函数初始化参数
	PID_init(&M2);
	
	voltage_power_supply=24;      //V,两个电机的共同参数
	
	//所有PID参数都并非最佳参数，请根据电机自行匹配
	M1.pole_pairs=2;              //电机极对数，磁铁的对数
	M1.voltage_sensor_align=2.5;  //V alignSensor() use it，大功率电机设置的值小一点比如0.5-1，小电机设置的大一点比如2-3
	M1.voltage_limit=10;           //V，最大值需小于12/1.732=6.9
	M1.velocity_limit=20;         
	M1.current_limit=50;          
	M1.torque_controller=Type_voltage;  //Type_dc_current;//  Type_foc_current;  //Type_voltage;
	M1.controller=Type_angle_openloop;  //Type_torque;  //Type_velocity;  //Type_angle; 
	M1.PID_d.P=0.6;               
	M1.PID_d.I=0.5;                 
	M1.PID_q.P=0.6;
	M1.PID_q.I=0.5;
	M1.PID_vel.P=0.1;             
	M1.PID_vel.I=0.6;
	M1.P_ang.P=20;                
	M1.PID_vel.output_ramp=50;    
	M1.LPF_vel.Tf=0.05;           
	M1.target=0;
	
	M2.pole_pairs=2;              //电机极对数
	M2.voltage_sensor_align=1.5;    //V alignSensor() use it，大功率电机设置的值小一点比如0.5-1，小电机设置的大一点比如2-3
	M2.voltage_limit=13;           //V，最大值需小于12/1.732=6.9
	M2.velocity_limit=20;         
	M2.current_limit=50;          
	M2.torque_controller=Type_voltage;  //Type_dc_current;//  Type_foc_current;  //Type_voltage;
	//M2.controller=Type_torque;  //Type_torque;  //Type_velocity;  //Type_angle; 
	M2.PID_d.P=0.6;             
	M2.PID_d.I=0.5;                
	M2.PID_q.P=0.6;
	M2.PID_q.I=0.5;
	M2.PID_vel.P=0.1;             
	M2.PID_vel.I=0.6;
	M2.P_ang.P = 20;							
	M2.PID_vel.output_ramp=50;    
	M2.LPF_vel.Tf=0.05;         
	M2.target=0;
	
	Hall_init();                  
	Motor_init(&M1);
	Motor_init(&M2);
	Motor_initFOC(&M1,4.188,CW);
	Motor_initFOC(&M2, 0, UNKNOWN); //(&M2, 2.094, CW); 
  printf("Motor ready.\r\n");
	//***************************************************//
	zeta = 0.7854;
  epsilon = 3.1416;
  lambda = 0;
  AF = 45;
  AC = 14.2891;
  AB = 5.8710;
  EF = 11.5000;
  DE = 24.4060;
  CD = 15.2790;
  DK = 2.8000;
  BK = 28.6700;
  phi = 156.5722;
  FH = 28;
  FG = 5.2818;
  psy = 0.7854;
  HI = 11;
  GI = 32.4708;
  CE = 39.6850;

    // 输入变量,角度传感值
	//**************************
  alpha = 0.3491;
  beta = 0.3491;
  gamma = 0;
	//**************************
  // double
  F_0 = 0;
  F_1 = 0;
  F_2 = 0;
	// 计算 delta0, delta1, delta2, theta2
    delta0 = calculate_delta0();
    delta1 = calculate_delta1();
    delta2 = calculate_delta2();
    theta2 = calculate_theta2();
    printf("delta0 = %f\r\n", delta0);
    printf("delta1 = %f\r\n", delta1);
    printf("delta2 = %f\r\n", delta2);
    printf("theta2 = %f\r\n", theta2);

    results = calculate_JA();
    // h1, h2实时输入
    h1 = 1.0;
    h2 = 2.0;
    rho = 8.5000;
    px = 16.5000;
    py = -1.5000;
    judge_1 = 1; // judge_1 = 1是耦合，0是自适应
    judge_2 = 1 ; // judge_2 = 1是耦合，0是自适应

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

 

    // 第一步: 计算 T * JA
    matrix_multiply(&T[0][0], &JA[0][0], &temp[0][0], 1, 4, 3);
    // 第二步: 计算 JT 的逆矩阵
    if (!inverse_3x3(&JT[0][0], &invJT[0][0])) {
        return -1; // 矩阵不可逆，退出
    }
    // 第三步: 计算 temp * invJT
    matrix_multiply(&temp[0][0], &invJT[0][0], &F[0][0], 1, 3, 3);

    // 打印结果
    printf("\n result matrix F:\r\n");
    printf("[ %8.4f, %8.4f, %8.4f ]\r\n", F[0][0], F[0][1], F[0][2]);

	//**********************************************************//
	while(1)
	{
		
		time_cntr +=timecount();
		if(time_cntr>=5000)  //us
		{
			time_cntr=0;
			LED_blink;
		
		}
		move(&M1, M1.target);
		loopFOC(&M1,ADC_Channel_3);
		move(&M2, M2.target);
		loopFOC(&M2,ADC_Channel_4);
		commander_run();
	}
}

void commander_run(void)
{
	if((USART_RX_STA&0x8000)!=0)
	{
		switch(USART_RX_BUF[0])
			{ 
			case 'Z'://切换模式
				switch(USART_RX_BUF[1])
				{
					case'A':
						M1.controller = Type_torque;
						M1.target = 0;
						printf("Type_torque!\r\n");
						break;
					case 'B':
						M1.controller = Type_angle_openloop;
						M1.target = 0;
						printf("Type_angle_openloop!\r\n");
						break;
				}
			  break;
			case 'H':
				printf("Hello World!\r\n");
				break;
			case 'A':   //A6.28
				M1.target=atof((const char *)(USART_RX_BUF+1));
				printf("A=%.4f\r\n", M1.target);
				break;
			case 'B':   //B6.28
				M2.target=atof((const char *)(USART_RX_BUF+1));
				printf("B=%.4f\r\n", M2.target);
				break;
			case 'T':   //T6.28
				M1.target=atof((const char *)(USART_RX_BUF+1));
				M2.target=M1.target;
				printf("T=%.4f\r\n", M1.target);
				break;
			
			case 'M':  //M1，简单模仿官方通信协议
				switch(USART_RX_BUF[1])
				{
					case 'P':   //设置电流环的P参数,MP1
						M1.PID_d.P=atof((const char *)(USART_RX_BUF+2));
					  M1.PID_q.P=M1.PID_d.P;
					  printf("M1.current.P=%.4f\r\n", M1.PID_d.P);
					  break;
					case 'I':   //设置电流环的I参数,MI0.02
						M1.PID_d.I=atof((const char *)(USART_RX_BUF+2));
					  M1.PID_q.I=M1.PID_d.I;
					  printf("M1.current.I=%.4f\r\n", M1.PID_d.I);
					  break;
					case 'V':   //MV  读实时速度
					  printf("M1.vel=%.2f\r\n", M1.shaft_velocity);
						break;
					case 'A':   //MA  读绝对角度
					  printf("M1.ang=%.2f\r\n", M1.shaft_angle);
						break;
					case 'G':  //MG读取角度传感器的值
						printf("M1.new_ang=%.2f\r\n", M1.shaft_angle_new);
						break;
				}
				break;
			case 'N':  //M2
				switch(USART_RX_BUF[1])
				{
					case 'P':
						M2.PID_d.P=atof((const char *)(USART_RX_BUF+2));
					  M2.PID_q.P=M2.PID_d.P;
					  printf("M2.current.P=%.4f\r\n", M2.PID_d.P);
					  break;
					case 'I':
						M2.PID_d.I=atof((const char *)(USART_RX_BUF+2));
					  M2.PID_q.I=M2.PID_d.I;
					  printf("M2.current.I=%.4f\r\n", M2.PID_d.I);
					  break;
					case 'V':   //MV  读实时速度
					  printf("M2.vel=%.2f\r\n", M2.shaft_velocity);
						break;
					case 'A':   //MA  读绝对角度
					  printf("M2.ang=%.2f\r\n", M2.shaft_angle);
						break;
				}
				break;
		}
		USART_RX_STA=0;
	}
}

// 计算 delta0
double calculate_delta0() {
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

// 计算 delta1
double calculate_delta1() {
    return CD + DE - sqrt(AC * AC + AF * AF + EF * EF + 2 * AC * AF * cos(alpha - epsilon) +
        2 * EF * cos(beta + zeta + asin((AC * sin(alpha - epsilon)) /
        sqrt(AC * AC + 2 * cos(alpha - epsilon) * AC * AF + AF * AF))) *
        sqrt(AC * AC + 2 * cos(alpha - epsilon) * AC * AF + AF * AF));
}

// 计算 delta2
double calculate_delta2() {
    double delta2_1 = FG * FG + FH * FH + HI * HI;
    double delta2_2 = cos(gamma + psy + asin((FG * sin(beta - phi)) / sqrt(FG * FG + FH * FH + 2 * FG * FH * cos(beta - phi))));
    double delta2_3 = sqrt(FG * FG + 2 * cos(beta - phi) * FG * FH + FH * FH);
    return sqrt(delta2_1 + 2 * HI * delta2_2 * delta2_3 + 2 * FG * FH * cos(beta - phi)) - GI;
}

// 计算 theta2
double calculate_theta2() {
    return PI - zeta - beta;
}



// 实现函数
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



// 计算雅可比矩阵 JA
JA_Result calculate_JA(){
      JA_Result res;
      double a_tmp;
      double a_tmp_tmp;
      double a_tmp_tmp_tmp;
      double a_tmp_tmp_tmp_tmp;
      double b_a_tmp;
      double b_a_tmp_tmp;
      double b_a_tmp_tmp_tmp;
      double b_reslut1_1_tmp;
      double b_reslut1_1_tmp_tmp;
      double c_a_tmp;
      double c_a_tmp_tmp;
      double c_reslut1_1_tmp;
      double c_reslut1_1_tmp_tmp;
      double d_a_tmp;
      double d_a_tmp_tmp;
      double d_reslut1_1_tmp;
      double d_reslut1_1_tmp_tmp;
      double e_a_tmp;
      double e_a_tmp_tmp;
      double e_reslut1_1_tmp;
      double e_reslut1_1_tmp_tmp;
      double f_a_tmp;
      double f_a_tmp_tmp;
      double f_reslut1_1_tmp;
      double f_reslut1_1_tmp_tmp;
      double g_a_tmp;
      double g_a_tmp_tmp;
      double g_reslut1_1_tmp;
      double g_reslut1_1_tmp_tmp;
      double h_a_tmp;
      double h_a_tmp_tmp;
      double h_reslut1_1_tmp;
      double h_reslut1_1_tmp_tmp;
      double i_a_tmp;
      double i_a_tmp_tmp;
      double i_reslut1_1_tmp;
      double j_a_tmp;
      double j_a_tmp_tmp;
      double j_reslut1_1_tmp;
      double k_a_tmp;
      double k_a_tmp_tmp;
      double k_reslut1_1_tmp;
      double l_a_tmp;
      double l_a_tmp_tmp;
      double l_reslut1_1_tmp;
      double m_a_tmp;
      double m_a_tmp_tmp;
      double m_reslut1_1_tmp;
      double n_reslut1_1_tmp;
      double o_reslut1_1_tmp;
      double p_reslut1_1_tmp;
      double q_reslut1_1_tmp;
      double r_reslut1_1_tmp;
      double reslut1_1_tmp;
      double reslut1_1_tmp_tmp;
      double reslut1_1_tmp_tmp_tmp;
      double reslut1_1_tmp_tmp_tmp_tmp;
      double result1_1, result1_2, result1_3;
      double result2_1, result2_2, result2_3;
      double result3_1, result3_2, result3_3;
      double result4_1, result4_2, result4_3;

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

// 计算JT矩阵
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

// 计算 tao矩阵
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

// 矩阵乘法：A(m×n) * B(n×p) = C(m×p)
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

// 计算3×3矩阵的逆矩阵
int inverse_3x3(double *A, double *invA) {
    // 计算行列式
    double det = A[0]*(A[4]*A[8] - A[5]*A[7]) -
                 A[1]*(A[3]*A[8] - A[5]*A[6]) +
                 A[2]*(A[3]*A[7] - A[4]*A[6]);

    if (det == 0.0) {
        printf("错误：JT矩阵不可逆（行列式为0）\n");
        return 0; // 失败
    }

    // 计算伴随矩阵并除以行列式
    invA[0] = (A[4]*A[8] - A[5]*A[7]) / det;
    invA[1] = (A[2]*A[7] - A[1]*A[8]) / det;
    invA[2] = (A[1]*A[5] - A[2]*A[4]) / det;

    invA[3] = (A[5]*A[6] - A[3]*A[8]) / det;
    invA[4] = (A[0]*A[8] - A[2]*A[6]) / det;
    invA[5] = (A[2]*A[3] - A[0]*A[5]) / det;

    invA[6] = (A[3]*A[7] - A[4]*A[6]) / det;
    invA[7] = (A[1]*A[6] - A[0]*A[7]) / det;
    invA[8] = (A[0]*A[4] - A[1]*A[3]) / det;

    return 1; // 成功
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
