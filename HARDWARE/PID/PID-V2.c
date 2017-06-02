//C语言实现PID算法
#include <stdio.h>
#include "PID-V2.h"
#include <math.h>
#include <stdlib.h>


pid_type warm,*pid;//pid声明
int process_point, set_point,dead_band;
float p_gain, i_gain, d_gain, integral_val,new_integ;//p_gain p增益

/*------------------------------------------------------------------------
　　pid_init
　　
　　DESCRIPTION This function initializes the pointers in the _pid structure
　　to the process variable and the setpoint. *pv and *sp are
　　integer pointers.
　　------------------------------------------------------------------------*/
void pid_init(pid_type *warm, int process_point, int set_point)
{
	pid_type *pid;
		
	pid = warm;
	pid->pv = process_point;
	pid->sp = set_point;
}

/*------------------------------------------------------------------------
pid_tune
　　
DESCRIPTION Sets the proportional gain (p_gain), integral gain (i_gain),
derivitive gain (d_gain), and the dead band (dead_band) of
　　a pid control structure _pid.
------------------------------------------------------------------------*/
//设定PID参数 －－－－ P,I,D,死区
void pid_tune(pid_type *pid, float p_gain, float i_gain, float d_gain, int dead_band)
{
	pid->pgain = p_gain;
	pid->igain = i_gain;
	pid->dgain = d_gain;
	pid->deadband = dead_band;
	pid->integral= integral_val;
	pid->last_error=0;
}

/*------------------------------------------------------------------------
pid_setinteg　
DESCRIPTION Set a new value for the integral term of the pid equation.
This is useful for setting the initial output of the
pid controller at start up.
　　------------------------------------------------------------------------*/
//设定输出初始值
void pid_setinteg(pid_type *pid,float new_integ)
{
	pid->integral = new_integ;
	pid->last_error = 0;
}

/*------------------------------------------------------------------------
pid_bumpless
　　
ESCRIPTION Bumpless transfer algorithim. When suddenly changing
setpoints,   or when restarting the PID equation after an
extended pause, the derivative of the equation can cause
a bump in the controller output. This function will help
smooth out that bump. The process value in *pv should
be the updated just before this function is used.
　　------------------------------------------------------------------------*/
//pid_bumpless 实现无扰切换  当突然改变设定值时，或重新启动后，
//将引起扰动输出。这个函数将能实现平顺扰动，在调用该函数之前需要先更新PV值
void pid_bumpless(pid_type *pid)
{
		pid->last_error = (pid->sp)-(pid->pv);//设定值与反馈值的偏差，手动时，SP值跟随PV值
}

/*------------------------------------------------------------------------
pid_calc　　
DESCRIPTION Performs PID calculations for the _pid structure *a. This function uses the positional form of the pid equation, and incorporates an integral windup prevention algorithim. Rectangular integration is used, so this function must be repeated on a consistent time basis for accurate control.
		　　
RETURN VALUE The new output value for the pid loop.　
USAGE #include "control.h"*/

float pid_calc(pid_type *pid)
{
		int err;
		float pterm, dterm, result, ferror;
			
		err = (pid->sp) - (pid->pv);
		if (abs(err) > pid->deadband)//大于死区
		{
			ferror = (float) err; /*do integer to float conversion only once*/
			pterm = pid->pgain * ferror;//比例项
			if (pterm > 100 || pterm < -100)
			{
				pid->integral = 0.0;
			}
			else
			{
				pid->integral += pid->igain * ferror;//积分项
				if (pid->integral > 100.0)
				{
					pid->integral = 100.0;
				}
				else if (pid->integral < 0.0) 
					pid->integral = 0.0;
			}
			dterm = ((float)(err - pid->last_error)) * pid->dgain;//微分项
			result = pterm + pid->integral + dterm;//P，I，D
		}
		else result = pid->integral;//输出
		pid->last_error = err;//保存上次偏差
		return (result);
}
u16 max =100,min = 0;
u16 rand_result = 0;

#include "delay.h"
void PID_main(void)
{  
		float display_value;
		int count=0;
		pid = &warm;
//		// printf("Enter the values of Process point, Set point, P gain, I gain, D gain \n");
//		// scanf("%d%d%f%f%f", &process_point, &set_point, &p_gain, &i_gain, &d_gain);
		process_point = 100;
		set_point = 40;
		p_gain = (float)(5.2);
		i_gain = (float)(0.77);
		d_gain = (float)(0.18);
		dead_band = 2;
		integral_val =(float)(0.01);
//		printf("The values of Process point, Set point, P gain, I gain, D gain \n");
//		printf(" %6d %6d %4f %4f %4f \r\\n", process_point, set_point, p_gain, i_gain, d_gain);
//		printf("Enter the values of Process point\n");
	  
		while(count<=20)
		{
//				scanf("%d",&process_point);
			
//			  rand_result = rand()%(max-min)+min;//产生随机数[0-100]
			  if(count<10) rand_result = 50;
			  else if(count>10&&count<15) rand_result = 40;
////			  else if(rand_result>30&&rand_result<60) rand_result = 60;
//			  else rand_result = 0 ;
//			rand_result = 0;
			  process_point = rand_result;
			  delay_ms(200);
			
				pid_init(&warm, process_point, set_point);// 设定PV,SP 值 
				pid_tune(&warm, p_gain,i_gain,d_gain,dead_band);//设定参数值
				pid_setinteg(&warm,0.01); //初始化PID 输出值
				//Get input value for process point
				pid_bumpless(&warm);
				// how to display output
				display_value = pid_calc(&warm);
			
			  Data_Send_User(display_value);//传送到匿名地面站
			
				printf("%f\n", display_value);
				//printf("\n%f%f%f%f",warm.pv,warm.sp,warm.igain,warm.dgain);
				count++;
//				process_point = 100;
		}
//		process_point = 100;
}

//PID算法
/******************************************************************************************************
* PID控制算法 PID Control Algorithm
* 在仿真和实验中，航向预估控制方法和与之对比的常规控制方法的控制器均采用增量PID算法，
* 且两者的比例、积分和微分系数一样.增量PID算法如式(7)所示：
*  ΔU = U(k)-U(k-1) = Kp*[e(k)-e(k-1)]+Ki*e(k)+Kd*[e(k)-2*e(k-1)+e(k-2)]
* 其中：Kp、Ki、Kd分别为比例、积分和微分放大系数，u(k)表示第k个采样时刻的控制量,e(k)表示第k个采样时刻的航向输入偏差.
*******************************************************************************************************/

/*
void PID_Calc(int PID_EK)
{
 long deltaU,tmp;

 deltaU = (long)u_dbPIDKp.IntData*(PID_EK-PID_EK_1);  // 增量计算
 deltaU +=(long)u_dbPIDKi.IntData*PID_EK;
 tmp = (long)PID_EK-(2*PID_EK_1)+PID_EK_2;
 deltaU +=tmp* u_dbPIDKd.IntData;
 PID_UK = PID_UK_1+deltaU;       // 结果
 if(PID_UK>4095000) PID_UK = 4095000;
 else if(PID_UK<0) PID_UK=0;
 PID_UK_1 = PID_UK;         // 保存新的K-1次输出值
 PID_EK_2 = PID_EK_1;        // 保存新的K-1次输入值
 PID_EK_1 = PID_EK;
}
PID是比例，积分，微分的缩写，
Uo(N)=P*E(N)+I*[E(N)+E(N-1)+...+E(0)]+D*[E(N)-E(N-1)]

E-误差
P--改变P可提高响应速度，减小静态误差，但太大会增大超调量和稳定时间。
I--与P的作用基本相似，但要使静态误差为0，必须使用积分。
D--与P,I的作用相反，主要是为了减小超调，减小稳定时间。

三个参数要综合考虑，一般先将I,D设为0，调好P,达到基本的响应速度和误差，再加上I,使误差为0，这时再加入D,三个参数要反复调试，最终达到较好的结果。不同的控制对象，调试的难度相差很大，祝好运！




PID算法实现
PID算法是目前一般控制领域中经常使用的自动控制算法，它依据给定的设定值，反馈值，以及比例系数，积分和微分时间，计算出一定的控制量，使被控对象能保持在设定的工作范围，并且可以自动的消除外部扰动。下面介绍PID算法的实现以及其离散化的过程和依据。
一、功能模块的运算公式：
OUT  = GAIN *（E +	   						 + 			  ）
其中GAIN表示比例系数，RATE表示微分时间系数，RESET表示积分时间系数，E表示过程变量PV与设定值SP的偏差，S表示拉氏变换的算子。以上公式是以拉氏变换表达式表示的输入与输出关系的传递函数。
	二、线性替换法：
	 传递函数和脉冲传递函数是控制工程领域中应用最为广泛的模型描述方式。替换法的基本思想是：对于给定的函数G（S），设法找到S域到Z域的某种映射关系，它将S域的变量S映射到Z平面上，由此得到与连续系统传递函数G(S)相对应的离散传递函数G(Z)。进而根据G(Z)由Z的反变换求得系统的时域离散模型——即差分方程，据此便可以进行快速求解。
	根据Z变换理论，S域到Z域的最基本的映射关系是Z变换式 Z = e 或s = (lnz)/T。如果按照这一映射关系直接代入G(s)，得到的函数将是相当复杂的，不便于算法实现，所以往往借助于Z变换的基本映射关系Z = e 或s = (lnz)/T或作一些简化处理。
	有很多种简化处理方法，有PADE近似公式，以及从中引申出简单替换法和双线性替换法。其中简单变换法比较简单，但是其主要缺点是可能会使得离散模型不稳定，所以按照简单替换法化建立的仿真模型不可靠，不实用。
	双线性变换相当于数值积分法中的梯形积分公式。相比于简单替换法而言，具有不影响系统的稳定性，并具有较高的精度。此外双线性替换还可以保持模型的阶次不变，在低频输入时频率特性吻合很好。因此，我们的PID算法采用了双线性替换的方法，下面介绍双线性替换的具体过程和步骤。
三、            积分，微分部分的离散化处理
根据介绍的双线性替换法，分别对上面的比例，积分，微分部分进行离散化处理，然后对各个部分的输入输出关系进行叠加处理，即可得到离散化的PID算式。
a)        比例环节：OUT(k) = GAIN *  E(k)；其中k表示第k次，由上式可以看出，在比例环节中，输出的控制量OUT(k)为本次过程变量与设定值的偏差E(k)乘以一个放大系数GAIN而得。
b)        积分环节：在对积分环节进行离散化处理中，采用的是双线性替换法，即把传递公式中的S用以下算式代替即可,下式中T为采样时间周期：
S = ( 2 / T ) *
即有OUT = E * [(GAIN / RESET) * T * (Z + 1)] / [2 * (Z – 1 )]；将之展开并进行适当的变形，可得到公式OUT( Z – 1) = C * E *（Z + 1），其中C = (GAIN * T) / ( 2* RESET)；再离散化就可得到：
	OUT( K+1) – OUT(K) = C* [E( K+ 1) – E(K)]；
	即：OUT( K) = OUT(K–1) + C * [ E(K) + E(K–1)] ；
c)        微分环节：类似于积分环节的变换，我们可以得到：
OUT(K) = A * [PV(K) – PV(K–1)] – B * OUT(K –1)；
其中A = (2 * GAIN * RATE) / ( T + 2*α*RATE);
		 B = (1 – 2 * α * RATE) / ( T + 2*α*RATE);
	把上面离散化后的三个式子联合起来，就得到了PID算法的离散形势：
	OUT(k) = GAIN * E(k) + OUT(K–1) + C * [ E(K) + E(K–1)] + A * [PV(K) – PV(K–1)] – B * OUT(K–1)；


*/



//比较典型的PID算法控制程序源代码
/*====================================================================================================这是一个比较典型的PID处理程序,在使用单片机作为控制cpu时,请稍作简化,具体的PID参数必须由具体对象通过实验确定。由于单片机的处理速度和ram资源的限制,一般不采用浮点数运算,而将所有参数全部用整数,运算
到最后再除以一个2的N次方数据（相当于移位）,作类似定点数运算,可大大提高运算速度,根据控制精度的不同要求,当精度要求很高时,注意保留移位引起的“余数”,做好余数补偿。这个程序只是一般常用pid算法的基本架构,没有包含输入输出处理部分。
=====================================================================================================*/

/*
#include
#include*/
/*====================================================================================================
PID Function
The PID (比例、积分、微分) function is used in mainly
control applications. PIDCalc performs one iteration of the PID
algorithm.
While the PID function works, main is just a dummy program showing
a typical usage.
=====================================================================================================*/
/*
typedef struct PID {
double SetPoint; // 设定目标Desired value
double Proportion; // 比例常数Proportional Const
double Integral; // 积分常数Integral Const
double Derivative; // 微分常数Derivative Const
double LastError; // Error[-1]

double PrevError; // Error[-2]
double SumError; // Sums of Errors
} PID;

*/
/*====================================================================================================
PID计算部分
=====================================================================================================*/
//double PIDCalc( PID *pp, double NextPoint )
//{
//double dError,
//Error;
//Error = pp->SetPoint - NextPoint; // 偏差
//pp->SumError += Error; // 积分
//dError = pp->LastError - pp->PrevError; // 当前微分
//pp->PrevError = pp->LastError;
//pp->LastError = Error;
//return (pp->Proportion * Error // 比例项
//+ pp->Integral * pp->SumError // 积分项
//+ pp->Derivative * dError // 微分项
//);
//}
/*====================================================================================================
Initialize PID Structure
=====================================================================================================*/
//void PIDInit (PID *pp)
//{
//memset ( pp,0,sizeof(PID));
//}
/*====================================================================================================
Main Program
=====================================================================================================*/
//double sensor (void) // Dummy Sensor Function
//{
//return 100.0;
//}
//void actuator(double rDelta) // Dummy Actuator Function
//{}
//void main(void)
//{
//PID sPID; // PID Control Structure
//double rOut; // PID Response (Output)
//double rIn; // PID Feedback (Input)
//PIDInit ( &sPID ); // Initialize Structure
//sPID.Proportion = 0.5; // Set PID Coefficients
//sPID.Integral = 0.5;
//sPID.Derivative = 0.0;
//sPID.SetPoint = 100.0; // Set PID Setpoint
//for (;;) { // Mock Up of PID Processing
//rIn = sensor (); // Read Input
//rOut = PIDCalc ( &sPID,rIn ); // Perform PID Interation
//actuator ( rOut ); // Effect Needed Changes
//}



