///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

float accAngleSmooth[3];

///////////////////////////////////////////////////////////////////////////////

void initOrientation()
{
    int initLoops = 150;
    float accAngle[NUMAXIS] = { 0.0f, 0.0f, 0.0f };
    int i;

    for (i = 0; i < initLoops; i++)
    {
        readMPU6050();//��MPU6050�õ����ٶȺ����������ݣ������� ���ʼ����λ���ƾ��󣨸���IMU��Ԫ�ķ�λȷ���ľ���A �� ��˺������ 

        computeMPU6050TCBias();//	�����¶Ȳ���ƫ��ֵ

			//��������˺�ļ��ٶ�����-�¶Ȳ���ƫ�* ��(1/8192) * 9.8065�� 
			//(1/8192) * 9.8065  (8192 LSB = 1 G)
		 //1G���̵�8192����������֮1����Ӧ�������ٶ�9.8065m/1G��8192��֮1
        sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
        sensors.accel500Hz[ZAXIS] = -((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

			//����ŷ���ǻ�������
        accAngle[ROLL]  += atan2f(-sensors.accel500Hz[YAXIS], -sensors.accel500Hz[ZAXIS]);
        accAngle[PITCH] += atan2f(sensors.accel500Hz[XAXIS], -sensors.accel500Hz[ZAXIS]);

			//��ȡŷ��������ƽ��ֵ
        accAngleSmooth[ROLL ] = accAngle[ROLL ] / (float)initLoops;
        accAngleSmooth[PITCH] = accAngle[PITCH] / (float)initLoops;

        delay(2);
    }
		
		//�õ���ǰ��λ ,��ʼ��һ�Σ���Ҫ����̨����Ϊ����ֻ���˼��ٶ����ݼ���ŷ���ǣ����ٶ������ǳ��ڿ��ŵģ������Ǽ��ٶȼƶ�
		//�񶯺����У�����Ϊ�˼�С����ʼ����λ��ʱ��Ҫ����̨��
    sensors.evvgcCFAttitude500Hz[PITCH] = accAngleSmooth[PITCH];
    sensors.evvgcCFAttitude500Hz[ROLL ] = accAngleSmooth[ROLL ];
    sensors.evvgcCFAttitude500Hz[YAW  ] = 0.0f;
}

///////////////////////////////////////////////////////////////////////////////
//�˺����Ƿ�λ���Ƶĺ��ĺ���
void getOrientation(float *smoothAcc, float *orient, float *accData, float *gyroData, float dt)
{
    float accAngle[3];
    float gyroRate[3];

	//ͨ��ʹ��atan2f����������ٶ����ݵõ�ŷ���� ��ת�Ǻ� �����ǡ�
    accAngle[ROLL ] = atan2f(-accData[YAXIS], -accData[ZAXIS]);
    accAngle[PITCH] = atan2f(accData[XAXIS], -accData[ZAXIS]);

	//���� smoothAcc ��ͨ�����ٶ����ݾ���atan2f�������������ŷ���ǣ����ҽ�����һ���ͺ��˲�
	//�����˲��㷨Ҳ���ڵ�ͨ�˲���һ�֣����ŵ㣺 �������Ը��ž������õ��������� �����ڲ���Ƶ�ʽϸߵĳ���,
	// ȱ�㣺 ��λ�ͺ������ȵ� �ͺ�̶�ȡ����aֵ��С�� ���������˲�Ƶ�ʸ��ڲ���Ƶ�ʵ�1/2�ĸ����ź�,������a��ֵ��99.0f
    smoothAcc[ROLL]  = ((smoothAcc[ROLL ] * 99.0f) + accAngle[ROLL ]) / 100.0f;
    smoothAcc[PITCH] = ((smoothAcc[PITCH] * 99.0f) + accAngle[PITCH]) / 100.0f;

    gyroRate[PITCH] =  gyroData[PITCH];
	//ͨ�������˲����ںϸ��ݼ��ٶȺ������Ǽ�������ĽǶȣ�orient[PITCH]���ϴ��ںϺ�ĽǶȣ�gyroRate[PITCH] * dt�Ǹ���������
	//���ݼ���õ��ĽǶȣ����ٶ�*����ʱ�������ǻ��ȣ����ȺͽǶȺ����׵Ŀ����໥ת������ΪʲôҪ���������ںϣ�
	//�𣺼��ٶȼƺ������Ƕ��ܼ������̬����Ϊ��Ҫ�������ںϣ�����Ϊ���ٶȼƶ���֮����Ŷ������У����������ݼ��������̬���ţ�
	//����������Ȼ������Щ�����У�������ʹ�������ǻ����Ư�ƣ��������Ҫ���л������������������ǣ��������ż��ٶȼ�.	
	//��ͨ�����ٶȼƵõ��ĽǶȼ�ȥ��һ���ںϺ�ĽǶ�Ȼ�����һ������ϵ�����������ϵ��ԽС���ںϵļ��ٶȼƵ����ݱ���ԽС��
	//�������������ǣ����������ǵı���������1���������ż��ٶȼƣ����ٶȼƵ������������������ǵ�Ư�Ʋ�������
	//�����������ǵ�Ư�ƽ�������������Ч�������˼��ٶȼƺ������Ǹ��Ե�������ʱ���ƫ��.
    orient[PITCH]   = (orient[PITCH] + gyroRate[PITCH] * dt) + 0.0002f * (smoothAcc[PITCH] - orient[PITCH]);
	
		//���������һ����Һ�x�ᵥ������Ƕ�,��Ϊ����֪�������Ĵ�С
	 //����IMU��Ԫ�����Ǿ�ֹˮƽ״̬
    gyroRate[ROLL]  =  gyroData[ROLL] * cosf(fabsf(orient[PITCH])) + gyroData[YAW] * sinf(orient[PITCH]);
	
	//ͨ�������˲����ںϸ��ݼ��ٶȺ������Ǽ�������ĽǶȣ�orient[PITCH]���ϴ��ںϺ�ĽǶȣ�gyroRate[PITCH] * dt�Ǹ���������
	//���ݼ���õ��ĽǶȣ����ٶ�*����ʱ�������ǻ��ȣ����ȺͽǶȺ����׵Ŀ����໥ת������ΪʲôҪ���������ںϣ�
	//�𣺼��ٶȼƺ������Ƕ��ܼ������̬����Ϊ��Ҫ�������ںϣ�����Ϊ���ٶȼƶ���֮����Ŷ������У����������ݼ��������̬���ţ�
	//����������Ȼ������Щ�����У�������ʹ�������ǻ����Ư�ƣ��������Ҫ���л������������������ǣ��������ż��ٶȼ�.	
	//��ͨ�����ٶȼƵõ��ĽǶȼ�ȥ��һ���ںϺ�ĽǶ�Ȼ�����һ������ϵ�����������ϵ��ԽС���ںϵļ��ٶȼƵ����ݱ���ԽС��
	//�������������ǣ����������ǵı���������1���������ż��ٶȼƣ����ٶȼƵ������������������ǵ�Ư�Ʋ�������
	//�����������ǵ�Ư�ƽ�������������Ч�������˼��ٶȼƺ������Ǹ��Ե�������ʱ���ƫ��.
    orient[ROLL]    = (orient[ROLL] + gyroRate[ROLL] * dt) + 0.0002f * (smoothAcc[ROLL] - orient[ROLL]);

		//���������һ����Һ�x�ᵥ������Ƕ�,��Ϊ����֪�������Ĵ�С
	 //����IMU��Ԫ�����Ǿ�ֹˮƽ״̬
    gyroRate[YAW]   =  gyroData[YAW] * cosf(fabsf(orient[PITCH])) - gyroData[ROLL] * sinf(orient[PITCH]);
		
    orient[YAW]     = (orient[YAW] + gyroRate[YAW] * dt);//�������ǽ��л��ֵõ�ƫ����YAW
}

///////////////////////////////////////////////////////////////////////////////
