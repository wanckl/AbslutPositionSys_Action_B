#include "led.h" 


//��ʼ��PC6��PC7Ϊ�����.��ʹ���������ڵ�ʱ��
//LED IO��ʼ��
void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOFʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);				//��ʼ��
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7);

}


