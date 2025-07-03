#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "OLED.h"

//ALIENTEK ̽����STM32F407������ ʵ��12
//OLED��ʾʵ��-�⺯���汾 
//����֧�֣�www.openedv.com
//�Ա����̣�http://eboard.taobao.com  
//������������ӿƼ����޹�˾  
//���ߣ�����ԭ�� @ALIENTEK

// ������IO����
#define BUZZER_PIN GPIO_Pin_8
#define BUZZER_GPIO GPIOA
#define BUZZER_RCC RCC_AHB1Periph_GPIOA

void Buzzer_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ʹ��GPIOʱ��
    RCC_AHB1PeriphClockCmd(BUZZER_RCC, ENABLE);
    
    // ���÷���������Ϊ���ģʽ
    GPIO_InitStructure.GPIO_Pin = BUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BUZZER_GPIO, &GPIO_InitStructure);
    
    // ��ʼ״̬�رշ�����
    GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
}

void Buzzer_On(void) {
    GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN);
}

void Buzzer_Off(void) {
    GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
}
// ����������IO����
#define HALL_PIN GPIO_Pin_1
#define HALL_GPIO GPIOA
#define HALL_RCC RCC_AHB1Periph_GPIOA
#define HALL_EXTI_LINE EXTI_Line1
#define HALL_EXTI_PORT_SOURCE EXTI_PortSourceGPIOA
#define HALL_EXTI_PIN_SOURCE EXTI_PinSource1
#define HALL_EXTI_IRQ EXTI1_IRQn

void HallSensor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // ʹ��GPIOʱ��
    RCC_AHB1PeriphClockCmd(HALL_RCC, ENABLE);
    
    // ʹ��SYSCFGʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    // ���û�������������Ϊ����ģʽ
    GPIO_InitStructure.GPIO_Pin = HALL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(HALL_GPIO, &GPIO_InitStructure);
    
    // ����EXTI��·����������������
    SYSCFG_EXTILineConfig(HALL_EXTI_PORT_SOURCE, HALL_EXTI_PIN_SOURCE);
    
    // ����EXTI
    EXTI_InitStructure.EXTI_Line = HALL_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // �Ÿֿ���ʱ����
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // ����NVIC
    NVIC_InitStructure.NVIC_IRQChannel = HALL_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
// ������IO����
#define TRIG_PIN GPIO_Pin_5
#define ECHO_PIN GPIO_Pin_6
#define ULTRASONIC_GPIO GPIOA
#define ULTRASONIC_RCC RCC_AHB1Periph_GPIOA

void Ultrasonic_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // ʹ��GPIOʱ��
    RCC_AHB1PeriphClockCmd(ULTRASONIC_RCC, ENABLE);
    
    // ����TRIG����Ϊ���
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ULTRASONIC_GPIO, &GPIO_InitStructure);
    
    // ����ECHO����Ϊ����
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ULTRASONIC_GPIO, &GPIO_InitStructure);
    
    // ��ʼ��TRIG����Ϊ�͵�ƽ
    GPIO_ResetBits(ULTRASONIC_GPIO, TRIG_PIN);
}

float Ultrasonic_GetDistance(void) {
    uint32_t start_time, end_time, duration;
    float distance;
    
    // ����10us�Ĵ�������
    GPIO_SetBits(ULTRASONIC_GPIO, TRIG_PIN);
    delay_us(10);
    GPIO_ResetBits(ULTRASONIC_GPIO, TRIG_PIN);
    
    // �ȴ�ECHO���ű��
    while(GPIO_ReadInputDataBit(ULTRASONIC_GPIO, ECHO_PIN) == 0);
    start_time = TIM_GetCounter(TIM2);
    
    // �ȴ�ECHO���ű��
    while(GPIO_ReadInputDataBit(ULTRASONIC_GPIO, ECHO_PIN) == 1);
    end_time = TIM_GetCounter(TIM2);
    
    // �����������ʱ�� (us)
    duration = end_time - start_time;
    
    // ������� (cm)������Ϊ340m/s����0.034cm/us
    distance = duration * 0.034f / 2.0f;
    
    return distance;
}
// �������
#define MAX_SPEED 30.0f         // ���ȫ�ٶ� km/h
#define MIN_DISTANCE 50.0f      // ��С��ȫ���� cm
#define WHEEL_CIRCUMFERENCE 2.0f // �����ܳ� m

// ȫ�ֱ���
volatile uint32_t hall_pulse_count = 0;
volatile uint32_t last_hall_time = 0;
float current_speed = 0.0f;
float total_distance = 0.0f;
float current_distance = 0.0f;
uint32_t last_update_time = 0;

// �����������жϴ�����
void EXTI1_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
        uint32_t current_time = TIM_GetCounter(TIM2);
        if(last_hall_time != 0) {
            uint32_t time_diff = current_time - last_hall_time;
            // �����ٶ� (m/s)
            float speed_ms = WHEEL_CIRCUMFERENCE / (time_diff * 0.000001f); // ����TIM2ʱ��Ϊ1MHz
            current_speed = speed_ms * 3.6f; // ת��Ϊkm/h
            
            // �����ܾ���
            total_distance += WHEEL_CIRCUMFERENCE;
        }
        last_hall_time = current_time;
        hall_pulse_count++;
        
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}


int main(void)
{ 
	 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	u8 t=0;
	char buff[20]={0};
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);     //��ʼ����ʱ����
	//uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	//LED_Init();					//��ʼ��LED
	OLED_Init();
	Ultrasonic_Init();
  HallSensor_Init();
  Buzzer_Init();
	
	// TIM2��ʼ�� - ���ڼ�ʱ
   
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 83; // 84MHz / 84 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
	
	  OLED_ShowString(1, 1, "Speed:    km/h");
    OLED_ShowString(2, 1, "Distance:   m");
    OLED_ShowString(3, 1, "Rear:     cm");
		
//  OLED_ShowString(0,0,"ALIENTEK",24);  
//	OLED_ShowString(0,24, "0.96' OLED TEST",16);  
// 	OLED_ShowString(0,40,"ATOM 2014/5/4",12);  
// 	OLED_ShowString(0,52,"ASCII:",12);  
// 	OLED_ShowString(64,52,"CODE:",12);  
	last_update_time = TIM_GetCounter(TIM2);
	while(1) 
	{		
		// ÿ200ms����һ��
        uint32_t current_time = TIM_GetCounter(TIM2);
        if(current_time - last_update_time > 200000) { // 200ms
            last_update_time = current_time;
            
            // ��ȡ����������������
            current_distance = Ultrasonic_GetDistance();
            
            // ��ʾ��Ϣ
					sprintf(buff,"Speed: %.2fkm/h",current_speed);
					OLED_ShowString(1, 1, buff);
					sprintf(buff,"Distance:%.2fm",total_distance);
					OLED_ShowString(2, 1, buff);
					sprintf(buff,"Rear:%.2fcm",current_distance);
					OLED_ShowString(3, 1, buff);
     
            
            // ��鱨������
            if(current_speed > MAX_SPEED || current_distance < MIN_DISTANCE) {
                //Buzzer_On();
							Buzzer_Off();
            } else {
                //Buzzer_Off();
							Buzzer_On();
            }
        }
        // ������ʱ
        delay_ms(10);
		
	}
}
