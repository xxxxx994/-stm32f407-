#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "OLED.h"

//ALIENTEK 探索者STM32F407开发板 实验12
//OLED显示实验-库函数版本 
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK

// 蜂鸣器IO定义
#define BUZZER_PIN GPIO_Pin_8
#define BUZZER_GPIO GPIOA
#define BUZZER_RCC RCC_AHB1Periph_GPIOA

void Buzzer_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIO时钟
    RCC_AHB1PeriphClockCmd(BUZZER_RCC, ENABLE);
    
    // 配置蜂鸣器引脚为输出模式
    GPIO_InitStructure.GPIO_Pin = BUZZER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BUZZER_GPIO, &GPIO_InitStructure);
    
    // 初始状态关闭蜂鸣器
    GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
}

void Buzzer_On(void) {
    GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN);
}

void Buzzer_Off(void) {
    GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
}
// 霍尔传感器IO定义
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
    
    // 使能GPIO时钟
    RCC_AHB1PeriphClockCmd(HALL_RCC, ENABLE);
    
    // 使能SYSCFG时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    
    // 配置霍尔传感器引脚为输入模式
    GPIO_InitStructure.GPIO_Pin = HALL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(HALL_GPIO, &GPIO_InitStructure);
    
    // 连接EXTI线路到霍尔传感器引脚
    SYSCFG_EXTILineConfig(HALL_EXTI_PORT_SOURCE, HALL_EXTI_PIN_SOURCE);
    
    // 配置EXTI
    EXTI_InitStructure.EXTI_Line = HALL_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 磁钢靠近时触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = HALL_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
// 超声波IO定义
#define TRIG_PIN GPIO_Pin_5
#define ECHO_PIN GPIO_Pin_6
#define ULTRASONIC_GPIO GPIOA
#define ULTRASONIC_RCC RCC_AHB1Periph_GPIOA

void Ultrasonic_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIO时钟
    RCC_AHB1PeriphClockCmd(ULTRASONIC_RCC, ENABLE);
    
    // 配置TRIG引脚为输出
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ULTRASONIC_GPIO, &GPIO_InitStructure);
    
    // 配置ECHO引脚为输入
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ULTRASONIC_GPIO, &GPIO_InitStructure);
    
    // 初始化TRIG引脚为低电平
    GPIO_ResetBits(ULTRASONIC_GPIO, TRIG_PIN);
}

float Ultrasonic_GetDistance(void) {
    uint32_t start_time, end_time, duration;
    float distance;
    
    // 发送10us的触发脉冲
    GPIO_SetBits(ULTRASONIC_GPIO, TRIG_PIN);
    delay_us(10);
    GPIO_ResetBits(ULTRASONIC_GPIO, TRIG_PIN);
    
    // 等待ECHO引脚变高
    while(GPIO_ReadInputDataBit(ULTRASONIC_GPIO, ECHO_PIN) == 0);
    start_time = TIM_GetCounter(TIM2);
    
    // 等待ECHO引脚变低
    while(GPIO_ReadInputDataBit(ULTRASONIC_GPIO, ECHO_PIN) == 1);
    end_time = TIM_GetCounter(TIM2);
    
    // 计算脉冲持续时间 (us)
    duration = end_time - start_time;
    
    // 计算距离 (cm)，声速为340m/s，即0.034cm/us
    distance = duration * 0.034f / 2.0f;
    
    return distance;
}
// 定义参数
#define MAX_SPEED 30.0f         // 最大安全速度 km/h
#define MIN_DISTANCE 50.0f      // 最小安全距离 cm
#define WHEEL_CIRCUMFERENCE 2.0f // 车轮周长 m

// 全局变量
volatile uint32_t hall_pulse_count = 0;
volatile uint32_t last_hall_time = 0;
float current_speed = 0.0f;
float total_distance = 0.0f;
float current_distance = 0.0f;
uint32_t last_update_time = 0;

// 霍尔传感器中断处理函数
void EXTI1_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
        uint32_t current_time = TIM_GetCounter(TIM2);
        if(last_hall_time != 0) {
            uint32_t time_diff = current_time - last_hall_time;
            // 计算速度 (m/s)
            float speed_ms = WHEEL_CIRCUMFERENCE / (time_diff * 0.000001f); // 假设TIM2时钟为1MHz
            current_speed = speed_ms * 3.6f; // 转换为km/h
            
            // 更新总距离
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数
	//uart_init(115200);	//初始化串口波特率为115200
	//LED_Init();					//初始化LED
	OLED_Init();
	Ultrasonic_Init();
  HallSensor_Init();
  Buzzer_Init();
	
	// TIM2初始化 - 用于计时
   
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
		// 每200ms更新一次
        uint32_t current_time = TIM_GetCounter(TIM2);
        if(current_time - last_update_time > 200000) { // 200ms
            last_update_time = current_time;
            
            // 读取超声波传感器距离
            current_distance = Ultrasonic_GetDistance();
            
            // 显示信息
					sprintf(buff,"Speed: %.2fkm/h",current_speed);
					OLED_ShowString(1, 1, buff);
					sprintf(buff,"Distance:%.2fm",total_distance);
					OLED_ShowString(2, 1, buff);
					sprintf(buff,"Rear:%.2fcm",current_distance);
					OLED_ShowString(3, 1, buff);
     
            
            // 检查报警条件
            if(current_speed > MAX_SPEED || current_distance < MIN_DISTANCE) {
                //Buzzer_On();
							Buzzer_Off();
            } else {
                //Buzzer_Off();
							Buzzer_On();
            }
        }
        // 短暂延时
        delay_ms(10);
		
	}
}
