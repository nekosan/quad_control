/*
 * クァッドコプターのコントロールプログラム
 *
 */

/*
 * モータの番号は、1から4まで。
 * X軸のモータを1と3(CCW)
 * Y軸のモータを2と4(CW)
 *
 * Gyro X: roll 前方にCWが正
 * Gyro Y: pitch 左方にCWが正
 * Gyro Z: yaw 上方にCWが正
 */


//#define USE_COM

#include "stm32f10x.h"
#include "platform_config.h"

#ifdef USE_COM
#include "com_config.h"
#endif

//#include "delay.h"
#include <stdlib.h>
#include "math.h"

//モーションセンサのI2Cアドレス
#define LSM9DS0_GYR_ADRS 0x6b << 1
#define LSM9DS0_ACC_ADRS 0x1d << 1


//初期化関数群
void Init_GPIO();
void Init_NVIC();
void Init_TIM3();
void Init_I2C();
void Init_USART1();

//ループ関数
void loop();

//センサ関連関数
uint8_t lsm9ds0_init();
uint8_t lsm9ds0_get_acc();
uint8_t lsm9ds0_get_gyr();
uint8_t lsm9ds0_gyr_offset();

//傾き計算関数
void calc_tilt();

//i2c操作関数
extern uint8_t i2c_write(uint8_t address, uint8_t reg_adrs, uint8_t *d, uint8_t len);
extern uint8_t i2c_read(uint8_t address, uint8_t reg_adrs, uint8_t *d, uint8_t len);

//USART関連関数
void usart1_send(char *s);

//スロットル関連関数
void set_throttle();
void control();

//傾き検出用変数
int16_t gyr_offset[3] = {0};//ジャイロセンサオフセット
int16_t acc_var[3];//加速度センサの値 0:x 1:y 2:z
int16_t gyr_var[3];//ジャイロセンサの値 0:x 1:y 2:z
double tilt[2] = {0.0, 0.0};//傾き
double tilt_before[2] = {0.0, 0.0};//1回前の傾き
const double period = 0.1;//周期
const double pi = 3.14159265;
double tilt_acc[2], dps_gyr[2];

//USART1用変数
uint8_t u1_rx_buf = 0;
int8_t u1_rx_data[8] = {0};
uint8_t u1_rx_state = 0;//受信バイト数
uint8_t u1_tx_buf[64];
uint8_t u1_tx_pt;

//飛行パラメタ変数
uint16_t throttle[4] = {0};
uint16_t all_power;//全出力
double aim_angle[3] = {0.0};//3軸についての目標角度
double Kpt = 0.01, Kit = 0.001;//傾きPI補償定数
double Kpo = 600.0;//角速度P補償定数
double Kpy = 5.0;//ヨー軸角速度P補償定数

int flag = 0;
int main(void)
{
    BoardInit();

    Init_NVIC();
    Init_GPIO();
    Init_TIM3();
    Init_I2C();
    Init_USART1();

#ifdef USE_COM
    COM_Configuration();
#endif

    lsm9ds0_init();
    lsm9ds0_gyr_offset();

    //ブラシレスアンプに1ms長のパルスを送って初期化
    TIM_SetCompare1(TIM3, 1000);
    TIM_SetCompare2(TIM3, 1000);
    TIM_SetCompare3(TIM3, 1000);
    TIM_SetCompare4(TIM3, 1000);

    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    while(!flag);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    GPIO_SetBits(GPIOB, GPIO_Pin_14);

    all_power = 1000;//出力値の初期値設定

    SysTick_Config(1440000 - 1);//1440000 / 72000000 = 0.02[s]

    while(1){
    }
}

//SysTick呼ばれたときに呼ぶ関数
void loop()
{
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    //9軸センサから加速度と角速度のデータを取得
    if(lsm9ds0_get_acc())return;
    if(lsm9ds0_get_gyr())return;

    //傾きを計算
    calc_tilt();
    control();

    //モータの出力を反映
    set_throttle();

    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
}

//モータ出力の決定
//角速度をP補償で制御し、定常誤差を傾きPI補償で補正
void control()
{
    double y[3];//ロールピッチヨー角の出力
    double t_out[3] = {0};//傾き補償出力
    static double t_int[3] = {0};//傾き積分値
    double e[3];//傾き目標値との誤差

    e[0] = aim_angle[0] - tilt[0];
    e[1] = aim_angle[1] - tilt[1];
    t_int[0] += e[0];
    t_int[1] += e[1];

    if(all_power < 1300){
        t_int[0] = 0;
        t_int[1] = 0;
    }

    t_out[0] = Kpt * e[0] + Kit * t_int[0];
    t_out[1] = Kpt * e[1] + Kit * t_int[1];

    y[0] = Kpo * (t_out[0] - dps_gyr[0]);
    y[1] = Kpo * (t_out[1] - dps_gyr[1]);
    y[2] = Kpy * (-dps_gyr[2]);
    y[2] = 0;

    throttle[0] = - (int)(y[0]) - (int)(y[1]) - (int)(y[2]) + all_power;
    throttle[1] =   (int)(y[0]) - (int)(y[1]) + (int)(y[2]) + all_power;
    throttle[2] =   (int)(y[0]) + (int)(y[1]) - (int)(y[2]) + all_power;
    throttle[3] = - (int)(y[0]) + (int)(y[1]) + (int)(y[2]) + all_power;

}

//傾き検出の高精度化
void calc_tilt()
{

    tilt_acc[0] = asin((double)(acc_var[1]) / sqrt((double)(acc_var[1] * acc_var[1] + acc_var[2] * acc_var[2])));
    tilt_acc[1] = asin((double)(acc_var[0]) / sqrt((double)(acc_var[0] * acc_var[0] + acc_var[2] * acc_var[2])));

    dps_gyr[0] = (double)(gyr_var[0]) / 32768.0 * 500.0 * period * pi / 180.0;
    dps_gyr[1] = (double)(gyr_var[1]) / 32768.0 * 500.0 * period * pi / 180.0;

    tilt[0] = 0.97 * (tilt_before[0] + dps_gyr[0] * 0.02) + 0.03 * tilt_acc[0];
    tilt_before[0] = tilt[0];
    tilt[1] = 0.97 * (tilt_before[1] + dps_gyr[1] * 0.02) - 0.03 * tilt_acc[1];
    tilt_before[1] = tilt[1];
}

//スロットルセット関数
void set_throttle()
{
    int i;
    for(i = 0; i < 4; i++){
        if(throttle[i] < 1000){
            throttle[i] = 1000;
        }
        else if(throttle[i] > 1800){
            throttle[i] = 1800;
        }
    }

    TIM_SetCompare1(TIM3, throttle[0]);
    TIM_SetCompare2(TIM3, throttle[1]);
    TIM_SetCompare3(TIM3, throttle[2]);
    TIM_SetCompare4(TIM3, throttle[3]);
}

uint8_t lsm9ds0_init()
{
    uint8_t buf;

    buf = 0xff;
    if(i2c_write(LSM9DS0_GYR_ADRS, 0x20, &buf, 1)){
        return 1;
    }
    buf = 0x10;//±500dps
    if(i2c_write(LSM9DS0_GYR_ADRS, 0x23, &buf, 1)){
        return 1;
    }

    buf = 0xa7;
    if(i2c_write(LSM9DS0_ACC_ADRS, 0x20, &buf, 1)){
        return 1;
    }
    buf = 0x08;//Range : ±4g
    if(i2c_write(LSM9DS0_ACC_ADRS, 0x21, &buf, 1)){
        return 1;
    }

    return 0;
}

uint8_t lsm9ds0_gyr_offset()
{
    int i;
    int16_t sum[3] = {0};
    for(i = 0; i < 10; i++){
        if(lsm9ds0_get_gyr())return 1;
        sum[0] += gyr_var[0];
        sum[1] += gyr_var[1];
        sum[2] += gyr_var[2];
    }
    gyr_offset[0] = sum[0] / 10;
    gyr_offset[1] = sum[1] / 10;
    gyr_offset[2] = sum[2] / 10;
    return 0;
}

uint8_t lsm9ds0_get_acc()
{
    uint8_t buf[6];
    if(i2c_read(LSM9DS0_ACC_ADRS, 0x28 | 0x80, buf, 6)){
        return 1;
    }

    acc_var[0] = (int16_t)((buf[1] << 8) |  buf[0]);
    acc_var[1] = (int16_t)((buf[3] << 8) |  buf[2]);
    acc_var[2] = (int16_t)((buf[5] << 8) |  buf[4]);

    return 0;
}

uint8_t lsm9ds0_get_gyr()
{
    uint8_t buf[6];
    if(i2c_read(LSM9DS0_GYR_ADRS, 0x28 | 0x80, buf, 6)){
        return 1;
    }

    gyr_var[0] = (int16_t)((buf[1] << 8) |  buf[0]) - gyr_offset[0];
    gyr_var[1] = (int16_t)((buf[3] << 8) |  buf[2]) - gyr_offset[1];
    gyr_var[2] = (int16_t)((buf[5] << 8) |  buf[4]) - gyr_offset[2];

    return 0;
}

void SysTick_Handler(void)
{
    loop();
}

void usart1_send(char *s)
{
    u1_tx_pt = 0;

    while(*s != 0){
        u1_tx_buf[u1_tx_pt] = *s;
        u1_tx_pt++;s++;
    }
    u1_tx_buf[u1_tx_pt] = 0;
    u1_tx_pt = 0;

    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        USART_SendData(USART1, u1_tx_buf[u1_tx_pt++]);
        if(u1_tx_buf[u1_tx_pt] == 0){
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
    else if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        u1_rx_buf = (uint8_t)USART_ReceiveData(USART1);
        if(u1_rx_buf == 0x80){
            u1_rx_state = 1;
        }
        else if(u1_rx_state > 0){
            u1_rx_data[u1_rx_state - 1] = (int8_t)(u1_rx_buf);
            u1_rx_state += 1;
            if(u1_rx_state > 5){
                all_power = 1000 + (int)(u1_rx_data[0]) * 6;
                aim_angle[0] = (double)(u1_rx_data[1]) / 15.0;
                aim_angle[1] = (double)(u1_rx_data[2]) / 15.0;
                aim_angle[2] = (double)(u1_rx_data[3]) / 15.0;
                if(u1_rx_data[4] & 0x01)flag = 1;
                u1_rx_state = 0;
            }
        }
    }
}

void Init_GPIO()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(USART1_GPIO_RCC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  I2C1_SCL_PIN | I2C1_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(I2C1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //USART
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA , &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA , &GPIO_InitStructure);
}

void Init_I2C()
{
    I2C_InitTypeDef i2c;

    RCC_APB1PeriphClockCmd(I2C1_RCC, ENABLE);

    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 100000;

    I2C_Init(I2C1, &i2c);

    I2C_Cmd(I2C1, ENABLE);
}

void Init_NVIC()
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Init_TIM3()
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(TIM3_RCC, ENABLE);//クロックを供給

    // CLK = 72MHz Prescaler = 72 TIM2 = 1MHz
    // Period = 30000

    TIM_TimeBaseStructure.TIM_Period = 29999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);

    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);

    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Disable);

    TIM_Cmd(TIM3, ENABLE);
}

void Init_USART1()
{
    USART_InitTypeDef USART_InitStructure;

    RCC_APB1PeriphClockCmd(USART1_RCC, ENABLE);

    USART_InitStructure.USART_BaudRate = 38400 - 1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}
