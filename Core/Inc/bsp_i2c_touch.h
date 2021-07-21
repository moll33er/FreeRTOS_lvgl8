#ifndef __I2C_TOUCH_H
#define	__I2C_TOUCH_H

#include "stm32f4xx.h"

/*ʹ������IIC����SOFT_IIC������Ϊ1��Ӳ��IIC������Ϊ0
!!ʹ��Ӳ��IICʱ�ǳ����׳��ִ��󣬲��Ƽ�*/
#define SOFT_IIC      1

/*�趨ʹ�õĵ�����IIC�豸��ַ*/
#define GTP_ADDRESS            0xBA

#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

/*I2C����*/
#define GTP_I2C                          I2C2
#define GTP_I2C_CLK_ENABLE()             __HAL_RCC_I2C2_CLK_ENABLE()
#define GTP_I2C_CLK_INIT				 RCC_APB1PeriphClockCmd	

#define GTP_I2C_SCL_PIN                  GPIO_PIN_4                 
#define GTP_I2C_SCL_GPIO_PORT            GPIOH                       
#define GTP_I2C_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOH_CLK_ENABLE()
#define GTP_I2C_SCL_AF                   GPIO_AF4_I2C2

#define GTP_I2C_SDA_PIN                  GPIO_PIN_5                  
#define GTP_I2C_SDA_GPIO_PORT            GPIOH                     
#define GTP_I2C_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOH_CLK_ENABLE()
#define GTP_I2C_SDA_AF                   GPIO_AF4_I2C2

/*��λ����*/
#define GTP_RST_GPIO_PORT                GPIOD
#define GTP_RST_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()
#define GTP_RST_GPIO_PIN                 GPIO_PIN_11
/*�ж�����*/
#define GTP_INT_GPIO_PORT                GPIOD
#define GTP_INT_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOD_CLK_ENABLE()
#define GTP_INT_GPIO_PIN                 GPIO_PIN_13
#define GTP_INT_EXTI_IRQ                 EXTI15_10_IRQn
/*�жϷ�����*/
#define GTP_IRQHandler                   EXTI15_10_IRQHandler


//����IICʹ�õĺ�
#define I2C_SCL_1()  HAL_GPIO_WritePin(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN,GPIO_PIN_SET)		/* SCL = 1 */
#define I2C_SCL_0()  HAL_GPIO_WritePin(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN,GPIO_PIN_RESET)		/* SCL = 0 */

#define I2C_SDA_1()  HAL_GPIO_WritePin(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN,GPIO_PIN_SET)		/* SDA = 1 */
#define I2C_SDA_0()  HAL_GPIO_WritePin(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN,GPIO_PIN_RESET)		/* SDA = 0 */

#define I2C_SDA_READ()  HAL_GPIO_ReadPin(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)	/* ��SDA����״̬ */

//�����ӿ�
void I2C_Touch_Init(void);
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead);
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer,  uint8_t NumByteToWrite);
void I2C_ResetChip(void);
void I2C_GTP_IRQDisable(void);
void I2C_GTP_IRQEnable(void);

#endif /* __I2C_TOUCH_H */