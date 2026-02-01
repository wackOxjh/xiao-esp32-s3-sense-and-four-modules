#include "stm32f10x.h"

#define	SCL(x)	GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)(x))
#define	SDA(x)	GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)(x))

enum DAC_VoltageGear {
	ZERO = 0,	    // 零挡位
	LOW = 1024,    // 低档位：≈0.819V（1024×LSB）
	MID = 2048,    // 中档位：≈1.65V
	HIGH = 3072,   // 高档位：≈2.47V
	FULL = 4095		// 满量程：≈3.3V
};

#define MCP4725_CMD_FAST_WRITE    0x00  // 快速写入DAC寄存器（不影响EEPROM）
#define MCP4725_CMD_WRITE_DAC     0x40  // 写入DAC寄存器（C2=0,C1=1,C0=0）
#define MCP4725_CMD_WRITE_DAC_EEPROM 0x60  // 写入DAC并保存到EEPROM（C2=0,C1=1,C0=1）

uint8_t tx_data[3],rx_data[3];

void MCP4725_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	//开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SCL(1);
	SDA(1);
}

void I2C_Start(void)
{
	SDA(1);
	SCL(1);
	SDA(0);
	SCL(0);
}

void I2C_Stop(void)
{
	SDA(0);
	SCL(1);
	SDA(1);
}

void I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		SDA(!!(Byte & (0x80 >> i)));
		SCL(1);
		SCL(0);
	}
	SCL(1);	
	SCL(0);
}

/*
写命令
*/
void MCP_WriteCommand(uint8_t Command)
{
	I2C_Start();
	I2C_SendByte(0xC4);					//从机地址
	I2C_SendByte(0x00);					//写命令
	I2C_SendByte(Command);
	I2C_Stop;
}

/**
  * @brief  写数据
  * @param  dac_value 要写入的数据；
			power_mode：MCP4725_CMD_FAST_WRITE
						MCP4725_CMD_WRITE_DAC
						MCP4725_CMD_WRITE_DAC_EEPROM
  * @retval 无          
  */                    
void MCP_WriteData(uint8_t dac_value, uint8_t power_mode)
{
	if (dac_value > 4095) return TX_DATA_ERROR;
	I2C_Start();
	I2C_SendByte(0xC4);		//从机地址0x62+0(写)=0xC4
//	I2C_SendByte(0x40);		//写数据
//	dac_value = ()|()
//	tx_data[1] = (dac_value & 0x0F) << 4;	// 第2字节：DAC数据低4位（D3-D0）+ 无关位
//	tx_data[2] = (dac_value >> 4) & 0xFF;	// 第3字节：DAC数据高8位（D11-D4）
	//本次实验仅使用0，0模式。
	tx_data[0] = (0x00 & 0xF0)|((dac_value >> 8) & 0x0F);	// 第2字节：命令 与 dac_value的高4位
	tx_data[1] = (dac_value & 0xFF)							// 第3字节：低8位
	
	I2C_Stop();
	
}

void MCP_ReceiveData(uint8_t dac_value, uint8_t power_mode)
{
	
	I2C_Start();
	I2C_SendByte(0xC5);		//从机地址0x62+1(读)=0xC5
	
	if(0xC5 &= 0x01) I2C_SendByte(0x00)	//0（未完成），0（不重制），×，×，×，0，0（普通模式），0
	
	I2C_Stop();
}
