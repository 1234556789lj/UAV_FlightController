#include "Int_SI24R1.h"

// 定义一个静态地址，供发送和接收使用，地址长度为5字节
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01};

// hal库实现SPI读写函数，读写一个字节
static uint8_t SPI_RW(uint8_t byte)
{
	uint8_t rx_data = 0;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1, 0xFF);
	return rx_data;
}

/********************************************************
写入寄存器值，返回状态值，单个字节写入
reg:寄存器地址，格式为SI24R1_WRITE_REG|reg
value:要写入寄存器的值
*********************************************************/
uint8_t Int_SI24R1_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CS_LOW;
	status = SPI_RW(reg);
	SPI_RW(value);
	CS_HIGH;

	return (status);
}

/********************************************************
写入缓冲区数据，返回状态值，多个字节写入
reg:寄存器地址，格式为SI24R1_WRITE_REG+reg
					pBuf:写入数据的地址
					bytes:写入的字节数
*********************************************************/
uint8_t Int_SI24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status, byte_ctr;

	CS_LOW;
	status = SPI_RW(reg);
	for (byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
	{
		SPI_RW(*pBuf++);
	}
	CS_HIGH;

	return (status);
}

/********************************************************
读取寄存器值，返回状态值，单个字节读取
reg:寄存器地址，格式为SI24R1_READ_REG|reg
  ֵĴֵ
*********************************************************/
uint8_t Int_SI24R1_Read_Reg(uint8_t reg)
{
	uint8_t value;

	CS_LOW;
	SPI_RW(reg);
	value = SPI_RW(0);
	CS_HIGH;

	return (value);
}

/********************************************************
写入缓冲区数据，返回状态值，多个字节读取
reg:寄存器地址，格式为SI24R1_READ_REG+reg
					pBuf:读取数据的地址
					bytes:读取的字节数
*********************************************************/
uint8_t Int_SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status, byte_ctr;

	CS_LOW;
	status = SPI_RW(reg);
	for (byte_ctr = 0; byte_ctr < bytes; byte_ctr++)
		pBuf[byte_ctr] = SPI_RW(0);
	CS_HIGH;

	return (status);
}

/********************************************************
函数功能：SI24R1接收模式初始化
入口参数：无
返回  值：无
*********************************************************/
void Int_SI24R1_RX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 接收通道0地址
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);						   // 使能通道0自动应答
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);					   // 使能接收通道0
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, CHANNEL_RF);					   // 射频信道（避开WiFi频段）
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);			   // 接收通道0有效数据宽度
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, RF_CONFIG);				   // 数据速率+发射功率
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0f);						   // CRC使能，16位CRC校验，上电，接收模式
	CE_HIGH; // 拉高CE启动接收设备
}

/********************************************************
函数功能：SI24R1发送模式初始化
入口参数：无
返回  值：无
*********************************************************/
void Int_SI24R1_TX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);	   // 写入发送地址
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 接收通道0地址与发送地址相同（用于接收ACK）

	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);	   // 使能通道0自动应答
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);  // 使能接收通道0
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + SETUP_RETR, 0x2a); // 自动重发：500us+86us延迟，重试10次
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, CHANNEL_RF);	   // 射频信道
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, RF_CONFIG);   // 数据速率+发射功率
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0e);	   // CRC使能，16位CRC校验，上电，发送模式
	CE_HIGH;
}

/********************************************************
函数功能：读取接收数据
入口参数：rxbuf:接收数据存放首地址
返回  值：0:接收到数据
		  1:没有接收到数据
*********************************************************/
uint8_t Int_SI24R1_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	state = Int_SI24R1_Read_Reg(STATUS); // 读取状态寄存器

	if (state & RX_DR) // 接收到数据
	{
		Int_SI24R1_Read_Buf(SI24R1_READ_REG + RD_RX_PLOAD, rxbuf, TX_PLOAD_WIDTH); // 读取payload
		Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, RX_DR);					   // 仅清除RX_DR中断标志
		Int_SI24R1_Write_Reg(FLUSH_RX, 0xff);									   // 清除RX FIFO

		return 0;
	}
	// 未收到数据，也清除可能残留的中断标志
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, RX_DR | TX_DS | MAX_RT);
	return 1;
}

/********************************************************
函数功能：发送一个数据包
入口参数：txbuf:要发送的数据
返回  值：0发送成功，1发送失败
*********************************************************/
uint8_t Int_SI24R1_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	uint16_t timeout = 1000; // 超时计数

	CE_LOW;
	Int_SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); // 写入TX FIFO
	CE_HIGH;												  // CE置高，启动发送

	/* 等待发送完成或达到最大重发次数 */
	state = Int_SI24R1_Read_Reg(STATUS);
	while (((state & TX_DS) == 0) && ((state & MAX_RT) == 0) && (--timeout))
	{
		vTaskDelay(1);
		state = Int_SI24R1_Read_Reg(STATUS);
	}

	/* 超时处理 */
	if (timeout == 0)
	{
		CE_LOW;
		Int_SI24R1_Write_Reg(FLUSH_TX, 0xff);
		CE_HIGH;
		Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, MAX_RT);
		return 1;
	}

	/* 清除本次发送相关的中断标志 */
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, state & (TX_DS | MAX_RT));

	if (state & MAX_RT) // 达到最大重发次数
	{
		CE_LOW;
		Int_SI24R1_Write_Reg(FLUSH_TX, 0xff);
		CE_HIGH;
		return 1;
	}

	if (state & TX_DS) // 发送成功
	{
		return 0;
	}

	return 1;
}

/********************************************************

*********************************************************/
uint8_t read_buf[5] = {0};

/*
函数功能：初始化SI24R1模块
入口参数：无
返回  值：return 0:初始化成功
			  return 1:初始化失败
*/
uint8_t Int_SI24R1_Check(void)
{

	// si24r1芯片需要先读取一次，保证spi正常后再写入
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, read_buf, TX_ADR_WIDTH);		// 从TX_ADDR寄存器读取数据到read_buf
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH); // 写入测试数据到TX_ADDR寄存器
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, read_buf, TX_ADR_WIDTH);		// 从TX_ADDR寄存器读取数据到read_buf
	for (uint8_t i = 0; i < TX_ADR_WIDTH; i++)
	{
		if (read_buf[i] != TX_ADDRESS[i])
		{
			return 1;
		}
	}

	return 0;

	// 串口打印读写结果，检查是否一致
	// debug_printf("buff:%02X %02X %02X %02X %02X\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4]);
}

void Int_SI24R1_Init(void)
{
	// 上电后芯片延迟（此处运行在 main()，调度器尚未启动，必须用 HAL_Delay）
	HAL_Delay(200);
	// 校验检测
	while (Int_SI24R1_Check() == 1)
	{
		// 每两次检测间隔10ms
		HAL_Delay(10);
	}
	// 初始化完成，设置默认模式为接收
	Int_SI24R1_RX_Mode();
	debug_printf("SI24R1 init success\n");
}
