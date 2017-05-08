/*!
 * @file DFRobot_LoRo.h
 * @brief LORO
 * @n lora mode
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yangyang]
 * @version  V1.0
 * @date  2017-04-10
 */
#ifndef __DFRobot_LoRo_H__
#define __DFRobot_LoRo_H__

#include <Arduino.h>
#include <SPI.h>

#define LORA_DEFAULT_SS_PIN    	D2 // 25
#define LORA_DEFAULT_RESET_PIN 	D3 // 26
#define LORA_DEFAULT_DIO0_PIN  	D4 // 27

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP			 		 0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_LR_IRQFLAGSMASK      0x11
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_SYMB_TIMEOUT_LSB  	 0x1f
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_LR_PAYLOADMAXLENGTH  0x23 
#define REG_LR_HOPPERIOD         0x24 
#define REG_LR_FIFORXBYTEADDR    0x25
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_DIO_MAPPING_2        0x41
#define REG_VERSION              0x42
#define REG_PA_DAC		 		 0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255


class DFRobot_SX1278 : public Stream {
public:
	DFRobot_SX1278();

	int begin(long frequency, uint8_t power=17, uint8_t sf=12, bool crc=true, uint8_t bw=7, uint8_t ecr=2, bool hm=true);
	void end();

	void sendPackage(const uint8_t *buffer, size_t size);
	void reservePackage(char *buffer);
	
	int beginPacket();
	int endPacket();

	int parsePacket();
	int packetRssi();
	float packetSnr();

	virtual int read();
	virtual size_t write(uint8_t byte);
	virtual size_t write(const uint8_t *buffer, size_t size);

	virtual int available();
	virtual int peek();
	virtual void flush();

	void onReceive(void(*callback)(int));

	void receive(int size = 0);
	void idle();
	void sleep();

	void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
	void setFrequency();
	
	void setSpreadingFactor(int sf);
	void setPacketCRCCheck(bool crc);
	
	void setBandWidth(uint8_t bw);
	void setErrorCodingRate(int cr);
	void setHeaderMode(bool hm);
	
	void setPreambleLength();
	
	void setSyncWord(int sw);
	
	void setOpMode(uint8_t opMode);

	byte random();

	void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
	void setSPIFrequency(uint32_t frequency);

	void dumpRegisters(Stream& out);
	
	void cycleSendReceive(char *rxbuffer, const uint8_t *txbuffer, size_t size);

private:
	void loraConfig();
	
	//void loraConfig_test();
	void setContinueReceive();
	void setSymbTimeout(unsigned int value );
	void setRFMode( uint8_t opMode );
	void ProcessSend(const uint8_t *txbuffer, size_t size);
	void ProcessRecv(char *rxbuffer);
	
	void explicitHeaderMode();
	void implicitHeaderMode();
	
	
	void clearIRQFlags();

	void handleDio0Rise();
	
	void readData(char *buffer, size_t size);
	void writeData(const uint8_t *buffer, size_t size);

	uint8_t readRegister(uint8_t address);
	void writeRegister(uint8_t address, uint8_t value);

	static void onDio0Rise();

	int _ss;
	int _reset;
	int _dio0;
	int _packetIndex;
	int _implicitHeaderMode;
	void (*_onReceive)(int);
	
	long _frequency;
	uint8_t _power;
	uint8_t _sf;
	bool _crc;
	uint8_t _bw;
	uint8_t _cr;
	bool _hm;

	SPISettings _spiSettings;
};

extern DFRobot_SX1278 LoRa;

#endif
