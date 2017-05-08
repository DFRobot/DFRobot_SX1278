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
#include "DFRobot_LoRo.h"


DFRobot_LoRo::DFRobot_LoRo() :
	_spiSettings(5E6, MSBFIRST, SPI_MODE0),
	_ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
	_frequency(0),
	_packetIndex(0),
	_implicitHeaderMode(0),
	_onReceive(NULL)
{
}

int DFRobot_LoRo::begin(long frequency, uint8_t power, uint8_t sf, bool crc, uint8_t bw, uint8_t cr, bool hm)
{
	_frequency 	= frequency;
	_power		= power;
	
	_sf			= sf;
	_crc		= crc;
	
	_bw			= bw;
	_cr			= cr;
	_hm			= hm;

	// setup pins
	pinMode(_ss, OUTPUT);
	pinMode(_reset, OUTPUT);

	// perform reset
	digitalWrite(_reset, LOW);
	delay(10);
	digitalWrite(_reset, HIGH);
	delay(10);

	// set SS high
	digitalWrite(_ss, HIGH);

	// start SPI
	SPI.begin();/*
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV32); // 500KHz
	SPI.setDataMode(SPI_MODE0);*/

	// check version
	uint8_t version = readRegister(REG_VERSION);
	if (version != 0x12) {
		return 0;
	}

	loraConfig();
	//loraConfig_test();

	return 1;
}

//---------------------------------
// Supply voltage = 3.3 V
// Temperature = 25° C
// f XOSC = 32 MHz
// bandwidth (BW) = 125 kHz
// Spreading Factor (SF) = 12
// Error Correction Code (EC) = 4/6
// Packet Error Rate (PER)= 1%
// CRC on payload enabled
// Output power = 13 dBm in transmission
// Payload length = 64 bytes
// Preamble Length = 12 symbols (programmed register PreambleLength=8)
// With matched impedances
//---------------------------------

void DFRobot_LoRo::loraConfig()
{
	// put in sleep mode
	sleep();

	// set frequency
	setFrequency();
	
	// High Power Settings
	writeRegister(REG_PA_DAC, 0x87); // high power
	// set output power to 17 dBm
	setTxPower(_power);
	
	// Over Current Protection control
	writeRegister(REG_OCP, 0x2b);//setOCP
	
	// set LNA boost
	writeRegister(REG_LNA, 0x23);
	// set auto AGC
	writeRegister(REG_MODEM_CONFIG_3, 0x04);

	// set base addresses
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

	setSpreadingFactor(_sf);
	setPacketCRCCheck(_crc);
	
	setBandWidth(_bw);
	setErrorCodingRate(_cr);
	setHeaderMode(_hm);
	
	// set symb timeout
	writeRegister(REG_SYMB_TIMEOUT_LSB, 0x64);
	
	// set preamble length
	setPreambleLength();
	
	// put in standby mode
	idle();
}
/*
void DFRobot_LoRo::loraConfig()
{
	setOpMode(0x00);//sleep mode
	setRFMode(0x80);//lora mode
	
	// set carrier frequency
	setFrequency();
	
	// set output power to 17 dBm
	setTxPower(17);
	// High Power Settings
	writeRegister(REG_PA_DAC, 0x87); // high power
	// Over Current Protection control
	writeRegister(REG_OCP, 0x2b);//setOCP
	// set LNA boost
	writeRegister(REG_LNA, 0x23);
	
	
	setSpreadingFactor(12);
	// single TX
	writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0xf7) | (0x0 << 3));
	setPacketCRCCheck(true);
	writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & ~0x01);
	
	setBandWidth(7);
	setErrorCodingRate(2);
	setHeaderMode(true);
	
	writeRegister(REG_DETECTION_OPTIMIZE, 0x05);
	writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
	
	// set symb timeout
	writeRegister(REG_SYMB_TIMEOUT_LSB, 0x64);
	// set preamble length
	setPreambleLength();
	
	// set base addresses
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
	
	setOpMode(0x01);//standby mode
}
*/
void DFRobot_LoRo::sendPackage(const uint8_t *buffer, size_t size)
{
	setOpMode(0x01);//standby mode
	
	// High Power Settings
	writeRegister(REG_PA_DAC, 0x87); // high power
	writeRegister(REG_LR_HOPPERIOD, 0x0);// frequency Hopping off
	writeRegister(REG_DIO_MAPPING_1, (readRegister(REG_DIO_MAPPING_1) & 0x3f) | (0x01 << 6));
	clearIRQFlags();
	writeRegister(REG_LR_IRQFLAGSMASK, 0xf7);
	
	writeRegister(REG_PAYLOAD_LENGTH, 1 + size);
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	// reset FIFO address
	writeRegister(REG_FIFO_ADDR_PTR, 0);
	
	writeData(buffer, size);
	
	writeRegister(REG_DIO_MAPPING_1,0x40);  
	writeRegister(REG_DIO_MAPPING_2,0x00);
	setOpMode(0x03);
	
	//while(!digitalRead(_dio0));
	//while(Irq_flag&0x08 != 0x08)  //xx
	while((readRegister(REG_IRQ_FLAGS)&0x08) != 0x08);
	clearIRQFlags();
}

void DFRobot_LoRo::reservePackage(char *buffer)
{
	setOpMode(0x01);//standby mode
	int packetLength = 0;
	
	// High Power Settings
	writeRegister(REG_PA_DAC, 0x84); // default
	writeRegister(REG_LR_HOPPERIOD, 0xff);// frequency Hopping max
	writeRegister(REG_DIO_MAPPING_1, (readRegister(REG_DIO_MAPPING_1) & 0x3f) | (0x0 << 6));
	clearIRQFlags();
	writeRegister(REG_LR_IRQFLAGSMASK, 0xcf);
	
	writeRegister(REG_FIFO_RX_BASE_ADDR, 0x00);
	// reset FIFO address
	writeRegister(REG_FIFO_ADDR_PTR, 0);
	
	setOpMode(MODE_RX_CONTINUOUS);
	//while(!digitalRead(_dio0));
	
	//if(readRegister(REG_IRQ_FLAGS)){
		//return;
	//}
	
	packetLength = readRegister(REG_RX_NB_BYTES);
	
	readData(buffer, (size_t)packetLength);
}

void DFRobot_LoRo::setContinueReceive()
{
	setOpMode(0x01);
	writeRegister(0x11, 0x9f);
	writeRegister(0x24, 0xff);
	writeRegister(0x40, 0x00);
	writeRegister(0x41, 0x00);
	setOpMode(0x05);
}

void DFRobot_LoRo::setSymbTimeout(unsigned int value )
{
	unsigned char RECVER_DAT[2];
	RECVER_DAT[0]=readRegister( 0x1e );    
	RECVER_DAT[1]=readRegister( 0x1f );  
	RECVER_DAT[0] = ( RECVER_DAT[0] & 0xfc ) | ( ( value >> 8 ) & ~0xfc );
	RECVER_DAT[1] = value & 0xFF;
	writeRegister( 0x1e, RECVER_DAT[0]);
	writeRegister( 0x1f, RECVER_DAT[1]);
}

void DFRobot_LoRo::setOpMode(uint8_t opMode)
{
	uint8_t opModePrev;
	opModePrev = readRegister(0x01);  
	opModePrev &= 0xf8;  
	opModePrev |= (uint8_t)opMode; 
	writeRegister( 0x01, opModePrev);
}

void DFRobot_LoRo::setRFMode( uint8_t opMode )
{
	uint8_t opModePrev;
	opModePrev=readRegister(0x01); 
	opModePrev &=0x7F; 
	opModePrev |= (uint8_t)opMode;  
	writeRegister( 0x01, opModePrev); 	
}

void DFRobot_LoRo::ProcessSend(const uint8_t *txbuffer, size_t size)
{
	idle();
	
	writeRegister(REG_LR_HOPPERIOD, 0);
	writeRegister(REG_LR_IRQFLAGSMASK, 0xf7);
	writeRegister(REG_PAYLOAD_LENGTH, size);
	//writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(REG_FIFO_ADDR_PTR, 0);
	
	int currentLength = readRegister(REG_PAYLOAD_LENGTH);
	// write data
	for (size_t i = 0; i < size; i++) {
		writeRegister(0x80, txbuffer[i]);
	}
	
	writeRegister(REG_DIO_MAPPING_1, 0x40);
	writeRegister(REG_DIO_MAPPING_2, 0x00);
	
	// put in TX mode
	setOpMode(MODE_TX);

	// wait for TX done
	while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);
	
	setContinueReceive();
}

void DFRobot_LoRo::ProcessRecv(char *rxbuffer)
{
	uint8_t packetLength = 0;
	int irqFlags = readRegister(REG_IRQ_FLAGS);
	
	if((irqFlags&0x40)==0x40){
		if(readRegister(REG_RX_NB_BYTES)){
			while (available()) {
				rxbuffer[packetLength++] = (char)read();
			}
		}
	}else if((irqFlags&0x08)==0x08){
		setOpMode(0x01);
		writeRegister(REG_LR_IRQFLAGSMASK, 0x9f);
		writeRegister(REG_LR_HOPPERIOD, 0xff);
		writeRegister(REG_DIO_MAPPING_1, 0x00);
		writeRegister(REG_DIO_MAPPING_2, 0x00);
		setOpMode(0x05);
	}else{
		setOpMode(0x01);
		writeRegister(REG_LR_IRQFLAGSMASK, 0x9f);
		writeRegister(REG_LR_HOPPERIOD, 0xff);
		writeRegister(REG_DIO_MAPPING_1, 0x02);
		writeRegister(REG_DIO_MAPPING_2, 0x00);
		setOpMode(0x05);
	}
	
	writeRegister(REG_IRQ_FLAGS, 0xff);
}

void DFRobot_LoRo::cycleSendReceive(char *rxbuffer, const uint8_t *txbuffer, size_t size)
{
	int i;
	for(i=0; i<3; i++)
		ProcessSend(txbuffer, size);
	ProcessRecv(rxbuffer);
}

void DFRobot_LoRo::end()
{
	// put in sleep mode
	sleep();

	// stop SPI
	SPI.end();
}

int DFRobot_LoRo::beginPacket()
{
	// put in standby mode
	idle();
	setOpMode(0x01);

	// reset FIFO address and paload length
	writeRegister(REG_FIFO_ADDR_PTR, 0);
	writeRegister(REG_PAYLOAD_LENGTH, 0);
	
	// High Power Settings
	writeRegister(REG_PA_DAC, 0x87); // high power

	return 1;
}

int DFRobot_LoRo::endPacket()
{
	// put in TX mode
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

	// wait for TX done
	while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

	// clear IRQ's
	writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

	return 1;
}

int DFRobot_LoRo::parsePacket()
{
	int packetLength = 0;
	int irqFlags = readRegister(REG_IRQ_FLAGS);

	// clear IRQ's
	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		// received a packet
		_packetIndex = 0;

		// read packet length
		if (_implicitHeaderMode) {
			packetLength = readRegister(REG_PAYLOAD_LENGTH);
		} else {
			packetLength = readRegister(REG_RX_NB_BYTES);
		}

		// set FIFO address to current RX address
		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

		// put in standby mode
		idle();
	} else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
		// not currently in RX mode

		// reset FIFO address
		writeRegister(REG_FIFO_ADDR_PTR, 0);

		// put in single RX mode
		writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}

	return packetLength;
}

int DFRobot_LoRo::packetRssi()
{
	return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float DFRobot_LoRo::packetSnr()
{
	return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

size_t DFRobot_LoRo::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t DFRobot_LoRo::write(const uint8_t *buffer, size_t size)
{
	int currentLength = readRegister(REG_PAYLOAD_LENGTH);

	// check size
	if ((currentLength + size) > MAX_PKT_LENGTH) {
		size = MAX_PKT_LENGTH - currentLength;
	}

	// write data
	for (size_t i = 0; i < size; i++) {
		writeRegister(REG_FIFO, buffer[i]);
	}

	// update length
	writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

	return size;
}

int DFRobot_LoRo::available()
{
	idle();
	setOpMode(0x01);
	return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int DFRobot_LoRo::read()
{
	if (!available()) {
		return -1;
	}

	_packetIndex++;

	return readRegister(REG_FIFO);
}

int DFRobot_LoRo::peek()
{
	if (!available()) {
		return -1;
	}

	// store current FIFO address
	int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

	// read
	uint8_t b = readRegister(REG_FIFO);

	// restore FIFO address
	writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

	return b;
}

void DFRobot_LoRo::flush()
{
}

void DFRobot_LoRo::onReceive(void(*callback)(int))
{
	_onReceive = callback;

	if (callback) {
		writeRegister(REG_DIO_MAPPING_1, 0x00);

		attachInterrupt(digitalPinToInterrupt(_dio0), DFRobot_LoRo::onDio0Rise, RISING);
	} else {
		detachInterrupt(digitalPinToInterrupt(_dio0));
	}
}

void DFRobot_LoRo::receive(int size)
{
	if (size > 0) {
		implicitHeaderMode();

		writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	} else {
		explicitHeaderMode();
	}

	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void DFRobot_LoRo::idle()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void DFRobot_LoRo::sleep()
{
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
	writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void DFRobot_LoRo::setTxPower(int level, int outputPin)
{
	if (PA_OUTPUT_RFO_PIN == outputPin) {
		// RFO
		if (level < 0) {
			level = 0;
		} else if (level > 14) {
			level = 14;
		}

		writeRegister(REG_PA_CONFIG, 0x70 | level);
	} else {
		// PA BOOST
		if (level < 2) {
			level = 2;
		} else if (level > 17) {
			level = 17;
		}

		writeRegister(REG_PA_CONFIG, 0x70 | PA_BOOST | (level - 2));
	}
}

void DFRobot_LoRo::setFrequency()
{
	writeRegister(REG_FRF_MSB, 0x6c);
	writeRegister(REG_FRF_MID, 0x80);
	writeRegister(REG_FRF_LSB, 0x00);
}

void DFRobot_LoRo::setSpreadingFactor(int sf)
{
	uint8_t value;
	value=readRegister(REG_MODEM_CONFIG_2);
	value &=0x0f; 
	value |= (uint8_t)(sf << 4);
	writeRegister(REG_MODEM_CONFIG_2, value);
}

void DFRobot_LoRo::setBandWidth(uint8_t bw)
{
	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void DFRobot_LoRo::setErrorCodingRate(int cr)
{
	writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void DFRobot_LoRo::setPreambleLength()
{
	writeRegister(REG_PREAMBLE_MSB, 0x00);
	writeRegister(REG_PREAMBLE_LSB, 0x08);
}

void DFRobot_LoRo::clearIRQFlags() {
  writeRegister(REG_IRQ_FLAGS, 0xff);
}

void DFRobot_LoRo::setSyncWord(int sw)
{
	writeRegister(REG_SYNC_WORD, sw);
}

void DFRobot_LoRo::setPacketCRCCheck(bool crc)
{
	if(crc) {
		// Enable CRC generation and check on payload
		writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | (0x1 << 2));
	} else {
		// Disable CRC generation and check on payload
		writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & ~(0x1 << 2));
	}
}

byte DFRobot_LoRo::random()
{
	return readRegister(REG_RSSI_WIDEBAND);
}

void DFRobot_LoRo::setPins(int ss, int reset, int dio0)
{
	_ss = ss;
	_reset = reset;
	_dio0 = dio0;
}

void DFRobot_LoRo::setSPIFrequency(uint32_t frequency)
{
	_spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void DFRobot_LoRo::dumpRegisters(Stream& out)
{
	for (int i = 0; i < 128; i++) {
		out.print("0x");
		out.print(i, HEX);
		out.print(": 0x");
		out.println(readRegister(i), HEX);
	}
}

void DFRobot_LoRo::explicitHeaderMode()
{
	_implicitHeaderMode = 0;

	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & ~0x01);
}

void DFRobot_LoRo::implicitHeaderMode()
{
	_implicitHeaderMode = 1;

	writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void DFRobot_LoRo::setHeaderMode(bool hm)
{
	if(hm) {
		explicitHeaderMode();
	} else {
		implicitHeaderMode();
	}
}

void DFRobot_LoRo::handleDio0Rise()
{
	int irqFlags = readRegister(REG_IRQ_FLAGS);

	// clear IRQ's
	writeRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		// received a packet
		_packetIndex = 0;

		// read packet length
		int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

		// set FIFO address to current RX address
		writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

		if (_onReceive) {
			_onReceive(packetLength);
		}

		// reset FIFO address
		writeRegister(REG_FIFO_ADDR_PTR, 0);
	}
}

void DFRobot_LoRo::readData(char *buffer, size_t size)
{
	uint8_t i;
	
	digitalWrite(_ss, LOW);
	SPI.beginTransaction(_spiSettings);
	SPI.transfer(REG_FIFO | 0x0);
	for(i=0; i<size; i++){
		buffer[i] = SPI.transfer(0x0);
	}
	SPI.endTransaction();
	digitalWrite(_ss, HIGH);
}

void DFRobot_LoRo::writeData(const uint8_t *buffer, size_t size)
{
	uint8_t i;
	
	digitalWrite(_ss, LOW);
	SPI.beginTransaction(_spiSettings);
	SPI.transfer(REG_FIFO | 0x80);
	for(i=0; i<size; i++){
		SPI.transfer(buffer[i]);
	}
	SPI.endTransaction();
	digitalWrite(_ss, HIGH);
}

uint8_t DFRobot_LoRo::readRegister(uint8_t address)
{
	uint8_t response;

	digitalWrite(_ss, LOW);

	SPI.beginTransaction(_spiSettings);
	SPI.transfer(address & 0x7f);
	response = SPI.transfer(0x00);
	SPI.endTransaction();

	digitalWrite(_ss, HIGH);

	return response;
}

void DFRobot_LoRo::writeRegister(uint8_t address, uint8_t value)
{
	digitalWrite(_ss, LOW);

	SPI.beginTransaction(_spiSettings);
	SPI.transfer(address | 0x80);
	SPI.transfer(value);
	SPI.endTransaction();

	digitalWrite(_ss, HIGH);
}

void DFRobot_LoRo::onDio0Rise()
{
	LoRa.handleDio0Rise();
}

DFRobot_LoRo LoRa;
