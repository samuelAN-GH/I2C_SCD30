#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "ustimer.h"
//#include "SCD30.h"

#define I2C_TXBUFFER_SIZE   3      // Max transmit buffer size for write cmds (bytes)
#define I2C_RXBUFFER_SIZE   18      // Max transmit buffer size for write cmds (bytes)

#define SCD30_ADDRESS                 0xC2            // 0x61-R = 0110 0001 - 0 = 1100 0010 = 0xC2 = full 8 bits I2C address (includes end R/nW bit)
#define SCD30_CMD_GET_FW_REV          {0xD1, 0x00}    // [WRITEcmd + READ] Read FW version
#define SCD30_CMD_SOFT_RST            {0xD3, 0x04}    // [WRITE, no arg] Restarts the sensor
#define SCD30_CMD_START_CONT_MEAS     {0x10}          // [WRITE, arg = uint 700...1400 mbar] Trigger continuous measurement with ambiant pressure compensation
#define SCD30_CMD_STOP_CONT_MEAS      {0x01, 0x04}    // [WRITE, no arg] Stop continuous measurement
#define SCD30_CMD_SET_GET_INTERVAL    {0x46, 0x00}    // [WRITEcmd + arg = uint 2...1800 secs // WRITEcmd + READ] Set measurement interval
#define SCD30_CMD_GET_DATA_RDY        {0x02, 0x02}    // [WRITEcmd + READ] Check if data buffer is ready
#define SCD30_CMD_GET_MEAS            {0x03, 0x00}    // [WRITEcmd + READ] Read CO2 value, humidity, temperature (! check DATA_RDY == 1 before use)
#define SCD30_CMD_SET_GET_ASC         {0x53, 0x06}    // [WRITEcmd + arg = 0x0...1 // WRITEcmd + READ] (Des-)activate automatic self calibration (activate = 1)
#define SCD30_CMD_SET_GET_FRC         {0x52, 0x04}    // [WRITEcmd + arg = uint 0...2000 ppm // WRITEcmd + READ] Forced recalibration value
#define SCD30_CMD_SET_GET_TEMP_OFFSET {0x54, 0x03}    // [WRITEcmd + arg = uint x0.01°C // WRITEcmd + READ] Set temperature offset
#define SCD30_CMD_SET_GET_ALT_COMP    {0x54, 0x03}    // [WRITEcmd + arg = uint meters over sea // WRITEcmd + READ] Altitude compensation

#define I2C_FREQ 50000;             // Recommanded value for SCD30

//////////////////////////////////////////////////////////////////////

//GLOBAL VARIABLES

// debug
 //static volatile uint16_t val;
 static volatile float co2;
 static volatile float temp;
 static volatile float hum;
 static volatile uint8_t debug_rxbuffer0;
 static volatile uint16_t debug_rxbuffer;
 static volatile uint16_t debug_txbuffer;
 static volatile uint8_t leng;
// static volatile I2C_TransferReturn_TypeDef debug_result;
 static volatile I2C_TransferSeq_TypeDef debug_i2cTransfer;

// Buffer definition (uninitialized)
uint8_t i2c_txBuffer[I2C_TXBUFFER_SIZE]; // Array of I2C_TXBUFFER_SIZE bytes
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE]; // Array of I2C_TXBUFFER_SIZE bytes

// Transmission flag ==1 if transmission on-going
volatile bool i2c_startTx;

//////////////////////////////////////////////////////////////////////

// CONF. FUNCTIONS

// Clock Management Unit initialization
void initCMU(void)
 {
   // Enable clocks to the I2C and GPIO
   // CMU_ClockEnable(cmuClock_I2C0, true); // (NOT mandatory on EFRE32xG21)
   // CMU_ClockEnable(cmuClock_GPIO, true); // (NOT mandatory on EFRE32xG21)
 }

void initGPIO(void)
{
  // Configure BUTTON0(PB0) as input and interrupt
  GPIO_PinModeSet(gpioPortB, 0, gpioModeInputPull, 1);
  GPIO_ExtIntConfig(gpioPortB, 0, 0, false, true, true);

  // Configure LED0(PD02) and LED1(PD03) as output
  GPIO_PinModeSet(gpioPortD, 2, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 0);

  // Enable interface of SI7021 sensor through PD04
  GPIO_PinModeSet(gpioPortD, 4, gpioModePushPull, 0);

  // Enable EVEN (thus including PB0) interrupt to catch button press that starts I2C transfer
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void initI2C(void)
{
  // Use default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  i2cInit.freq = I2C_FREQ;

  // Using PC02 (SDA) and PC00 (SCL)
  GPIO_PinModeSet(gpioPortC, 2, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);

  // Route I2C pins to GPIO
  GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
                        | (gpioPortC << _GPIO_I2C_SDAROUTE_PORT_SHIFT
                        | (2 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
                        | (gpioPortC << _GPIO_I2C_SCLROUTE_PORT_SHIFT
                        | (0 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
  GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

  // Initialize the I2C
  I2C_Init(I2C0, &i2cInit);

  // Set the status flags and index
  i2c_startTx = false;

  // Enable automatic STOP on NACK
  I2C0->CTRL = I2C_CTRL_AUTOSN;
}

void I2C_RequestAndReadback(uint8_t slaveAddress, uint8_t *requestCmd, uint8_t numBytesCmd, uint8_t *rxBuff, uint8_t numBytesRx)
{
  // Make request to slave and readback result (Hold Master Mode -- with clock stretch during measurement)
  // Data stored in rxBuff using its pointer
  // numBytes = number of bytes of the data received from slave sensor
  // WARNING : slaveAddress is the full 8 bits I2C address including the READ/nWRITE bit (7bits address + R/nW bit)

  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initialize I2C transfer
  i2cTransfer.addr          = slaveAddress;
  i2cTransfer.flags         = I2C_FLAG_WRITE_READ; // must write requestCmd before reading response
  i2cTransfer.buf[0].data   = requestCmd;   // must be a pointer
  i2cTransfer.buf[0].len    = numBytesCmd;  //bytes
  i2cTransfer.buf[1].data   = rxBuff;
  i2cTransfer.buf[1].len    = numBytesRx;

  // debug_i2cTransfer = i2cTransfer;

  result = I2C_TransferInit(I2C0, &i2cTransfer); // returns the status of on-going transfer, i2cTransferInProgress if transfer not finished

  // I2C_Transfer continues the transfer initiated by I2C_TransferInit
  // Returns TransferDone if transfer successful, i2cTransferInProgress if transfer not finished
  while (result == i2cTransferInProgress) {
    result = I2C_Transfer(I2C0);
  }

  // Error occured during transfer
  if (result != i2cTransferDone) {
    // LED1 ON to indicate I2C transmission problem
    GPIO_PinOutSet(gpioPortD, 3);
  }

  i2c_startTx = false;
//   rxbuffer = rxBuff;
//   debug_result = result;
}

void I2C_Read(uint8_t slaveAddress, uint8_t *rxBuff, uint8_t numBytes)
{
  // Simply read data and flush into rxBuff

  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initialize I2C transfer
  i2cTransfer.addr          = slaveAddress;
  i2cTransfer.flags         = I2C_FLAG_READ;  // must write requestCmd before reading response
  i2cTransfer.buf[0].data   = rxBuff;       // rxBuffer = data[numBytes]
  i2cTransfer.buf[0].len    = numBytes;       // numBytes is < I2C_TXBUFFER_SIZE


  // debug_i2cTransfer = i2cTransfer;

  result = I2C_TransferInit(I2C0, &i2cTransfer); // returns the status of on-going transfer, i2cTransferInProgress if transfer not finished

  // I2C_Transfer continues the transfer initiated by I2C_TransferInit
  // Returns TransferDone if transfer successful, i2cTransferInProgress if transfer not finished
  while (result == i2cTransferInProgress) {
    result = I2C_Transfer(I2C0);

  }


  leng = sizeof(rxBuff);

  // Error occured during transfer
  if (result != i2cTransferDone) {
    // LED1 ON to indicate I2C transmission problem
    GPIO_PinOutSet(gpioPortD, 3);
  }

  i2c_startTx = false;
//   rxbuffer = rxBuff;
//   debug_result = result;
}

void I2C_WriteCmd(uint8_t slaveAddress, uint8_t *writeCmd, uint8_t numBytesCmd)
{
  // Write data without args (only writeCmd)

  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initialize I2C transfer
  i2cTransfer.addr          = slaveAddress;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = writeCmd;     // txBuffer = writeCmd[1byte] + data[numBytes]
  i2cTransfer.buf[0].len    = numBytesCmd; // numBytes is < I2C_TXBUFFER_SIZE
  i2cTransfer.buf[1].data   = NULL;     // txBuffer = writeCmd[1byte] + data[numBytes]
  i2cTransfer.buf[1].len    = 0; // numBytes is < I2C_TXBUFFER_SIZE

  //debug_i2cTransfer = i2cTransfer;
  debug_i2cTransfer = i2cTransfer;
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Send data
  while (result == i2cTransferInProgress) {
    result = I2C_Transfer(I2C0);
  }



  if (result != i2cTransferDone) {
    // LED1 ON and infinite while loop to indicate I2C transmission problem
    GPIO_PinOutSet(gpioPortD, 3);
  }
}

void I2C_WriteCmdArgs(uint8_t slaveAddress, uint8_t *writeCmd, uint8_t *settingValue, uint8_t numBytesCmd, uint8_t numBytesData)
{
  // Write data with following args (data), of length numBytesData

  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;


  uint8_t txBuffer[numBytesCmd + numBytesData];  // 1 byte is added in order to add the writeCmd prior data to write

  //txBuffer[0] = writeCmd;   // 1st byte of txBuffer is the writeCmd
  for(int i = 0; i < (numBytesCmd); i++)
  {
      txBuffer[i] = writeCmd[i];  // data in txBuff is added after the writeCmd (after the 1st byte)
  }
  for(int i = numBytesCmd; i < (numBytesCmd+numBytesData); i++)
  {
      txBuffer[i] = settingValue[i-numBytesCmd];  // data in txBuff is added after the writeCmd (after the 1st byte)
  }

  // Initialize I2C transfer
  i2cTransfer.addr          = slaveAddress;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = txBuffer;     // txBuffer = writeCmd[1byte] + data[numBytes]
  i2cTransfer.buf[0].len    = (numBytesCmd+numBytesData); // numBytes is < I2C_TXBUFFER_SIZE
  i2cTransfer.buf[1].data   = NULL;
  i2cTransfer.buf[1].len    = 0;


  // Initialize I2C transfer
  //i2cTransfer.addr          = slaveAddress;
  //i2cTransfer.flags         = I2C_FLAG_WRITE_WRITE;
  //i2cTransfer.buf[0].data   = writeCmd;     // txBuffer = writeCmd[1byte] + data[numBytes]
  //i2cTransfer.buf[0].len    = numBytesCmd;  // numBytes is < I2C_TXBUFFER_SIZE
  //i2cTransfer.buf[1].data   = settingValue;         // txBuffer = writeCmd[1byte] + data[numBytes]
  //i2cTransfer.buf[1].len    = numBytesData; // numBytes is < I2C_TXBUFFER_SIZE


  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Send data
  while (result == i2cTransferInProgress) {
    result = I2C_Transfer(I2C0);
  }

  if (result != i2cTransferDone) {
    // LED1 ON and infinite while loop to indicate I2C transmission problem
    GPIO_PinOutSet(gpioPortD, 3);
  }
}

void GPIO_EVEN_IRQHandler(void)
{
  // Clear pending
  uint32_t interruptMask = GPIO_IntGet();
  GPIO_IntClear(interruptMask);

  // Re-enable I2C
  I2C_Enable(I2C0, true);

  i2c_startTx = true;
}

float Hex2Float(uint8_t *hexInput, uint8_t len)
{
    unsigned int binaryValue;

    unsigned char buffer[len];

    buffer[0] = hexInput[0];
    printf("%d\n", buffer[0]);
    binaryValue = ((unsigned int)buffer[0] << (len-1)*8);

    printf("%d\n", binaryValue);
    for (int i = 1; i < len; i++)
    {
        buffer[i] = hexInput[i];
        binaryValue = binaryValue | (((unsigned int)buffer[i]) << (len-i-1)*8);
        printf("%d\n", binaryValue);
    }

    return *(float*)&binaryValue;
}

int main(void)
{
  // Chip errata
  CHIP_Init();

  // Initialize the I2C
  initCMU();
  initGPIO();
  initI2C();

  USTIMER_Init();

  // Enable Si7021 sensor I2C interface by setting PD04
  GPIO_PinOutSet(gpioPortD, 4);

  // i2c_startTx = true if Interrupt on Button0 (IRQHandler)
  while (1) {

    if (i2c_startTx) {
      // Transmitting data

      // uint8_t cmd[2] = SCD30_CMD_SET_GET_INTERVAL;

      uint8_t cmd[2] = SCD30_CMD_GET_MEAS;
      uint8_t numBytesCmd = sizeof(cmd);

      uint8_t intValue[2];
      intValue[0] = 0x00;
      intValue[1] = 0x05;
      intValue[2] = 0x74; //CRC

      // I2C_WriteCmdArgs(SCD30_ADDRESS, cmd, intValue, numBytesCmd, 3);
      // USTIMER_Delay(3000);
      // I2C_WriteCmd(SCD30_ADDRESS, cmd, numBytesCmd);
      // USTIMER_Delay(3000);
      // I2C_Read(SCD30_ADDRESS, i2c_rxBuffer, 3);

      I2C_WriteCmd(SCD30_ADDRESS, cmd, numBytesCmd);
      USTIMER_Delay(3000);
      I2C_Read(SCD30_ADDRESS, i2c_rxBuffer, 18);


      unsigned int tempU32;
      // read data is in a buffer. In case of I2C CRCs have been removed
      // beforehand. Content of the buffer is the following
      unsigned char buffer[4];
      buffer[0] = i2c_rxBuffer[12]; // MMSB CO2
      buffer[1] = i2c_rxBuffer[13]; // MLSB CO2
      buffer[2] = i2c_rxBuffer[15]; // LMSB CO2
      buffer[3] = i2c_rxBuffer[16]; // LLSB CO2
      // cast 4 bytes to one unsigned 32 bit integer
      tempU32 = (unsigned int)((((unsigned int)buffer[0]) << 24) |
      (((unsigned int)buffer[1]) << 16) |
      (((unsigned int)buffer[2]) << 8) |
      ((unsigned int)buffer[3]));
      // cast unsigned 32 bit integer to 32 bit float
      co2 = *(float*)&tempU32; // co2Concentration = 439.09f

      uint8_t hexVal[4];
      float data[3];

      for (int i = 0; i <= 3; i++)
      {
          hexVal[0]= i2c_rxBuffer[0+i*6];
          hexVal[1]= i2c_rxBuffer[1+i*6];
          hexVal[2]= i2c_rxBuffer[3+i*6];
          hexVal[3]= i2c_rxBuffer[4+i*6];

          data[i] = Hex2Float(hexVal, 4);
      }

      co2 = data[0];
      temp = data[1];
      hum = data[2];

      debug_txbuffer = cmd[0] << 8 | cmd[1];
      debug_rxbuffer = i2c_rxBuffer[0] << 8 | i2c_rxBuffer[1];

      } else {
        // Toggle LED0 when transmission passed
        GPIO_PinOutToggle(gpioPortD, 2);
        // Transmission complete
        i2c_startTx = false;
      }
    }
}
