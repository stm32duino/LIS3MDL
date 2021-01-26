/**
 ******************************************************************************
 * @file    LIS3MDLSensor.h
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Abstract Class of an LIS3MDL magnetometer sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LIS3MDLSensor_H__
#define __LIS3MDLSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"
#include "LIS3MDL_MAG_Driver.h"

/* Defines -------------------------------------------------------------------*/
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_4G   0.14  /**< Sensitivity value for 4 gauss full scale [LSB/gauss] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_8G   0.29  /**< Sensitivity value for 8 gauss full scale [LSB/gauss] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_12G  0.43  /**< Sensitivity value for 12 gauss full scale [LSB/gauss] */
#define LIS3MDL_MAG_SENSITIVITY_FOR_FS_16G  0.58  /**< Sensitivity value for 16 gauss full scale [LSB/gauss] */

/* Typedefs ------------------------------------------------------------------*/
typedef enum
{
  LIS3MDL_STATUS_OK = 0,
  LIS3MDL_STATUS_ERROR,
  LIS3MDL_STATUS_TIMEOUT,
  LIS3MDL_STATUS_NOT_IMPLEMENTED
} LIS3MDLStatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LSM303AGR Inertial Measurement Unit (IMU) 6 axes
 * sensor.
 */
class LIS3MDLSensor
{
  public:
    LIS3MDLSensor                       (TwoWire *i2c, uint8_t address=LIS3MDL_MAG_I2C_ADDRESS_HIGH);
    LIS3MDLSensor                       (SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    LIS3MDLStatusTypeDef begin          (void);
    LIS3MDLStatusTypeDef end            (void);
    LIS3MDLStatusTypeDef Enable         (void);
    LIS3MDLStatusTypeDef Disable        (void);
    LIS3MDLStatusTypeDef ReadID         (uint8_t *p_id);
    LIS3MDLStatusTypeDef GetAxes        (int32_t *pData);
    LIS3MDLStatusTypeDef GetSensitivity (float *pfData);
    LIS3MDLStatusTypeDef GetAxesRaw     (int16_t *pData);
	LIS3MDLStatusTypeDef GetODR         (float *odr);
	LIS3MDLStatusTypeDef SetODR         (float odr);
	LIS3MDLStatusTypeDef GetFS          (float *fullScale);
	LIS3MDLStatusTypeDef SetFS          (float fullScale);
	LIS3MDLStatusTypeDef ReadReg        (uint8_t reg, uint8_t *data);
	LIS3MDLStatusTypeDef WriteReg       (uint8_t reg, uint8_t data);
	
	/**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i=0; i<NumByteToRead; i++) {
          *(pBuffer+i) = dev_spi->transfer(0x00);
        }
         
        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }
		
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (byte) NumByteToRead);

        int i=0;
        while (dev_i2c->available())
        {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i=0; i<NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;                    
      }
  
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (int i = 0 ; i < NumByteToWrite ; i++)
          dev_i2c->write(pBuffer[i]);

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;
    
    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;

};

#ifdef __cplusplus
extern "C" {
#endif
uint8_t LIS3MDL_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t LIS3MDL_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
}
#endif

#endif