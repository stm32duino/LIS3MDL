# LIS3MDL
Arduino library to support the LIS3MDL high-performance 3D magnetometer

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LIS3MDLSensor Magneto(&dev_i2c);
    Magneto.begin();
    Magneto.Enable();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LIS3MDLSensor Magneto(&dev_spi, CS_PIN);
    Magneto.begin();
    Magneto.Enable();

The access to the sensor values is done as explained below:  

  Read magnetometer.  

    int32_t magnetometer[3];
    Magneto.GetAxes(magnetometer);  

## Documentation

You can find the source files at  
https://github.com/stm32duino/LIS3MDL

The LIS3MDL datasheet is available at  
http://www.st.com/content/st_com/en/products/mems-and-sensors/e-compasses/lis3mdl.html
