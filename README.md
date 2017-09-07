# LIS3MDL
Arduino library to support the LIS3MDL high-performance 3D magnetometer

## API

This sensor uses I2C to communicate. It is then required to create a TwoWire interface before accessing to the sensors:  

    dev_i2c = new TwoWire(I2C2_SDA, I2C2_SCL);  
    dev_i2c->begin();  

An instance can be created and enbaled following the procedure below:  

    Magneto = new LIS3MDLSensor(dev_i2c);  
    Magneto->Enable();  

The access to the sensor values is done as explained below:  

  Read magnetometer.  

    Magneto->GetAxes(magnetometer);  

## Documentation

You can find the source files at  
https://github.com/stm32duino/LIS3MDL

The LIS3MDL datasheet is available at  
http://www.st.com/content/st_com/en/products/mems-and-sensors/e-compasses/lis3mdl.html
