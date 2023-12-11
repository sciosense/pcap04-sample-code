# Libraries and Routines for SPI, I2C and UART
Source Libraries for different devices, using SPI, I2C or UART communication interface.
* Flow Sensing, datahseets are [here](https://www.sciosense.com/products/flow-sensing/): <br>UFC - GP21, GP22, GP30, AS6031, AS6040, ...
* Precision Time Measurement, datahseets are [here](https://www.sciosense.com/products/precision-time-measurement/): <br>TDC - GP21, GPX2, AS6500, AS6501, ...
* Sensor Interfaces, datahseets are [here](https://www.sciosense.com/products/sensor-interfaces/): <br>PICOSTRAIN - PS081, PS09 <br>PICOCAP - PCap01, PCap02, PCap04


### How to include these libraries in your project
For example using STM32CubeIDE.
- Open 'Properties' for selected 'Project'
- Select 'C/C++ General', 'Code Analysis', 'Path and Symbols'
- Select tab, called 'Includes', 'Languages' = 'GNU C'
- Include directory, where the source libraries are located, e.g. <../../_Src_Lib>
