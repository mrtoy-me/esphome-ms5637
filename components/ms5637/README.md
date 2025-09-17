
The MS5637 Arduino library originally written by TEConnectivity with MIT license.<BR>
This library was updated by Nathan Seidle @ SparkFun Electronics, 2018.<BR>

The MS5637 Sparkfun Arduino library has been adapted and modified<BR>
so it functions under the ESPHome component framework by @mrtoy-me, 2023<BR>
  
MIT License<BR>
Copyright (c) 2016 TE Connectivity<BR>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Usage: MS5637 component
Copy component's files to a ***components*** directory under your homeassistant's esphome directory.<BR>
The following yaml can then be used so ESPHome accesses the component files:
```
external_components:
  - source: components
```
The component uses the sensor's default i2c address of 0x76.<BR>

## Example Basic Configuartion YAML
```
sensor:
  - platform: ms5637
    pressure:
      name: MS5637 Pressure
```
## Configuration variables:

- **pressure** (*Required*): The information to setup the pressure sensor.

  - **name** (**Required**): The name for the pressure sensor.
  - **id** (*Optional*): Set the ID of this sensor for use in lambdas.
  - All other options from Esphome Sensor configuration.

- **temperature** (*Optional*): The information to setup the temperature sensor.

  - **name** (**Required**, string): The name for the temperature sensor.
  - **id** (*Optional*): Set the ID of this sensor for use in lambdas.
  - All other options from Esphome Sensor configuration.

- **resolution** (*Optional*): Set sensor sampling resolution. In summary, resolution impacts 
  the adc conversion time (1-17ms), sensor current draw (0.6-20uA) and reading resolution (0.1-0.02mbar).
  Given the sensor error band is typically +/- 2mbar, the component's default resolution should meet
  the requirements of most applications. For more details on resolution, refer to the datasheet.<BR>
  ``OSR_256``, ``OSR_512``, ``OSR_1024``, ``OSR_2048``, ``OSR_4096``, ``OSR_8192``. Defaults to ``OSR_2048``.

- **address** (*Optional*): Manually specify the I²C address of
  the sensor. Defaults to ``0x76``.

- **update_interval** (*Optional*): The interval to check the
  sensor. Defaults to ``60s``.


## Example Advanced Configuration YAML
```
sensor:
  - platform: ms5637
    resolution: OSR_2048
    pressure:
      name: MS5637 Pressure
    temperature:
      name: MS5637 Temperature
    address: 0x76
    update_interval: 60s
```
      