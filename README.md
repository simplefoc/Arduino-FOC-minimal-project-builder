# Arduino *SimpleFOClibrary* v2.2 - minimal project builder 

> This repo is still in its early stage, it will be made up to date soon 😄

![MinimalBuild](https://github.com/askuric/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg?)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)

This is the branch of the [*SimpleFOClibrary*](https://github.com/askuric/Arduino-FOC) repository intended to be used to simplify the creation of the projects with minimal code possible which is specific for certain **motor+sensor+driver** combination. 

### Repository structure
Library source code structure
```shell
├─── Arduino-FOC/src
| |
| ├─ BLDCMotor.cpp/h           # BLDC motor handling class  
| ├─ StepperMotor.cpp/h        # Stepper motor handling class 
| |
│ ├─── common                  # Contains all the common utility classes and functions
| | |
| | ├─ defaults.h              # default motion control parameters
| | ├─ foc_utils.cpp./h        # utility functions of the FOC algorithm
| | ├─ time_utils.cpp/h        # utility functions for dealing with time measurements and delays
| | ├─ pid.cpp./h              # class implementing PID controller
| | ├─ lowpass_filter.cpp./h   # class implementing Low pass filter
| | |
| | ├─── base_classes
| | | ├─ FOCMotor.cpp./h        # common class for all implemented motors  
| | | ├─ BLDCDriver.h           # common class for all BLDC drivers  
| | | ├─ StepperDriver.h        # common class for all Stepper drivers
| | | └─ Sensor./h              # common class for all implemented sensors
| |
| ├─── drivers  
| | ├─ BLDCDriver3PWM.cpp/h         # Implementation of generic 3PWM bldc driver
| | ├─ BLDCDriver6PWM.cpp/h         # Implementation of generic 6PWM bldc driver
| | ├─ StepperDriver2PWM.cpp/h      # Implementation of generic 2PWM stepper driver
| | ├─ StepperDriver4PWM.cpp/h      # Implementation of generic 4PWM stepper driver
| | |      
| | ├─ hardware_api.h               # common mcu specific api handling pwm setting and configuration
| | |
| | ├─── hardware_specific          # mcu specific hadrware_api.h implementations
| | | ├─ atmega2560_mcu.cpp         # ATMega 2560 implementation
| | | ├─ atmega328_mcu.cpp          # ATMega 328 (Arduino UNO) implementation
| | | ├─ esp32_mcu.cpp              # esp32 implementation
| | | ├─ stm32_mcu.cpp              # stm32 implementation
| | | ├─ teensy_mcu.cpp             # teensy implementation
| | | └─ generic_mcu./h             # generic implementation - if not nay of above (not complete)       
| |
| ├─── sensors 
| │ ├─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
| │ ├─ MagneticSensorSPI.cpp/h      # class implementing SPI communication for Magnetic sensors
| │ ├─ MagneticSensorI2C.cpp/h      # class implementing I2C communication for Magnetic sensors
| │ ├─ MagneticSensorAnalog.cpp/h   # class implementing Analog output for Magnetic sensors
    └─ HallSensor.cpp/h             # class implementing Hall sensor
```

Minimal project examples provided for quick start:
```shell
├─── minimal_project_examples       # Project examples
│ ├─ atmega2560_stepper_encoder     # ATMega2560 + BLDC motor + 3PWM driver + encoder
| |
│ ├─ atmega328_bldc_encoder         # ATMega328 + BLDC motor + 3PWM driver + Encoder
│ ├─ atmega328_bldc_magnetic_i2c    # ATMega328 + BLDC motor + 3PWM driver + I2C magnetic sensor
│ ├─ atmega328_bldc_openloop        # ATMega328 + BLDC motor + 3PWM driver
│ ├─ atmega328_driver_standalone    # ATMega328 + 3PWM driver
│ |
│ ├─ esp32_bldc_magnetic_spi        # ESP32 + BLDC motor  + 3PWM driver + SPI magnetic sensor
│ ├─ esp32_stepper_openloop         # ESP32 + Stepper motor + 4PWM driver
| |
│ ├─ stm32_bldc_encoder             # stm32 + BLDC motor + 6PWM driver + encoder
  └─ stm32_bldc_hall                # stm32 + BLDC motor + 3PWM driver + hall sensors
```



# Creating your own minimal project

Creating your own minimal project version is very simple and is done in four steps:
- Step 0: Download minimal branch contents to your PC
- Step 1: Create your the arduino project
- Step 2: Add **driver** specific code
- Step 3: Add **motor** specific code
- Step 4: Add **sensor** specific code

## Step 0. Download the code
#### Github website download
- Make sure you are in [minimal branch](https://github.com/askuric/Arduino-FOC/tree/minimal) 
- Download the code by clicking on the `Clone or Download > Download ZIP`.
- Unzip it 

#### Using terminal
- Open the terminal:
  ```sh
  cd *to you desired directory*
  git clone -b minimal https://github.com/askuric/Arduino-FOC.git
  ```

## Step 1. Creating the Arduino project

Open a directory you want to use as your arduino project directory `my_arduino_project` and create `my_arduino_project.ino` file. After this you create `src` folder in this directory and copy the folder named `common` from the `library_source` folder.   Your project directory should now have structure:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
│ | ├─── common 
| | | ├─ defaults.h              # default motion control parameters
| | | ├─ foc_utils.cpp./h        # utility functions of the FOC algorithm
| | | ├─ time_utils.cpp/h        # utility functions for dealing with time measurements and delays
| | | ├─ pid.cpp./h              # class implementing PID controller
| | | ├─ lowpass_filter.cpp./h   # class implementing Low pass filter
| | | └─── base_classes             # common class for all implemented sensors  
```
## Step 2. Add driver specific code
First create a `drivers` folder in `src` folder. If you wish to use the 3PWM or 6PWM BLDC driver in your project with your setup you will have to copy the `BLDCDriver3PWM.cpp/h` files or `BLDCDriver3PWM.cpp/h` files from the `library_source/drivers` folder in your drivers folder. If you wish to use the 4PWM or 2PWM stepper motor make sure to copy the `StepperDriver4PWM.cpp/h` or `StepperDriver2PWM.cpp/h` files and place them to the `src/drivers` folder.
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | |
│   └───  drivers      
|     └─ BLDCDriver3PWM.cpp/h # BLDC motor handling class  
```
Next from the `library_source/drivers` directory copy the `hardware_api.h` file to the `src/drivers` folder as well as the `hardware_specific` folder. Finally in the `hardware_specific` folder  leave only the `x_mcu.cpp` file which corresponds to your mcu architecture. For example, for esp32 boards
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | |
│   └───  drivers      
|     ├─ BLDCDriver3PWM.cpp/h         # BLDC driver handling class  
|     ├─ hardware_api.h               # common mcu specific api handling pwm setting and configuration
|     └─── hardware_specific          # mcu specific hadrware_api.h implementations
|       └─ esp32_mcu.cpp              # esp32 implementation
```

And in your Arduino code in the `my_arduino_project.ino` file make sure to add the the include:
```cpp
#include "src/drivers/BLDCDriver3PWM.h"
```
For the combination of stepper driver 4pwm and stm32 board the structure will be:
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | |
│   └───  drivers      
|     ├─ StepperDriver4PWM.cpp/h      # Stepper driver handling class  
|     ├─ hardware_api.h               # common mcu specific api handling pwm setting and configuration
|     └─── hardware_specific          # mcu specific hadrware_api.h implementations
|       └─ stm32_mcu.cpp              # stm32 implementation
```
And the include:
```cpp
#include "src/drivers/StepperDriver4PWM.h"
```
If you wish to run your drivers in the standalone mode these are all the files that you will need. See the `atmega328_driver_standalone` project example.

## Step 3. Add motor specific code
If you wish to use the BLDC motor with your setup you will have to copy the `BLDCMotor.cpp/h` from the `library_source` folder, and if you wish to use the stepper motor make sure to copy the `StepperMotor.cpp/h` files and place them to the `src` folder
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | ├─── drivers            # Driver handling software
| | |
|   └─ BLDCMotor.cpp/h      # BLDC motor handling class  
```
And in your Arduino code in the `my_arduino_project.ino` file make sure to add the the include:
```cpp
#include "src/BLDCMotor.h"
```
For stepper motors the procedure is equivalent:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | ├─── drivers            # Driver handling software
| | |
|   └─ StepperMotor.cpp/h   # Stepper motor handling class 
```
And the include:
```cpp
#include "src/StepperMotor.h"
```
If you wish to run your motor in the open loop mode these are all the files that you will need. See the `esp32_stepper_openloop` and  `atmega328_bldc_openloop` project examples.

## Step 4. Add sensor specific code
In order to support the different position sensors you will first have to create the `sensors` folder in your `src` folder. And then copy their `*.cpp` and `*.h` files which correspond to the sensor into your `src/sensors` directory. You can find the sensor implementations in the `library_source/sensors` directory.


### Example: Encoder sensor 
For example if you wish to use BLDC motor and encoder as a sensor, your arduino project will have structure:
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common             # Common utility classes and functions
| | ├─── drivers            # Driver handling software
│   ├─── sensors      
|   | └─ Encoder.cpp/h      # Encoder class implementing the Quadrature encoder operations
|   |
|   └─ BLDCMotor.cpp/h      # BLDC motor handling class 
```
And in your in your arduino project  `my_arduino_project.ino` add the line:
```cpp
#include "src/sensors/Encoder.h"
```
See `atmega328_bldc_encoder` and `stm32_bldc_encoder` project example for BLDC motors or `atmega2560_stepper_encoder` for stepper equivalent. 

### Example: SPI Magnetic sensor 
If you wish to use Stepper motor and SPI magnetic sensor in your project, your folder structure will be:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common                   # Common utility classes and functions
| | ├─── drivers                  # Driver handling software
│   ├─── sensors      
|   | └─ MagneticSensorSPI.cpp/h  # class implementing SPI communication for Magnetic sensors
|   |
|   └─ StepperMotor.cpp/h         # Stepper motor handling class  
```
And in your in your arduino project  `my_arduino_project.ino` add the line:
```cpp
#include "src/sensors/MagneticSensorSPI.h"
```
See `esp32_bldc_magnetic_spi` project example or `atmega328_bldc_magnetic_i2c` for I2C magnetic sensors equivalent.


### Example: Multiple sensors: analog magnetic sensor and encoder
For example if you wish to use magnetic sensor with SPI communication, your arduino project will have structure:

```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common                       # Common utility classes and functions
| | ├─── drivers                      # Driver handling software
│   ├─── sensors      
|   | ├─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
|   | └─ MagneticSensorAnalog.cpp/h   # class implementing Analog output for Magnetic sensors
|   |
|   └─ StepperMotor.cpp/h             # Stepper motor handling class 
```
And added includes should be:
```cpp
#include "src/sensors/MagneticSensorAnalog.h"
#include "src/sensors/Encoder.h"
```
### Example: Sensors standalone - *without motor/driver*
It is possible to use the sensors developed in this library as standalone sensors. For that you can need to do steps 0. and 1. and then just add the sensor specific code. This is one possible project structure if you wish to use an encoder as a standalone sensor:  
```shell
├─── my_arduino_project
| ├─ my_arduino_project.ino
| └─── src
| | ├─── common                       # Common utility classes and functions
│   └─── sensors      
|     └─ Encoder.cpp/h                # Encoder class implementing the Quadrature encoder operations
```

And you can include it directly to the arduino project:
```cpp
#include "src/sensors/Encoder.h"
```

## Documentation
Find out more information about the Arduino *Simple**FOC**library* and *Simple**FOC**project* in [docs website](https://docs.simplefoc.com/) 


## Arduino FOC repo structure
Branch  | Description | Status
------------ | ------------- | ------------ 
[master](https://github.com/simplefoc/Arduino-FOC) | Stable and tested library version | ![Library Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Compile/badge.svg)
[dev](https://github.com/simplefoc/Arduino-FOC/tree/dev) | Development library version | ![Library Dev Compile](https://github.com/simplefoc/Arduino-FOC/workflows/Library%20Dev%20Compile/badge.svg?branch=dev)
[minimal](https://github.com/simplefoc/Arduino-FOC/tree/minimal) | Minimal Arduino example with integrated library | ![MinimalBuild](https://github.com/simplefoc/Arduino-FOC/workflows/MinimalBuild/badge.svg?branch=minimal)
