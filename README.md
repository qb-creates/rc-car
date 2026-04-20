# RC Car Project
This project is a custom-built remote-controlled (RC) car, designed from the ground up with custom electronics, PCB design, and embedded software for both the car and its remote. The project demonstrates:

<div align="center">
	<img src="images/completed-car-side-profile.JPEG" width="500" alt="Completed RC Car Side Profile">
</div>

## Table of Contents
1. [Software Used](#software)
2. [ Car Control Circuit](#carcontrolcuit)
    - [Steering Server Control](#steering)
    - [Drive Motor Control](#drivemotor)
    - [RF Receiver ](#rfreceiver)
    - [Parts List](#carpartslist) 
3. [Remote Control Circuit](#remotecontrolcircuit)
    - [Control Input Processing](#controlciruitprocessing)
        - [Throttle Input (Left Analog Stick)](#throttleprocessing)
        - [Steering Input (Right Analog Stick)](#steeringprocessing)
    - [RF Transmitter ](#rftransmitter)
    - [Parts List](#remotepartslist) 
4. [Hardware Design](#hardware-design)
5. [Electronics & PCB](#electronics--pcb)
6. [Software](#software)
7. [Gallery](#gallery)

## 1. Software Used<a name="software"></a>
- VSCode with PlatformIO extension: https://docs.platformio.org/en/latest/what-is-platformio.html
- AVRDUDE (Flash Uploader): https://github.com/avrdudes/avrdude
- KiCad (PCB Design): https://www.kicad.org
- AutoDesk Fusion 360 Personal (Case Design): https://www.autodesk.com/products/fusion-360/personal

## 3. Remote Control Circuit<a name="remotecontrolcircuit"></a>

### Control Input Processing<a name="controlciruitprocessing"></a>
#### Throttle Input (Left Analog Stick)<a name="throttleprocessing"></a>
#### Steering Input (Right Analog Stick)<a name="steeringprocessing"></a>

### RF Transmitter<a name="rftransmitter"></a>

### Parts List<a name="remotepartslist"></a>

## 2. Car Control Circuit<a name="carcontrolcuit"></a>

### Steering Servo Control<a name="steering"></a>

### Drive Motor Control<a name="drivemotor"></a>

### RF Receiver<a name="rfreceiver"></a>

### Parts List<a name="carpartslist"></a>
|_**Part Number**_|_**Quantity**_|
|:-----|:--------:|
|<a href="https://www.digikey.com/en/products/detail/microchip-technology/ATTINY861-20PU/1245922">ATtiny861-20PU</a>| x1 |
|<a href="https://www.digikey.com/en/products/filter/rectangular-connectors/headers-male-pins/314">ISP Header</a>| x1 |
|<a href="https://www.digikey.com/en/products/filter/ceramic-capacitors/60">22pF Capacitor</a>| x2 |
|<a href="https://www.digikey.com/en/products/filter/ceramic-capacitors/60">100nF Capacitor</a>| x1 |
|<a href="https://www.digikey.com/en/products/filter/oscillators/172">16Mhz Crystal</a>| x1 |
|<a href="https://www.digikey.com/en/products/filter/through-hole-resistors/53">10K resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/filter/through-hole-resistors/53">330 resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/filter/diodes/rectifiers/single-diodes/280">1n4148 Diode</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/e-switch/RP3502ABLK/280446?s=N4IgTCBcDaIKIHECMBOADGgtAOQCIgF0BfIA">Push Button</a>| x1 |
|<a href="https://www.digikey.com/en/products/filter/rectangular-connectors/headers-male-pins/314">Pcb Board 4cm x 6cm</a>| x1 |
|<a href="https://www.amazon.com/gp/product/B08CRTR7CZ/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1">Brushless Motor</a>| x1 |
|<a href="https://www.amazon.com/gp/product/B09H5L3KN5/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1">Barrel Jack</a>| x1 |
|<a href="https://www.sparkfun.com/products/18772">Infrared Emitter</a>| x1 ||
|<a href="https://www.amazon.com/gp/product/B071GRSFBD/ref=ppx_yo_dt_b_asin_title_o03_s00?ie=UTF8&psc=1">ESC 3A UBEC</a>| x1 |
|<a href="https://www.amazon.com/Adapter-Regulated-Switching-Interchangeable-Equipment/dp/B0BFPXZ7S1/ref=sr_1_3_sspa?crid=396GU23EXCZA8&keywords=9v+power+supply&qid=1697854150&sprefix=9v+power%2Caps%2C93&sr=8-3-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1">9V 1A Power Supply</a>| x1 |
|<a href="https://www.adafruit.com/product/1407">5V 200mA Inductive Charging Set </a>| x1 |


## Hardware Design
All hardware was designed in KiCad and fabricated for this project. The main board controls the car's motors and steering, while a separate remote board reads joystick inputs and transmits commands wirelessly.

- **Main Board:** Controls DC motor (drive) and servo (steering)
- **Remote Board:** Reads joystick positions and sends commands via RF
- **Microcontroller:** ATmega1284 for both car and remote
- **Wireless:** NRF24L01+ modules for robust 2.4GHz communication

### Schematics & PCB
Schematic and PCB files are available in the `eda/rc-car-kicad/` directory. Example files:
- `rc-car-kicad.kicad_sch` – Main schematic
- `rc-car-kicad.kicad_pcb` – Main PCB layout
- `rc-car-panel.kicad_pcb` – Panelized PCB for manufacturing

---

## Electronics & PCB
The electronics are split into two main systems:

### Car Controller
- Reads RF commands and controls the drive motor and steering servo
- PWM for motor speed and steering angle
- USART for debugging

### Remote Controller
- Reads analog joystick for speed and steering
- Packages data and transmits via NRF24L01

#### Example: Motor and Steering Control (Car)
```cpp
// src/main.cpp (car)
configureMotorPWM();
configureSteeringPWM();
while (true) {
	readAndPrintRFData(&payload);
	OCR1B = payload.ocrSteering;
	if (payload.ocrMotor >= -15 && payload.ocrMotor <= 15) {
		stopMotor();
	}
	// ...
}
```

#### Example: Joystick Reading & RF Transmission (Remote)
```cpp
// src/main.cpp (remote)
int16_t speedValue = readSpeedJoystick();
uint16_t steeringValue = readSteeringJoystick();
payload.ocrMotor = speedValue;
payload.ocrSteering = steeringValue;
rfTransmitData(payload);
```

---

## Software
The firmware is organized into two PlatformIO projects:

- `software/rc-car-atmega1284/` – Car controller code
- `software/rc-car-remote-atmega1284/` – Remote controller code

Each project uses modular libraries for:
- Motor and steering PWM control
- RF24 radio communication
- Joystick input (remote)
- USART serial output

See the `src/` and `lib/` folders in each project for implementation details.

---

## Gallery
<div align="center">
	<img src="images/car-components-side-profile.JPEG" width="350" alt="Car Components Side Profile">
	<img src="images/front-pcb.PNG" width="350" alt="Front PCB">
	<img src="images/full-board-pcb.PNG" width="350" alt="Full Board PCB">
	<img src="images/microcontroller-brain.JPEG" width="350" alt="Microcontroller Brain">
	<img src="images/power-delivery-board.JPEG" width="350" alt="Power Delivery Board">
	<img src="images/steering-components.JPEG" width="350" alt="Steering Components">
</div>

---

## License
See [LICENSE](LICENSE) for details.