# RC Car Project
This project is a custom-built remote-controlled (RC) car, designed from the ground up with custom electronics, PCB design, and embedded software for both the car and its remote.

- Bidirectional Motor Driver (H-Bridge): https://en.wikipedia.org/wiki/H-bridge
- Steering Servo PWM Control: https://en.wikipedia.org/wiki/Servo_control
- nRF24L01 Wireless Transceiver Module: https://www.instructables.com/NRF24L01-Tutorial-Arduino-Wireless-Communication/

<div align="center">
	<img src="images/completed-car.JPEG" width="500" alt="Completed RC Car Side Profile">
</div>

## Table of Contents
1. [Software Used](#software)
2. [Drive and Steering Control Circuit](#driveandsteeringcircuit)
    - [RF Receiver](#rfreceiver)
    - [Steering Servo Control](#steering)
    - [Drive Motor Control](#drivemotor)
    - [Power Consumption](#powerconsumption)
    - [Charging Notes](#carcharging)
    - [Parts List](#carpartslist)
3. [Remote Control Circuit](#remotecontrolcircuit)
    - [RF Transmitter](#rftransmitter)
    - [Control Input Processing](#controlinputprocessing)
        - [Throttle Input (Left Analog Stick)](#throttleprocessing)
        - [Steering Input (Right Analog Stick)](#steeringprocessing)
    - [Charging Notes](#remotecharging)
    - [Parts List](#remotepartslist)
4. [Schematics](#schematics)
    - [Drive and Steering System Schematic](#driveschematic)
    - [Controller Schematic](#controllerschematic)
5. [STL Viewer](#stlviewer)
    - [Case Base](#casebase)
    - [Case Top](#casetop)

## 1. Software Used<a name="software"></a>
- VSCode with PlatformIO extension: https://docs.platformio.org/en/latest/what-is-platformio.html
- AVRDUDE (Flash Uploader): https://github.com/avrdudes/avrdude
- KiCad (PCB Design): https://www.kicad.org
- Tinkercad (Controller and parts Design): https://www.tinkercad.com/

## 2. Drive and Steering Control Circuit<a name="driveandsteeringcircuit"></a>
This board is the car's real-time control core. It receives wireless throttle/steering commands from the handheld transmitter, converts steering targets into stable hardware PWM for the MG90S servo, and drives the rear motor through a protected H-bridge path with direction dead-time and command deadbands for smoother behavior. The result is a compact control system designed for responsive handling, predictable failsafe behavior, and reliable power delivery under changing load.

<div align="center">
 <table>
     <tr>
         <td><img src="images/car-side-profile.JPEG" width="360" alt="Car Side Profile"></td>
         <td><img src="images/car-top-profile.JPEG" width="360" alt="Car Top Profile"></td>
     </tr>
     <tr>
         <td><img src="images/car-angled-profile.JPEG" width="360" alt="Car Angled Profile"></td>
         <td><img src="images/car-side-profile-complete.JPEG" width="360" alt="Car Side Profile Complete"></td>
     </tr>
 </table>
</div>

### RF Receiver<a name="rfreceiver"></a>
The car uses an nRF24L01+ module with the RF24 library over SPI to receive drive and steering commands from the handheld controller. In firmware, the radio object is created as `RF24 radio(CE_PIN, CSN_PIN)` with CE on PB0 and CSN on PB1.

The RF stack is configured in `configureRFRadio()` with:

- `radio.begin()` hardware check (halts if radio is not detected)
- `radio.setPALevel(RF24_PA_LOW)` to reduce TX power/current
- `radio.enableDynamicPayloads()`
- `radio.setAutoAck(false)`
- Pipe/address setup using `"jag-1"` and `"jag-2"`
- `radio.startListening()` so this node stays in RX mode

An important part of the RF path is Timer0 timing configuration. Because this firmware includes `#include <RF24.h>`, the RF24 stack depends on `millis()`/`micros()` timing for operations such as CE pulse timing, transmit timing windows, retry timing, and transaction spacing.

The RF24 library itself does not directly configure Timer0. Instead, those timing primitives are provided by the Arduino core (`millis()`/`micros()`), and the project sets Timer0 for a 50 us timing base:

```cpp
TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
TCCR0B = (1 << WGM02) | (1 << CS01); // prescaler = 8
OCR0A = 49;
```

At 8 MHz CPU clock with prescaler 8:

- Timer clock = 8 MHz / 8 = 1 MHz
- Timer tick = 1 us
- Period = (49 + 1) x 1 us = 50 us

So the Timer0 event period is 50 us (20 kHz). Timer0 is intentionally kept running at this rate so the `millis()`/`micros()` time base used by RF24 remains available and stable.

The receiver expects a `MotorControlPayload` packet with two fields:

- `int16_t ocrMotor` for signed throttle/drive command
- `uint16_t ocrSteering` for servo pulse width command in microseconds

This payload is 4 bytes total (`2 + 2`), and after reception the car firmware applies it directly:

- `OCR1B = payload.ocrSteering` for steering position
- `payload.ocrMotor` drives H-bridge direction/speed logic with a deadband of -15 to +15

Reliability/safety behavior tied to RF reception:

- On first valid packet, watchdog is enabled (`WDTO_2S`)
- Each additional valid packet resets the watchdog
- If RF packets stop arriving, watchdog timeout forces a reset/failsafe

### Steering Servo Control<a name="steering"></a>
Steering is driven by an MG90S servo using Timer1 hardware PWM on OC1B (PD4). The car firmware configures Timer1 in Fast PWM mode 14 with ICR1 as TOP, prescaler = 8, and ICR1 = 20000. With an 8 MHz clock, that gives a 1 us timer tick and about a 50 Hz control period (20 ms), which matches standard hobby servo timing.

The commanded steering pulse is written directly to OCR1B. In this project, steering values are transmitted as absolute pulse widths in microseconds, centered at 1900 us. On the controller side, joystick input is mapped and clamped to approximately 1520 us to 2280 us, then sent over RF. On the car side, OCR1B is updated from the payload each loop:

- Neutral: ~1900 us
- Left/Right command range: ~1520 us to ~2280 us
- PWM period: ~20 ms (50 Hz)
- Steering deadzone: +/-90 us around center snaps to 1900 us to reduce twitch

This keeps steering response deterministic because the servo waveform is generated in hardware (Timer1), not by software toggling.

### Drive Motor Control<a name="drivemotor"></a>
The drive motor uses a discrete H-bridge driven by a hybrid approach: latch lines select direction, while PWM is generated in software via Timer2 interrupts.

H-bridge signal assignment:

- PC0: Forward latch
- PC1: Forward PWM
- PC2: Reverse latch
- PC3: Reverse PWM

Timer2 is configured for Fast PWM with OCR2A as TOP. With OCR2A = 199 and prescaler = 8 at 8 MHz, PWM frequency is about 5 kHz. Instead of using OC2A/OC2B output pins directly, compare interrupts are used:

- TIMER2_COMPA_vect sets the active direction PWM pin high (PC1 for forward or PC3 for reverse)
- TIMER2_COMPB_vect clears that same active pin low

This effectively creates direction-dependent PWM on one of two output pins while the opposite side remains off.

Direction changes are intentionally staged to protect the bridge and drivetrain:

1. Disable Timer2 PWM (stop switching)
2. Force all H-bridge outputs low
3. Wait 150 ms dead-time
4. Assert new direction latch (PC0 forward or PC2 reverse)
5. Re-enable Timer2 PWM

That delay prevents hard instant reversals and helps ensure MOSFETs on the same side of the H-bridge are not on at the same time during direction transitions, reducing shoot-through/braking stress.

Additional control logic from the car firmware:

- Speed deadband: commands between -15 and +15 stop the motor to avoid joystick/noise creep
- Steering deadzone: commands within +/-90 us of center are snapped to neutral (1900 us)
- Direction is only changed when needed (or from stopped state)
- A watchdog is enabled after first RF packet and reset on each valid packet; if RF traffic is lost, the MCU resets after the watchdog timeout (2 s)

### Why RF reception is harder with this PWM approach
Using Timer2 compare interrupts for every PWM cycle means the MCU is servicing frequent interrupts continuously while also polling and reading nRF24 payloads over SPI. That interrupt load can steal time from RF handling and make robust payload reception harder than a pure hardware-PWM motor path.

In practice, the current tuning (about 5 kHz PWM, deadband around zero, staged direction changes) was a workable sweet spot between motor smoothness and reliable radio updates. A strictly hardware PWM implementation for both drive channels would reduce ISR overhead and leave more CPU time for RF processing.

### Power Consumption<a name="powerconsumption"></a>
This power system was sized around worst-case drive events (motor stall plus active steering), not just average cruising current.

Estimated load envelope:

- 9V drive motor stall current at 7.2V: ~600 mA
- MG90S steering servo working current: ~600-700 mA
- Control circuitry (MCU, RF, logic, support electronics): ~20 mA

Worst-case instantaneous estimate:

- Total peak demand: $0.60 + 0.70 + 0.02 \approx 1.32\,A$

Battery capability:

- 2S 200 mAh pack, 20C rating
- Maximum continuous supply: $0.2\,Ah \times 20 = 4\,A$

Power-stage device headroom:

- IRLML6244 continuous drain current: 6.3 A (comfortably above motor stall current)
- IRLML2246 continuous source current: -1.3 A (above the ~600 mA motor stall requirement in this bridge path)

Conclusion: the battery and MOSFET current limits provide practical headroom over the expected peak load, so the system can supply drive, steering, and control electronics simultaneously under normal and transient operation.

### Charging Notes<a name="carcharging"></a>
For safe charging practice on the car side, do not charge the car while it is powered on. Turn the car off first, then connect the charger.

---
### Parts List<a name="carpartslist"></a>
|_**Part Number**_|_**Quantity**_|
|:-----|:--------:|
|<a href="https://www.digikey.com/en/products/detail/rubycon/25YXG220MEFC6-3X11/3134189">220uF Capacitor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/taiyo-yuden/TMK107B7105KA-T/2714162">1uF Capacitor</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/samsung-electro-mechanics/CL21A106KAYNNNG/3894413">10uF Capacitor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/murata-electronics/GRM188R61E225KA12D/4905349">2.2uF Capacitor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/kemet/C0603C104K3RACTU/416044">.1uF Capacitor</a>| x4 |
|<a href="https://www.digikey.com/en/products/detail/venkel/CTL1206FRD1T/13245061">LED</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/diodes-incorporated/1N5819HW-7-F/814970">1N5819HW-7-F Diode</a>| x4 |
|<a href="https://www.amazon.com/2-54mm-Breakaway-Female-Connector-Arduino/dp/B01MQ48T2V/ref=sr_1_2_sspa?crid=37UI1XQQ1YOGK&dib=eyJ2IjoiMSJ9.dQRG7A390Cr7G0eECwl3ySotcaXfI28I1uUKHcmiPHf-hvTrpk4-mLxeOfBP_o2wgeNP09ah4l4Z_nh4K_ky6uhpN37tS8i2I8bk7hElYNXx-FAMr_ipA9P94e9JhgRABouD4lHJnLV6bc2VHVILnmQpPnT6LynWmghNohKPKs16E5RMkmBnBZLmoQyECbt8tk0IWHyfDAvT8Id0p6m7HikZdiL1Mb9oZ9zR8lDkjQg.dCZRPJ_88iquw6HqnK28jkzvtgYozecW_lBzuIykXWo&dib_tag=se&keywords=header%2Bpins&qid=1778354973&sprefix=header%2Bpins%2Caps%2C146&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1">Header Pins</a>| x17 |
|<a href="https://www.digikey.com/en/products/detail/onsemi/MMBT3904LT1G/919601">MMBT3904</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/umw/UMWIRLML2246TR/24889419">IRLML2246 / UMWIRLML2246TR</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/infineon-technologies/IRLML6244TRPBF/2393871">IRLML6244</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/stackpole-electronics-inc/RMCF0603FT470K/1761140">470k ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/yageo/RC0603JR-07510KL/726802">500k ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/koa-speer-electronics-inc/RK73B1JTTD154J/9844780">150k ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/yageo/RC0603JR-071K5L/726689">1.5k ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/stackpole-electronics-inc/RMCF0603JT10K0/1758104">10k ohm Resistor</a>| x5 |
|<a href="https://www.digikey.com/en/products/detail/yageo/RC0603JR-071K3L/726686">1.3k ohm Resistor</a>| x4 |
|<a href="https://www.digikey.com/en/products/detail/stackpole-electronics-inc/RMCF0603FT10R0/1761152">10 ohm Resistor</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/omron-electronics-inc-emc-div/A6S-1104-PH/3102847">SW_SPST</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/texas-instruments/TPS3840DL35DBVR/15857118">TPS3840</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/microchip-technology/MIC3975-5-0YMM-TR/1029778">MIC3975-5.0YMM Voltage Regulator</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/microchip-technology/MIC5225-3-3YM5-TR/1815447">MIC5225-3.3YM5 Voltage Regulator</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA164A-AU/2271202">ATmega164A-A</a>| x1 |
|<a href="https://www.amazon.com/dp/B0BWJ4RKGV?ref=ppx_yo2ov_dt_b_fed_asin_title">MG90S servo motor</a>| x1 |
|<a href="https://www.amazon.com/uxcell-Micro-11500-12000RPM-Remote-Control/dp/B07M99JK6Y/ref=sr_1_6_pp?crid=15ZR5I0IA9JJI&dib=eyJ2IjoiMSJ9.0dTLxilAb6xq-6TUjWUHu4LHY67tPVw3OW_p63iFn0nxHTF8tJOEU3O_Cb8oOwE6WxJZW0BjBXIf0G8GfpGx34Xib0qnggH2QTWBRLxwTrt1dVHrFBr4m3qlymM1dp_HGhddBT5o2u24L-f4An4mHsWxuHhcwAWkuWetTyXovK-j6vw5tc8bw8An9Jwehol9gVoWqmTKBzJJhRpDE5yd1FT2j8fYInDLiEd9pn63in9N35NYOYHjUvUYXkuJJkr2DH7aythkyAL53UUCrgou9nhxGCkFBAQQLPBe2CI7QtI.hByQYkM03NWvQAiFiiVJo0fbgudInFzwV0ohKh8QIlI&dib_tag=se&keywords=9v+motor&qid=1778356523&sprefix=9v+motor%2Caps%2C163&sr=8-6">9V DC Motor</a>| x1 |
|<a href="https://www.amazon.com/KiNSMART-Jaguar-Project-Metal-Model/dp/B0GXDP3B83/ref=sr_1_2?crid=HY25IPNV0BHN&dib=eyJ2IjoiMSJ9.kNHmPeKKto7l4mN5kOvqBU7R0Xlsnn2LHCeKZya09DTJalbcDjd4JoTcYEG4oBi7Ljllb7X6_TGZSaIGPy1Xm1sebzbI1yp2mHZeyxYMDKpjf4Jqk94GJpw4gnU1N_5tkqw1yrXIqMnX5ZyADOEFBC7UcSktNJJQAYE5tSmkBo9atjGda3yxCk-vgOEnLoZFFx2acgDjpZFOOCdXU92NyMzapczCn7FhwygMaJ_Cei5oG747OONrkMHwEvzIolQ_2xz7kVeILcwJkr2IywmRw9VX98jR5jMxBVIL_H83mhw.naunVJhW78f1ztCTSTYnX7RhDgAuPRFgKA2PJmHRRKo&dib_tag=se&keywords=kinsmart%2B1%3A38&qid=1779063547&sprefix=kinsmart%2B1%2B38%2Caps%2C216&sr=8-2&th=1">Kinsmart Jaguar 1:38</a>| x1 |
|<a href="https://www.amazon.com/HiLetgo-NRF24L01-Wireless-Transceiver-Module/dp/B00LX47OCY/ref=sr_1_1_sspa?crid=1USO9UOFDEZL5&dib=eyJ2IjoiMSJ9.HpGu4TebgrLEY6IjfmGnCKONGE1zifAy342llWfR4vcsJ4_OTj71wcfjuLFi42g9LnfOsZybnBvz4HCtPFZh7IoO0VCtoV4SHTwJkmzj3SyTmBWTWfYEwK0bZ-6KAhnJqpXuroU3ExNMIQ_0sb6zAw01BAymwhYK7jVncUvl8YxZV7HAVItE-ISceLL5caDSPRu-nl4dzw8eF-t1VvKSdHE_Pz68YolGVn5D4_TDIAE.6OIyGVxp3k88Grsb6QKkfbgzYiEAVdqjkrvRE4yyM6M&dib_tag=se&keywords=nrf24l01&qid=1778356552&sprefix=nrf%2Caps%2C232&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1">NRF24L01+ Module</a>| x1 |
|<a href="https://www.amazon.com/dp/B07MW2L96L?ref=ppx_yo2ov_dt_b_fed_asin_title">7.4V battery</a>| x1 |
|<a href="https://www.amazon.com/200PCS-Module-Plastic-Single-Spindle/dp/B0DKHG96PL/ref=sr_1_6?crid=W0RZLTTF36ZV&dib=eyJ2IjoiMSJ9.rd3Q4coQN-xK_iVME0RsSWQTjAsKta_V4h4vm12weDUJC74fnjatzucJO07yHtkLeIzfxeMC2dxmPiht58xboKdyTAKydnddtAAmDD_aARQfSeJf72BiNVGOLvJEh9z4o6wPQQIsW7IFXJZAsVFzRvb3aBc2lONKzi84gw8UxRRc2rQ9rWUQvz_yelBZoDNSvcFoSqvusfdJFjuKQ47WFcnAPOwQnc2LdEzRRlFGYalz5QPsVB-hWhAqh67OA6riZCc13JSazX8bvwV50yC13BdBVPq8QEGZLQo3CGB_GXM.rLefd6TJ3JQPyoYEO4myNas1mQIUsuXzyBHIw13Vpyw&dib_tag=se&keywords=dc%2Bmotor%2Bgears&qid=1778804461&sprefix=dc%2Bmotor%2Bgear%2Caps%2C142&sr=8-6&th=1">Gears </a>| x1 |
|<a href="https://www.amazon.com/ruthex-Threaded-Insert-pieces-ultrasound/dp/B088QJG676/ref=sr_1_3?crid=327WC4A3U357H&dib=eyJ2IjoiMSJ9.v_Af1Sv2DMXyynp1rO3uN2cjI_LkHpW2Xd3NuL9RUn1yz2Ym6BTHo47Bnvh-mF3rih3MIxPZyDCFRwN3f8bgTgfOqS-2FD8o-WWmIpuq7XJIrvCSKP6iwhtXO1FB6g2J8vFhgMjuaMVbjKPUpn3U8iuk4FPTnzt9IQcKtjUuxKiJM3PQJf3MPDf15V62fIp3_oibwlJtUKJ7oyWbckn9BPgwhhw0z9gTb7ylLCQjULA.KUBGIt58ag9IJXW2aWAnvHSlQ59y6f_kavaLpQD422E&dib_tag=se&keywords=inserts%2Bm2&qid=1778806075&sprefix=inserts%2Bm%2Caps%2C197&sr=8-3&th=1">M2 Inserts</a>| x3 |
|<a href="https://www.amazon.com/Phillips-Countersunk-Electronic-Accessories-Samsung/dp/B07HC3LQYS/ref=sr_1_8?crid=2ISFEC45EBS5Q&dib=eyJ2IjoiMSJ9.sToeJ_cHiwrPYQ_C9rq2gwq_BqFxCk_dAqNz8qbKlTKQla66SuHvAVoMEMQE3FrKbT_cXuKk3EDQL7eTiH6WYWt4xdVsIXdoV99uXBCs7qfK_HdB1wotMUmIz4MZM-fYqvCkvNTV6tpBtMOlWfMOOT3xG69H9dmbPd9TCbIKidT_fAgNMZEY2BR0qPmHW3JfhP0KSYKgUQ9dyiHrulr41WxGtIHFVkrdDqeaHCOoUuo.r_V_du0eyowAqg1oQ__QllHDwAeUxV1av7Z4HFjVBKw&dib_tag=se&keywords=m2%2Bscrews&qid=1778806119&sprefix=m2%2Bscrew%2Caps%2C149&sr=8-8&th=1">M2 screws</a>| x3 |

## 3. Remote Control Circuit<a name="remotecontrolcircuit"></a>
The handheld controller samples two joystick axes, converts them into throttle and steering commands, and transmits those values over 2.4 GHz to the car at a steady update rate. The design goal is low-latency control with stable center behavior so the car tracks driver input without twitching or drift.

<div align="center">
 <table>
     <tr>
         <td><img src="images/radio-controller-front.JPEG" width="360" alt="Radio Controller Front"></td>
         <td><img src="images/radio-controller-back.JPEG" width="360" alt="Radio Controller Back"></td>
     </tr>
     <tr>
         <td><img src="images/radio-controller-mounted.JPEG" width="360" alt="Radio Controller Mounted"></td>
         <td><img src="images/radio-controller-complete.JPEG" width="360" alt="Radio Controller Complete"></td>
     </tr>
 </table>
</div>

### RF Transmitter<a name="rftransmitter"></a>
The remote uses an nRF24L01+ module with RF24 over SPI to transmit `MotorControlPayload` packets containing `ocrMotor` and `ocrSteering`. During initialization, firmware performs a radio hardware check, configures dynamic payload mode, and sets TX/RX addresses for the paired node. Timer0 is also configured with a 50 us base so the `millis()`/`micros()` timing used by RF24 remains stable while packets are sent continuously.

### Control Input Processing<a name="controlinputprocessing"></a>
Joystick values are read through the ADC and mapped into the exact command domains expected by the vehicle firmware. This preprocessing keeps control semantics consistent across the RF link: signed throttle around zero and steering in absolute pulse-width units centered at neutral.

#### Throttle Input (Left Analog Stick)<a name="throttleprocessing"></a>
Throttle is sampled on ADC channel 0, inverted to match intuitive stick direction, scaled into a signed drive command, and centered around zero. Positive values represent forward demand, negative values represent reverse demand, and near-zero commands cooperate with the car-side deadband to prevent idle creep.

#### Steering Input (Right Analog Stick)<a name="steeringprocessing"></a>
Steering is sampled on ADC channel 1, mapped around the 1900 us center point, and clamped to the valid steering window before transmission. A center deadzone is applied so small stick jitter and ADC noise snap back to neutral, reducing servo chatter and improving straight-line stability.

### Charging Notes<a name="remotecharging"></a>
The remote has a built-in LiPo charging circuit. When a charger is connected, the controller may automatically power on while charging. You can place the power switch in the OFF position before charging if you prefer it to stay off.

---
### Parts List<a name="remotepartslist"></a>
|_**Part Number**_|_**Quantity**_|
|:-----|:--------:|
|<a href="https://www.digikey.com/en/products/detail/jst-sales-america-inc/S2B-PH-SM4-TB/926655">JST Connector</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/samsung-electro-mechanics/CL10A475KP8NNNC/3886702">4.7uF Capacitor</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/taiyo-yuden/TMK107B7105KA-T/2714162">1uF Capacitor</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/murata-electronics/GRM188R61E225KA12D/4905349">2.2uF Capacitor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/kemet/C0603C104K3RACTU/416044">.1uF Capacitor</a>| x4 |
|<a href="https://www.digikey.com/en/products/detail/venkel/CTL1206FRD1T/13245061">LED</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/gct/USB4125-GF-A/13547384">USB_C_Receptacle_USB2.0_14P</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/w-rth-elektronik/61201021621/2060590?gclsrc=aw.ds&gad_source=1&gad_campaignid=20234014242&gbraid=0AAAAADrbLliydMvIIXJNX99D3SyONNFow&gclid=CjwKCAjwtvvPBhBuEiwAPMijr9iKDJDvfxlLCPFpasA9r130lziQwH1Oz1Nx0EE10dvVf4E5pzsRdxoCSPkQAvD_BwE">AVR-ISP-10</a>| x1 |
|<a href="https://www.amazon.com/2-54mm-Breakaway-Female-Connector-Arduino/dp/B01MQ48T2V/ref=sr_1_2_sspa?crid=37UI1XQQ1YOGK&dib=eyJ2IjoiMSJ9.dQRG7A390Cr7G0eECwl3ySotcaXfI28I1uUKHcmiPHf-hvTrpk4-mLxeOfBP_o2wgeNP09ah4l4Z_nh4K_ky6uhpN37tS8i2I8bk7hElYNXx-FAMr_ipA9P94e9JhgRABouD4lHJnLV6bc2VHVILnmQpPnT6LynWmghNohKPKs16E5RMkmBnBZLmoQyECbt8tk0IWHyfDAvT8Id0p6m7HikZdiL1Mb9oZ9zR8lDkjQg.dCZRPJ_88iquw6HqnK28jkzvtgYozecW_lBzuIykXWo&dib_tag=se&keywords=header%2Bpins&qid=1778354973&sprefix=header%2Bpins%2Caps%2C146&sr=8-2-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1">nRF24L01+ Headers</a>| x1 |
|<a href="https://www.amazon.com/Automation-Joysticks-Compatible-Controllers-Precision/dp/B09Y2R1GLV">Joystick B09Y2R1GLV</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/infineon-technologies/IRLML6244TRPBF/2393871">IRLML6244</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/yageo/RC0603FR-075K1L/727268">5.1k ohm Resistor</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/koa-speer-electronics-inc/RK73B1JTTD154J/9844780">150k ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/yageo/RC0603JR-071K5L/726689">1.5k ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/stackpole-electronics-inc/RMCF0603JT10K0/1758104">10k ohm Resistor</a>| x2 |
|<a href="https://www.digikey.com/en/products/detail/yageo/RC0603FR-07330RL/727162">330 ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/yageo/RC0603JR-071K3L/726686">1.3k ohm Resistor</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/omron-electronics-inc-emc-div/A6S-1104-PH/3102847">SW_SPST</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/texas-instruments/TPS3840DL35DBVR/15857118">TPS3840</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/microchip-technology/MCP73831T-4ADI-OT/1874437?gclsrc=aw.ds&gad_source=1&gad_campaignid=20790518593&gbraid=0AAAAADrbLlgvRBylAxpp-GjrKMif8Czig&gclid=CjwKCAjwtvvPBhBuEiwAPMijr3YD6e-gwd-PDBrTAI6AF4DaDnOhNqFPZpuGwQkKEnNPv-7e_AYXQRoCGHgQAvD_BwE">MCP73831-2-OT</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/microchip-technology/MIC5225-3-3YM5-TR/1815447">MIC5225-3.3YM5 Voltage Regulator</a>| x1 |
|<a href="https://www.digikey.com/en/products/detail/microchip-technology/ATMEGA164A-AU/2271202">ATmega164A-A</a>| x1 |
|<a href="https://www.amazon.com/HiLetgo-NRF24L01-Wireless-Transceiver-Module/dp/B00LX47OCY/ref=sr_1_1_sspa?crid=1USO9UOFDEZL5&dib=eyJ2IjoiMSJ9.HpGu4TebgrLEY6IjfmGnCKONGE1zifAy342llWfR4vcsJ4_OTj71wcfjuLFi42g9LnfOsZybnBvz4HCtPFZh7IoO0VCtoV4SHTwJkmzj3SyTmBWTWfYEwK0bZ-6KAhnJqpXuroU3ExNMIQ_0sb6zAw01BAymwhYK7jVncUvl8YxZV7HAVItE-ISceLL5caDSPRu-nl4dzw8eF-t1VvKSdHE_Pz68YolGVn5D4_TDIAE.6OIyGVxp3k88Grsb6QKkfbgzYiEAVdqjkrvRE4yyM6M&dib_tag=se&keywords=nrf24l01&qid=1778356552&sprefix=nrf%2Caps%2C232&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1">NRF24L01+ Module</a>| x1 |
|<a href="https://www.amazon.com/EEMB-Rechargeable-Connector-Parrott-Polarity/dp/B0B7R8CS2C/ref=sr_1_9?crid=2PNJXHT8O2BD7&dib=eyJ2IjoiMSJ9.ytY0ATgoZUTp8fkKfIbFwxfJ-RPFGOh8Fljs3SALwOdc0E5um9MqwU3Nf561TyqW5eWp_bfODQRFYKAYV0HQlPTNrv5PexF4DVU2RbBdxKcuSmBcDQuw5ImA3T8MQCieUYtHxHjlPQKIey3tmjioRLIZljCtEmJfrXf2yw62FtfaherhM4QZP-lGmU_qZ4IiXu3H0jwThQ-BqoBcTMGELR8DD_BVenflcsgEmfo4edXbIZRGZtWTI9cXMjF2NkhywL7GquoxWDFoC_LZDs7lVysMe3o4p0KdNnO_wNiqgRM.z8oLo5uD7M9zWKGX-XX1qytnO6Q6kw132uBizXV_u2c&dib_tag=se&keywords=250ma+hour+lipo+battery&qid=1779063612&sprefix=250ma+hour+lipo+battery%2Caps%2C142&sr=8-9">3.7v batter</a>| x1 |
|<a href="https://www.amazon.com/ruthex-Threaded-Insert-pieces-ultrasound/dp/B088QJG676/ref=sr_1_3?crid=327WC4A3U357H&dib=eyJ2IjoiMSJ9.v_Af1Sv2DMXyynp1rO3uN2cjI_LkHpW2Xd3NuL9RUn1yz2Ym6BTHo47Bnvh-mF3rih3MIxPZyDCFRwN3f8bgTgfOqS-2FD8o-WWmIpuq7XJIrvCSKP6iwhtXO1FB6g2J8vFhgMjuaMVbjKPUpn3U8iuk4FPTnzt9IQcKtjUuxKiJM3PQJf3MPDf15V62fIp3_oibwlJtUKJ7oyWbckn9BPgwhhw0z9gTb7ylLCQjULA.KUBGIt58ag9IJXW2aWAnvHSlQ59y6f_kavaLpQD422E&dib_tag=se&keywords=inserts%2Bm2&qid=1778806075&sprefix=inserts%2Bm%2Caps%2C197&sr=8-3&th=1">M2 Inserts</a>| x10 |
|<a href="https://www.amazon.com/Phillips-Countersunk-Electronic-Accessories-Samsung/dp/B07HC3LQYS/ref=sr_1_8?crid=2ISFEC45EBS5Q&dib=eyJ2IjoiMSJ9.sToeJ_cHiwrPYQ_C9rq2gwq_BqFxCk_dAqNz8qbKlTKQla66SuHvAVoMEMQE3FrKbT_cXuKk3EDQL7eTiH6WYWt4xdVsIXdoV99uXBCs7qfK_HdB1wotMUmIz4MZM-fYqvCkvNTV6tpBtMOlWfMOOT3xG69H9dmbPd9TCbIKidT_fAgNMZEY2BR0qPmHW3JfhP0KSYKgUQ9dyiHrulr41WxGtIHFVkrdDqeaHCOoUuo.r_V_du0eyowAqg1oQ__QllHDwAeUxV1av7Z4HFjVBKw&dib_tag=se&keywords=m2%2Bscrews&qid=1778806119&sprefix=m2%2Bscrew%2Caps%2C149&sr=8-8&th=1">M2 Screws</a>| x10 |

## 4. Schematics <a name="schematics"></a>
### Car Schematic<a name="escschematic"></a>
<div>
    <img src="images/drive-and-steering-circuit.JPG" width="700" alt="Drive and Steering Circuit">
</div>

---
### Car Remote Schematic<a name="clockschematic"></a>
<div>
    <img src="images/controller-circuit.JPG" width="700" alt="Controller Circuit">
</div>

## 6. STL Viewer <a name="stlviewer"></a>
### Car Parts <a name="casebase"></a>
```stl
solid ASCII
  facet normal 9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.460356e+01 2.235000e+01 3.578584e+00
      vertex   2.460356e+01 1.323333e+01 3.578584e+00
      vertex   2.447150e+01 2.235000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 3.474057e-16 -2.225209e-01
    outer loop
      vertex   2.447150e+01 2.235000e+01 3.000000e+00
      vertex   2.460356e+01 1.323333e+01 3.578584e+00
      vertex   2.447150e+01 1.323333e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 4.124421e-16 2.225209e-01
    outer loop
      vertex   2.447150e+01 2.235000e+01 3.000000e+00
      vertex   2.447150e+01 1.323333e+01 3.000000e+00
      vertex   2.460356e+01 2.235000e+01 2.421416e+00
    endloop
  endfacet
  facet normal 9.749279e-01 4.335758e-17 2.225209e-01
    outer loop
      vertex   2.460356e+01 2.235000e+01 2.421416e+00
      vertex   2.447150e+01 1.323333e+01 3.000000e+00
      vertex   2.460356e+01 1.323333e+01 2.421416e+00
    endloop
  endfacet
  facet normal 7.818315e-01 1.214852e-16 6.234898e-01
    outer loop
      vertex   2.460356e+01 2.235000e+01 2.421416e+00
      vertex   2.460356e+01 1.323333e+01 2.421416e+00
      vertex   2.497358e+01 2.235000e+01 1.957427e+00
    endloop
  endfacet
  facet normal 7.818315e-01 7.592827e-17 6.234898e-01
    outer loop
      vertex   2.497358e+01 2.235000e+01 1.957427e+00
      vertex   2.460356e+01 1.323333e+01 2.421416e+00
      vertex   2.497358e+01 1.323333e+01 1.957427e+00
    endloop
  endfacet
  facet normal 4.338837e-01 1.097195e-16 9.009689e-01
    outer loop
      vertex   2.497358e+01 2.235000e+01 1.957427e+00
      vertex   2.497358e+01 1.323333e+01 1.957427e+00
      vertex   2.550827e+01 2.235000e+01 1.699933e+00
    endloop
  endfacet
  facet normal 4.338837e-01 1.536073e-16 9.009689e-01
    outer loop
      vertex   2.550827e+01 2.235000e+01 1.699933e+00
      vertex   2.497358e+01 1.323333e+01 1.957427e+00
      vertex   2.550827e+01 1.323333e+01 1.699933e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.704913e-16 1.000000e+00
    outer loop
      vertex   2.550827e+01 2.235000e+01 1.699933e+00
      vertex   2.550827e+01 1.323333e+01 1.699933e+00
      vertex   2.610174e+01 2.235000e+01 1.699933e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.704913e-16 1.000000e+00
    outer loop
      vertex   2.610174e+01 2.235000e+01 1.699933e+00
      vertex   2.550827e+01 1.323333e+01 1.699933e+00
      vertex   2.610174e+01 1.323333e+01 1.699933e+00
    endloop
  endfacet
  facet normal -4.338837e-01 1.536073e-16 9.009689e-01
    outer loop
      vertex   2.610174e+01 2.235000e+01 1.699933e+00
      vertex   2.610174e+01 1.323333e+01 1.699933e+00
      vertex   2.663643e+01 2.235000e+01 1.957427e+00
    endloop
  endfacet
  facet normal -4.338837e-01 8.777562e-17 9.009689e-01
    outer loop
      vertex   2.663643e+01 2.235000e+01 1.957427e+00
      vertex   2.610174e+01 1.323333e+01 1.699933e+00
      vertex   2.663643e+01 1.323333e+01 1.957427e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.074262e-17 6.234898e-01
    outer loop
      vertex   2.663643e+01 2.235000e+01 1.957427e+00
      vertex   2.663643e+01 1.323333e+01 1.957427e+00
      vertex   2.700645e+01 2.235000e+01 2.421416e+00
    endloop
  endfacet
  facet normal -7.818315e-01 1.214852e-16 6.234898e-01
    outer loop
      vertex   2.700645e+01 2.235000e+01 2.421416e+00
      vertex   2.663643e+01 1.323333e+01 1.957427e+00
      vertex   2.700645e+01 1.323333e+01 2.421416e+00
    endloop
  endfacet
  facet normal -9.749279e-01 4.335758e-17 2.225209e-01
    outer loop
      vertex   2.700645e+01 2.235000e+01 2.421416e+00
      vertex   2.700645e+01 1.323333e+01 2.421416e+00
      vertex   2.713850e+01 2.235000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 3.251818e-17 2.225209e-01
    outer loop
      vertex   2.713850e+01 2.235000e+01 3.000000e+00
      vertex   2.700645e+01 1.323333e+01 2.421416e+00
      vertex   2.713850e+01 1.323333e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.713850e+01 2.235000e+01 3.000000e+00
      vertex   2.713850e+01 1.323333e+01 3.000000e+00
      vertex   2.700645e+01 2.235000e+01 3.578584e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -1.083939e-17 -2.225209e-01
    outer loop
      vertex   2.700645e+01 2.235000e+01 3.578584e+00
      vertex   2.713850e+01 1.323333e+01 3.000000e+00
      vertex   2.700645e+01 1.323333e+01 3.578584e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -3.037131e-17 -6.234898e-01
    outer loop
      vertex   2.700645e+01 2.235000e+01 3.578584e+00
      vertex   2.700645e+01 1.323333e+01 3.578584e+00
      vertex   2.663643e+01 2.235000e+01 4.042572e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -1.214852e-16 -6.234898e-01
    outer loop
      vertex   2.663643e+01 2.235000e+01 4.042572e+00
      vertex   2.700645e+01 1.323333e+01 3.578584e+00
      vertex   2.663643e+01 1.323333e+01 4.042572e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -1.755512e-16 -9.009689e-01
    outer loop
      vertex   2.663643e+01 2.235000e+01 4.042572e+00
      vertex   2.663643e+01 1.323333e+01 4.042572e+00
      vertex   2.610174e+01 2.235000e+01 4.300066e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -8.777562e-17 -9.009689e-01
    outer loop
      vertex   2.610174e+01 2.235000e+01 4.300066e+00
      vertex   2.663643e+01 1.323333e+01 4.042572e+00
      vertex   2.610174e+01 1.323333e+01 4.300066e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -9.742359e-17 -1.000000e+00
    outer loop
      vertex   2.610174e+01 2.235000e+01 4.300066e+00
      vertex   2.610174e+01 1.323333e+01 4.300066e+00
      vertex   2.550827e+01 2.235000e+01 4.300066e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -9.742359e-17 -1.000000e+00
    outer loop
      vertex   2.550827e+01 2.235000e+01 4.300066e+00
      vertex   2.610174e+01 1.323333e+01 4.300066e+00
      vertex   2.550827e+01 1.323333e+01 4.300066e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -8.777562e-17 -9.009689e-01
    outer loop
      vertex   2.550827e+01 2.235000e+01 4.300066e+00
      vertex   2.550827e+01 1.323333e+01 4.300066e+00
      vertex   2.497358e+01 2.235000e+01 4.042572e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -1.755512e-16 -9.009689e-01
    outer loop
      vertex   2.497358e+01 2.235000e+01 4.042572e+00
      vertex   2.550827e+01 1.323333e+01 4.300066e+00
      vertex   2.497358e+01 1.323333e+01 4.042572e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -1.214852e-16 -6.234898e-01
    outer loop
      vertex   2.497358e+01 2.235000e+01 4.042572e+00
      vertex   2.497358e+01 1.323333e+01 4.042572e+00
      vertex   2.460356e+01 2.235000e+01 3.578584e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -9.111392e-17 -6.234898e-01
    outer loop
      vertex   2.460356e+01 2.235000e+01 3.578584e+00
      vertex   2.497358e+01 1.323333e+01 4.042572e+00
      vertex   2.460356e+01 1.323333e+01 3.578584e+00
    endloop
  endfacet
  facet normal 9.749279e-01 3.251818e-17 2.225209e-01
    outer loop
      vertex   2.460356e+01 -5.000000e+00 2.421416e+00
      vertex   2.460356e+01 4.116667e+00 2.421416e+00
      vertex   2.447150e+01 -5.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -3.474057e-16 2.225209e-01
    outer loop
      vertex   2.447150e+01 -5.000000e+00 3.000000e+00
      vertex   2.460356e+01 4.116667e+00 2.421416e+00
      vertex   2.447150e+01 4.116667e+00 3.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -4.124421e-16 -2.225209e-01
    outer loop
      vertex   2.447150e+01 -5.000000e+00 3.000000e+00
      vertex   2.447150e+01 4.116667e+00 3.000000e+00
      vertex   2.460356e+01 -5.000000e+00 3.578584e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.460356e+01 -5.000000e+00 3.578584e+00
      vertex   2.447150e+01 4.116667e+00 3.000000e+00
      vertex   2.460356e+01 4.116667e+00 3.578584e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -9.111392e-17 -6.234898e-01
    outer loop
      vertex   2.460356e+01 -5.000000e+00 3.578584e+00
      vertex   2.460356e+01 4.116667e+00 3.578584e+00
      vertex   2.497358e+01 -5.000000e+00 4.042572e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -6.074262e-17 -6.234898e-01
    outer loop
      vertex   2.497358e+01 -5.000000e+00 4.042572e+00
      vertex   2.460356e+01 4.116667e+00 3.578584e+00
      vertex   2.497358e+01 4.116667e+00 4.042572e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -8.777562e-17 -9.009689e-01
    outer loop
      vertex   2.497358e+01 -5.000000e+00 4.042572e+00
      vertex   2.497358e+01 4.116667e+00 4.042572e+00
      vertex   2.550827e+01 -5.000000e+00 4.300066e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -8.777562e-17 -9.009689e-01
    outer loop
      vertex   2.550827e+01 -5.000000e+00 4.300066e+00
      vertex   2.497358e+01 4.116667e+00 4.042572e+00
      vertex   2.550827e+01 4.116667e+00 4.300066e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -9.742359e-17 -1.000000e+00
    outer loop
      vertex   2.550827e+01 -5.000000e+00 4.300066e+00
      vertex   2.550827e+01 4.116667e+00 4.300066e+00
      vertex   2.610174e+01 -5.000000e+00 4.300066e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -9.742359e-17 -1.000000e+00
    outer loop
      vertex   2.610174e+01 -5.000000e+00 4.300066e+00
      vertex   2.550827e+01 4.116667e+00 4.300066e+00
      vertex   2.610174e+01 4.116667e+00 4.300066e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -8.777562e-17 -9.009689e-01
    outer loop
      vertex   2.610174e+01 -5.000000e+00 4.300066e+00
      vertex   2.610174e+01 4.116667e+00 4.300066e+00
      vertex   2.663643e+01 -5.000000e+00 4.042572e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -8.777562e-17 -9.009689e-01
    outer loop
      vertex   2.663643e+01 -5.000000e+00 4.042572e+00
      vertex   2.610174e+01 4.116667e+00 4.300066e+00
      vertex   2.663643e+01 4.116667e+00 4.042572e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -6.074262e-17 -6.234898e-01
    outer loop
      vertex   2.663643e+01 -5.000000e+00 4.042572e+00
      vertex   2.663643e+01 4.116667e+00 4.042572e+00
      vertex   2.700645e+01 -5.000000e+00 3.578584e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -9.111392e-17 -6.234898e-01
    outer loop
      vertex   2.700645e+01 -5.000000e+00 3.578584e+00
      vertex   2.663643e+01 4.116667e+00 4.042572e+00
      vertex   2.700645e+01 4.116667e+00 3.578584e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.700645e+01 -5.000000e+00 3.578584e+00
      vertex   2.700645e+01 4.116667e+00 3.578584e+00
      vertex   2.713850e+01 -5.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.713850e+01 -5.000000e+00 3.000000e+00
      vertex   2.700645e+01 4.116667e+00 3.578584e+00
      vertex   2.713850e+01 4.116667e+00 3.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 3.251818e-17 2.225209e-01
    outer loop
      vertex   2.713850e+01 -5.000000e+00 3.000000e+00
      vertex   2.713850e+01 4.116667e+00 3.000000e+00
      vertex   2.700645e+01 -5.000000e+00 2.421416e+00
    endloop
  endfacet
  facet normal -9.749279e-01 2.167879e-17 2.225209e-01
    outer loop
      vertex   2.700645e+01 -5.000000e+00 2.421416e+00
      vertex   2.713850e+01 4.116667e+00 3.000000e+00
      vertex   2.700645e+01 4.116667e+00 2.421416e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.074262e-17 6.234898e-01
    outer loop
      vertex   2.700645e+01 -5.000000e+00 2.421416e+00
      vertex   2.700645e+01 4.116667e+00 2.421416e+00
      vertex   2.663643e+01 -5.000000e+00 1.957427e+00
    endloop
  endfacet
  facet normal -7.818315e-01 1.214852e-16 6.234898e-01
    outer loop
      vertex   2.663643e+01 -5.000000e+00 1.957427e+00
      vertex   2.700645e+01 4.116667e+00 2.421416e+00
      vertex   2.663643e+01 4.116667e+00 1.957427e+00
    endloop
  endfacet
  facet normal -4.338837e-01 1.755512e-16 9.009689e-01
    outer loop
      vertex   2.663643e+01 -5.000000e+00 1.957427e+00
      vertex   2.663643e+01 4.116667e+00 1.957427e+00
      vertex   2.610174e+01 -5.000000e+00 1.699933e+00
    endloop
  endfacet
  facet normal -4.338837e-01 1.097195e-16 9.009689e-01
    outer loop
      vertex   2.610174e+01 -5.000000e+00 1.699933e+00
      vertex   2.663643e+01 4.116667e+00 1.957427e+00
      vertex   2.610174e+01 4.116667e+00 1.699933e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.217795e-16 1.000000e+00
    outer loop
      vertex   2.610174e+01 -5.000000e+00 1.699933e+00
      vertex   2.610174e+01 4.116667e+00 1.699933e+00
      vertex   2.550827e+01 -5.000000e+00 1.699933e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.217795e-16 1.000000e+00
    outer loop
      vertex   2.550827e+01 -5.000000e+00 1.699933e+00
      vertex   2.610174e+01 4.116667e+00 1.699933e+00
      vertex   2.550827e+01 4.116667e+00 1.699933e+00
    endloop
  endfacet
  facet normal 4.338837e-01 1.097195e-16 9.009689e-01
    outer loop
      vertex   2.550827e+01 -5.000000e+00 1.699933e+00
      vertex   2.550827e+01 4.116667e+00 1.699933e+00
      vertex   2.497358e+01 -5.000000e+00 1.957427e+00
    endloop
  endfacet
  facet normal 4.338837e-01 1.316634e-16 9.009689e-01
    outer loop
      vertex   2.497358e+01 -5.000000e+00 1.957427e+00
      vertex   2.550827e+01 4.116667e+00 1.699933e+00
      vertex   2.497358e+01 4.116667e+00 1.957427e+00
    endloop
  endfacet
  facet normal 7.818315e-01 9.111392e-17 6.234898e-01
    outer loop
      vertex   2.497358e+01 -5.000000e+00 1.957427e+00
      vertex   2.497358e+01 4.116667e+00 1.957427e+00
      vertex   2.460356e+01 -5.000000e+00 2.421416e+00
    endloop
  endfacet
  facet normal 7.818315e-01 9.111392e-17 6.234898e-01
    outer loop
      vertex   2.460356e+01 -5.000000e+00 2.421416e+00
      vertex   2.497358e+01 4.116667e+00 1.957427e+00
      vertex   2.460356e+01 4.116667e+00 2.421416e+00
    endloop
  endfacet
  facet normal 9.749279e-01 3.251818e-17 2.225209e-01
    outer loop
      vertex   2.447150e+01 1.323333e+01 3.000000e+00
      vertex   2.447150e+01 4.116667e+00 3.000000e+00
      vertex   2.460356e+01 1.323333e+01 2.421416e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.167879e-17 2.225209e-01
    outer loop
      vertex   2.460356e+01 1.323333e+01 2.421416e+00
      vertex   2.447150e+01 4.116667e+00 3.000000e+00
      vertex   2.460356e+01 4.116667e+00 2.421416e+00
    endloop
  endfacet
  facet normal 7.818315e-01 6.074262e-17 6.234898e-01
    outer loop
      vertex   2.460356e+01 1.323333e+01 2.421416e+00
      vertex   2.460356e+01 4.116667e+00 2.421416e+00
      vertex   2.497358e+01 1.323333e+01 1.957427e+00
    endloop
  endfacet
  facet normal 7.818315e-01 9.111392e-17 6.234898e-01
    outer loop
      vertex   2.497358e+01 1.323333e+01 1.957427e+00
      vertex   2.460356e+01 4.116667e+00 2.421416e+00
      vertex   2.497358e+01 4.116667e+00 1.957427e+00
    endloop
  endfacet
  facet normal 4.338837e-01 1.316634e-16 9.009689e-01
    outer loop
      vertex   2.497358e+01 1.323333e+01 1.957427e+00
      vertex   2.497358e+01 4.116667e+00 1.957427e+00
      vertex   2.550827e+01 1.323333e+01 1.699933e+00
    endloop
  endfacet
  facet normal 4.338837e-01 1.316634e-16 9.009689e-01
    outer loop
      vertex   2.550827e+01 1.323333e+01 1.699933e+00
      vertex   2.497358e+01 4.116667e+00 1.957427e+00
      vertex   2.550827e+01 4.116667e+00 1.699933e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.461354e-16 1.000000e+00
    outer loop
      vertex   2.550827e+01 1.323333e+01 1.699933e+00
      vertex   2.550827e+01 4.116667e+00 1.699933e+00
      vertex   2.610174e+01 1.323333e+01 1.699933e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.461354e-16 1.000000e+00
    outer loop
      vertex   2.610174e+01 1.323333e+01 1.699933e+00
      vertex   2.550827e+01 4.116667e+00 1.699933e+00
      vertex   2.610174e+01 4.116667e+00 1.699933e+00
    endloop
  endfacet
  facet normal -4.338837e-01 1.316634e-16 9.009689e-01
    outer loop
      vertex   2.610174e+01 1.323333e+01 1.699933e+00
      vertex   2.610174e+01 4.116667e+00 1.699933e+00
      vertex   2.663643e+01 1.323333e+01 1.957427e+00
    endloop
  endfacet
  facet normal -4.338837e-01 1.316634e-16 9.009689e-01
    outer loop
      vertex   2.663643e+01 1.323333e+01 1.957427e+00
      vertex   2.610174e+01 4.116667e+00 1.699933e+00
      vertex   2.663643e+01 4.116667e+00 1.957427e+00
    endloop
  endfacet
  facet normal -7.818315e-01 9.111392e-17 6.234898e-01
    outer loop
      vertex   2.663643e+01 1.323333e+01 1.957427e+00
      vertex   2.663643e+01 4.116667e+00 1.957427e+00
      vertex   2.700645e+01 1.323333e+01 2.421416e+00
    endloop
  endfacet
  facet normal -7.818315e-01 9.111392e-17 6.234898e-01
    outer loop
      vertex   2.700645e+01 1.323333e+01 2.421416e+00
      vertex   2.663643e+01 4.116667e+00 1.957427e+00
      vertex   2.700645e+01 4.116667e+00 2.421416e+00
    endloop
  endfacet
  facet normal -9.749279e-01 3.251818e-17 2.225209e-01
    outer loop
      vertex   2.700645e+01 1.323333e+01 2.421416e+00
      vertex   2.700645e+01 4.116667e+00 2.421416e+00
      vertex   2.713850e+01 1.323333e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 3.251818e-17 2.225209e-01
    outer loop
      vertex   2.713850e+01 1.323333e+01 3.000000e+00
      vertex   2.700645e+01 4.116667e+00 2.421416e+00
      vertex   2.713850e+01 4.116667e+00 3.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.713850e+01 1.323333e+01 3.000000e+00
      vertex   2.713850e+01 4.116667e+00 3.000000e+00
      vertex   2.700645e+01 1.323333e+01 3.578584e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -4.335758e-17 -2.225209e-01
    outer loop
      vertex   2.700645e+01 1.323333e+01 3.578584e+00
      vertex   2.713850e+01 4.116667e+00 3.000000e+00
      vertex   2.700645e+01 4.116667e+00 3.578584e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -1.214852e-16 -6.234898e-01
    outer loop
      vertex   2.700645e+01 1.323333e+01 3.578584e+00
      vertex   2.700645e+01 4.116667e+00 3.578584e+00
      vertex   2.663643e+01 1.323333e+01 4.042572e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -1.214852e-16 -6.234898e-01
    outer loop
      vertex   2.663643e+01 1.323333e+01 4.042572e+00
      vertex   2.700645e+01 4.116667e+00 3.578584e+00
      vertex   2.663643e+01 4.116667e+00 4.042572e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -1.755512e-16 -9.009689e-01
    outer loop
      vertex   2.663643e+01 1.323333e+01 4.042572e+00
      vertex   2.663643e+01 4.116667e+00 4.042572e+00
      vertex   2.610174e+01 1.323333e+01 4.300066e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -1.755512e-16 -9.009689e-01
    outer loop
      vertex   2.610174e+01 1.323333e+01 4.300066e+00
      vertex   2.663643e+01 4.116667e+00 4.042572e+00
      vertex   2.610174e+01 4.116667e+00 4.300066e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.948472e-16 -1.000000e+00
    outer loop
      vertex   2.610174e+01 1.323333e+01 4.300066e+00
      vertex   2.610174e+01 4.116667e+00 4.300066e+00
      vertex   2.550827e+01 1.323333e+01 4.300066e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.948472e-16 -1.000000e+00
    outer loop
      vertex   2.550827e+01 1.323333e+01 4.300066e+00
      vertex   2.610174e+01 4.116667e+00 4.300066e+00
      vertex   2.550827e+01 4.116667e+00 4.300066e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -1.755512e-16 -9.009689e-01
    outer loop
      vertex   2.550827e+01 1.323333e+01 4.300066e+00
      vertex   2.550827e+01 4.116667e+00 4.300066e+00
      vertex   2.497358e+01 1.323333e+01 4.042572e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -1.755512e-16 -9.009689e-01
    outer loop
      vertex   2.497358e+01 1.323333e+01 4.042572e+00
      vertex   2.550827e+01 4.116667e+00 4.300066e+00
      vertex   2.497358e+01 4.116667e+00 4.042572e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -1.214852e-16 -6.234898e-01
    outer loop
      vertex   2.497358e+01 1.323333e+01 4.042572e+00
      vertex   2.497358e+01 4.116667e+00 4.042572e+00
      vertex   2.460356e+01 1.323333e+01 3.578584e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -9.111392e-17 -6.234898e-01
    outer loop
      vertex   2.460356e+01 1.323333e+01 3.578584e+00
      vertex   2.497358e+01 4.116667e+00 4.042572e+00
      vertex   2.460356e+01 4.116667e+00 3.578584e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.460356e+01 1.323333e+01 3.578584e+00
      vertex   2.460356e+01 4.116667e+00 3.578584e+00
      vertex   2.447150e+01 1.323333e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -3.251818e-17 -2.225209e-01
    outer loop
      vertex   2.447150e+01 1.323333e+01 3.000000e+00
      vertex   2.460356e+01 4.116667e+00 3.578584e+00
      vertex   2.447150e+01 4.116667e+00 3.000000e+00
    endloop
  endfacet
  facet normal -9.848078e-01 -1.691743e-17 -1.736482e-01
    outer loop
      vertex   2.345577e+01 2.235000e+01 1.644949e+00
      vertex   2.345577e+01 8.675000e+00 1.644949e+00
      vertex   2.330500e+01 2.235000e+01 2.500000e+00
    endloop
  endfacet
  facet normal -9.848078e-01 -2.819572e-17 -1.736482e-01
    outer loop
      vertex   2.330500e+01 2.235000e+01 2.500000e+00
      vertex   2.345577e+01 8.675000e+00 1.644949e+00
      vertex   2.330500e+01 8.675000e+00 2.500000e+00
    endloop
  endfacet
  facet normal -9.848078e-01 2.819572e-17 1.736482e-01
    outer loop
      vertex   2.330500e+01 2.235000e+01 2.500000e+00
      vertex   2.330500e+01 8.675000e+00 2.500000e+00
      vertex   2.345577e+01 2.235000e+01 3.355050e+00
    endloop
  endfacet
  facet normal -9.848078e-01 1.691743e-17 1.736482e-01
    outer loop
      vertex   2.345577e+01 2.235000e+01 3.355050e+00
      vertex   2.330500e+01 8.675000e+00 2.500000e+00
      vertex   2.345577e+01 8.675000e+00 3.355050e+00
    endloop
  endfacet
  facet normal -8.660254e-01 4.871180e-17 5.000000e-01
    outer loop
      vertex   2.345577e+01 2.235000e+01 3.355050e+00
      vertex   2.345577e+01 8.675000e+00 3.355050e+00
      vertex   2.388989e+01 2.235000e+01 4.106969e+00
    endloop
  endfacet
  facet normal -8.660254e-01 9.742359e-17 5.000000e-01
    outer loop
      vertex   2.388989e+01 2.235000e+01 4.106969e+00
      vertex   2.345577e+01 8.675000e+00 3.355050e+00
      vertex   2.388989e+01 8.675000e+00 4.106969e+00
    endloop
  endfacet
  facet normal -6.427876e-01 1.492616e-16 7.660444e-01
    outer loop
      vertex   2.388989e+01 2.235000e+01 4.106969e+00
      vertex   2.388989e+01 8.675000e+00 4.106969e+00
      vertex   2.455500e+01 2.235000e+01 4.665063e+00
    endloop
  endfacet
  facet normal -6.427876e-01 9.950774e-17 7.660444e-01
    outer loop
      vertex   2.455500e+01 2.235000e+01 4.665063e+00
      vertex   2.388989e+01 8.675000e+00 4.106969e+00
      vertex   2.455500e+01 8.675000e+00 4.665063e+00
    endloop
  endfacet
  facet normal -3.420201e-01 1.220643e-16 9.396926e-01
    outer loop
      vertex   2.455500e+01 2.235000e+01 4.665063e+00
      vertex   2.455500e+01 8.675000e+00 4.665063e+00
      vertex   2.537088e+01 2.235000e+01 4.962019e+00
    endloop
  endfacet
  facet normal -3.420201e-01 1.830965e-16 9.396926e-01
    outer loop
      vertex   2.537088e+01 2.235000e+01 4.962019e+00
      vertex   2.455500e+01 8.675000e+00 4.665063e+00
      vertex   2.537088e+01 8.675000e+00 4.962019e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.948472e-16 1.000000e+00
    outer loop
      vertex   2.537088e+01 2.235000e+01 4.962019e+00
      vertex   2.537088e+01 8.675000e+00 4.962019e+00
      vertex   2.623912e+01 2.235000e+01 4.962019e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.948472e-16 1.000000e+00
    outer loop
      vertex   2.623912e+01 2.235000e+01 4.962019e+00
      vertex   2.537088e+01 8.675000e+00 4.962019e+00
      vertex   2.623912e+01 8.675000e+00 4.962019e+00
    endloop
  endfacet
  facet normal 3.420201e-01 1.830965e-16 9.396926e-01
    outer loop
      vertex   2.623912e+01 2.235000e+01 4.962019e+00
      vertex   2.623912e+01 8.675000e+00 4.962019e+00
      vertex   2.705500e+01 2.235000e+01 4.665063e+00
    endloop
  endfacet
  facet normal 3.420201e-01 1.220643e-16 9.396926e-01
    outer loop
      vertex   2.705500e+01 2.235000e+01 4.665063e+00
      vertex   2.623912e+01 8.675000e+00 4.962019e+00
      vertex   2.705500e+01 8.675000e+00 4.665063e+00
    endloop
  endfacet
  facet normal 6.427876e-01 9.950774e-17 7.660444e-01
    outer loop
      vertex   2.705500e+01 2.235000e+01 4.665063e+00
      vertex   2.705500e+01 8.675000e+00 4.665063e+00
      vertex   2.772012e+01 2.235000e+01 4.106969e+00
    endloop
  endfacet
  facet normal 6.427876e-01 1.492616e-16 7.660444e-01
    outer loop
      vertex   2.772012e+01 2.235000e+01 4.106969e+00
      vertex   2.705500e+01 8.675000e+00 4.665063e+00
      vertex   2.772012e+01 8.675000e+00 4.106969e+00
    endloop
  endfacet
  facet normal 8.660254e-01 9.742359e-17 5.000000e-01
    outer loop
      vertex   2.772012e+01 2.235000e+01 4.106969e+00
      vertex   2.772012e+01 8.675000e+00 4.106969e+00
      vertex   2.815424e+01 2.235000e+01 3.355050e+00
    endloop
  endfacet
  facet normal 8.660254e-01 4.871180e-17 5.000000e-01
    outer loop
      vertex   2.815424e+01 2.235000e+01 3.355050e+00
      vertex   2.772012e+01 8.675000e+00 4.106969e+00
      vertex   2.815424e+01 8.675000e+00 3.355050e+00
    endloop
  endfacet
  facet normal 9.848078e-01 1.691743e-17 1.736482e-01
    outer loop
      vertex   2.815424e+01 2.235000e+01 3.355050e+00
      vertex   2.815424e+01 8.675000e+00 3.355050e+00
      vertex   2.830500e+01 2.235000e+01 2.500000e+00
    endloop
  endfacet
  facet normal 9.848078e-01 2.255657e-17 1.736482e-01
    outer loop
      vertex   2.830500e+01 2.235000e+01 2.500000e+00
      vertex   2.815424e+01 8.675000e+00 3.355050e+00
      vertex   2.830500e+01 8.675000e+00 2.500000e+00
    endloop
  endfacet
  facet normal 9.848078e-01 -2.255657e-17 -1.736482e-01
    outer loop
      vertex   2.830500e+01 2.235000e+01 2.500000e+00
      vertex   2.830500e+01 8.675000e+00 2.500000e+00
      vertex   2.815424e+01 2.235000e+01 1.644949e+00
    endloop
  endfacet
  facet normal 9.848078e-01 -3.101529e-17 -1.736482e-01
    outer loop
      vertex   2.815424e+01 2.235000e+01 1.644949e+00
      vertex   2.830500e+01 8.675000e+00 2.500000e+00
      vertex   2.815424e+01 8.675000e+00 1.644949e+00
    endloop
  endfacet
  facet normal 8.660254e-01 -8.930496e-17 -5.000000e-01
    outer loop
      vertex   2.815424e+01 2.235000e+01 1.644949e+00
      vertex   2.815424e+01 8.675000e+00 1.644949e+00
      vertex   2.772012e+01 2.235000e+01 8.930306e-01
    endloop
  endfacet
  facet normal 8.660254e-01 -2.841521e-17 -5.000000e-01
    outer loop
      vertex   2.772012e+01 2.235000e+01 8.930306e-01
      vertex   2.815424e+01 8.675000e+00 1.644949e+00
      vertex   2.772012e+01 8.675000e+00 8.930306e-01
    endloop
  endfacet
  facet normal 6.427876e-01 -4.353463e-17 -7.660444e-01
    outer loop
      vertex   2.772012e+01 2.235000e+01 8.930306e-01
      vertex   2.772012e+01 8.675000e+00 8.930306e-01
      vertex   2.705500e+01 2.235000e+01 3.349361e-01
    endloop
  endfacet
  facet normal 6.427876e-01 -8.085004e-17 -7.660444e-01
    outer loop
      vertex   2.705500e+01 2.235000e+01 3.349361e-01
      vertex   2.772012e+01 8.675000e+00 8.930306e-01
      vertex   2.705500e+01 8.675000e+00 3.349361e-01
    endloop
  endfacet
  facet normal 3.420201e-01 -9.917725e-17 -9.396926e-01
    outer loop
      vertex   2.705500e+01 2.235000e+01 3.349361e-01
      vertex   2.705500e+01 8.675000e+00 3.349361e-01
      vertex   2.623912e+01 2.235000e+01 3.798024e-02
    endloop
  endfacet
  facet normal 3.420201e-01 -1.182498e-16 -9.396926e-01
    outer loop
      vertex   2.623912e+01 2.235000e+01 3.798024e-02
      vertex   2.705500e+01 8.675000e+00 3.349361e-01
      vertex   2.623912e+01 8.675000e+00 3.798024e-02
    endloop
  endfacet
  facet normal -0.000000e+00 -1.258388e-16 -1.000000e+00
    outer loop
      vertex   2.623912e+01 2.235000e+01 3.798024e-02
      vertex   2.623912e+01 8.675000e+00 3.798024e-02
      vertex   2.537088e+01 2.235000e+01 3.798024e-02
    endloop
  endfacet
  facet normal 0.000000e+00 -1.258388e-16 -1.000000e+00
    outer loop
      vertex   2.537088e+01 2.235000e+01 3.798024e-02
      vertex   2.623912e+01 8.675000e+00 3.798024e-02
      vertex   2.537088e+01 8.675000e+00 3.798024e-02
    endloop
  endfacet
  facet normal -3.420201e-01 -1.182498e-16 -9.396926e-01
    outer loop
      vertex   2.537088e+01 2.235000e+01 3.798024e-02
      vertex   2.537088e+01 8.675000e+00 3.798024e-02
      vertex   2.455500e+01 2.235000e+01 3.349361e-01
    endloop
  endfacet
  facet normal -3.420201e-01 -1.182498e-16 -9.396926e-01
    outer loop
      vertex   2.455500e+01 2.235000e+01 3.349361e-01
      vertex   2.537088e+01 8.675000e+00 3.798024e-02
      vertex   2.455500e+01 8.675000e+00 3.349361e-01
    endloop
  endfacet
  facet normal -6.427876e-01 -9.639812e-17 -7.660444e-01
    outer loop
      vertex   2.455500e+01 2.235000e+01 3.349361e-01
      vertex   2.455500e+01 8.675000e+00 3.349361e-01
      vertex   2.388989e+01 2.235000e+01 8.930306e-01
    endloop
  endfacet
  facet normal -6.427876e-01 -4.353463e-17 -7.660444e-01
    outer loop
      vertex   2.388989e+01 2.235000e+01 8.930306e-01
      vertex   2.455500e+01 8.675000e+00 3.349361e-01
      vertex   2.388989e+01 8.675000e+00 8.930306e-01
    endloop
  endfacet
  facet normal -8.660254e-01 -2.841521e-17 -5.000000e-01
    outer loop
      vertex   2.388989e+01 2.235000e+01 8.930306e-01
      vertex   2.388989e+01 8.675000e+00 8.930306e-01
      vertex   2.345577e+01 2.235000e+01 1.644949e+00
    endloop
  endfacet
  facet normal -8.660254e-01 -4.871180e-17 -5.000000e-01
    outer loop
      vertex   2.345577e+01 2.235000e+01 1.644949e+00
      vertex   2.388989e+01 8.675000e+00 8.930306e-01
      vertex   2.345577e+01 8.675000e+00 1.644949e+00
    endloop
  endfacet
  facet normal -9.848078e-01 3.383486e-17 1.736482e-01
    outer loop
      vertex   2.345577e+01 -5.000000e+00 3.355050e+00
      vertex   2.345577e+01 8.675000e+00 3.355050e+00
      vertex   2.330500e+01 -5.000000e+00 2.500000e+00
    endloop
  endfacet
  facet normal -9.848078e-01 2.255657e-17 1.736482e-01
    outer loop
      vertex   2.330500e+01 -5.000000e+00 2.500000e+00
      vertex   2.345577e+01 8.675000e+00 3.355050e+00
      vertex   2.330500e+01 8.675000e+00 2.500000e+00
    endloop
  endfacet
  facet normal -9.848078e-01 -2.255657e-17 -1.736482e-01
    outer loop
      vertex   2.330500e+01 -5.000000e+00 2.500000e+00
      vertex   2.330500e+01 8.675000e+00 2.500000e+00
      vertex   2.345577e+01 -5.000000e+00 1.644949e+00
    endloop
  endfacet
  facet normal -9.848078e-01 -2.537614e-17 -1.736482e-01
    outer loop
      vertex   2.345577e+01 -5.000000e+00 1.644949e+00
      vertex   2.330500e+01 8.675000e+00 2.500000e+00
      vertex   2.345577e+01 8.675000e+00 1.644949e+00
    endloop
  endfacet
  facet normal -8.660254e-01 -7.306769e-17 -5.000000e-01
    outer loop
      vertex   2.345577e+01 -5.000000e+00 1.644949e+00
      vertex   2.345577e+01 8.675000e+00 1.644949e+00
      vertex   2.388989e+01 -5.000000e+00 8.930306e-01
    endloop
  endfacet
  facet normal -8.660254e-01 -1.055422e-16 -5.000000e-01
    outer loop
      vertex   2.388989e+01 -5.000000e+00 8.930306e-01
      vertex   2.345577e+01 8.675000e+00 1.644949e+00
      vertex   2.388989e+01 8.675000e+00 8.930306e-01
    endloop
  endfacet
  facet normal -6.427876e-01 -1.617001e-16 -7.660444e-01
    outer loop
      vertex   2.388989e+01 -5.000000e+00 8.930306e-01
      vertex   2.388989e+01 8.675000e+00 8.930306e-01
      vertex   2.455500e+01 -5.000000e+00 3.349361e-01
    endloop
  endfacet
  facet normal -6.427876e-01 -1.057270e-16 -7.660444e-01
    outer loop
      vertex   2.455500e+01 -5.000000e+00 3.349361e-01
      vertex   2.388989e+01 8.675000e+00 8.930306e-01
      vertex   2.455500e+01 8.675000e+00 3.349361e-01
    endloop
  endfacet
  facet normal -3.420201e-01 -1.296933e-16 -9.396926e-01
    outer loop
      vertex   2.455500e+01 -5.000000e+00 3.349361e-01
      vertex   2.455500e+01 8.675000e+00 3.349361e-01
      vertex   2.537088e+01 -5.000000e+00 3.798024e-02
    endloop
  endfacet
  facet normal -3.420201e-01 -1.296933e-16 -9.396926e-01
    outer loop
      vertex   2.537088e+01 -5.000000e+00 3.798024e-02
      vertex   2.455500e+01 8.675000e+00 3.349361e-01
      vertex   2.537088e+01 8.675000e+00 3.798024e-02
    endloop
  endfacet
  facet normal 0.000000e+00 -1.380168e-16 -1.000000e+00
    outer loop
      vertex   2.537088e+01 -5.000000e+00 3.798024e-02
      vertex   2.537088e+01 8.675000e+00 3.798024e-02
      vertex   2.623912e+01 -5.000000e+00 3.798024e-02
    endloop
  endfacet
  facet normal 0.000000e+00 -1.380168e-16 -1.000000e+00
    outer loop
      vertex   2.623912e+01 -5.000000e+00 3.798024e-02
      vertex   2.537088e+01 8.675000e+00 3.798024e-02
      vertex   2.623912e+01 8.675000e+00 3.798024e-02
    endloop
  endfacet
  facet normal 3.420201e-01 -1.296933e-16 -9.396926e-01
    outer loop
      vertex   2.623912e+01 -5.000000e+00 3.798024e-02
      vertex   2.623912e+01 8.675000e+00 3.798024e-02
      vertex   2.705500e+01 -5.000000e+00 3.349361e-01
    endloop
  endfacet
  facet normal 3.420201e-01 -1.487659e-16 -9.396926e-01
    outer loop
      vertex   2.705500e+01 -5.000000e+00 3.349361e-01
      vertex   2.623912e+01 8.675000e+00 3.798024e-02
      vertex   2.705500e+01 8.675000e+00 3.349361e-01
    endloop
  endfacet
  facet normal 6.427876e-01 -1.212751e-16 -7.660444e-01
    outer loop
      vertex   2.705500e+01 -5.000000e+00 3.349361e-01
      vertex   2.705500e+01 8.675000e+00 3.349361e-01
      vertex   2.772012e+01 -5.000000e+00 8.930306e-01
    endloop
  endfacet
  facet normal 6.427876e-01 -1.554808e-16 -7.660444e-01
    outer loop
      vertex   2.772012e+01 -5.000000e+00 8.930306e-01
      vertex   2.705500e+01 8.675000e+00 3.349361e-01
      vertex   2.772012e+01 8.675000e+00 8.930306e-01
    endloop
  endfacet
  facet normal 8.660254e-01 -1.014829e-16 -5.000000e-01
    outer loop
      vertex   2.772012e+01 -5.000000e+00 8.930306e-01
      vertex   2.772012e+01 8.675000e+00 8.930306e-01
      vertex   2.815424e+01 -5.000000e+00 1.644949e+00
    endloop
  endfacet
  facet normal 8.660254e-01 -4.871180e-17 -5.000000e-01
    outer loop
      vertex   2.815424e+01 -5.000000e+00 1.644949e+00
      vertex   2.772012e+01 8.675000e+00 8.930306e-01
      vertex   2.815424e+01 8.675000e+00 1.644949e+00
    endloop
  endfacet
  facet normal 9.848078e-01 -1.691743e-17 -1.736482e-01
    outer loop
      vertex   2.815424e+01 -5.000000e+00 1.644949e+00
      vertex   2.815424e+01 8.675000e+00 1.644949e+00
      vertex   2.830500e+01 -5.000000e+00 2.500000e+00
    endloop
  endfacet
  facet normal 9.848078e-01 -2.255657e-17 -1.736482e-01
    outer loop
      vertex   2.830500e+01 -5.000000e+00 2.500000e+00
      vertex   2.815424e+01 8.675000e+00 1.644949e+00
      vertex   2.830500e+01 8.675000e+00 2.500000e+00
    endloop
  endfacet
  facet normal 9.848078e-01 2.255657e-17 1.736482e-01
    outer loop
      vertex   2.830500e+01 -5.000000e+00 2.500000e+00
      vertex   2.830500e+01 8.675000e+00 2.500000e+00
      vertex   2.815424e+01 -5.000000e+00 3.355050e+00
    endloop
  endfacet
  facet normal 9.848078e-01 2.819572e-17 1.736482e-01
    outer loop
      vertex   2.815424e+01 -5.000000e+00 3.355050e+00
      vertex   2.830500e+01 8.675000e+00 2.500000e+00
      vertex   2.815424e+01 8.675000e+00 3.355050e+00
    endloop
  endfacet
  facet normal 8.660254e-01 8.118633e-17 5.000000e-01
    outer loop
      vertex   2.815424e+01 -5.000000e+00 3.355050e+00
      vertex   2.815424e+01 8.675000e+00 3.355050e+00
      vertex   2.772012e+01 -5.000000e+00 4.106969e+00
    endloop
  endfacet
  facet normal 8.660254e-01 3.247453e-17 5.000000e-01
    outer loop
      vertex   2.772012e+01 -5.000000e+00 4.106969e+00
      vertex   2.815424e+01 8.675000e+00 3.355050e+00
      vertex   2.772012e+01 8.675000e+00 4.106969e+00
    endloop
  endfacet
  facet normal 6.427876e-01 4.975387e-17 7.660444e-01
    outer loop
      vertex   2.772012e+01 -5.000000e+00 4.106969e+00
      vertex   2.772012e+01 8.675000e+00 4.106969e+00
      vertex   2.705500e+01 -5.000000e+00 4.665063e+00
    endloop
  endfacet
  facet normal 6.427876e-01 9.950774e-17 7.660444e-01
    outer loop
      vertex   2.705500e+01 -5.000000e+00 4.665063e+00
      vertex   2.772012e+01 8.675000e+00 4.106969e+00
      vertex   2.705500e+01 8.675000e+00 4.665063e+00
    endloop
  endfacet
  facet normal 3.420201e-01 1.220643e-16 9.396926e-01
    outer loop
      vertex   2.705500e+01 -5.000000e+00 4.665063e+00
      vertex   2.705500e+01 8.675000e+00 4.665063e+00
      vertex   2.623912e+01 -5.000000e+00 4.962019e+00
    endloop
  endfacet
  facet normal 3.420201e-01 6.103215e-17 9.396926e-01
    outer loop
      vertex   2.623912e+01 -5.000000e+00 4.962019e+00
      vertex   2.705500e+01 8.675000e+00 4.665063e+00
      vertex   2.623912e+01 8.675000e+00 4.962019e+00
    endloop
  endfacet
  facet normal 0.000000e+00 6.494906e-17 1.000000e+00
    outer loop
      vertex   2.623912e+01 -5.000000e+00 4.962019e+00
      vertex   2.623912e+01 8.675000e+00 4.962019e+00
      vertex   2.537088e+01 -5.000000e+00 4.962019e+00
    endloop
  endfacet
  facet normal 0.000000e+00 6.494906e-17 1.000000e+00
    outer loop
      vertex   2.537088e+01 -5.000000e+00 4.962019e+00
      vertex   2.623912e+01 8.675000e+00 4.962019e+00
      vertex   2.537088e+01 8.675000e+00 4.962019e+00
    endloop
  endfacet
  facet normal -3.420201e-01 6.103215e-17 9.396926e-01
    outer loop
      vertex   2.537088e+01 -5.000000e+00 4.962019e+00
      vertex   2.537088e+01 8.675000e+00 4.962019e+00
      vertex   2.455500e+01 -5.000000e+00 4.665063e+00
    endloop
  endfacet
  facet normal -3.420201e-01 1.220643e-16 9.396926e-01
    outer loop
      vertex   2.455500e+01 -5.000000e+00 4.665063e+00
      vertex   2.537088e+01 8.675000e+00 4.962019e+00
      vertex   2.455500e+01 8.675000e+00 4.665063e+00
    endloop
  endfacet
  facet normal -6.427876e-01 9.950774e-17 7.660444e-01
    outer loop
      vertex   2.455500e+01 -5.000000e+00 4.665063e+00
      vertex   2.455500e+01 8.675000e+00 4.665063e+00
      vertex   2.388989e+01 -5.000000e+00 4.106969e+00
    endloop
  endfacet
  facet normal -6.427876e-01 4.975387e-17 7.660444e-01
    outer loop
      vertex   2.388989e+01 -5.000000e+00 4.106969e+00
      vertex   2.455500e+01 8.675000e+00 4.665063e+00
      vertex   2.388989e+01 8.675000e+00 4.106969e+00
    endloop
  endfacet
  facet normal -8.660254e-01 3.247453e-17 5.000000e-01
    outer loop
      vertex   2.388989e+01 -5.000000e+00 4.106969e+00
      vertex   2.388989e+01 8.675000e+00 4.106969e+00
      vertex   2.345577e+01 -5.000000e+00 3.355050e+00
    endloop
  endfacet
  facet normal -8.660254e-01 9.742359e-17 5.000000e-01
    outer loop
      vertex   2.345577e+01 -5.000000e+00 3.355050e+00
      vertex   2.388989e+01 8.675000e+00 4.106969e+00
      vertex   2.345577e+01 8.675000e+00 3.355050e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.460356e+01 2.235000e+01 2.421416e+00
      vertex   2.330500e+01 2.235000e+01 2.500000e+00
      vertex   2.447150e+01 2.235000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.447150e+01 2.235000e+01 3.000000e+00
      vertex   2.330500e+01 2.235000e+01 2.500000e+00
      vertex   2.345577e+01 2.235000e+01 3.355050e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.447150e+01 2.235000e+01 3.000000e+00
      vertex   2.345577e+01 2.235000e+01 3.355050e+00
      vertex   2.460356e+01 2.235000e+01 3.578584e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.460356e+01 2.235000e+01 3.578584e+00
      vertex   2.345577e+01 2.235000e+01 3.355050e+00
      vertex   2.388989e+01 2.235000e+01 4.106969e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.460356e+01 2.235000e+01 3.578584e+00
      vertex   2.388989e+01 2.235000e+01 4.106969e+00
      vertex   2.497358e+01 2.235000e+01 4.042572e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.497358e+01 2.235000e+01 4.042572e+00
      vertex   2.388989e+01 2.235000e+01 4.106969e+00
      vertex   2.455500e+01 2.235000e+01 4.665063e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.497358e+01 2.235000e+01 4.042572e+00
      vertex   2.455500e+01 2.235000e+01 4.665063e+00
      vertex   2.550827e+01 2.235000e+01 4.300066e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.550827e+01 2.235000e+01 4.300066e+00
      vertex   2.455500e+01 2.235000e+01 4.665063e+00
      vertex   2.537088e+01 2.235000e+01 4.962019e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.550827e+01 2.235000e+01 4.300066e+00
      vertex   2.537088e+01 2.235000e+01 4.962019e+00
      vertex   2.610174e+01 2.235000e+01 4.300066e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.610174e+01 2.235000e+01 4.300066e+00
      vertex   2.537088e+01 2.235000e+01 4.962019e+00
      vertex   2.623912e+01 2.235000e+01 4.962019e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.610174e+01 2.235000e+01 4.300066e+00
      vertex   2.623912e+01 2.235000e+01 4.962019e+00
      vertex   2.705500e+01 2.235000e+01 4.665063e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.330500e+01 2.235000e+01 2.500000e+00
      vertex   2.460356e+01 2.235000e+01 2.421416e+00
      vertex   2.345577e+01 2.235000e+01 1.644949e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.345577e+01 2.235000e+01 1.644949e+00
      vertex   2.460356e+01 2.235000e+01 2.421416e+00
      vertex   2.497358e+01 2.235000e+01 1.957427e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.345577e+01 2.235000e+01 1.644949e+00
      vertex   2.497358e+01 2.235000e+01 1.957427e+00
      vertex   2.388989e+01 2.235000e+01 8.930306e-01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.388989e+01 2.235000e+01 8.930306e-01
      vertex   2.497358e+01 2.235000e+01 1.957427e+00
      vertex   2.455500e+01 2.235000e+01 3.349361e-01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.455500e+01 2.235000e+01 3.349361e-01
      vertex   2.497358e+01 2.235000e+01 1.957427e+00
      vertex   2.550827e+01 2.235000e+01 1.699933e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.455500e+01 2.235000e+01 3.349361e-01
      vertex   2.550827e+01 2.235000e+01 1.699933e+00
      vertex   2.537088e+01 2.235000e+01 3.798024e-02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.537088e+01 2.235000e+01 3.798024e-02
      vertex   2.550827e+01 2.235000e+01 1.699933e+00
      vertex   2.610174e+01 2.235000e+01 1.699933e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.537088e+01 2.235000e+01 3.798024e-02
      vertex   2.610174e+01 2.235000e+01 1.699933e+00
      vertex   2.623912e+01 2.235000e+01 3.798024e-02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.623912e+01 2.235000e+01 3.798024e-02
      vertex   2.610174e+01 2.235000e+01 1.699933e+00
      vertex   2.705500e+01 2.235000e+01 3.349361e-01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.705500e+01 2.235000e+01 3.349361e-01
      vertex   2.610174e+01 2.235000e+01 1.699933e+00
      vertex   2.663643e+01 2.235000e+01 1.957427e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.705500e+01 2.235000e+01 3.349361e-01
      vertex   2.663643e+01 2.235000e+01 1.957427e+00
      vertex   2.772012e+01 2.235000e+01 8.930306e-01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.772012e+01 2.235000e+01 8.930306e-01
      vertex   2.663643e+01 2.235000e+01 1.957427e+00
      vertex   2.815424e+01 2.235000e+01 1.644949e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.815424e+01 2.235000e+01 1.644949e+00
      vertex   2.663643e+01 2.235000e+01 1.957427e+00
      vertex   2.700645e+01 2.235000e+01 2.421416e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.815424e+01 2.235000e+01 1.644949e+00
      vertex   2.700645e+01 2.235000e+01 2.421416e+00
      vertex   2.830500e+01 2.235000e+01 2.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.830500e+01 2.235000e+01 2.500000e+00
      vertex   2.700645e+01 2.235000e+01 2.421416e+00
      vertex   2.713850e+01 2.235000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.830500e+01 2.235000e+01 2.500000e+00
      vertex   2.713850e+01 2.235000e+01 3.000000e+00
      vertex   2.815424e+01 2.235000e+01 3.355050e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.815424e+01 2.235000e+01 3.355050e+00
      vertex   2.713850e+01 2.235000e+01 3.000000e+00
      vertex   2.700645e+01 2.235000e+01 3.578584e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.815424e+01 2.235000e+01 3.355050e+00
      vertex   2.700645e+01 2.235000e+01 3.578584e+00
      vertex   2.772012e+01 2.235000e+01 4.106969e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.772012e+01 2.235000e+01 4.106969e+00
      vertex   2.700645e+01 2.235000e+01 3.578584e+00
      vertex   2.663643e+01 2.235000e+01 4.042572e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.772012e+01 2.235000e+01 4.106969e+00
      vertex   2.663643e+01 2.235000e+01 4.042572e+00
      vertex   2.705500e+01 2.235000e+01 4.665063e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.705500e+01 2.235000e+01 4.665063e+00
      vertex   2.663643e+01 2.235000e+01 4.042572e+00
      vertex   2.610174e+01 2.235000e+01 4.300066e+00
    endloop
  endfacet
  facet normal 3.128681e-16 -1.000000e+00 -1.606500e-15
    outer loop
      vertex   2.460356e+01 -5.000000e+00 3.578584e+00
      vertex   2.345577e+01 -5.000000e+00 3.355050e+00
      vertex   2.447150e+01 -5.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 4.816419e-16 -1.000000e+00 -1.123670e-15
    outer loop
      vertex   2.447150e+01 -5.000000e+00 3.000000e+00
      vertex   2.345577e+01 -5.000000e+00 3.355050e+00
      vertex   2.330500e+01 -5.000000e+00 2.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.447150e+01 -5.000000e+00 3.000000e+00
      vertex   2.330500e+01 -5.000000e+00 2.500000e+00
      vertex   2.460356e+01 -5.000000e+00 2.421416e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.460356e+01 -5.000000e+00 2.421416e+00
      vertex   2.330500e+01 -5.000000e+00 2.500000e+00
      vertex   2.345577e+01 -5.000000e+00 1.644949e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.460356e+01 -5.000000e+00 2.421416e+00
      vertex   2.345577e+01 -5.000000e+00 1.644949e+00
      vertex   2.497358e+01 -5.000000e+00 1.957427e+00
    endloop
  endfacet
  facet normal 2.173480e-16 -1.000000e+00 -1.055730e-15
    outer loop
      vertex   2.497358e+01 -5.000000e+00 1.957427e+00
      vertex   2.345577e+01 -5.000000e+00 1.644949e+00
      vertex   2.388989e+01 -5.000000e+00 8.930306e-01
    endloop
  endfacet
  facet normal 3.425553e-16 -1.000000e+00 -1.183207e-15
    outer loop
      vertex   2.497358e+01 -5.000000e+00 1.957427e+00
      vertex   2.388989e+01 -5.000000e+00 8.930306e-01
      vertex   2.455500e+01 -5.000000e+00 3.349361e-01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.345577e+01 -5.000000e+00 3.355050e+00
      vertex   2.460356e+01 -5.000000e+00 3.578584e+00
      vertex   2.388989e+01 -5.000000e+00 4.106969e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.388989e+01 -5.000000e+00 4.106969e+00
      vertex   2.460356e+01 -5.000000e+00 3.578584e+00
      vertex   2.497358e+01 -5.000000e+00 4.042572e+00
    endloop
  endfacet
  facet normal -8.831541e-17 -1.000000e+00 -1.486198e-15
    outer loop
      vertex   2.388989e+01 -5.000000e+00 4.106969e+00
      vertex   2.497358e+01 -5.000000e+00 4.042572e+00
      vertex   2.455500e+01 -5.000000e+00 4.665063e+00
    endloop
  endfacet
  facet normal 5.190406e-16 -1.000000e+00 -1.077799e-15
    outer loop
      vertex   2.455500e+01 -5.000000e+00 4.665063e+00
      vertex   2.497358e+01 -5.000000e+00 4.042572e+00
      vertex   2.550827e+01 -5.000000e+00 4.300066e+00
    endloop
  endfacet
  facet normal 4.540581e-16 -1.000000e+00 -1.247514e-15
    outer loop
      vertex   2.455500e+01 -5.000000e+00 4.665063e+00
      vertex   2.550827e+01 -5.000000e+00 4.300066e+00
      vertex   2.537088e+01 -5.000000e+00 4.962019e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -1.341754e-15
    outer loop
      vertex   2.537088e+01 -5.000000e+00 4.962019e+00
      vertex   2.550827e+01 -5.000000e+00 4.300066e+00
      vertex   2.610174e+01 -5.000000e+00 4.300066e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -1.341754e-15
    outer loop
      vertex   2.537088e+01 -5.000000e+00 4.962019e+00
      vertex   2.610174e+01 -5.000000e+00 4.300066e+00
      vertex   2.623912e+01 -5.000000e+00 4.962019e+00
    endloop
  endfacet
  facet normal -4.540581e-16 -1.000000e+00 -1.247514e-15
    outer loop
      vertex   2.623912e+01 -5.000000e+00 4.962019e+00
      vertex   2.610174e+01 -5.000000e+00 4.300066e+00
      vertex   2.705500e+01 -5.000000e+00 4.665063e+00
    endloop
  endfacet
  facet normal -5.190406e-16 -1.000000e+00 -1.077799e-15
    outer loop
      vertex   2.705500e+01 -5.000000e+00 4.665063e+00
      vertex   2.610174e+01 -5.000000e+00 4.300066e+00
      vertex   2.663643e+01 -5.000000e+00 4.042572e+00
    endloop
  endfacet
  facet normal 8.831541e-17 -1.000000e+00 -1.486198e-15
    outer loop
      vertex   2.705500e+01 -5.000000e+00 4.665063e+00
      vertex   2.663643e+01 -5.000000e+00 4.042572e+00
      vertex   2.772012e+01 -5.000000e+00 4.106969e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.772012e+01 -5.000000e+00 4.106969e+00
      vertex   2.663643e+01 -5.000000e+00 4.042572e+00
      vertex   2.700645e+01 -5.000000e+00 3.578584e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   2.772012e+01 -5.000000e+00 4.106969e+00
      vertex   2.700645e+01 -5.000000e+00 3.578584e+00
      vertex   2.815424e+01 -5.000000e+00 3.355050e+00
    endloop
  endfacet
  facet normal -3.128681e-16 -1.000000e+00 -1.606500e-15
    outer loop
      vertex   2.815424e+01 -5.000000e+00 3.355050e+00
      vertex   2.700645e+01 -5.000000e+00 3.578584e+00
      vertex   2.713850e+01 -5.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal -4.816419e-16 -1.000000e+00 -1.123670e-15
    outer loop
      vertex   2.815424e+01 -5.000000e+00 3.355050e+00
      vertex   2.713850e+01 -5.000000e+00 3.000000e+00
      vertex   2.830500e+01 -5.000000e+00 2.500000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.830500e+01 -5.000000e+00 2.500000e+00
      vertex   2.713850e+01 -5.000000e+00 3.000000e+00
      vertex   2.700645e+01 -5.000000e+00 2.421416e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.830500e+01 -5.000000e+00 2.500000e+00
      vertex   2.700645e+01 -5.000000e+00 2.421416e+00
      vertex   2.815424e+01 -5.000000e+00 1.644949e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.815424e+01 -5.000000e+00 1.644949e+00
      vertex   2.700645e+01 -5.000000e+00 2.421416e+00
      vertex   2.663643e+01 -5.000000e+00 1.957427e+00
    endloop
  endfacet
  facet normal -2.173480e-16 -1.000000e+00 -1.055730e-15
    outer loop
      vertex   2.815424e+01 -5.000000e+00 1.644949e+00
      vertex   2.663643e+01 -5.000000e+00 1.957427e+00
      vertex   2.772012e+01 -5.000000e+00 8.930306e-01
    endloop
  endfacet
  facet normal -3.425553e-16 -1.000000e+00 -1.183207e-15
    outer loop
      vertex   2.772012e+01 -5.000000e+00 8.930306e-01
      vertex   2.663643e+01 -5.000000e+00 1.957427e+00
      vertex   2.705500e+01 -5.000000e+00 3.349361e-01
    endloop
  endfacet
  facet normal -1.008558e-15 -1.000000e+00 -1.355024e-15
    outer loop
      vertex   2.705500e+01 -5.000000e+00 3.349361e-01
      vertex   2.663643e+01 -5.000000e+00 1.957427e+00
      vertex   2.610174e+01 -5.000000e+00 1.699933e+00
    endloop
  endfacet
  facet normal 1.888308e-16 -1.000000e+00 -5.188084e-16
    outer loop
      vertex   2.705500e+01 -5.000000e+00 3.349361e-01
      vertex   2.610174e+01 -5.000000e+00 1.699933e+00
      vertex   2.623912e+01 -5.000000e+00 3.798024e-02
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 -5.344185e-16
    outer loop
      vertex   2.623912e+01 -5.000000e+00 3.798024e-02
      vertex   2.610174e+01 -5.000000e+00 1.699933e+00
      vertex   2.537088e+01 -5.000000e+00 3.798024e-02
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -5.344185e-16
    outer loop
      vertex   2.537088e+01 -5.000000e+00 3.798024e-02
      vertex   2.610174e+01 -5.000000e+00 1.699933e+00
      vertex   2.550827e+01 -5.000000e+00 1.699933e+00
    endloop
  endfacet
  facet normal -1.888308e-16 -1.000000e+00 -5.188084e-16
    outer loop
      vertex   2.537088e+01 -5.000000e+00 3.798024e-02
      vertex   2.550827e+01 -5.000000e+00 1.699933e+00
      vertex   2.455500e+01 -5.000000e+00 3.349361e-01
    endloop
  endfacet
  facet normal 1.008558e-15 -1.000000e+00 -1.355024e-15
    outer loop
      vertex   2.455500e+01 -5.000000e+00 3.349361e-01
      vertex   2.550827e+01 -5.000000e+00 1.699933e+00
      vertex   2.497358e+01 -5.000000e+00 1.957427e+00
    endloop
  endfacet
  facet normal 9.789418e-01 -7.831972e-07 -2.041394e-01
    outer loop
      vertex   2.334903e+01 2.214745e+00 3.010667e+00
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
      vertex   2.334962e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal 9.333690e-01 -1.099662e-04 -3.589184e-01
    outer loop
      vertex   2.334962e+01 2.214745e+00 3.013500e+00
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
      vertex   2.364258e+01 1.500000e+01 3.771411e+00
    endloop
  endfacet
  facet normal 9.327465e-01 -1.331513e-16 -3.605328e-01
    outer loop
      vertex   2.334962e+01 2.214745e+00 3.013500e+00
      vertex   2.364258e+01 1.500000e+01 3.771411e+00
      vertex   2.334962e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal 9.298686e-01 3.210239e-04 -3.678917e-01
    outer loop
      vertex   2.334962e+01 -5.000000e+00 3.013500e+00
      vertex   2.364258e+01 1.500000e+01 3.771411e+00
      vertex   2.366294e+01 -5.000000e+00 3.805425e+00
    endloop
  endfacet
  facet normal 7.674611e-01 -3.089471e-04 -6.410954e-01
    outer loop
      vertex   2.366294e+01 -5.000000e+00 3.805425e+00
      vertex   2.364258e+01 1.500000e+01 3.771411e+00
      vertex   2.416900e+01 1.500000e+01 4.401592e+00
    endloop
  endfacet
  facet normal 7.531890e-01 5.502296e-04 -6.578039e-01
    outer loop
      vertex   2.366294e+01 -5.000000e+00 3.805425e+00
      vertex   2.416900e+01 1.500000e+01 4.401592e+00
      vertex   2.422316e+01 -5.000000e+00 4.446880e+00
    endloop
  endfacet
  facet normal 5.193203e-01 -5.287000e-04 -8.545795e-01
    outer loop
      vertex   2.422316e+01 -5.000000e+00 4.446880e+00
      vertex   2.416900e+01 1.500000e+01 4.401592e+00
      vertex   2.487071e+01 1.500000e+01 4.828019e+00
    endloop
  endfacet
  facet normal 4.896933e-01 7.605213e-04 -8.718944e-01
    outer loop
      vertex   2.422316e+01 -5.000000e+00 4.446880e+00
      vertex   2.487071e+01 1.500000e+01 4.828019e+00
      vertex   2.496571e+01 -5.000000e+00 4.863929e+00
    endloop
  endfacet
  facet normal 2.155347e-01 -7.295226e-04 -9.764959e-01
    outer loop
      vertex   2.496571e+01 -5.000000e+00 4.863929e+00
      vertex   2.487071e+01 1.500000e+01 4.828019e+00
      vertex   2.567254e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal 1.957234e-01 0.000000e+00 -9.806591e-01
    outer loop
      vertex   2.496571e+01 -5.000000e+00 4.863929e+00
      vertex   2.567254e+01 1.500000e+01 5.005000e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 9.791766e-01 -2.610658e-07 -2.030101e-01
    outer loop
      vertex   2.334903e+01 2.214745e+00 3.010667e+00
      vertex   2.334845e+01 2.214745e+00 3.007834e+00
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal 9.794102e-01 -4.207328e-17 -2.018804e-01
    outer loop
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
      vertex   2.334845e+01 2.214745e+00 3.007834e+00
      vertex   2.334786e+01 2.214745e+00 3.005000e+00
    endloop
  endfacet
  facet normal 1.697499e-01 6.345655e-03 -9.854667e-01
    outer loop
      vertex   2.496571e+01 -5.000000e+00 4.863929e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
      vertex   2.580500e+01 -5.000000e+00 5.008500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.166666e-03 -9.999993e-01
    outer loop
      vertex   2.580500e+01 -5.000000e+00 5.008500e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
      vertex   2.593747e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal -1.697499e-01 6.345655e-03 -9.854667e-01
    outer loop
      vertex   2.580500e+01 -5.000000e+00 5.008500e+00
      vertex   2.593747e+01 -2.000000e+00 5.005000e+00
      vertex   2.664430e+01 -5.000000e+00 4.863929e+00
    endloop
  endfacet
  facet normal -1.957234e-01 0.000000e+00 -9.806591e-01
    outer loop
      vertex   2.664430e+01 -5.000000e+00 4.863929e+00
      vertex   2.593747e+01 -2.000000e+00 5.005000e+00
      vertex   2.593747e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal -2.155347e-01 -7.295226e-04 -9.764959e-01
    outer loop
      vertex   2.664430e+01 -5.000000e+00 4.863929e+00
      vertex   2.593747e+01 1.500000e+01 5.005000e+00
      vertex   2.673929e+01 1.500000e+01 4.828019e+00
    endloop
  endfacet
  facet normal -4.896933e-01 7.605213e-04 -8.718944e-01
    outer loop
      vertex   2.664430e+01 -5.000000e+00 4.863929e+00
      vertex   2.673929e+01 1.500000e+01 4.828019e+00
      vertex   2.738685e+01 -5.000000e+00 4.446880e+00
    endloop
  endfacet
  facet normal -5.193203e-01 -5.287000e-04 -8.545795e-01
    outer loop
      vertex   2.738685e+01 -5.000000e+00 4.446880e+00
      vertex   2.673929e+01 1.500000e+01 4.828019e+00
      vertex   2.744101e+01 1.500000e+01 4.401592e+00
    endloop
  endfacet
  facet normal -7.531890e-01 5.502296e-04 -6.578039e-01
    outer loop
      vertex   2.738685e+01 -5.000000e+00 4.446880e+00
      vertex   2.744101e+01 1.500000e+01 4.401592e+00
      vertex   2.794707e+01 -5.000000e+00 3.805425e+00
    endloop
  endfacet
  facet normal -7.674611e-01 -3.089471e-04 -6.410954e-01
    outer loop
      vertex   2.794707e+01 -5.000000e+00 3.805425e+00
      vertex   2.744101e+01 1.500000e+01 4.401592e+00
      vertex   2.796743e+01 1.500000e+01 3.771411e+00
    endloop
  endfacet
  facet normal -9.298686e-01 3.210239e-04 -3.678917e-01
    outer loop
      vertex   2.794707e+01 -5.000000e+00 3.805425e+00
      vertex   2.796743e+01 1.500000e+01 3.771411e+00
      vertex   2.826038e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal -9.327465e-01 -1.331513e-16 -3.605328e-01
    outer loop
      vertex   2.826038e+01 -5.000000e+00 3.013500e+00
      vertex   2.796743e+01 1.500000e+01 3.771411e+00
      vertex   2.826038e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal -9.333690e-01 -1.099662e-04 -3.589184e-01
    outer loop
      vertex   2.826038e+01 2.214745e+00 3.013500e+00
      vertex   2.796743e+01 1.500000e+01 3.771411e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal -9.789418e-01 -7.831972e-07 -2.041394e-01
    outer loop
      vertex   2.826038e+01 2.214745e+00 3.013500e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
      vertex   2.826097e+01 2.214745e+00 3.010667e+00
    endloop
  endfacet
  facet normal -9.791766e-01 -2.610658e-07 -2.030101e-01
    outer loop
      vertex   2.826097e+01 2.214745e+00 3.010667e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
      vertex   2.826156e+01 2.214745e+00 3.007834e+00
    endloop
  endfacet
  facet normal -9.794102e-01 -4.207659e-17 -2.018804e-01
    outer loop
      vertex   2.826156e+01 2.214745e+00 3.007834e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
      vertex   2.826215e+01 2.214745e+00 3.005000e+00
    endloop
  endfacet
  facet normal 8.222398e-17 8.000000e-01 6.000000e-01
    outer loop
      vertex   3.241863e+01 -2.000000e+00 5.005000e+00
      vertex   2.593747e+01 -2.000000e+00 5.005000e+00
      vertex   3.241863e+01 -3.500000e+00 7.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 8.000000e-01 6.000000e-01
    outer loop
      vertex   3.241863e+01 -3.500000e+00 7.005000e+00
      vertex   2.593747e+01 -2.000000e+00 5.005000e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 8.000000e-01 6.000000e-01
    outer loop
      vertex   3.241863e+01 -3.500000e+00 7.005000e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
      vertex   1.919137e+01 -3.500000e+00 7.005000e+00
    endloop
  endfacet
  facet normal -8.222398e-17 8.000000e-01 6.000000e-01
    outer loop
      vertex   1.919137e+01 -3.500000e+00 7.005000e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
      vertex   1.919137e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal -1.370400e-16 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.919137e+01 -2.000000e+00 5.005000e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 8.867291e-17 2.089832e-16 1.000000e+00
    outer loop
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
      vertex   2.567254e+01 -2.000000e+00 5.005000e+00
      vertex   2.567254e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 2.664535e-16 1.000000e+00
    outer loop
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
      vertex   2.567254e+01 1.500000e+01 5.005000e+00
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 2.664535e-16 1.000000e+00
    outer loop
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
      vertex   2.593747e+01 1.500000e+01 5.005000e+00
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal -8.867291e-17 2.089832e-16 1.000000e+00
    outer loop
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
      vertex   2.593747e+01 1.500000e+01 5.005000e+00
      vertex   2.593747e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 1.370400e-16 0.000000e+00 1.000000e+00
    outer loop
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
      vertex   2.593747e+01 -2.000000e+00 5.005000e+00
      vertex   3.241863e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -3.011101e-14
    outer loop
      vertex   2.567254e+01 1.500000e+01 5.005000e+00
      vertex   2.487071e+01 1.500000e+01 4.828019e+00
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal 8.925558e-16 1.000000e+00 -1.468766e-15
    outer loop
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
      vertex   2.487071e+01 1.500000e+01 4.828019e+00
      vertex   2.416900e+01 1.500000e+01 4.401592e+00
    endloop
  endfacet
  facet normal 9.721606e-16 1.000000e+00 -8.120903e-16
    outer loop
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
      vertex   2.416900e+01 1.500000e+01 4.401592e+00
      vertex   2.364258e+01 1.500000e+01 3.771411e+00
    endloop
  endfacet
  facet normal 2.823338e-15 1.000000e+00 5.867585e-15
    outer loop
      vertex   2.364258e+01 1.500000e+01 3.771411e+00
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
      vertex   1.768188e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
      vertex   1.768188e+01 1.500000e+01 3.005000e+00
      vertex   1.803694e+01 1.500000e+01 6.217180e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.803694e+01 1.500000e+01 6.217180e+00
      vertex   1.768188e+01 1.500000e+01 3.005000e+00
      vertex   1.660188e+01 1.500000e+01 4.061059e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.803694e+01 1.500000e+01 6.217180e+00
      vertex   1.660188e+01 1.500000e+01 4.061059e+00
      vertex   1.750746e+01 1.500000e+01 6.928600e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.750746e+01 1.500000e+01 6.928600e+00
      vertex   1.660188e+01 1.500000e+01 4.061059e+00
      vertex   1.577148e+01 1.500000e+01 5.054570e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.750746e+01 1.500000e+01 6.928600e+00
      vertex   1.577148e+01 1.500000e+01 5.054570e+00
      vertex   1.691414e+01 1.500000e+01 7.950840e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.691414e+01 1.500000e+01 7.950840e+00
      vertex   1.577148e+01 1.500000e+01 5.054570e+00
      vertex   1.469178e+01 1.500000e+01 6.796220e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.691414e+01 1.500000e+01 7.950840e+00
      vertex   1.469178e+01 1.500000e+01 6.796220e+00
      vertex   1.427893e+01 1.500000e+01 7.675620e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.691414e+01 1.500000e+01 7.950840e+00
      vertex   1.427893e+01 1.500000e+01 7.675620e+00
      vertex   1.611363e+01 1.500000e+01 1.006593e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.611363e+01 1.500000e+01 1.006593e+01
      vertex   1.427893e+01 1.500000e+01 7.675620e+00
      vertex   1.355566e+01 1.500000e+01 1.004709e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.611363e+01 1.500000e+01 1.006593e+01
      vertex   1.355566e+01 1.500000e+01 1.004709e+01
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
      vertex   1.355566e+01 1.500000e+01 1.004709e+01
      vertex   1.338927e+01 1.500000e+01 1.111341e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
      vertex   1.338927e+01 1.500000e+01 1.111341e+01
      vertex   1.332247e+01 1.500000e+01 1.278364e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
      vertex   1.332247e+01 1.500000e+01 1.278364e+01
      vertex   1.584692e+01 1.500000e+01 1.329231e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.584692e+01 1.500000e+01 1.329231e+01
      vertex   1.332247e+01 1.500000e+01 1.278364e+01
      vertex   1.348100e+01 1.500000e+01 1.455591e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.584692e+01 1.500000e+01 1.329231e+01
      vertex   1.348100e+01 1.500000e+01 1.455591e+01
      vertex   1.602499e+01 1.500000e+01 1.455995e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.602499e+01 1.500000e+01 1.455995e+01
      vertex   1.348100e+01 1.500000e+01 1.455591e+01
      vertex   1.384628e+01 1.500000e+01 1.613265e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.602499e+01 1.500000e+01 1.455995e+01
      vertex   1.384628e+01 1.500000e+01 1.613265e+01
      vertex   1.643872e+01 1.500000e+01 1.597755e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.643872e+01 1.500000e+01 1.597755e+01
      vertex   1.384628e+01 1.500000e+01 1.613265e+01
      vertex   1.444780e+01 1.500000e+01 1.769260e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.643872e+01 1.500000e+01 1.597755e+01
      vertex   1.444780e+01 1.500000e+01 1.769260e+01
      vertex   1.695559e+01 1.500000e+01 1.714862e+01
    endloop
  endfacet
  facet normal -2.342126e-16 1.000000e+00 -1.079737e-15
    outer loop
      vertex   1.695559e+01 1.500000e+01 1.714862e+01
      vertex   1.444780e+01 1.500000e+01 1.769260e+01
      vertex   1.523546e+01 1.500000e+01 1.916692e+01
    endloop
  endfacet
  facet normal 4.795574e-16 1.000000e+00 -4.714144e-16
    outer loop
      vertex   1.695559e+01 1.500000e+01 1.714862e+01
      vertex   1.523546e+01 1.500000e+01 1.916692e+01
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
      vertex   1.523546e+01 1.500000e+01 1.916692e+01
      vertex   1.573813e+01 1.500000e+01 1.987798e+01
    endloop
  endfacet
  facet normal -3.723719e-16 1.000000e+00 -7.387344e-16
    outer loop
      vertex   1.695559e+01 1.500000e+01 1.714862e+01
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
      vertex   1.823186e+01 1.500000e+01 1.993134e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
      vertex   1.823186e+01 1.500000e+01 1.993134e+01
      vertex   1.837111e+01 1.500000e+01 1.981328e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.837111e+01 1.500000e+01 1.981328e+01
      vertex   1.847196e+01 1.500000e+01 1.942827e+01
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.392812e+01 1.500000e+01 3.005000e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal -2.823338e-15 1.000000e+00 5.867585e-15
    outer loop
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
      vertex   2.796743e+01 1.500000e+01 3.771411e+00
    endloop
  endfacet
  facet normal -9.721606e-16 1.000000e+00 -8.120903e-16
    outer loop
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
      vertex   2.796743e+01 1.500000e+01 3.771411e+00
      vertex   2.744101e+01 1.500000e+01 4.401592e+00
    endloop
  endfacet
  facet normal -8.925558e-16 1.000000e+00 -1.468766e-15
    outer loop
      vertex   2.744101e+01 1.500000e+01 4.401592e+00
      vertex   2.673929e+01 1.500000e+01 4.828019e+00
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -3.011101e-14
    outer loop
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
      vertex   2.673929e+01 1.500000e+01 4.828019e+00
      vertex   2.593747e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.392812e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.392812e+01 1.500000e+01 3.005000e+00
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.715733e+01 1.500000e+01 7.280180e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.715733e+01 1.500000e+01 7.280180e+00
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.517856e+01 1.500000e+01 9.029750e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.715733e+01 1.500000e+01 7.280180e+00
      vertex   3.517856e+01 1.500000e+01 9.029750e+00
      vertex   3.759880e+01 1.500000e+01 8.382330e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.759880e+01 1.500000e+01 8.382330e+00
      vertex   3.517856e+01 1.500000e+01 9.029750e+00
      vertex   3.560813e+01 1.500000e+01 1.054590e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.759880e+01 1.500000e+01 8.382330e+00
      vertex   3.560813e+01 1.500000e+01 1.054590e+01
      vertex   3.796211e+01 1.500000e+01 9.625240e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.796211e+01 1.500000e+01 9.625240e+00
      vertex   3.560813e+01 1.500000e+01 1.054590e+01
      vertex   3.815228e+01 1.500000e+01 1.057795e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.815228e+01 1.500000e+01 1.057795e+01
      vertex   3.560813e+01 1.500000e+01 1.054590e+01
      vertex   3.576308e+01 1.500000e+01 1.171769e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.815228e+01 1.500000e+01 1.057795e+01
      vertex   3.576308e+01 1.500000e+01 1.171769e+01
      vertex   3.830000e+01 1.500000e+01 1.229805e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.830000e+01 1.500000e+01 1.229805e+01
      vertex   3.576308e+01 1.500000e+01 1.171769e+01
      vertex   3.577472e+01 1.500000e+01 1.319439e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.830000e+01 1.500000e+01 1.229805e+01
      vertex   3.577472e+01 1.500000e+01 1.319439e+01
      vertex   3.814949e+01 1.500000e+01 1.444987e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.814949e+01 1.500000e+01 1.444987e+01
      vertex   3.577472e+01 1.500000e+01 1.319439e+01
      vertex   3.771808e+01 1.500000e+01 1.623173e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.771808e+01 1.500000e+01 1.623173e+01
      vertex   3.577472e+01 1.500000e+01 1.319439e+01
      vertex   3.527403e+01 1.500000e+01 1.570032e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.771808e+01 1.500000e+01 1.623173e+01
      vertex   3.527403e+01 1.500000e+01 1.570032e+01
      vertex   3.697619e+01 1.500000e+01 1.808199e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.697619e+01 1.500000e+01 1.808199e+01
      vertex   3.527403e+01 1.500000e+01 1.570032e+01
      vertex   3.495847e+01 1.500000e+01 1.652930e+01
    endloop
  endfacet
  facet normal 8.121085e-16 1.000000e+00 -1.055335e-15
    outer loop
      vertex   3.697619e+01 1.500000e+01 1.808199e+01
      vertex   3.495847e+01 1.500000e+01 1.652930e+01
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
    endloop
  endfacet
  facet normal -6.438007e-16 1.000000e+00 -3.296971e-16
    outer loop
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
      vertex   3.495847e+01 1.500000e+01 1.652930e+01
      vertex   3.450937e+01 1.500000e+01 1.740626e+01
    endloop
  endfacet
  facet normal -4.772982e-16 1.000000e+00 -4.922911e-16
    outer loop
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
      vertex   3.450937e+01 1.500000e+01 1.740626e+01
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
    endloop
  endfacet
  facet normal -6.603672e-16 1.000000e+00 -4.189349e-16
    outer loop
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
      vertex   3.450937e+01 1.500000e+01 1.740626e+01
      vertex   3.387621e+01 1.500000e+01 1.840431e+01
    endloop
  endfacet
  facet normal 2.308642e-17 1.000000e+00 -1.133895e-15
    outer loop
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
      vertex   3.387621e+01 1.500000e+01 1.840431e+01
      vertex   3.343137e+01 1.500000e+01 1.996185e+01
    endloop
  endfacet
  facet normal 2.256711e-15 1.000000e+00 -4.959627e-16
    outer loop
      vertex   3.343137e+01 1.500000e+01 1.996185e+01
      vertex   3.387621e+01 1.500000e+01 1.840431e+01
      vertex   3.325771e+01 1.500000e+01 1.917167e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.343137e+01 1.500000e+01 1.996185e+01
      vertex   3.325771e+01 1.500000e+01 1.917167e+01
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
      vertex   3.325771e+01 1.500000e+01 1.917167e+01
      vertex   3.313252e+01 1.500000e+01 1.955088e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -3.693180e-16 -1.000000e+00
    outer loop
      vertex   2.334962e+01 -5.000000e+00 3.013500e+00
      vertex   1.919137e+01 -5.000000e+00 3.013500e+00
      vertex   2.334962e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -3.693180e-16 -1.000000e+00
    outer loop
      vertex   2.334962e+01 2.214745e+00 3.013500e+00
      vertex   1.919137e+01 -5.000000e+00 3.013500e+00
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.776357e-16 -1.000000e+00
    outer loop
      vertex   1.768188e+01 1.500000e+01 3.005000e+00
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
      vertex   1.919137e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal 7.403175e-17 -2.084069e-16 -1.000000e+00
    outer loop
      vertex   1.919137e+01 5.000000e+00 3.005000e+00
      vertex   2.334786e+01 1.500000e+01 3.005000e+00
      vertex   2.334786e+01 2.214745e+00 3.005000e+00
    endloop
  endfacet
  facet normal 1.068424e-16 -1.594429e-16 -1.000000e+00
    outer loop
      vertex   1.919137e+01 5.000000e+00 3.005000e+00
      vertex   2.334786e+01 2.214745e+00 3.005000e+00
      vertex   1.919137e+01 2.214745e+00 3.005000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.776357e-16 -1.000000e+00
    outer loop
      vertex   1.919137e+01 5.000000e+00 3.005000e+00
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
      vertex   1.768188e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal 1.068424e-16 -1.000000e+00 -4.701218e-13
    outer loop
      vertex   2.334786e+01 2.214745e+00 3.005000e+00
      vertex   2.334845e+01 2.214745e+00 3.007834e+00
      vertex   1.919137e+01 2.214745e+00 3.005000e+00
    endloop
  endfacet
  facet normal -1.780372e-16 -1.000000e+00 -5.224579e-14
    outer loop
      vertex   1.919137e+01 2.214745e+00 3.005000e+00
      vertex   2.334845e+01 2.214745e+00 3.007834e+00
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal -1.067972e-16 -1.000000e+00 2.214198e-17
    outer loop
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
      vertex   2.334845e+01 2.214745e+00 3.007834e+00
      vertex   2.334903e+01 2.214745e+00 3.010667e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 1.567749e-13
    outer loop
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
      vertex   2.334903e+01 2.214745e+00 3.010667e+00
      vertex   2.334962e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 1.567749e-13
    outer loop
      vertex   2.826038e+01 2.214745e+00 3.013500e+00
      vertex   2.826097e+01 2.214745e+00 3.010667e+00
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal 1.067972e-16 -1.000000e+00 2.214198e-17
    outer loop
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
      vertex   2.826097e+01 2.214745e+00 3.010667e+00
      vertex   2.826156e+01 2.214745e+00 3.007834e+00
    endloop
  endfacet
  facet normal 1.780372e-16 -1.000000e+00 -5.224579e-14
    outer loop
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
      vertex   2.826156e+01 2.214745e+00 3.007834e+00
      vertex   3.241863e+01 2.214745e+00 3.005000e+00
    endloop
  endfacet
  facet normal -1.068424e-16 -1.000000e+00 -4.701218e-13
    outer loop
      vertex   3.241863e+01 2.214745e+00 3.005000e+00
      vertex   2.826156e+01 2.214745e+00 3.007834e+00
      vertex   2.826215e+01 2.214745e+00 3.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -3.693180e-16 -1.000000e+00
    outer loop
      vertex   2.826038e+01 2.214745e+00 3.013500e+00
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
      vertex   2.826038e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -3.693180e-16 -1.000000e+00
    outer loop
      vertex   2.826038e+01 -5.000000e+00 3.013500e+00
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
      vertex   3.241863e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal -1.068424e-16 -1.594429e-16 -1.000000e+00
    outer loop
      vertex   3.241863e+01 2.214745e+00 3.005000e+00
      vertex   2.826215e+01 2.214745e+00 3.005000e+00
      vertex   3.241863e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal -7.403175e-17 -2.084069e-16 -1.000000e+00
    outer loop
      vertex   3.241863e+01 5.000000e+00 3.005000e+00
      vertex   2.826215e+01 2.214745e+00 3.005000e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.776357e-16 -1.000000e+00
    outer loop
      vertex   3.241863e+01 5.000000e+00 3.005000e+00
      vertex   2.826215e+01 1.500000e+01 3.005000e+00
      vertex   3.392812e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.776357e-16 -1.000000e+00
    outer loop
      vertex   3.392812e+01 1.500000e+01 3.005000e+00
      vertex   3.392812e+01 5.000000e+00 3.005000e+00
      vertex   3.241863e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 2.214745e+00 3.005000e+00
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
      vertex   1.919137e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 5.000000e+00 3.005000e+00
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
      vertex   1.919137e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   1.919137e+01 -2.000000e+00 5.005000e+00
      vertex   1.919137e+01 2.214745e+00 3.013500e+00
      vertex   1.919137e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 -2.000000e+00 5.005000e+00
      vertex   1.919137e+01 -5.000000e+00 3.013500e+00
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
      vertex   1.919137e+01 -3.500000e+00 7.005000e+00
      vertex   1.919137e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal -8.759766e-01 -4.408605e-03 4.823335e-01
    outer loop
      vertex   1.523546e+01 1.500000e+01 1.916692e+01
      vertex   1.428591e+01 5.000000e+00 1.735102e+01
      vertex   1.554305e+01 5.000000e+00 1.963414e+01
    endloop
  endfacet
  facet normal -8.820144e-01 -1.816979e-03 4.712189e-01
    outer loop
      vertex   1.444780e+01 1.500000e+01 1.769260e+01
      vertex   1.428591e+01 5.000000e+00 1.735102e+01
      vertex   1.523546e+01 1.500000e+01 1.916692e+01
    endloop
  endfacet
  facet normal -9.373283e-01 3.272654e-03 3.484321e-01
    outer loop
      vertex   1.444780e+01 1.500000e+01 1.769260e+01
      vertex   1.394741e+01 5.000000e+00 1.644041e+01
      vertex   1.428591e+01 5.000000e+00 1.735102e+01
    endloop
  endfacet
  facet normal -9.317649e-01 1.162514e-03 3.630605e-01
    outer loop
      vertex   1.390870e+01 1.125000e+01 1.632105e+01
      vertex   1.394741e+01 5.000000e+00 1.644041e+01
      vertex   1.444780e+01 1.500000e+01 1.769260e+01
    endloop
  endfacet
  facet normal -9.622571e-01 -7.626913e-04 2.721409e-01
    outer loop
      vertex   1.390870e+01 1.125000e+01 1.632105e+01
      vertex   1.362430e+01 5.000000e+00 1.529793e+01
      vertex   1.394741e+01 5.000000e+00 1.644041e+01
    endloop
  endfacet
  facet normal -9.831724e-01 1.493376e-02 1.820688e-01
    outer loop
      vertex   1.344740e+01 5.000000e+00 1.434267e+01
      vertex   1.362430e+01 5.000000e+00 1.529793e+01
      vertex   1.390870e+01 1.125000e+01 1.632105e+01
    endloop
  endfacet
  facet normal -9.977476e-01 -2.011628e-03 6.704924e-02
    outer loop
      vertex   1.332247e+01 1.500000e+01 1.278364e+01
      vertex   1.331000e+01 5.000000e+00 1.229805e+01
      vertex   1.344740e+01 5.000000e+00 1.434267e+01
    endloop
  endfacet
  facet normal -9.991961e-01 3.186529e-03 -3.996184e-02
    outer loop
      vertex   1.338927e+01 1.500000e+01 1.111341e+01
      vertex   1.331000e+01 5.000000e+00 1.229805e+01
      vertex   1.332247e+01 1.500000e+01 1.278364e+01
    endloop
  endfacet
  facet normal -9.960609e-01 -2.604109e-03 -8.863304e-02
    outer loop
      vertex   1.343009e+01 1.125000e+01 1.076485e+01
      vertex   1.331000e+01 5.000000e+00 1.229805e+01
      vertex   1.338927e+01 1.500000e+01 1.111341e+01
    endloop
  endfacet
  facet normal -9.830116e-01 -2.569418e-02 -1.817362e-01
    outer loop
      vertex   1.361047e+01 1.125000e+01 9.789170e+00
      vertex   1.331000e+01 5.000000e+00 1.229805e+01
      vertex   1.343009e+01 1.125000e+01 1.076485e+01
    endloop
  endfacet
  facet normal -9.901862e-01 -8.395653e-03 -1.395023e-01
    outer loop
      vertex   1.361047e+01 1.125000e+01 9.789170e+00
      vertex   1.373067e+01 5.000000e+00 9.312140e+00
      vertex   1.331000e+01 5.000000e+00 1.229805e+01
    endloop
  endfacet
  facet normal -9.555255e-01 4.130077e-03 -2.948796e-01
    outer loop
      vertex   1.427893e+01 1.500000e+01 7.675620e+00
      vertex   1.373067e+01 5.000000e+00 9.312140e+00
      vertex   1.361047e+01 1.125000e+01 9.789170e+00
    endloop
  endfacet
  facet normal -9.490244e-01 4.477806e-04 -3.152023e-01
    outer loop
      vertex   1.427893e+01 1.500000e+01 7.675620e+00
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
      vertex   1.373067e+01 5.000000e+00 9.312140e+00
    endloop
  endfacet
  facet normal -9.052090e-01 -8.283617e-04 -4.249659e-01
    outer loop
      vertex   1.469178e+01 1.500000e+01 6.796220e+00
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
      vertex   1.427893e+01 1.500000e+01 7.675620e+00
    endloop
  endfacet
  facet normal -8.907226e-01 -4.379921e-03 -4.545262e-01
    outer loop
      vertex   1.501004e+01 1.125000e+01 6.208670e+00
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
      vertex   1.469178e+01 1.500000e+01 6.796220e+00
    endloop
  endfacet
  facet normal -8.451664e-01 -2.984257e-02 -5.336695e-01
    outer loop
      vertex   1.548101e+01 1.125000e+01 5.462800e+00
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
      vertex   1.501004e+01 1.125000e+01 6.208670e+00
    endloop
  endfacet
  facet normal -8.645862e-01 -1.436756e-02 -5.022790e-01
    outer loop
      vertex   1.548101e+01 1.125000e+01 5.462800e+00
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
    endloop
  endfacet
  facet normal -8.098741e-01 -1.126529e-03 -5.866027e-01
    outer loop
      vertex   1.577148e+01 1.500000e+01 5.054570e+00
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
      vertex   1.548101e+01 1.125000e+01 5.462800e+00
    endloop
  endfacet
  facet normal -6.991379e-01 -1.270072e-16 -7.149868e-01
    outer loop
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
      vertex   1.660188e+01 1.500000e+01 4.061059e+00
      vertex   1.768188e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal -7.235622e-01 -5.251324e-03 -6.902392e-01
    outer loop
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
      vertex   1.660188e+01 1.500000e+01 4.061059e+00
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal -7.672802e-01 1.119453e-03 -6.413110e-01
    outer loop
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
      vertex   1.577148e+01 1.500000e+01 5.054570e+00
      vertex   1.660188e+01 1.500000e+01 4.061059e+00
    endloop
  endfacet
  facet normal -8.455200e-01 7.372622e-03 -5.338928e-01
    outer loop
      vertex   1.501004e+01 1.125000e+01 6.208670e+00
      vertex   1.577148e+01 1.500000e+01 5.054570e+00
      vertex   1.548101e+01 1.125000e+01 5.462800e+00
    endloop
  endfacet
  facet normal -8.498833e-01 1.042058e-02 -5.268679e-01
    outer loop
      vertex   1.501004e+01 1.125000e+01 6.208670e+00
      vertex   1.469178e+01 1.500000e+01 6.796220e+00
      vertex   1.577148e+01 1.500000e+01 5.054570e+00
    endloop
  endfacet
  facet normal -9.564854e-01 6.083773e-03 -2.917168e-01
    outer loop
      vertex   1.361047e+01 1.125000e+01 9.789170e+00
      vertex   1.355566e+01 1.500000e+01 1.004709e+01
      vertex   1.427893e+01 1.500000e+01 7.675620e+00
    endloop
  endfacet
  facet normal -9.833345e-01 -1.868867e-03 -1.817960e-01
    outer loop
      vertex   1.343009e+01 1.125000e+01 1.076485e+01
      vertex   1.355566e+01 1.500000e+01 1.004709e+01
      vertex   1.361047e+01 1.125000e+01 9.789170e+00
    endloop
  endfacet
  facet normal -9.880371e-01 3.575316e-03 -1.541750e-01
    outer loop
      vertex   1.343009e+01 1.125000e+01 1.076485e+01
      vertex   1.338927e+01 1.500000e+01 1.111341e+01
      vertex   1.355566e+01 1.500000e+01 1.004709e+01
    endloop
  endfacet
  facet normal -9.960222e-01 1.446777e-03 8.909403e-02
    outer loop
      vertex   1.344740e+01 5.000000e+00 1.434267e+01
      vertex   1.348100e+01 1.500000e+01 1.455591e+01
      vertex   1.332247e+01 1.500000e+01 1.278364e+01
    endloop
  endfacet
  facet normal -9.741978e-01 -1.539336e-03 2.256905e-01
    outer loop
      vertex   1.344740e+01 5.000000e+00 1.434267e+01
      vertex   1.384628e+01 1.500000e+01 1.613265e+01
      vertex   1.348100e+01 1.500000e+01 1.455591e+01
    endloop
  endfacet
  facet normal -9.709028e-01 -4.131582e-03 2.394382e-01
    outer loop
      vertex   1.390870e+01 1.125000e+01 1.632105e+01
      vertex   1.384628e+01 1.500000e+01 1.613265e+01
      vertex   1.344740e+01 5.000000e+00 1.434267e+01
    endloop
  endfacet
  facet normal -9.330337e-01 2.544743e-03 3.597800e-01
    outer loop
      vertex   1.390870e+01 1.125000e+01 1.632105e+01
      vertex   1.444780e+01 1.500000e+01 1.769260e+01
      vertex   1.384628e+01 1.500000e+01 1.613265e+01
    endloop
  endfacet
  facet normal -8.165631e-01 1.853741e-03 5.772532e-01
    outer loop
      vertex   1.554305e+01 5.000000e+00 1.963414e+01
      vertex   1.573813e+01 1.500000e+01 1.987798e+01
      vertex   1.523546e+01 1.500000e+01 1.916692e+01
    endloop
  endfacet
  facet normal 7.992401e-01 1.046879e-16 6.010119e-01
    outer loop
      vertex   1.757178e+01 1.666670e+00 6.839960e+00
      vertex   1.814542e+01 -5.000000e+00 6.077120e+00
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
    endloop
  endfacet
  facet normal 7.773622e-01 1.310002e-04 -6.290533e-01
    outer loop
      vertex   1.838657e+01 5.000000e+00 1.922256e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
      vertex   1.840698e+01 1.166667e+01 1.924917e+01
    endloop
  endfacet
  facet normal 7.786630e-01 -1.661022e-04 -6.274424e-01
    outer loop
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
      vertex   1.840698e+01 1.166667e+01 1.924917e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 8.479257e-01 -6.131612e-03 -5.300796e-01
    outer loop
      vertex   1.683991e+01 8.333331e+00 1.692121e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
      vertex   1.706441e+01 1.666670e+00 1.735744e+01
    endloop
  endfacet
  facet normal 8.562983e-01 2.759202e-03 -5.164742e-01
    outer loop
      vertex   1.683991e+01 8.333331e+00 1.692121e+01
      vertex   1.695559e+01 1.500000e+01 1.714862e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 9.148520e-01 -2.100837e-03 -4.037839e-01
    outer loop
      vertex   1.643872e+01 1.500000e+01 1.597755e+01
      vertex   1.695559e+01 1.500000e+01 1.714862e+01
      vertex   1.683991e+01 8.333331e+00 1.692121e+01
    endloop
  endfacet
  facet normal 8.868725e-01 -3.662500e-04 -4.620141e-01
    outer loop
      vertex   1.687372e+01 -5.000000e+00 1.699668e+01
      vertex   1.683991e+01 8.333331e+00 1.692121e+01
      vertex   1.706441e+01 1.666670e+00 1.735744e+01
    endloop
  endfacet
  facet normal 9.205989e-01 1.240055e-04 -3.905094e-01
    outer loop
      vertex   1.643872e+01 1.500000e+01 1.597755e+01
      vertex   1.683991e+01 8.333331e+00 1.692121e+01
      vertex   1.687372e+01 -5.000000e+00 1.699668e+01
    endloop
  endfacet
  facet normal 9.333176e-01 2.003885e-03 -3.590463e-01
    outer loop
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
      vertex   1.643872e+01 1.500000e+01 1.597755e+01
      vertex   1.687372e+01 -5.000000e+00 1.699668e+01
    endloop
  endfacet
  facet normal 9.599507e-01 -1.588396e-03 -2.801644e-01
    outer loop
      vertex   1.602499e+01 1.500000e+01 1.455995e+01
      vertex   1.643872e+01 1.500000e+01 1.597755e+01
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
    endloop
  endfacet
  facet normal 9.630030e-01 -1.247192e-03 -2.694878e-01
    outer loop
      vertex   1.602499e+01 1.500000e+01 1.455995e+01
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
      vertex   1.607558e+01 1.666670e+00 1.480244e+01
    endloop
  endfacet
  facet normal 9.902689e-01 4.094784e-03 -1.391067e-01
    outer loop
      vertex   1.584692e+01 1.500000e+01 1.329231e+01
      vertex   1.602499e+01 1.500000e+01 1.455995e+01
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
    endloop
  endfacet
  facet normal 9.999863e-01 -2.133498e-03 4.774445e-03
    outer loop
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
      vertex   1.584692e+01 1.500000e+01 1.329231e+01
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
    endloop
  endfacet
  facet normal 9.872017e-01 -1.143769e-03 1.594725e-01
    outer loop
      vertex   1.611363e+01 1.500000e+01 1.006593e+01
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
      vertex   1.602479e+01 -5.000000e+00 1.047244e+01
    endloop
  endfacet
  facet normal 9.670342e-01 8.802428e-04 2.546450e-01
    outer loop
      vertex   1.632589e+01 -5.000000e+00 9.328990e+00
      vertex   1.611363e+01 1.500000e+01 1.006593e+01
      vertex   1.602479e+01 -5.000000e+00 1.047244e+01
    endloop
  endfacet
  facet normal 9.201134e-01 -4.665006e-03 3.916243e-01
    outer loop
      vertex   1.710114e+01 1.666670e+00 7.586970e+00
      vertex   1.611363e+01 1.500000e+01 1.006593e+01
      vertex   1.632589e+01 -5.000000e+00 9.328990e+00
    endloop
  endfacet
  facet normal 9.352505e-01 3.456919e-03 3.539697e-01
    outer loop
      vertex   1.691414e+01 1.500000e+01 7.950840e+00
      vertex   1.611363e+01 1.500000e+01 1.006593e+01
      vertex   1.710114e+01 1.666670e+00 7.586970e+00
    endloop
  endfacet
  facet normal 8.654437e-01 -1.534718e-03 5.010039e-01
    outer loop
      vertex   1.735574e+01 8.333331e+00 7.167590e+00
      vertex   1.691414e+01 1.500000e+01 7.950840e+00
      vertex   1.710114e+01 1.666670e+00 7.586970e+00
    endloop
  endfacet
  facet normal 8.648755e-01 -1.687474e-03 5.019837e-01
    outer loop
      vertex   1.750746e+01 1.500000e+01 6.928600e+00
      vertex   1.691414e+01 1.500000e+01 7.950840e+00
      vertex   1.735574e+01 8.333331e+00 7.167590e+00
    endloop
  endfacet
  facet normal 8.460787e-01 1.221179e-03 5.330566e-01
    outer loop
      vertex   1.757178e+01 1.666670e+00 6.839960e+00
      vertex   1.735574e+01 8.333331e+00 7.167590e+00
      vertex   1.710114e+01 1.666670e+00 7.586970e+00
    endloop
  endfacet
  facet normal 8.022019e-01 3.146639e-03 5.970445e-01
    outer loop
      vertex   1.803694e+01 1.500000e+01 6.217180e+00
      vertex   1.750746e+01 1.500000e+01 6.928600e+00
      vertex   1.735574e+01 8.333331e+00 7.167590e+00
    endloop
  endfacet
  facet normal 8.184444e-01 -1.715141e-03 5.745832e-01
    outer loop
      vertex   1.803694e+01 1.500000e+01 6.217180e+00
      vertex   1.735574e+01 8.333331e+00 7.167590e+00
      vertex   1.757178e+01 1.666670e+00 6.839960e+00
    endloop
  endfacet
  facet normal 8.300302e-01 -5.065598e-03 -5.576954e-01
    outer loop
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
      vertex   1.695559e+01 1.500000e+01 1.714862e+01
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
    endloop
  endfacet
  facet normal 8.399168e-01 0.000000e+00 -5.427151e-01
    outer loop
      vertex   1.780591e+01 -5.000000e+00 1.850500e+01
      vertex   1.706441e+01 1.666670e+00 1.735744e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 8.506444e-01 4.117672e-03 -5.257254e-01
    outer loop
      vertex   1.687372e+01 -5.000000e+00 1.699668e+01
      vertex   1.706441e+01 1.666670e+00 1.735744e+01
      vertex   1.780591e+01 -5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 9.914745e-01 6.586860e-03 -1.301340e-01
    outer loop
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
      vertex   1.607558e+01 1.666670e+00 1.480244e+01
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
    endloop
  endfacet
  facet normal 9.931939e-01 1.650422e-03 -1.164608e-01
    outer loop
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
      vertex   1.602499e+01 1.500000e+01 1.455995e+01
      vertex   1.607558e+01 1.666670e+00 1.480244e+01
    endloop
  endfacet
  facet normal 9.988939e-01 -5.253752e-04 4.701802e-02
    outer loop
      vertex   1.584546e+01 -5.000000e+00 1.164120e+01
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
    endloop
  endfacet
  facet normal 9.884322e-01 -6.660127e-04 1.516619e-01
    outer loop
      vertex   1.584546e+01 -5.000000e+00 1.164120e+01
      vertex   1.602479e+01 -5.000000e+00 1.047244e+01
      vertex   1.585467e+01 1.500000e+01 1.166900e+01
    endloop
  endfacet
  facet normal 9.252620e-01 -8.501544e-03 3.792333e-01
    outer loop
      vertex   1.677432e+01 -5.000000e+00 8.234900e+00
      vertex   1.710114e+01 1.666670e+00 7.586970e+00
      vertex   1.632589e+01 -5.000000e+00 9.328990e+00
    endloop
  endfacet
  facet normal 8.439728e-01 1.074661e-02 5.362783e-01
    outer loop
      vertex   1.814542e+01 -5.000000e+00 6.077120e+00
      vertex   1.710114e+01 1.666670e+00 7.586970e+00
      vertex   1.677432e+01 -5.000000e+00 8.234900e+00
    endloop
  endfacet
  facet normal 8.460204e-01 1.180531e-02 5.330199e-01
    outer loop
      vertex   1.814542e+01 -5.000000e+00 6.077120e+00
      vertex   1.757178e+01 1.666670e+00 6.839960e+00
      vertex   1.710114e+01 1.666670e+00 7.586970e+00
    endloop
  endfacet
  facet normal 7.157887e-01 1.860690e-16 6.983169e-01
    outer loop
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 7.241447e-01 -1.803654e-03 6.896457e-01
    outer loop
      vertex   1.803694e+01 1.500000e+01 6.217180e+00
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
      vertex   1.919137e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal 7.987409e-01 2.376804e-04 6.016751e-01
    outer loop
      vertex   1.803694e+01 1.500000e+01 6.217180e+00
      vertex   1.757178e+01 1.666670e+00 6.839960e+00
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
    endloop
  endfacet
  facet normal -7.157887e-01 1.778947e-16 6.983169e-01
    outer loop
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal -7.524048e-01 -8.082476e-03 6.586514e-01
    outer loop
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.241863e+01 1.500000e+01 5.005000e+00
    endloop
  endfacet
  facet normal -8.231719e-01 1.008602e-16 5.677923e-01
    outer loop
      vertex   3.346458e+01 -5.000000e+00 6.077120e+00
      vertex   3.447097e+01 -5.000000e+00 7.536160e+00
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
    endloop
  endfacet
  facet normal -8.147339e-01 2.605426e-03 5.798292e-01
    outer loop
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
      vertex   3.447097e+01 -5.000000e+00 7.536160e+00
    endloop
  endfacet
  facet normal -8.553458e-01 -1.920863e-05 5.180575e-01
    outer loop
      vertex   3.459323e+01 5.000000e+00 7.738390e+00
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.447097e+01 -5.000000e+00 7.536160e+00
    endloop
  endfacet
  facet normal -8.900537e-01 -6.763389e-03 4.558055e-01
    outer loop
      vertex   3.517856e+01 1.500000e+01 9.029750e+00
      vertex   3.410254e+01 1.500000e+01 6.928600e+00
      vertex   3.459323e+01 5.000000e+00 7.738390e+00
    endloop
  endfacet
  facet normal -9.068930e-01 -1.329435e-03 4.213589e-01
    outer loop
      vertex   3.500425e+01 -5.000000e+00 8.591480e+00
      vertex   3.517856e+01 1.500000e+01 9.029750e+00
      vertex   3.459323e+01 5.000000e+00 7.738390e+00
    endloop
  endfacet
  facet normal -9.457451e-01 1.122765e-03 3.249076e-01
    outer loop
      vertex   3.543673e+01 1.666670e+00 9.827310e+00
      vertex   3.517856e+01 1.500000e+01 9.029750e+00
      vertex   3.500425e+01 -5.000000e+00 8.591480e+00
    endloop
  endfacet
  facet normal -9.621251e-01 -2.323371e-03 2.725985e-01
    outer loop
      vertex   3.560813e+01 1.500000e+01 1.054590e+01
      vertex   3.517856e+01 1.500000e+01 9.029750e+00
      vertex   3.543673e+01 1.666670e+00 9.827310e+00
    endloop
  endfacet
  facet normal -9.732449e-01 1.277638e-04 2.297704e-01
    outer loop
      vertex   3.561711e+01 1.666670e+00 1.059135e+01
      vertex   3.560813e+01 1.500000e+01 1.054590e+01
      vertex   3.543673e+01 1.666670e+00 9.827310e+00
    endloop
  endfacet
  facet normal -9.913701e-01 -2.208113e-04 1.310925e-01
    outer loop
      vertex   3.576308e+01 1.500000e+01 1.171769e+01
      vertex   3.560813e+01 1.500000e+01 1.054590e+01
      vertex   3.561711e+01 1.666670e+00 1.059135e+01
    endloop
  endfacet
  facet normal -9.935041e-01 1.264282e-03 1.137889e-01
    outer loop
      vertex   3.579563e+01 1.666670e+00 1.215003e+01
      vertex   3.576308e+01 1.500000e+01 1.171769e+01
      vertex   3.561711e+01 1.666670e+00 1.059135e+01
    endloop
  endfacet
  facet normal -9.996643e-01 -1.601882e-03 2.586047e-02
    outer loop
      vertex   3.579764e+01 8.333331e+00 1.264066e+01
      vertex   3.576308e+01 1.500000e+01 1.171769e+01
      vertex   3.579563e+01 1.666670e+00 1.215003e+01
    endloop
  endfacet
  facet normal -9.999606e-01 -4.092474e-03 7.881855e-03
    outer loop
      vertex   3.577472e+01 1.500000e+01 1.319439e+01
      vertex   3.576308e+01 1.500000e+01 1.171769e+01
      vertex   3.579764e+01 8.333331e+00 1.264066e+01
    endloop
  endfacet
  facet normal -9.999973e-01 1.308837e-04 2.317080e-03
    outer loop
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
      vertex   3.579764e+01 8.333331e+00 1.264066e+01
      vertex   3.579563e+01 1.666670e+00 1.215003e+01
    endloop
  endfacet
  facet normal -9.979242e-01 -2.913225e-04 -6.439935e-02
    outer loop
      vertex   3.572815e+01 8.333331e+00 1.371746e+01
      vertex   3.579764e+01 8.333331e+00 1.264066e+01
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
    endloop
  endfacet
  facet normal -9.979224e-01 1.918165e-03 -6.439924e-02
    outer loop
      vertex   3.572815e+01 8.333331e+00 1.371746e+01
      vertex   3.577472e+01 1.500000e+01 1.319439e+01
      vertex   3.579764e+01 8.333331e+00 1.264066e+01
    endloop
  endfacet
  facet normal -9.805824e-01 -8.522313e-03 -1.959222e-01
    outer loop
      vertex   3.527403e+01 1.500000e+01 1.570032e+01
      vertex   3.577472e+01 1.500000e+01 1.319439e+01
      vertex   3.572815e+01 8.333331e+00 1.371746e+01
    endloop
  endfacet
  facet normal -9.778483e-01 -4.366190e-03 -2.092695e-01
    outer loop
      vertex   3.548988e+01 1.666670e+00 1.496991e+01
      vertex   3.527403e+01 1.500000e+01 1.570032e+01
      vertex   3.572815e+01 8.333331e+00 1.371746e+01
    endloop
  endfacet
  facet normal -9.522671e-01 -2.252123e-03 -3.052579e-01
    outer loop
      vertex   3.495847e+01 1.500000e+01 1.652930e+01
      vertex   3.548988e+01 1.666670e+00 1.496991e+01
      vertex   3.520419e+01 -5.000000e+00 1.591032e+01
    endloop
  endfacet
  facet normal -9.345694e-01 4.358980e-03 -3.557541e-01
    outer loop
      vertex   3.495847e+01 1.500000e+01 1.652930e+01
      vertex   3.527403e+01 1.500000e+01 1.570032e+01
      vertex   3.548988e+01 1.666670e+00 1.496991e+01
    endloop
  endfacet
  facet normal -9.075469e-01 1.846811e-03 -4.199467e-01
    outer loop
      vertex   3.450103e+01 -5.000000e+00 1.742992e+01
      vertex   3.495847e+01 1.500000e+01 1.652930e+01
      vertex   3.520419e+01 -5.000000e+00 1.591032e+01
    endloop
  endfacet
  facet normal -8.900739e-01 -1.680682e-04 -4.558161e-01
    outer loop
      vertex   3.450937e+01 1.500000e+01 1.740626e+01
      vertex   3.495847e+01 1.500000e+01 1.652930e+01
      vertex   3.450103e+01 -5.000000e+00 1.742992e+01
    endloop
  endfacet
  facet normal -8.403398e-01 -2.908321e-04 -5.420599e-01
    outer loop
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
      vertex   3.450937e+01 1.500000e+01 1.740626e+01
      vertex   3.450103e+01 -5.000000e+00 1.742992e+01
    endloop
  endfacet
  facet normal -8.444127e-01 6.959955e-04 -5.356928e-01
    outer loop
      vertex   3.387621e+01 1.500000e+01 1.840431e+01
      vertex   3.450937e+01 1.500000e+01 1.740626e+01
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -8.925163e-01 1.791109e-03 4.510115e-01
    outer loop
      vertex   3.447097e+01 -5.000000e+00 7.536160e+00
      vertex   3.500425e+01 -5.000000e+00 8.591480e+00
      vertex   3.459323e+01 5.000000e+00 7.738390e+00
    endloop
  endfacet
  facet normal -9.730396e-01 2.053840e-02 2.297219e-01
    outer loop
      vertex   3.500425e+01 -5.000000e+00 8.591480e+00
      vertex   3.561711e+01 1.666670e+00 1.059135e+01
      vertex   3.543673e+01 1.666670e+00 9.827310e+00
    endloop
  endfacet
  facet normal -9.660073e-01 1.132921e-02 2.582662e-01
    outer loop
      vertex   3.571499e+01 -5.000000e+00 1.124990e+01
      vertex   3.561711e+01 1.666670e+00 1.059135e+01
      vertex   3.500425e+01 -5.000000e+00 8.591480e+00
    endloop
  endfacet
  facet normal -9.934994e-01 -3.346307e-03 1.137884e-01
    outer loop
      vertex   3.571499e+01 -5.000000e+00 1.124990e+01
      vertex   3.579563e+01 1.666670e+00 1.215003e+01
      vertex   3.561711e+01 1.666670e+00 1.059135e+01
    endloop
  endfacet
  facet normal -9.984812e-01 4.665590e-03 5.489586e-02
    outer loop
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
      vertex   3.579563e+01 1.666670e+00 1.215003e+01
      vertex   3.571499e+01 -5.000000e+00 1.124990e+01
    endloop
  endfacet
  facet normal -9.896443e-01 5.633587e-03 -1.434307e-01
    outer loop
      vertex   3.544338e+01 -5.000000e+00 1.515862e+01
      vertex   3.572815e+01 8.333331e+00 1.371746e+01
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
    endloop
  endfacet
  facet normal -9.841338e-01 1.842279e-03 -1.774183e-01
    outer loop
      vertex   3.544338e+01 -5.000000e+00 1.515862e+01
      vertex   3.548988e+01 1.666670e+00 1.496991e+01
      vertex   3.572815e+01 8.333331e+00 1.371746e+01
    endloop
  endfacet
  facet normal -9.529193e-01 -1.936377e-03 -3.032177e-01
    outer loop
      vertex   3.520419e+01 -5.000000e+00 1.591032e+01
      vertex   3.548988e+01 1.666670e+00 1.496991e+01
      vertex   3.544338e+01 -5.000000e+00 1.515862e+01
    endloop
  endfacet
  facet normal -7.785810e-01 -3.067280e-06 -6.275442e-01
    outer loop
      vertex   3.322541e+01 7.500000e+00 1.921178e+01
      vertex   3.325771e+01 1.500000e+01 1.917167e+01
      vertex   3.387621e+01 1.500000e+01 1.840431e+01
    endloop
  endfacet
  facet normal -7.750771e-01 -7.724190e-04 -6.318661e-01
    outer loop
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
      vertex   3.322541e+01 7.500000e+00 1.921178e+01
      vertex   3.387621e+01 1.500000e+01 1.840431e+01
    endloop
  endfacet
  facet normal -8.391072e-01 0.000000e+00 -5.439660e-01
    outer loop
      vertex   3.450103e+01 -5.000000e+00 1.742992e+01
      vertex   3.380409e+01 -5.000000e+00 1.850500e+01
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.334962e+01 -5.000000e+00 3.013500e+00
      vertex   2.366294e+01 -5.000000e+00 3.805425e+00
      vertex   1.919137e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal -2.900907e-16 -1.000000e+00 1.637983e-15
    outer loop
      vertex   1.919137e+01 -5.000000e+00 3.013500e+00
      vertex   2.366294e+01 -5.000000e+00 3.805425e+00
      vertex   2.422316e+01 -5.000000e+00 4.446880e+00
    endloop
  endfacet
  facet normal 1.131261e-16 -1.000000e+00 2.225175e-16
    outer loop
      vertex   1.919137e+01 -5.000000e+00 3.013500e+00
      vertex   2.422316e+01 -5.000000e+00 4.446880e+00
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
      vertex   2.422316e+01 -5.000000e+00 4.446880e+00
      vertex   2.496571e+01 -5.000000e+00 4.863929e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
      vertex   2.496571e+01 -5.000000e+00 4.863929e+00
      vertex   2.580500e+01 -5.000000e+00 5.008500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
      vertex   2.580500e+01 -5.000000e+00 5.008500e+00
      vertex   3.241863e+01 -5.000000e+00 7.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 -5.000000e+00 7.005000e+00
      vertex   2.580500e+01 -5.000000e+00 5.008500e+00
      vertex   2.664430e+01 -5.000000e+00 4.863929e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 -5.000000e+00 7.005000e+00
      vertex   2.664430e+01 -5.000000e+00 4.863929e+00
      vertex   2.738685e+01 -5.000000e+00 4.446880e+00
    endloop
  endfacet
  facet normal -1.131261e-16 -1.000000e+00 2.225175e-16
    outer loop
      vertex   3.241863e+01 -5.000000e+00 7.005000e+00
      vertex   2.738685e+01 -5.000000e+00 4.446880e+00
      vertex   3.241863e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal 2.900907e-16 -1.000000e+00 1.637983e-15
    outer loop
      vertex   3.241863e+01 -5.000000e+00 3.013500e+00
      vertex   2.738685e+01 -5.000000e+00 4.446880e+00
      vertex   2.794707e+01 -5.000000e+00 3.805425e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 -5.000000e+00 3.013500e+00
      vertex   2.794707e+01 -5.000000e+00 3.805425e+00
      vertex   2.826038e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.919137e+01 -3.500000e+00 7.005000e+00
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
      vertex   3.241863e+01 -3.500000e+00 7.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   3.241863e+01 -3.500000e+00 7.005000e+00
      vertex   1.919137e+01 -5.000000e+00 7.005000e+00
      vertex   3.241863e+01 -5.000000e+00 7.005000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 -0.000000e+00
    outer loop
      vertex   3.241863e+01 2.214745e+00 3.005000e+00
      vertex   3.241863e+01 5.000000e+00 3.005000e+00
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
      vertex   3.241863e+01 5.000000e+00 3.005000e+00
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
      vertex   3.241863e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 -3.500000e+00 7.005000e+00
      vertex   3.241863e+01 -5.000000e+00 7.005000e+00
      vertex   3.241863e+01 -2.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 -2.000000e+00 5.005000e+00
      vertex   3.241863e+01 -5.000000e+00 7.005000e+00
      vertex   3.241863e+01 -5.000000e+00 3.013500e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   3.241863e+01 -2.000000e+00 5.005000e+00
      vertex   3.241863e+01 -5.000000e+00 3.013500e+00
      vertex   3.241863e+01 2.214745e+00 3.013500e+00
    endloop
  endfacet
  facet normal 9.718620e-01 1.088179e-02 2.352995e-01
    outer loop
      vertex   3.814949e+01 1.500000e+01 1.444987e+01
      vertex   3.771808e+01 1.500000e+01 1.623173e+01
      vertex   3.792668e+01 1.125000e+01 1.554357e+01
    endloop
  endfacet
  facet normal 9.963178e-01 5.473424e-03 -8.556269e-02
    outer loop
      vertex   3.815228e+01 1.500000e+01 1.057795e+01
      vertex   3.830000e+01 1.500000e+01 1.229805e+01
      vertex   3.821394e+01 1.125000e+01 1.105606e+01
    endloop
  endfacet
  facet normal 9.829046e-01 -7.293804e-03 -1.839711e-01
    outer loop
      vertex   3.790993e+01 1.125000e+01 9.431820e+00
      vertex   3.815228e+01 1.500000e+01 1.057795e+01
      vertex   3.821394e+01 1.125000e+01 1.105606e+01
    endloop
  endfacet
  facet normal 9.806482e-01 -3.549151e-03 -1.957460e-01
    outer loop
      vertex   3.790993e+01 1.125000e+01 9.431820e+00
      vertex   3.796211e+01 1.500000e+01 9.625240e+00
      vertex   3.815228e+01 1.500000e+01 1.057795e+01
    endloop
  endfacet
  facet normal 9.598343e-01 1.115327e-03 -2.805652e-01
    outer loop
      vertex   3.790993e+01 1.125000e+01 9.431820e+00
      vertex   3.759880e+01 1.500000e+01 8.382330e+00
      vertex   3.796211e+01 1.500000e+01 9.625240e+00
    endloop
  endfacet
  facet normal 9.282994e-01 1.699527e-04 -3.718336e-01
    outer loop
      vertex   3.719906e+01 5.000000e+00 7.379790e+00
      vertex   3.715733e+01 1.500000e+01 7.280180e+00
      vertex   3.759880e+01 1.500000e+01 8.382330e+00
    endloop
  endfacet
  facet normal 8.895404e-01 -8.386943e-04 -4.568558e-01
    outer loop
      vertex   3.660747e+01 5.000000e+00 6.227910e+00
      vertex   3.715733e+01 1.500000e+01 7.280180e+00
      vertex   3.719906e+01 5.000000e+00 7.379790e+00
    endloop
  endfacet
  facet normal 8.951801e-01 -2.322850e-03 -4.456985e-01
    outer loop
      vertex   3.669263e+01 1.125000e+01 6.366380e+00
      vertex   3.715733e+01 1.500000e+01 7.280180e+00
      vertex   3.660747e+01 5.000000e+00 6.227910e+00
    endloop
  endfacet
  facet normal 8.390057e-01 2.844103e-02 -5.433789e-01
    outer loop
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
      vertex   3.715733e+01 1.500000e+01 7.280180e+00
      vertex   3.669263e+01 1.125000e+01 6.366380e+00
    endloop
  endfacet
  facet normal 7.059582e-01 -1.210728e-16 -7.082535e-01
    outer loop
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
      vertex   3.392812e+01 5.000000e+00 3.005000e+00
      vertex   3.392812e+01 1.500000e+01 3.005000e+00
    endloop
  endfacet
  facet normal 6.834347e-01 5.739760e-03 -7.299891e-01
    outer loop
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
      vertex   3.471620e+01 5.000000e+00 3.742820e+00
      vertex   3.392812e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal 7.627246e-01 -2.948051e-03 -6.467167e-01
    outer loop
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
      vertex   3.579742e+01 5.000000e+00 5.017990e+00
      vertex   3.471620e+01 5.000000e+00 3.742820e+00
    endloop
  endfacet
  facet normal 8.309363e-01 7.435779e-03 -5.563178e-01
    outer loop
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
      vertex   3.660747e+01 5.000000e+00 6.227910e+00
      vertex   3.579742e+01 5.000000e+00 5.017990e+00
    endloop
  endfacet
  facet normal 8.171074e-01 1.638504e-03 -5.764832e-01
    outer loop
      vertex   3.669263e+01 1.125000e+01 6.366380e+00
      vertex   3.660747e+01 5.000000e+00 6.227910e+00
      vertex   3.522704e+01 1.500000e+01 4.299710e+00
    endloop
  endfacet
  facet normal 9.351435e-01 -1.865047e-03 -3.542642e-01
    outer loop
      vertex   3.759880e+01 1.500000e+01 8.382330e+00
      vertex   3.773419e+01 5.000000e+00 8.792360e+00
      vertex   3.719906e+01 5.000000e+00 7.379790e+00
    endloop
  endfacet
  facet normal 9.602565e-01 1.556311e-03 -2.791146e-01
    outer loop
      vertex   3.790993e+01 1.125000e+01 9.431820e+00
      vertex   3.773419e+01 5.000000e+00 8.792360e+00
      vertex   3.759880e+01 1.500000e+01 8.382330e+00
    endloop
  endfacet
  facet normal 9.736577e-01 -4.052416e-03 -2.279787e-01
    outer loop
      vertex   3.790993e+01 1.125000e+01 9.431820e+00
      vertex   3.815228e+01 5.000000e+00 1.057795e+01
      vertex   3.773419e+01 5.000000e+00 8.792360e+00
    endloop
  endfacet
  facet normal 9.829213e-01 4.376357e-03 -1.839742e-01
    outer loop
      vertex   3.821394e+01 1.125000e+01 1.105606e+01
      vertex   3.815228e+01 5.000000e+00 1.057795e+01
      vertex   3.790993e+01 1.125000e+01 9.431820e+00
    endloop
  endfacet
  facet normal 9.985195e-01 -5.713044e-03 -5.409415e-02
    outer loop
      vertex   3.828640e+01 5.000000e+00 1.305366e+01
      vertex   3.815228e+01 5.000000e+00 1.057795e+01
      vertex   3.821394e+01 1.125000e+01 1.105606e+01
    endloop
  endfacet
  facet normal 9.985801e-01 -5.362898e-03 -5.300083e-02
    outer loop
      vertex   3.821394e+01 1.125000e+01 1.105606e+01
      vertex   3.830000e+01 1.500000e+01 1.229805e+01
      vertex   3.828640e+01 5.000000e+00 1.305366e+01
    endloop
  endfacet
  facet normal 9.975551e-01 3.915509e-03 6.977436e-02
    outer loop
      vertex   3.814949e+01 1.500000e+01 1.444987e+01
      vertex   3.828640e+01 5.000000e+00 1.305366e+01
      vertex   3.830000e+01 1.500000e+01 1.229805e+01
    endloop
  endfacet
  facet normal 9.911308e-01 -4.971726e-03 1.327970e-01
    outer loop
      vertex   3.814949e+01 1.500000e+01 1.444987e+01
      vertex   3.798570e+01 5.000000e+00 1.529793e+01
      vertex   3.828640e+01 5.000000e+00 1.305366e+01
    endloop
  endfacet
  facet normal 9.790328e-01 1.239353e-03 2.036991e-01
    outer loop
      vertex   3.792668e+01 1.125000e+01 1.554357e+01
      vertex   3.798570e+01 5.000000e+00 1.529793e+01
      vertex   3.814949e+01 1.500000e+01 1.444987e+01
    endloop
  endfacet
  facet normal 9.597724e-01 -1.971679e-03 2.807721e-01
    outer loop
      vertex   3.792668e+01 1.125000e+01 1.554357e+01
      vertex   3.756145e+01 5.000000e+00 1.674816e+01
      vertex   3.798570e+01 5.000000e+00 1.529793e+01
    endloop
  endfacet
  facet normal 9.569889e-01 -6.462256e-06 2.901246e-01
    outer loop
      vertex   3.771808e+01 1.500000e+01 1.623173e+01
      vertex   3.756145e+01 5.000000e+00 1.674816e+01
      vertex   3.792668e+01 1.125000e+01 1.554357e+01
    endloop
  endfacet
  facet normal 9.281573e-01 4.681616e-03 3.721588e-01
    outer loop
      vertex   3.771808e+01 1.500000e+01 1.623173e+01
      vertex   3.697619e+01 1.500000e+01 1.808199e+01
      vertex   3.756145e+01 5.000000e+00 1.674816e+01
    endloop
  endfacet
  facet normal 9.157260e-01 -0.000000e+00 4.018032e-01
    outer loop
      vertex   3.697619e+01 5.000000e+00 1.808199e+01
      vertex   3.756145e+01 5.000000e+00 1.674816e+01
      vertex   3.697619e+01 1.500000e+01 1.808199e+01
    endloop
  endfacet
  facet normal 8.733599e-01 -0.000000e+00 4.870754e-01
    outer loop
      vertex   3.697619e+01 5.000000e+00 1.808199e+01
      vertex   3.697619e+01 1.500000e+01 1.808199e+01
      vertex   3.631935e+01 5.000000e+00 1.925975e+01
    endloop
  endfacet
  facet normal 8.733599e-01 1.730440e-16 4.870754e-01
    outer loop
      vertex   3.631935e+01 5.000000e+00 1.925975e+01
      vertex   3.697619e+01 1.500000e+01 1.808199e+01
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
    endloop
  endfacet
  facet normal 8.010612e-01 2.126592e-16 5.985824e-01
    outer loop
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
      vertex   3.583192e+01 1.000000e+01 1.991206e+01
      vertex   3.631935e+01 5.000000e+00 1.925975e+01
    endloop
  endfacet
  facet normal 8.139890e-01 -3.571169e-03 5.808693e-01
    outer loop
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
      vertex   3.585266e+01 1.333333e+01 1.990349e+01
      vertex   3.583192e+01 1.000000e+01 1.991206e+01
    endloop
  endfacet
  facet normal 7.994946e-01 -4.254752e-04 6.006731e-01
    outer loop
      vertex   3.584527e+01 5.000000e+00 1.989075e+01
      vertex   3.631935e+01 5.000000e+00 1.925975e+01
      vertex   3.583192e+01 1.000000e+01 1.991206e+01
    endloop
  endfacet
  facet normal 3.615963e-01 -3.008134e-03 9.323299e-01
    outer loop
      vertex   3.555069e+01 5.000000e+00 2.000500e+01
      vertex   3.584527e+01 5.000000e+00 1.989075e+01
      vertex   3.583192e+01 1.000000e+01 1.991206e+01
    endloop
  endfacet
  facet normal 3.137867e-01 3.373278e-16 9.494935e-01
    outer loop
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
      vertex   3.555069e+01 5.000000e+00 2.000500e+01
      vertex   3.583192e+01 1.000000e+01 1.991206e+01
    endloop
  endfacet
  facet normal 8.165836e-02 -1.393122e-02 9.965630e-01
    outer loop
      vertex   3.573852e+01 1.333333e+01 1.996631e+01
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
      vertex   3.583192e+01 1.000000e+01 1.991206e+01
    endloop
  endfacet
  facet normal 6.947648e-01 6.166375e-02 7.165888e-01
    outer loop
      vertex   3.573852e+01 1.333333e+01 1.996631e+01
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
    endloop
  endfacet
  facet normal 4.725046e-01 1.992828e-01 8.585020e-01
    outer loop
      vertex   3.573852e+01 1.333333e+01 1.996631e+01
      vertex   3.585266e+01 1.333333e+01 1.990349e+01
      vertex   3.631935e+01 1.500000e+01 1.925975e+01
    endloop
  endfacet
  facet normal 4.821760e-01 -7.477232e-04 8.760741e-01
    outer loop
      vertex   3.573852e+01 1.333333e+01 1.996631e+01
      vertex   3.583192e+01 1.000000e+01 1.991206e+01
      vertex   3.585266e+01 1.333333e+01 1.990349e+01
    endloop
  endfacet
  facet normal -2.035606e-02 3.561858e-04 9.997927e-01
    outer loop
      vertex   3.343137e+01 1.500000e+01 1.996185e+01
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
    endloop
  endfacet
  facet normal -1.863339e-02 3.552097e-16 9.998264e-01
    outer loop
      vertex   3.555069e+01 5.000000e+00 2.000500e+01
      vertex   3.555069e+01 1.500000e+01 2.000500e+01
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
    endloop
  endfacet
  facet normal -9.879119e-01 -1.442057e-02 1.543444e-01
    outer loop
      vertex   3.314383e+01 8.020831e+00 1.948707e+01
      vertex   3.323889e+01 5.000000e+00 1.981328e+01
      vertex   3.317396e+01 8.333331e+00 1.970912e+01
    endloop
  endfacet
  facet normal -8.441397e-01 3.098369e-04 5.361231e-01
    outer loop
      vertex   3.323889e+01 5.000000e+00 1.981328e+01
      vertex   3.324381e+01 8.333331e+00 1.981910e+01
      vertex   3.317396e+01 8.333331e+00 1.970912e+01
    endloop
  endfacet
  facet normal -9.574425e-01 2.579863e-03 -2.886124e-01
    outer loop
      vertex   3.314383e+01 8.020831e+00 1.948707e+01
      vertex   3.325771e+01 1.500000e+01 1.917167e+01
      vertex   3.322541e+01 7.500000e+00 1.921178e+01
    endloop
  endfacet
  facet normal -9.885133e-01 9.312706e-03 -1.508468e-01
    outer loop
      vertex   3.314383e+01 8.020831e+00 1.948707e+01
      vertex   3.314915e+01 1.083333e+01 1.962584e+01
      vertex   3.325771e+01 1.500000e+01 1.917167e+01
    endloop
  endfacet
  facet normal -1.301553e-01 -4.861573e-02 9.903010e-01
    outer loop
      vertex   3.314383e+01 8.020831e+00 1.948707e+01
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
      vertex   3.314915e+01 1.083333e+01 1.962584e+01
    endloop
  endfacet
  facet normal -9.928069e-01 1.100984e-02 1.192193e-01
    outer loop
      vertex   3.317396e+01 8.333331e+00 1.970912e+01
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
      vertex   3.314383e+01 8.020831e+00 1.948707e+01
    endloop
  endfacet
  facet normal -8.441397e-01 4.660540e-04 5.361231e-01
    outer loop
      vertex   3.324381e+01 8.333331e+00 1.981910e+01
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
      vertex   3.317396e+01 8.333331e+00 1.970912e+01
    endloop
  endfacet
  facet normal -5.778414e-01 -1.460942e-03 8.161478e-01
    outer loop
      vertex   3.324381e+01 8.333331e+00 1.981910e+01
      vertex   3.344298e+01 1.083333e+01 1.996459e+01
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
    endloop
  endfacet
  facet normal -9.495486e-01 -9.429451e-03 -3.134782e-01
    outer loop
      vertex   3.314915e+01 1.083333e+01 1.962584e+01
      vertex   3.313252e+01 1.500000e+01 1.955088e+01
      vertex   3.325771e+01 1.500000e+01 1.917167e+01
    endloop
  endfacet
  facet normal -5.713511e-01 -1.052339e-03 8.207051e-01
    outer loop
      vertex   3.343137e+01 1.500000e+01 1.996185e+01
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
      vertex   3.344298e+01 1.083333e+01 1.996459e+01
    endloop
  endfacet
  facet normal -5.596038e-01 -3.647420e-03 8.287522e-01
    outer loop
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
      vertex   3.344298e+01 1.083333e+01 1.996459e+01
      vertex   3.324381e+01 8.333331e+00 1.981910e+01
    endloop
  endfacet
  facet normal -9.128840e-01 3.700156e-03 4.082023e-01
    outer loop
      vertex   3.314915e+01 1.083333e+01 1.962584e+01
      vertex   3.326427e+01 1.500000e+01 1.984552e+01
      vertex   3.313252e+01 1.500000e+01 1.955088e+01
    endloop
  endfacet
  facet normal 7.895460e-02 8.755235e-04 9.968778e-01
    outer loop
      vertex   3.344298e+01 1.083333e+01 1.996459e+01
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
      vertex   3.343137e+01 1.500000e+01 1.996185e+01
    endloop
  endfacet
  facet normal -7.807770e-01 -4.090505e-03 -6.247963e-01
    outer loop
      vertex   3.318915e+01 5.000000e+00 1.927346e+01
      vertex   3.322541e+01 7.500000e+00 1.921178e+01
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -9.553137e-01 6.564846e-03 -2.955209e-01
    outer loop
      vertex   3.318915e+01 5.000000e+00 1.927346e+01
      vertex   3.314383e+01 8.020831e+00 1.948707e+01
      vertex   3.322541e+01 7.500000e+00 1.921178e+01
    endloop
  endfacet
  facet normal -9.955533e-01 -2.142236e-02 9.173185e-02
    outer loop
      vertex   3.323889e+01 5.000000e+00 1.981328e+01
      vertex   3.314383e+01 8.020831e+00 1.948707e+01
      vertex   3.318915e+01 5.000000e+00 1.927346e+01
    endloop
  endfacet
  facet normal -5.303255e-01 -6.975571e-04 8.477938e-01
    outer loop
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
      vertex   3.324381e+01 8.333331e+00 1.981910e+01
      vertex   3.323889e+01 5.000000e+00 1.981328e+01
    endloop
  endfacet
  facet normal 5.090978e-01 -2.395894e-03 8.607053e-01
    outer loop
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
      vertex   1.832911e+01 9.583331e+00 1.985874e+01
      vertex   1.823186e+01 1.500000e+01 1.993134e+01
    endloop
  endfacet
  facet normal 5.872201e-01 -7.685127e-03 8.093908e-01
    outer loop
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
      vertex   1.840357e+01 5.000000e+00 1.976120e+01
      vertex   1.832911e+01 9.583331e+00 1.985874e+01
    endloop
  endfacet
  facet normal 9.656524e-01 1.016234e-02 2.596386e-01
    outer loop
      vertex   1.840357e+01 5.000000e+00 1.976120e+01
      vertex   1.847117e+01 5.000000e+00 1.950978e+01
      vertex   1.832911e+01 9.583331e+00 1.985874e+01
    endloop
  endfacet
  facet normal 9.342020e-01 -1.436113e-03 -3.567417e-01
    outer loop
      vertex   1.840698e+01 1.166667e+01 1.924917e+01
      vertex   1.846869e+01 1.166667e+01 1.941077e+01
      vertex   1.838657e+01 5.000000e+00 1.922256e+01
    endloop
  endfacet
  facet normal 6.466849e-01 1.387151e-03 7.627560e-01
    outer loop
      vertex   1.832911e+01 9.583331e+00 1.985874e+01
      vertex   1.837111e+01 1.500000e+01 1.981328e+01
      vertex   1.823186e+01 1.500000e+01 1.993134e+01
    endloop
  endfacet
  facet normal 9.090335e-01 -3.551304e-03 4.167079e-01
    outer loop
      vertex   1.847117e+01 5.000000e+00 1.950978e+01
      vertex   1.837111e+01 1.500000e+01 1.981328e+01
      vertex   1.832911e+01 9.583331e+00 1.985874e+01
    endloop
  endfacet
  facet normal 9.673618e-01 1.988994e-03 2.533912e-01
    outer loop
      vertex   1.847117e+01 5.000000e+00 1.950978e+01
      vertex   1.847196e+01 1.500000e+01 1.942827e+01
      vertex   1.837111e+01 1.500000e+01 1.981328e+01
    endloop
  endfacet
  facet normal 9.977425e-01 -6.261406e-04 -6.715217e-02
    outer loop
      vertex   1.846869e+01 1.166667e+01 1.941077e+01
      vertex   1.847196e+01 1.500000e+01 1.942827e+01
      vertex   1.847117e+01 5.000000e+00 1.950978e+01
    endloop
  endfacet
  facet normal 8.421927e-01 2.004539e-03 -5.391730e-01
    outer loop
      vertex   1.846869e+01 1.166667e+01 1.941077e+01
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
      vertex   1.847196e+01 1.500000e+01 1.942827e+01
    endloop
  endfacet
  facet normal 9.335996e-01 3.593261e-02 -3.565116e-01
    outer loop
      vertex   1.840698e+01 1.166667e+01 1.924917e+01
      vertex   1.816248e+01 1.500000e+01 1.894486e+01
      vertex   1.846869e+01 1.166667e+01 1.941077e+01
    endloop
  endfacet
  facet normal 3.388498e-02 4.588835e-03 9.994152e-01
    outer loop
      vertex   1.823186e+01 1.500000e+01 1.993134e+01
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
    endloop
  endfacet
  facet normal 9.592467e-01 -3.839333e-03 -2.825438e-01
    outer loop
      vertex   1.847117e+01 5.000000e+00 1.950978e+01
      vertex   1.838657e+01 5.000000e+00 1.922256e+01
      vertex   1.846869e+01 1.166667e+01 1.941077e+01
    endloop
  endfacet
  facet normal 1.139971e-02 3.553763e-16 9.999350e-01
    outer loop
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
      vertex   1.605931e+01 5.000000e+00 2.000500e+01
    endloop
  endfacet
  facet normal -3.915796e-01 2.381642e-03 9.201411e-01
    outer loop
      vertex   1.579755e+01 5.000000e+00 1.992915e+01
      vertex   1.599703e+01 1.000000e+01 2.000110e+01
      vertex   1.573813e+01 1.500000e+01 1.987798e+01
    endloop
  endfacet
  facet normal -6.249151e-02 3.545770e-16 9.980455e-01
    outer loop
      vertex   1.605931e+01 5.000000e+00 2.000500e+01
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
      vertex   1.599703e+01 1.000000e+01 2.000110e+01
    endloop
  endfacet
  facet normal -2.783176e-01 -2.717611e-03 9.604852e-01
    outer loop
      vertex   1.579755e+01 5.000000e+00 1.992915e+01
      vertex   1.605931e+01 5.000000e+00 2.000500e+01
      vertex   1.599703e+01 1.000000e+01 2.000110e+01
    endloop
  endfacet
  facet normal -7.571810e-01 -1.156673e-03 6.532041e-01
    outer loop
      vertex   1.554305e+01 5.000000e+00 1.963414e+01
      vertex   1.579755e+01 5.000000e+00 1.992915e+01
      vertex   1.573813e+01 1.500000e+01 1.987798e+01
    endloop
  endfacet
  facet normal -3.677613e-01 3.855560e-03 9.299122e-01
    outer loop
      vertex   1.605931e+01 1.500000e+01 2.000500e+01
      vertex   1.573813e+01 1.500000e+01 1.987798e+01
      vertex   1.599703e+01 1.000000e+01 2.000110e+01
    endloop
  endfacet
  facet normal -5.556520e-16 -1.000000e+00 6.620587e-16
    outer loop
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
      vertex   1.806883e+01 5.000000e+00 6.012840e+00
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal 6.883162e-16 -1.000000e+00 5.020256e-16
    outer loop
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
      vertex   1.806883e+01 5.000000e+00 6.012840e+00
      vertex   1.731050e+01 5.000000e+00 7.052570e+00
    endloop
  endfacet
  facet normal -6.394178e-16 -1.000000e+00 3.802010e-16
    outer loop
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
      vertex   1.731050e+01 5.000000e+00 7.052570e+00
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
      vertex   1.731050e+01 5.000000e+00 7.052570e+00
      vertex   1.685985e+01 5.000000e+00 7.821330e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
      vertex   1.685985e+01 5.000000e+00 7.821330e+00
      vertex   1.623773e+01 5.000000e+00 9.277420e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.597051e+01 5.000000e+00 4.798990e+00
      vertex   1.623773e+01 5.000000e+00 9.277420e+00
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
      vertex   1.623773e+01 5.000000e+00 9.277420e+00
      vertex   1.587128e+01 5.000000e+00 1.071591e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
      vertex   1.587128e+01 5.000000e+00 1.071591e+01
      vertex   1.574884e+01 5.000000e+00 1.159860e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.424067e+01 5.000000e+00 7.776610e+00
      vertex   1.574884e+01 5.000000e+00 1.159860e+01
      vertex   1.373067e+01 5.000000e+00 9.312140e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.373067e+01 5.000000e+00 9.312140e+00
      vertex   1.574884e+01 5.000000e+00 1.159860e+01
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.373067e+01 5.000000e+00 9.312140e+00
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.331000e+01 5.000000e+00 1.229805e+01
    endloop
  endfacet
  facet normal -2.674816e-16 -1.000000e+00 1.321168e-15
    outer loop
      vertex   1.331000e+01 5.000000e+00 1.229805e+01
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.344740e+01 5.000000e+00 1.434267e+01
    endloop
  endfacet
  facet normal -1.040564e-15 -1.000000e+00 1.926969e-16
    outer loop
      vertex   1.344740e+01 5.000000e+00 1.434267e+01
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.362430e+01 5.000000e+00 1.529793e+01
    endloop
  endfacet
  facet normal -9.496148e-16 -1.000000e+00 2.685654e-16
    outer loop
      vertex   1.362430e+01 5.000000e+00 1.529793e+01
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.394741e+01 5.000000e+00 1.644041e+01
    endloop
  endfacet
  facet normal 2.355202e-15 -1.000000e+00 1.871353e-15
    outer loop
      vertex   1.394741e+01 5.000000e+00 1.644041e+01
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.583308e+01 5.000000e+00 1.406719e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.394741e+01 5.000000e+00 1.644041e+01
      vertex   1.583308e+01 5.000000e+00 1.406719e+01
      vertex   1.428591e+01 5.000000e+00 1.735102e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.428591e+01 5.000000e+00 1.735102e+01
      vertex   1.583308e+01 5.000000e+00 1.406719e+01
      vertex   1.608557e+01 5.000000e+00 1.522775e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.428591e+01 5.000000e+00 1.735102e+01
      vertex   1.608557e+01 5.000000e+00 1.522775e+01
      vertex   1.554305e+01 5.000000e+00 1.963414e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.554305e+01 5.000000e+00 1.963414e+01
      vertex   1.608557e+01 5.000000e+00 1.522775e+01
      vertex   1.643220e+01 5.000000e+00 1.626017e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.554305e+01 5.000000e+00 1.963414e+01
      vertex   1.643220e+01 5.000000e+00 1.626017e+01
      vertex   1.579755e+01 5.000000e+00 1.992915e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.579755e+01 5.000000e+00 1.992915e+01
      vertex   1.643220e+01 5.000000e+00 1.626017e+01
      vertex   1.605931e+01 5.000000e+00 2.000500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.605931e+01 5.000000e+00 2.000500e+01
      vertex   1.643220e+01 5.000000e+00 1.626017e+01
      vertex   1.713438e+01 5.000000e+00 1.767936e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.605931e+01 5.000000e+00 2.000500e+01
      vertex   1.713438e+01 5.000000e+00 1.767936e+01
      vertex   1.768126e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.605931e+01 5.000000e+00 2.000500e+01
      vertex   1.768126e+01 5.000000e+00 1.850500e+01
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
      vertex   1.768126e+01 5.000000e+00 1.850500e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
      vertex   1.838657e+01 5.000000e+00 1.922256e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.847117e+01 5.000000e+00 1.950978e+01
      vertex   1.840357e+01 5.000000e+00 1.976120e+01
      vertex   1.838657e+01 5.000000e+00 1.922256e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.838657e+01 5.000000e+00 1.922256e+01
      vertex   1.840357e+01 5.000000e+00 1.976120e+01
      vertex   1.809959e+01 5.000000e+00 1.998174e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 8.881784e-16
    outer loop
      vertex   1.919137e+01 5.000000e+00 3.005000e+00
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal 5.132982e-16 -1.000000e+00 5.007690e-16
    outer loop
      vertex   1.768188e+01 5.000000e+00 3.005000e+00
      vertex   1.919137e+01 5.000000e+00 5.005000e+00
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.658650e+01 4.700000e+00 1.850500e+01
      vertex   1.658650e+01 -5.000000e+00 1.850500e+01
      vertex   1.768126e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.768126e+01 4.700000e+00 1.850500e+01
      vertex   1.658650e+01 -5.000000e+00 1.850500e+01
      vertex   1.780591e+01 -5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.768126e+01 4.700000e+00 1.850500e+01
      vertex   1.780591e+01 -5.000000e+00 1.850500e+01
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.780591e+01 5.000000e+00 1.850500e+01
      vertex   1.768126e+01 5.000000e+00 1.850500e+01
      vertex   1.768126e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
      vertex   3.392874e+01 5.000000e+00 1.850500e+01
      vertex   3.555069e+01 5.000000e+00 2.000500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.555069e+01 5.000000e+00 2.000500e+01
      vertex   3.392874e+01 5.000000e+00 1.850500e+01
      vertex   3.447562e+01 5.000000e+00 1.767936e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.555069e+01 5.000000e+00 2.000500e+01
      vertex   3.447562e+01 5.000000e+00 1.767936e+01
      vertex   3.497935e+01 5.000000e+00 1.671383e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.323889e+01 5.000000e+00 1.981328e+01
      vertex   3.318915e+01 5.000000e+00 1.927346e+01
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
      vertex   3.318915e+01 5.000000e+00 1.927346e+01
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.348380e+01 5.000000e+00 1.996648e+01
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
      vertex   3.392874e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.555069e+01 5.000000e+00 2.000500e+01
      vertex   3.497935e+01 5.000000e+00 1.671383e+01
      vertex   3.584527e+01 5.000000e+00 1.989075e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.584527e+01 5.000000e+00 1.989075e+01
      vertex   3.497935e+01 5.000000e+00 1.671383e+01
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.584527e+01 5.000000e+00 1.989075e+01
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
      vertex   3.631935e+01 5.000000e+00 1.925975e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.631935e+01 5.000000e+00 1.925975e+01
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
      vertex   3.697619e+01 5.000000e+00 1.808199e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.697619e+01 5.000000e+00 1.808199e+01
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
      vertex   3.580563e+01 5.000000e+00 1.387128e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.697619e+01 5.000000e+00 1.808199e+01
      vertex   3.580563e+01 5.000000e+00 1.387128e+01
      vertex   3.756145e+01 5.000000e+00 1.674816e+01
    endloop
  endfacet
  facet normal -3.530855e-15 -1.000000e+00 2.154954e-15
    outer loop
      vertex   3.756145e+01 5.000000e+00 1.674816e+01
      vertex   3.580563e+01 5.000000e+00 1.387128e+01
      vertex   3.589799e+01 5.000000e+00 1.278614e+01
    endloop
  endfacet
  facet normal 9.440303e-16 -1.000000e+00 2.761669e-16
    outer loop
      vertex   3.756145e+01 5.000000e+00 1.674816e+01
      vertex   3.589799e+01 5.000000e+00 1.278614e+01
      vertex   3.798570e+01 5.000000e+00 1.529793e+01
    endloop
  endfacet
  facet normal -1.310159e-16 -1.000000e+00 1.169707e-15
    outer loop
      vertex   3.798570e+01 5.000000e+00 1.529793e+01
      vertex   3.589799e+01 5.000000e+00 1.278614e+01
      vertex   3.828640e+01 5.000000e+00 1.305366e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.828640e+01 5.000000e+00 1.305366e+01
      vertex   3.589799e+01 5.000000e+00 1.278614e+01
      vertex   3.815228e+01 5.000000e+00 1.057795e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.815228e+01 5.000000e+00 1.057795e+01
      vertex   3.589799e+01 5.000000e+00 1.278614e+01
      vertex   3.588007e+01 5.000000e+00 1.189518e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.815228e+01 5.000000e+00 1.057795e+01
      vertex   3.588007e+01 5.000000e+00 1.189518e+01
      vertex   3.773419e+01 5.000000e+00 8.792360e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.773419e+01 5.000000e+00 8.792360e+00
      vertex   3.588007e+01 5.000000e+00 1.189518e+01
      vertex   3.719906e+01 5.000000e+00 7.379790e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.719906e+01 5.000000e+00 7.379790e+00
      vertex   3.588007e+01 5.000000e+00 1.189518e+01
      vertex   3.567750e+01 5.000000e+00 1.042520e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.719906e+01 5.000000e+00 7.379790e+00
      vertex   3.567750e+01 5.000000e+00 1.042520e+01
      vertex   3.660747e+01 5.000000e+00 6.227910e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.660747e+01 5.000000e+00 6.227910e+00
      vertex   3.567750e+01 5.000000e+00 1.042520e+01
      vertex   3.537227e+01 5.000000e+00 9.277420e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.660747e+01 5.000000e+00 6.227910e+00
      vertex   3.537227e+01 5.000000e+00 9.277420e+00
      vertex   3.579742e+01 5.000000e+00 5.017990e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.579742e+01 5.000000e+00 5.017990e+00
      vertex   3.537227e+01 5.000000e+00 9.277420e+00
      vertex   3.455955e+01 5.000000e+00 7.473960e+00
    endloop
  endfacet
  facet normal 1.030406e-15 -1.000000e+00 5.193506e-16
    outer loop
      vertex   3.579742e+01 5.000000e+00 5.017990e+00
      vertex   3.455955e+01 5.000000e+00 7.473960e+00
      vertex   3.471620e+01 5.000000e+00 3.742820e+00
    endloop
  endfacet
  facet normal -6.442608e-16 -1.000000e+00 4.490407e-16
    outer loop
      vertex   3.471620e+01 5.000000e+00 3.742820e+00
      vertex   3.455955e+01 5.000000e+00 7.473960e+00
      vertex   3.354117e+01 5.000000e+00 6.012840e+00
    endloop
  endfacet
  facet normal -4.934764e-16 -1.000000e+00 5.270912e-16
    outer loop
      vertex   3.471620e+01 5.000000e+00 3.742820e+00
      vertex   3.354117e+01 5.000000e+00 6.012840e+00
      vertex   3.392812e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal 5.556520e-16 -1.000000e+00 6.620587e-16
    outer loop
      vertex   3.392812e+01 5.000000e+00 3.005000e+00
      vertex   3.354117e+01 5.000000e+00 6.012840e+00
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
    endloop
  endfacet
  facet normal -5.132982e-16 -1.000000e+00 5.007690e-16
    outer loop
      vertex   3.392812e+01 5.000000e+00 3.005000e+00
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 8.881784e-16
    outer loop
      vertex   3.241863e+01 5.000000e+00 5.005000e+00
      vertex   3.241863e+01 5.000000e+00 3.005000e+00
      vertex   3.392812e+01 5.000000e+00 3.005000e+00
    endloop
  endfacet
  facet normal -6.428253e-01 -1.399010e-16 -7.660128e-01
    outer loop
      vertex   3.354117e+01 4.700000e+00 6.012840e+00
      vertex   3.423053e+01 4.700000e+00 5.434340e+00
      vertex   3.423053e+01 -5.000000e+00 5.434340e+00
    endloop
  endfacet
  facet normal -6.428683e-01 -1.360648e-16 -7.659768e-01
    outer loop
      vertex   3.346458e+01 -5.000000e+00 6.077120e+00
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
      vertex   3.354117e+01 4.700000e+00 6.012840e+00
    endloop
  endfacet
  facet normal -6.428683e-01 0.000000e+00 -7.659768e-01
    outer loop
      vertex   3.354117e+01 4.700000e+00 6.012840e+00
      vertex   3.346458e+01 5.000000e+00 6.077120e+00
      vertex   3.354117e+01 5.000000e+00 6.012840e+00
    endloop
  endfacet
  facet normal -6.428296e-01 -5.202833e-07 -7.660092e-01
    outer loop
      vertex   3.423053e+01 -5.000000e+00 5.434340e+00
      vertex   3.346458e+01 -5.000000e+00 6.077120e+00
      vertex   3.354117e+01 4.700000e+00 6.012840e+00
    endloop
  endfacet
  facet normal 8.203919e-01 0.000000e+00 -5.718016e-01
    outer loop
      vertex   3.455955e+01 5.000000e+00 7.473960e+00
      vertex   3.455955e+01 4.700000e+00 7.473960e+00
      vertex   3.354117e+01 5.000000e+00 6.012840e+00
    endloop
  endfacet
  facet normal 8.203919e-01 0.000000e+00 -5.718016e-01
    outer loop
      vertex   3.354117e+01 5.000000e+00 6.012840e+00
      vertex   3.455955e+01 4.700000e+00 7.473960e+00
      vertex   3.354117e+01 4.700000e+00 6.012840e+00
    endloop
  endfacet
  facet normal 9.117013e-01 0.000000e+00 -4.108537e-01
    outer loop
      vertex   3.537227e+01 5.000000e+00 9.277420e+00
      vertex   3.537227e+01 4.700000e+00 9.277420e+00
      vertex   3.455955e+01 5.000000e+00 7.473960e+00
    endloop
  endfacet
  facet normal 9.117013e-01 0.000000e+00 -4.108537e-01
    outer loop
      vertex   3.455955e+01 5.000000e+00 7.473960e+00
      vertex   3.537227e+01 4.700000e+00 9.277420e+00
      vertex   3.455955e+01 4.700000e+00 7.473960e+00
    endloop
  endfacet
  facet normal 9.664117e-01 0.000000e+00 -2.569987e-01
    outer loop
      vertex   3.567750e+01 5.000000e+00 1.042520e+01
      vertex   3.537227e+01 4.700000e+00 9.277420e+00
      vertex   3.537227e+01 5.000000e+00 9.277420e+00
    endloop
  endfacet
  facet normal 9.662336e-01 -1.284190e-01 -2.233859e-01
    outer loop
      vertex   3.567750e+01 5.000000e+00 1.042520e+01
      vertex   3.581785e+01 4.700000e+00 1.120473e+01
      vertex   3.537227e+01 4.700000e+00 9.277420e+00
    endloop
  endfacet
  facet normal 9.848340e-01 1.080907e-01 -1.357146e-01
    outer loop
      vertex   3.588007e+01 5.000000e+00 1.189518e+01
      vertex   3.581785e+01 4.700000e+00 1.120473e+01
      vertex   3.567750e+01 5.000000e+00 1.042520e+01
    endloop
  endfacet
  facet normal 9.961536e-01 -6.031391e-02 -6.356297e-02
    outer loop
      vertex   3.588007e+01 5.000000e+00 1.189518e+01
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
      vertex   3.581785e+01 4.700000e+00 1.120473e+01
    endloop
  endfacet
  facet normal 9.997314e-01 1.152186e-02 -2.010722e-02
    outer loop
      vertex   3.589799e+01 5.000000e+00 1.278614e+01
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
      vertex   3.588007e+01 5.000000e+00 1.189518e+01
    endloop
  endfacet
  facet normal 9.884625e-01 -1.259518e-01 8.413103e-02
    outer loop
      vertex   3.580563e+01 5.000000e+00 1.387128e+01
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
      vertex   3.589799e+01 5.000000e+00 1.278614e+01
    endloop
  endfacet
  facet normal 9.975152e-01 5.073290e-02 4.888437e-02
    outer loop
      vertex   3.580563e+01 5.000000e+00 1.387128e+01
      vertex   3.583052e+01 4.700000e+00 1.367473e+01
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
    endloop
  endfacet
  facet normal 9.735999e-01 -6.296977e-02 2.194038e-01
    outer loop
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
      vertex   3.583052e+01 4.700000e+00 1.367473e+01
      vertex   3.580563e+01 5.000000e+00 1.387128e+01
    endloop
  endfacet
  facet normal 9.783182e-01 4.193819e-02 2.028170e-01
    outer loop
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
      vertex   3.546912e+01 4.700000e+00 1.541800e+01
      vertex   3.583052e+01 4.700000e+00 1.367473e+01
    endloop
  endfacet
  facet normal 9.358243e-01 1.637861e-02 3.520862e-01
    outer loop
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
      vertex   3.501619e+01 4.700000e+00 1.662186e+01
      vertex   3.546912e+01 4.700000e+00 1.541800e+01
    endloop
  endfacet
  facet normal 9.366635e-01 7.680495e-03 3.501463e-01
    outer loop
      vertex   3.497935e+01 5.000000e+00 1.671383e+01
      vertex   3.501619e+01 4.700000e+00 1.662186e+01
      vertex   3.544692e+01 5.000000e+00 1.546305e+01
    endloop
  endfacet
  facet normal 8.861141e-01 -3.290876e-02 4.622973e-01
    outer loop
      vertex   3.447562e+01 5.000000e+00 1.767936e+01
      vertex   3.501619e+01 4.700000e+00 1.662186e+01
      vertex   3.497935e+01 5.000000e+00 1.671383e+01
    endloop
  endfacet
  facet normal 8.950864e-01 5.167522e-02 4.428883e-01
    outer loop
      vertex   3.447562e+01 5.000000e+00 1.767936e+01
      vertex   3.457714e+01 4.700000e+00 1.750919e+01
      vertex   3.501619e+01 4.700000e+00 1.662186e+01
    endloop
  endfacet
  facet normal 8.377299e-01 -2.592140e-02 5.454693e-01
    outer loop
      vertex   3.447562e+01 5.000000e+00 1.767936e+01
      vertex   3.392874e+01 4.700000e+00 1.850500e+01
      vertex   3.457714e+01 4.700000e+00 1.750919e+01
    endloop
  endfacet
  facet normal 8.336995e-01 0.000000e+00 5.522183e-01
    outer loop
      vertex   3.392874e+01 5.000000e+00 1.850500e+01
      vertex   3.392874e+01 4.700000e+00 1.850500e+01
      vertex   3.447562e+01 5.000000e+00 1.767936e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   3.392874e+01 5.000000e+00 1.850500e+01
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
      vertex   3.392874e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   3.392874e+01 4.700000e+00 1.850500e+01
      vertex   3.380409e+01 5.000000e+00 1.850500e+01
      vertex   3.380409e+01 -5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   3.392874e+01 4.700000e+00 1.850500e+01
      vertex   3.380409e+01 -5.000000e+00 1.850500e+01
      vertex   3.502350e+01 -5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   3.502350e+01 -5.000000e+00 1.850500e+01
      vertex   3.502350e+01 4.700000e+00 1.850500e+01
      vertex   3.392874e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 3.235608e-16 -1.000000e+00 3.855618e-16
    outer loop
      vertex   3.346458e+01 -5.000000e+00 6.077120e+00
      vertex   3.423053e+01 -5.000000e+00 5.434340e+00
      vertex   3.447097e+01 -5.000000e+00 7.536160e+00
    endloop
  endfacet
  facet normal -1.854425e-17 -1.000000e+00 4.246972e-16
    outer loop
      vertex   3.447097e+01 -5.000000e+00 7.536160e+00
      vertex   3.423053e+01 -5.000000e+00 5.434340e+00
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.447097e+01 -5.000000e+00 7.536160e+00
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
      vertex   3.500425e+01 -5.000000e+00 8.591480e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.500425e+01 -5.000000e+00 8.591480e+00
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
      vertex   3.571499e+01 -5.000000e+00 1.124990e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   3.571499e+01 -5.000000e+00 1.124990e+01
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
      vertex   3.623458e+01 -5.000000e+00 9.012280e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.571499e+01 -5.000000e+00 1.124990e+01
      vertex   3.623458e+01 -5.000000e+00 9.012280e+00
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
    endloop
  endfacet
  facet normal 5.872518e-16 -1.000000e+00 5.698314e-16
    outer loop
      vertex   3.571499e+01 -5.000000e+00 1.124990e+01
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
    endloop
  endfacet
  facet normal 3.343635e-16 -1.000000e+00 4.861425e-16
    outer loop
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
      vertex   3.678446e+01 -5.000000e+00 1.204520e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
      vertex   3.678446e+01 -5.000000e+00 1.204520e+01
      vertex   3.672988e+01 -5.000000e+00 1.369138e+01
    endloop
  endfacet
  facet normal -2.629218e-16 -1.000000e+00 2.540526e-16
    outer loop
      vertex   3.672988e+01 -5.000000e+00 1.369138e+01
      vertex   3.605497e+01 -5.000000e+00 1.648895e+01
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
    endloop
  endfacet
  facet normal 1.104154e-15 -1.000000e+00 1.600268e-16
    outer loop
      vertex   3.579609e+01 -5.000000e+00 1.272499e+01
      vertex   3.605497e+01 -5.000000e+00 1.648895e+01
      vertex   3.544338e+01 -5.000000e+00 1.515862e+01
    endloop
  endfacet
  facet normal 7.240026e-16 -1.000000e+00 3.347931e-16
    outer loop
      vertex   3.544338e+01 -5.000000e+00 1.515862e+01
      vertex   3.605497e+01 -5.000000e+00 1.648895e+01
      vertex   3.553609e+01 -5.000000e+00 1.761105e+01
    endloop
  endfacet
  facet normal 1.017305e-15 -1.000000e+00 3.237053e-16
    outer loop
      vertex   3.544338e+01 -5.000000e+00 1.515862e+01
      vertex   3.553609e+01 -5.000000e+00 1.761105e+01
      vertex   3.520419e+01 -5.000000e+00 1.591032e+01
    endloop
  endfacet
  facet normal 6.795059e-16 -1.000000e+00 3.896273e-16
    outer loop
      vertex   3.520419e+01 -5.000000e+00 1.591032e+01
      vertex   3.553609e+01 -5.000000e+00 1.761105e+01
      vertex   3.502350e+01 -5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -6.160804e-16 -1.000000e+00 2.994046e-16
    outer loop
      vertex   3.520419e+01 -5.000000e+00 1.591032e+01
      vertex   3.502350e+01 -5.000000e+00 1.850500e+01
      vertex   3.450103e+01 -5.000000e+00 1.742992e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.450103e+01 -5.000000e+00 1.742992e+01
      vertex   3.502350e+01 -5.000000e+00 1.850500e+01
      vertex   3.380409e+01 -5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
      vertex   3.660005e+01 4.700000e+00 1.040787e+01
      vertex   3.581785e+01 4.700000e+00 1.120473e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.581785e+01 4.700000e+00 1.120473e+01
      vertex   3.660005e+01 4.700000e+00 1.040787e+01
      vertex   3.601994e+01 4.700000e+00 8.432070e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.581785e+01 4.700000e+00 1.120473e+01
      vertex   3.601994e+01 4.700000e+00 8.432070e+00
      vertex   3.537227e+01 4.700000e+00 9.277420e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.537227e+01 4.700000e+00 9.277420e+00
      vertex   3.601994e+01 4.700000e+00 8.432070e+00
      vertex   3.479207e+01 4.700000e+00 6.172750e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.537227e+01 4.700000e+00 9.277420e+00
      vertex   3.479207e+01 4.700000e+00 6.172750e+00
      vertex   3.455955e+01 4.700000e+00 7.473960e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.455955e+01 4.700000e+00 7.473960e+00
      vertex   3.479207e+01 4.700000e+00 6.172750e+00
      vertex   3.423053e+01 4.700000e+00 5.434340e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.455955e+01 4.700000e+00 7.473960e+00
      vertex   3.423053e+01 4.700000e+00 5.434340e+00
      vertex   3.354117e+01 4.700000e+00 6.012840e+00
    endloop
  endfacet
  facet normal -2.321140e-15 1.000000e+00 -1.137501e-16
    outer loop
      vertex   3.583052e+01 4.700000e+00 1.367473e+01
      vertex   3.653517e+01 4.700000e+00 1.491223e+01
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
    endloop
  endfacet
  facet normal 6.902201e-16 1.000000e+00 -8.798819e-16
    outer loop
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
      vertex   3.653517e+01 4.700000e+00 1.491223e+01
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.589348e+01 4.700000e+00 1.239000e+01
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
      vertex   3.660005e+01 4.700000e+00 1.040787e+01
    endloop
  endfacet
  facet normal -1.628937e-15 1.000000e+00 -5.079001e-16
    outer loop
      vertex   3.653517e+01 4.700000e+00 1.491223e+01
      vertex   3.583052e+01 4.700000e+00 1.367473e+01
      vertex   3.601404e+01 4.700000e+00 1.658360e+01
    endloop
  endfacet
  facet normal 1.510017e-15 1.000000e+00 -7.059354e-16
    outer loop
      vertex   3.601404e+01 4.700000e+00 1.658360e+01
      vertex   3.583052e+01 4.700000e+00 1.367473e+01
      vertex   3.546912e+01 4.700000e+00 1.541800e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.601404e+01 4.700000e+00 1.658360e+01
      vertex   3.546912e+01 4.700000e+00 1.541800e+01
      vertex   3.501619e+01 4.700000e+00 1.662186e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.601404e+01 4.700000e+00 1.658360e+01
      vertex   3.501619e+01 4.700000e+00 1.662186e+01
      vertex   3.502350e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.502350e+01 4.700000e+00 1.850500e+01
      vertex   3.501619e+01 4.700000e+00 1.662186e+01
      vertex   3.457714e+01 4.700000e+00 1.750919e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.502350e+01 4.700000e+00 1.850500e+01
      vertex   3.457714e+01 4.700000e+00 1.750919e+01
      vertex   3.392874e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 8.888275e-01 4.740208e-03 4.582173e-01
    outer loop
      vertex   3.601404e+01 4.700000e+00 1.658360e+01
      vertex   3.502350e+01 4.700000e+00 1.850500e+01
      vertex   3.553609e+01 -5.000000e+00 1.761105e+01
    endloop
  endfacet
  facet normal 9.076548e-01 -2.654823e-04 4.197174e-01
    outer loop
      vertex   3.605497e+01 -5.000000e+00 1.648895e+01
      vertex   3.601404e+01 4.700000e+00 1.658360e+01
      vertex   3.553609e+01 -5.000000e+00 1.761105e+01
    endloop
  endfacet
  facet normal 9.546699e-01 1.123862e-03 2.976646e-01
    outer loop
      vertex   3.653517e+01 4.700000e+00 1.491223e+01
      vertex   3.601404e+01 4.700000e+00 1.658360e+01
      vertex   3.605497e+01 -5.000000e+00 1.648895e+01
    endloop
  endfacet
  facet normal 9.632640e-01 -4.038023e-03 2.685260e-01
    outer loop
      vertex   3.660971e+01 -7.999998e-01 1.456213e+01
      vertex   3.653517e+01 4.700000e+00 1.491223e+01
      vertex   3.605497e+01 -5.000000e+00 1.648895e+01
    endloop
  endfacet
  facet normal 9.922045e-01 5.522217e-03 1.244983e-01
    outer loop
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
      vertex   3.653517e+01 4.700000e+00 1.491223e+01
      vertex   3.660971e+01 -7.999998e-01 1.456213e+01
    endloop
  endfacet
  facet normal 9.929554e-01 3.858112e-03 1.184255e-01
    outer loop
      vertex   3.672988e+01 -5.000000e+00 1.369138e+01
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
      vertex   3.660971e+01 -7.999998e-01 1.456213e+01
    endloop
  endfacet
  facet normal 9.979333e-01 1.677534e-02 -6.203002e-02
    outer loop
      vertex   3.660005e+01 4.700000e+00 1.040787e+01
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
      vertex   3.678430e+01 -7.999998e-01 1.188465e+01
    endloop
  endfacet
  facet normal 9.915937e-01 -1.521359e-03 -1.293818e-01
    outer loop
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
      vertex   3.660005e+01 4.700000e+00 1.040787e+01
      vertex   3.678430e+01 -7.999998e-01 1.188465e+01
    endloop
  endfacet
  facet normal 9.883153e-01 -1.399597e-03 -1.524166e-01
    outer loop
      vertex   3.656720e+01 -7.999998e-01 1.024537e+01
      vertex   3.660005e+01 4.700000e+00 1.040787e+01
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
    endloop
  endfacet
  facet normal 9.594947e-01 2.592712e-03 -2.817148e-01
    outer loop
      vertex   3.601994e+01 4.700000e+00 8.432070e+00
      vertex   3.660005e+01 4.700000e+00 1.040787e+01
      vertex   3.656720e+01 -7.999998e-01 1.024537e+01
    endloop
  endfacet
  facet normal 9.654759e-01 1.025119e-02 -2.602907e-01
    outer loop
      vertex   3.624424e+01 -7.999998e-01 9.047440e+00
      vertex   3.601994e+01 4.700000e+00 8.432070e+00
      vertex   3.656720e+01 -7.999998e-01 1.024537e+01
    endloop
  endfacet
  facet normal 9.274624e-01 -4.009747e-03 -3.738950e-01
    outer loop
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
      vertex   3.601994e+01 4.700000e+00 8.432070e+00
      vertex   3.624424e+01 -7.999998e-01 9.047440e+00
    endloop
  endfacet
  facet normal 8.786068e-01 6.941941e-03 -4.774954e-01
    outer loop
      vertex   3.479207e+01 4.700000e+00 6.172750e+00
      vertex   3.601994e+01 4.700000e+00 8.432070e+00
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
    endloop
  endfacet
  facet normal 8.370815e-01 -6.816269e-03 -5.470357e-01
    outer loop
      vertex   3.423053e+01 -5.000000e+00 5.434340e+00
      vertex   3.479207e+01 4.700000e+00 6.172750e+00
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
    endloop
  endfacet
  facet normal 7.959814e-01 -1.108522e-16 -6.053211e-01
    outer loop
      vertex   3.423053e+01 4.700000e+00 5.434340e+00
      vertex   3.479207e+01 4.700000e+00 6.172750e+00
      vertex   3.423053e+01 -5.000000e+00 5.434340e+00
    endloop
  endfacet
  facet normal 9.222955e-01 1.113984e-03 -3.864838e-01
    outer loop
      vertex   3.624424e+01 -7.999998e-01 9.047440e+00
      vertex   3.623458e+01 -5.000000e+00 9.012280e+00
      vertex   3.563736e+01 -5.000000e+00 7.587090e+00
    endloop
  endfacet
  facet normal 9.688674e-01 -1.559623e-04 -2.475801e-01
    outer loop
      vertex   3.624424e+01 -7.999998e-01 9.047440e+00
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
      vertex   3.623458e+01 -5.000000e+00 9.012280e+00
    endloop
  endfacet
  facet normal 9.655173e-01 -4.397421e-03 -2.603018e-01
    outer loop
      vertex   3.656720e+01 -7.999998e-01 1.024537e+01
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
      vertex   3.624424e+01 -7.999998e-01 9.047440e+00
    endloop
  endfacet
  facet normal 9.926333e-01 -4.590185e-03 -1.210702e-01
    outer loop
      vertex   3.678430e+01 -7.999998e-01 1.188465e+01
      vertex   3.678446e+01 -5.000000e+00 1.204520e+01
      vertex   3.657887e+01 -5.000000e+00 1.035961e+01
    endloop
  endfacet
  facet normal 9.999144e-01 5.378180e-04 1.307153e-02
    outer loop
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
      vertex   3.678446e+01 -5.000000e+00 1.204520e+01
      vertex   3.678430e+01 -7.999998e-01 1.188465e+01
    endloop
  endfacet
  facet normal 9.989307e-01 6.774575e-03 -4.573333e-02
    outer loop
      vertex   3.679597e+01 -7.999998e-01 1.291876e+01
      vertex   3.678446e+01 -5.000000e+00 1.204520e+01
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
    endloop
  endfacet
  facet normal 9.994045e-01 -9.630718e-03 3.313564e-02
    outer loop
      vertex   3.679597e+01 -7.999998e-01 1.291876e+01
      vertex   3.672988e+01 -5.000000e+00 1.369138e+01
      vertex   3.678446e+01 -5.000000e+00 1.204520e+01
    endloop
  endfacet
  facet normal 9.950339e-01 2.646481e-03 9.950124e-02
    outer loop
      vertex   3.676579e+01 4.700000e+00 1.307428e+01
      vertex   3.672988e+01 -5.000000e+00 1.369138e+01
      vertex   3.679597e+01 -7.999998e-01 1.291876e+01
    endloop
  endfacet
  facet normal 9.719008e-01 -2.080272e-02 2.344696e-01
    outer loop
      vertex   3.660971e+01 -7.999998e-01 1.456213e+01
      vertex   3.605497e+01 -5.000000e+00 1.648895e+01
      vertex   3.672988e+01 -5.000000e+00 1.369138e+01
    endloop
  endfacet
  facet normal 8.675062e-01 0.000000e+00 4.974264e-01
    outer loop
      vertex   3.502350e+01 4.700000e+00 1.850500e+01
      vertex   3.502350e+01 -5.000000e+00 1.850500e+01
      vertex   3.553609e+01 -5.000000e+00 1.761105e+01
    endloop
  endfacet
  facet normal -8.336995e-01 0.000000e+00 5.522183e-01
    outer loop
      vertex   1.713438e+01 5.000000e+00 1.767936e+01
      vertex   1.713438e+01 4.700000e+00 1.767936e+01
      vertex   1.768126e+01 5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -8.336995e-01 0.000000e+00 5.522183e-01
    outer loop
      vertex   1.768126e+01 5.000000e+00 1.850500e+01
      vertex   1.713438e+01 4.700000e+00 1.767936e+01
      vertex   1.768126e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -8.962926e-01 0.000000e+00 4.434632e-01
    outer loop
      vertex   1.643220e+01 5.000000e+00 1.626017e+01
      vertex   1.643220e+01 4.700000e+00 1.626017e+01
      vertex   1.713438e+01 5.000000e+00 1.767936e+01
    endloop
  endfacet
  facet normal -8.962926e-01 0.000000e+00 4.434632e-01
    outer loop
      vertex   1.713438e+01 5.000000e+00 1.767936e+01
      vertex   1.643220e+01 4.700000e+00 1.626017e+01
      vertex   1.713438e+01 4.700000e+00 1.767936e+01
    endloop
  endfacet
  facet normal -9.479951e-01 0.000000e+00 3.182849e-01
    outer loop
      vertex   1.608557e+01 5.000000e+00 1.522775e+01
      vertex   1.643220e+01 4.700000e+00 1.626017e+01
      vertex   1.643220e+01 5.000000e+00 1.626017e+01
    endloop
  endfacet
  facet normal -9.485435e-01 -6.500692e-03 3.165801e-01
    outer loop
      vertex   1.608557e+01 5.000000e+00 1.522775e+01
      vertex   1.605606e+01 4.700000e+00 1.513317e+01
      vertex   1.643220e+01 4.700000e+00 1.626017e+01
    endloop
  endfacet
  facet normal -9.766312e-01 3.234247e-02 2.124747e-01
    outer loop
      vertex   1.583308e+01 5.000000e+00 1.406719e+01
      vertex   1.586566e+01 4.700000e+00 1.426261e+01
      vertex   1.608557e+01 5.000000e+00 1.522775e+01
    endloop
  endfacet
  facet normal -9.765053e-01 2.872545e-02 2.135703e-01
    outer loop
      vertex   1.605606e+01 4.700000e+00 1.513317e+01
      vertex   1.608557e+01 5.000000e+00 1.522775e+01
      vertex   1.586566e+01 4.700000e+00 1.426261e+01
    endloop
  endfacet
  facet normal -9.948301e-01 -5.073508e-02 8.797166e-02
    outer loop
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.586566e+01 4.700000e+00 1.426261e+01
      vertex   1.583308e+01 5.000000e+00 1.406719e+01
    endloop
  endfacet
  facet normal -9.895010e-01 8.697902e-02 1.154230e-01
    outer loop
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.572805e+01 4.700000e+00 1.308290e+01
      vertex   1.586566e+01 4.700000e+00 1.426261e+01
    endloop
  endfacet
  facet normal -9.997547e-01 -2.119094e-02 6.434830e-03
    outer loop
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
      vertex   1.572805e+01 4.700000e+00 1.308290e+01
    endloop
  endfacet
  facet normal -9.978820e-01 6.029022e-02 -2.442355e-02
    outer loop
      vertex   1.574884e+01 5.000000e+00 1.159860e+01
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
      vertex   1.571978e+01 5.000000e+00 1.278593e+01
    endloop
  endfacet
  facet normal -9.865954e-01 -8.888549e-02 -1.368533e-01
    outer loop
      vertex   1.587128e+01 5.000000e+00 1.071591e+01
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
      vertex   1.574884e+01 5.000000e+00 1.159860e+01
    endloop
  endfacet
  facet normal -9.874904e-01 -8.121147e-02 -1.351571e-01
    outer loop
      vertex   1.587128e+01 5.000000e+00 1.071591e+01
      vertex   1.594911e+01 4.700000e+00 1.032753e+01
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
    endloop
  endfacet
  facet normal -9.668059e-01 6.802705e-02 -2.462898e-01
    outer loop
      vertex   1.623773e+01 5.000000e+00 9.277420e+00
      vertex   1.594911e+01 4.700000e+00 1.032753e+01
      vertex   1.587128e+01 5.000000e+00 1.071591e+01
    endloop
  endfacet
  facet normal -9.529904e-01 -9.280184e-02 -2.884390e-01
    outer loop
      vertex   1.623773e+01 5.000000e+00 9.277420e+00
      vertex   1.637897e+01 4.700000e+00 8.907290e+00
      vertex   1.594911e+01 4.700000e+00 1.032753e+01
    endloop
  endfacet
  facet normal -9.183519e-01 5.173165e-02 -3.923693e-01
    outer loop
      vertex   1.685985e+01 5.000000e+00 7.821330e+00
      vertex   1.637897e+01 4.700000e+00 8.907290e+00
      vertex   1.623773e+01 5.000000e+00 9.277420e+00
    endloop
  endfacet
  facet normal -9.183504e-01 5.170201e-02 -3.923768e-01
    outer loop
      vertex   1.685985e+01 5.000000e+00 7.821330e+00
      vertex   1.676795e+01 4.700000e+00 7.996890e+00
      vertex   1.637897e+01 4.700000e+00 8.907290e+00
    endloop
  endfacet
  facet normal -8.622666e-01 -3.165698e-02 -5.054642e-01
    outer loop
      vertex   1.731050e+01 5.000000e+00 7.052570e+00
      vertex   1.676795e+01 4.700000e+00 7.996890e+00
      vertex   1.685985e+01 5.000000e+00 7.821330e+00
    endloop
  endfacet
  facet normal -8.607276e-01 -4.065808e-02 -5.074395e-01
    outer loop
      vertex   1.731050e+01 5.000000e+00 7.052570e+00
      vertex   1.747181e+01 4.700000e+00 6.802990e+00
      vertex   1.676795e+01 4.700000e+00 7.996890e+00
    endloop
  endfacet
  facet normal -8.066807e-01 5.571923e-02 -5.883550e-01
    outer loop
      vertex   1.806883e+01 5.000000e+00 6.012840e+00
      vertex   1.747181e+01 4.700000e+00 6.802990e+00
      vertex   1.731050e+01 5.000000e+00 7.052570e+00
    endloop
  endfacet
  facet normal -7.978587e-01 -0.000000e+00 -6.028445e-01
    outer loop
      vertex   1.806883e+01 5.000000e+00 6.012840e+00
      vertex   1.806883e+01 4.700000e+00 6.012840e+00
      vertex   1.747181e+01 4.700000e+00 6.802990e+00
    endloop
  endfacet
  facet normal 6.428253e-01 -1.402796e-16 -7.660128e-01
    outer loop
      vertex   1.737947e+01 -5.000000e+00 5.434340e+00
      vertex   1.737947e+01 4.700000e+00 5.434340e+00
      vertex   1.806883e+01 4.700000e+00 6.012840e+00
    endloop
  endfacet
  facet normal 6.428296e-01 -5.202833e-07 -7.660092e-01
    outer loop
      vertex   1.806883e+01 4.700000e+00 6.012840e+00
      vertex   1.814542e+01 -5.000000e+00 6.077120e+00
      vertex   1.737947e+01 -5.000000e+00 5.434340e+00
    endloop
  endfacet
  facet normal 6.428683e-01 0.000000e+00 -7.659768e-01
    outer loop
      vertex   1.806883e+01 5.000000e+00 6.012840e+00
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
      vertex   1.806883e+01 4.700000e+00 6.012840e+00
    endloop
  endfacet
  facet normal 6.428683e-01 -1.361902e-16 -7.659768e-01
    outer loop
      vertex   1.806883e+01 4.700000e+00 6.012840e+00
      vertex   1.814542e+01 5.000000e+00 6.077120e+00
      vertex   1.814542e+01 -5.000000e+00 6.077120e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.780591e+01 -5.000000e+00 1.850500e+01
      vertex   1.658650e+01 -5.000000e+00 1.850500e+01
      vertex   1.687372e+01 -5.000000e+00 1.699668e+01
    endloop
  endfacet
  facet normal 8.401946e-16 -1.000000e+00 1.599931e-16
    outer loop
      vertex   1.687372e+01 -5.000000e+00 1.699668e+01
      vertex   1.658650e+01 -5.000000e+00 1.850500e+01
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
    endloop
  endfacet
  facet normal -6.246384e-16 -1.000000e+00 3.437898e-16
    outer loop
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
      vertex   1.658650e+01 -5.000000e+00 1.850500e+01
      vertex   1.584142e+01 -5.000000e+00 1.715125e+01
    endloop
  endfacet
  facet normal -2.814424e-16 -1.000000e+00 3.997999e-16
    outer loop
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
      vertex   1.584142e+01 -5.000000e+00 1.715125e+01
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.616662e+01 -5.000000e+00 1.515862e+01
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.580837e+01 -5.000000e+00 1.242916e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
      vertex   1.584546e+01 -5.000000e+00 1.164120e+01
    endloop
  endfacet
  facet normal 2.744290e-16 -1.000000e+00 3.488148e-16
    outer loop
      vertex   1.584546e+01 -5.000000e+00 1.164120e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
      vertex   1.519743e+01 -5.000000e+00 9.604760e+00
    endloop
  endfacet
  facet normal 6.897685e-16 -1.000000e+00 2.166468e-16
    outer loop
      vertex   1.584546e+01 -5.000000e+00 1.164120e+01
      vertex   1.519743e+01 -5.000000e+00 9.604760e+00
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.652827e+01 -5.000000e+00 6.599050e+00
      vertex   1.632589e+01 -5.000000e+00 9.328990e+00
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
      vertex   1.632589e+01 -5.000000e+00 9.328990e+00
      vertex   1.602479e+01 -5.000000e+00 1.047244e+01
    endloop
  endfacet
  facet normal -1.507361e-15 -1.000000e+00 5.286476e-16
    outer loop
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
      vertex   1.602479e+01 -5.000000e+00 1.047244e+01
      vertex   1.584546e+01 -5.000000e+00 1.164120e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   1.632589e+01 -5.000000e+00 9.328990e+00
      vertex   1.652827e+01 -5.000000e+00 6.599050e+00
      vertex   1.677432e+01 -5.000000e+00 8.234900e+00
    endloop
  endfacet
  facet normal -8.653468e-16 -1.000000e+00 1.301577e-16
    outer loop
      vertex   1.677432e+01 -5.000000e+00 8.234900e+00
      vertex   1.652827e+01 -5.000000e+00 6.599050e+00
      vertex   1.737947e+01 -5.000000e+00 5.434340e+00
    endloop
  endfacet
  facet normal -2.252913e-16 -1.000000e+00 2.684618e-16
    outer loop
      vertex   1.677432e+01 -5.000000e+00 8.234900e+00
      vertex   1.737947e+01 -5.000000e+00 5.434340e+00
      vertex   1.814542e+01 -5.000000e+00 6.077120e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.605606e+01 4.700000e+00 1.513317e+01
      vertex   1.584142e+01 4.700000e+00 1.715125e+01
      vertex   1.643220e+01 4.700000e+00 1.626017e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.643220e+01 4.700000e+00 1.626017e+01
      vertex   1.584142e+01 4.700000e+00 1.715125e+01
      vertex   1.658650e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.643220e+01 4.700000e+00 1.626017e+01
      vertex   1.658650e+01 4.700000e+00 1.850500e+01
      vertex   1.713438e+01 4.700000e+00 1.767936e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.713438e+01 4.700000e+00 1.767936e+01
      vertex   1.658650e+01 4.700000e+00 1.850500e+01
      vertex   1.768126e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.605606e+01 4.700000e+00 1.513317e+01
      vertex   1.586566e+01 4.700000e+00 1.426261e+01
      vertex   1.584142e+01 4.700000e+00 1.715125e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.584142e+01 4.700000e+00 1.715125e+01
      vertex   1.586566e+01 4.700000e+00 1.426261e+01
      vertex   1.522883e+01 4.700000e+00 1.545769e+01
    endloop
  endfacet
  facet normal -2.318246e-15 1.000000e+00 -1.235339e-15
    outer loop
      vertex   1.522883e+01 4.700000e+00 1.545769e+01
      vertex   1.586566e+01 4.700000e+00 1.426261e+01
      vertex   1.572805e+01 4.700000e+00 1.308290e+01
    endloop
  endfacet
  facet normal -6.321326e-16 1.000000e+00 -8.808905e-16
    outer loop
      vertex   1.522883e+01 4.700000e+00 1.545769e+01
      vertex   1.572805e+01 4.700000e+00 1.308290e+01
      vertex   1.488012e+01 4.700000e+00 1.369138e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.488012e+01 4.700000e+00 1.369138e+01
      vertex   1.572805e+01 4.700000e+00 1.308290e+01
      vertex   1.481260e+01 4.700000e+00 1.235415e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.481260e+01 4.700000e+00 1.235415e+01
      vertex   1.572805e+01 4.700000e+00 1.308290e+01
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.481260e+01 4.700000e+00 1.235415e+01
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
      vertex   1.500995e+01 4.700000e+00 1.040787e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.500995e+01 4.700000e+00 1.040787e+01
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
      vertex   1.572104e+01 4.700000e+00 1.199386e+01
      vertex   1.594911e+01 4.700000e+00 1.032753e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
      vertex   1.594911e+01 4.700000e+00 1.032753e+01
      vertex   1.597264e+01 4.700000e+00 7.587090e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.597264e+01 4.700000e+00 7.587090e+00
      vertex   1.594911e+01 4.700000e+00 1.032753e+01
      vertex   1.637897e+01 4.700000e+00 8.907290e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.597264e+01 4.700000e+00 7.587090e+00
      vertex   1.637897e+01 4.700000e+00 8.907290e+00
      vertex   1.676795e+01 4.700000e+00 7.996890e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.597264e+01 4.700000e+00 7.587090e+00
      vertex   1.676795e+01 4.700000e+00 7.996890e+00
      vertex   1.737947e+01 4.700000e+00 5.434340e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   1.737947e+01 4.700000e+00 5.434340e+00
      vertex   1.676795e+01 4.700000e+00 7.996890e+00
      vertex   1.747181e+01 4.700000e+00 6.802990e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.737947e+01 4.700000e+00 5.434340e+00
      vertex   1.747181e+01 4.700000e+00 6.802990e+00
      vertex   1.806883e+01 4.700000e+00 6.012840e+00
    endloop
  endfacet
  facet normal -8.370491e-01 1.112943e-02 -5.470146e-01
    outer loop
      vertex   1.597264e+01 4.700000e+00 7.587090e+00
      vertex   1.737947e+01 4.700000e+00 5.434340e+00
      vertex   1.629877e+01 -7.999998e-01 6.976140e+00
    endloop
  endfacet
  facet normal -8.827697e-01 -1.581141e-04 -4.698059e-01
    outer loop
      vertex   1.591007e+01 -7.999998e-01 7.706510e+00
      vertex   1.597264e+01 4.700000e+00 7.587090e+00
      vertex   1.629877e+01 -7.999998e-01 6.976140e+00
    endloop
  endfacet
  facet normal -9.231429e-01 2.154470e-03 -3.844510e-01
    outer loop
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
      vertex   1.597264e+01 4.700000e+00 7.587090e+00
      vertex   1.591007e+01 -7.999998e-01 7.706510e+00
    endloop
  endfacet
  facet normal -9.243140e-01 1.457118e-03 -3.816301e-01
    outer loop
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
      vertex   1.591007e+01 -7.999998e-01 7.706510e+00
    endloop
  endfacet
  facet normal -9.540477e-01 9.956058e-04 -2.996532e-01
    outer loop
      vertex   1.519743e+01 -5.000000e+00 9.604760e+00
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
    endloop
  endfacet
  facet normal -9.620784e-01 3.231744e-03 -2.727540e-01
    outer loop
      vertex   1.508721e+01 -7.999998e-01 1.004330e+01
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
      vertex   1.519743e+01 -5.000000e+00 9.604760e+00
    endloop
  endfacet
  facet normal -9.633177e-01 4.254384e-03 -2.683299e-01
    outer loop
      vertex   1.500995e+01 4.700000e+00 1.040787e+01
      vertex   1.543824e+01 4.700000e+00 8.870290e+00
      vertex   1.508721e+01 -7.999998e-01 1.004330e+01
    endloop
  endfacet
  facet normal -9.799370e-01 -5.543072e-04 -1.993067e-01
    outer loop
      vertex   1.496351e+01 -7.999998e-01 1.065150e+01
      vertex   1.500995e+01 4.700000e+00 1.040787e+01
      vertex   1.508721e+01 -7.999998e-01 1.004330e+01
    endloop
  endfacet
  facet normal -9.948982e-01 -6.296044e-04 -1.008815e-01
    outer loop
      vertex   1.481260e+01 4.700000e+00 1.235415e+01
      vertex   1.500995e+01 4.700000e+00 1.040787e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
    endloop
  endfacet
  facet normal -9.987272e-01 9.794121e-04 5.042828e-02
    outer loop
      vertex   1.488012e+01 4.700000e+00 1.369138e+01
      vertex   1.481260e+01 4.700000e+00 1.235415e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
    endloop
  endfacet
  facet normal -9.968794e-01 -2.656291e-03 7.889520e-02
    outer loop
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
      vertex   1.488012e+01 4.700000e+00 1.369138e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
    endloop
  endfacet
  facet normal -9.810529e-01 4.731073e-03 1.936823e-01
    outer loop
      vertex   1.522883e+01 4.700000e+00 1.545769e+01
      vertex   1.488012e+01 4.700000e+00 1.369138e+01
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
    endloop
  endfacet
  facet normal -9.699657e-01 -1.469284e-03 2.432372e-01
    outer loop
      vertex   1.536392e+01 -7.999998e-01 1.596317e+01
      vertex   1.522883e+01 4.700000e+00 1.545769e+01
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
    endloop
  endfacet
  facet normal -9.389616e-01 8.545153e-03 3.439159e-01
    outer loop
      vertex   1.557738e+01 -7.999998e-01 1.654596e+01
      vertex   1.522883e+01 4.700000e+00 1.545769e+01
      vertex   1.536392e+01 -7.999998e-01 1.596317e+01
    endloop
  endfacet
  facet normal -9.403437e-01 7.710160e-03 3.401386e-01
    outer loop
      vertex   1.584142e+01 4.700000e+00 1.715125e+01
      vertex   1.522883e+01 4.700000e+00 1.545769e+01
      vertex   1.557738e+01 -7.999998e-01 1.654596e+01
    endloop
  endfacet
  facet normal -8.760745e-01 0.000000e+00 4.821758e-01
    outer loop
      vertex   1.584142e+01 4.700000e+00 1.715125e+01
      vertex   1.584142e+01 -5.000000e+00 1.715125e+01
      vertex   1.658650e+01 4.700000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -8.760745e-01 0.000000e+00 4.821758e-01
    outer loop
      vertex   1.658650e+01 4.700000e+00 1.850500e+01
      vertex   1.584142e+01 -5.000000e+00 1.715125e+01
      vertex   1.658650e+01 -5.000000e+00 1.850500e+01
    endloop
  endfacet
  facet normal -9.165874e-01 0.000000e+00 3.998344e-01
    outer loop
      vertex   1.557738e+01 -7.999998e-01 1.654596e+01
      vertex   1.584142e+01 -5.000000e+00 1.715125e+01
      vertex   1.584142e+01 4.700000e+00 1.715125e+01
    endloop
  endfacet
  facet normal -9.545758e-01 -1.713990e-02 2.974750e-01
    outer loop
      vertex   1.557738e+01 -7.999998e-01 1.654596e+01
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
      vertex   1.584142e+01 -5.000000e+00 1.715125e+01
    endloop
  endfacet
  facet normal -9.380787e-01 -4.418865e-02 3.435925e-01
    outer loop
      vertex   1.536392e+01 -7.999998e-01 1.596317e+01
      vertex   1.495456e+01 -5.000000e+00 1.430538e+01
      vertex   1.557738e+01 -7.999998e-01 1.654596e+01
    endloop
  endfacet
  facet normal -9.972643e-01 5.154245e-03 -7.373778e-02
    outer loop
      vertex   1.496351e+01 -7.999998e-01 1.065150e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
      vertex   1.500995e+01 4.700000e+00 1.040787e+01
    endloop
  endfacet
  facet normal -9.787402e-01 -4.941108e-02 -1.990633e-01
    outer loop
      vertex   1.508721e+01 -7.999998e-01 1.004330e+01
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
      vertex   1.496351e+01 -7.999998e-01 1.065150e+01
    endloop
  endfacet
  facet normal -9.907521e-01 -1.188726e-02 -1.351624e-01
    outer loop
      vertex   1.508721e+01 -7.999998e-01 1.004330e+01
      vertex   1.519743e+01 -5.000000e+00 9.604760e+00
      vertex   1.480829e+01 -5.000000e+00 1.245719e+01
    endloop
  endfacet
  facet normal -8.824604e-01 -2.647254e-02 -4.696413e-01
    outer loop
      vertex   1.629877e+01 -7.999998e-01 6.976140e+00
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
      vertex   1.591007e+01 -7.999998e-01 7.706510e+00
    endloop
  endfacet
  facet normal -8.985154e-01 -9.697567e-03 -4.388348e-01
    outer loop
      vertex   1.629877e+01 -7.999998e-01 6.976140e+00
      vertex   1.652827e+01 -5.000000e+00 6.599050e+00
      vertex   1.544455e+01 -5.000000e+00 8.817970e+00
    endloop
  endfacet
  facet normal -8.274776e-01 5.195167e-03 -5.614746e-01
    outer loop
      vertex   1.737947e+01 4.700000e+00 5.434340e+00
      vertex   1.652827e+01 -5.000000e+00 6.599050e+00
      vertex   1.629877e+01 -7.999998e-01 6.976140e+00
    endloop
  endfacet
  facet normal -8.073697e-01 -1.080549e-16 -5.900459e-01
    outer loop
      vertex   1.737947e+01 -5.000000e+00 5.434340e+00
      vertex   1.652827e+01 -5.000000e+00 6.599050e+00
      vertex   1.737947e+01 4.700000e+00 5.434340e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.009585e+02 9.258500e+00 3.991500e+00
      vertex   -1.009585e+02 9.258500e+00 3.000000e+00
      vertex   -8.994150e+01 9.258500e+00 3.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.994150e+01 9.258500e+00 3.991500e+00
      vertex   -1.009585e+02 9.258500e+00 3.000000e+00
      vertex   -8.994150e+01 9.258500e+00 3.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.994150e+01 9.258500e+00 3.991500e+00
      vertex   -8.994150e+01 9.258500e+00 3.000000e+00
      vertex   -8.994150e+01 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -8.994150e+01 4.241500e+00 3.991500e+00
      vertex   -8.994150e+01 9.258500e+00 3.000000e+00
      vertex   -8.994150e+01 4.241500e+00 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.009585e+02 4.241500e+00 3.991500e+00
      vertex   -1.009585e+02 4.241500e+00 3.000000e+00
      vertex   -1.009585e+02 9.258500e+00 3.991500e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.009585e+02 9.258500e+00 3.991500e+00
      vertex   -1.009585e+02 4.241500e+00 3.000000e+00
      vertex   -1.009585e+02 9.258500e+00 3.000000e+00
    endloop
  endfacet
  facet normal 6.997148e-32 1.770338e-16 1.000000e+00
    outer loop
      vertex   -1.009585e+02 4.241500e+00 3.991500e+00
      vertex   -1.009585e+02 9.258500e+00 3.991500e+00
      vertex   -1.077000e+02 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal -1.734195e-17 2.003367e-16 1.000000e+00
    outer loop
      vertex   -1.077000e+02 4.241500e+00 3.991500e+00
      vertex   -1.009585e+02 9.258500e+00 3.991500e+00
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
    endloop
  endfacet
  facet normal -0.000000e+00 2.114711e-16 1.000000e+00
    outer loop
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
      vertex   -1.009585e+02 9.258500e+00 3.991500e+00
      vertex   -8.394150e+01 1.975850e+01 3.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 2.114711e-16 1.000000e+00
    outer loop
      vertex   -8.394150e+01 1.975850e+01 3.991500e+00
      vertex   -1.009585e+02 9.258500e+00 3.991500e+00
      vertex   -8.994150e+01 9.258500e+00 3.991500e+00
    endloop
  endfacet
  facet normal 1.948513e-17 2.003367e-16 1.000000e+00
    outer loop
      vertex   -8.394150e+01 1.975850e+01 3.991500e+00
      vertex   -8.994150e+01 9.258500e+00 3.991500e+00
      vertex   -8.394150e+01 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal -7.861879e-32 1.770338e-16 1.000000e+00
    outer loop
      vertex   -8.394150e+01 4.241500e+00 3.991500e+00
      vertex   -8.994150e+01 9.258500e+00 3.991500e+00
      vertex   -8.994150e+01 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.394150e+01 1.975850e+01 5.500000e+00
      vertex   -8.394150e+01 1.975850e+01 3.991500e+00
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
      vertex   -8.394150e+01 1.975850e+01 3.991500e+00
      vertex   -8.394150e+01 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal 3.952437e-16 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.077000e+02 4.241500e+00 3.991500e+00
      vertex   -1.077000e+02 4.241500e+00 5.500000e+00
      vertex   -1.009585e+02 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -1.766348e-15
    outer loop
      vertex   -1.009585e+02 4.241500e+00 3.991500e+00
      vertex   -1.077000e+02 4.241500e+00 5.500000e+00
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -1.766348e-15
    outer loop
      vertex   -1.009585e+02 4.241500e+00 3.991500e+00
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
      vertex   -8.994150e+01 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal -4.440892e-16 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.994150e+01 4.241500e+00 3.991500e+00
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
      vertex   -8.394150e+01 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.994150e+01 4.241500e+00 3.991500e+00
      vertex   -8.994150e+01 4.241500e+00 3.000000e+00
      vertex   -1.009585e+02 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.009585e+02 4.241500e+00 3.991500e+00
      vertex   -8.994150e+01 4.241500e+00 3.000000e+00
      vertex   -1.009585e+02 4.241500e+00 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 1.975850e+01 5.500000e+00
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
      vertex   -8.394150e+01 1.975850e+01 5.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.394150e+01 1.975850e+01 5.500000e+00
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
      vertex   -8.394150e+01 1.975850e+01 3.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.077000e+02 1.975850e+01 5.500000e+00
      vertex   -8.394150e+01 1.975850e+01 5.500000e+00
      vertex   -1.077000e+02 2.100000e+01 5.500000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.077000e+02 2.100000e+01 5.500000e+00
      vertex   -8.394150e+01 1.975850e+01 5.500000e+00
      vertex   -8.195000e+01 2.100000e+01 5.500000e+00
    endloop
  endfacet
  facet normal -9.228166e-17 1.480297e-16 1.000000e+00
    outer loop
      vertex   -8.195000e+01 2.100000e+01 5.500000e+00
      vertex   -8.394150e+01 1.975850e+01 5.500000e+00
      vertex   -8.195000e+01 3.000000e+00 5.500000e+00
    endloop
  endfacet
  facet normal 1.070484e-16 1.717172e-16 1.000000e+00
    outer loop
      vertex   -8.195000e+01 3.000000e+00 5.500000e+00
      vertex   -8.394150e+01 1.975850e+01 5.500000e+00
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -8.195000e+01 3.000000e+00 5.500000e+00
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
      vertex   -1.077000e+02 3.000000e+00 5.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.077000e+02 3.000000e+00 5.500000e+00
      vertex   -8.394150e+01 4.241500e+00 5.500000e+00
      vertex   -1.077000e+02 4.241500e+00 5.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.195000e+01 2.100000e+01 5.500000e+00
      vertex   -8.195000e+01 2.100000e+01 3.000000e+00
      vertex   -1.077000e+02 2.100000e+01 5.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 2.100000e+01 5.500000e+00
      vertex   -8.195000e+01 2.100000e+01 3.000000e+00
      vertex   -1.077000e+02 2.100000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.195000e+01 3.000000e+00 3.000000e+00
      vertex   -8.195000e+01 3.000000e+00 5.500000e+00
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
      vertex   -8.195000e+01 3.000000e+00 5.500000e+00
      vertex   -1.077000e+02 3.000000e+00 5.500000e+00
    endloop
  endfacet
  facet normal -9.914574e-17 -1.770338e-16 -1.000000e+00
    outer loop
      vertex   -1.009585e+02 9.258500e+00 3.000000e+00
      vertex   -1.009585e+02 4.241500e+00 3.000000e+00
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -7.154075e-16 -1.000000e+00
    outer loop
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
      vertex   -1.009585e+02 4.241500e+00 3.000000e+00
      vertex   -8.994150e+01 4.241500e+00 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -7.154075e-16 -1.000000e+00
    outer loop
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
      vertex   -8.994150e+01 4.241500e+00 3.000000e+00
      vertex   -8.195000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 8.363774e-17 -1.770338e-16 -1.000000e+00
    outer loop
      vertex   -8.195000e+01 3.000000e+00 3.000000e+00
      vertex   -8.994150e+01 4.241500e+00 3.000000e+00
      vertex   -8.994150e+01 9.258500e+00 3.000000e+00
    endloop
  endfacet
  facet normal 4.838775e-17 -2.220446e-16 -1.000000e+00
    outer loop
      vertex   -8.195000e+01 3.000000e+00 3.000000e+00
      vertex   -8.994150e+01 9.258500e+00 3.000000e+00
      vertex   -8.195000e+01 2.100000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.891109e-16 -1.000000e+00
    outer loop
      vertex   -8.195000e+01 2.100000e+01 3.000000e+00
      vertex   -8.994150e+01 9.258500e+00 3.000000e+00
      vertex   -1.077000e+02 2.100000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.891109e-16 -1.000000e+00
    outer loop
      vertex   -1.077000e+02 2.100000e+01 3.000000e+00
      vertex   -8.994150e+01 9.258500e+00 3.000000e+00
      vertex   -1.009585e+02 9.258500e+00 3.000000e+00
    endloop
  endfacet
  facet normal -5.735974e-17 -2.220446e-16 -1.000000e+00
    outer loop
      vertex   -1.077000e+02 2.100000e+01 3.000000e+00
      vertex   -1.009585e+02 9.258500e+00 3.000000e+00
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 1.975850e+01 5.500000e+00
      vertex   -1.077000e+02 2.100000e+01 5.500000e+00
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
      vertex   -1.077000e+02 2.100000e+01 5.500000e+00
      vertex   -1.077000e+02 2.100000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
      vertex   -1.077000e+02 2.100000e+01 3.000000e+00
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 1.975850e+01 3.991500e+00
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
      vertex   -1.077000e+02 4.241500e+00 3.991500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 4.241500e+00 3.991500e+00
      vertex   -1.077000e+02 3.000000e+00 3.000000e+00
      vertex   -1.077000e+02 3.000000e+00 5.500000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.077000e+02 4.241500e+00 3.991500e+00
      vertex   -1.077000e+02 3.000000e+00 5.500000e+00
      vertex   -1.077000e+02 4.241500e+00 5.500000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.195000e+01 2.100000e+01 3.000000e+00
      vertex   -8.195000e+01 2.100000e+01 5.500000e+00
      vertex   -8.195000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -8.195000e+01 3.000000e+00 3.000000e+00
      vertex   -8.195000e+01 2.100000e+01 5.500000e+00
      vertex   -8.195000e+01 3.000000e+00 5.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.270850e+01 1.525850e+01 4.991500e+00
      vertex   -5.270850e+01 1.525850e+01 3.000000e+00
      vertex   -4.669150e+01 1.525850e+01 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -4.669150e+01 1.525850e+01 4.991500e+00
      vertex   -5.270850e+01 1.525850e+01 3.000000e+00
      vertex   -4.669150e+01 1.525850e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -1.622156e-08 -5.957099e-08
    outer loop
      vertex   -4.669150e+01 1.525850e+01 4.991500e+00
      vertex   -4.669150e+01 1.525850e+01 3.000000e+00
      vertex   -4.669150e+01 4.241500e+00 4.991500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 7.238656e-24 3.016688e-08
    outer loop
      vertex   -4.669150e+01 4.241500e+00 4.991500e+00
      vertex   -4.669150e+01 1.525850e+01 3.000000e+00
      vertex   -4.669150e+01 4.241500e+00 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -6.449512e-16 5.957099e-08
    outer loop
      vertex   -5.270850e+01 4.241500e+00 4.991500e+00
      vertex   -5.270850e+01 4.241500e+00 3.000000e+00
      vertex   -5.270850e+01 1.525850e+01 4.991500e+00
    endloop
  endfacet
  facet normal 1.000000e+00 1.447731e-23 5.957098e-08
    outer loop
      vertex   -5.270850e+01 1.525850e+01 4.991500e+00
      vertex   -5.270850e+01 4.241500e+00 3.000000e+00
      vertex   -5.270850e+01 1.525850e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 2.418567e-16 1.000000e+00
    outer loop
      vertex   -5.270850e+01 4.241500e+00 4.991500e+00
      vertex   -5.270850e+01 1.525850e+01 4.991500e+00
      vertex   -7.570850e+01 4.241500e+00 4.991500e+00
    endloop
  endfacet
  facet normal 6.179329e-18 2.289562e-16 1.000000e+00
    outer loop
      vertex   -7.570850e+01 4.241500e+00 4.991500e+00
      vertex   -5.270850e+01 1.525850e+01 4.991500e+00
      vertex   -7.570850e+01 1.975850e+01 4.991500e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.973730e-16 1.000000e+00
    outer loop
      vertex   -7.570850e+01 1.975850e+01 4.991500e+00
      vertex   -5.270850e+01 1.525850e+01 4.991500e+00
      vertex   -4.669150e+01 1.975850e+01 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.973730e-16 1.000000e+00
    outer loop
      vertex   -4.669150e+01 1.975850e+01 4.991500e+00
      vertex   -5.270850e+01 1.525850e+01 4.991500e+00
      vertex   -4.669150e+01 1.525850e+01 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -5.887825e-16
    outer loop
      vertex   -7.570850e+01 4.241500e+00 4.991500e+00
      vertex   -7.570850e+01 4.241500e+00 6.500000e+00
      vertex   -5.270850e+01 4.241500e+00 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -5.887825e-16
    outer loop
      vertex   -5.270850e+01 4.241500e+00 4.991500e+00
      vertex   -7.570850e+01 4.241500e+00 6.500000e+00
      vertex   -4.669150e+01 4.241500e+00 6.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -5.887825e-16
    outer loop
      vertex   -5.270850e+01 4.241500e+00 4.991500e+00
      vertex   -4.669150e+01 4.241500e+00 6.500000e+00
      vertex   -4.669150e+01 4.241500e+00 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 4.459846e-16
    outer loop
      vertex   -4.669150e+01 4.241500e+00 3.000000e+00
      vertex   -5.270850e+01 4.241500e+00 3.000000e+00
      vertex   -4.669150e+01 4.241500e+00 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 4.459846e-16
    outer loop
      vertex   -4.669150e+01 4.241500e+00 4.991500e+00
      vertex   -5.270850e+01 4.241500e+00 3.000000e+00
      vertex   -5.270850e+01 4.241500e+00 4.991500e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.570850e+01 4.241500e+00 6.500000e+00
      vertex   -7.570850e+01 4.241500e+00 4.991500e+00
      vertex   -7.570850e+01 1.975850e+01 6.500000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.570850e+01 1.975850e+01 6.500000e+00
      vertex   -7.570850e+01 4.241500e+00 4.991500e+00
      vertex   -7.570850e+01 1.975850e+01 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.570850e+01 1.975850e+01 6.500000e+00
      vertex   -7.570850e+01 1.975850e+01 4.991500e+00
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
      vertex   -7.570850e+01 1.975850e+01 4.991500e+00
      vertex   -4.669150e+01 1.975850e+01 4.991500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -1.622156e-08 9.550973e-24
    outer loop
      vertex   -4.669150e+01 4.241500e+00 4.991500e+00
      vertex   -4.669150e+01 4.241500e+00 6.500000e+00
      vertex   -4.669150e+01 1.525850e+01 4.991500e+00
    endloop
  endfacet
  facet normal -1.000000e+00 3.392489e-23 1.184706e-07
    outer loop
      vertex   -4.669150e+01 1.525850e+01 4.991500e+00
      vertex   -4.669150e+01 4.241500e+00 6.500000e+00
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 3.971399e-08 0.000000e+00
    outer loop
      vertex   -4.669150e+01 1.525850e+01 4.991500e+00
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
      vertex   -4.669150e+01 1.975850e+01 4.991500e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 3.806479e-16
    outer loop
      vertex   -4.470000e+01 3.000000e+00 3.000000e+00
      vertex   -4.470000e+01 3.000000e+00 6.500000e+00
      vertex   -7.770000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 3.806479e-16
    outer loop
      vertex   -7.770000e+01 3.000000e+00 3.000000e+00
      vertex   -4.470000e+01 3.000000e+00 6.500000e+00
      vertex   -7.770000e+01 3.000000e+00 6.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -4.470000e+01 2.100000e+01 6.500000e+00
      vertex   -4.470000e+01 2.100000e+01 3.000000e+00
      vertex   -7.770000e+01 2.100000e+01 6.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.770000e+01 2.100000e+01 6.500000e+00
      vertex   -4.470000e+01 2.100000e+01 3.000000e+00
      vertex   -7.770000e+01 2.100000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -7.154075e-16 -1.000000e+00
    outer loop
      vertex   -5.270850e+01 4.241500e+00 3.000000e+00
      vertex   -4.669150e+01 4.241500e+00 3.000000e+00
      vertex   -4.470000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 2.952113e-16 -2.418567e-16 -1.000000e+00
    outer loop
      vertex   -4.470000e+01 3.000000e+00 3.000000e+00
      vertex   -4.669150e+01 4.241500e+00 3.000000e+00
      vertex   -4.669150e+01 1.525850e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.652989e-16 -2.467162e-16 -1.000000e+00
    outer loop
      vertex   -4.470000e+01 3.000000e+00 3.000000e+00
      vertex   -4.669150e+01 1.525850e+01 3.000000e+00
      vertex   -4.470000e+01 2.100000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.546945e-16 -1.000000e+00
    outer loop
      vertex   -4.470000e+01 2.100000e+01 3.000000e+00
      vertex   -4.669150e+01 1.525850e+01 3.000000e+00
      vertex   -5.270850e+01 1.525850e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.546945e-16 -1.000000e+00
    outer loop
      vertex   -4.470000e+01 2.100000e+01 3.000000e+00
      vertex   -5.270850e+01 1.525850e+01 3.000000e+00
      vertex   -7.770000e+01 2.100000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.114090e-17 -2.467162e-16 -1.000000e+00
    outer loop
      vertex   -7.770000e+01 2.100000e+01 3.000000e+00
      vertex   -5.270850e+01 1.525850e+01 3.000000e+00
      vertex   -7.770000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal -2.352453e-17 -2.418567e-16 -1.000000e+00
    outer loop
      vertex   -7.770000e+01 3.000000e+00 3.000000e+00
      vertex   -5.270850e+01 1.525850e+01 3.000000e+00
      vertex   -5.270850e+01 4.241500e+00 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -7.154075e-16 -1.000000e+00
    outer loop
      vertex   -7.770000e+01 3.000000e+00 3.000000e+00
      vertex   -5.270850e+01 4.241500e+00 3.000000e+00
      vertex   -4.470000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.770000e+01 2.100000e+01 6.500000e+00
      vertex   -7.770000e+01 2.100000e+01 3.000000e+00
      vertex   -7.770000e+01 3.000000e+00 6.500000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.770000e+01 3.000000e+00 6.500000e+00
      vertex   -7.770000e+01 2.100000e+01 3.000000e+00
      vertex   -7.770000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
      vertex   -7.770000e+01 2.100000e+01 6.500000e+00
      vertex   -7.570850e+01 1.975850e+01 6.500000e+00
    endloop
  endfacet
  facet normal 1.538028e-16 2.467162e-16 1.000000e+00
    outer loop
      vertex   -7.570850e+01 1.975850e+01 6.500000e+00
      vertex   -7.770000e+01 2.100000e+01 6.500000e+00
      vertex   -7.770000e+01 3.000000e+00 6.500000e+00
    endloop
  endfacet
  facet normal -1.784140e-16 2.861953e-16 1.000000e+00
    outer loop
      vertex   -7.570850e+01 1.975850e+01 6.500000e+00
      vertex   -7.770000e+01 3.000000e+00 6.500000e+00
      vertex   -7.570850e+01 4.241500e+00 6.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.570850e+01 4.241500e+00 6.500000e+00
      vertex   -7.770000e+01 3.000000e+00 6.500000e+00
      vertex   -4.470000e+01 3.000000e+00 6.500000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.570850e+01 4.241500e+00 6.500000e+00
      vertex   -4.470000e+01 3.000000e+00 6.500000e+00
      vertex   -4.669150e+01 4.241500e+00 6.500000e+00
    endloop
  endfacet
  facet normal 1.784140e-16 2.861953e-16 1.000000e+00
    outer loop
      vertex   -4.669150e+01 4.241500e+00 6.500000e+00
      vertex   -4.470000e+01 3.000000e+00 6.500000e+00
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
    endloop
  endfacet
  facet normal -1.538028e-16 2.467162e-16 1.000000e+00
    outer loop
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
      vertex   -4.470000e+01 3.000000e+00 6.500000e+00
      vertex   -4.470000e+01 2.100000e+01 6.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -4.669150e+01 1.975850e+01 6.500000e+00
      vertex   -4.470000e+01 2.100000e+01 6.500000e+00
      vertex   -7.770000e+01 2.100000e+01 6.500000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -4.470000e+01 2.100000e+01 3.000000e+00
      vertex   -4.470000e+01 2.100000e+01 6.500000e+00
      vertex   -4.470000e+01 3.000000e+00 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -4.470000e+01 3.000000e+00 3.000000e+00
      vertex   -4.470000e+01 2.100000e+01 6.500000e+00
      vertex   -4.470000e+01 3.000000e+00 6.500000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 4.000000e+00
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
      vertex   -5.708497e+00 1.700000e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 1.700000e+01 4.000000e+00
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
      vertex   -5.708497e+00 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 1.700000e+01 2.000000e+01
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
      vertex   -7.699997e+00 1.700000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 1.700000e+01 2.000000e+01
      vertex   -7.699997e+00 1.700000e+01 1.850000e+01
      vertex   -7.699997e+00 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 2.000000e+01
      vertex   -7.699997e+00 1.700000e+01 1.850000e+01
      vertex   -9.699997e+00 1.700000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 4.000000e+00 4.000000e+00
      vertex   -5.699997e+00 4.000000e+00 4.000000e+00
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
      vertex   -5.699997e+00 4.000000e+00 4.000000e+00
      vertex   -5.699997e+00 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
      vertex   -5.699997e+00 4.000000e+00 8.000000e+00
      vertex   -7.699997e+00 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 4.000000e+00 4.000000e+00
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
      vertex   -3.270000e+01 4.000000e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.270000e+01 4.000000e+00 4.000000e+00
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
      vertex   -3.270000e+01 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.270000e+01 4.000000e+00 8.000000e+00
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
      vertex   -3.070000e+01 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.169150e+01 1.100000e+01 4.000000e+00
      vertex   -1.183920e+01 1.030509e+01 4.000000e+00
      vertex   -1.183920e+01 1.169491e+01 4.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.183920e+01 1.169491e+01 4.000000e+00
      vertex   -1.183920e+01 1.030509e+01 4.000000e+00
      vertex   -1.225679e+01 1.226966e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.225679e+01 1.226966e+01 4.000000e+00
      vertex   -1.183920e+01 1.030509e+01 4.000000e+00
      vertex   -1.225679e+01 9.730337e+00 4.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.225679e+01 1.226966e+01 4.000000e+00
      vertex   -1.225679e+01 9.730337e+00 4.000000e+00
      vertex   -1.287204e+01 1.262488e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.287204e+01 1.262488e+01 4.000000e+00
      vertex   -1.225679e+01 9.730337e+00 4.000000e+00
      vertex   -1.287204e+01 9.375120e+00 4.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.287204e+01 1.262488e+01 4.000000e+00
      vertex   -1.287204e+01 9.375120e+00 4.000000e+00
      vertex   -1.357858e+01 1.269914e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.357858e+01 1.269914e+01 4.000000e+00
      vertex   -1.287204e+01 9.375120e+00 4.000000e+00
      vertex   -1.357858e+01 9.300859e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.357858e+01 1.269914e+01 4.000000e+00
      vertex   -1.357858e+01 9.300859e+00 4.000000e+00
      vertex   -1.425425e+01 1.247960e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.425425e+01 1.247960e+01 4.000000e+00
      vertex   -1.357858e+01 9.300859e+00 4.000000e+00
      vertex   -1.425425e+01 9.520396e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.425425e+01 1.247960e+01 4.000000e+00
      vertex   -1.425425e+01 9.520396e+00 4.000000e+00
      vertex   -1.478220e+01 1.200423e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.478220e+01 1.200423e+01 4.000000e+00
      vertex   -1.425425e+01 9.520396e+00 4.000000e+00
      vertex   -1.478220e+01 9.995769e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.478220e+01 1.200423e+01 4.000000e+00
      vertex   -1.478220e+01 9.995769e+00 4.000000e+00
      vertex   -1.507116e+01 1.135522e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.507116e+01 1.135522e+01 4.000000e+00
      vertex   -1.478220e+01 9.995769e+00 4.000000e+00
      vertex   -1.507116e+01 1.064478e+01 4.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.329150e+01 1.100000e+01 4.000000e+00
      vertex   -2.343920e+01 1.030509e+01 4.000000e+00
      vertex   -2.343920e+01 1.169491e+01 4.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.343920e+01 1.169491e+01 4.000000e+00
      vertex   -2.343920e+01 1.030509e+01 4.000000e+00
      vertex   -2.385679e+01 1.226966e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.385679e+01 1.226966e+01 4.000000e+00
      vertex   -2.343920e+01 1.030509e+01 4.000000e+00
      vertex   -2.385679e+01 9.730337e+00 4.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.385679e+01 1.226966e+01 4.000000e+00
      vertex   -2.385679e+01 9.730337e+00 4.000000e+00
      vertex   -2.447204e+01 1.262488e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.447204e+01 1.262488e+01 4.000000e+00
      vertex   -2.385679e+01 9.730337e+00 4.000000e+00
      vertex   -2.447204e+01 9.375120e+00 4.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.447204e+01 1.262488e+01 4.000000e+00
      vertex   -2.447204e+01 9.375120e+00 4.000000e+00
      vertex   -2.517858e+01 1.269914e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.517858e+01 1.269914e+01 4.000000e+00
      vertex   -2.447204e+01 9.375120e+00 4.000000e+00
      vertex   -2.517858e+01 9.300859e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.517858e+01 1.269914e+01 4.000000e+00
      vertex   -2.517858e+01 9.300859e+00 4.000000e+00
      vertex   -2.585425e+01 1.247960e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.585425e+01 1.247960e+01 4.000000e+00
      vertex   -2.517858e+01 9.300859e+00 4.000000e+00
      vertex   -2.585425e+01 9.520396e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.585425e+01 1.247960e+01 4.000000e+00
      vertex   -2.585425e+01 9.520396e+00 4.000000e+00
      vertex   -2.638220e+01 1.200423e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.638220e+01 1.200423e+01 4.000000e+00
      vertex   -2.585425e+01 9.520396e+00 4.000000e+00
      vertex   -2.638220e+01 9.995769e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.638220e+01 1.200423e+01 4.000000e+00
      vertex   -2.638220e+01 9.995769e+00 4.000000e+00
      vertex   -2.667116e+01 1.135522e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.667116e+01 1.135522e+01 4.000000e+00
      vertex   -2.638220e+01 9.995769e+00 4.000000e+00
      vertex   -2.667116e+01 1.064478e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -5.699997e+00 4.000000e+00 4.000000e+00
      vertex   -7.699997e+00 4.000000e+00 4.000000e+00
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal 3.416071e-17 -3.416071e-17 -1.000000e+00
    outer loop
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
      vertex   -7.699997e+00 4.000000e+00 4.000000e+00
      vertex   -7.699997e+00 1.700000e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -4.034057e-17 -1.000000e+00
    outer loop
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
      vertex   -7.699997e+00 1.700000e+01 4.000000e+00
      vertex   -5.708497e+00 1.700000e+01 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
      vertex   -5.699997e+00 5.991500e+00 4.000000e+00
      vertex   -5.699997e+00 4.000000e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -4.034057e-17 -1.000000e+00
    outer loop
      vertex   -3.269150e+01 1.700000e+01 4.000000e+00
      vertex   -3.070000e+01 1.700000e+01 4.000000e+00
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal -3.416071e-17 -3.416071e-17 -1.000000e+00
    outer loop
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
      vertex   -3.070000e+01 1.700000e+01 4.000000e+00
      vertex   -3.070000e+01 4.000000e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
      vertex   -3.070000e+01 4.000000e+00 4.000000e+00
      vertex   -3.270000e+01 4.000000e+00 4.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -3.270000e+01 4.000000e+00 4.000000e+00
      vertex   -3.270000e+01 5.991500e+00 4.000000e+00
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.269150e+01 1.700000e+01 2.000000e+01
      vertex   -3.269150e+01 1.700000e+01 4.000000e+00
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
      vertex   -3.269150e+01 1.700000e+01 4.000000e+00
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
      vertex   -3.269150e+01 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.269150e+01 7.500000e+00 6.000000e+00
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
      vertex   -3.269150e+01 5.991500e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.269150e+01 7.500000e+00 6.000000e+00
      vertex   -3.269150e+01 5.991500e+00 8.000000e+00
      vertex   -3.269150e+01 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
      vertex   -3.269150e+01 1.000000e+01 2.000000e+01
      vertex   -3.269150e+01 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -2.220446e-16
    outer loop
      vertex   -3.270000e+01 5.991500e+00 4.000000e+00
      vertex   -3.270000e+01 5.991500e+00 8.000000e+00
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal -2.320179e-29 1.000000e+00 -2.220446e-16
    outer loop
      vertex   -3.269150e+01 5.991500e+00 4.000000e+00
      vertex   -3.270000e+01 5.991500e+00 8.000000e+00
      vertex   -3.269150e+01 5.991500e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 4.000000e+00
      vertex   -3.269150e+01 1.700000e+01 4.000000e+00
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
      vertex   -3.269150e+01 1.700000e+01 4.000000e+00
      vertex   -3.269150e+01 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
      vertex   -3.269150e+01 1.700000e+01 2.000000e+01
      vertex   -3.070000e+01 1.700000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 1.850000e+01
      vertex   -3.269150e+01 1.700000e+01 2.000000e+01
      vertex   -3.070000e+01 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 1.850000e+01
      vertex   -3.070000e+01 1.700000e+01 2.000000e+01
      vertex   -2.870000e+01 1.700000e+01 1.850000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.269150e+01 1.700000e+01 2.000000e+01
      vertex   -3.269150e+01 1.000000e+01 2.000000e+01
      vertex   -3.070000e+01 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 2.000000e+01
      vertex   -3.269150e+01 1.000000e+01 2.000000e+01
      vertex   -3.070000e+01 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.000000e+01 2.000000e+01
      vertex   -3.269150e+01 1.000000e+01 2.000000e+01
      vertex   -3.070000e+01 1.000000e+01 1.850000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.000000e+01 1.850000e+01
      vertex   -3.269150e+01 1.000000e+01 2.000000e+01
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.000000e+01 1.850000e+01
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
      vertex   -3.070000e+01 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.000000e+01 1.850000e+01
      vertex   -2.870000e+01 1.000000e+01 1.850000e+01
      vertex   -3.070000e+01 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -2.220446e-16
    outer loop
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
      vertex   -5.699997e+00 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal 2.320179e-29 1.000000e+00 -2.220446e-16
    outer loop
      vertex   -5.699997e+00 5.991500e+00 4.000000e+00
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
      vertex   -5.699997e+00 5.991500e+00 8.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -1.613623e-16 3.234104e-16
    outer loop
      vertex   -5.708497e+00 1.700000e+01 4.000000e+00
      vertex   -5.708497e+00 1.000000e+01 6.000000e+00
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
      vertex   -5.708497e+00 1.000000e+01 6.000000e+00
      vertex   -5.708497e+00 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 5.991500e+00 4.000000e+00
      vertex   -5.708497e+00 7.500000e+00 6.000000e+00
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
      vertex   -5.708497e+00 7.500000e+00 6.000000e+00
      vertex   -5.708497e+00 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -2.220446e-16 1.110223e-16
    outer loop
      vertex   -5.708497e+00 1.700000e+01 4.000000e+00
      vertex   -5.708497e+00 1.700000e+01 2.000000e+01
      vertex   -5.708497e+00 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 1.000000e+01 6.000000e+00
      vertex   -5.708497e+00 1.700000e+01 2.000000e+01
      vertex   -5.708497e+00 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.699997e+00 5.991500e+00 8.000000e+00
      vertex   -5.699997e+00 4.000000e+00 8.000000e+00
      vertex   -5.699997e+00 5.991500e+00 4.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.699997e+00 5.991500e+00 4.000000e+00
      vertex   -5.699997e+00 4.000000e+00 8.000000e+00
      vertex   -5.699997e+00 4.000000e+00 4.000000e+00
    endloop
  endfacet
  facet normal -1.044916e-13 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.699997e+00 5.991500e+00 8.000000e+00
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
      vertex   -5.699997e+00 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 4.459846e-16 1.000000e+00
    outer loop
      vertex   -5.699997e+00 4.000000e+00 8.000000e+00
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
      vertex   -7.699997e+00 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 1.922194e-16 2.537653e-16 1.000000e+00
    outer loop
      vertex   -7.699997e+00 4.000000e+00 8.000000e+00
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
      vertex   -7.699997e+00 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.699997e+00 7.500000e+00 8.000000e+00
      vertex   -5.708497e+00 5.991500e+00 8.000000e+00
      vertex   -5.708497e+00 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.269150e+01 7.500000e+00 8.000000e+00
      vertex   -3.070000e+01 7.500000e+00 8.000000e+00
      vertex   -3.269150e+01 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.269150e+01 7.500000e+00 6.000000e+00
      vertex   -3.070000e+01 7.500000e+00 8.000000e+00
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -5.708497e+00 7.500000e+00 6.000000e+00
      vertex   -7.699997e+00 7.500000e+00 6.000000e+00
      vertex   -5.708497e+00 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.708497e+00 7.500000e+00 8.000000e+00
      vertex   -7.699997e+00 7.500000e+00 6.000000e+00
      vertex   -7.699997e+00 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
      vertex   -3.070000e+01 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 4.000000e+00 8.000000e+00
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
      vertex   -3.070000e+01 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 7.500000e+00 6.000000e+00
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
      vertex   -7.699997e+00 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 7.500000e+00 8.000000e+00
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
      vertex   -7.699997e+00 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.270000e+01 5.991500e+00 4.000000e+00
      vertex   -3.270000e+01 4.000000e+00 4.000000e+00
      vertex   -3.270000e+01 5.991500e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.270000e+01 5.991500e+00 8.000000e+00
      vertex   -3.270000e+01 4.000000e+00 4.000000e+00
      vertex   -3.270000e+01 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 1.850000e+01
      vertex   -7.699997e+00 1.000000e+01 1.850000e+01
      vertex   -9.699997e+00 1.700000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -9.699997e+00 1.700000e+01 1.850000e+01
      vertex   -7.699997e+00 1.000000e+01 1.850000e+01
      vertex   -9.699997e+00 1.000000e+01 1.850000e+01
    endloop
  endfacet
  facet normal -6.000000e-01 0.000000e+00 8.000000e-01
    outer loop
      vertex   -9.699997e+00 1.700000e+01 1.850000e+01
      vertex   -9.699997e+00 1.000000e+01 1.850000e+01
      vertex   -7.699997e+00 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal -6.000000e-01 0.000000e+00 8.000000e-01
    outer loop
      vertex   -7.699997e+00 1.700000e+01 2.000000e+01
      vertex   -9.699997e+00 1.000000e+01 1.850000e+01
      vertex   -7.699997e+00 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.870000e+01 1.700000e+01 1.850000e+01
      vertex   -2.870000e+01 1.000000e+01 1.850000e+01
      vertex   -3.070000e+01 1.700000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 1.850000e+01
      vertex   -2.870000e+01 1.000000e+01 1.850000e+01
      vertex   -3.070000e+01 1.000000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 6.000000e-01 -0.000000e+00 8.000000e-01
    outer loop
      vertex   -2.870000e+01 1.000000e+01 1.850000e+01
      vertex   -2.870000e+01 1.700000e+01 1.850000e+01
      vertex   -3.070000e+01 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 6.000000e-01 -0.000000e+00 8.000000e-01
    outer loop
      vertex   -3.070000e+01 1.000000e+01 2.000000e+01
      vertex   -2.870000e+01 1.700000e+01 1.850000e+01
      vertex   -3.070000e+01 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 1.044916e-13 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.270000e+01 5.991500e+00 8.000000e+00
      vertex   -3.270000e+01 4.000000e+00 8.000000e+00
      vertex   -3.269150e+01 5.991500e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 4.459846e-16 1.000000e+00
    outer loop
      vertex   -3.269150e+01 5.991500e+00 8.000000e+00
      vertex   -3.270000e+01 4.000000e+00 8.000000e+00
      vertex   -3.070000e+01 4.000000e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.922194e-16 2.537653e-16 1.000000e+00
    outer loop
      vertex   -3.269150e+01 5.991500e+00 8.000000e+00
      vertex   -3.070000e+01 4.000000e+00 8.000000e+00
      vertex   -3.070000e+01 7.500000e+00 8.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.070000e+01 7.500000e+00 8.000000e+00
      vertex   -3.269150e+01 7.500000e+00 8.000000e+00
      vertex   -3.269150e+01 5.991500e+00 8.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
      vertex   -7.699997e+00 1.700000e+01 1.850000e+01
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 1.850000e+01
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
      vertex   -7.699997e+00 1.000000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
      vertex   -5.708497e+00 1.000000e+01 6.000000e+00
      vertex   -7.699997e+00 1.000000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 1.850000e+01
      vertex   -5.708497e+00 1.000000e+01 6.000000e+00
      vertex   -5.708497e+00 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 1.850000e+01
      vertex   -5.708497e+00 1.000000e+01 2.000000e+01
      vertex   -7.699997e+00 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 2.000000e+01
      vertex   -9.699997e+00 1.000000e+01 1.850000e+01
      vertex   -7.699997e+00 1.000000e+01 1.850000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.708497e+00 1.000000e+01 2.000000e+01
      vertex   -5.708497e+00 1.700000e+01 2.000000e+01
      vertex   -7.699997e+00 1.000000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 2.000000e+01
      vertex   -5.708497e+00 1.700000e+01 2.000000e+01
      vertex   -7.699997e+00 1.700000e+01 2.000000e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 1.850000e+01
      vertex   -3.070000e+01 1.000000e+01 1.850000e+01
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
      vertex   -3.070000e+01 1.000000e+01 1.850000e+01
      vertex   -3.070000e+01 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
      vertex   -7.699997e+00 1.700000e+01 4.000000e+00
      vertex   -7.699997e+00 2.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+01 6.000000e+00
      vertex   -7.699997e+00 1.700000e+01 4.000000e+00
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
      vertex   -7.699997e+00 1.700000e+01 4.000000e+00
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -7.699997e+00 1.700000e+01 4.000000e+00
      vertex   -7.699997e+00 4.000000e+00 4.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -7.699997e+00 4.000000e+00 4.000000e+00
      vertex   -7.699997e+00 2.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+00 6.000000e+00
      vertex   -7.699997e+00 4.000000e+00 4.000000e+00
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 1.158494e-16 -5.033011e-15 1.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+01 6.000000e+00
      vertex   -3.070000e+01 2.000000e+01 6.000000e+00
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -5.921189e-15 1.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
      vertex   -3.070000e+01 2.000000e+01 6.000000e+00
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.699997e+00 1.700000e+01 6.000000e+00
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
      vertex   -3.070000e+01 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 7.105427e-16 1.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
      vertex   -3.070000e+01 1.000000e+01 6.000000e+00
      vertex   -7.699997e+00 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 7.105427e-16 1.000000e+00
    outer loop
      vertex   -7.699997e+00 7.500000e+00 6.000000e+00
      vertex   -3.070000e+01 1.000000e+01 6.000000e+00
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.699997e+00 7.500000e+00 6.000000e+00
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 7.993606e-15 1.000000e+00
    outer loop
      vertex   -7.699997e+00 4.000000e+00 6.000000e+00
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
      vertex   -7.699997e+00 2.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 1.544658e-16 9.769963e-15 1.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+00 6.000000e+00
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
      vertex   -3.070000e+01 2.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 7.105427e-16 1.000000e+00
    outer loop
      vertex   -3.070000e+01 1.000000e+01 6.000000e+00
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 7.105427e-16 1.000000e+00
    outer loop
      vertex   -3.070000e+01 7.500000e+00 6.000000e+00
      vertex   -3.269150e+01 1.000000e+01 6.000000e+00
      vertex   -3.269150e+01 7.500000e+00 6.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 7.105427e-16 1.000000e+00
    outer loop
      vertex   -7.699997e+00 7.500000e+00 6.000000e+00
      vertex   -5.708497e+00 7.500000e+00 6.000000e+00
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 7.105427e-16 1.000000e+00
    outer loop
      vertex   -7.699997e+00 1.000000e+01 6.000000e+00
      vertex   -5.708497e+00 7.500000e+00 6.000000e+00
      vertex   -5.708497e+00 1.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 9.233134e-17
    outer loop
      vertex   -2.343920e+01 1.169491e+01 4.000000e+00
      vertex   -2.343920e+01 1.169491e+01 1.654429e-14
      vertex   -2.329150e+01 1.100000e+01 4.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 9.233134e-17
    outer loop
      vertex   -2.329150e+01 1.100000e+01 4.000000e+00
      vertex   -2.343920e+01 1.169491e+01 1.654429e-14
      vertex   -2.329150e+01 1.100000e+01 1.666634e-14
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 -9.233134e-17
    outer loop
      vertex   -2.329150e+01 1.100000e+01 4.000000e+00
      vertex   -2.329150e+01 1.100000e+01 1.666634e-14
      vertex   -2.343920e+01 1.030509e+01 4.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 -9.233134e-17
    outer loop
      vertex   -2.343920e+01 1.030509e+01 4.000000e+00
      vertex   -2.329150e+01 1.100000e+01 1.666634e-14
      vertex   -2.343920e+01 1.030509e+01 1.682453e-14
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 -2.610291e-16
    outer loop
      vertex   -2.343920e+01 1.030509e+01 4.000000e+00
      vertex   -2.343920e+01 1.030509e+01 1.682453e-14
      vertex   -2.385679e+01 9.730337e+00 4.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 -2.610291e-16
    outer loop
      vertex   -2.385679e+01 9.730337e+00 4.000000e+00
      vertex   -2.343920e+01 1.030509e+01 1.682453e-14
      vertex   -2.385679e+01 9.730337e+00 1.699151e-14
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 -3.845925e-16
    outer loop
      vertex   -2.385679e+01 9.730337e+00 4.000000e+00
      vertex   -2.385679e+01 9.730337e+00 1.699151e-14
      vertex   -2.447204e+01 9.375120e+00 4.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 -3.845925e-16
    outer loop
      vertex   -2.447204e+01 9.375120e+00 4.000000e+00
      vertex   -2.385679e+01 9.730337e+00 1.699151e-14
      vertex   -2.447204e+01 9.375120e+00 1.713842e-14
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 -4.416564e-16
    outer loop
      vertex   -2.447204e+01 9.375120e+00 4.000000e+00
      vertex   -2.447204e+01 9.375120e+00 1.713842e-14
      vertex   -2.517858e+01 9.300859e+00 4.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 -4.416564e-16
    outer loop
      vertex   -2.517858e+01 9.300859e+00 4.000000e+00
      vertex   -2.447204e+01 9.375120e+00 1.713842e-14
      vertex   -2.517858e+01 9.300859e+00 1.723985e-14
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 -4.223539e-16
    outer loop
      vertex   -2.517858e+01 9.300859e+00 4.000000e+00
      vertex   -2.517858e+01 9.300859e+00 1.723985e-14
      vertex   -2.585425e+01 9.520396e+00 4.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 -4.223539e-16
    outer loop
      vertex   -2.585425e+01 9.520396e+00 4.000000e+00
      vertex   -2.517858e+01 9.300859e+00 1.723985e-14
      vertex   -2.585425e+01 9.520396e+00 1.727825e-14
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 -3.300226e-16
    outer loop
      vertex   -2.585425e+01 9.520396e+00 4.000000e+00
      vertex   -2.585425e+01 9.520396e+00 1.727825e-14
      vertex   -2.638220e+01 9.995769e+00 4.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 -3.300226e-16
    outer loop
      vertex   -2.638220e+01 9.995769e+00 4.000000e+00
      vertex   -2.585425e+01 9.520396e+00 1.727825e-14
      vertex   -2.638220e+01 9.995769e+00 1.724700e-14
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 -1.806274e-16
    outer loop
      vertex   -2.638220e+01 9.995769e+00 4.000000e+00
      vertex   -2.638220e+01 9.995769e+00 1.724700e-14
      vertex   -2.667116e+01 1.064478e+01 4.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 -1.806274e-16
    outer loop
      vertex   -2.667116e+01 1.064478e+01 4.000000e+00
      vertex   -2.638220e+01 9.995769e+00 1.724700e-14
      vertex   -2.667116e+01 1.064478e+01 1.715150e-14
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.667116e+01 1.064478e+01 4.000000e+00
      vertex   -2.667116e+01 1.064478e+01 1.715150e-14
      vertex   -2.667116e+01 1.135522e+01 4.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.667116e+01 1.135522e+01 4.000000e+00
      vertex   -2.667116e+01 1.064478e+01 1.715150e-14
      vertex   -2.667116e+01 1.135522e+01 1.700825e-14
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 1.806274e-16
    outer loop
      vertex   -2.667116e+01 1.135522e+01 4.000000e+00
      vertex   -2.667116e+01 1.135522e+01 1.700825e-14
      vertex   -2.638220e+01 1.200423e+01 4.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 1.806274e-16
    outer loop
      vertex   -2.638220e+01 1.200423e+01 4.000000e+00
      vertex   -2.667116e+01 1.135522e+01 1.700825e-14
      vertex   -2.638220e+01 1.200423e+01 1.684203e-14
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 3.300226e-16
    outer loop
      vertex   -2.638220e+01 1.200423e+01 4.000000e+00
      vertex   -2.638220e+01 1.200423e+01 1.684203e-14
      vertex   -2.585425e+01 1.247960e+01 4.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 3.300226e-16
    outer loop
      vertex   -2.585425e+01 1.247960e+01 4.000000e+00
      vertex   -2.638220e+01 1.200423e+01 1.684203e-14
      vertex   -2.585425e+01 1.247960e+01 1.668158e-14
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 4.223539e-16
    outer loop
      vertex   -2.585425e+01 1.247960e+01 4.000000e+00
      vertex   -2.585425e+01 1.247960e+01 1.668158e-14
      vertex   -2.517858e+01 1.269914e+01 4.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 8.447079e-16
    outer loop
      vertex   -2.517858e+01 1.269914e+01 4.000000e+00
      vertex   -2.585425e+01 1.247960e+01 1.668158e-14
      vertex   -2.517858e+01 1.269914e+01 1.655464e-14
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 8.833129e-16
    outer loop
      vertex   -2.517858e+01 1.269914e+01 4.000000e+00
      vertex   -2.517858e+01 1.269914e+01 1.655464e-14
      vertex   -2.447204e+01 1.262488e+01 4.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 4.416564e-16
    outer loop
      vertex   -2.447204e+01 1.262488e+01 4.000000e+00
      vertex   -2.517858e+01 1.269914e+01 1.655464e-14
      vertex   -2.447204e+01 1.262488e+01 1.648316e-14
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 3.845925e-16
    outer loop
      vertex   -2.447204e+01 1.262488e+01 4.000000e+00
      vertex   -2.447204e+01 1.262488e+01 1.648316e-14
      vertex   -2.385679e+01 1.226966e+01 4.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 3.845925e-16
    outer loop
      vertex   -2.385679e+01 1.226966e+01 4.000000e+00
      vertex   -2.447204e+01 1.262488e+01 1.648316e-14
      vertex   -2.385679e+01 1.226966e+01 1.647950e-14
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 2.610291e-16
    outer loop
      vertex   -2.385679e+01 1.226966e+01 4.000000e+00
      vertex   -2.385679e+01 1.226966e+01 1.647950e-14
      vertex   -2.343920e+01 1.169491e+01 4.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 2.610291e-16
    outer loop
      vertex   -2.343920e+01 1.169491e+01 4.000000e+00
      vertex   -2.385679e+01 1.226966e+01 1.647950e-14
      vertex   -2.343920e+01 1.169491e+01 1.654429e-14
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 9.233134e-17
    outer loop
      vertex   -1.183920e+01 1.169491e+01 4.000000e+00
      vertex   -1.183920e+01 1.169491e+01 1.476794e-14
      vertex   -1.169150e+01 1.100000e+01 4.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 9.233134e-17
    outer loop
      vertex   -1.169150e+01 1.100000e+01 4.000000e+00
      vertex   -1.183920e+01 1.169491e+01 1.476794e-14
      vertex   -1.169150e+01 1.100000e+01 1.488998e-14
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 -9.233134e-17
    outer loop
      vertex   -1.169150e+01 1.100000e+01 4.000000e+00
      vertex   -1.169150e+01 1.100000e+01 1.488998e-14
      vertex   -1.183920e+01 1.030509e+01 4.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 -9.233134e-17
    outer loop
      vertex   -1.183920e+01 1.030509e+01 4.000000e+00
      vertex   -1.169150e+01 1.100000e+01 1.488998e-14
      vertex   -1.183920e+01 1.030509e+01 1.504817e-14
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 -2.610291e-16
    outer loop
      vertex   -1.183920e+01 1.030509e+01 4.000000e+00
      vertex   -1.183920e+01 1.030509e+01 1.504817e-14
      vertex   -1.225679e+01 9.730337e+00 4.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 -2.610291e-16
    outer loop
      vertex   -1.225679e+01 9.730337e+00 4.000000e+00
      vertex   -1.183920e+01 1.030509e+01 1.504817e-14
      vertex   -1.225679e+01 9.730337e+00 1.521516e-14
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 -3.845925e-16
    outer loop
      vertex   -1.225679e+01 9.730337e+00 4.000000e+00
      vertex   -1.225679e+01 9.730337e+00 1.521516e-14
      vertex   -1.287204e+01 9.375120e+00 4.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 -3.845925e-16
    outer loop
      vertex   -1.287204e+01 9.375120e+00 4.000000e+00
      vertex   -1.225679e+01 9.730337e+00 1.521516e-14
      vertex   -1.287204e+01 9.375120e+00 1.536206e-14
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 -4.416564e-16
    outer loop
      vertex   -1.287204e+01 9.375120e+00 4.000000e+00
      vertex   -1.287204e+01 9.375120e+00 1.536206e-14
      vertex   -1.357858e+01 9.300859e+00 4.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 -4.416564e-16
    outer loop
      vertex   -1.357858e+01 9.300859e+00 4.000000e+00
      vertex   -1.287204e+01 9.375120e+00 1.536206e-14
      vertex   -1.357858e+01 9.300859e+00 1.546349e-14
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 -4.223539e-16
    outer loop
      vertex   -1.357858e+01 9.300859e+00 4.000000e+00
      vertex   -1.357858e+01 9.300859e+00 1.546349e-14
      vertex   -1.425425e+01 9.520396e+00 4.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 -4.223539e-16
    outer loop
      vertex   -1.425425e+01 9.520396e+00 4.000000e+00
      vertex   -1.357858e+01 9.300859e+00 1.546349e-14
      vertex   -1.425425e+01 9.520396e+00 1.550190e-14
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 -3.300226e-16
    outer loop
      vertex   -1.425425e+01 9.520396e+00 4.000000e+00
      vertex   -1.425425e+01 9.520396e+00 1.550190e-14
      vertex   -1.478220e+01 9.995769e+00 4.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 -3.300226e-16
    outer loop
      vertex   -1.478220e+01 9.995769e+00 4.000000e+00
      vertex   -1.425425e+01 9.520396e+00 1.550190e-14
      vertex   -1.478220e+01 9.995769e+00 1.547065e-14
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 -1.806274e-16
    outer loop
      vertex   -1.478220e+01 9.995769e+00 4.000000e+00
      vertex   -1.478220e+01 9.995769e+00 1.547065e-14
      vertex   -1.507116e+01 1.064478e+01 4.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 -1.806274e-16
    outer loop
      vertex   -1.507116e+01 1.064478e+01 4.000000e+00
      vertex   -1.478220e+01 9.995769e+00 1.547065e-14
      vertex   -1.507116e+01 1.064478e+01 1.537514e-14
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.507116e+01 1.064478e+01 4.000000e+00
      vertex   -1.507116e+01 1.064478e+01 1.537514e-14
      vertex   -1.507116e+01 1.135522e+01 4.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.507116e+01 1.135522e+01 4.000000e+00
      vertex   -1.507116e+01 1.064478e+01 1.537514e-14
      vertex   -1.507116e+01 1.135522e+01 1.523189e-14
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 1.806274e-16
    outer loop
      vertex   -1.507116e+01 1.135522e+01 4.000000e+00
      vertex   -1.507116e+01 1.135522e+01 1.523189e-14
      vertex   -1.478220e+01 1.200423e+01 4.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 1.806274e-16
    outer loop
      vertex   -1.478220e+01 1.200423e+01 4.000000e+00
      vertex   -1.507116e+01 1.135522e+01 1.523189e-14
      vertex   -1.478220e+01 1.200423e+01 1.506567e-14
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 3.300226e-16
    outer loop
      vertex   -1.478220e+01 1.200423e+01 4.000000e+00
      vertex   -1.478220e+01 1.200423e+01 1.506567e-14
      vertex   -1.425425e+01 1.247960e+01 4.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 3.300226e-16
    outer loop
      vertex   -1.425425e+01 1.247960e+01 4.000000e+00
      vertex   -1.478220e+01 1.200423e+01 1.506567e-14
      vertex   -1.425425e+01 1.247960e+01 1.490522e-14
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 4.223539e-16
    outer loop
      vertex   -1.425425e+01 1.247960e+01 4.000000e+00
      vertex   -1.425425e+01 1.247960e+01 1.490522e-14
      vertex   -1.357858e+01 1.269914e+01 4.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 8.447079e-16
    outer loop
      vertex   -1.357858e+01 1.269914e+01 4.000000e+00
      vertex   -1.425425e+01 1.247960e+01 1.490522e-14
      vertex   -1.357858e+01 1.269914e+01 1.477828e-14
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 8.833129e-16
    outer loop
      vertex   -1.357858e+01 1.269914e+01 4.000000e+00
      vertex   -1.357858e+01 1.269914e+01 1.477828e-14
      vertex   -1.287204e+01 1.262488e+01 4.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 4.416564e-16
    outer loop
      vertex   -1.287204e+01 1.262488e+01 4.000000e+00
      vertex   -1.357858e+01 1.269914e+01 1.477828e-14
      vertex   -1.287204e+01 1.262488e+01 1.470680e-14
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 3.845925e-16
    outer loop
      vertex   -1.287204e+01 1.262488e+01 4.000000e+00
      vertex   -1.287204e+01 1.262488e+01 1.470680e-14
      vertex   -1.225679e+01 1.226966e+01 4.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 3.845925e-16
    outer loop
      vertex   -1.225679e+01 1.226966e+01 4.000000e+00
      vertex   -1.287204e+01 1.262488e+01 1.470680e-14
      vertex   -1.225679e+01 1.226966e+01 1.470314e-14
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 2.610291e-16
    outer loop
      vertex   -1.225679e+01 1.226966e+01 4.000000e+00
      vertex   -1.225679e+01 1.226966e+01 1.470314e-14
      vertex   -1.183920e+01 1.169491e+01 4.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 2.610291e-16
    outer loop
      vertex   -1.183920e+01 1.169491e+01 4.000000e+00
      vertex   -1.225679e+01 1.226966e+01 1.470314e-14
      vertex   -1.183920e+01 1.169491e+01 1.476794e-14
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+00 6.000000e+00
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -7.699997e+00 2.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+00 6.000000e+00
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
      vertex   -3.070000e+01 2.000000e+01 6.000000e+00
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
      vertex   -3.070000e+01 2.000000e+01 6.000000e+00
      vertex   -7.699997e+00 2.000000e+01 6.000000e+00
    endloop
  endfacet
  facet normal -1.653911e-16 -2.107802e-16 -1.000000e+00
    outer loop
      vertex   -2.343920e+01 1.169491e+01 1.654429e-14
      vertex   -1.507116e+01 1.135522e+01 1.523189e-14
      vertex   -2.329150e+01 1.100000e+01 1.666634e-14
    endloop
  endfacet
  facet normal -1.657863e-16 -2.016338e-16 -1.000000e+00
    outer loop
      vertex   -2.329150e+01 1.100000e+01 1.666634e-14
      vertex   -1.507116e+01 1.135522e+01 1.523189e-14
      vertex   -1.507116e+01 1.064478e+01 1.537514e-14
    endloop
  endfacet
  facet normal -1.653911e-16 -1.924874e-16 -1.000000e+00
    outer loop
      vertex   -2.329150e+01 1.100000e+01 1.666634e-14
      vertex   -1.507116e+01 1.064478e+01 1.537514e-14
      vertex   -2.343920e+01 1.030509e+01 1.682453e-14
    endloop
  endfacet
  facet normal -1.642624e-16 -2.202897e-16 -1.000000e+00
    outer loop
      vertex   -2.343920e+01 1.030509e+01 1.682453e-14
      vertex   -1.507116e+01 1.064478e+01 1.537514e-14
      vertex   -1.478220e+01 9.995769e+00 1.547065e-14
    endloop
  endfacet
  facet normal -1.625525e-16 -1.724328e-16 -1.000000e+00
    outer loop
      vertex   -2.343920e+01 1.030509e+01 1.682453e-14
      vertex   -1.478220e+01 9.995769e+00 1.547065e-14
      vertex   -2.385679e+01 9.730337e+00 1.699151e-14
    endloop
  endfacet
  facet normal -1.604607e-16 -2.439482e-16 -1.000000e+00
    outer loop
      vertex   -2.385679e+01 9.730337e+00 1.699151e-14
      vertex   -1.478220e+01 9.995769e+00 1.547065e-14
      vertex   -1.425425e+01 9.520396e+00 1.550190e-14
    endloop
  endfacet
  facet normal -1.581792e-16 -1.395944e-16 -1.000000e+00
    outer loop
      vertex   -2.385679e+01 9.730337e+00 1.699151e-14
      vertex   -1.425425e+01 9.520396e+00 1.550190e-14
      vertex   -2.447204e+01 9.375120e+00 1.713842e-14
    endloop
  endfacet
  facet normal -1.572972e-16 -2.016222e-16 -1.000000e+00
    outer loop
      vertex   -2.447204e+01 9.375120e+00 1.713842e-14
      vertex   -1.425425e+01 9.520396e+00 1.550190e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
    endloop
  endfacet
  facet normal -1.544658e-16 -1.951832e-16 -1.000000e+00
    outer loop
      vertex   -2.447204e+01 9.375120e+00 1.713842e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
    endloop
  endfacet
  facet normal -1.642624e-16 -1.829779e-16 -1.000000e+00
    outer loop
      vertex   -1.507116e+01 1.135522e+01 1.523189e-14
      vertex   -2.343920e+01 1.169491e+01 1.654429e-14
      vertex   -1.478220e+01 1.200423e+01 1.506567e-14
    endloop
  endfacet
  facet normal -1.625525e-16 -2.308349e-16 -1.000000e+00
    outer loop
      vertex   -1.478220e+01 1.200423e+01 1.506567e-14
      vertex   -2.343920e+01 1.169491e+01 1.654429e-14
      vertex   -2.385679e+01 1.226966e+01 1.647950e-14
    endloop
  endfacet
  facet normal -1.604607e-16 -1.593195e-16 -1.000000e+00
    outer loop
      vertex   -1.478220e+01 1.200423e+01 1.506567e-14
      vertex   -2.385679e+01 1.226966e+01 1.647950e-14
      vertex   -1.425425e+01 1.247960e+01 1.490522e-14
    endloop
  endfacet
  facet normal -1.581792e-16 -2.636733e-16 -1.000000e+00
    outer loop
      vertex   -1.425425e+01 1.247960e+01 1.490522e-14
      vertex   -2.385679e+01 1.226966e+01 1.647950e-14
      vertex   -2.447204e+01 1.262488e+01 1.648316e-14
    endloop
  endfacet
  facet normal -1.571540e-16 -1.915719e-16 -1.000000e+00
    outer loop
      vertex   -1.425425e+01 1.247960e+01 1.490522e-14
      vertex   -2.447204e+01 1.262488e+01 1.648316e-14
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
    endloop
  endfacet
  facet normal -1.544658e-16 -1.976853e-16 -1.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
      vertex   -2.447204e+01 1.262488e+01 1.648316e-14
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
    endloop
  endfacet
  facet normal -1.187782e-16 -1.675487e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
      vertex   -2.447204e+01 1.262488e+01 1.648316e-14
      vertex   -2.517858e+01 1.269914e+01 1.655464e-14
    endloop
  endfacet
  facet normal -1.305443e-16 -1.764471e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
      vertex   -2.517858e+01 1.269914e+01 1.655464e-14
      vertex   -2.585425e+01 1.247960e+01 1.668158e-14
    endloop
  endfacet
  facet normal -1.397171e-16 -1.823575e-16 -1.000000e+00
    outer loop
      vertex   -2.585425e+01 1.247960e+01 1.668158e-14
      vertex   -2.638220e+01 1.200423e+01 1.684203e-14
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
    endloop
  endfacet
  facet normal -1.514388e-16 -1.886874e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
      vertex   -2.638220e+01 1.200423e+01 1.684203e-14
      vertex   -2.667116e+01 1.135522e+01 1.700825e-14
    endloop
  endfacet
  facet normal -1.700757e-16 -1.973730e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
      vertex   -2.667116e+01 1.135522e+01 1.700825e-14
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
    endloop
  endfacet
  facet normal -1.601818e-16 -2.016338e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -2.667116e+01 1.135522e+01 1.700825e-14
      vertex   -2.667116e+01 1.064478e+01 1.715150e-14
    endloop
  endfacet
  facet normal -1.417031e-16 -2.102457e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -2.667116e+01 1.064478e+01 1.715150e-14
      vertex   -2.638220e+01 9.995769e+00 1.724700e-14
    endloop
  endfacet
  facet normal -1.339060e-16 -2.144562e-16 -1.000000e+00
    outer loop
      vertex   -2.638220e+01 9.995769e+00 1.724700e-14
      vertex   -2.585425e+01 9.520396e+00 1.727825e-14
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
    endloop
  endfacet
  facet normal -1.278043e-16 -2.183878e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -2.585425e+01 9.520396e+00 1.727825e-14
      vertex   -2.517858e+01 9.300859e+00 1.723985e-14
    endloop
  endfacet
  facet normal -1.199776e-16 -2.243069e-16 -1.000000e+00
    outer loop
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -2.517858e+01 9.300859e+00 1.723985e-14
      vertex   -2.447204e+01 9.375120e+00 1.713842e-14
    endloop
  endfacet
  facet normal -1.481672e-16 -2.071192e-16 -1.000000e+00
    outer loop
      vertex   -1.183920e+01 1.169491e+01 1.476794e-14
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
      vertex   -1.169150e+01 1.100000e+01 1.488998e-14
    endloop
  endfacet
  facet normal -1.701429e-16 -1.973730e-16 -1.000000e+00
    outer loop
      vertex   -1.169150e+01 1.100000e+01 1.488998e-14
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
    endloop
  endfacet
  facet normal -1.611565e-16 -1.933875e-16 -1.000000e+00
    outer loop
      vertex   -1.169150e+01 1.100000e+01 1.488998e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -1.183920e+01 1.030509e+01 1.504817e-14
    endloop
  endfacet
  facet normal -1.448777e-16 -1.852743e-16 -1.000000e+00
    outer loop
      vertex   -1.183920e+01 1.030509e+01 1.504817e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -1.225679e+01 9.730337e+00 1.521516e-14
    endloop
  endfacet
  facet normal -1.351250e-16 -1.795254e-16 -1.000000e+00
    outer loop
      vertex   -1.225679e+01 9.730337e+00 1.521516e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -1.287204e+01 9.375120e+00 1.536206e-14
    endloop
  endfacet
  facet normal -1.254011e-16 -1.727062e-16 -1.000000e+00
    outer loop
      vertex   -1.287204e+01 9.375120e+00 1.536206e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -1.357858e+01 9.300859e+00 1.546349e-14
    endloop
  endfacet
  facet normal -1.085539e-16 -1.591409e-16 -1.000000e+00
    outer loop
      vertex   -1.357858e+01 9.300859e+00 1.546349e-14
      vertex   -7.699997e+00 2.000000e+00 1.598721e-14
      vertex   -1.425425e+01 9.520396e+00 1.550190e-14
    endloop
  endfacet
  facet normal -1.373387e-16 -2.125160e-16 -1.000000e+00
    outer loop
      vertex   -1.183920e+01 1.169491e+01 1.476794e-14
      vertex   -1.225679e+01 1.226966e+01 1.470314e-14
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
    endloop
  endfacet
  facet normal -1.308514e-16 -2.163401e-16 -1.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
      vertex   -1.225679e+01 1.226966e+01 1.470314e-14
      vertex   -1.287204e+01 1.262488e+01 1.470680e-14
    endloop
  endfacet
  facet normal -1.243831e-16 -2.208762e-16 -1.000000e+00
    outer loop
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
      vertex   -1.287204e+01 1.262488e+01 1.470680e-14
      vertex   -1.357858e+01 1.269914e+01 1.477828e-14
    endloop
  endfacet
  facet normal -1.131765e-16 -2.298996e-16 -1.000000e+00
    outer loop
      vertex   -1.357858e+01 1.269914e+01 1.477828e-14
      vertex   -1.425425e+01 1.247960e+01 1.490522e-14
      vertex   -7.699997e+00 2.000000e+01 1.243450e-14
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 6.000000e+00
      vertex   -3.070000e+01 2.000000e+01 6.000000e+00
      vertex   -3.070000e+01 1.700000e+01 4.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 4.000000e+00
      vertex   -3.070000e+01 2.000000e+01 6.000000e+00
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 4.000000e+00
      vertex   -3.070000e+01 2.000000e+01 1.598721e-14
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 1.700000e+01 4.000000e+00
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -3.070000e+01 4.000000e+00 4.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 4.000000e+00 4.000000e+00
      vertex   -3.070000e+01 2.000000e+00 1.953993e-14
      vertex   -3.070000e+01 2.000000e+00 6.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.070000e+01 4.000000e+00 4.000000e+00
      vertex   -3.070000e+01 2.000000e+00 6.000000e+00
      vertex   -3.070000e+01 4.000000e+00 6.000000e+00
    endloop
  endfacet
endsolid
```
### Remote Control <a name="casebase"></a>
```stl

```

---


### Schematics & PCB
Schematic and PCB files are available in the `eda/rc-car-kicad/` directory. Example files:
- `rc-car-kicad.kicad_sch` – Main schematic
- `rc-car-kicad.kicad_pcb` – Main PCB layout
- `rc-car-panel.kicad_pcb` – Panelized PCB for manufacturing

---

## License
See [LICENSE](LICENSE) for details.