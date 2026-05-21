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

```stl
solid ASCII
  facet normal -9.807853e-01 1.950903e-01 -4.620667e-16
    outer loop
      vertex   -6.980633e+01 -2.908028e+01 -1.422585e-15
      vertex   -6.980633e+01 -2.908028e+01 3.000000e+00
      vertex   -6.969150e+01 -2.850300e+01 -1.592468e-15
    endloop
  endfacet
  facet normal -9.807853e-01 1.950903e-01 -2.310334e-16
    outer loop
      vertex   -6.969150e+01 -2.850300e+01 -1.592468e-15
      vertex   -6.980633e+01 -2.908028e+01 3.000000e+00
      vertex   -6.969150e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.807853e-01 -1.950903e-01 2.310334e-16
    outer loop
      vertex   -6.969150e+01 -2.850300e+01 -1.592468e-15
      vertex   -6.969150e+01 -2.850300e+01 3.000000e+00
      vertex   -6.980633e+01 -2.792572e+01 -1.790348e-15
    endloop
  endfacet
  facet normal -9.807853e-01 -1.950903e-01 6.931001e-16
    outer loop
      vertex   -6.980633e+01 -2.792572e+01 -1.790348e-15
      vertex   -6.969150e+01 -2.850300e+01 3.000000e+00
      vertex   -6.980633e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.314696e-01 -5.555702e-01 1.973782e-15
    outer loop
      vertex   -6.980633e+01 -2.792572e+01 -1.790348e-15
      vertex   -6.980633e+01 -2.792572e+01 3.000000e+00
      vertex   -7.013333e+01 -2.743633e+01 -1.986097e-15
    endloop
  endfacet
  facet normal -8.314696e-01 -5.555702e-01 1.315855e-15
    outer loop
      vertex   -7.013333e+01 -2.743633e+01 -1.986097e-15
      vertex   -6.980633e+01 -2.792572e+01 3.000000e+00
      vertex   -7.013333e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.555702e-01 -8.314696e-01 1.969316e-15
    outer loop
      vertex   -7.013333e+01 -2.743633e+01 -1.986097e-15
      vertex   -7.013333e+01 -2.743633e+01 3.000000e+00
      vertex   -7.062272e+01 -2.710933e+01 -2.149915e-15
    endloop
  endfacet
  facet normal -5.555702e-01 -8.314696e-01 0.000000e+00
    outer loop
      vertex   -7.062272e+01 -2.710933e+01 -2.149915e-15
      vertex   -7.013333e+01 -2.743633e+01 3.000000e+00
      vertex   -7.062272e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.950903e-01 -9.807853e-01 0.000000e+00
    outer loop
      vertex   -7.062272e+01 -2.710933e+01 -2.149915e-15
      vertex   -7.062272e+01 -2.710933e+01 3.000000e+00
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
    endloop
  endfacet
  facet normal -1.950903e-01 -9.807853e-01 1.161483e-15
    outer loop
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
      vertex   -7.062272e+01 -2.710933e+01 3.000000e+00
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.950903e-01 -9.807853e-01 1.161483e-15
    outer loop
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
      vertex   -7.177728e+01 -2.710933e+01 -2.290657e-15
    endloop
  endfacet
  facet normal 1.950903e-01 -9.807853e-01 0.000000e+00
    outer loop
      vertex   -7.177728e+01 -2.710933e+01 -2.290657e-15
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
      vertex   -7.177728e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.555702e-01 -8.314696e-01 0.000000e+00
    outer loop
      vertex   -7.177728e+01 -2.710933e+01 -2.290657e-15
      vertex   -7.177728e+01 -2.710933e+01 3.000000e+00
      vertex   -7.226667e+01 -2.743633e+01 -2.246154e-15
    endloop
  endfacet
  facet normal 5.555702e-01 -8.314696e-01 1.969316e-15
    outer loop
      vertex   -7.226667e+01 -2.743633e+01 -2.246154e-15
      vertex   -7.177728e+01 -2.710933e+01 3.000000e+00
      vertex   -7.226667e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.314696e-01 -5.555702e-01 1.315855e-15
    outer loop
      vertex   -7.226667e+01 -2.743633e+01 -2.246154e-15
      vertex   -7.226667e+01 -2.743633e+01 3.000000e+00
      vertex   -7.259367e+01 -2.792572e+01 -2.130129e-15
    endloop
  endfacet
  facet normal 8.314696e-01 -5.555702e-01 1.315855e-15
    outer loop
      vertex   -7.259367e+01 -2.792572e+01 -2.130129e-15
      vertex   -7.226667e+01 -2.743633e+01 3.000000e+00
      vertex   -7.259367e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.807853e-01 -1.950903e-01 4.620667e-16
    outer loop
      vertex   -7.259367e+01 -2.792572e+01 -2.130129e-15
      vertex   -7.259367e+01 -2.792572e+01 3.000000e+00
      vertex   -7.270850e+01 -2.850300e+01 -1.960245e-15
    endloop
  endfacet
  facet normal 9.807853e-01 -1.950903e-01 2.310334e-16
    outer loop
      vertex   -7.270850e+01 -2.850300e+01 -1.960245e-15
      vertex   -7.259367e+01 -2.792572e+01 3.000000e+00
      vertex   -7.270850e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.807853e-01 1.950903e-01 -2.310334e-16
    outer loop
      vertex   -7.270850e+01 -2.850300e+01 -1.960245e-15
      vertex   -7.270850e+01 -2.850300e+01 3.000000e+00
      vertex   -7.259367e+01 -2.908028e+01 -1.762366e-15
    endloop
  endfacet
  facet normal 9.807853e-01 1.950903e-01 -2.310334e-16
    outer loop
      vertex   -7.259367e+01 -2.908028e+01 -1.762366e-15
      vertex   -7.270850e+01 -2.850300e+01 3.000000e+00
      vertex   -7.259367e+01 -2.908028e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.314696e-01 5.555702e-01 -6.579273e-16
    outer loop
      vertex   -7.259367e+01 -2.908028e+01 -1.762366e-15
      vertex   -7.259367e+01 -2.908028e+01 3.000000e+00
      vertex   -7.226667e+01 -2.956967e+01 -1.566617e-15
    endloop
  endfacet
  facet normal 8.314696e-01 5.555702e-01 -6.579273e-16
    outer loop
      vertex   -7.226667e+01 -2.956967e+01 -1.566617e-15
      vertex   -7.259367e+01 -2.908028e+01 3.000000e+00
      vertex   -7.226667e+01 -2.956967e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.555702e-01 8.314696e-01 -9.846578e-16
    outer loop
      vertex   -7.226667e+01 -2.956967e+01 -1.566617e-15
      vertex   -7.226667e+01 -2.956967e+01 3.000000e+00
      vertex   -7.177728e+01 -2.989667e+01 -1.402799e-15
    endloop
  endfacet
  facet normal 5.555702e-01 8.314696e-01 -9.846578e-16
    outer loop
      vertex   -7.177728e+01 -2.989667e+01 -1.402799e-15
      vertex   -7.226667e+01 -2.956967e+01 3.000000e+00
      vertex   -7.177728e+01 -2.989667e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.950903e-01 9.807853e-01 -1.161483e-15
    outer loop
      vertex   -7.177728e+01 -2.989667e+01 -1.402799e-15
      vertex   -7.177728e+01 -2.989667e+01 3.000000e+00
      vertex   -7.120000e+01 -3.001150e+01 -1.295851e-15
    endloop
  endfacet
  facet normal 1.950903e-01 9.807853e-01 -1.161483e-15
    outer loop
      vertex   -7.120000e+01 -3.001150e+01 -1.295851e-15
      vertex   -7.177728e+01 -2.989667e+01 3.000000e+00
      vertex   -7.120000e+01 -3.001150e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.950903e-01 9.807853e-01 -1.161483e-15
    outer loop
      vertex   -7.120000e+01 -3.001150e+01 -1.295851e-15
      vertex   -7.120000e+01 -3.001150e+01 3.000000e+00
      vertex   -7.062272e+01 -2.989667e+01 -1.262057e-15
    endloop
  endfacet
  facet normal -1.950903e-01 9.807853e-01 -1.161483e-15
    outer loop
      vertex   -7.062272e+01 -2.989667e+01 -1.262057e-15
      vertex   -7.120000e+01 -3.001150e+01 3.000000e+00
      vertex   -7.062272e+01 -2.989667e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.555702e-01 8.314696e-01 -9.846578e-16
    outer loop
      vertex   -7.062272e+01 -2.989667e+01 -1.262057e-15
      vertex   -7.062272e+01 -2.989667e+01 3.000000e+00
      vertex   -7.013333e+01 -2.956967e+01 -1.306559e-15
    endloop
  endfacet
  facet normal -5.555702e-01 8.314696e-01 -9.846578e-16
    outer loop
      vertex   -7.013333e+01 -2.956967e+01 -1.306559e-15
      vertex   -7.062272e+01 -2.989667e+01 3.000000e+00
      vertex   -7.013333e+01 -2.956967e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.314696e-01 5.555702e-01 -6.579273e-16
    outer loop
      vertex   -7.013333e+01 -2.956967e+01 -1.306559e-15
      vertex   -7.013333e+01 -2.956967e+01 3.000000e+00
      vertex   -6.980633e+01 -2.908028e+01 -1.422585e-15
    endloop
  endfacet
  facet normal -8.314696e-01 5.555702e-01 -1.315855e-15
    outer loop
      vertex   -6.980633e+01 -2.908028e+01 -1.422585e-15
      vertex   -7.013333e+01 -2.956967e+01 3.000000e+00
      vertex   -6.980633e+01 -2.908028e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.807853e-01 1.950903e-01 -2.310334e-16
    outer loop
      vertex   -4.940633e+01 -2.908028e+01 3.537723e-16
      vertex   -4.940633e+01 -2.908028e+01 3.000000e+00
      vertex   -4.929150e+01 -2.850300e+01 1.838885e-16
    endloop
  endfacet
  facet normal -9.807853e-01 1.950903e-01 -2.310334e-16
    outer loop
      vertex   -4.929150e+01 -2.850300e+01 1.838885e-16
      vertex   -4.940633e+01 -2.908028e+01 3.000000e+00
      vertex   -4.929150e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.807853e-01 -1.950903e-01 2.310334e-16
    outer loop
      vertex   -4.929150e+01 -2.850300e+01 1.838885e-16
      vertex   -4.929150e+01 -2.850300e+01 3.000000e+00
      vertex   -4.940633e+01 -2.792572e+01 -1.399066e-17
    endloop
  endfacet
  facet normal -9.807853e-01 -1.950903e-01 2.310334e-16
    outer loop
      vertex   -4.940633e+01 -2.792572e+01 -1.399066e-17
      vertex   -4.929150e+01 -2.850300e+01 3.000000e+00
      vertex   -4.940633e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.314696e-01 -5.555702e-01 6.579273e-16
    outer loop
      vertex   -4.940633e+01 -2.792572e+01 -1.399066e-17
      vertex   -4.940633e+01 -2.792572e+01 3.000000e+00
      vertex   -4.973333e+01 -2.743633e+01 -2.097399e-16
    endloop
  endfacet
  facet normal -8.314696e-01 -5.555702e-01 1.315855e-15
    outer loop
      vertex   -4.973333e+01 -2.743633e+01 -2.097399e-16
      vertex   -4.940633e+01 -2.792572e+01 3.000000e+00
      vertex   -4.973333e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.555702e-01 -8.314696e-01 1.969316e-15
    outer loop
      vertex   -4.973333e+01 -2.743633e+01 -2.097399e-16
      vertex   -4.973333e+01 -2.743633e+01 3.000000e+00
      vertex   -5.022272e+01 -2.710933e+01 -3.735581e-16
    endloop
  endfacet
  facet normal -5.555702e-01 -8.314696e-01 1.315855e-15
    outer loop
      vertex   -5.022272e+01 -2.710933e+01 -3.735581e-16
      vertex   -4.973333e+01 -2.743633e+01 3.000000e+00
      vertex   -5.022272e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.950903e-01 -9.807853e-01 4.620667e-16
    outer loop
      vertex   -5.022272e+01 -2.710933e+01 -3.735581e-16
      vertex   -5.022272e+01 -2.710933e+01 3.000000e+00
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
    endloop
  endfacet
  facet normal -1.950903e-01 -9.807853e-01 0.000000e+00
    outer loop
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
      vertex   -5.022272e+01 -2.710933e+01 3.000000e+00
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.950903e-01 -9.807853e-01 0.000000e+00
    outer loop
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
      vertex   -5.137728e+01 -2.710933e+01 -5.143002e-16
    endloop
  endfacet
  facet normal 1.950903e-01 -9.807853e-01 4.620667e-16
    outer loop
      vertex   -5.137728e+01 -2.710933e+01 -5.143002e-16
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
      vertex   -5.137728e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.555702e-01 -8.314696e-01 1.315855e-15
    outer loop
      vertex   -5.137728e+01 -2.710933e+01 -5.143002e-16
      vertex   -5.137728e+01 -2.710933e+01 3.000000e+00
      vertex   -5.186667e+01 -2.743633e+01 -4.697975e-16
    endloop
  endfacet
  facet normal 5.555702e-01 -8.314696e-01 1.969316e-15
    outer loop
      vertex   -5.186667e+01 -2.743633e+01 -4.697975e-16
      vertex   -5.137728e+01 -2.710933e+01 3.000000e+00
      vertex   -5.186667e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.314696e-01 -5.555702e-01 1.315855e-15
    outer loop
      vertex   -5.186667e+01 -2.743633e+01 -4.697975e-16
      vertex   -5.186667e+01 -2.743633e+01 3.000000e+00
      vertex   -5.219367e+01 -2.792572e+01 -3.537723e-16
    endloop
  endfacet
  facet normal 8.314696e-01 -5.555702e-01 6.579273e-16
    outer loop
      vertex   -5.219367e+01 -2.792572e+01 -3.537723e-16
      vertex   -5.186667e+01 -2.743633e+01 3.000000e+00
      vertex   -5.219367e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.807853e-01 -1.950903e-01 2.310334e-16
    outer loop
      vertex   -5.219367e+01 -2.792572e+01 -3.537723e-16
      vertex   -5.219367e+01 -2.792572e+01 3.000000e+00
      vertex   -5.230850e+01 -2.850300e+01 -1.838885e-16
    endloop
  endfacet
  facet normal 9.807853e-01 -1.950903e-01 2.310334e-16
    outer loop
      vertex   -5.230850e+01 -2.850300e+01 -1.838885e-16
      vertex   -5.219367e+01 -2.792572e+01 3.000000e+00
      vertex   -5.230850e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.807853e-01 1.950903e-01 -2.310334e-16
    outer loop
      vertex   -5.230850e+01 -2.850300e+01 -1.838885e-16
      vertex   -5.230850e+01 -2.850300e+01 3.000000e+00
      vertex   -5.219367e+01 -2.908028e+01 1.399066e-17
    endloop
  endfacet
  facet normal 9.807853e-01 1.950903e-01 -2.310334e-16
    outer loop
      vertex   -5.219367e+01 -2.908028e+01 1.399066e-17
      vertex   -5.230850e+01 -2.850300e+01 3.000000e+00
      vertex   -5.219367e+01 -2.908028e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.314696e-01 5.555702e-01 -6.579273e-16
    outer loop
      vertex   -5.219367e+01 -2.908028e+01 1.399066e-17
      vertex   -5.219367e+01 -2.908028e+01 3.000000e+00
      vertex   -5.186667e+01 -2.956967e+01 2.097399e-16
    endloop
  endfacet
  facet normal 8.314696e-01 5.555702e-01 -6.579273e-16
    outer loop
      vertex   -5.186667e+01 -2.956967e+01 2.097399e-16
      vertex   -5.219367e+01 -2.908028e+01 3.000000e+00
      vertex   -5.186667e+01 -2.956967e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.555702e-01 8.314696e-01 -9.846578e-16
    outer loop
      vertex   -5.186667e+01 -2.956967e+01 2.097399e-16
      vertex   -5.186667e+01 -2.956967e+01 3.000000e+00
      vertex   -5.137728e+01 -2.989667e+01 3.735581e-16
    endloop
  endfacet
  facet normal 5.555702e-01 8.314696e-01 -2.300512e-15
    outer loop
      vertex   -5.137728e+01 -2.989667e+01 3.735581e-16
      vertex   -5.186667e+01 -2.956967e+01 3.000000e+00
      vertex   -5.137728e+01 -2.989667e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.950903e-01 9.807853e-01 -1.623550e-15
    outer loop
      vertex   -5.137728e+01 -2.989667e+01 3.735581e-16
      vertex   -5.137728e+01 -2.989667e+01 3.000000e+00
      vertex   -5.080000e+01 -3.001150e+01 4.805055e-16
    endloop
  endfacet
  facet normal 1.950903e-01 9.807853e-01 -2.322966e-15
    outer loop
      vertex   -5.080000e+01 -3.001150e+01 4.805055e-16
      vertex   -5.137728e+01 -2.989667e+01 3.000000e+00
      vertex   -5.080000e+01 -3.001150e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.950903e-01 9.807853e-01 -2.322966e-15
    outer loop
      vertex   -5.080000e+01 -3.001150e+01 4.805055e-16
      vertex   -5.080000e+01 -3.001150e+01 3.000000e+00
      vertex   -5.022272e+01 -2.989667e+01 5.143002e-16
    endloop
  endfacet
  facet normal -1.950903e-01 9.807853e-01 -1.623550e-15
    outer loop
      vertex   -5.022272e+01 -2.989667e+01 5.143002e-16
      vertex   -5.080000e+01 -3.001150e+01 3.000000e+00
      vertex   -5.022272e+01 -2.989667e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.555702e-01 8.314696e-01 -2.300512e-15
    outer loop
      vertex   -5.022272e+01 -2.989667e+01 5.143002e-16
      vertex   -5.022272e+01 -2.989667e+01 3.000000e+00
      vertex   -4.973333e+01 -2.956967e+01 4.697975e-16
    endloop
  endfacet
  facet normal -5.555702e-01 8.314696e-01 -9.846578e-16
    outer loop
      vertex   -4.973333e+01 -2.956967e+01 4.697975e-16
      vertex   -5.022272e+01 -2.989667e+01 3.000000e+00
      vertex   -4.973333e+01 -2.956967e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.314696e-01 5.555702e-01 -6.579273e-16
    outer loop
      vertex   -4.973333e+01 -2.956967e+01 4.697975e-16
      vertex   -4.973333e+01 -2.956967e+01 3.000000e+00
      vertex   -4.940633e+01 -2.908028e+01 3.537723e-16
    endloop
  endfacet
  facet normal -8.314696e-01 5.555702e-01 -6.579273e-16
    outer loop
      vertex   -4.940633e+01 -2.908028e+01 3.537723e-16
      vertex   -4.973333e+01 -2.956967e+01 3.000000e+00
      vertex   -4.940633e+01 -2.908028e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.897417e-01 1.428684e-01 -3.383803e-16
    outer loop
      vertex   -6.978756e+01 -3.056577e+01 7.225984e-15
      vertex   -6.978756e+01 -3.056577e+01 3.000000e+00
      vertex   -6.968718e+01 -3.126120e+01 7.393176e-15
    endloop
  endfacet
  facet normal 9.897417e-01 1.428684e-01 -3.383803e-16
    outer loop
      vertex   -6.968718e+01 -3.126120e+01 7.393176e-15
      vertex   -6.978756e+01 -3.056577e+01 3.000000e+00
      vertex   -6.968718e+01 -3.126120e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.684219e-01 2.493171e-01 -5.905015e-16
    outer loop
      vertex   -6.968718e+01 -3.126120e+01 7.393176e-15
      vertex   -6.968718e+01 -3.126120e+01 3.000000e+00
      vertex   -6.951098e+01 -3.194563e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.362409e-01 3.513589e-01 0.000000e+00
    outer loop
      vertex   -6.926049e+01 -3.261308e+01 3.000000e+00
      vertex   -6.926049e+01 -3.261308e+01 7.718185e-15
      vertex   -6.951098e+01 -3.194563e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.362409e-01 3.513589e-01 -8.321850e-16
    outer loop
      vertex   -6.951098e+01 -3.194563e+01 3.000000e+00
      vertex   -6.926049e+01 -3.261308e+01 7.718185e-15
      vertex   -6.951098e+01 -3.194563e+01 7.557722e-15
    endloop
  endfacet
  facet normal 9.684219e-01 2.493171e-01 -5.905015e-16
    outer loop
      vertex   -6.951098e+01 -3.194563e+01 3.000000e+00
      vertex   -6.951098e+01 -3.194563e+01 7.557722e-15
      vertex   -6.968718e+01 -3.126120e+01 7.393176e-15
    endloop
  endfacet
  facet normal 8.942841e-01 4.474997e-01 0.000000e+00
    outer loop
      vertex   -6.926049e+01 -3.261308e+01 3.000000e+00
      vertex   -6.893791e+01 -3.325772e+01 3.000000e+00
      vertex   -6.926049e+01 -3.261308e+01 7.718185e-15
    endloop
  endfacet
  facet normal 8.942841e-01 4.474997e-01 0.000000e+00
    outer loop
      vertex   -6.926049e+01 -3.261308e+01 7.718185e-15
      vertex   -6.893791e+01 -3.325772e+01 3.000000e+00
      vertex   -6.893791e+01 -3.325772e+01 7.873166e-15
    endloop
  endfacet
  facet normal 8.438325e-01 5.366067e-01 -0.000000e+00
    outer loop
      vertex   -6.893791e+01 -3.325772e+01 7.873166e-15
      vertex   -6.893791e+01 -3.325772e+01 3.000000e+00
      vertex   -6.854606e+01 -3.387392e+01 8.021311e-15
    endloop
  endfacet
  facet normal 8.438325e-01 5.366067e-01 -1.270940e-15
    outer loop
      vertex   -6.854606e+01 -3.387392e+01 8.021311e-15
      vertex   -6.893791e+01 -3.325772e+01 3.000000e+00
      vertex   -6.854606e+01 -3.387392e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.862416e-01 6.179192e-01 -1.463527e-15
    outer loop
      vertex   -6.854606e+01 -3.387392e+01 8.021311e-15
      vertex   -6.854606e+01 -3.387392e+01 3.000000e+00
      vertex   -6.808834e+01 -3.445632e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.862416e-01 6.179192e-01 -1.473829e-15
    outer loop
      vertex   -6.854606e+01 -3.387392e+01 8.021311e-15
      vertex   -6.808834e+01 -3.445632e+01 3.000000e+00
      vertex   -6.808834e+01 -3.445632e+01 8.161328e-15
    endloop
  endfacet
  facet normal 7.228377e-01 6.910178e-01 -1.636659e-15
    outer loop
      vertex   -6.808834e+01 -3.445632e+01 8.161328e-15
      vertex   -6.808834e+01 -3.445632e+01 3.000000e+00
      vertex   -6.756876e+01 -3.499983e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.228377e-01 6.910178e-01 -3.273014e-15
    outer loop
      vertex   -6.808834e+01 -3.445632e+01 8.161328e-15
      vertex   -6.756876e+01 -3.499983e+01 3.000000e+00
      vertex   -6.756876e+01 -3.499983e+01 8.291994e-15
    endloop
  endfacet
  facet normal 6.548414e-01 7.557663e-01 -3.580028e-15
    outer loop
      vertex   -6.756876e+01 -3.499983e+01 8.291994e-15
      vertex   -6.756876e+01 -3.499983e+01 3.000000e+00
      vertex   -6.699185e+01 -3.549969e+01 8.412170e-15
    endloop
  endfacet
  facet normal 6.548414e-01 7.557663e-01 -1.790014e-15
    outer loop
      vertex   -6.699185e+01 -3.549969e+01 8.412170e-15
      vertex   -6.756876e+01 -3.499983e+01 3.000000e+00
      vertex   -6.699185e+01 -3.549969e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.833208e-01 8.122419e-01 -1.923775e-15
    outer loop
      vertex   -6.699185e+01 -3.549969e+01 8.412170e-15
      vertex   -6.699185e+01 -3.549969e+01 3.000000e+00
      vertex   -6.636265e+01 -3.595156e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.091713e-01 8.606652e-01 3.734554e-16
    outer loop
      vertex   -6.568664e+01 -3.635149e+01 3.000000e+00
      vertex   -6.568664e+01 -3.635149e+01 8.616954e-15
      vertex   -6.636265e+01 -3.595156e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.091713e-01 8.606652e-01 0.000000e+00
    outer loop
      vertex   -6.636265e+01 -3.595156e+01 3.000000e+00
      vertex   -6.568664e+01 -3.635149e+01 8.616954e-15
      vertex   -6.636265e+01 -3.595156e+01 8.520806e-15
    endloop
  endfacet
  facet normal 5.833208e-01 8.122419e-01 0.000000e+00
    outer loop
      vertex   -6.636265e+01 -3.595156e+01 3.000000e+00
      vertex   -6.636265e+01 -3.595156e+01 8.520806e-15
      vertex   -6.699185e+01 -3.549969e+01 8.412170e-15
    endloop
  endfacet
  facet normal 4.331164e-01 9.013380e-01 -8.314605e-17
    outer loop
      vertex   -6.568664e+01 -3.635149e+01 3.000000e+00
      vertex   -6.496974e+01 -3.669598e+01 3.000000e+00
      vertex   -6.568664e+01 -3.635149e+01 8.616954e-15
    endloop
  endfacet
  facet normal 4.331164e-01 9.013380e-01 -2.140278e-15
    outer loop
      vertex   -6.568664e+01 -3.635149e+01 8.616954e-15
      vertex   -6.496974e+01 -3.669598e+01 3.000000e+00
      vertex   -6.496974e+01 -3.669598e+01 8.699775e-15
    endloop
  endfacet
  facet normal 3.557205e-01 9.345924e-01 -2.213559e-15
    outer loop
      vertex   -6.496974e+01 -3.669598e+01 8.699775e-15
      vertex   -6.496974e+01 -3.669598e+01 3.000000e+00
      vertex   -6.421818e+01 -3.698204e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.557205e-01 9.345924e-01 -5.292343e-16
    outer loop
      vertex   -6.496974e+01 -3.669598e+01 8.699775e-15
      vertex   -6.421818e+01 -3.698204e+01 3.000000e+00
      vertex   -6.421818e+01 -3.698204e+01 8.768546e-15
    endloop
  endfacet
  facet normal 2.774117e-01 9.607511e-01 -9.614302e-16
    outer loop
      vertex   -6.421818e+01 -3.698204e+01 8.768546e-15
      vertex   -6.421818e+01 -3.698204e+01 3.000000e+00
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
    endloop
  endfacet
  facet normal 2.774117e-01 9.607511e-01 6.570428e-16
    outer loop
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
      vertex   -6.421818e+01 -3.698204e+01 3.000000e+00
      vertex   -6.343854e+01 -3.720716e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.985059e-01 9.800997e-01 4.701563e-16
    outer loop
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
      vertex   -6.343854e+01 -3.720716e+01 3.000000e+00
      vertex   -6.263761e+01 -3.736937e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.192336e-01 9.928662e-01 -2.069178e-15
    outer loop
      vertex   -6.182239e+01 -3.746727e+01 3.000000e+00
      vertex   -6.182239e+01 -3.746727e+01 8.885203e-15
      vertex   -6.263761e+01 -3.736937e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.192336e-01 9.928662e-01 2.824018e-16
    outer loop
      vertex   -6.263761e+01 -3.736937e+01 3.000000e+00
      vertex   -6.182239e+01 -3.746727e+01 8.885203e-15
      vertex   -6.263761e+01 -3.736937e+01 8.861667e-15
    endloop
  endfacet
  facet normal 1.985059e-01 9.800997e-01 4.701563e-16
    outer loop
      vertex   -6.263761e+01 -3.736937e+01 3.000000e+00
      vertex   -6.263761e+01 -3.736937e+01 8.861667e-15
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
    endloop
  endfacet
  facet normal 3.976502e-02 9.992091e-01 -2.272420e-15
    outer loop
      vertex   -6.182239e+01 -3.746727e+01 3.000000e+00
      vertex   -6.100000e+01 -3.750000e+01 3.000000e+00
      vertex   -6.182239e+01 -3.746727e+01 8.885203e-15
    endloop
  endfacet
  facet normal 3.976502e-02 9.992091e-01 9.414337e-17
    outer loop
      vertex   -6.182239e+01 -3.746727e+01 8.885203e-15
      vertex   -6.100000e+01 -3.750000e+01 3.000000e+00
      vertex   -6.100000e+01 -3.750000e+01 8.893072e-15
    endloop
  endfacet
  facet normal -3.976502e-02 9.992091e-01 -9.418248e-17
    outer loop
      vertex   -6.100000e+01 -3.750000e+01 8.893072e-15
      vertex   -6.100000e+01 -3.750000e+01 3.000000e+00
      vertex   -6.017761e+01 -3.746727e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.976502e-02 9.992091e-01 2.177944e-15
    outer loop
      vertex   -6.100000e+01 -3.750000e+01 8.893072e-15
      vertex   -6.017761e+01 -3.746727e+01 3.000000e+00
      vertex   -6.017761e+01 -3.746727e+01 8.885203e-15
    endloop
  endfacet
  facet normal -1.192336e-01 9.928662e-01 1.786776e-15
    outer loop
      vertex   -6.017761e+01 -3.746727e+01 8.885203e-15
      vertex   -6.017761e+01 -3.746727e+01 3.000000e+00
      vertex   -5.936239e+01 -3.736937e+01 8.861667e-15
    endloop
  endfacet
  facet normal -1.192336e-01 9.928662e-01 -5.648036e-16
    outer loop
      vertex   -5.936239e+01 -3.736937e+01 8.861667e-15
      vertex   -6.017761e+01 -3.746727e+01 3.000000e+00
      vertex   -5.936239e+01 -3.736937e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.985059e-01 9.800997e-01 -9.403127e-16
    outer loop
      vertex   -5.936239e+01 -3.736937e+01 8.861667e-15
      vertex   -5.936239e+01 -3.736937e+01 3.000000e+00
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.774117e-01 9.607511e-01 -6.570428e-16
    outer loop
      vertex   -5.778182e+01 -3.698204e+01 3.000000e+00
      vertex   -5.778182e+01 -3.698204e+01 8.768546e-15
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.774117e-01 9.607511e-01 -6.570428e-16
    outer loop
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
      vertex   -5.778182e+01 -3.698204e+01 8.768546e-15
      vertex   -5.856146e+01 -3.720716e+01 8.822668e-15
    endloop
  endfacet
  facet normal -1.985059e-01 9.800997e-01 -4.701563e-16
    outer loop
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
      vertex   -5.856146e+01 -3.720716e+01 8.822668e-15
      vertex   -5.936239e+01 -3.736937e+01 8.861667e-15
    endloop
  endfacet
  facet normal -3.557205e-01 9.345924e-01 -8.425154e-16
    outer loop
      vertex   -5.778182e+01 -3.698204e+01 3.000000e+00
      vertex   -5.703026e+01 -3.669598e+01 3.000000e+00
      vertex   -5.778182e+01 -3.698204e+01 8.768546e-15
    endloop
  endfacet
  facet normal -3.557205e-01 9.345924e-01 1.369106e-15
    outer loop
      vertex   -5.778182e+01 -3.698204e+01 8.768546e-15
      vertex   -5.703026e+01 -3.669598e+01 3.000000e+00
      vertex   -5.703026e+01 -3.669598e+01 8.699775e-15
    endloop
  endfacet
  facet normal -4.331164e-01 9.013380e-01 1.108972e-15
    outer loop
      vertex   -5.703026e+01 -3.669598e+01 8.699775e-15
      vertex   -5.703026e+01 -3.669598e+01 3.000000e+00
      vertex   -5.631336e+01 -3.635149e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.331164e-01 9.013380e-01 -1.023611e-15
    outer loop
      vertex   -5.703026e+01 -3.669598e+01 8.699775e-15
      vertex   -5.631336e+01 -3.635149e+01 3.000000e+00
      vertex   -5.631336e+01 -3.635149e+01 8.616954e-15
    endloop
  endfacet
  facet normal -5.091713e-01 8.606652e-01 -1.205960e-15
    outer loop
      vertex   -5.631336e+01 -3.635149e+01 8.616954e-15
      vertex   -5.631336e+01 -3.635149e+01 3.000000e+00
      vertex   -5.563735e+01 -3.595156e+01 8.520806e-15
    endloop
  endfacet
  facet normal -5.091713e-01 8.606652e-01 0.000000e+00
    outer loop
      vertex   -5.563735e+01 -3.595156e+01 8.520806e-15
      vertex   -5.631336e+01 -3.635149e+01 3.000000e+00
      vertex   -5.563735e+01 -3.595156e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.833208e-01 8.122419e-01 0.000000e+00
    outer loop
      vertex   -5.563735e+01 -3.595156e+01 8.520806e-15
      vertex   -5.563735e+01 -3.595156e+01 3.000000e+00
      vertex   -5.500815e+01 -3.549969e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.548414e-01 7.557663e-01 2.390383e-16
    outer loop
      vertex   -5.443124e+01 -3.499983e+01 3.000000e+00
      vertex   -5.443124e+01 -3.499983e+01 8.291994e-15
      vertex   -5.500815e+01 -3.549969e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.548414e-01 7.557663e-01 -1.550976e-15
    outer loop
      vertex   -5.500815e+01 -3.549969e+01 3.000000e+00
      vertex   -5.443124e+01 -3.499983e+01 8.291994e-15
      vertex   -5.500815e+01 -3.549969e+01 8.412170e-15
    endloop
  endfacet
  facet normal -5.833208e-01 8.122419e-01 -1.381581e-15
    outer loop
      vertex   -5.500815e+01 -3.549969e+01 3.000000e+00
      vertex   -5.500815e+01 -3.549969e+01 8.412170e-15
      vertex   -5.563735e+01 -3.595156e+01 8.520806e-15
    endloop
  endfacet
  facet normal -7.228377e-01 6.910178e-01 -7.536454e-17
    outer loop
      vertex   -5.443124e+01 -3.499983e+01 3.000000e+00
      vertex   -5.391166e+01 -3.445632e+01 3.000000e+00
      vertex   -5.443124e+01 -3.499983e+01 8.291994e-15
    endloop
  endfacet
  facet normal -7.228377e-01 6.910178e-01 1.648812e-15
    outer loop
      vertex   -5.443124e+01 -3.499983e+01 8.291994e-15
      vertex   -5.391166e+01 -3.445632e+01 3.000000e+00
      vertex   -5.391166e+01 -3.445632e+01 8.161328e-15
    endloop
  endfacet
  facet normal -7.862416e-01 6.179192e-01 1.463527e-15
    outer loop
      vertex   -5.391166e+01 -3.445632e+01 8.161328e-15
      vertex   -5.391166e+01 -3.445632e+01 3.000000e+00
      vertex   -5.345394e+01 -3.387392e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.862416e-01 6.179192e-01 2.947659e-15
    outer loop
      vertex   -5.391166e+01 -3.445632e+01 8.161328e-15
      vertex   -5.345394e+01 -3.387392e+01 3.000000e+00
      vertex   -5.345394e+01 -3.387392e+01 8.021311e-15
    endloop
  endfacet
  facet normal -8.438325e-01 5.366067e-01 2.541880e-15
    outer loop
      vertex   -5.345394e+01 -3.387392e+01 8.021311e-15
      vertex   -5.345394e+01 -3.387392e+01 3.000000e+00
      vertex   -5.306209e+01 -3.325772e+01 7.873166e-15
    endloop
  endfacet
  facet normal -8.438325e-01 5.366067e-01 -7.276568e-16
    outer loop
      vertex   -5.306209e+01 -3.325772e+01 7.873166e-15
      vertex   -5.345394e+01 -3.387392e+01 3.000000e+00
      vertex   -5.306209e+01 -3.325772e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.942841e-01 4.474997e-01 -1.058198e-15
    outer loop
      vertex   -5.306209e+01 -3.325772e+01 7.873166e-15
      vertex   -5.306209e+01 -3.325772e+01 3.000000e+00
      vertex   -5.273951e+01 -3.261308e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.362409e-01 3.513589e-01 4.160925e-16
    outer loop
      vertex   -5.248902e+01 -3.194563e+01 3.000000e+00
      vertex   -5.248902e+01 -3.194563e+01 7.557722e-15
      vertex   -5.273951e+01 -3.261308e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.362409e-01 3.513589e-01 -1.385279e-15
    outer loop
      vertex   -5.273951e+01 -3.261308e+01 3.000000e+00
      vertex   -5.248902e+01 -3.194563e+01 7.557722e-15
      vertex   -5.273951e+01 -3.261308e+01 7.718185e-15
    endloop
  endfacet
  facet normal -8.942841e-01 4.474997e-01 -1.058198e-15
    outer loop
      vertex   -5.273951e+01 -3.261308e+01 3.000000e+00
      vertex   -5.273951e+01 -3.261308e+01 7.718185e-15
      vertex   -5.306209e+01 -3.325772e+01 7.873166e-15
    endloop
  endfacet
  facet normal -9.684219e-01 2.493171e-01 2.952507e-16
    outer loop
      vertex   -5.248902e+01 -3.194563e+01 3.000000e+00
      vertex   -5.231282e+01 -3.126120e+01 3.000000e+00
      vertex   -5.248902e+01 -3.194563e+01 7.557722e-15
    endloop
  endfacet
  facet normal -9.684219e-01 2.493171e-01 2.945435e-16
    outer loop
      vertex   -5.248902e+01 -3.194563e+01 7.557722e-15
      vertex   -5.231282e+01 -3.126120e+01 3.000000e+00
      vertex   -5.231282e+01 -3.126120e+01 7.393176e-15
    endloop
  endfacet
  facet normal -9.897417e-01 1.428684e-01 1.691902e-16
    outer loop
      vertex   -5.231282e+01 -3.126120e+01 7.393176e-15
      vertex   -5.231282e+01 -3.126120e+01 3.000000e+00
      vertex   -5.221244e+01 -3.056577e+01 7.225984e-15
    endloop
  endfacet
  facet normal -9.897417e-01 1.428684e-01 -2.174989e-15
    outer loop
      vertex   -5.221244e+01 -3.056577e+01 7.225984e-15
      vertex   -5.231282e+01 -3.126120e+01 3.000000e+00
      vertex   -5.221244e+01 -3.056577e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.937782e-01 -1.113771e-01 -0.000000e+00
    outer loop
      vertex   -7.273699e+01 -3.047472e+01 3.000000e+00
      vertex   -7.273699e+01 -3.047472e+01 7.204093e-15
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.937782e-01 -1.113771e-01 -0.000000e+00
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
      vertex   -7.273699e+01 -3.047472e+01 7.204093e-15
      vertex   -7.263081e+01 -3.142212e+01 7.431864e-15
    endloop
  endfacet
  facet normal -9.753279e-01 -2.207609e-01 -0.000000e+00
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
      vertex   -7.263081e+01 -3.142212e+01 7.431864e-15
      vertex   -7.241928e+01 -3.235665e+01 7.656537e-15
    endloop
  endfacet
  facet normal -9.753279e-01 -2.207609e-01 -5.214129e-16
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
      vertex   -7.241928e+01 -3.235665e+01 7.656537e-15
      vertex   -7.241928e+01 -3.235665e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.453521e-01 -3.260512e-01 -7.722443e-16
    outer loop
      vertex   -7.241928e+01 -3.235665e+01 3.000000e+00
      vertex   -7.241928e+01 -3.235665e+01 7.656537e-15
      vertex   -7.210432e+01 -3.326983e+01 7.876079e-15
    endloop
  endfacet
  facet normal -9.453521e-01 -3.260512e-01 0.000000e+00
    outer loop
      vertex   -7.241928e+01 -3.235665e+01 3.000000e+00
      vertex   -7.210432e+01 -3.326983e+01 7.876079e-15
      vertex   -7.210432e+01 -3.326983e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.049223e-01 -4.255767e-01 -0.000000e+00
    outer loop
      vertex   -7.210432e+01 -3.326983e+01 3.000000e+00
      vertex   -7.210432e+01 -3.326983e+01 7.876079e-15
      vertex   -7.168879e+01 -3.415340e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.049223e-01 -4.255767e-01 -1.007968e-15
    outer loop
      vertex   -7.168879e+01 -3.415340e+01 3.000000e+00
      vertex   -7.210432e+01 -3.326983e+01 7.876079e-15
      vertex   -7.168879e+01 -3.415340e+01 8.088501e-15
    endloop
  endfacet
  facet normal -8.553540e-01 -5.180440e-01 -1.226975e-15
    outer loop
      vertex   -7.168879e+01 -3.415340e+01 3.000000e+00
      vertex   -7.168879e+01 -3.415340e+01 8.088501e-15
      vertex   -7.117645e+01 -3.499934e+01 8.291878e-15
    endloop
  endfacet
  facet normal -8.553540e-01 -5.180440e-01 -1.234828e-15
    outer loop
      vertex   -7.168879e+01 -3.415340e+01 3.000000e+00
      vertex   -7.117645e+01 -3.499934e+01 8.291878e-15
      vertex   -7.117645e+01 -3.499934e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.980723e-01 -6.025617e-01 -1.427153e-15
    outer loop
      vertex   -7.117645e+01 -3.499934e+01 3.000000e+00
      vertex   -7.117645e+01 -3.499934e+01 8.291878e-15
      vertex   -7.057193e+01 -3.580001e+01 8.484369e-15
    endloop
  endfacet
  facet normal -7.980723e-01 -6.025617e-01 -1.438618e-15
    outer loop
      vertex   -7.117645e+01 -3.499934e+01 3.000000e+00
      vertex   -7.057193e+01 -3.580001e+01 8.484369e-15
      vertex   -7.057193e+01 -3.580001e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.344933e-01 -6.786160e-01 -1.607285e-15
    outer loop
      vertex   -7.057193e+01 -3.580001e+01 3.000000e+00
      vertex   -7.057193e+01 -3.580001e+01 8.484369e-15
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
    endloop
  endfacet
  facet normal -7.344933e-01 -6.786160e-01 1.889307e-15
    outer loop
      vertex   -7.057193e+01 -3.580001e+01 3.000000e+00
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
      vertex   -6.988071e+01 -3.654814e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.659327e-01 -7.460118e-01 1.387580e-15
    outer loop
      vertex   -6.988071e+01 -3.654814e+01 3.000000e+00
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
      vertex   -6.910906e+01 -3.723696e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.659327e-01 -7.460118e-01 -1.766911e-15
    outer loop
      vertex   -6.910906e+01 -3.723696e+01 3.000000e+00
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
      vertex   -6.910906e+01 -3.723696e+01 8.829833e-15
    endloop
  endfacet
  facet normal -5.935492e-01 -8.047977e-01 -1.906144e-15
    outer loop
      vertex   -6.910906e+01 -3.723696e+01 3.000000e+00
      vertex   -6.910906e+01 -3.723696e+01 8.829833e-15
      vertex   -6.826396e+01 -3.786023e+01 8.979677e-15
    endloop
  endfacet
  facet normal -5.935492e-01 -8.047977e-01 -1.022034e-15
    outer loop
      vertex   -6.910906e+01 -3.723696e+01 3.000000e+00
      vertex   -6.826396e+01 -3.786023e+01 8.979677e-15
      vertex   -6.826396e+01 -3.786023e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.183171e-01 -8.551885e-01 -1.595743e-15
    outer loop
      vertex   -6.826396e+01 -3.786023e+01 3.000000e+00
      vertex   -6.826396e+01 -3.786023e+01 8.979677e-15
      vertex   -6.735306e+01 -3.841231e+01 9.112405e-15
    endloop
  endfacet
  facet normal -5.183171e-01 -8.551885e-01 -2.015168e-15
    outer loop
      vertex   -6.826396e+01 -3.786023e+01 3.000000e+00
      vertex   -6.735306e+01 -3.841231e+01 9.112405e-15
      vertex   -6.735306e+01 -3.841231e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.410247e-01 -8.974950e-01 -2.125695e-15
    outer loop
      vertex   -6.735306e+01 -3.841231e+01 3.000000e+00
      vertex   -6.735306e+01 -3.841231e+01 9.112405e-15
      vertex   -6.638463e+01 -3.888820e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.410247e-01 -8.974950e-01 -0.000000e+00
    outer loop
      vertex   -6.638463e+01 -3.888820e+01 3.000000e+00
      vertex   -6.735306e+01 -3.841231e+01 9.112405e-15
      vertex   -6.638463e+01 -3.888820e+01 9.226814e-15
    endloop
  endfacet
  facet normal -3.622869e-01 -9.320666e-01 -0.000000e+00
    outer loop
      vertex   -6.638463e+01 -3.888820e+01 3.000000e+00
      vertex   -6.638463e+01 -3.888820e+01 9.226814e-15
      vertex   -6.536742e+01 -3.928358e+01 9.321869e-15
    endloop
  endfacet
  facet normal -3.622869e-01 -9.320666e-01 -4.916936e-16
    outer loop
      vertex   -6.638463e+01 -3.888820e+01 3.000000e+00
      vertex   -6.536742e+01 -3.928358e+01 9.321869e-15
      vertex   -6.536742e+01 -3.928358e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.825685e-01 -9.592471e-01 -9.334403e-16
    outer loop
      vertex   -6.536742e+01 -3.928358e+01 3.000000e+00
      vertex   -6.536742e+01 -3.928358e+01 9.321869e-15
      vertex   -6.431066e+01 -3.959487e+01 9.396709e-15
    endloop
  endfacet
  facet normal -2.825685e-01 -9.592471e-01 -9.237911e-16
    outer loop
      vertex   -6.536742e+01 -3.928358e+01 3.000000e+00
      vertex   -6.431066e+01 -3.959487e+01 9.396709e-15
      vertex   -6.431066e+01 -3.959487e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.022118e-01 -9.793418e-01 -1.361680e-15
    outer loop
      vertex   -6.431066e+01 -3.959487e+01 3.000000e+00
      vertex   -6.431066e+01 -3.959487e+01 9.396709e-15
      vertex   -6.322391e+01 -3.981926e+01 9.450655e-15
    endloop
  endfacet
  facet normal -2.022118e-01 -9.793418e-01 -1.834239e-15
    outer loop
      vertex   -6.431066e+01 -3.959487e+01 3.000000e+00
      vertex   -6.322391e+01 -3.981926e+01 9.450655e-15
      vertex   -6.322391e+01 -3.981926e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.214650e-01 -9.925957e-01 -2.063252e-15
    outer loop
      vertex   -6.322391e+01 -3.981926e+01 3.000000e+00
      vertex   -6.322391e+01 -3.981926e+01 9.450655e-15
      vertex   -6.211701e+01 -3.995471e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.214650e-01 -9.925957e-01 -0.000000e+00
    outer loop
      vertex   -6.211701e+01 -3.995471e+01 3.000000e+00
      vertex   -6.322391e+01 -3.981926e+01 9.450655e-15
      vertex   -6.211701e+01 -3.995471e+01 9.483220e-15
    endloop
  endfacet
  facet normal -4.051002e-02 -9.991791e-01 -0.000000e+00
    outer loop
      vertex   -6.211701e+01 -3.995471e+01 3.000000e+00
      vertex   -6.211701e+01 -3.995471e+01 9.483220e-15
      vertex   -6.100000e+01 -4.000000e+01 9.494108e-15
    endloop
  endfacet
  facet normal -4.051002e-02 -9.991791e-01 9.517275e-17
    outer loop
      vertex   -6.211701e+01 -3.995471e+01 3.000000e+00
      vertex   -6.100000e+01 -4.000000e+01 9.494108e-15
      vertex   -6.100000e+01 -4.000000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 4.051002e-02 -9.991791e-01 -9.594701e-17
    outer loop
      vertex   -6.100000e+01 -4.000000e+01 3.000000e+00
      vertex   -6.100000e+01 -4.000000e+01 9.494108e-15
      vertex   -5.988299e+01 -3.995471e+01 9.483220e-15
    endloop
  endfacet
  facet normal 4.051002e-02 -9.991791e-01 -1.924145e-16
    outer loop
      vertex   -6.100000e+01 -4.000000e+01 3.000000e+00
      vertex   -5.988299e+01 -3.995471e+01 9.483220e-15
      vertex   -5.988299e+01 -3.995471e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.214650e-01 -9.925957e-01 -5.753738e-16
    outer loop
      vertex   -5.988299e+01 -3.995471e+01 3.000000e+00
      vertex   -5.988299e+01 -3.995471e+01 9.483220e-15
      vertex   -5.877609e+01 -3.981926e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.214650e-01 -9.925957e-01 -8.630607e-16
    outer loop
      vertex   -5.877609e+01 -3.981926e+01 3.000000e+00
      vertex   -5.988299e+01 -3.995471e+01 9.483220e-15
      vertex   -5.877609e+01 -3.981926e+01 9.450655e-15
    endloop
  endfacet
  facet normal 2.022118e-01 -9.793418e-01 -1.436802e-15
    outer loop
      vertex   -5.877609e+01 -3.981926e+01 3.000000e+00
      vertex   -5.877609e+01 -3.981926e+01 9.450655e-15
      vertex   -5.768934e+01 -3.959487e+01 9.396709e-15
    endloop
  endfacet
  facet normal 2.022118e-01 -9.793418e-01 -4.752346e-16
    outer loop
      vertex   -5.877609e+01 -3.981926e+01 3.000000e+00
      vertex   -5.768934e+01 -3.959487e+01 9.396709e-15
      vertex   -5.768934e+01 -3.959487e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.825685e-01 -9.592471e-01 -6.692567e-16
    outer loop
      vertex   -5.768934e+01 -3.959487e+01 3.000000e+00
      vertex   -5.768934e+01 -3.959487e+01 9.396709e-15
      vertex   -5.663258e+01 -3.928358e+01 9.321869e-15
    endloop
  endfacet
  facet normal 2.825685e-01 -9.592471e-01 -4.031089e-16
    outer loop
      vertex   -5.768934e+01 -3.959487e+01 3.000000e+00
      vertex   -5.663258e+01 -3.928358e+01 9.321869e-15
      vertex   -5.663258e+01 -3.928358e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.622869e-01 -9.320666e-01 -1.224694e-15
    outer loop
      vertex   -5.663258e+01 -3.928358e+01 3.000000e+00
      vertex   -5.663258e+01 -3.928358e+01 9.321869e-15
      vertex   -5.561537e+01 -3.888820e+01 9.226814e-15
    endloop
  endfacet
  facet normal 3.622869e-01 -9.320666e-01 -3.560540e-16
    outer loop
      vertex   -5.663258e+01 -3.928358e+01 3.000000e+00
      vertex   -5.561537e+01 -3.888820e+01 9.226814e-15
      vertex   -5.561537e+01 -3.888820e+01 3.000000e+00
    endloop
  endfacet
  facet normal 4.410247e-01 -8.974950e-01 -1.007974e-15
    outer loop
      vertex   -5.561537e+01 -3.888820e+01 3.000000e+00
      vertex   -5.561537e+01 -3.888820e+01 9.226814e-15
      vertex   -5.464694e+01 -3.841231e+01 3.000000e+00
    endloop
  endfacet
  facet normal 4.410247e-01 -8.974950e-01 2.162277e-15
    outer loop
      vertex   -5.464694e+01 -3.841231e+01 3.000000e+00
      vertex   -5.561537e+01 -3.888820e+01 9.226814e-15
      vertex   -5.464694e+01 -3.841231e+01 9.112405e-15
    endloop
  endfacet
  facet normal 5.183171e-01 -8.551885e-01 1.595743e-15
    outer loop
      vertex   -5.464694e+01 -3.841231e+01 3.000000e+00
      vertex   -5.464694e+01 -3.841231e+01 9.112405e-15
      vertex   -5.373604e+01 -3.786023e+01 8.979677e-15
    endloop
  endfacet
  facet normal 5.183171e-01 -8.551885e-01 -4.169313e-16
    outer loop
      vertex   -5.464694e+01 -3.841231e+01 3.000000e+00
      vertex   -5.373604e+01 -3.786023e+01 8.979677e-15
      vertex   -5.373604e+01 -3.786023e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.935492e-01 -8.047977e-01 -9.054696e-16
    outer loop
      vertex   -5.373604e+01 -3.786023e+01 3.000000e+00
      vertex   -5.373604e+01 -3.786023e+01 8.979677e-15
      vertex   -5.289094e+01 -3.723696e+01 8.829833e-15
    endloop
  endfacet
  facet normal 5.935492e-01 -8.047977e-01 2.431737e-15
    outer loop
      vertex   -5.373604e+01 -3.786023e+01 3.000000e+00
      vertex   -5.289094e+01 -3.723696e+01 8.829833e-15
      vertex   -5.289094e+01 -3.723696e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.659327e-01 -7.460118e-01 1.956576e-15
    outer loop
      vertex   -5.289094e+01 -3.723696e+01 3.000000e+00
      vertex   -5.289094e+01 -3.723696e+01 8.829833e-15
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.659327e-01 -7.460118e-01 1.956576e-15
    outer loop
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
      vertex   -5.289094e+01 -3.723696e+01 8.829833e-15
      vertex   -5.211929e+01 -3.654814e+01 8.664231e-15
    endloop
  endfacet
  facet normal 7.344933e-01 -6.786160e-01 1.474941e-15
    outer loop
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
      vertex   -5.211929e+01 -3.654814e+01 8.664231e-15
      vertex   -5.142807e+01 -3.580001e+01 8.484369e-15
    endloop
  endfacet
  facet normal 7.344933e-01 -6.786160e-01 -2.543298e-16
    outer loop
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
      vertex   -5.142807e+01 -3.580001e+01 8.484369e-15
      vertex   -5.142807e+01 -3.580001e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.980723e-01 -6.025617e-01 -9.261243e-16
    outer loop
      vertex   -5.142807e+01 -3.580001e+01 3.000000e+00
      vertex   -5.142807e+01 -3.580001e+01 8.484369e-15
      vertex   -5.082355e+01 -3.499934e+01 8.291878e-15
    endloop
  endfacet
  facet normal 7.980723e-01 -6.025617e-01 -9.221908e-16
    outer loop
      vertex   -5.142807e+01 -3.580001e+01 3.000000e+00
      vertex   -5.082355e+01 -3.499934e+01 8.291878e-15
      vertex   -5.082355e+01 -3.499934e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.553540e-01 -5.180440e-01 -1.597821e-15
    outer loop
      vertex   -5.082355e+01 -3.499934e+01 3.000000e+00
      vertex   -5.082355e+01 -3.499934e+01 8.291878e-15
      vertex   -5.031121e+01 -3.415340e+01 8.088501e-15
    endloop
  endfacet
  facet normal 8.553540e-01 -5.180440e-01 2.450946e-15
    outer loop
      vertex   -5.082355e+01 -3.499934e+01 3.000000e+00
      vertex   -5.031121e+01 -3.415340e+01 8.088501e-15
      vertex   -5.031121e+01 -3.415340e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.049223e-01 -4.255767e-01 2.015936e-15
    outer loop
      vertex   -5.031121e+01 -3.415340e+01 3.000000e+00
      vertex   -5.031121e+01 -3.415340e+01 8.088501e-15
      vertex   -4.989568e+01 -3.326983e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.049223e-01 -4.255767e-01 3.023904e-15
    outer loop
      vertex   -4.989568e+01 -3.326983e+01 3.000000e+00
      vertex   -5.031121e+01 -3.415340e+01 8.088501e-15
      vertex   -4.989568e+01 -3.326983e+01 7.876079e-15
    endloop
  endfacet
  facet normal 9.453521e-01 -3.260512e-01 2.316733e-15
    outer loop
      vertex   -4.989568e+01 -3.326983e+01 3.000000e+00
      vertex   -4.989568e+01 -3.326983e+01 7.876079e-15
      vertex   -4.958072e+01 -3.235665e+01 7.656537e-15
    endloop
  endfacet
  facet normal 9.453521e-01 -3.260512e-01 -7.087569e-16
    outer loop
      vertex   -4.989568e+01 -3.326983e+01 3.000000e+00
      vertex   -4.958072e+01 -3.235665e+01 7.656537e-15
      vertex   -4.958072e+01 -3.235665e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.753279e-01 -2.207609e-01 -1.264307e-15
    outer loop
      vertex   -4.958072e+01 -3.235665e+01 3.000000e+00
      vertex   -4.958072e+01 -3.235665e+01 7.656537e-15
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
    endloop
  endfacet
  facet normal 9.753279e-01 -2.207609e-01 -9.945468e-16
    outer loop
      vertex   -4.958072e+01 -3.235665e+01 3.000000e+00
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
      vertex   -4.936919e+01 -3.142212e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.937782e-01 -1.113771e-01 -1.694255e-15
    outer loop
      vertex   -4.936919e+01 -3.142212e+01 3.000000e+00
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
      vertex   -4.926301e+01 -3.047472e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.937782e-01 -1.113771e-01 5.275879e-16
    outer loop
      vertex   -4.926301e+01 -3.047472e+01 3.000000e+00
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
      vertex   -4.926301e+01 -3.047472e+01 7.204093e-15
    endloop
  endfacet
  facet normal -6.869601e-01 -7.266951e-01 -2.487628e-15
    outer loop
      vertex   -5.221244e+01 -3.056577e+01 7.225984e-15
      vertex   -5.221244e+01 -3.056577e+01 3.000000e+00
      vertex   -5.278018e+01 -3.002907e+01 -1.317018e-15
    endloop
  endfacet
  facet normal -6.869601e-01 -7.266951e-01 0.000000e+00
    outer loop
      vertex   -5.278018e+01 -3.002907e+01 -1.317018e-15
      vertex   -5.221244e+01 -3.056577e+01 3.000000e+00
      vertex   -5.278018e+01 -3.002907e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.777227e-01 -4.791689e-01 0.000000e+00
    outer loop
      vertex   -5.278018e+01 -3.002907e+01 -1.317018e-15
      vertex   -5.278018e+01 -3.002907e+01 3.000000e+00
      vertex   -5.315453e+01 -2.934334e+01 -1.540505e-15
    endloop
  endfacet
  facet normal -8.777227e-01 -4.791689e-01 0.000000e+00
    outer loop
      vertex   -5.315453e+01 -2.934334e+01 -1.540505e-15
      vertex   -5.278018e+01 -3.002907e+01 3.000000e+00
      vertex   -5.315453e+01 -2.934334e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.827673e-01 -1.848473e-01 0.000000e+00
    outer loop
      vertex   -5.315453e+01 -2.934334e+01 -1.540505e-15
      vertex   -5.315453e+01 -2.934334e+01 3.000000e+00
      vertex   -5.329895e+01 -2.857554e+01 -1.787026e-15
    endloop
  endfacet
  facet normal -9.827673e-01 -1.848473e-01 0.000000e+00
    outer loop
      vertex   -5.329895e+01 -2.857554e+01 -1.787026e-15
      vertex   -5.315453e+01 -2.934334e+01 3.000000e+00
      vertex   -5.329895e+01 -2.857554e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.918352e-01 1.275265e-01 0.000000e+00
    outer loop
      vertex   -5.329895e+01 -2.857554e+01 -1.787026e-15
      vertex   -5.329895e+01 -2.857554e+01 3.000000e+00
      vertex   -5.319932e+01 -2.780066e+01 -2.032505e-15
    endloop
  endfacet
  facet normal -9.918352e-01 1.275265e-01 1.510217e-16
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 -2.032505e-15
      vertex   -5.329895e+01 -2.857554e+01 3.000000e+00
      vertex   -5.319932e+01 -2.780066e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.040409e-01 4.274460e-01 5.061978e-16
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 -2.032505e-15
      vertex   -5.319932e+01 -2.780066e+01 3.000000e+00
      vertex   -5.286537e+01 -2.709436e+01 -2.252968e-15
    endloop
  endfacet
  facet normal -9.040409e-01 4.274460e-01 5.061978e-16
    outer loop
      vertex   -5.286537e+01 -2.709436e+01 -2.252968e-15
      vertex   -5.319932e+01 -2.780066e+01 3.000000e+00
      vertex   -5.286537e+01 -2.709436e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.279583e-01 6.856214e-01 8.119388e-16
    outer loop
      vertex   -5.286537e+01 -2.709436e+01 -2.252968e-15
      vertex   -5.286537e+01 -2.709436e+01 3.000000e+00
      vertex   -5.232972e+01 -2.652563e+01 -2.426886e-15
    endloop
  endfacet
  facet normal -7.279583e-01 6.856214e-01 -1.724152e-15
    outer loop
      vertex   -5.232972e+01 -2.652563e+01 -2.426886e-15
      vertex   -5.286537e+01 -2.709436e+01 3.000000e+00
      vertex   -5.232972e+01 -2.652563e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.807837e-01 8.768392e-01 -1.138724e-15
    outer loop
      vertex   -5.232972e+01 -2.652563e+01 -2.426886e-15
      vertex   -5.232972e+01 -2.652563e+01 3.000000e+00
      vertex   -5.164467e+01 -2.615002e+01 -2.537274e-15
    endloop
  endfacet
  facet normal -4.807837e-01 8.768392e-01 -2.277449e-15
    outer loop
      vertex   -5.164467e+01 -2.615002e+01 -2.537274e-15
      vertex   -5.232972e+01 -2.652563e+01 3.000000e+00
      vertex   -5.164467e+01 -2.615002e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.866559e-01 9.824254e-01 -8.841799e-16
    outer loop
      vertex   -5.164467e+01 -2.615002e+01 -2.537274e-15
      vertex   -5.164467e+01 -2.615002e+01 3.000000e+00
      vertex   -5.087714e+01 -2.600419e+01 -2.573350e-15
    endloop
  endfacet
  facet normal -1.866559e-01 9.824254e-01 0.000000e+00
    outer loop
      vertex   -5.087714e+01 -2.600419e+01 -2.573350e-15
      vertex   -5.164467e+01 -2.615002e+01 3.000000e+00
      vertex   -5.087714e+01 -2.600419e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.257006e-01 9.920682e-01 -0.000000e+00
    outer loop
      vertex   -5.087714e+01 -2.600419e+01 -2.573350e-15
      vertex   -5.087714e+01 -2.600419e+01 3.000000e+00
      vertex   -5.010208e+01 -2.610239e+01 -2.531593e-15
    endloop
  endfacet
  facet normal 1.257006e-01 9.920682e-01 -2.349690e-15
    outer loop
      vertex   -5.010208e+01 -2.610239e+01 -2.531593e-15
      vertex   -5.087714e+01 -2.600419e+01 3.000000e+00
      vertex   -5.010208e+01 -2.610239e+01 3.000000e+00
    endloop
  endfacet
  facet normal 4.257813e-01 9.048261e-01 -2.143059e-15
    outer loop
      vertex   -5.010208e+01 -2.610239e+01 -2.531593e-15
      vertex   -5.010208e+01 -2.610239e+01 3.000000e+00
      vertex   -4.939517e+01 -2.643504e+01 -2.416079e-15
    endloop
  endfacet
  facet normal 4.257813e-01 9.048261e-01 -1.071529e-15
    outer loop
      vertex   -4.939517e+01 -2.643504e+01 -2.416079e-15
      vertex   -5.010208e+01 -2.610239e+01 3.000000e+00
      vertex   -4.939517e+01 -2.643504e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.842803e-01 7.292191e-01 -8.635689e-16
    outer loop
      vertex   -4.939517e+01 -2.643504e+01 -2.416079e-15
      vertex   -4.939517e+01 -2.643504e+01 3.000000e+00
      vertex   -4.882546e+01 -2.696964e+01 -2.238091e-15
    endloop
  endfacet
  facet normal 6.842803e-01 7.292191e-01 -8.635689e-16
    outer loop
      vertex   -4.882546e+01 -2.696964e+01 -2.238091e-15
      vertex   -4.939517e+01 -2.643504e+01 3.000000e+00
      vertex   -4.882546e+01 -2.696964e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.759528e-01 4.823968e-01 -5.712726e-16
    outer loop
      vertex   -4.882546e+01 -2.696964e+01 -2.238091e-15
      vertex   -4.882546e+01 -2.696964e+01 3.000000e+00
      vertex   -4.844858e+01 -2.765399e+01 -2.015010e-15
    endloop
  endfacet
  facet normal 8.759528e-01 4.823968e-01 -5.712726e-16
    outer loop
      vertex   -4.844858e+01 -2.765399e+01 -2.015010e-15
      vertex   -4.882546e+01 -2.696964e+01 3.000000e+00
      vertex   -4.844858e+01 -2.765399e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.820801e-01 1.884639e-01 -2.231860e-16
    outer loop
      vertex   -4.844858e+01 -2.765399e+01 -2.015010e-15
      vertex   -4.844858e+01 -2.765399e+01 3.000000e+00
      vertex   -4.830134e+01 -2.842126e+01 -1.768622e-15
    endloop
  endfacet
  facet normal 9.820801e-01 1.884639e-01 -4.463721e-16
    outer loop
      vertex   -4.830134e+01 -2.842126e+01 -1.768622e-15
      vertex   -4.844858e+01 -2.765399e+01 3.000000e+00
      vertex   -4.830134e+01 -2.842126e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.922979e-01 -1.238744e-01 2.933935e-16
    outer loop
      vertex   -4.830134e+01 -2.842126e+01 -1.768622e-15
      vertex   -4.830134e+01 -2.842126e+01 3.000000e+00
      vertex   -4.839812e+01 -2.919650e+01 -1.522989e-15
    endloop
  endfacet
  facet normal 9.922979e-01 -1.238744e-01 1.466967e-16
    outer loop
      vertex   -4.839812e+01 -2.919650e+01 -1.522989e-15
      vertex   -4.830134e+01 -2.842126e+01 3.000000e+00
      vertex   -4.839812e+01 -2.919650e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.056083e-01 -4.241151e-01 5.022532e-16
    outer loop
      vertex   -4.839812e+01 -2.919650e+01 -1.522989e-15
      vertex   -4.839812e+01 -2.919650e+01 3.000000e+00
      vertex   -4.872946e+01 -2.990402e+01 -1.302100e-15
    endloop
  endfacet
  facet normal 9.056083e-01 -4.241151e-01 -1.140405e-15
    outer loop
      vertex   -4.872946e+01 -2.990402e+01 -1.302100e-15
      vertex   -4.839812e+01 -2.919650e+01 3.000000e+00
      vertex   -4.872946e+01 -2.990402e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.304773e-01 -6.829369e-01 -1.125983e-16
    outer loop
      vertex   -4.872946e+01 -2.990402e+01 -1.302100e-15
      vertex   -4.872946e+01 -2.990402e+01 3.000000e+00
      vertex   -4.926301e+01 -3.047472e+01 7.204093e-15
    endloop
  endfacet
  facet normal 7.304773e-01 -6.829369e-01 3.235039e-15
    outer loop
      vertex   -4.926301e+01 -3.047472e+01 7.204093e-15
      vertex   -4.872946e+01 -2.990402e+01 3.000000e+00
      vertex   -4.926301e+01 -3.047472e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.304773e-01 -6.829369e-01 0.000000e+00
    outer loop
      vertex   -7.273699e+01 -3.047472e+01 7.204093e-15
      vertex   -7.273699e+01 -3.047472e+01 3.000000e+00
      vertex   -7.327054e+01 -2.990402e+01 -1.358072e-15
    endloop
  endfacet
  facet normal -7.304773e-01 -6.829369e-01 0.000000e+00
    outer loop
      vertex   -7.327054e+01 -2.990402e+01 -1.358072e-15
      vertex   -7.273699e+01 -3.047472e+01 3.000000e+00
      vertex   -7.327054e+01 -2.990402e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.056083e-01 -4.241151e-01 0.000000e+00
    outer loop
      vertex   -7.327054e+01 -2.990402e+01 -1.358072e-15
      vertex   -7.327054e+01 -2.990402e+01 3.000000e+00
      vertex   -7.360188e+01 -2.919650e+01 -1.587918e-15
    endloop
  endfacet
  facet normal -9.056083e-01 -4.241151e-01 0.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 -1.587918e-15
      vertex   -7.327054e+01 -2.990402e+01 3.000000e+00
      vertex   -7.360188e+01 -2.919650e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.922979e-01 -1.238744e-01 0.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 -1.587918e-15
      vertex   -7.360188e+01 -2.919650e+01 3.000000e+00
      vertex   -7.369866e+01 -2.842126e+01 -1.836166e-15
    endloop
  endfacet
  facet normal -9.922979e-01 -1.238744e-01 0.000000e+00
    outer loop
      vertex   -7.369866e+01 -2.842126e+01 -1.836166e-15
      vertex   -7.360188e+01 -2.919650e+01 3.000000e+00
      vertex   -7.369866e+01 -2.842126e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.820801e-01 1.884639e-01 0.000000e+00
    outer loop
      vertex   -7.369866e+01 -2.842126e+01 -1.836166e-15
      vertex   -7.369866e+01 -2.842126e+01 3.000000e+00
      vertex   -7.355142e+01 -2.765399e+01 -2.078574e-15
    endloop
  endfacet
  facet normal -9.820801e-01 1.884639e-01 0.000000e+00
    outer loop
      vertex   -7.355142e+01 -2.765399e+01 -2.078574e-15
      vertex   -7.369866e+01 -2.842126e+01 3.000000e+00
      vertex   -7.355142e+01 -2.765399e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.759528e-01 4.823968e-01 0.000000e+00
    outer loop
      vertex   -7.355142e+01 -2.765399e+01 -2.078574e-15
      vertex   -7.355142e+01 -2.765399e+01 3.000000e+00
      vertex   -7.317454e+01 -2.696964e+01 -2.291467e-15
    endloop
  endfacet
  facet normal -8.759528e-01 4.823968e-01 0.000000e+00
    outer loop
      vertex   -7.317454e+01 -2.696964e+01 -2.291467e-15
      vertex   -7.355142e+01 -2.765399e+01 3.000000e+00
      vertex   -7.317454e+01 -2.696964e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.842803e-01 7.292191e-01 0.000000e+00
    outer loop
      vertex   -7.317454e+01 -2.696964e+01 -2.291467e-15
      vertex   -7.317454e+01 -2.696964e+01 3.000000e+00
      vertex   -7.260483e+01 -2.643504e+01 -2.454055e-15
    endloop
  endfacet
  facet normal -6.842803e-01 7.292191e-01 8.635689e-16
    outer loop
      vertex   -7.260483e+01 -2.643504e+01 -2.454055e-15
      vertex   -7.317454e+01 -2.696964e+01 3.000000e+00
      vertex   -7.260483e+01 -2.643504e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.257813e-01 9.048261e-01 1.071529e-15
    outer loop
      vertex   -7.260483e+01 -2.643504e+01 -2.454055e-15
      vertex   -7.260483e+01 -2.643504e+01 3.000000e+00
      vertex   -7.189792e+01 -2.610239e+01 -2.550459e-15
    endloop
  endfacet
  facet normal -4.257813e-01 9.048261e-01 -2.016905e-15
    outer loop
      vertex   -7.189792e+01 -2.610239e+01 -2.550459e-15
      vertex   -7.260483e+01 -2.643504e+01 3.000000e+00
      vertex   -7.189792e+01 -2.610239e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.257006e-01 9.920682e-01 -5.954378e-16
    outer loop
      vertex   -7.189792e+01 -2.610239e+01 -2.550459e-15
      vertex   -7.189792e+01 -2.610239e+01 3.000000e+00
      vertex   -7.112286e+01 -2.600419e+01 -2.571265e-15
    endloop
  endfacet
  facet normal -1.257006e-01 9.920682e-01 0.000000e+00
    outer loop
      vertex   -7.112286e+01 -2.600419e+01 -2.571265e-15
      vertex   -7.189792e+01 -2.610239e+01 3.000000e+00
      vertex   -7.112286e+01 -2.600419e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.866559e-01 9.824254e-01 -0.000000e+00
    outer loop
      vertex   -7.112286e+01 -2.600419e+01 -2.571265e-15
      vertex   -7.112286e+01 -2.600419e+01 3.000000e+00
      vertex   -7.035533e+01 -2.615002e+01 -2.514440e-15
    endloop
  endfacet
  facet normal 1.866559e-01 9.824254e-01 -0.000000e+00
    outer loop
      vertex   -7.035533e+01 -2.615002e+01 -2.514440e-15
      vertex   -7.112286e+01 -2.600419e+01 3.000000e+00
      vertex   -7.035533e+01 -2.615002e+01 3.000000e+00
    endloop
  endfacet
  facet normal 4.807837e-01 8.768392e-01 -0.000000e+00
    outer loop
      vertex   -7.035533e+01 -2.615002e+01 -2.514440e-15
      vertex   -7.035533e+01 -2.615002e+01 3.000000e+00
      vertex   -6.967028e+01 -2.652563e+01 -2.385534e-15
    endloop
  endfacet
  facet normal 4.807837e-01 8.768392e-01 -2.076773e-15
    outer loop
      vertex   -6.967028e+01 -2.652563e+01 -2.385534e-15
      vertex   -7.035533e+01 -2.615002e+01 3.000000e+00
      vertex   -6.967028e+01 -2.652563e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.279583e-01 6.856214e-01 -1.623878e-15
    outer loop
      vertex   -6.967028e+01 -2.652563e+01 -2.385534e-15
      vertex   -6.967028e+01 -2.652563e+01 3.000000e+00
      vertex   -6.913463e+01 -2.709436e+01 -2.197137e-15
    endloop
  endfacet
  facet normal 7.279583e-01 6.856214e-01 -8.119388e-16
    outer loop
      vertex   -6.913463e+01 -2.709436e+01 -2.197137e-15
      vertex   -6.967028e+01 -2.652563e+01 3.000000e+00
      vertex   -6.913463e+01 -2.709436e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.040409e-01 4.274460e-01 -5.061978e-16
    outer loop
      vertex   -6.913463e+01 -2.709436e+01 -2.197137e-15
      vertex   -6.913463e+01 -2.709436e+01 3.000000e+00
      vertex   -6.880068e+01 -2.780066e+01 -1.967646e-15
    endloop
  endfacet
  facet normal 9.040409e-01 4.274460e-01 -5.061978e-16
    outer loop
      vertex   -6.880068e+01 -2.780066e+01 -1.967646e-15
      vertex   -6.913463e+01 -2.709436e+01 3.000000e+00
      vertex   -6.880068e+01 -2.780066e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.918352e-01 1.275265e-01 -1.510217e-16
    outer loop
      vertex   -6.880068e+01 -2.780066e+01 -1.967646e-15
      vertex   -6.880068e+01 -2.780066e+01 3.000000e+00
      vertex   -6.870105e+01 -2.857554e+01 -1.719474e-15
    endloop
  endfacet
  facet normal 9.918352e-01 1.275265e-01 -1.510217e-16
    outer loop
      vertex   -6.870105e+01 -2.857554e+01 -1.719474e-15
      vertex   -6.880068e+01 -2.780066e+01 3.000000e+00
      vertex   -6.870105e+01 -2.857554e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.827673e-01 -1.848473e-01 2.189031e-16
    outer loop
      vertex   -6.870105e+01 -2.857554e+01 -1.719474e-15
      vertex   -6.870105e+01 -2.857554e+01 3.000000e+00
      vertex   -6.884547e+01 -2.934334e+01 -1.476857e-15
    endloop
  endfacet
  facet normal 9.827673e-01 -1.848473e-01 0.000000e+00
    outer loop
      vertex   -6.884547e+01 -2.934334e+01 -1.476857e-15
      vertex   -6.870105e+01 -2.857554e+01 3.000000e+00
      vertex   -6.884547e+01 -2.934334e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.777227e-01 -4.791689e-01 0.000000e+00
    outer loop
      vertex   -6.884547e+01 -2.934334e+01 -1.476857e-15
      vertex   -6.884547e+01 -2.934334e+01 3.000000e+00
      vertex   -6.921982e+01 -3.002907e+01 -1.263489e-15
    endloop
  endfacet
  facet normal 8.777227e-01 -4.791689e-01 -3.590280e-15
    outer loop
      vertex   -6.921982e+01 -3.002907e+01 -1.263489e-15
      vertex   -6.884547e+01 -2.934334e+01 3.000000e+00
      vertex   -6.921982e+01 -3.002907e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.869601e-01 -7.266951e-01 -2.393517e-15
    outer loop
      vertex   -6.921982e+01 -3.002907e+01 -1.263489e-15
      vertex   -6.921982e+01 -3.002907e+01 3.000000e+00
      vertex   -6.978756e+01 -3.056577e+01 7.225984e-15
    endloop
  endfacet
  facet normal 6.869601e-01 -7.266951e-01 1.721160e-15
    outer loop
      vertex   -6.978756e+01 -3.056577e+01 7.225984e-15
      vertex   -6.921982e+01 -3.002907e+01 3.000000e+00
      vertex   -6.978756e+01 -3.056577e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.304475e-16 -3.687273e-16 -1.000000e+00
    outer loop
      vertex   -6.980633e+01 -2.792572e+01 -1.790348e-15
      vertex   -6.880068e+01 -2.780066e+01 -1.967646e-15
      vertex   -6.969150e+01 -2.850300e+01 -1.592468e-15
    endloop
  endfacet
  facet normal -1.531294e-16 -3.399586e-16 -1.000000e+00
    outer loop
      vertex   -6.969150e+01 -2.850300e+01 -1.592468e-15
      vertex   -6.880068e+01 -2.780066e+01 -1.967646e-15
      vertex   -6.870105e+01 -2.857554e+01 -1.719474e-15
    endloop
  endfacet
  facet normal -1.476333e-16 -2.649181e-16 -1.000000e+00
    outer loop
      vertex   -6.969150e+01 -2.850300e+01 -1.592468e-15
      vertex   -6.870105e+01 -2.857554e+01 -1.719474e-15
      vertex   -6.980633e+01 -2.908028e+01 -1.422585e-15
    endloop
  endfacet
  facet normal -1.359916e-16 -2.904113e-16 -1.000000e+00
    outer loop
      vertex   -6.980633e+01 -2.908028e+01 -1.422585e-15
      vertex   -6.870105e+01 -2.857554e+01 -1.719474e-15
      vertex   -6.884547e+01 -2.934334e+01 -1.476857e-15
    endloop
  endfacet
  facet normal -1.026183e-16 -1.685126e-16 -1.000000e+00
    outer loop
      vertex   -6.980633e+01 -2.908028e+01 -1.422585e-15
      vertex   -6.884547e+01 -2.934334e+01 -1.476857e-15
      vertex   -7.013333e+01 -2.956967e+01 -1.306559e-15
    endloop
  endfacet
  facet normal -8.578046e-17 -2.643237e-16 -1.000000e+00
    outer loop
      vertex   -7.013333e+01 -2.956967e+01 -1.306559e-15
      vertex   -6.884547e+01 -2.934334e+01 -1.476857e-15
      vertex   -6.921982e+01 -3.002907e+01 -1.263489e-15
    endloop
  endfacet
  facet normal -1.214914e-17 -1.179109e-16 -1.000000e+00
    outer loop
      vertex   -7.013333e+01 -2.956967e+01 -1.306559e-15
      vertex   -6.921982e+01 -3.002907e+01 -1.263489e-15
      vertex   -7.062272e+01 -2.989667e+01 -1.262057e-15
    endloop
  endfacet
  facet normal -1.358282e-15 -1.438116e-14 -1.000000e+00
    outer loop
      vertex   -7.062272e+01 -2.989667e+01 -1.262057e-15
      vertex   -6.921982e+01 -3.002907e+01 -1.263489e-15
      vertex   -6.978756e+01 -3.056577e+01 7.225984e-15
    endloop
  endfacet
  facet normal 2.068367e-15 -1.010408e-14 -1.000000e+00
    outer loop
      vertex   -7.062272e+01 -2.989667e+01 -1.262057e-15
      vertex   -6.978756e+01 -3.056577e+01 7.225984e-15
      vertex   -7.120000e+01 -3.001150e+01 -1.295851e-15
    endloop
  endfacet
  facet normal 2.578952e-15 -8.802965e-15 -1.000000e+00
    outer loop
      vertex   -7.120000e+01 -3.001150e+01 -1.295851e-15
      vertex   -6.978756e+01 -3.056577e+01 7.225984e-15
      vertex   -7.263081e+01 -3.142212e+01 7.431864e-15
    endloop
  endfacet
  facet normal -5.279487e-15 -8.321096e-16 -1.000000e+00
    outer loop
      vertex   -7.120000e+01 -3.001150e+01 -1.295851e-15
      vertex   -7.263081e+01 -3.142212e+01 7.431864e-15
      vertex   -7.273699e+01 -3.047472e+01 7.204093e-15
    endloop
  endfacet
  facet normal -1.168504e-16 -4.780609e-16 -1.000000e+00
    outer loop
      vertex   -6.980633e+01 -2.792572e+01 -1.790348e-15
      vertex   -7.013333e+01 -2.743633e+01 -1.986097e-15
      vertex   -6.880068e+01 -2.780066e+01 -1.967646e-15
    endloop
  endfacet
  facet normal -8.611591e-17 -3.656397e-16 -1.000000e+00
    outer loop
      vertex   -6.880068e+01 -2.780066e+01 -1.967646e-15
      vertex   -7.013333e+01 -2.743633e+01 -1.986097e-15
      vertex   -6.913463e+01 -2.709436e+01 -2.197137e-15
    endloop
  endfacet
  facet normal -2.629963e-17 -5.403307e-16 -1.000000e+00
    outer loop
      vertex   -6.913463e+01 -2.709436e+01 -2.197137e-15
      vertex   -7.013333e+01 -2.743633e+01 -1.986097e-15
      vertex   -7.062272e+01 -2.710933e+01 -2.149915e-15
    endloop
  endfacet
  facet normal -2.813553e-17 -3.577613e-16 -1.000000e+00
    outer loop
      vertex   -6.913463e+01 -2.709436e+01 -2.197137e-15
      vertex   -7.062272e+01 -2.710933e+01 -2.149915e-15
      vertex   -6.967028e+01 -2.652563e+01 -2.385534e-15
    endloop
  endfacet
  facet normal -1.750505e-17 -3.751076e-16 -1.000000e+00
    outer loop
      vertex   -6.967028e+01 -2.652563e+01 -2.385534e-15
      vertex   -7.062272e+01 -2.710933e+01 -2.149915e-15
      vertex   -7.035533e+01 -2.615002e+01 -2.514440e-15
    endloop
  endfacet
  facet normal 1.039158e-16 -4.089522e-16 -1.000000e+00
    outer loop
      vertex   -7.035533e+01 -2.615002e+01 -2.514440e-15
      vertex   -7.062272e+01 -2.710933e+01 -2.149915e-15
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
    endloop
  endfacet
  facet normal 1.351617e-17 -3.185320e-16 -1.000000e+00
    outer loop
      vertex   -7.035533e+01 -2.615002e+01 -2.514440e-15
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
      vertex   -7.112286e+01 -2.600419e+01 -2.571265e-15
    endloop
  endfacet
  facet normal 1.351617e-17 -3.185320e-16 -1.000000e+00
    outer loop
      vertex   -7.112286e+01 -2.600419e+01 -2.571265e-15
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
      vertex   -7.189792e+01 -2.610239e+01 -2.550459e-15
    endloop
  endfacet
  facet normal 1.351617e-17 -3.185320e-16 -1.000000e+00
    outer loop
      vertex   -7.189792e+01 -2.610239e+01 -2.550459e-15
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
      vertex   -7.260483e+01 -2.643504e+01 -2.454055e-15
    endloop
  endfacet
  facet normal 8.579807e-17 -1.370275e-16 -1.000000e+00
    outer loop
      vertex   -7.260483e+01 -2.643504e+01 -2.454055e-15
      vertex   -7.120000e+01 -2.699450e+01 -2.256862e-15
      vertex   -7.177728e+01 -2.710933e+01 -2.290657e-15
    endloop
  endfacet
  facet normal -2.695187e-17 -2.754063e-16 -1.000000e+00
    outer loop
      vertex   -7.260483e+01 -2.643504e+01 -2.454055e-15
      vertex   -7.177728e+01 -2.710933e+01 -2.290657e-15
      vertex   -7.317454e+01 -2.696964e+01 -2.291467e-15
    endloop
  endfacet
  facet normal -1.132997e-17 -1.191369e-16 -1.000000e+00
    outer loop
      vertex   -7.317454e+01 -2.696964e+01 -2.291467e-15
      vertex   -7.177728e+01 -2.710933e+01 -2.290657e-15
      vertex   -7.226667e+01 -2.743633e+01 -2.246154e-15
    endloop
  endfacet
  facet normal -8.573127e-17 -2.638753e-16 -1.000000e+00
    outer loop
      vertex   -7.317454e+01 -2.696964e+01 -2.291467e-15
      vertex   -7.226667e+01 -2.743633e+01 -2.246154e-15
      vertex   -7.355142e+01 -2.765399e+01 -2.078574e-15
    endloop
  endfacet
  facet normal -1.017947e-16 -1.690629e-16 -1.000000e+00
    outer loop
      vertex   -7.355142e+01 -2.765399e+01 -2.078574e-15
      vertex   -7.226667e+01 -2.743633e+01 -2.246154e-15
      vertex   -7.259367e+01 -2.792572e+01 -2.130129e-15
    endloop
  endfacet
  facet normal -1.360573e-16 -2.898284e-16 -1.000000e+00
    outer loop
      vertex   -7.355142e+01 -2.765399e+01 -2.078574e-15
      vertex   -7.259367e+01 -2.792572e+01 -2.130129e-15
      vertex   -7.369866e+01 -2.842126e+01 -1.836166e-15
    endloop
  endfacet
  facet normal -1.471888e-16 -2.650066e-16 -1.000000e+00
    outer loop
      vertex   -7.369866e+01 -2.842126e+01 -1.836166e-15
      vertex   -7.259367e+01 -2.792572e+01 -2.130129e-15
      vertex   -7.270850e+01 -2.850300e+01 -1.960245e-15
    endloop
  endfacet
  facet normal -1.533269e-16 -3.393600e-16 -1.000000e+00
    outer loop
      vertex   -7.369866e+01 -2.842126e+01 -1.836166e-15
      vertex   -7.270850e+01 -2.850300e+01 -1.960245e-15
      vertex   -7.360188e+01 -2.919650e+01 -1.587918e-15
    endloop
  endfacet
  facet normal -1.305189e-16 -3.687415e-16 -1.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 -1.587918e-15
      vertex   -7.270850e+01 -2.850300e+01 -1.960245e-15
      vertex   -7.259367e+01 -2.908028e+01 -1.762366e-15
    endloop
  endfacet
  facet normal -1.178403e-16 -4.787224e-16 -1.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 -1.587918e-15
      vertex   -7.259367e+01 -2.908028e+01 -1.762366e-15
      vertex   -7.226667e+01 -2.956967e+01 -1.566617e-15
    endloop
  endfacet
  facet normal -8.611001e-17 -3.651890e-16 -1.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 -1.587918e-15
      vertex   -7.226667e+01 -2.956967e+01 -1.566617e-15
      vertex   -7.327054e+01 -2.990402e+01 -1.358072e-15
    endloop
  endfacet
  facet normal -2.728521e-17 -5.418057e-16 -1.000000e+00
    outer loop
      vertex   -7.327054e+01 -2.990402e+01 -1.358072e-15
      vertex   -7.226667e+01 -2.956967e+01 -1.566617e-15
      vertex   -7.177728e+01 -2.989667e+01 -1.402799e-15
    endloop
  endfacet
  facet normal 4.370680e-17 -1.496218e-14 -1.000000e+00
    outer loop
      vertex   -7.327054e+01 -2.990402e+01 -1.358072e-15
      vertex   -7.177728e+01 -2.989667e+01 -1.402799e-15
      vertex   -7.273699e+01 -3.047472e+01 7.204093e-15
    endloop
  endfacet
  facet normal -2.087183e-15 -1.142435e-14 -1.000000e+00
    outer loop
      vertex   -7.273699e+01 -3.047472e+01 7.204093e-15
      vertex   -7.177728e+01 -2.989667e+01 -1.402799e-15
      vertex   -7.120000e+01 -3.001150e+01 -1.295851e-15
    endloop
  endfacet
  facet normal -1.912210e-15 -7.231418e-16 -1.000000e+00
    outer loop
      vertex   -4.940633e+01 -2.792572e+01 -1.399066e-17
      vertex   -4.830134e+01 -2.842126e+01 -1.768622e-15
      vertex   -4.929150e+01 -2.850300e+01 1.838885e-16
    endloop
  endfacet
  facet normal -1.966011e-15 -7.141588e-17 -1.000000e+00
    outer loop
      vertex   -4.929150e+01 -2.850300e+01 1.838885e-16
      vertex   -4.830134e+01 -2.842126e+01 -1.768622e-15
      vertex   -4.839812e+01 -2.919650e+01 -1.522989e-15
    endloop
  endfacet
  facet normal -1.852911e-15 7.428259e-17 -1.000000e+00
    outer loop
      vertex   -4.929150e+01 -2.850300e+01 1.838885e-16
      vertex   -4.839812e+01 -2.919650e+01 -1.522989e-15
      vertex   -4.940633e+01 -2.908028e+01 3.537723e-16
    endloop
  endfacet
  facet normal -1.753719e-15 9.347178e-16 -1.000000e+00
    outer loop
      vertex   -4.940633e+01 -2.908028e+01 3.537723e-16
      vertex   -4.839812e+01 -2.919650e+01 -1.522989e-15
      vertex   -4.973333e+01 -2.956967e+01 4.697975e-16
    endloop
  endfacet
  facet normal -1.616854e-15 4.450042e-16 -1.000000e+00
    outer loop
      vertex   -4.973333e+01 -2.956967e+01 4.697975e-16
      vertex   -4.839812e+01 -2.919650e+01 -1.522989e-15
      vertex   -4.872946e+01 -2.990402e+01 -1.302100e-15
    endloop
  endfacet
  facet normal -1.208167e-15 1.672056e-15 -1.000000e+00
    outer loop
      vertex   -4.973333e+01 -2.956967e+01 4.697975e-16
      vertex   -4.872946e+01 -2.990402e+01 -1.302100e-15
      vertex   -5.022272e+01 -2.989667e+01 5.143002e-16
    endloop
  endfacet
  facet normal -1.283867e-15 -1.370466e-14 -1.000000e+00
    outer loop
      vertex   -5.022272e+01 -2.989667e+01 5.143002e-16
      vertex   -4.872946e+01 -2.990402e+01 -1.302100e-15
      vertex   -4.926301e+01 -3.047472e+01 7.204093e-15
    endloop
  endfacet
  facet normal 1.774540e-15 -8.626906e-15 -1.000000e+00
    outer loop
      vertex   -5.022272e+01 -2.989667e+01 5.143002e-16
      vertex   -4.926301e+01 -3.047472e+01 7.204093e-15
      vertex   -5.080000e+01 -3.001150e+01 4.805055e-16
    endloop
  endfacet
  facet normal 4.161508e-15 -7.068128e-16 -1.000000e+00
    outer loop
      vertex   -5.080000e+01 -3.001150e+01 4.805055e-16
      vertex   -4.926301e+01 -3.047472e+01 7.204093e-15
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
    endloop
  endfacet
  facet normal -2.032836e-15 -6.989768e-15 -1.000000e+00
    outer loop
      vertex   -5.080000e+01 -3.001150e+01 4.805055e-16
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
      vertex   -5.221244e+01 -3.056577e+01 7.225984e-15
    endloop
  endfacet
  facet normal -2.748061e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.221244e+01 -3.056577e+01 7.225984e-15
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
      vertex   -5.231282e+01 -3.126120e+01 7.393176e-15
    endloop
  endfacet
  facet normal -2.141755e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.231282e+01 -3.126120e+01 7.393176e-15
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
      vertex   -5.248902e+01 -3.194563e+01 7.557722e-15
    endloop
  endfacet
  facet normal -8.349638e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.248902e+01 -3.194563e+01 7.557722e-15
      vertex   -4.936919e+01 -3.142212e+01 7.431864e-15
      vertex   -4.958072e+01 -3.235665e+01 7.656537e-15
    endloop
  endfacet
  facet normal 3.618028e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.248902e+01 -3.194563e+01 7.557722e-15
      vertex   -4.958072e+01 -3.235665e+01 7.656537e-15
      vertex   -5.273951e+01 -3.261308e+01 7.718185e-15
    endloop
  endfacet
  facet normal -5.715041e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.273951e+01 -3.261308e+01 7.718185e-15
      vertex   -4.958072e+01 -3.235665e+01 7.656537e-15
      vertex   -4.989568e+01 -3.326983e+01 7.876079e-15
    endloop
  endfacet
  facet normal 1.145141e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.273951e+01 -3.261308e+01 7.718185e-15
      vertex   -4.989568e+01 -3.326983e+01 7.876079e-15
      vertex   -5.306209e+01 -3.325772e+01 7.873166e-15
    endloop
  endfacet
  facet normal 1.137927e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.306209e+01 -3.325772e+01 7.873166e-15
      vertex   -4.989568e+01 -3.326983e+01 7.876079e-15
      vertex   -5.031121e+01 -3.415340e+01 8.088501e-15
    endloop
  endfacet
  facet normal -6.385595e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.306209e+01 -3.325772e+01 7.873166e-15
      vertex   -5.031121e+01 -3.415340e+01 8.088501e-15
      vertex   -5.345394e+01 -3.387392e+01 8.021311e-15
    endloop
  endfacet
  facet normal -5.719149e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.345394e+01 -3.387392e+01 8.021311e-15
      vertex   -5.031121e+01 -3.415340e+01 8.088501e-15
      vertex   -5.082355e+01 -3.499934e+01 8.291878e-15
    endloop
  endfacet
  facet normal 8.188983e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.345394e+01 -3.387392e+01 8.021311e-15
      vertex   -5.082355e+01 -3.499934e+01 8.291878e-15
      vertex   -5.391166e+01 -3.445632e+01 8.161328e-15
    endloop
  endfacet
  facet normal 8.185611e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.391166e+01 -3.445632e+01 8.161328e-15
      vertex   -5.082355e+01 -3.499934e+01 8.291878e-15
      vertex   -5.142807e+01 -3.580001e+01 8.484369e-15
    endloop
  endfacet
  facet normal -1.588898e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.391166e+01 -3.445632e+01 8.161328e-15
      vertex   -5.142807e+01 -3.580001e+01 8.484369e-15
      vertex   -5.443124e+01 -3.499983e+01 8.291994e-15
    endloop
  endfacet
  facet normal -1.708111e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.443124e+01 -3.499983e+01 8.291994e-15
      vertex   -5.142807e+01 -3.580001e+01 8.484369e-15
      vertex   -5.211929e+01 -3.654814e+01 8.664231e-15
    endloop
  endfacet
  facet normal 1.455841e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.443124e+01 -3.499983e+01 8.291994e-15
      vertex   -5.211929e+01 -3.654814e+01 8.664231e-15
      vertex   -5.500815e+01 -3.549969e+01 8.412170e-15
    endloop
  endfacet
  facet normal 3.261712e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.500815e+01 -3.549969e+01 8.412170e-15
      vertex   -5.211929e+01 -3.654814e+01 8.664231e-15
      vertex   -5.563735e+01 -3.595156e+01 8.520806e-15
    endloop
  endfacet
  facet normal 5.471241e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.563735e+01 -3.595156e+01 8.520806e-15
      vertex   -5.211929e+01 -3.654814e+01 8.664231e-15
      vertex   -5.289094e+01 -3.723696e+01 8.829833e-15
    endloop
  endfacet
  facet normal -1.253085e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.563735e+01 -3.595156e+01 8.520806e-15
      vertex   -5.289094e+01 -3.723696e+01 8.829833e-15
      vertex   -5.631336e+01 -3.635149e+01 8.616954e-15
    endloop
  endfacet
  facet normal 3.764423e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.631336e+01 -3.635149e+01 8.616954e-15
      vertex   -5.289094e+01 -3.723696e+01 8.829833e-15
      vertex   -5.373604e+01 -3.786023e+01 8.979677e-15
    endloop
  endfacet
  facet normal -5.006735e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.631336e+01 -3.635149e+01 8.616954e-15
      vertex   -5.373604e+01 -3.786023e+01 8.979677e-15
      vertex   -5.703026e+01 -3.669598e+01 8.699775e-15
    endloop
  endfacet
  facet normal -1.369941e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.703026e+01 -3.669598e+01 8.699775e-15
      vertex   -5.373604e+01 -3.786023e+01 8.979677e-15
      vertex   -5.464694e+01 -3.841231e+01 9.112405e-15
    endloop
  endfacet
  facet normal -1.250302e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.703026e+01 -3.669598e+01 8.699775e-15
      vertex   -5.464694e+01 -3.841231e+01 9.112405e-15
      vertex   -5.778182e+01 -3.698204e+01 8.768546e-15
    endloop
  endfacet
  facet normal -6.512208e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.778182e+01 -3.698204e+01 8.768546e-15
      vertex   -5.464694e+01 -3.841231e+01 9.112405e-15
      vertex   -5.561537e+01 -3.888820e+01 9.226814e-15
    endloop
  endfacet
  facet normal 6.931611e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.778182e+01 -3.698204e+01 8.768546e-15
      vertex   -5.561537e+01 -3.888820e+01 9.226814e-15
      vertex   -5.856146e+01 -3.720716e+01 8.822668e-15
    endloop
  endfacet
  facet normal 1.234828e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.856146e+01 -3.720716e+01 8.822668e-15
      vertex   -5.561537e+01 -3.888820e+01 9.226814e-15
      vertex   -5.663258e+01 -3.928358e+01 9.321869e-15
    endloop
  endfacet
  facet normal -4.939659e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.856146e+01 -3.720716e+01 8.822668e-15
      vertex   -5.663258e+01 -3.928358e+01 9.321869e-15
      vertex   -5.768934e+01 -3.959487e+01 9.396709e-15
    endloop
  endfacet
  facet normal -1.895007e-15 -6.847828e-16 -1.000000e+00
    outer loop
      vertex   -4.830134e+01 -2.842126e+01 -1.768622e-15
      vertex   -4.940633e+01 -2.792572e+01 -1.399066e-17
      vertex   -4.844858e+01 -2.765399e+01 -2.015010e-15
    endloop
  endfacet
  facet normal -1.660942e-15 -1.509790e-15 -1.000000e+00
    outer loop
      vertex   -4.844858e+01 -2.765399e+01 -2.015010e-15
      vertex   -4.940633e+01 -2.792572e+01 -1.399066e-17
      vertex   -4.973333e+01 -2.743633e+01 -2.097399e-16
    endloop
  endfacet
  facet normal -1.610656e-15 -1.212981e-15 -1.000000e+00
    outer loop
      vertex   -4.844858e+01 -2.765399e+01 -2.015010e-15
      vertex   -4.973333e+01 -2.743633e+01 -2.097399e-16
      vertex   -4.882546e+01 -2.696964e+01 -2.238091e-15
    endloop
  endfacet
  facet normal -1.117188e-15 -2.172961e-15 -1.000000e+00
    outer loop
      vertex   -4.882546e+01 -2.696964e+01 -2.238091e-15
      vertex   -4.973333e+01 -2.743633e+01 -2.097399e-16
      vertex   -5.022272e+01 -2.710933e+01 -3.735581e-16
    endloop
  endfacet
  facet normal -1.175863e-15 -1.586022e-15 -1.000000e+00
    outer loop
      vertex   -4.882546e+01 -2.696964e+01 -2.238091e-15
      vertex   -5.022272e+01 -2.710933e+01 -3.735581e-16
      vertex   -4.939517e+01 -2.643504e+01 -2.416079e-15
    endloop
  endfacet
  facet normal -3.353978e-16 -2.617531e-15 -1.000000e+00
    outer loop
      vertex   -4.939517e+01 -2.643504e+01 -2.416079e-15
      vertex   -5.022272e+01 -2.710933e+01 -3.735581e-16
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
    endloop
  endfacet
  facet normal -6.713490e-16 -1.773936e-15 -1.000000e+00
    outer loop
      vertex   -4.939517e+01 -2.643504e+01 -2.416079e-15
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
      vertex   -5.010208e+01 -2.610239e+01 -2.531593e-15
    endloop
  endfacet
  facet normal -2.160262e-16 -2.130151e-15 -1.000000e+00
    outer loop
      vertex   -5.010208e+01 -2.610239e+01 -2.531593e-15
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
      vertex   -5.087714e+01 -2.600419e+01 -2.573350e-15
    endloop
  endfacet
  facet normal 3.493473e-16 -2.086111e-15 -1.000000e+00
    outer loop
      vertex   -5.087714e+01 -2.600419e+01 -2.573350e-15
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
      vertex   -5.164467e+01 -2.615002e+01 -2.537274e-15
    endloop
  endfacet
  facet normal 4.528935e-16 -1.982541e-15 -1.000000e+00
    outer loop
      vertex   -5.164467e+01 -2.615002e+01 -2.537274e-15
      vertex   -5.080000e+01 -2.699450e+01 -4.805055e-16
      vertex   -5.137728e+01 -2.710933e+01 -5.143002e-16
    endloop
  endfacet
  facet normal 8.632058e-16 -1.868171e-15 -1.000000e+00
    outer loop
      vertex   -5.164467e+01 -2.615002e+01 -2.537274e-15
      vertex   -5.137728e+01 -2.710933e+01 -5.143002e-16
      vertex   -5.232972e+01 -2.652563e+01 -2.426886e-15
    endloop
  endfacet
  facet normal 1.154381e-15 -1.393046e-15 -1.000000e+00
    outer loop
      vertex   -5.232972e+01 -2.652563e+01 -2.426886e-15
      vertex   -5.137728e+01 -2.710933e+01 -5.143002e-16
      vertex   -5.286537e+01 -2.709436e+01 -2.252968e-15
    endloop
  endfacet
  facet normal 1.149718e-15 -1.856768e-15 -1.000000e+00
    outer loop
      vertex   -5.286537e+01 -2.709436e+01 -2.252968e-15
      vertex   -5.137728e+01 -2.710933e+01 -5.143002e-16
      vertex   -5.186667e+01 -2.743633e+01 -4.697975e-16
    endloop
  endfacet
  facet normal 1.444719e-15 -9.952292e-16 -1.000000e+00
    outer loop
      vertex   -5.286537e+01 -2.709436e+01 -2.252968e-15
      vertex   -5.186667e+01 -2.743633e+01 -4.697975e-16
      vertex   -5.319932e+01 -2.780066e+01 -2.032505e-15
    endloop
  endfacet
  facet normal 1.514019e-15 -1.248715e-15 -1.000000e+00
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 -2.032505e-15
      vertex   -5.186667e+01 -2.743633e+01 -4.697975e-16
      vertex   -5.219367e+01 -2.792572e+01 -3.537723e-16
    endloop
  endfacet
  facet normal 1.593300e-15 -6.112114e-16 -1.000000e+00
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 -2.032505e-15
      vertex   -5.219367e+01 -2.792572e+01 -3.537723e-16
      vertex   -5.230850e+01 -2.850300e+01 -1.838885e-16
    endloop
  endfacet
  facet normal 1.657410e-15 -5.298977e-16 -1.000000e+00
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 -2.032505e-15
      vertex   -5.230850e+01 -2.850300e+01 -1.838885e-16
      vertex   -5.329895e+01 -2.857554e+01 -1.787026e-15
    endloop
  endfacet
  facet normal 1.620103e-15 -2.052125e-17 -1.000000e+00
    outer loop
      vertex   -5.329895e+01 -2.857554e+01 -1.787026e-15
      vertex   -5.230850e+01 -2.850300e+01 -1.838885e-16
      vertex   -5.219367e+01 -2.908028e+01 1.399066e-17
    endloop
  endfacet
  facet normal 1.622186e-15 -1.595964e-17 -1.000000e+00
    outer loop
      vertex   -5.329895e+01 -2.857554e+01 -1.787026e-15
      vertex   -5.219367e+01 -2.908028e+01 1.399066e-17
      vertex   -5.315453e+01 -2.934334e+01 -1.540505e-15
    endloop
  endfacet
  facet normal 1.460203e-15 5.756927e-16 -1.000000e+00
    outer loop
      vertex   -5.315453e+01 -2.934334e+01 -1.540505e-15
      vertex   -5.219367e+01 -2.908028e+01 1.399066e-17
      vertex   -5.186667e+01 -2.956967e+01 2.097399e-16
    endloop
  endfacet
  facet normal 1.439900e-15 4.601638e-16 -1.000000e+00
    outer loop
      vertex   -5.315453e+01 -2.934334e+01 -1.540505e-15
      vertex   -5.186667e+01 -2.956967e+01 2.097399e-16
      vertex   -5.278018e+01 -3.002907e+01 -1.317018e-15
    endloop
  endfacet
  facet normal 1.097344e-15 1.141321e-15 -1.000000e+00
    outer loop
      vertex   -5.278018e+01 -3.002907e+01 -1.317018e-15
      vertex   -5.186667e+01 -2.956967e+01 2.097399e-16
      vertex   -5.137728e+01 -2.989667e+01 3.735581e-16
    endloop
  endfacet
  facet normal 2.461584e-15 -1.331378e-14 -1.000000e+00
    outer loop
      vertex   -5.278018e+01 -3.002907e+01 -1.317018e-15
      vertex   -5.137728e+01 -2.989667e+01 3.735581e-16
      vertex   -5.221244e+01 -3.056577e+01 7.225984e-15
    endloop
  endfacet
  facet normal -1.483526e-15 -8.389561e-15 -1.000000e+00
    outer loop
      vertex   -5.221244e+01 -3.056577e+01 7.225984e-15
      vertex   -5.137728e+01 -2.989667e+01 3.735581e-16
      vertex   -5.080000e+01 -3.001150e+01 4.805055e-16
    endloop
  endfacet
  facet normal 0.000000e+00 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.978756e+01 -3.056577e+01 7.225984e-15
      vertex   -6.968718e+01 -3.126120e+01 7.393176e-15
      vertex   -7.263081e+01 -3.142212e+01 7.431864e-15
    endloop
  endfacet
  facet normal -1.055795e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 7.431864e-15
      vertex   -6.968718e+01 -3.126120e+01 7.393176e-15
      vertex   -6.951098e+01 -3.194563e+01 7.557722e-15
    endloop
  endfacet
  facet normal 3.515637e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 7.431864e-15
      vertex   -6.951098e+01 -3.194563e+01 7.557722e-15
      vertex   -7.241928e+01 -3.235665e+01 7.656537e-15
    endloop
  endfacet
  facet normal 2.698446e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.241928e+01 -3.235665e+01 7.656537e-15
      vertex   -6.951098e+01 -3.194563e+01 7.557722e-15
      vertex   -6.926049e+01 -3.261308e+01 7.718185e-15
    endloop
  endfacet
  facet normal 3.692795e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.241928e+01 -3.235665e+01 7.656537e-15
      vertex   -6.926049e+01 -3.261308e+01 7.718185e-15
      vertex   -7.210432e+01 -3.326983e+01 7.876079e-15
    endloop
  endfacet
  facet normal 7.241886e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.210432e+01 -3.326983e+01 7.876079e-15
      vertex   -6.926049e+01 -3.261308e+01 7.718185e-15
      vertex   -6.893791e+01 -3.325772e+01 7.873166e-15
    endloop
  endfacet
  facet normal 7.238479e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.210432e+01 -3.326983e+01 7.876079e-15
      vertex   -6.893791e+01 -3.325772e+01 7.873166e-15
      vertex   -7.168879e+01 -3.415340e+01 8.088501e-15
    endloop
  endfacet
  facet normal -5.241007e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.168879e+01 -3.415340e+01 8.088501e-15
      vertex   -6.893791e+01 -3.325772e+01 7.873166e-15
      vertex   -6.854606e+01 -3.387392e+01 8.021311e-15
    endloop
  endfacet
  facet normal -1.407790e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.168879e+01 -3.415340e+01 8.088501e-15
      vertex   -6.854606e+01 -3.387392e+01 8.021311e-15
      vertex   -7.117645e+01 -3.499934e+01 8.291878e-15
    endloop
  endfacet
  facet normal 3.131082e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.117645e+01 -3.499934e+01 8.291878e-15
      vertex   -6.854606e+01 -3.387392e+01 8.021311e-15
      vertex   -6.808834e+01 -3.445632e+01 8.161328e-15
    endloop
  endfacet
  facet normal -2.464485e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.117645e+01 -3.499934e+01 8.291878e-15
      vertex   -6.808834e+01 -3.445632e+01 8.161328e-15
      vertex   -7.057193e+01 -3.580001e+01 8.484369e-15
    endloop
  endfacet
  facet normal -7.222264e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.057193e+01 -3.580001e+01 8.484369e-15
      vertex   -6.808834e+01 -3.445632e+01 8.161328e-15
      vertex   -6.756876e+01 -3.499983e+01 8.291994e-15
    endloop
  endfacet
  facet normal 1.672892e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -7.057193e+01 -3.580001e+01 8.484369e-15
      vertex   -6.756876e+01 -3.499983e+01 8.291994e-15
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
    endloop
  endfacet
  facet normal -7.219046e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
      vertex   -6.756876e+01 -3.499983e+01 8.291994e-15
      vertex   -6.699185e+01 -3.549969e+01 8.412170e-15
    endloop
  endfacet
  facet normal -3.763514e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
      vertex   -6.699185e+01 -3.549969e+01 8.412170e-15
      vertex   -6.636265e+01 -3.595156e+01 8.520806e-15
    endloop
  endfacet
  facet normal -6.411611e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.988071e+01 -3.654814e+01 8.664231e-15
      vertex   -6.636265e+01 -3.595156e+01 8.520806e-15
      vertex   -6.910906e+01 -3.723696e+01 8.829833e-15
    endloop
  endfacet
  facet normal -2.506169e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.910906e+01 -3.723696e+01 8.829833e-15
      vertex   -6.636265e+01 -3.595156e+01 8.520806e-15
      vertex   -6.568664e+01 -3.635149e+01 8.616954e-15
    endloop
  endfacet
  facet normal -9.753277e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.910906e+01 -3.723696e+01 8.829833e-15
      vertex   -6.568664e+01 -3.635149e+01 8.616954e-15
      vertex   -6.826396e+01 -3.786023e+01 8.979677e-15
    endloop
  endfacet
  facet normal 2.253031e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.826396e+01 -3.786023e+01 8.979677e-15
      vertex   -6.568664e+01 -3.635149e+01 8.616954e-15
      vertex   -6.496974e+01 -3.669598e+01 8.699775e-15
    endloop
  endfacet
  facet normal 9.846454e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.826396e+01 -3.786023e+01 8.979677e-15
      vertex   -6.496974e+01 -3.669598e+01 8.699775e-15
      vertex   -6.735306e+01 -3.841231e+01 9.112405e-15
    endloop
  endfacet
  facet normal -1.350326e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.735306e+01 -3.841231e+01 9.112405e-15
      vertex   -6.496974e+01 -3.669598e+01 8.699775e-15
      vertex   -6.421818e+01 -3.698204e+01 8.768546e-15
    endloop
  endfacet
  facet normal 1.011106e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.735306e+01 -3.841231e+01 9.112405e-15
      vertex   -6.421818e+01 -3.698204e+01 8.768546e-15
      vertex   -6.638463e+01 -3.888820e+01 9.226814e-15
    endloop
  endfacet
  facet normal 5.495331e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.638463e+01 -3.888820e+01 9.226814e-15
      vertex   -6.421818e+01 -3.698204e+01 8.768546e-15
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
    endloop
  endfacet
  facet normal -1.474933e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.638463e+01 -3.888820e+01 9.226814e-15
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
      vertex   -6.536742e+01 -3.928358e+01 9.321869e-15
    endloop
  endfacet
  facet normal -1.040857e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.536742e+01 -3.928358e+01 9.321869e-15
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
      vertex   -6.431066e+01 -3.959487e+01 9.396709e-15
    endloop
  endfacet
  facet normal 9.602205e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.431066e+01 -3.959487e+01 9.396709e-15
      vertex   -6.343854e+01 -3.720716e+01 8.822668e-15
      vertex   -6.263761e+01 -3.736937e+01 8.861667e-15
    endloop
  endfacet
  facet normal 8.646765e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.431066e+01 -3.959487e+01 9.396709e-15
      vertex   -6.263761e+01 -3.736937e+01 8.861667e-15
      vertex   -6.322391e+01 -3.981926e+01 9.450655e-15
    endloop
  endfacet
  facet normal -1.631790e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.322391e+01 -3.981926e+01 9.450655e-15
      vertex   -6.263761e+01 -3.736937e+01 8.861667e-15
      vertex   -6.182239e+01 -3.746727e+01 8.885203e-15
    endloop
  endfacet
  facet normal -1.570949e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.322391e+01 -3.981926e+01 9.450655e-15
      vertex   -6.182239e+01 -3.746727e+01 8.885203e-15
      vertex   -6.211701e+01 -3.995471e+01 9.483220e-15
    endloop
  endfacet
  facet normal 1.247407e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.211701e+01 -3.995471e+01 9.483220e-15
      vertex   -6.182239e+01 -3.746727e+01 8.885203e-15
      vertex   -6.100000e+01 -3.750000e+01 8.893072e-15
    endloop
  endfacet
  facet normal 9.931284e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.211701e+01 -3.995471e+01 9.483220e-15
      vertex   -6.100000e+01 -3.750000e+01 8.893072e-15
      vertex   -6.100000e+01 -4.000000e+01 9.494108e-15
    endloop
  endfacet
  facet normal -9.931284e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -6.100000e+01 -4.000000e+01 9.494108e-15
      vertex   -6.100000e+01 -3.750000e+01 8.893072e-15
      vertex   -5.988299e+01 -3.995471e+01 9.483220e-15
    endloop
  endfacet
  facet normal 7.676353e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.988299e+01 -3.995471e+01 9.483220e-15
      vertex   -6.100000e+01 -3.750000e+01 8.893072e-15
      vertex   -6.017761e+01 -3.746727e+01 8.885203e-15
    endloop
  endfacet
  facet normal 1.372374e-30 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.988299e+01 -3.995471e+01 9.483220e-15
      vertex   -6.017761e+01 -3.746727e+01 8.885203e-15
      vertex   -5.877609e+01 -3.981926e+01 9.450655e-15
    endloop
  endfacet
  facet normal -3.839505e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.877609e+01 -3.981926e+01 9.450655e-15
      vertex   -6.017761e+01 -3.746727e+01 8.885203e-15
      vertex   -5.936239e+01 -3.736937e+01 8.861667e-15
    endloop
  endfacet
  facet normal -8.823229e-31 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.877609e+01 -3.981926e+01 9.450655e-15
      vertex   -5.936239e+01 -3.736937e+01 8.861667e-15
      vertex   -5.768934e+01 -3.959487e+01 9.396709e-15
    endloop
  endfacet
  facet normal -9.602205e-32 -2.404143e-16 -1.000000e+00
    outer loop
      vertex   -5.768934e+01 -3.959487e+01 9.396709e-15
      vertex   -5.936239e+01 -3.736937e+01 8.861667e-15
      vertex   -5.856146e+01 -3.720716e+01 8.822668e-15
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.980633e+01 -2.908028e+01 3.000000e+00
      vertex   -6.870105e+01 -2.857554e+01 3.000000e+00
      vertex   -6.969150e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.969150e+01 -2.850300e+01 3.000000e+00
      vertex   -6.870105e+01 -2.857554e+01 3.000000e+00
      vertex   -6.880068e+01 -2.780066e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.969150e+01 -2.850300e+01 3.000000e+00
      vertex   -6.880068e+01 -2.780066e+01 3.000000e+00
      vertex   -6.980633e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.041923e-16 8.378102e-16 1.000000e+00
    outer loop
      vertex   -6.980633e+01 -2.792572e+01 3.000000e+00
      vertex   -6.880068e+01 -2.780066e+01 3.000000e+00
      vertex   -7.013333e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.852958e-16 5.411487e-16 1.000000e+00
    outer loop
      vertex   -7.013333e+01 -2.743633e+01 3.000000e+00
      vertex   -6.880068e+01 -2.780066e+01 3.000000e+00
      vertex   -6.913463e+01 -2.709436e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.013333e+01 -2.743633e+01 3.000000e+00
      vertex   -6.913463e+01 -2.709436e+01 3.000000e+00
      vertex   -7.062272e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.062272e+01 -2.710933e+01 3.000000e+00
      vertex   -6.913463e+01 -2.709436e+01 3.000000e+00
      vertex   -6.967028e+01 -2.652563e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.062272e+01 -2.710933e+01 3.000000e+00
      vertex   -6.967028e+01 -2.652563e+01 3.000000e+00
      vertex   -7.035533e+01 -2.615002e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.870105e+01 -2.857554e+01 3.000000e+00
      vertex   -6.980633e+01 -2.908028e+01 3.000000e+00
      vertex   -6.884547e+01 -2.934334e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.100166e-16 7.671007e-16 1.000000e+00
    outer loop
      vertex   -6.884547e+01 -2.934334e+01 3.000000e+00
      vertex   -6.980633e+01 -2.908028e+01 3.000000e+00
      vertex   -7.013333e+01 -2.956967e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.555311e-16 5.081126e-16 1.000000e+00
    outer loop
      vertex   -6.884547e+01 -2.934334e+01 3.000000e+00
      vertex   -7.013333e+01 -2.956967e+01 3.000000e+00
      vertex   -6.921982e+01 -3.002907e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.921982e+01 -3.002907e+01 3.000000e+00
      vertex   -7.013333e+01 -2.956967e+01 3.000000e+00
      vertex   -7.062272e+01 -2.989667e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.420079e-15 1.504676e-14 1.000000e+00
    outer loop
      vertex   -6.921982e+01 -3.002907e+01 3.000000e+00
      vertex   -7.062272e+01 -2.989667e+01 3.000000e+00
      vertex   -6.978756e+01 -3.056577e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.115241e-15 1.063404e-14 1.000000e+00
    outer loop
      vertex   -6.978756e+01 -3.056577e+01 3.000000e+00
      vertex   -7.062272e+01 -2.989667e+01 3.000000e+00
      vertex   -7.120000e+01 -3.001150e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.642213e-15 9.291169e-15 1.000000e+00
    outer loop
      vertex   -6.978756e+01 -3.056577e+01 3.000000e+00
      vertex   -7.120000e+01 -3.001150e+01 3.000000e+00
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.453239e-15 1.079910e-15 1.000000e+00
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
      vertex   -7.120000e+01 -3.001150e+01 3.000000e+00
      vertex   -7.273699e+01 -3.047472e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.297561e-15 1.155062e-14 1.000000e+00
    outer loop
      vertex   -7.273699e+01 -3.047472e+01 3.000000e+00
      vertex   -7.120000e+01 -3.001150e+01 3.000000e+00
      vertex   -7.177728e+01 -2.989667e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.626663e-17 1.549179e-14 1.000000e+00
    outer loop
      vertex   -7.273699e+01 -3.047472e+01 3.000000e+00
      vertex   -7.177728e+01 -2.989667e+01 3.000000e+00
      vertex   -7.327054e+01 -2.990402e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.327054e+01 -2.990402e+01 3.000000e+00
      vertex   -7.177728e+01 -2.989667e+01 3.000000e+00
      vertex   -7.226667e+01 -2.956967e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.808465e-16 5.429773e-16 1.000000e+00
    outer loop
      vertex   -7.327054e+01 -2.990402e+01 3.000000e+00
      vertex   -7.226667e+01 -2.956967e+01 3.000000e+00
      vertex   -7.360188e+01 -2.919650e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.712731e-17 8.425309e-16 1.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 3.000000e+00
      vertex   -7.226667e+01 -2.956967e+01 3.000000e+00
      vertex   -7.259367e+01 -2.908028e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 3.000000e+00
      vertex   -7.259367e+01 -2.908028e+01 3.000000e+00
      vertex   -7.270850e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.360188e+01 -2.919650e+01 3.000000e+00
      vertex   -7.270850e+01 -2.850300e+01 3.000000e+00
      vertex   -7.369866e+01 -2.842126e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.369866e+01 -2.842126e+01 3.000000e+00
      vertex   -7.270850e+01 -2.850300e+01 3.000000e+00
      vertex   -7.259367e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.369866e+01 -2.842126e+01 3.000000e+00
      vertex   -7.259367e+01 -2.792572e+01 3.000000e+00
      vertex   -7.355142e+01 -2.765399e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.164219e-16 7.628208e-16 1.000000e+00
    outer loop
      vertex   -7.355142e+01 -2.765399e+01 3.000000e+00
      vertex   -7.259367e+01 -2.792572e+01 3.000000e+00
      vertex   -7.226667e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.599756e-16 5.057500e-16 1.000000e+00
    outer loop
      vertex   -7.355142e+01 -2.765399e+01 3.000000e+00
      vertex   -7.226667e+01 -2.743633e+01 3.000000e+00
      vertex   -7.317454e+01 -2.696964e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.317454e+01 -2.696964e+01 3.000000e+00
      vertex   -7.226667e+01 -2.743633e+01 3.000000e+00
      vertex   -7.177728e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.317454e+01 -2.696964e+01 3.000000e+00
      vertex   -7.177728e+01 -2.710933e+01 3.000000e+00
      vertex   -7.260483e+01 -2.643504e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.260483e+01 -2.643504e+01 3.000000e+00
      vertex   -7.177728e+01 -2.710933e+01 3.000000e+00
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.260483e+01 -2.643504e+01 3.000000e+00
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
      vertex   -7.189792e+01 -2.610239e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.189792e+01 -2.610239e+01 3.000000e+00
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
      vertex   -7.112286e+01 -2.600419e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.112286e+01 -2.600419e+01 3.000000e+00
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
      vertex   -7.035533e+01 -2.615002e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.035533e+01 -2.615002e+01 3.000000e+00
      vertex   -7.120000e+01 -2.699450e+01 3.000000e+00
      vertex   -7.062272e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.583587e-15 -5.139075e-16 1.000000e+00
    outer loop
      vertex   -4.940633e+01 -2.908028e+01 3.000000e+00
      vertex   -4.839812e+01 -2.919650e+01 3.000000e+00
      vertex   -4.929150e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.719027e-15 -3.394321e-16 1.000000e+00
    outer loop
      vertex   -4.929150e+01 -2.850300e+01 3.000000e+00
      vertex   -4.839812e+01 -2.919650e+01 3.000000e+00
      vertex   -4.830134e+01 -2.842126e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.647531e-15 5.266267e-16 1.000000e+00
    outer loop
      vertex   -4.929150e+01 -2.850300e+01 3.000000e+00
      vertex   -4.830134e+01 -2.842126e+01 3.000000e+00
      vertex   -4.940633e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.638425e-15 5.063210e-16 1.000000e+00
    outer loop
      vertex   -4.940633e+01 -2.792572e+01 3.000000e+00
      vertex   -4.830134e+01 -2.842126e+01 3.000000e+00
      vertex   -4.844858e+01 -2.765399e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.122300e-15 2.325505e-15 1.000000e+00
    outer loop
      vertex   -4.940633e+01 -2.792572e+01 3.000000e+00
      vertex   -4.844858e+01 -2.765399e+01 3.000000e+00
      vertex   -4.973333e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.027412e-15 1.765439e-15 1.000000e+00
    outer loop
      vertex   -4.973333e+01 -2.743633e+01 3.000000e+00
      vertex   -4.844858e+01 -2.765399e+01 3.000000e+00
      vertex   -4.882546e+01 -2.696964e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.658788e-15 2.482551e-15 1.000000e+00
    outer loop
      vertex   -4.973333e+01 -2.743633e+01 3.000000e+00
      vertex   -4.882546e+01 -2.696964e+01 3.000000e+00
      vertex   -5.022272e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.723367e-15 1.836546e-15 1.000000e+00
    outer loop
      vertex   -5.022272e+01 -2.710933e+01 3.000000e+00
      vertex   -4.882546e+01 -2.696964e+01 3.000000e+00
      vertex   -4.939517e+01 -2.643504e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.317939e-16 3.176242e-15 1.000000e+00
    outer loop
      vertex   -5.022272e+01 -2.710933e+01 3.000000e+00
      vertex   -4.939517e+01 -2.643504e+01 3.000000e+00
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.027298e-15 2.183106e-15 1.000000e+00
    outer loop
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
      vertex   -4.939517e+01 -2.643504e+01 3.000000e+00
      vertex   -5.010208e+01 -2.610239e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.443136e-16 2.717429e-15 1.000000e+00
    outer loop
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
      vertex   -5.010208e+01 -2.610239e+01 3.000000e+00
      vertex   -5.087714e+01 -2.600419e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.550946e-15 -7.970581e-16 1.000000e+00
    outer loop
      vertex   -4.940633e+01 -2.908028e+01 3.000000e+00
      vertex   -4.973333e+01 -2.956967e+01 3.000000e+00
      vertex   -4.839812e+01 -2.919650e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.476963e-15 -5.323414e-16 1.000000e+00
    outer loop
      vertex   -4.839812e+01 -2.919650e+01 3.000000e+00
      vertex   -4.973333e+01 -2.956967e+01 3.000000e+00
      vertex   -4.872946e+01 -2.990402e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.771322e-15 -2.650971e-15 1.000000e+00
    outer loop
      vertex   -4.872946e+01 -2.990402e+01 3.000000e+00
      vertex   -4.973333e+01 -2.956967e+01 3.000000e+00
      vertex   -5.022272e+01 -2.989667e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.852465e-15 1.383119e-14 1.000000e+00
    outer loop
      vertex   -4.872946e+01 -2.990402e+01 3.000000e+00
      vertex   -5.022272e+01 -2.989667e+01 3.000000e+00
      vertex   -4.926301e+01 -3.047472e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.608293e-15 8.085435e-15 1.000000e+00
    outer loop
      vertex   -4.926301e+01 -3.047472e+01 3.000000e+00
      vertex   -5.022272e+01 -2.989667e+01 3.000000e+00
      vertex   -5.080000e+01 -3.001150e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.776271e-15 8.919648e-16 1.000000e+00
    outer loop
      vertex   -4.926301e+01 -3.047472e+01 3.000000e+00
      vertex   -5.080000e+01 -3.001150e+01 3.000000e+00
      vertex   -4.936919e+01 -3.142212e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.823039e-15 6.571374e-15 1.000000e+00
    outer loop
      vertex   -4.936919e+01 -3.142212e+01 3.000000e+00
      vertex   -5.080000e+01 -3.001150e+01 3.000000e+00
      vertex   -5.221244e+01 -3.056577e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.463633e-17 6.335792e-16 1.000000e+00
    outer loop
      vertex   -4.936919e+01 -3.142212e+01 3.000000e+00
      vertex   -5.221244e+01 -3.056577e+01 3.000000e+00
      vertex   -5.231282e+01 -3.126120e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.480669e-15 7.443826e-15 1.000000e+00
    outer loop
      vertex   -5.080000e+01 -3.001150e+01 3.000000e+00
      vertex   -5.137728e+01 -2.989667e+01 3.000000e+00
      vertex   -5.221244e+01 -3.056577e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.146979e-15 1.321997e-14 1.000000e+00
    outer loop
      vertex   -5.221244e+01 -3.056577e+01 3.000000e+00
      vertex   -5.137728e+01 -2.989667e+01 3.000000e+00
      vertex   -5.278018e+01 -3.002907e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.664240e-15 -2.490712e-15 1.000000e+00
    outer loop
      vertex   -5.278018e+01 -3.002907e+01 3.000000e+00
      vertex   -5.137728e+01 -2.989667e+01 3.000000e+00
      vertex   -5.186667e+01 -2.956967e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.544052e-15 -7.412434e-16 1.000000e+00
    outer loop
      vertex   -5.278018e+01 -3.002907e+01 3.000000e+00
      vertex   -5.186667e+01 -2.956967e+01 3.000000e+00
      vertex   -5.315453e+01 -2.934334e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.554249e-15 -7.992653e-16 1.000000e+00
    outer loop
      vertex   -5.315453e+01 -2.934334e+01 3.000000e+00
      vertex   -5.186667e+01 -2.956967e+01 3.000000e+00
      vertex   -5.219367e+01 -2.908028e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.637266e-15 -4.960395e-16 1.000000e+00
    outer loop
      vertex   -5.315453e+01 -2.934334e+01 3.000000e+00
      vertex   -5.219367e+01 -2.908028e+01 3.000000e+00
      vertex   -5.329895e+01 -2.857554e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.651604e-15 -5.274368e-16 1.000000e+00
    outer loop
      vertex   -5.329895e+01 -2.857554e+01 3.000000e+00
      vertex   -5.219367e+01 -2.908028e+01 3.000000e+00
      vertex   -5.230850e+01 -2.850300e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.715809e-15 3.491887e-16 1.000000e+00
    outer loop
      vertex   -5.329895e+01 -2.857554e+01 3.000000e+00
      vertex   -5.230850e+01 -2.850300e+01 3.000000e+00
      vertex   -5.319932e+01 -2.780066e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.585622e-15 5.143121e-16 1.000000e+00
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 3.000000e+00
      vertex   -5.230850e+01 -2.850300e+01 3.000000e+00
      vertex   -5.219367e+01 -2.792572e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.342111e-15 2.472378e-15 1.000000e+00
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 3.000000e+00
      vertex   -5.219367e+01 -2.792572e+01 3.000000e+00
      vertex   -5.186667e+01 -2.743633e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.110957e-15 1.626856e-15 1.000000e+00
    outer loop
      vertex   -5.319932e+01 -2.780066e+01 3.000000e+00
      vertex   -5.186667e+01 -2.743633e+01 3.000000e+00
      vertex   -5.286537e+01 -2.709436e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.764026e-15 2.640052e-15 1.000000e+00
    outer loop
      vertex   -5.286537e+01 -2.709436e+01 3.000000e+00
      vertex   -5.186667e+01 -2.743633e+01 3.000000e+00
      vertex   -5.137728e+01 -2.710933e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.773775e-15 1.670615e-15 1.000000e+00
    outer loop
      vertex   -5.286537e+01 -2.709436e+01 3.000000e+00
      vertex   -5.137728e+01 -2.710933e+01 3.000000e+00
      vertex   -5.232972e+01 -2.652563e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.321066e-15 2.409322e-15 1.000000e+00
    outer loop
      vertex   -5.232972e+01 -2.652563e+01 3.000000e+00
      vertex   -5.137728e+01 -2.710933e+01 3.000000e+00
      vertex   -5.164467e+01 -2.615002e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.234665e-16 2.631644e-15 1.000000e+00
    outer loop
      vertex   -5.164467e+01 -2.615002e+01 3.000000e+00
      vertex   -5.137728e+01 -2.710933e+01 3.000000e+00
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.037467e-16 2.651368e-15 1.000000e+00
    outer loop
      vertex   -5.164467e+01 -2.615002e+01 3.000000e+00
      vertex   -5.080000e+01 -2.699450e+01 3.000000e+00
      vertex   -5.087714e+01 -2.600419e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.241928e+01 -3.235665e+01 3.000000e+00
      vertex   -6.951098e+01 -3.194563e+01 3.000000e+00
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
      vertex   -6.951098e+01 -3.194563e+01 3.000000e+00
      vertex   -6.968718e+01 -3.126120e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.463633e-17 6.335792e-16 1.000000e+00
    outer loop
      vertex   -7.263081e+01 -3.142212e+01 3.000000e+00
      vertex   -6.968718e+01 -3.126120e+01 3.000000e+00
      vertex   -6.978756e+01 -3.056577e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.951098e+01 -3.194563e+01 3.000000e+00
      vertex   -7.241928e+01 -3.235665e+01 3.000000e+00
      vertex   -6.926049e+01 -3.261308e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.926049e+01 -3.261308e+01 3.000000e+00
      vertex   -7.241928e+01 -3.235665e+01 3.000000e+00
      vertex   -7.210432e+01 -3.326983e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.926049e+01 -3.261308e+01 3.000000e+00
      vertex   -7.210432e+01 -3.326983e+01 3.000000e+00
      vertex   -6.893791e+01 -3.325772e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.919983e-18 5.017078e-16 1.000000e+00
    outer loop
      vertex   -6.893791e+01 -3.325772e+01 3.000000e+00
      vertex   -7.210432e+01 -3.326983e+01 3.000000e+00
      vertex   -7.168879e+01 -3.415340e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.065793e-17 6.821075e-16 1.000000e+00
    outer loop
      vertex   -6.893791e+01 -3.325772e+01 3.000000e+00
      vertex   -7.168879e+01 -3.415340e+01 3.000000e+00
      vertex   -6.854606e+01 -3.387392e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.854606e+01 -3.387392e+01 3.000000e+00
      vertex   -7.168879e+01 -3.415340e+01 3.000000e+00
      vertex   -7.117645e+01 -3.499934e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.854606e+01 -3.387392e+01 3.000000e+00
      vertex   -7.117645e+01 -3.499934e+01 3.000000e+00
      vertex   -6.808834e+01 -3.445632e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.722010e-16 9.792865e-16 1.000000e+00
    outer loop
      vertex   -6.808834e+01 -3.445632e+01 3.000000e+00
      vertex   -7.117645e+01 -3.499934e+01 3.000000e+00
      vertex   -7.057193e+01 -3.580001e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.357086e-16 2.253325e-16 1.000000e+00
    outer loop
      vertex   -6.808834e+01 -3.445632e+01 3.000000e+00
      vertex   -7.057193e+01 -3.580001e+01 3.000000e+00
      vertex   -6.756876e+01 -3.499983e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.373241e-16 2.192694e-16 1.000000e+00
    outer loop
      vertex   -6.756876e+01 -3.499983e+01 3.000000e+00
      vertex   -7.057193e+01 -3.580001e+01 3.000000e+00
      vertex   -6.988071e+01 -3.654814e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.166876e-16 2.500838e-16 1.000000e+00
    outer loop
      vertex   -6.756876e+01 -3.499983e+01 3.000000e+00
      vertex   -6.988071e+01 -3.654814e+01 3.000000e+00
      vertex   -6.699185e+01 -3.549969e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.696415e-16 1.590103e-15 1.000000e+00
    outer loop
      vertex   -6.699185e+01 -3.549969e+01 3.000000e+00
      vertex   -6.988071e+01 -3.654814e+01 3.000000e+00
      vertex   -6.636265e+01 -3.595156e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.636265e+01 -3.595156e+01 3.000000e+00
      vertex   -6.988071e+01 -3.654814e+01 3.000000e+00
      vertex   -6.910906e+01 -3.723696e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.636265e+01 -3.595156e+01 3.000000e+00
      vertex   -6.910906e+01 -3.723696e+01 3.000000e+00
      vertex   -6.568664e+01 -3.635149e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.568664e+01 -3.635149e+01 3.000000e+00
      vertex   -6.910906e+01 -3.723696e+01 3.000000e+00
      vertex   -6.826396e+01 -3.786023e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.568664e+01 -3.635149e+01 3.000000e+00
      vertex   -6.826396e+01 -3.786023e+01 3.000000e+00
      vertex   -6.496974e+01 -3.669598e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.795757e-16 5.081064e-16 1.000000e+00
    outer loop
      vertex   -6.496974e+01 -3.669598e+01 3.000000e+00
      vertex   -6.826396e+01 -3.786023e+01 3.000000e+00
      vertex   -6.735306e+01 -3.841231e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.442916e-17 1.692761e-16 1.000000e+00
    outer loop
      vertex   -6.496974e+01 -3.669598e+01 3.000000e+00
      vertex   -6.735306e+01 -3.841231e+01 3.000000e+00
      vertex   -6.421818e+01 -3.698204e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.345753e-17 1.494877e-16 1.000000e+00
    outer loop
      vertex   -6.421818e+01 -3.698204e+01 3.000000e+00
      vertex   -6.735306e+01 -3.841231e+01 3.000000e+00
      vertex   -6.638463e+01 -3.888820e+01 3.000000e+00
    endloop
  endfacet
  facet normal 5.064893e-17 1.754109e-16 1.000000e+00
    outer loop
      vertex   -6.421818e+01 -3.698204e+01 3.000000e+00
      vertex   -6.638463e+01 -3.888820e+01 3.000000e+00
      vertex   -6.343854e+01 -3.720716e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.107713e-17 1.571350e-16 1.000000e+00
    outer loop
      vertex   -6.343854e+01 -3.720716e+01 3.000000e+00
      vertex   -6.638463e+01 -3.888820e+01 3.000000e+00
      vertex   -6.536742e+01 -3.928358e+01 3.000000e+00
    endloop
  endfacet
  facet normal 4.946527e-17 1.679218e-16 1.000000e+00
    outer loop
      vertex   -6.343854e+01 -3.720716e+01 3.000000e+00
      vertex   -6.536742e+01 -3.928358e+01 3.000000e+00
      vertex   -6.431066e+01 -3.959487e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.507484e-17 1.731780e-16 1.000000e+00
    outer loop
      vertex   -6.343854e+01 -3.720716e+01 3.000000e+00
      vertex   -6.431066e+01 -3.959487e+01 3.000000e+00
      vertex   -6.263761e+01 -3.736937e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.566561e-17 1.727338e-16 1.000000e+00
    outer loop
      vertex   -6.263761e+01 -3.736937e+01 3.000000e+00
      vertex   -6.431066e+01 -3.959487e+01 3.000000e+00
      vertex   -6.322391e+01 -3.981926e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.116052e-17 1.762051e-16 1.000000e+00
    outer loop
      vertex   -6.263761e+01 -3.736937e+01 3.000000e+00
      vertex   -6.322391e+01 -3.981926e+01 3.000000e+00
      vertex   -6.182239e+01 -3.746727e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.153509e-17 1.759819e-16 1.000000e+00
    outer loop
      vertex   -6.182239e+01 -3.746727e+01 3.000000e+00
      vertex   -6.322391e+01 -3.981926e+01 3.000000e+00
      vertex   -6.211701e+01 -3.995471e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.071637e-18 1.776950e-16 1.000000e+00
    outer loop
      vertex   -6.182239e+01 -3.746727e+01 3.000000e+00
      vertex   -6.211701e+01 -3.995471e+01 3.000000e+00
      vertex   -6.100000e+01 -3.750000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 7.201937e-18 1.776357e-16 1.000000e+00
    outer loop
      vertex   -6.100000e+01 -3.750000e+01 3.000000e+00
      vertex   -6.211701e+01 -3.995471e+01 3.000000e+00
      vertex   -6.100000e+01 -4.000000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.201937e-18 1.776357e-16 1.000000e+00
    outer loop
      vertex   -6.100000e+01 -3.750000e+01 3.000000e+00
      vertex   -6.100000e+01 -4.000000e+01 3.000000e+00
      vertex   -5.988299e+01 -3.995471e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.071637e-18 1.776950e-16 1.000000e+00
    outer loop
      vertex   -6.100000e+01 -3.750000e+01 3.000000e+00
      vertex   -5.988299e+01 -3.995471e+01 3.000000e+00
      vertex   -6.017761e+01 -3.746727e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.153509e-17 1.759819e-16 1.000000e+00
    outer loop
      vertex   -6.017761e+01 -3.746727e+01 3.000000e+00
      vertex   -5.988299e+01 -3.995471e+01 3.000000e+00
      vertex   -5.877609e+01 -3.981926e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.116052e-17 1.762051e-16 1.000000e+00
    outer loop
      vertex   -6.017761e+01 -3.746727e+01 3.000000e+00
      vertex   -5.877609e+01 -3.981926e+01 3.000000e+00
      vertex   -5.936239e+01 -3.736937e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.566561e-17 1.727338e-16 1.000000e+00
    outer loop
      vertex   -5.936239e+01 -3.736937e+01 3.000000e+00
      vertex   -5.877609e+01 -3.981926e+01 3.000000e+00
      vertex   -5.768934e+01 -3.959487e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.507484e-17 1.731780e-16 1.000000e+00
    outer loop
      vertex   -5.936239e+01 -3.736937e+01 3.000000e+00
      vertex   -5.768934e+01 -3.959487e+01 3.000000e+00
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.946527e-17 1.679218e-16 1.000000e+00
    outer loop
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
      vertex   -5.768934e+01 -3.959487e+01 3.000000e+00
      vertex   -5.663258e+01 -3.928358e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.107713e-17 1.571350e-16 1.000000e+00
    outer loop
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
      vertex   -5.663258e+01 -3.928358e+01 3.000000e+00
      vertex   -5.561537e+01 -3.888820e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.064893e-17 1.754109e-16 1.000000e+00
    outer loop
      vertex   -5.856146e+01 -3.720716e+01 3.000000e+00
      vertex   -5.561537e+01 -3.888820e+01 3.000000e+00
      vertex   -5.778182e+01 -3.698204e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.345753e-17 1.494877e-16 1.000000e+00
    outer loop
      vertex   -5.778182e+01 -3.698204e+01 3.000000e+00
      vertex   -5.561537e+01 -3.888820e+01 3.000000e+00
      vertex   -5.464694e+01 -3.841231e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.442916e-17 1.692761e-16 1.000000e+00
    outer loop
      vertex   -5.778182e+01 -3.698204e+01 3.000000e+00
      vertex   -5.464694e+01 -3.841231e+01 3.000000e+00
      vertex   -5.703026e+01 -3.669598e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.795757e-16 5.081064e-16 1.000000e+00
    outer loop
      vertex   -5.703026e+01 -3.669598e+01 3.000000e+00
      vertex   -5.464694e+01 -3.841231e+01 3.000000e+00
      vertex   -5.373604e+01 -3.786023e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.703026e+01 -3.669598e+01 3.000000e+00
      vertex   -5.373604e+01 -3.786023e+01 3.000000e+00
      vertex   -5.631336e+01 -3.635149e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.631336e+01 -3.635149e+01 3.000000e+00
      vertex   -5.373604e+01 -3.786023e+01 3.000000e+00
      vertex   -5.289094e+01 -3.723696e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.631336e+01 -3.635149e+01 3.000000e+00
      vertex   -5.289094e+01 -3.723696e+01 3.000000e+00
      vertex   -5.563735e+01 -3.595156e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.563735e+01 -3.595156e+01 3.000000e+00
      vertex   -5.289094e+01 -3.723696e+01 3.000000e+00
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.696415e-16 1.590103e-15 1.000000e+00
    outer loop
      vertex   -5.563735e+01 -3.595156e+01 3.000000e+00
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
      vertex   -5.500815e+01 -3.549969e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.166876e-16 2.500838e-16 1.000000e+00
    outer loop
      vertex   -5.500815e+01 -3.549969e+01 3.000000e+00
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
      vertex   -5.443124e+01 -3.499983e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.373241e-16 2.192694e-16 1.000000e+00
    outer loop
      vertex   -5.443124e+01 -3.499983e+01 3.000000e+00
      vertex   -5.211929e+01 -3.654814e+01 3.000000e+00
      vertex   -5.142807e+01 -3.580001e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.357086e-16 2.253325e-16 1.000000e+00
    outer loop
      vertex   -5.443124e+01 -3.499983e+01 3.000000e+00
      vertex   -5.142807e+01 -3.580001e+01 3.000000e+00
      vertex   -5.391166e+01 -3.445632e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.722010e-16 9.792865e-16 1.000000e+00
    outer loop
      vertex   -5.391166e+01 -3.445632e+01 3.000000e+00
      vertex   -5.142807e+01 -3.580001e+01 3.000000e+00
      vertex   -5.082355e+01 -3.499934e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.391166e+01 -3.445632e+01 3.000000e+00
      vertex   -5.082355e+01 -3.499934e+01 3.000000e+00
      vertex   -5.345394e+01 -3.387392e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.345394e+01 -3.387392e+01 3.000000e+00
      vertex   -5.082355e+01 -3.499934e+01 3.000000e+00
      vertex   -5.031121e+01 -3.415340e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.065793e-17 6.821075e-16 1.000000e+00
    outer loop
      vertex   -5.345394e+01 -3.387392e+01 3.000000e+00
      vertex   -5.031121e+01 -3.415340e+01 3.000000e+00
      vertex   -5.306209e+01 -3.325772e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.919983e-18 5.017078e-16 1.000000e+00
    outer loop
      vertex   -5.306209e+01 -3.325772e+01 3.000000e+00
      vertex   -5.031121e+01 -3.415340e+01 3.000000e+00
      vertex   -4.989568e+01 -3.326983e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.306209e+01 -3.325772e+01 3.000000e+00
      vertex   -4.989568e+01 -3.326983e+01 3.000000e+00
      vertex   -5.273951e+01 -3.261308e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.273951e+01 -3.261308e+01 3.000000e+00
      vertex   -4.989568e+01 -3.326983e+01 3.000000e+00
      vertex   -4.958072e+01 -3.235665e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.273951e+01 -3.261308e+01 3.000000e+00
      vertex   -4.958072e+01 -3.235665e+01 3.000000e+00
      vertex   -5.248902e+01 -3.194563e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.248902e+01 -3.194563e+01 3.000000e+00
      vertex   -4.958072e+01 -3.235665e+01 3.000000e+00
      vertex   -4.936919e+01 -3.142212e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.248902e+01 -3.194563e+01 3.000000e+00
      vertex   -4.936919e+01 -3.142212e+01 3.000000e+00
      vertex   -5.231282e+01 -3.126120e+01 3.000000e+00
    endloop
  endfacet
  facet normal 8.047901e-17 -9.805469e-01 -1.962848e-01
    outer loop
      vertex   -3.492941e+01 -2.945384e+01 6.836321e+00
      vertex   -3.942826e+01 -2.936650e+01 6.400000e+00
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
    endloop
  endfacet
  facet normal -7.823830e-17 -9.805469e-01 1.962848e-01
    outer loop
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
      vertex   -3.942826e+01 -2.936650e+01 6.400000e+00
      vertex   -3.947058e+01 -2.945384e+01 5.963679e+00
    endloop
  endfacet
  facet normal -7.897913e-17 -9.805469e-01 1.962848e-01
    outer loop
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
      vertex   -3.947058e+01 -2.945384e+01 5.963679e+00
      vertex   -3.492941e+01 -2.945384e+01 5.963679e+00
    endloop
  endfacet
  facet normal -2.176107e-16 -8.309742e-01 5.563110e-01
    outer loop
      vertex   -3.492941e+01 -2.945384e+01 5.963679e+00
      vertex   -3.947058e+01 -2.945384e+01 5.963679e+00
      vertex   -3.956785e+01 -2.969797e+01 5.599019e+00
    endloop
  endfacet
  facet normal 1.248750e-15 -8.309742e-01 5.563110e-01
    outer loop
      vertex   -3.492941e+01 -2.945384e+01 5.963679e+00
      vertex   -3.956785e+01 -2.969797e+01 5.599019e+00
      vertex   -3.483214e+01 -2.969797e+01 5.599019e+00
    endloop
  endfacet
  facet normal 8.359262e-16 -5.571386e-01 8.304195e-01
    outer loop
      vertex   -3.483214e+01 -2.969797e+01 5.599019e+00
      vertex   -3.956785e+01 -2.969797e+01 5.599019e+00
      vertex   -3.966150e+01 -3.006296e+01 5.354142e+00
    endloop
  endfacet
  facet normal -8.017059e-16 -5.571386e-01 8.304195e-01
    outer loop
      vertex   -3.483214e+01 -2.969797e+01 5.599019e+00
      vertex   -3.966150e+01 -3.006296e+01 5.354142e+00
      vertex   -3.473849e+01 -3.006296e+01 5.354142e+00
    endloop
  endfacet
  facet normal -2.837848e-16 -1.966207e-01 9.804796e-01
    outer loop
      vertex   -3.473849e+01 -3.006296e+01 5.354142e+00
      vertex   -3.966150e+01 -3.006296e+01 5.354142e+00
      vertex   -3.470000e+01 -3.050000e+01 5.266500e+00
    endloop
  endfacet
  facet normal -1.219177e-15 -1.966207e-01 9.804796e-01
    outer loop
      vertex   -3.470000e+01 -3.050000e+01 5.266500e+00
      vertex   -3.966150e+01 -3.006296e+01 5.354142e+00
      vertex   -3.970000e+01 -3.050000e+01 5.266500e+00
    endloop
  endfacet
  facet normal -1.219177e-15 1.966207e-01 9.804796e-01
    outer loop
      vertex   -3.470000e+01 -3.050000e+01 5.266500e+00
      vertex   -3.970000e+01 -3.050000e+01 5.266500e+00
      vertex   -3.473849e+01 -3.093704e+01 5.354142e+00
    endloop
  endfacet
  facet normal -4.587833e-16 1.966207e-01 9.804796e-01
    outer loop
      vertex   -3.473849e+01 -3.093704e+01 5.354142e+00
      vertex   -3.970000e+01 -3.050000e+01 5.266500e+00
      vertex   -3.966150e+01 -3.093704e+01 5.354142e+00
    endloop
  endfacet
  facet normal 4.570903e-16 5.571386e-01 8.304195e-01
    outer loop
      vertex   -3.473849e+01 -3.093704e+01 5.354142e+00
      vertex   -3.966150e+01 -3.093704e+01 5.354142e+00
      vertex   -3.956785e+01 -3.130203e+01 5.599019e+00
    endloop
  endfacet
  facet normal -3.032112e-15 -9.805469e-01 -1.962848e-01
    outer loop
      vertex   -3.942826e+01 -2.936650e+01 6.400000e+00
      vertex   -3.492941e+01 -2.945384e+01 6.836321e+00
      vertex   -3.947058e+01 -2.945384e+01 6.836321e+00
    endloop
  endfacet
  facet normal -2.491594e-15 -8.309742e-01 -5.563110e-01
    outer loop
      vertex   -3.947058e+01 -2.945384e+01 6.836321e+00
      vertex   -3.492941e+01 -2.945384e+01 6.836321e+00
      vertex   -3.483214e+01 -2.969797e+01 7.200981e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -8.309742e-01 -5.563110e-01
    outer loop
      vertex   -3.947058e+01 -2.945384e+01 6.836321e+00
      vertex   -3.483214e+01 -2.969797e+01 7.200981e+00
      vertex   -3.956785e+01 -2.969797e+01 7.200981e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -5.571386e-01 -8.304195e-01
    outer loop
      vertex   -3.956785e+01 -2.969797e+01 7.200981e+00
      vertex   -3.483214e+01 -2.969797e+01 7.200981e+00
      vertex   -3.473849e+01 -3.006296e+01 7.445858e+00
    endloop
  endfacet
  facet normal 6.990876e-16 -5.571386e-01 -8.304195e-01
    outer loop
      vertex   -3.956785e+01 -2.969797e+01 7.200981e+00
      vertex   -3.473849e+01 -3.006296e+01 7.445858e+00
      vertex   -3.966150e+01 -3.006296e+01 7.445858e+00
    endloop
  endfacet
  facet normal 4.956766e-16 -1.966207e-01 -9.804796e-01
    outer loop
      vertex   -3.966150e+01 -3.006296e+01 7.445858e+00
      vertex   -3.473849e+01 -3.006296e+01 7.445858e+00
      vertex   -3.970000e+01 -3.050000e+01 7.533500e+00
    endloop
  endfacet
  facet normal 1.741682e-16 -1.966207e-01 -9.804796e-01
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 7.533500e+00
      vertex   -3.473849e+01 -3.006296e+01 7.445858e+00
      vertex   -3.470000e+01 -3.050000e+01 7.533500e+00
    endloop
  endfacet
  facet normal 1.741682e-16 1.966207e-01 -9.804796e-01
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 7.533500e+00
      vertex   -3.470000e+01 -3.050000e+01 7.533500e+00
      vertex   -3.966150e+01 -3.093704e+01 7.445858e+00
    endloop
  endfacet
  facet normal 5.656760e-16 1.966207e-01 -9.804796e-01
    outer loop
      vertex   -3.966150e+01 -3.093704e+01 7.445858e+00
      vertex   -3.470000e+01 -3.050000e+01 7.533500e+00
      vertex   -3.473849e+01 -3.093704e+01 7.445858e+00
    endloop
  endfacet
  facet normal 1.972146e-16 5.571386e-01 -8.304195e-01
    outer loop
      vertex   -3.966150e+01 -3.093704e+01 7.445858e+00
      vertex   -3.473849e+01 -3.093704e+01 7.445858e+00
      vertex   -3.483214e+01 -3.130203e+01 7.200981e+00
    endloop
  endfacet
  facet normal 1.040099e-15 5.571386e-01 -8.304195e-01
    outer loop
      vertex   -3.966150e+01 -3.093704e+01 7.445858e+00
      vertex   -3.483214e+01 -3.130203e+01 7.200981e+00
      vertex   -3.956785e+01 -3.130203e+01 7.200981e+00
    endloop
  endfacet
  facet normal 1.040736e-15 8.309742e-01 -5.563110e-01
    outer loop
      vertex   -3.956785e+01 -3.130203e+01 7.200981e+00
      vertex   -3.483214e+01 -3.130203e+01 7.200981e+00
      vertex   -3.492941e+01 -3.154616e+01 6.836321e+00
    endloop
  endfacet
  facet normal 3.273028e-16 8.309742e-01 -5.563110e-01
    outer loop
      vertex   -3.956785e+01 -3.130203e+01 7.200981e+00
      vertex   -3.492941e+01 -3.154616e+01 6.836321e+00
      vertex   -3.947058e+01 -3.154616e+01 6.836321e+00
    endloop
  endfacet
  facet normal 1.151703e-16 9.805469e-01 -1.962848e-01
    outer loop
      vertex   -3.947058e+01 -3.154616e+01 6.836321e+00
      vertex   -3.492941e+01 -3.154616e+01 6.836321e+00
      vertex   -3.497173e+01 -3.163350e+01 6.400000e+00
    endloop
  endfacet
  facet normal 2.386378e-15 9.805469e-01 -1.962848e-01
    outer loop
      vertex   -3.947058e+01 -3.154616e+01 6.836321e+00
      vertex   -3.497173e+01 -3.163350e+01 6.400000e+00
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
    endloop
  endfacet
  facet normal 2.305933e-15 9.805469e-01 1.962848e-01
    outer loop
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
      vertex   -3.497173e+01 -3.163350e+01 6.400000e+00
      vertex   -3.492941e+01 -3.154616e+01 5.963679e+00
    endloop
  endfacet
  facet normal -1.610487e-15 9.805469e-01 1.962848e-01
    outer loop
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
      vertex   -3.492941e+01 -3.154616e+01 5.963679e+00
      vertex   -3.947058e+01 -3.154616e+01 5.963679e+00
    endloop
  endfacet
  facet normal -1.517810e-15 8.309742e-01 5.563110e-01
    outer loop
      vertex   -3.947058e+01 -3.154616e+01 5.963679e+00
      vertex   -3.492941e+01 -3.154616e+01 5.963679e+00
      vertex   -3.483214e+01 -3.130203e+01 5.599019e+00
    endloop
  endfacet
  facet normal 0.000000e+00 8.309742e-01 5.563110e-01
    outer loop
      vertex   -3.947058e+01 -3.154616e+01 5.963679e+00
      vertex   -3.483214e+01 -3.130203e+01 5.599019e+00
      vertex   -3.956785e+01 -3.130203e+01 5.599019e+00
    endloop
  endfacet
  facet normal -0.000000e+00 5.571386e-01 8.304195e-01
    outer loop
      vertex   -3.956785e+01 -3.130203e+01 5.599019e+00
      vertex   -3.483214e+01 -3.130203e+01 5.599019e+00
      vertex   -3.473849e+01 -3.093704e+01 5.354142e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 5.797307e-15
      vertex   -3.607379e+01 -2.995765e+01 3.850000e+00
      vertex   -3.595000e+01 -3.050000e+01 6.439294e-15
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.607379e+01 -2.995765e+01 3.850000e+00
      vertex   -3.595000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.595000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.607379e+01 -3.104235e+01 7.081280e-15
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 7.081280e-15
      vertex   -3.595000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.607379e+01 -3.104235e+01 3.850000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 7.081280e-15
      vertex   -3.607379e+01 -3.104235e+01 3.850000e+00
      vertex   -3.642063e+01 -3.147729e+01 7.596113e-15
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -3.147729e+01 7.596113e-15
      vertex   -3.607379e+01 -3.104235e+01 3.850000e+00
      vertex   -3.642063e+01 -3.147729e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -3.147729e+01 7.596113e-15
      vertex   -3.642063e+01 -3.147729e+01 3.850000e+00
      vertex   -3.692185e+01 -3.171866e+01 7.881824e-15
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.692185e+01 -3.171866e+01 7.881824e-15
      vertex   -3.642063e+01 -3.147729e+01 3.850000e+00
      vertex   -3.692185e+01 -3.171866e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.692185e+01 -3.171866e+01 7.881824e-15
      vertex   -3.692185e+01 -3.171866e+01 3.850000e+00
      vertex   -3.747815e+01 -3.171866e+01 7.881824e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -3.171866e+01 7.881824e-15
      vertex   -3.692185e+01 -3.171866e+01 3.850000e+00
      vertex   -3.747815e+01 -3.171866e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -3.171866e+01 7.881824e-15
      vertex   -3.747815e+01 -3.171866e+01 3.850000e+00
      vertex   -3.797936e+01 -3.147729e+01 7.596113e-15
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -3.147729e+01 7.596113e-15
      vertex   -3.747815e+01 -3.171866e+01 3.850000e+00
      vertex   -3.797936e+01 -3.147729e+01 3.850000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -3.147729e+01 7.596113e-15
      vertex   -3.797936e+01 -3.147729e+01 3.850000e+00
      vertex   -3.832621e+01 -3.104235e+01 7.081280e-15
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -3.104235e+01 7.081280e-15
      vertex   -3.797936e+01 -3.147729e+01 3.850000e+00
      vertex   -3.832621e+01 -3.104235e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -3.104235e+01 7.081280e-15
      vertex   -3.832621e+01 -3.104235e+01 3.850000e+00
      vertex   -3.845000e+01 -3.050000e+01 6.439294e-15
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.845000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.832621e+01 -3.104235e+01 3.850000e+00
      vertex   -3.845000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.845000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.845000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.832621e+01 -2.995765e+01 5.797307e-15
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.995765e+01 5.797307e-15
      vertex   -3.845000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.832621e+01 -2.995765e+01 3.850000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.995765e+01 5.797307e-15
      vertex   -3.832621e+01 -2.995765e+01 3.850000e+00
      vertex   -3.797936e+01 -2.952271e+01 5.282474e-15
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.952271e+01 5.282474e-15
      vertex   -3.832621e+01 -2.995765e+01 3.850000e+00
      vertex   -3.797936e+01 -2.952271e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.952271e+01 5.282474e-15
      vertex   -3.797936e+01 -2.952271e+01 3.850000e+00
      vertex   -3.747815e+01 -2.928134e+01 4.996763e-15
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.928134e+01 4.996763e-15
      vertex   -3.797936e+01 -2.952271e+01 3.850000e+00
      vertex   -3.747815e+01 -2.928134e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.928134e+01 4.996763e-15
      vertex   -3.747815e+01 -2.928134e+01 3.850000e+00
      vertex   -3.692185e+01 -2.928134e+01 4.996763e-15
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.928134e+01 4.996763e-15
      vertex   -3.747815e+01 -2.928134e+01 3.850000e+00
      vertex   -3.692185e+01 -2.928134e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.928134e+01 4.996763e-15
      vertex   -3.692185e+01 -2.928134e+01 3.850000e+00
      vertex   -3.642063e+01 -2.952271e+01 5.282474e-15
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.952271e+01 5.282474e-15
      vertex   -3.692185e+01 -2.928134e+01 3.850000e+00
      vertex   -3.642063e+01 -2.952271e+01 3.850000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.952271e+01 5.282474e-15
      vertex   -3.642063e+01 -2.952271e+01 3.850000e+00
      vertex   -3.607379e+01 -2.995765e+01 5.797307e-15
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 5.797307e-15
      vertex   -3.642063e+01 -2.952271e+01 3.850000e+00
      vertex   -3.607379e+01 -2.995765e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 1.495000e+01
      vertex   -3.607379e+01 -3.104235e+01 8.950000e+00
      vertex   -3.595000e+01 -3.050000e+01 1.495000e+01
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.607379e+01 -3.104235e+01 8.950000e+00
      vertex   -3.595000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.595000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.607379e+01 -2.995765e+01 1.495000e+01
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 1.495000e+01
      vertex   -3.595000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.607379e+01 -2.995765e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 1.495000e+01
      vertex   -3.607379e+01 -2.995765e+01 8.950000e+00
      vertex   -3.642063e+01 -2.952271e+01 1.495000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.952271e+01 1.495000e+01
      vertex   -3.607379e+01 -2.995765e+01 8.950000e+00
      vertex   -3.642063e+01 -2.952271e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.952271e+01 1.495000e+01
      vertex   -3.642063e+01 -2.952271e+01 8.950000e+00
      vertex   -3.692185e+01 -2.928134e+01 1.495000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.928134e+01 1.495000e+01
      vertex   -3.642063e+01 -2.952271e+01 8.950000e+00
      vertex   -3.692185e+01 -2.928134e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.928134e+01 1.495000e+01
      vertex   -3.692185e+01 -2.928134e+01 8.950000e+00
      vertex   -3.747815e+01 -2.928134e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.928134e+01 1.495000e+01
      vertex   -3.692185e+01 -2.928134e+01 8.950000e+00
      vertex   -3.747815e+01 -2.928134e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.928134e+01 1.495000e+01
      vertex   -3.747815e+01 -2.928134e+01 8.950000e+00
      vertex   -3.797936e+01 -2.952271e+01 1.495000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.952271e+01 1.495000e+01
      vertex   -3.747815e+01 -2.928134e+01 8.950000e+00
      vertex   -3.797936e+01 -2.952271e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.952271e+01 1.495000e+01
      vertex   -3.797936e+01 -2.952271e+01 8.950000e+00
      vertex   -3.832621e+01 -2.995765e+01 1.495000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.995765e+01 1.495000e+01
      vertex   -3.797936e+01 -2.952271e+01 8.950000e+00
      vertex   -3.832621e+01 -2.995765e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.995765e+01 1.495000e+01
      vertex   -3.832621e+01 -2.995765e+01 8.950000e+00
      vertex   -3.845000e+01 -3.050000e+01 1.495000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.845000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.832621e+01 -2.995765e+01 8.950000e+00
      vertex   -3.845000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 -0.000000e+00
    outer loop
      vertex   -3.845000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.845000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.832621e+01 -3.104235e+01 1.495000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 -0.000000e+00
    outer loop
      vertex   -3.832621e+01 -3.104235e+01 1.495000e+01
      vertex   -3.845000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.832621e+01 -3.104235e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 -0.000000e+00
    outer loop
      vertex   -3.832621e+01 -3.104235e+01 1.495000e+01
      vertex   -3.832621e+01 -3.104235e+01 8.950000e+00
      vertex   -3.797936e+01 -3.147729e+01 1.495000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 -0.000000e+00
    outer loop
      vertex   -3.797936e+01 -3.147729e+01 1.495000e+01
      vertex   -3.832621e+01 -3.104235e+01 8.950000e+00
      vertex   -3.797936e+01 -3.147729e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 -0.000000e+00
    outer loop
      vertex   -3.797936e+01 -3.147729e+01 1.495000e+01
      vertex   -3.797936e+01 -3.147729e+01 8.950000e+00
      vertex   -3.747815e+01 -3.171866e+01 1.495000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 -0.000000e+00
    outer loop
      vertex   -3.747815e+01 -3.171866e+01 1.495000e+01
      vertex   -3.797936e+01 -3.147729e+01 8.950000e+00
      vertex   -3.747815e+01 -3.171866e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -3.171866e+01 1.495000e+01
      vertex   -3.747815e+01 -3.171866e+01 8.950000e+00
      vertex   -3.692185e+01 -3.171866e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.692185e+01 -3.171866e+01 1.495000e+01
      vertex   -3.747815e+01 -3.171866e+01 8.950000e+00
      vertex   -3.692185e+01 -3.171866e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.692185e+01 -3.171866e+01 1.495000e+01
      vertex   -3.692185e+01 -3.171866e+01 8.950000e+00
      vertex   -3.642063e+01 -3.147729e+01 1.495000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -3.147729e+01 1.495000e+01
      vertex   -3.692185e+01 -3.171866e+01 8.950000e+00
      vertex   -3.642063e+01 -3.147729e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -3.147729e+01 1.495000e+01
      vertex   -3.642063e+01 -3.147729e+01 8.950000e+00
      vertex   -3.607379e+01 -3.104235e+01 1.495000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 1.495000e+01
      vertex   -3.642063e+01 -3.147729e+01 8.950000e+00
      vertex   -3.607379e+01 -3.104235e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 1.495000e+01
      vertex   -3.832621e+01 -2.995765e+01 1.495000e+01
      vertex   -3.595000e+01 -3.050000e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.832621e+01 -2.995765e+01 1.495000e+01
      vertex   -3.845000e+01 -3.050000e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.845000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.607379e+01 -3.104235e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 1.495000e+01
      vertex   -3.845000e+01 -3.050000e+01 1.495000e+01
      vertex   -3.832621e+01 -3.104235e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 1.495000e+01
      vertex   -3.832621e+01 -3.104235e+01 1.495000e+01
      vertex   -3.642063e+01 -3.147729e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -3.642063e+01 -3.147729e+01 1.495000e+01
      vertex   -3.832621e+01 -3.104235e+01 1.495000e+01
      vertex   -3.797936e+01 -3.147729e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.642063e+01 -3.147729e+01 1.495000e+01
      vertex   -3.797936e+01 -3.147729e+01 1.495000e+01
      vertex   -3.692185e+01 -3.171866e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.692185e+01 -3.171866e+01 1.495000e+01
      vertex   -3.797936e+01 -3.147729e+01 1.495000e+01
      vertex   -3.747815e+01 -3.171866e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.995765e+01 1.495000e+01
      vertex   -3.607379e+01 -2.995765e+01 1.495000e+01
      vertex   -3.797936e+01 -2.952271e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.952271e+01 1.495000e+01
      vertex   -3.607379e+01 -2.995765e+01 1.495000e+01
      vertex   -3.642063e+01 -2.952271e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.952271e+01 1.495000e+01
      vertex   -3.642063e+01 -2.952271e+01 1.495000e+01
      vertex   -3.747815e+01 -2.928134e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.928134e+01 1.495000e+01
      vertex   -3.642063e+01 -2.952271e+01 1.495000e+01
      vertex   -3.692185e+01 -2.928134e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 7.081280e-15
      vertex   -3.832621e+01 -3.104235e+01 7.081280e-15
      vertex   -3.595000e+01 -3.050000e+01 6.439294e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.832621e+01 -3.104235e+01 7.081280e-15
      vertex   -3.845000e+01 -3.050000e+01 6.439294e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.845000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.607379e+01 -2.995765e+01 5.797307e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 5.797307e-15
      vertex   -3.845000e+01 -3.050000e+01 6.439294e-15
      vertex   -3.832621e+01 -2.995765e+01 5.797307e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 5.797307e-15
      vertex   -3.832621e+01 -2.995765e+01 5.797307e-15
      vertex   -3.642063e+01 -2.952271e+01 5.282474e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.952271e+01 5.282474e-15
      vertex   -3.832621e+01 -2.995765e+01 5.797307e-15
      vertex   -3.797936e+01 -2.952271e+01 5.282474e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.952271e+01 5.282474e-15
      vertex   -3.797936e+01 -2.952271e+01 5.282474e-15
      vertex   -3.692185e+01 -2.928134e+01 4.996763e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.928134e+01 4.996763e-15
      vertex   -3.797936e+01 -2.952271e+01 5.282474e-15
      vertex   -3.747815e+01 -2.928134e+01 4.996763e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.832621e+01 -3.104235e+01 7.081280e-15
      vertex   -3.607379e+01 -3.104235e+01 7.081280e-15
      vertex   -3.797936e+01 -3.147729e+01 7.596113e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.797936e+01 -3.147729e+01 7.596113e-15
      vertex   -3.607379e+01 -3.104235e+01 7.081280e-15
      vertex   -3.642063e+01 -3.147729e+01 7.596113e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.797936e+01 -3.147729e+01 7.596113e-15
      vertex   -3.642063e+01 -3.147729e+01 7.596113e-15
      vertex   -3.747815e+01 -3.171866e+01 7.881824e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -3.747815e+01 -3.171866e+01 7.881824e-15
      vertex   -3.642063e+01 -3.147729e+01 7.596113e-15
      vertex   -3.692185e+01 -3.171866e+01 7.881824e-15
    endloop
  endfacet
  facet normal 8.361427e-01 5.477680e-01 -2.855823e-02
    outer loop
      vertex   -3.492941e+01 -2.945384e+01 5.963679e+00
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
    endloop
  endfacet
  facet normal 8.909416e-01 4.539574e-01 -1.206916e-02
    outer loop
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
    endloop
  endfacet
  facet normal 8.528132e-01 5.222160e-01 -3.660903e-16
    outer loop
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
      vertex   -3.517745e+01 -2.903054e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.124542e-01 7.017186e-01 -4.888246e-16
    outer loop
      vertex   -3.517745e+01 -2.903054e+01 8.950000e+00
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.071038e-01 7.071038e-01 2.898902e-03
    outer loop
      vertex   -3.517745e+01 -2.903054e+01 8.950000e+00
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.573053e+01 -2.847746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.923602e-01 8.703884e-01 -2.351579e-03
    outer loop
      vertex   -3.573053e+01 -2.847746e+01 8.950000e+00
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.625855e+01 -2.818404e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.539623e-01 8.909512e-01 1.113981e-02
    outer loop
      vertex   -3.573053e+01 -2.847746e+01 8.950000e+00
      vertex   -3.625855e+01 -2.818404e+01 7.000000e+00
      vertex   -3.642745e+01 -2.812236e+01 8.950000e+00
    endloop
  endfacet
  facet normal 2.545582e-01 9.670198e-01 -8.538277e-03
    outer loop
      vertex   -3.642745e+01 -2.812236e+01 8.950000e+00
      vertex   -3.625855e+01 -2.818404e+01 7.000000e+00
      vertex   -3.687913e+01 -2.802068e+01 7.000000e+00
    endloop
  endfacet
  facet normal 1.564162e-01 9.875732e-01 1.526616e-02
    outer loop
      vertex   -3.642745e+01 -2.812236e+01 8.950000e+00
      vertex   -3.687913e+01 -2.802068e+01 7.000000e+00
      vertex   -3.720000e+01 -2.800000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 9.999438e-01 -1.060254e-02
    outer loop
      vertex   -3.720000e+01 -2.800000e+01 8.950000e+00
      vertex   -3.687913e+01 -2.802068e+01 7.000000e+00
      vertex   -3.752086e+01 -2.802068e+01 7.000000e+00
    endloop
  endfacet
  facet normal -1.564162e-01 9.875732e-01 1.526616e-02
    outer loop
      vertex   -3.720000e+01 -2.800000e+01 8.950000e+00
      vertex   -3.752086e+01 -2.802068e+01 7.000000e+00
      vertex   -3.797254e+01 -2.812236e+01 8.950000e+00
    endloop
  endfacet
  facet normal -2.545582e-01 9.670198e-01 -8.538277e-03
    outer loop
      vertex   -3.797254e+01 -2.812236e+01 8.950000e+00
      vertex   -3.752086e+01 -2.802068e+01 7.000000e+00
      vertex   -3.814144e+01 -2.818404e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.539623e-01 8.909512e-01 1.113981e-02
    outer loop
      vertex   -3.797254e+01 -2.812236e+01 8.950000e+00
      vertex   -3.814144e+01 -2.818404e+01 7.000000e+00
      vertex   -3.866946e+01 -2.847746e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.923602e-01 8.703884e-01 -2.351579e-03
    outer loop
      vertex   -3.866946e+01 -2.847746e+01 8.950000e+00
      vertex   -3.814144e+01 -2.818404e+01 7.000000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -7.071038e-01 7.071038e-01 2.898902e-03
    outer loop
      vertex   -3.866946e+01 -2.847746e+01 8.950000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.124542e-01 7.017186e-01 -4.888246e-16
    outer loop
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
    endloop
  endfacet
  facet normal -8.528132e-01 5.222160e-01 -3.637812e-16
    outer loop
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
      vertex   -3.942826e+01 -2.936650e+01 6.400000e+00
    endloop
  endfacet
  facet normal -8.816187e-01 4.718774e-01 -8.956054e-03
    outer loop
      vertex   -3.942826e+01 -2.936650e+01 6.400000e+00
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
      vertex   -3.947058e+01 -2.945384e+01 5.963679e+00
    endloop
  endfacet
  facet normal -8.909237e-01 4.539483e-01 -1.363866e-02
    outer loop
      vertex   -3.947058e+01 -2.945384e+01 5.963679e+00
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
      vertex   -3.957764e+01 -2.972746e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.284212e-01 3.715278e-01 -1.070121e-03
    outer loop
      vertex   -3.947058e+01 -2.945384e+01 5.963679e+00
      vertex   -3.957764e+01 -2.972746e+01 3.850000e+00
      vertex   -3.956785e+01 -2.969797e+01 5.599019e+00
    endloop
  endfacet
  facet normal -9.688264e-01 2.477376e-01 1.242888e-03
    outer loop
      vertex   -3.956785e+01 -2.969797e+01 5.599019e+00
      vertex   -3.957764e+01 -2.972746e+01 3.850000e+00
      vertex   -3.966150e+01 -3.006296e+01 5.354142e+00
    endloop
  endfacet
  facet normal -9.874874e-01 1.564026e-01 -2.017014e-02
    outer loop
      vertex   -3.966150e+01 -3.006296e+01 5.354142e+00
      vertex   -3.957764e+01 -2.972746e+01 3.850000e+00
      vertex   -3.970000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.961429e-01 8.774608e-02 -2.177261e-16
    outer loop
      vertex   -3.966150e+01 -3.006296e+01 5.354142e+00
      vertex   -3.970000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.970000e+01 -3.050000e+01 5.266500e+00
    endloop
  endfacet
  facet normal -9.961429e-01 -8.774608e-02 2.200753e-16
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 5.266500e+00
      vertex   -3.970000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.966150e+01 -3.093704e+01 5.354142e+00
    endloop
  endfacet
  facet normal -9.874874e-01 -1.564026e-01 -2.017014e-02
    outer loop
      vertex   -3.966150e+01 -3.093704e+01 5.354142e+00
      vertex   -3.970000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.688264e-01 -2.477376e-01 1.242888e-03
    outer loop
      vertex   -3.966150e+01 -3.093704e+01 5.354142e+00
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
      vertex   -3.956785e+01 -3.130203e+01 5.599019e+00
    endloop
  endfacet
  facet normal -9.284212e-01 -3.715278e-01 -1.070121e-03
    outer loop
      vertex   -3.956785e+01 -3.130203e+01 5.599019e+00
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
      vertex   -3.947058e+01 -3.154616e+01 5.963679e+00
    endloop
  endfacet
  facet normal -8.361427e-01 -5.477680e-01 -2.855823e-02
    outer loop
      vertex   -3.947058e+01 -3.154616e+01 5.963679e+00
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
    endloop
  endfacet
  facet normal -8.909416e-01 -4.539574e-01 -1.206916e-02
    outer loop
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
      vertex   -3.922254e+01 -3.196946e+01 3.850000e+00
    endloop
  endfacet
  facet normal -8.528132e-01 -5.222160e-01 3.660903e-16
    outer loop
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
      vertex   -3.922254e+01 -3.196946e+01 3.850000e+00
      vertex   -3.922254e+01 -3.196946e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.071068e-01 -7.071068e-01 4.925780e-16
    outer loop
      vertex   -3.922254e+01 -3.196946e+01 8.950000e+00
      vertex   -3.922254e+01 -3.196946e+01 3.850000e+00
      vertex   -3.866946e+01 -3.252254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.071068e-01 -7.071068e-01 9.851561e-16
    outer loop
      vertex   -3.866946e+01 -3.252254e+01 8.950000e+00
      vertex   -3.922254e+01 -3.196946e+01 3.850000e+00
      vertex   -3.866946e+01 -3.252254e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.539905e-01 -8.910065e-01 1.241369e-15
    outer loop
      vertex   -3.866946e+01 -3.252254e+01 8.950000e+00
      vertex   -3.866946e+01 -3.252254e+01 3.850000e+00
      vertex   -3.797254e+01 -3.287764e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.539905e-01 -8.910065e-01 -0.000000e+00
    outer loop
      vertex   -3.797254e+01 -3.287764e+01 8.950000e+00
      vertex   -3.866946e+01 -3.252254e+01 3.850000e+00
      vertex   -3.797254e+01 -3.287764e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.564345e-01 -9.876883e-01 -0.000000e+00
    outer loop
      vertex   -3.797254e+01 -3.287764e+01 8.950000e+00
      vertex   -3.797254e+01 -3.287764e+01 3.850000e+00
      vertex   -3.720000e+01 -3.300000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.564345e-01 -9.876883e-01 -0.000000e+00
    outer loop
      vertex   -3.720000e+01 -3.300000e+01 8.950000e+00
      vertex   -3.797254e+01 -3.287764e+01 3.850000e+00
      vertex   -3.720000e+01 -3.300000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.564345e-01 -9.876883e-01 0.000000e+00
    outer loop
      vertex   -3.720000e+01 -3.300000e+01 8.950000e+00
      vertex   -3.720000e+01 -3.300000e+01 3.850000e+00
      vertex   -3.642745e+01 -3.287764e+01 8.950000e+00
    endloop
  endfacet
  facet normal 1.564345e-01 -9.876883e-01 0.000000e+00
    outer loop
      vertex   -3.642745e+01 -3.287764e+01 8.950000e+00
      vertex   -3.720000e+01 -3.300000e+01 3.850000e+00
      vertex   -3.642745e+01 -3.287764e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.539905e-01 -8.910065e-01 0.000000e+00
    outer loop
      vertex   -3.642745e+01 -3.287764e+01 8.950000e+00
      vertex   -3.642745e+01 -3.287764e+01 3.850000e+00
      vertex   -3.573053e+01 -3.252254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.539905e-01 -8.910065e-01 1.241369e-15
    outer loop
      vertex   -3.573053e+01 -3.252254e+01 8.950000e+00
      vertex   -3.642745e+01 -3.287764e+01 3.850000e+00
      vertex   -3.573053e+01 -3.252254e+01 3.850000e+00
    endloop
  endfacet
  facet normal 7.071068e-01 -7.071068e-01 9.851561e-16
    outer loop
      vertex   -3.573053e+01 -3.252254e+01 8.950000e+00
      vertex   -3.573053e+01 -3.252254e+01 3.850000e+00
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.071068e-01 -7.071068e-01 4.925780e-16
    outer loop
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
      vertex   -3.573053e+01 -3.252254e+01 3.850000e+00
      vertex   -3.517745e+01 -3.196946e+01 3.850000e+00
    endloop
  endfacet
  facet normal 8.528132e-01 -5.222160e-01 3.637812e-16
    outer loop
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
      vertex   -3.517745e+01 -3.196946e+01 3.850000e+00
      vertex   -3.497173e+01 -3.163350e+01 6.400000e+00
    endloop
  endfacet
  facet normal 8.816187e-01 -4.718774e-01 -8.956054e-03
    outer loop
      vertex   -3.497173e+01 -3.163350e+01 6.400000e+00
      vertex   -3.517745e+01 -3.196946e+01 3.850000e+00
      vertex   -3.492941e+01 -3.154616e+01 5.963679e+00
    endloop
  endfacet
  facet normal 8.909237e-01 -4.539483e-01 -1.363866e-02
    outer loop
      vertex   -3.492941e+01 -3.154616e+01 5.963679e+00
      vertex   -3.517745e+01 -3.196946e+01 3.850000e+00
      vertex   -3.482236e+01 -3.127254e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.284212e-01 -3.715278e-01 -1.070121e-03
    outer loop
      vertex   -3.492941e+01 -3.154616e+01 5.963679e+00
      vertex   -3.482236e+01 -3.127254e+01 3.850000e+00
      vertex   -3.483214e+01 -3.130203e+01 5.599019e+00
    endloop
  endfacet
  facet normal 9.688264e-01 -2.477376e-01 1.242888e-03
    outer loop
      vertex   -3.483214e+01 -3.130203e+01 5.599019e+00
      vertex   -3.482236e+01 -3.127254e+01 3.850000e+00
      vertex   -3.473849e+01 -3.093704e+01 5.354142e+00
    endloop
  endfacet
  facet normal 9.874874e-01 -1.564026e-01 -2.017014e-02
    outer loop
      vertex   -3.473849e+01 -3.093704e+01 5.354142e+00
      vertex   -3.482236e+01 -3.127254e+01 3.850000e+00
      vertex   -3.470000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.961429e-01 -8.774608e-02 1.021638e-14
    outer loop
      vertex   -3.473849e+01 -3.093704e+01 5.354142e+00
      vertex   -3.470000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.470000e+01 -3.050000e+01 5.266500e+00
    endloop
  endfacet
  facet normal 9.961429e-01 8.774608e-02 9.773600e-15
    outer loop
      vertex   -3.470000e+01 -3.050000e+01 5.266500e+00
      vertex   -3.470000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.473849e+01 -3.006296e+01 5.354142e+00
    endloop
  endfacet
  facet normal 9.874874e-01 1.564026e-01 -2.017014e-02
    outer loop
      vertex   -3.473849e+01 -3.006296e+01 5.354142e+00
      vertex   -3.470000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.688264e-01 2.477376e-01 1.242888e-03
    outer loop
      vertex   -3.473849e+01 -3.006296e+01 5.354142e+00
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
      vertex   -3.483214e+01 -2.969797e+01 5.599019e+00
    endloop
  endfacet
  facet normal 9.284212e-01 3.715278e-01 -1.070121e-03
    outer loop
      vertex   -3.483214e+01 -2.969797e+01 5.599019e+00
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
      vertex   -3.492941e+01 -2.945384e+01 5.963679e+00
    endloop
  endfacet
  facet normal 8.816187e-01 -4.718774e-01 8.956054e-03
    outer loop
      vertex   -3.497173e+01 -3.163350e+01 6.400000e+00
      vertex   -3.492941e+01 -3.154616e+01 6.836321e+00
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
    endloop
  endfacet
  facet normal 8.909237e-01 -4.539483e-01 1.363866e-02
    outer loop
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
      vertex   -3.492941e+01 -3.154616e+01 6.836321e+00
      vertex   -3.482236e+01 -3.127254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.284212e-01 -3.715278e-01 1.070121e-03
    outer loop
      vertex   -3.482236e+01 -3.127254e+01 8.950000e+00
      vertex   -3.492941e+01 -3.154616e+01 6.836321e+00
      vertex   -3.483214e+01 -3.130203e+01 7.200981e+00
    endloop
  endfacet
  facet normal 9.688264e-01 -2.477376e-01 -1.242888e-03
    outer loop
      vertex   -3.482236e+01 -3.127254e+01 8.950000e+00
      vertex   -3.483214e+01 -3.130203e+01 7.200981e+00
      vertex   -3.473849e+01 -3.093704e+01 7.445858e+00
    endloop
  endfacet
  facet normal 9.874874e-01 -1.564026e-01 2.017014e-02
    outer loop
      vertex   -3.482236e+01 -3.127254e+01 8.950000e+00
      vertex   -3.473849e+01 -3.093704e+01 7.445858e+00
      vertex   -3.470000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.961429e-01 -8.774608e-02 0.000000e+00
    outer loop
      vertex   -3.470000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.473849e+01 -3.093704e+01 7.445858e+00
      vertex   -3.470000e+01 -3.050000e+01 7.533500e+00
    endloop
  endfacet
  facet normal 9.961429e-01 8.774608e-02 0.000000e+00
    outer loop
      vertex   -3.470000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.470000e+01 -3.050000e+01 7.533500e+00
      vertex   -3.473849e+01 -3.006296e+01 7.445858e+00
    endloop
  endfacet
  facet normal 9.874874e-01 1.564026e-01 2.017014e-02
    outer loop
      vertex   -3.470000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.473849e+01 -3.006296e+01 7.445858e+00
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.688264e-01 2.477376e-01 -1.242888e-03
    outer loop
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
      vertex   -3.473849e+01 -3.006296e+01 7.445858e+00
      vertex   -3.483214e+01 -2.969797e+01 7.200981e+00
    endloop
  endfacet
  facet normal 9.284212e-01 3.715278e-01 1.070121e-03
    outer loop
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
      vertex   -3.483214e+01 -2.969797e+01 7.200981e+00
      vertex   -3.492941e+01 -2.945384e+01 6.836321e+00
    endloop
  endfacet
  facet normal 8.361427e-01 5.477680e-01 2.855823e-02
    outer loop
      vertex   -3.492941e+01 -2.945384e+01 6.836321e+00
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 8.909416e-01 4.539574e-01 1.206916e-02
    outer loop
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
      vertex   -3.497173e+01 -2.936650e+01 6.400000e+00
      vertex   -3.517745e+01 -2.903054e+01 8.950000e+00
    endloop
  endfacet
  facet normal -8.816187e-01 4.718774e-01 8.956054e-03
    outer loop
      vertex   -3.942826e+01 -2.936650e+01 6.400000e+00
      vertex   -3.947058e+01 -2.945384e+01 6.836321e+00
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
    endloop
  endfacet
  facet normal -8.909237e-01 4.539483e-01 1.363866e-02
    outer loop
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
      vertex   -3.947058e+01 -2.945384e+01 6.836321e+00
      vertex   -3.957764e+01 -2.972746e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.284212e-01 3.715278e-01 1.070121e-03
    outer loop
      vertex   -3.957764e+01 -2.972746e+01 8.950000e+00
      vertex   -3.947058e+01 -2.945384e+01 6.836321e+00
      vertex   -3.956785e+01 -2.969797e+01 7.200981e+00
    endloop
  endfacet
  facet normal -9.688264e-01 2.477376e-01 -1.242888e-03
    outer loop
      vertex   -3.957764e+01 -2.972746e+01 8.950000e+00
      vertex   -3.956785e+01 -2.969797e+01 7.200981e+00
      vertex   -3.966150e+01 -3.006296e+01 7.445858e+00
    endloop
  endfacet
  facet normal -9.874874e-01 1.564026e-01 2.017014e-02
    outer loop
      vertex   -3.957764e+01 -2.972746e+01 8.950000e+00
      vertex   -3.966150e+01 -3.006296e+01 7.445858e+00
      vertex   -3.970000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.961429e-01 8.774608e-02 -4.996838e-15
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.966150e+01 -3.006296e+01 7.445858e+00
      vertex   -3.970000e+01 -3.050000e+01 7.533500e+00
    endloop
  endfacet
  facet normal -9.961429e-01 -8.774608e-02 -4.996838e-15
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.970000e+01 -3.050000e+01 7.533500e+00
      vertex   -3.966150e+01 -3.093704e+01 7.445858e+00
    endloop
  endfacet
  facet normal -9.874874e-01 -1.564026e-01 2.017014e-02
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.966150e+01 -3.093704e+01 7.445858e+00
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.688264e-01 -2.477376e-01 -1.242888e-03
    outer loop
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
      vertex   -3.966150e+01 -3.093704e+01 7.445858e+00
      vertex   -3.956785e+01 -3.130203e+01 7.200981e+00
    endloop
  endfacet
  facet normal -9.284212e-01 -3.715278e-01 1.070121e-03
    outer loop
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
      vertex   -3.956785e+01 -3.130203e+01 7.200981e+00
      vertex   -3.947058e+01 -3.154616e+01 6.836321e+00
    endloop
  endfacet
  facet normal -8.361427e-01 -5.477680e-01 2.855823e-02
    outer loop
      vertex   -3.947058e+01 -3.154616e+01 6.836321e+00
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -8.909416e-01 -4.539574e-01 1.206916e-02
    outer loop
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
      vertex   -3.942826e+01 -3.163350e+01 6.400000e+00
      vertex   -3.922254e+01 -3.196946e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.124542e-01 7.017186e-01 0.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
      vertex   -3.570000e+01 -2.850000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 7.070982e-01 7.070982e-01 -4.915491e-03
    outer loop
      vertex   -3.570000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
      vertex   -3.573053e+01 -2.847746e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.923576e-01 8.703839e-01 3.987439e-03
    outer loop
      vertex   -3.570000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.573053e+01 -2.847746e+01 3.850000e+00
      vertex   -3.625855e+01 -2.818404e+01 5.000000e+00
    endloop
  endfacet
  facet normal 4.539095e-01 8.908476e-01 -1.888704e-02
    outer loop
      vertex   -3.625855e+01 -2.818404e+01 5.000000e+00
      vertex   -3.573053e+01 -2.847746e+01 3.850000e+00
      vertex   -3.642745e+01 -2.812236e+01 3.850000e+00
    endloop
  endfacet
  facet normal 2.545408e-01 9.669537e-01 1.447696e-02
    outer loop
      vertex   -3.625855e+01 -2.818404e+01 5.000000e+00
      vertex   -3.642745e+01 -2.812236e+01 3.850000e+00
      vertex   -3.687913e+01 -2.802068e+01 5.000000e+00
    endloop
  endfacet
  facet normal 1.563821e-01 9.873575e-01 -2.588044e-02
    outer loop
      vertex   -3.687913e+01 -2.802068e+01 5.000000e+00
      vertex   -3.642745e+01 -2.812236e+01 3.850000e+00
      vertex   -3.720000e+01 -2.800000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 9.998384e-01 1.797633e-02
    outer loop
      vertex   -3.687913e+01 -2.802068e+01 5.000000e+00
      vertex   -3.720000e+01 -2.800000e+01 3.850000e+00
      vertex   -3.752086e+01 -2.802068e+01 5.000000e+00
    endloop
  endfacet
  facet normal -1.563821e-01 9.873575e-01 -2.588044e-02
    outer loop
      vertex   -3.752086e+01 -2.802068e+01 5.000000e+00
      vertex   -3.720000e+01 -2.800000e+01 3.850000e+00
      vertex   -3.797254e+01 -2.812236e+01 3.850000e+00
    endloop
  endfacet
  facet normal -2.545408e-01 9.669537e-01 1.447696e-02
    outer loop
      vertex   -3.752086e+01 -2.802068e+01 5.000000e+00
      vertex   -3.797254e+01 -2.812236e+01 3.850000e+00
      vertex   -3.814144e+01 -2.818404e+01 5.000000e+00
    endloop
  endfacet
  facet normal -4.539095e-01 8.908476e-01 -1.888704e-02
    outer loop
      vertex   -3.814144e+01 -2.818404e+01 5.000000e+00
      vertex   -3.797254e+01 -2.812236e+01 3.850000e+00
      vertex   -3.866946e+01 -2.847746e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.923576e-01 8.703839e-01 3.987439e-03
    outer loop
      vertex   -3.814144e+01 -2.818404e+01 5.000000e+00
      vertex   -3.866946e+01 -2.847746e+01 3.850000e+00
      vertex   -3.870000e+01 -2.850000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -7.070982e-01 7.070982e-01 -4.915491e-03
    outer loop
      vertex   -3.870000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.866946e+01 -2.847746e+01 3.850000e+00
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
    endloop
  endfacet
  facet normal -7.124542e-01 7.017186e-01 0.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -3.104235e+01 8.950000e+00
      vertex   -3.482236e+01 -3.127254e+01 8.950000e+00
      vertex   -3.595000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.482236e+01 -3.127254e+01 8.950000e+00
      vertex   -3.470000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.470000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.286626e-16 2.330451e-15 1.000000e+00
    outer loop
      vertex   -3.482236e+01 -3.127254e+01 8.950000e+00
      vertex   -3.607379e+01 -3.104235e+01 8.950000e+00
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.086008e-15 8.660624e-16 1.000000e+00
    outer loop
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
      vertex   -3.607379e+01 -3.104235e+01 8.950000e+00
      vertex   -3.642063e+01 -3.147729e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.023628e-15 1.023628e-15 1.000000e+00
    outer loop
      vertex   -3.517745e+01 -3.196946e+01 8.950000e+00
      vertex   -3.642063e+01 -3.147729e+01 8.950000e+00
      vertex   -3.573053e+01 -3.252254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -6.479446e-16 1.271663e-15 1.000000e+00
    outer loop
      vertex   -3.573053e+01 -3.252254e+01 8.950000e+00
      vertex   -3.642063e+01 -3.147729e+01 8.950000e+00
      vertex   -3.642745e+01 -3.287764e+01 8.950000e+00
    endloop
  endfacet
  facet normal -6.123170e-16 1.271489e-15 1.000000e+00
    outer loop
      vertex   -3.642745e+01 -3.287764e+01 8.950000e+00
      vertex   -3.642063e+01 -3.147729e+01 8.950000e+00
      vertex   -3.692185e+01 -3.171866e+01 8.950000e+00
    endloop
  endfacet
  facet normal -2.273908e-16 1.435689e-15 1.000000e+00
    outer loop
      vertex   -3.642745e+01 -3.287764e+01 8.950000e+00
      vertex   -3.692185e+01 -3.171866e+01 8.950000e+00
      vertex   -3.720000e+01 -3.300000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.386327e-15 1.000000e+00
    outer loop
      vertex   -3.720000e+01 -3.300000e+01 8.950000e+00
      vertex   -3.692185e+01 -3.171866e+01 8.950000e+00
      vertex   -3.747815e+01 -3.171866e+01 8.950000e+00
    endloop
  endfacet
  facet normal 2.273908e-16 1.435689e-15 1.000000e+00
    outer loop
      vertex   -3.720000e+01 -3.300000e+01 8.950000e+00
      vertex   -3.747815e+01 -3.171866e+01 8.950000e+00
      vertex   -3.797254e+01 -3.287764e+01 8.950000e+00
    endloop
  endfacet
  facet normal 6.123170e-16 1.271489e-15 1.000000e+00
    outer loop
      vertex   -3.797254e+01 -3.287764e+01 8.950000e+00
      vertex   -3.747815e+01 -3.171866e+01 8.950000e+00
      vertex   -3.797936e+01 -3.147729e+01 8.950000e+00
    endloop
  endfacet
  facet normal 6.479446e-16 1.271663e-15 1.000000e+00
    outer loop
      vertex   -3.797254e+01 -3.287764e+01 8.950000e+00
      vertex   -3.797936e+01 -3.147729e+01 8.950000e+00
      vertex   -3.866946e+01 -3.252254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 1.023628e-15 1.023628e-15 1.000000e+00
    outer loop
      vertex   -3.866946e+01 -3.252254e+01 8.950000e+00
      vertex   -3.797936e+01 -3.147729e+01 8.950000e+00
      vertex   -3.922254e+01 -3.196946e+01 8.950000e+00
    endloop
  endfacet
  facet normal 1.086008e-15 8.660624e-16 1.000000e+00
    outer loop
      vertex   -3.922254e+01 -3.196946e+01 8.950000e+00
      vertex   -3.797936e+01 -3.147729e+01 8.950000e+00
      vertex   -3.832621e+01 -3.104235e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.286626e-16 2.330451e-15 1.000000e+00
    outer loop
      vertex   -3.922254e+01 -3.196946e+01 8.950000e+00
      vertex   -3.832621e+01 -3.104235e+01 8.950000e+00
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
      vertex   -3.832621e+01 -3.104235e+01 8.950000e+00
      vertex   -3.845000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.957764e+01 -3.127254e+01 8.950000e+00
      vertex   -3.845000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.970000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.845000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.957764e+01 -2.972746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.957764e+01 -2.972746e+01 8.950000e+00
      vertex   -3.845000e+01 -3.050000e+01 8.950000e+00
      vertex   -3.832621e+01 -2.995765e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.957764e+01 -2.972746e+01 8.950000e+00
      vertex   -3.832621e+01 -2.995765e+01 8.950000e+00
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
      vertex   -3.832621e+01 -2.995765e+01 8.950000e+00
      vertex   -3.797936e+01 -2.952271e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.922254e+01 -2.903054e+01 8.950000e+00
      vertex   -3.797936e+01 -2.952271e+01 8.950000e+00
      vertex   -3.866946e+01 -2.847746e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.866946e+01 -2.847746e+01 8.950000e+00
      vertex   -3.797936e+01 -2.952271e+01 8.950000e+00
      vertex   -3.797254e+01 -2.812236e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.797254e+01 -2.812236e+01 8.950000e+00
      vertex   -3.797936e+01 -2.952271e+01 8.950000e+00
      vertex   -3.747815e+01 -2.928134e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.797254e+01 -2.812236e+01 8.950000e+00
      vertex   -3.747815e+01 -2.928134e+01 8.950000e+00
      vertex   -3.720000e+01 -2.800000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.720000e+01 -2.800000e+01 8.950000e+00
      vertex   -3.747815e+01 -2.928134e+01 8.950000e+00
      vertex   -3.692185e+01 -2.928134e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.720000e+01 -2.800000e+01 8.950000e+00
      vertex   -3.692185e+01 -2.928134e+01 8.950000e+00
      vertex   -3.642745e+01 -2.812236e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.642745e+01 -2.812236e+01 8.950000e+00
      vertex   -3.692185e+01 -2.928134e+01 8.950000e+00
      vertex   -3.642063e+01 -2.952271e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.642745e+01 -2.812236e+01 8.950000e+00
      vertex   -3.642063e+01 -2.952271e+01 8.950000e+00
      vertex   -3.573053e+01 -2.847746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.573053e+01 -2.847746e+01 8.950000e+00
      vertex   -3.642063e+01 -2.952271e+01 8.950000e+00
      vertex   -3.517745e+01 -2.903054e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.517745e+01 -2.903054e+01 8.950000e+00
      vertex   -3.642063e+01 -2.952271e+01 8.950000e+00
      vertex   -3.607379e+01 -2.995765e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.517745e+01 -2.903054e+01 8.950000e+00
      vertex   -3.607379e+01 -2.995765e+01 8.950000e+00
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.482236e+01 -2.972746e+01 8.950000e+00
      vertex   -3.607379e+01 -2.995765e+01 8.950000e+00
      vertex   -3.595000e+01 -3.050000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -3.405673e-16 -7.773226e-17 -1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.995765e+01 3.850000e+00
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
      vertex   -3.595000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -5.748411e-16 -1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
      vertex   -3.470000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -5.748411e-16 -1.000000e+00
    outer loop
      vertex   -3.595000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.470000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.482236e+01 -3.127254e+01 3.850000e+00
    endloop
  endfacet
  facet normal -3.244566e-16 -1.653189e-16 -1.000000e+00
    outer loop
      vertex   -3.482236e+01 -2.972746e+01 3.850000e+00
      vertex   -3.607379e+01 -2.995765e+01 3.850000e+00
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.072327e-16 -7.760382e-16 -1.000000e+00
    outer loop
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
      vertex   -3.607379e+01 -2.995765e+01 3.850000e+00
      vertex   -3.642063e+01 -2.952271e+01 3.850000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -3.517745e+01 -2.903054e+01 3.850000e+00
      vertex   -3.642063e+01 -2.952271e+01 3.850000e+00
      vertex   -3.573053e+01 -2.847746e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.536306e-16 -6.296095e-16 -1.000000e+00
    outer loop
      vertex   -3.573053e+01 -2.847746e+01 3.850000e+00
      vertex   -3.642063e+01 -2.952271e+01 3.850000e+00
      vertex   -3.642745e+01 -2.812236e+01 3.850000e+00
    endloop
  endfacet
  facet normal -3.061585e-16 -6.357447e-16 -1.000000e+00
    outer loop
      vertex   -3.642745e+01 -2.812236e+01 3.850000e+00
      vertex   -3.642063e+01 -2.952271e+01 3.850000e+00
      vertex   -3.692185e+01 -2.928134e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.136954e-16 -7.178445e-16 -1.000000e+00
    outer loop
      vertex   -3.642745e+01 -2.812236e+01 3.850000e+00
      vertex   -3.692185e+01 -2.928134e+01 3.850000e+00
      vertex   -3.720000e+01 -2.800000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -6.931637e-16 -1.000000e+00
    outer loop
      vertex   -3.720000e+01 -2.800000e+01 3.850000e+00
      vertex   -3.692185e+01 -2.928134e+01 3.850000e+00
      vertex   -3.747815e+01 -2.928134e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.136954e-16 -7.178445e-16 -1.000000e+00
    outer loop
      vertex   -3.720000e+01 -2.800000e+01 3.850000e+00
      vertex   -3.747815e+01 -2.928134e+01 3.850000e+00
      vertex   -3.797254e+01 -2.812236e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.061585e-16 -6.357447e-16 -1.000000e+00
    outer loop
      vertex   -3.797254e+01 -2.812236e+01 3.850000e+00
      vertex   -3.747815e+01 -2.928134e+01 3.850000e+00
      vertex   -3.797936e+01 -2.952271e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.536306e-16 -6.296095e-16 -1.000000e+00
    outer loop
      vertex   -3.797254e+01 -2.812236e+01 3.850000e+00
      vertex   -3.797936e+01 -2.952271e+01 3.850000e+00
      vertex   -3.866946e+01 -2.847746e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 -1.000000e+00
    outer loop
      vertex   -3.866946e+01 -2.847746e+01 3.850000e+00
      vertex   -3.797936e+01 -2.952271e+01 3.850000e+00
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
    endloop
  endfacet
  facet normal -3.072327e-16 -7.760382e-16 -1.000000e+00
    outer loop
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
      vertex   -3.797936e+01 -2.952271e+01 3.850000e+00
      vertex   -3.832621e+01 -2.995765e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.244566e-16 -1.653189e-16 -1.000000e+00
    outer loop
      vertex   -3.922254e+01 -2.903054e+01 3.850000e+00
      vertex   -3.832621e+01 -2.995765e+01 3.850000e+00
      vertex   -3.957764e+01 -2.972746e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.405673e-16 -7.773226e-17 -1.000000e+00
    outer loop
      vertex   -3.957764e+01 -2.972746e+01 3.850000e+00
      vertex   -3.832621e+01 -2.995765e+01 3.850000e+00
      vertex   -3.845000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -5.748411e-16 -1.000000e+00
    outer loop
      vertex   -3.957764e+01 -2.972746e+01 3.850000e+00
      vertex   -3.845000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.970000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -5.748411e-16 -1.000000e+00
    outer loop
      vertex   -3.970000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.845000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
    endloop
  endfacet
  facet normal -3.405673e-16 -7.773226e-17 -1.000000e+00
    outer loop
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
      vertex   -3.845000e+01 -3.050000e+01 3.850000e+00
      vertex   -3.832621e+01 -3.104235e+01 3.850000e+00
    endloop
  endfacet
  facet normal -3.244566e-16 -1.653189e-16 -1.000000e+00
    outer loop
      vertex   -3.957764e+01 -3.127254e+01 3.850000e+00
      vertex   -3.832621e+01 -3.104235e+01 3.850000e+00
      vertex   -3.922254e+01 -3.196946e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.072327e-16 -7.760382e-16 -1.000000e+00
    outer loop
      vertex   -3.922254e+01 -3.196946e+01 3.850000e+00
      vertex   -3.832621e+01 -3.104235e+01 3.850000e+00
      vertex   -3.797936e+01 -3.147729e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -3.922254e+01 -3.196946e+01 3.850000e+00
      vertex   -3.797936e+01 -3.147729e+01 3.850000e+00
      vertex   -3.866946e+01 -3.252254e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.768153e-16 -3.148047e-16 -1.000000e+00
    outer loop
      vertex   -3.866946e+01 -3.252254e+01 3.850000e+00
      vertex   -3.797936e+01 -3.147729e+01 3.850000e+00
      vertex   -3.797254e+01 -3.287764e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.530793e-16 -3.178724e-16 -1.000000e+00
    outer loop
      vertex   -3.797254e+01 -3.287764e+01 3.850000e+00
      vertex   -3.797936e+01 -3.147729e+01 3.850000e+00
      vertex   -3.747815e+01 -3.171866e+01 3.850000e+00
    endloop
  endfacet
  facet normal -5.684770e-17 -3.589222e-16 -1.000000e+00
    outer loop
      vertex   -3.797254e+01 -3.287764e+01 3.850000e+00
      vertex   -3.747815e+01 -3.171866e+01 3.850000e+00
      vertex   -3.720000e+01 -3.300000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -3.465818e-16 -1.000000e+00
    outer loop
      vertex   -3.720000e+01 -3.300000e+01 3.850000e+00
      vertex   -3.747815e+01 -3.171866e+01 3.850000e+00
      vertex   -3.692185e+01 -3.171866e+01 3.850000e+00
    endloop
  endfacet
  facet normal 5.684770e-17 -3.589222e-16 -1.000000e+00
    outer loop
      vertex   -3.720000e+01 -3.300000e+01 3.850000e+00
      vertex   -3.692185e+01 -3.171866e+01 3.850000e+00
      vertex   -3.642745e+01 -3.287764e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.530793e-16 -3.178724e-16 -1.000000e+00
    outer loop
      vertex   -3.642745e+01 -3.287764e+01 3.850000e+00
      vertex   -3.692185e+01 -3.171866e+01 3.850000e+00
      vertex   -3.642063e+01 -3.147729e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.768153e-16 -3.148047e-16 -1.000000e+00
    outer loop
      vertex   -3.642745e+01 -3.287764e+01 3.850000e+00
      vertex   -3.642063e+01 -3.147729e+01 3.850000e+00
      vertex   -3.573053e+01 -3.252254e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -3.573053e+01 -3.252254e+01 3.850000e+00
      vertex   -3.642063e+01 -3.147729e+01 3.850000e+00
      vertex   -3.517745e+01 -3.196946e+01 3.850000e+00
    endloop
  endfacet
  facet normal -3.072327e-16 -7.760382e-16 -1.000000e+00
    outer loop
      vertex   -3.517745e+01 -3.196946e+01 3.850000e+00
      vertex   -3.642063e+01 -3.147729e+01 3.850000e+00
      vertex   -3.607379e+01 -3.104235e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.244566e-16 -1.653189e-16 -1.000000e+00
    outer loop
      vertex   -3.517745e+01 -3.196946e+01 3.850000e+00
      vertex   -3.607379e+01 -3.104235e+01 3.850000e+00
      vertex   -3.482236e+01 -3.127254e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.405673e-16 -7.773226e-17 -1.000000e+00
    outer loop
      vertex   -3.482236e+01 -3.127254e+01 3.850000e+00
      vertex   -3.607379e+01 -3.104235e+01 3.850000e+00
      vertex   -3.595000e+01 -3.050000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.680711e-15 -1.586033e-16 -1.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.625855e+01 -2.818404e+01 5.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 8.609867e-17 3.270730e-16 -1.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.625855e+01 -2.818404e+01 5.000000e+00
      vertex   -3.687913e+01 -2.802068e+01 5.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 3.468989e-16 -1.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.687913e+01 -2.802068e+01 5.000000e+00
      vertex   -3.752086e+01 -2.802068e+01 5.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 3.468989e-16 -1.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.752086e+01 -2.802068e+01 5.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -8.609867e-17 3.270730e-16 -1.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.752086e+01 -2.802068e+01 5.000000e+00
      vertex   -3.814144e+01 -2.818404e+01 5.000000e+00
    endloop
  endfacet
  facet normal -4.680711e-15 -1.586033e-16 -1.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.814144e+01 -2.818404e+01 5.000000e+00
      vertex   -3.870000e+01 -2.850000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -3.855267e-15 1.193223e-15 1.000000e+00
    outer loop
      vertex   -3.814144e+01 -2.818404e+01 7.000000e+00
      vertex   -3.797936e+01 -2.542729e+01 7.000000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 2.800903e-16 2.233646e-16 1.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.797936e+01 -2.542729e+01 7.000000e+00
      vertex   -3.832621e+01 -2.499235e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.562917e-16 1.726186e-16 1.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.832621e+01 -2.499235e+01 7.000000e+00
      vertex   -3.845000e+01 -2.445000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.526492e-16 9.399358e-16 1.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.542729e+01 7.000000e+00
      vertex   -3.814144e+01 -2.818404e+01 7.000000e+00
      vertex   -3.747815e+01 -2.566866e+01 7.000000e+00
    endloop
  endfacet
  facet normal -2.996498e-16 1.138314e-15 1.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.566866e+01 7.000000e+00
      vertex   -3.814144e+01 -2.818404e+01 7.000000e+00
      vertex   -3.752086e+01 -2.802068e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.132873e-15 1.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.566866e+01 7.000000e+00
      vertex   -3.752086e+01 -2.802068e+01 7.000000e+00
      vertex   -3.687913e+01 -2.802068e+01 7.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.132873e-15 1.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.566866e+01 7.000000e+00
      vertex   -3.687913e+01 -2.802068e+01 7.000000e+00
      vertex   -3.692185e+01 -2.566866e+01 7.000000e+00
    endloop
  endfacet
  facet normal 2.996498e-16 1.138314e-15 1.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.566866e+01 7.000000e+00
      vertex   -3.687913e+01 -2.802068e+01 7.000000e+00
      vertex   -3.625855e+01 -2.818404e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.526492e-16 9.399358e-16 1.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.566866e+01 7.000000e+00
      vertex   -3.625855e+01 -2.818404e+01 7.000000e+00
      vertex   -3.642063e+01 -2.542729e+01 7.000000e+00
    endloop
  endfacet
  facet normal 3.855267e-15 1.193223e-15 1.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.542729e+01 7.000000e+00
      vertex   -3.625855e+01 -2.818404e+01 7.000000e+00
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -2.800903e-16 2.233646e-16 1.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.542729e+01 7.000000e+00
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.607379e+01 -2.499235e+01 7.000000e+00
    endloop
  endfacet
  facet normal -7.562917e-16 1.726186e-16 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.499235e+01 7.000000e+00
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.595000e+01 -2.445000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -9.833404e-16 1.586033e-16 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -2.445000e+01 7.000000e+00
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -2.445000e+01 7.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.607379e+01 -2.390765e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.390765e+01 7.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.642063e+01 -2.347271e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.347271e+01 7.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.692185e+01 -2.323134e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.323134e+01 7.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.323134e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.747815e+01 -2.323134e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.323134e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.797936e+01 -2.347271e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.347271e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.832621e+01 -2.390765e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.390765e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.845000e+01 -2.445000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 9.833404e-16 1.586033e-16 1.000000e+00
    outer loop
      vertex   -3.845000e+01 -2.445000e+01 7.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.870000e+01 -2.850000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.850000e+01 7.000000e+00
      vertex   -3.570000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.570000e+01 -2.850000e+01 5.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.870000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.570000e+01 -2.290000e+01 5.000000e+00
      vertex   -3.870000e+01 -2.290000e+01 7.000000e+00
      vertex   -3.570000e+01 -2.290000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.390765e+01 7.000000e+00
      vertex   -3.607379e+01 -2.390765e+01 1.342000e+01
      vertex   -3.595000e+01 -2.445000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -3.595000e+01 -2.445000e+01 7.000000e+00
      vertex   -3.607379e+01 -2.390765e+01 1.342000e+01
      vertex   -3.595000e+01 -2.445000e+01 1.342000e+01
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.595000e+01 -2.445000e+01 7.000000e+00
      vertex   -3.595000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.607379e+01 -2.499235e+01 7.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.499235e+01 7.000000e+00
      vertex   -3.595000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.607379e+01 -2.499235e+01 1.342000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.499235e+01 7.000000e+00
      vertex   -3.607379e+01 -2.499235e+01 1.342000e+01
      vertex   -3.642063e+01 -2.542729e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.542729e+01 7.000000e+00
      vertex   -3.607379e+01 -2.499235e+01 1.342000e+01
      vertex   -3.642063e+01 -2.542729e+01 1.342000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.542729e+01 7.000000e+00
      vertex   -3.642063e+01 -2.542729e+01 1.342000e+01
      vertex   -3.692185e+01 -2.566866e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.566866e+01 7.000000e+00
      vertex   -3.642063e+01 -2.542729e+01 1.342000e+01
      vertex   -3.692185e+01 -2.566866e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.566866e+01 7.000000e+00
      vertex   -3.692185e+01 -2.566866e+01 1.342000e+01
      vertex   -3.747815e+01 -2.566866e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.566866e+01 7.000000e+00
      vertex   -3.692185e+01 -2.566866e+01 1.342000e+01
      vertex   -3.747815e+01 -2.566866e+01 1.342000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.566866e+01 7.000000e+00
      vertex   -3.747815e+01 -2.566866e+01 1.342000e+01
      vertex   -3.797936e+01 -2.542729e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.542729e+01 7.000000e+00
      vertex   -3.747815e+01 -2.566866e+01 1.342000e+01
      vertex   -3.797936e+01 -2.542729e+01 1.342000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.542729e+01 7.000000e+00
      vertex   -3.797936e+01 -2.542729e+01 1.342000e+01
      vertex   -3.832621e+01 -2.499235e+01 7.000000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.499235e+01 7.000000e+00
      vertex   -3.797936e+01 -2.542729e+01 1.342000e+01
      vertex   -3.832621e+01 -2.499235e+01 1.342000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.499235e+01 7.000000e+00
      vertex   -3.832621e+01 -2.499235e+01 1.342000e+01
      vertex   -3.845000e+01 -2.445000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.845000e+01 -2.445000e+01 7.000000e+00
      vertex   -3.832621e+01 -2.499235e+01 1.342000e+01
      vertex   -3.845000e+01 -2.445000e+01 1.342000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.845000e+01 -2.445000e+01 7.000000e+00
      vertex   -3.845000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.832621e+01 -2.390765e+01 7.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.390765e+01 7.000000e+00
      vertex   -3.845000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.832621e+01 -2.390765e+01 1.342000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.390765e+01 7.000000e+00
      vertex   -3.832621e+01 -2.390765e+01 1.342000e+01
      vertex   -3.797936e+01 -2.347271e+01 7.000000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.347271e+01 7.000000e+00
      vertex   -3.832621e+01 -2.390765e+01 1.342000e+01
      vertex   -3.797936e+01 -2.347271e+01 1.342000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.347271e+01 7.000000e+00
      vertex   -3.797936e+01 -2.347271e+01 1.342000e+01
      vertex   -3.747815e+01 -2.323134e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.323134e+01 7.000000e+00
      vertex   -3.797936e+01 -2.347271e+01 1.342000e+01
      vertex   -3.747815e+01 -2.323134e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.323134e+01 7.000000e+00
      vertex   -3.747815e+01 -2.323134e+01 1.342000e+01
      vertex   -3.692185e+01 -2.323134e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.323134e+01 7.000000e+00
      vertex   -3.747815e+01 -2.323134e+01 1.342000e+01
      vertex   -3.692185e+01 -2.323134e+01 1.342000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.323134e+01 7.000000e+00
      vertex   -3.692185e+01 -2.323134e+01 1.342000e+01
      vertex   -3.642063e+01 -2.347271e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.347271e+01 7.000000e+00
      vertex   -3.692185e+01 -2.323134e+01 1.342000e+01
      vertex   -3.642063e+01 -2.347271e+01 1.342000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.347271e+01 7.000000e+00
      vertex   -3.642063e+01 -2.347271e+01 1.342000e+01
      vertex   -3.607379e+01 -2.390765e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.390765e+01 7.000000e+00
      vertex   -3.642063e+01 -2.347271e+01 1.342000e+01
      vertex   -3.607379e+01 -2.390765e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.390765e+01 1.342000e+01
      vertex   -3.832621e+01 -2.390765e+01 1.342000e+01
      vertex   -3.595000e+01 -2.445000e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.832621e+01 -2.390765e+01 1.342000e+01
      vertex   -3.845000e+01 -2.445000e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.595000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.845000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.607379e+01 -2.499235e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.499235e+01 1.342000e+01
      vertex   -3.845000e+01 -2.445000e+01 1.342000e+01
      vertex   -3.832621e+01 -2.499235e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.607379e+01 -2.499235e+01 1.342000e+01
      vertex   -3.832621e+01 -2.499235e+01 1.342000e+01
      vertex   -3.642063e+01 -2.542729e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.542729e+01 1.342000e+01
      vertex   -3.832621e+01 -2.499235e+01 1.342000e+01
      vertex   -3.797936e+01 -2.542729e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.642063e+01 -2.542729e+01 1.342000e+01
      vertex   -3.797936e+01 -2.542729e+01 1.342000e+01
      vertex   -3.692185e+01 -2.566866e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.692185e+01 -2.566866e+01 1.342000e+01
      vertex   -3.797936e+01 -2.542729e+01 1.342000e+01
      vertex   -3.747815e+01 -2.566866e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.832621e+01 -2.390765e+01 1.342000e+01
      vertex   -3.607379e+01 -2.390765e+01 1.342000e+01
      vertex   -3.797936e+01 -2.347271e+01 1.342000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.347271e+01 1.342000e+01
      vertex   -3.607379e+01 -2.390765e+01 1.342000e+01
      vertex   -3.642063e+01 -2.347271e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.797936e+01 -2.347271e+01 1.342000e+01
      vertex   -3.642063e+01 -2.347271e+01 1.342000e+01
      vertex   -3.747815e+01 -2.323134e+01 1.342000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.747815e+01 -2.323134e+01 1.342000e+01
      vertex   -3.642063e+01 -2.347271e+01 1.342000e+01
      vertex   -3.692185e+01 -2.323134e+01 1.342000e+01
    endloop
  endfacet
  facet normal 8.047901e-17 -9.805469e-01 -1.962848e-01
    outer loop
      vertex   -1.892941e+01 -2.655384e+01 6.836321e+00
      vertex   -2.342826e+01 -2.646650e+01 6.400000e+00
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
    endloop
  endfacet
  facet normal -7.823830e-17 -9.805469e-01 1.962848e-01
    outer loop
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
      vertex   -2.342826e+01 -2.646650e+01 6.400000e+00
      vertex   -2.347058e+01 -2.655384e+01 5.963679e+00
    endloop
  endfacet
  facet normal 1.534942e-15 -9.805469e-01 1.962848e-01
    outer loop
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
      vertex   -2.347058e+01 -2.655384e+01 5.963679e+00
      vertex   -1.892941e+01 -2.655384e+01 5.963679e+00
    endloop
  endfacet
  facet normal 1.300200e-15 -8.309742e-01 5.563110e-01
    outer loop
      vertex   -1.892941e+01 -2.655384e+01 5.963679e+00
      vertex   -2.347058e+01 -2.655384e+01 5.963679e+00
      vertex   -2.356785e+01 -2.679797e+01 5.599019e+00
    endloop
  endfacet
  facet normal -3.138570e-16 -8.309742e-01 5.563110e-01
    outer loop
      vertex   -1.892941e+01 -2.655384e+01 5.963679e+00
      vertex   -2.356785e+01 -2.679797e+01 5.599019e+00
      vertex   -1.883214e+01 -2.679797e+01 5.599019e+00
    endloop
  endfacet
  facet normal -4.672329e-16 -5.571386e-01 8.304195e-01
    outer loop
      vertex   -1.883214e+01 -2.679797e+01 5.599019e+00
      vertex   -2.356785e+01 -2.679797e+01 5.599019e+00
      vertex   -2.366150e+01 -2.716296e+01 5.354142e+00
    endloop
  endfacet
  facet normal -5.515737e-16 -5.571386e-01 8.304195e-01
    outer loop
      vertex   -1.883214e+01 -2.679797e+01 5.599019e+00
      vertex   -2.366150e+01 -2.716296e+01 5.354142e+00
      vertex   -1.873849e+01 -2.716296e+01 5.354142e+00
    endloop
  endfacet
  facet normal -3.187845e-16 -1.966207e-01 9.804796e-01
    outer loop
      vertex   -1.873849e+01 -2.716296e+01 5.354142e+00
      vertex   -2.366150e+01 -2.716296e+01 5.354142e+00
      vertex   -1.870000e+01 -2.760000e+01 5.266500e+00
    endloop
  endfacet
  facet normal -1.741682e-16 -1.966207e-01 9.804796e-01
    outer loop
      vertex   -1.870000e+01 -2.760000e+01 5.266500e+00
      vertex   -2.366150e+01 -2.716296e+01 5.354142e+00
      vertex   -2.370000e+01 -2.760000e+01 5.266500e+00
    endloop
  endfacet
  facet normal -1.741682e-16 1.966207e-01 9.804796e-01
    outer loop
      vertex   -1.870000e+01 -2.760000e+01 5.266500e+00
      vertex   -2.370000e+01 -2.760000e+01 5.266500e+00
      vertex   -1.873849e+01 -2.803704e+01 5.354142e+00
    endloop
  endfacet
  facet normal -9.194601e-16 1.966207e-01 9.804796e-01
    outer loop
      vertex   -1.873849e+01 -2.803704e+01 5.354142e+00
      vertex   -2.370000e+01 -2.760000e+01 5.266500e+00
      vertex   -2.366150e+01 -2.803704e+01 5.354142e+00
    endloop
  endfacet
  facet normal -4.968529e-16 5.571386e-01 8.304195e-01
    outer loop
      vertex   -1.873849e+01 -2.803704e+01 5.354142e+00
      vertex   -2.366150e+01 -2.803704e+01 5.354142e+00
      vertex   -2.356785e+01 -2.840203e+01 5.599019e+00
    endloop
  endfacet
  facet normal -7.314154e-16 -9.805469e-01 -1.962848e-01
    outer loop
      vertex   -2.342826e+01 -2.646650e+01 6.400000e+00
      vertex   -1.892941e+01 -2.655384e+01 6.836321e+00
      vertex   -2.347058e+01 -2.655384e+01 6.836321e+00
    endloop
  endfacet
  facet normal -5.412945e-16 -8.309742e-01 -5.563110e-01
    outer loop
      vertex   -2.347058e+01 -2.655384e+01 6.836321e+00
      vertex   -1.892941e+01 -2.655384e+01 6.836321e+00
      vertex   -1.883214e+01 -2.679797e+01 7.200981e+00
    endloop
  endfacet
  facet normal 6.210362e-16 -8.309742e-01 -5.563110e-01
    outer loop
      vertex   -2.347058e+01 -2.655384e+01 6.836321e+00
      vertex   -1.883214e+01 -2.679797e+01 7.200981e+00
      vertex   -2.356785e+01 -2.679797e+01 7.200981e+00
    endloop
  endfacet
  facet normal 4.179631e-16 -5.571386e-01 -8.304195e-01
    outer loop
      vertex   -2.356785e+01 -2.679797e+01 7.200981e+00
      vertex   -1.883214e+01 -2.679797e+01 7.200981e+00
      vertex   -1.873849e+01 -2.716296e+01 7.445858e+00
    endloop
  endfacet
  facet normal 6.028829e-16 -5.571386e-01 -8.304195e-01
    outer loop
      vertex   -2.356785e+01 -2.679797e+01 7.200981e+00
      vertex   -1.873849e+01 -2.716296e+01 7.445858e+00
      vertex   -2.366150e+01 -2.716296e+01 7.445858e+00
    endloop
  endfacet
  facet normal 7.075683e-16 -1.966207e-01 -9.804796e-01
    outer loop
      vertex   -2.366150e+01 -2.716296e+01 7.445858e+00
      vertex   -1.873849e+01 -2.716296e+01 7.445858e+00
      vertex   -2.370000e+01 -2.760000e+01 7.533500e+00
    endloop
  endfacet
  facet normal -2.449541e-16 -1.966207e-01 -9.804796e-01
    outer loop
      vertex   -2.370000e+01 -2.760000e+01 7.533500e+00
      vertex   -1.873849e+01 -2.716296e+01 7.445858e+00
      vertex   -1.870000e+01 -2.760000e+01 7.533500e+00
    endloop
  endfacet
  facet normal 5.932904e-16 1.966207e-01 -9.804796e-01
    outer loop
      vertex   -2.370000e+01 -2.760000e+01 7.533500e+00
      vertex   -1.870000e+01 -2.760000e+01 7.533500e+00
      vertex   -2.366150e+01 -2.803704e+01 7.445858e+00
    endloop
  endfacet
  facet normal 3.537842e-16 1.966207e-01 -9.804796e-01
    outer loop
      vertex   -2.366150e+01 -2.803704e+01 7.445858e+00
      vertex   -1.870000e+01 -2.760000e+01 7.533500e+00
      vertex   -1.873849e+01 -2.803704e+01 7.445858e+00
    endloop
  endfacet
  facet normal 2.996383e-16 5.571386e-01 -8.304195e-01
    outer loop
      vertex   -2.366150e+01 -2.803704e+01 7.445858e+00
      vertex   -1.873849e+01 -2.803704e+01 7.445858e+00
      vertex   -1.883214e+01 -2.840203e+01 7.200981e+00
    endloop
  endfacet
  facet normal 2.066863e-16 5.571386e-01 -8.304195e-01
    outer loop
      vertex   -2.366150e+01 -2.803704e+01 7.445858e+00
      vertex   -1.883214e+01 -2.840203e+01 7.200981e+00
      vertex   -2.356785e+01 -2.840203e+01 7.200981e+00
    endloop
  endfacet
  facet normal -2.060512e-16 8.309742e-01 -5.563110e-01
    outer loop
      vertex   -2.356785e+01 -2.840203e+01 7.200981e+00
      vertex   -1.883214e+01 -2.840203e+01 7.200981e+00
      vertex   -1.892941e+01 -2.864616e+01 6.836321e+00
    endloop
  endfacet
  facet normal 1.295283e-15 8.309742e-01 -5.563110e-01
    outer loop
      vertex   -2.356785e+01 -2.840203e+01 7.200981e+00
      vertex   -1.892941e+01 -2.864616e+01 6.836321e+00
      vertex   -2.347058e+01 -2.864616e+01 6.836321e+00
    endloop
  endfacet
  facet normal 1.534232e-15 9.805469e-01 -1.962848e-01
    outer loop
      vertex   -2.347058e+01 -2.864616e+01 6.836321e+00
      vertex   -1.892941e+01 -2.864616e+01 6.836321e+00
      vertex   -1.897173e+01 -2.873350e+01 6.400000e+00
    endloop
  endfacet
  facet normal 3.848996e-17 9.805469e-01 -1.962848e-01
    outer loop
      vertex   -2.347058e+01 -2.864616e+01 6.836321e+00
      vertex   -1.897173e+01 -2.873350e+01 6.400000e+00
      vertex   -2.342826e+01 -2.873350e+01 6.400000e+00
    endloop
  endfacet
  facet normal -3.911915e-17 9.805469e-01 1.962848e-01
    outer loop
      vertex   -2.342826e+01 -2.873350e+01 6.400000e+00
      vertex   -1.897173e+01 -2.873350e+01 6.400000e+00
      vertex   -1.892941e+01 -2.864616e+01 5.963679e+00
    endloop
  endfacet
  facet normal -3.777263e-17 9.805469e-01 1.962848e-01
    outer loop
      vertex   -2.342826e+01 -2.873350e+01 6.400000e+00
      vertex   -1.892941e+01 -2.864616e+01 5.963679e+00
      vertex   -2.347058e+01 -2.864616e+01 5.963679e+00
    endloop
  endfacet
  facet normal -1.088053e-16 8.309742e-01 5.563110e-01
    outer loop
      vertex   -2.347058e+01 -2.864616e+01 5.963679e+00
      vertex   -1.892941e+01 -2.864616e+01 5.963679e+00
      vertex   -1.883214e+01 -2.840203e+01 5.599019e+00
    endloop
  endfacet
  facet normal -1.001671e-16 8.309742e-01 5.563110e-01
    outer loop
      vertex   -2.347058e+01 -2.864616e+01 5.963679e+00
      vertex   -1.883214e+01 -2.840203e+01 5.599019e+00
      vertex   -2.356785e+01 -2.840203e+01 5.599019e+00
    endloop
  endfacet
  facet normal -1.557443e-16 5.571386e-01 8.304195e-01
    outer loop
      vertex   -2.356785e+01 -2.840203e+01 5.599019e+00
      vertex   -1.883214e+01 -2.840203e+01 5.599019e+00
      vertex   -1.873849e+01 -2.803704e+01 5.354142e+00
    endloop
  endfacet
  facet normal 2.817302e-01 -9.594850e-01 -4.083691e-03
    outer loop
      vertex   -2.079452e+01 -3.463094e+01 7.850000e+00
      vertex   -2.088844e+01 -3.468222e+01 1.342000e+01
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal 1.423137e-01 -9.898132e-01 4.083691e-03
    outer loop
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.088844e+01 -3.468222e+01 1.342000e+01
      vertex   -2.130673e+01 -3.474236e+01 1.342000e+01
    endloop
  endfacet
  facet normal -2.817302e-01 -9.594850e-01 -4.083691e-03
    outer loop
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.130673e+01 -3.474236e+01 1.342000e+01
      vertex   -2.160548e+01 -3.463094e+01 7.850000e+00
    endloop
  endfacet
  facet normal -4.154115e-01 -9.096244e-01 4.083691e-03
    outer loop
      vertex   -2.160548e+01 -3.463094e+01 7.850000e+00
      vertex   -2.130673e+01 -3.474236e+01 1.342000e+01
      vertex   -2.169114e+01 -3.456681e+01 1.342000e+01
    endloop
  endfacet
  facet normal -7.557433e-01 -6.548553e-01 -4.083691e-03
    outer loop
      vertex   -2.160548e+01 -3.463094e+01 7.850000e+00
      vertex   -2.169114e+01 -3.456681e+01 1.342000e+01
      vertex   -2.188222e+01 -3.431156e+01 7.850000e+00
    endloop
  endfacet
  facet normal -8.412465e-01 -5.406363e-01 4.083691e-03
    outer loop
      vertex   -2.188222e+01 -3.431156e+01 7.850000e+00
      vertex   -2.169114e+01 -3.456681e+01 1.342000e+01
      vertex   -2.191962e+01 -3.421130e+01 1.342000e+01
    endloop
  endfacet
  facet normal -9.898132e-01 -1.423137e-01 -4.083691e-03
    outer loop
      vertex   -2.188222e+01 -3.431156e+01 7.850000e+00
      vertex   -2.191962e+01 -3.421130e+01 1.342000e+01
      vertex   -2.194236e+01 -3.389326e+01 7.850000e+00
    endloop
  endfacet
  facet normal -9.999917e-01 0.000000e+00 4.083691e-03
    outer loop
      vertex   -2.194236e+01 -3.389326e+01 7.850000e+00
      vertex   -2.191962e+01 -3.421130e+01 1.342000e+01
      vertex   -2.191962e+01 -3.378870e+01 1.342000e+01
    endloop
  endfacet
  facet normal -9.096244e-01 4.154115e-01 -4.083691e-03
    outer loop
      vertex   -2.194236e+01 -3.389326e+01 7.850000e+00
      vertex   -2.191962e+01 -3.378870e+01 1.342000e+01
      vertex   -2.176681e+01 -3.350885e+01 7.850000e+00
    endloop
  endfacet
  facet normal -8.412465e-01 5.406363e-01 4.083691e-03
    outer loop
      vertex   -2.176681e+01 -3.350885e+01 7.850000e+00
      vertex   -2.191962e+01 -3.378870e+01 1.342000e+01
      vertex   -2.169114e+01 -3.343319e+01 1.342000e+01
    endloop
  endfacet
  facet normal -5.406363e-01 8.412465e-01 -4.083691e-03
    outer loop
      vertex   -2.176681e+01 -3.350885e+01 7.850000e+00
      vertex   -2.169114e+01 -3.343319e+01 1.342000e+01
      vertex   -2.141130e+01 -3.328038e+01 7.850000e+00
    endloop
  endfacet
  facet normal -4.154115e-01 9.096244e-01 4.083691e-03
    outer loop
      vertex   -2.141130e+01 -3.328038e+01 7.850000e+00
      vertex   -2.169114e+01 -3.343319e+01 1.342000e+01
      vertex   -2.130673e+01 -3.325763e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.999917e-01 -4.083691e-03
    outer loop
      vertex   -2.141130e+01 -3.328038e+01 7.850000e+00
      vertex   -2.130673e+01 -3.325763e+01 1.342000e+01
      vertex   -2.098870e+01 -3.328038e+01 7.850000e+00
    endloop
  endfacet
  facet normal 1.423137e-01 9.898132e-01 4.083691e-03
    outer loop
      vertex   -2.098870e+01 -3.328038e+01 7.850000e+00
      vertex   -2.130673e+01 -3.325763e+01 1.342000e+01
      vertex   -2.088844e+01 -3.331777e+01 1.342000e+01
    endloop
  endfacet
  facet normal 5.406363e-01 8.412465e-01 -4.083691e-03
    outer loop
      vertex   -2.098870e+01 -3.328038e+01 7.850000e+00
      vertex   -2.088844e+01 -3.331777e+01 1.342000e+01
      vertex   -2.063318e+01 -3.350885e+01 7.850000e+00
    endloop
  endfacet
  facet normal 6.548553e-01 7.557433e-01 4.083691e-03
    outer loop
      vertex   -2.063318e+01 -3.350885e+01 7.850000e+00
      vertex   -2.088844e+01 -3.331777e+01 1.342000e+01
      vertex   -2.056906e+01 -3.359452e+01 1.342000e+01
    endloop
  endfacet
  facet normal 9.096244e-01 4.154115e-01 -4.083691e-03
    outer loop
      vertex   -2.063318e+01 -3.350885e+01 7.850000e+00
      vertex   -2.056906e+01 -3.359452e+01 1.342000e+01
      vertex   -2.045763e+01 -3.389326e+01 7.850000e+00
    endloop
  endfacet
  facet normal 9.594850e-01 2.817302e-01 4.083691e-03
    outer loop
      vertex   -2.045763e+01 -3.389326e+01 7.850000e+00
      vertex   -2.056906e+01 -3.359452e+01 1.342000e+01
      vertex   -2.045000e+01 -3.400000e+01 1.342000e+01
    endloop
  endfacet
  facet normal 9.898132e-01 -1.423137e-01 -4.083691e-03
    outer loop
      vertex   -2.045763e+01 -3.389326e+01 7.850000e+00
      vertex   -2.045000e+01 -3.400000e+01 1.342000e+01
      vertex   -2.051777e+01 -3.431156e+01 7.850000e+00
    endloop
  endfacet
  facet normal 9.594850e-01 -2.817302e-01 4.083691e-03
    outer loop
      vertex   -2.051777e+01 -3.431156e+01 7.850000e+00
      vertex   -2.045000e+01 -3.400000e+01 1.342000e+01
      vertex   -2.056906e+01 -3.440548e+01 1.342000e+01
    endloop
  endfacet
  facet normal 7.557433e-01 -6.548553e-01 -4.083691e-03
    outer loop
      vertex   -2.051777e+01 -3.431156e+01 7.850000e+00
      vertex   -2.056906e+01 -3.440548e+01 1.342000e+01
      vertex   -2.079452e+01 -3.463094e+01 7.850000e+00
    endloop
  endfacet
  facet normal 6.548553e-01 -7.557433e-01 4.083691e-03
    outer loop
      vertex   -2.079452e+01 -3.463094e+01 7.850000e+00
      vertex   -2.056906e+01 -3.440548e+01 1.342000e+01
      vertex   -2.088844e+01 -3.468222e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.045000e+01 -3.400000e+01 1.342000e+01
      vertex   -2.056906e+01 -3.359452e+01 1.342000e+01
      vertex   -2.056906e+01 -3.440548e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.056906e+01 -3.440548e+01 1.342000e+01
      vertex   -2.056906e+01 -3.359452e+01 1.342000e+01
      vertex   -2.088844e+01 -3.331777e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.056906e+01 -3.440548e+01 1.342000e+01
      vertex   -2.088844e+01 -3.331777e+01 1.342000e+01
      vertex   -2.088844e+01 -3.468222e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.088844e+01 -3.468222e+01 1.342000e+01
      vertex   -2.088844e+01 -3.331777e+01 1.342000e+01
      vertex   -2.130673e+01 -3.325763e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.088844e+01 -3.468222e+01 1.342000e+01
      vertex   -2.130673e+01 -3.325763e+01 1.342000e+01
      vertex   -2.130673e+01 -3.474236e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.130673e+01 -3.474236e+01 1.342000e+01
      vertex   -2.130673e+01 -3.325763e+01 1.342000e+01
      vertex   -2.169114e+01 -3.343319e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.130673e+01 -3.474236e+01 1.342000e+01
      vertex   -2.169114e+01 -3.343319e+01 1.342000e+01
      vertex   -2.169114e+01 -3.456681e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.169114e+01 -3.456681e+01 1.342000e+01
      vertex   -2.169114e+01 -3.343319e+01 1.342000e+01
      vertex   -2.191962e+01 -3.378870e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.169114e+01 -3.456681e+01 1.342000e+01
      vertex   -2.191962e+01 -3.378870e+01 1.342000e+01
      vertex   -2.191962e+01 -3.421130e+01 1.342000e+01
    endloop
  endfacet
  facet normal -5.515256e-15 1.316695e-15 1.000000e+00
    outer loop
      vertex   -2.025855e+01 -2.991596e+01 7.850000e+00
      vertex   -2.063318e+01 -3.350885e+01 7.850000e+00
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
      vertex   -2.063318e+01 -3.350885e+01 7.850000e+00
      vertex   -2.045763e+01 -3.389326e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
      vertex   -2.045763e+01 -3.389326e+01 7.850000e+00
      vertex   -1.970000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.970000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.045763e+01 -3.389326e+01 7.850000e+00
      vertex   -2.051777e+01 -3.431156e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.970000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.051777e+01 -3.431156e+01 7.850000e+00
      vertex   -2.079452e+01 -3.463094e+01 7.850000e+00
    endloop
  endfacet
  facet normal -2.007318e-16 7.625433e-16 1.000000e+00
    outer loop
      vertex   -2.025855e+01 -2.991596e+01 7.850000e+00
      vertex   -2.087913e+01 -3.007932e+01 7.850000e+00
      vertex   -2.063318e+01 -3.350885e+01 7.850000e+00
    endloop
  endfacet
  facet normal 5.234324e-16 8.144768e-16 1.000000e+00
    outer loop
      vertex   -2.063318e+01 -3.350885e+01 7.850000e+00
      vertex   -2.087913e+01 -3.007932e+01 7.850000e+00
      vertex   -2.098870e+01 -3.328038e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 8.323925e-16 1.000000e+00
    outer loop
      vertex   -2.098870e+01 -3.328038e+01 7.850000e+00
      vertex   -2.087913e+01 -3.007932e+01 7.850000e+00
      vertex   -2.152086e+01 -3.007932e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 8.323925e-16 1.000000e+00
    outer loop
      vertex   -2.098870e+01 -3.328038e+01 7.850000e+00
      vertex   -2.152086e+01 -3.007932e+01 7.850000e+00
      vertex   -2.141130e+01 -3.328038e+01 7.850000e+00
    endloop
  endfacet
  facet normal -5.234324e-16 8.144768e-16 1.000000e+00
    outer loop
      vertex   -2.141130e+01 -3.328038e+01 7.850000e+00
      vertex   -2.152086e+01 -3.007932e+01 7.850000e+00
      vertex   -2.176681e+01 -3.350885e+01 7.850000e+00
    endloop
  endfacet
  facet normal 2.007318e-16 7.625433e-16 1.000000e+00
    outer loop
      vertex   -2.176681e+01 -3.350885e+01 7.850000e+00
      vertex   -2.152086e+01 -3.007932e+01 7.850000e+00
      vertex   -2.214144e+01 -2.991596e+01 7.850000e+00
    endloop
  endfacet
  facet normal 5.515256e-15 1.316695e-15 1.000000e+00
    outer loop
      vertex   -2.176681e+01 -3.350885e+01 7.850000e+00
      vertex   -2.214144e+01 -2.991596e+01 7.850000e+00
      vertex   -2.270000e+01 -2.960000e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.176681e+01 -3.350885e+01 7.850000e+00
      vertex   -2.270000e+01 -2.960000e+01 7.850000e+00
      vertex   -2.194236e+01 -3.389326e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.194236e+01 -3.389326e+01 7.850000e+00
      vertex   -2.270000e+01 -2.960000e+01 7.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.194236e+01 -3.389326e+01 7.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.188222e+01 -3.431156e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.188222e+01 -3.431156e+01 7.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.160548e+01 -3.463094e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.160548e+01 -3.463094e+01 7.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.079452e+01 -3.463094e+01 7.850000e+00
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
      vertex   -1.970000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal -8.058513e-16 -1.839303e-16 -1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 3.850000e+00
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
    endloop
  endfacet
  facet normal -5.800687e-16 -3.004297e-16 -1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
      vertex   -1.882829e+01 -2.680943e+01 3.850000e+00
    endloop
  endfacet
  facet normal 2.191311e-17 -1.154558e-15 -1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
      vertex   -1.882829e+01 -2.680943e+01 3.850000e+00
      vertex   -1.870011e+01 -2.757628e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.838940e-16 -4.885077e-16 -1.000000e+00
    outer loop
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
      vertex   -2.007379e+01 -2.705764e+01 3.850000e+00
      vertex   -1.973822e+01 -2.557189e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.411373e-15 -9.165637e-16 -1.000000e+00
    outer loop
      vertex   -1.973822e+01 -2.557189e+01 3.850000e+00
      vertex   -2.007379e+01 -2.705764e+01 3.850000e+00
      vertex   -2.042063e+01 -2.662271e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.818209e-16 -3.129007e-16 -1.000000e+00
    outer loop
      vertex   -1.973822e+01 -2.557189e+01 3.850000e+00
      vertex   -2.042063e+01 -2.662271e+01 3.850000e+00
      vertex   -2.043197e+01 -2.522090e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.531573e-16 -3.180344e-16 -1.000000e+00
    outer loop
      vertex   -2.043197e+01 -2.522090e+01 3.850000e+00
      vertex   -2.042063e+01 -2.662271e+01 3.850000e+00
      vertex   -2.092185e+01 -2.638134e+01 3.850000e+00
    endloop
  endfacet
  facet normal -5.648627e-17 -3.588438e-16 -1.000000e+00
    outer loop
      vertex   -2.043197e+01 -2.522090e+01 3.850000e+00
      vertex   -2.092185e+01 -2.638134e+01 3.850000e+00
      vertex   -2.120000e+01 -2.510000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -3.465818e-16 -1.000000e+00
    outer loop
      vertex   -2.120000e+01 -2.510000e+01 3.850000e+00
      vertex   -2.092185e+01 -2.638134e+01 3.850000e+00
      vertex   -2.147815e+01 -2.638134e+01 3.850000e+00
    endloop
  endfacet
  facet normal 5.648627e-17 -3.588438e-16 -1.000000e+00
    outer loop
      vertex   -2.120000e+01 -2.510000e+01 3.850000e+00
      vertex   -2.147815e+01 -2.638134e+01 3.850000e+00
      vertex   -2.196803e+01 -2.522090e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.531573e-16 -3.180344e-16 -1.000000e+00
    outer loop
      vertex   -2.196803e+01 -2.522090e+01 3.850000e+00
      vertex   -2.147815e+01 -2.638134e+01 3.850000e+00
      vertex   -2.197936e+01 -2.662271e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.818209e-16 -3.129007e-16 -1.000000e+00
    outer loop
      vertex   -2.196803e+01 -2.522090e+01 3.850000e+00
      vertex   -2.197936e+01 -2.662271e+01 3.850000e+00
      vertex   -2.266177e+01 -2.557189e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.411373e-15 -9.165637e-16 -1.000000e+00
    outer loop
      vertex   -2.266177e+01 -2.557189e+01 3.850000e+00
      vertex   -2.197936e+01 -2.662271e+01 3.850000e+00
      vertex   -2.232621e+01 -2.705764e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.838940e-16 -4.885077e-16 -1.000000e+00
    outer loop
      vertex   -2.266177e+01 -2.557189e+01 3.850000e+00
      vertex   -2.232621e+01 -2.705764e+01 3.850000e+00
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
    endloop
  endfacet
  facet normal 8.058513e-16 -1.839303e-16 -1.000000e+00
    outer loop
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
      vertex   -2.232621e+01 -2.705764e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 5.800687e-16 -3.004297e-16 -1.000000e+00
    outer loop
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.357171e+01 -2.680943e+01 3.850000e+00
    endloop
  endfacet
  facet normal -2.191311e-17 -1.154558e-15 -1.000000e+00
    outer loop
      vertex   -2.357171e+01 -2.680943e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.369988e+01 -2.757628e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.098933e-17 -5.790062e-16 -1.000000e+00
    outer loop
      vertex   -2.369988e+01 -2.757628e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.358628e+01 -2.834542e+01 3.850000e+00
    endloop
  endfacet
  facet normal -2.951577e-16 -1.458325e-16 -1.000000e+00
    outer loop
      vertex   -2.358628e+01 -2.834542e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.324188e+01 -2.904246e+01 3.850000e+00
    endloop
  endfacet
  facet normal -3.961128e-16 -9.041015e-17 -1.000000e+00
    outer loop
      vertex   -2.324188e+01 -2.904246e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.232621e+01 -2.814235e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.849701e-15 1.388311e-15 -1.000000e+00
    outer loop
      vertex   -2.324188e+01 -2.904246e+01 3.850000e+00
      vertex   -2.232621e+01 -2.814235e+01 3.850000e+00
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.836058e-15 4.431595e-16 -1.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.232621e+01 -2.814235e+01 3.850000e+00
      vertex   -2.197936e+01 -2.857729e+01 3.850000e+00
    endloop
  endfacet
  facet normal 6.245282e-16 1.296846e-15 -1.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.197936e+01 -2.857729e+01 3.850000e+00
      vertex   -2.147815e+01 -2.881866e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.508969e-15 -8.623091e-17 -1.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.147815e+01 -2.881866e+01 3.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 2.246149e-16 -1.000000e+00
    outer loop
      vertex   -2.270000e+01 -3.475000e+01 3.850000e+00
      vertex   -2.147815e+01 -2.881866e+01 3.850000e+00
      vertex   -2.092185e+01 -2.881866e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 2.246149e-16 -1.000000e+00
    outer loop
      vertex   -2.270000e+01 -3.475000e+01 3.850000e+00
      vertex   -2.092185e+01 -2.881866e+01 3.850000e+00
      vertex   -1.970000e+01 -3.475000e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.508969e-15 -8.623091e-17 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -3.475000e+01 3.850000e+00
      vertex   -2.092185e+01 -2.881866e+01 3.850000e+00
      vertex   -1.970000e+01 -2.960000e+01 3.850000e+00
    endloop
  endfacet
  facet normal -6.245282e-16 1.296846e-15 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.092185e+01 -2.881866e+01 3.850000e+00
      vertex   -2.042063e+01 -2.857729e+01 3.850000e+00
    endloop
  endfacet
  facet normal -1.836058e-15 4.431595e-16 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.042063e+01 -2.857729e+01 3.850000e+00
      vertex   -2.007379e+01 -2.814235e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.849701e-15 1.388311e-15 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.007379e+01 -2.814235e+01 3.850000e+00
      vertex   -1.915811e+01 -2.904246e+01 3.850000e+00
    endloop
  endfacet
  facet normal 3.961128e-16 -9.041015e-17 -1.000000e+00
    outer loop
      vertex   -1.915811e+01 -2.904246e+01 3.850000e+00
      vertex   -2.007379e+01 -2.814235e+01 3.850000e+00
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 2.951577e-16 -1.458325e-16 -1.000000e+00
    outer loop
      vertex   -1.915811e+01 -2.904246e+01 3.850000e+00
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
      vertex   -1.881371e+01 -2.834542e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.098933e-17 -5.790062e-16 -1.000000e+00
    outer loop
      vertex   -1.881371e+01 -2.834542e+01 3.850000e+00
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
      vertex   -1.870011e+01 -2.757628e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 3.850000e+00
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
      vertex   -1.970000e+01 -3.475000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.970000e+01 -3.475000e+01 3.850000e+00
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
      vertex   -1.970000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.970000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
      vertex   -1.970000e+01 -3.475000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.970000e+01 -3.475000e+01 3.850000e+00
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.270000e+01 -3.475000e+01 3.850000e+00
      vertex   -2.120000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.960000e+01 7.850000e+00
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 7.850000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.270000e+01 -3.475000e+01 7.850000e+00
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.270000e+01 -3.475000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 1.231391e-16
    outer loop
      vertex   -2.007379e+01 -2.209235e+01 1.342000e+01
      vertex   -2.007379e+01 -2.209235e+01 7.000000e+00
      vertex   -1.995000e+01 -2.155000e+01 1.342000e+01
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 1.231391e-16
    outer loop
      vertex   -1.995000e+01 -2.155000e+01 1.342000e+01
      vertex   -2.007379e+01 -2.209235e+01 7.000000e+00
      vertex   -1.995000e+01 -2.155000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -1.231391e-16
    outer loop
      vertex   -1.995000e+01 -2.155000e+01 1.342000e+01
      vertex   -1.995000e+01 -2.155000e+01 7.000000e+00
      vertex   -2.007379e+01 -2.100764e+01 1.342000e+01
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -2.462782e-16
    outer loop
      vertex   -2.007379e+01 -2.100764e+01 1.342000e+01
      vertex   -1.995000e+01 -2.155000e+01 7.000000e+00
      vertex   -2.007379e+01 -2.100764e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -6.900563e-16
    outer loop
      vertex   -2.007379e+01 -2.100764e+01 1.342000e+01
      vertex   -2.007379e+01 -2.100764e+01 7.000000e+00
      vertex   -2.042063e+01 -2.057271e+01 1.342000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -6.900563e-16
    outer loop
      vertex   -2.042063e+01 -2.057271e+01 1.342000e+01
      vertex   -2.007379e+01 -2.100764e+01 7.000000e+00
      vertex   -2.042063e+01 -2.057271e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -9.971603e-16
    outer loop
      vertex   -2.042063e+01 -2.057271e+01 1.342000e+01
      vertex   -2.042063e+01 -2.057271e+01 7.000000e+00
      vertex   -2.092185e+01 -2.033134e+01 1.342000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -4.985801e-16
    outer loop
      vertex   -2.092185e+01 -2.033134e+01 1.342000e+01
      vertex   -2.042063e+01 -2.057271e+01 7.000000e+00
      vertex   -2.092185e+01 -2.033134e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -5.533822e-16
    outer loop
      vertex   -2.092185e+01 -2.033134e+01 1.342000e+01
      vertex   -2.092185e+01 -2.033134e+01 7.000000e+00
      vertex   -2.147815e+01 -2.033134e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -5.533822e-16
    outer loop
      vertex   -2.147815e+01 -2.033134e+01 1.342000e+01
      vertex   -2.092185e+01 -2.033134e+01 7.000000e+00
      vertex   -2.147815e+01 -2.033134e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 -4.985801e-16
    outer loop
      vertex   -2.147815e+01 -2.033134e+01 1.342000e+01
      vertex   -2.147815e+01 -2.033134e+01 7.000000e+00
      vertex   -2.197936e+01 -2.057271e+01 1.342000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 -9.971603e-16
    outer loop
      vertex   -2.197936e+01 -2.057271e+01 1.342000e+01
      vertex   -2.147815e+01 -2.033134e+01 7.000000e+00
      vertex   -2.197936e+01 -2.057271e+01 7.000000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 -6.900563e-16
    outer loop
      vertex   -2.197936e+01 -2.057271e+01 1.342000e+01
      vertex   -2.197936e+01 -2.057271e+01 7.000000e+00
      vertex   -2.232621e+01 -2.100764e+01 1.342000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 -6.900563e-16
    outer loop
      vertex   -2.232621e+01 -2.100764e+01 1.342000e+01
      vertex   -2.197936e+01 -2.057271e+01 7.000000e+00
      vertex   -2.232621e+01 -2.100764e+01 7.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 -2.462782e-16
    outer loop
      vertex   -2.232621e+01 -2.100764e+01 1.342000e+01
      vertex   -2.232621e+01 -2.100764e+01 7.000000e+00
      vertex   -2.245000e+01 -2.155000e+01 1.342000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 -1.231391e-16
    outer loop
      vertex   -2.245000e+01 -2.155000e+01 1.342000e+01
      vertex   -2.232621e+01 -2.100764e+01 7.000000e+00
      vertex   -2.245000e+01 -2.155000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 1.231391e-16
    outer loop
      vertex   -2.245000e+01 -2.155000e+01 1.342000e+01
      vertex   -2.245000e+01 -2.155000e+01 7.000000e+00
      vertex   -2.232621e+01 -2.209235e+01 1.342000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 1.231391e-16
    outer loop
      vertex   -2.232621e+01 -2.209235e+01 1.342000e+01
      vertex   -2.245000e+01 -2.155000e+01 7.000000e+00
      vertex   -2.232621e+01 -2.209235e+01 7.000000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 3.450282e-16
    outer loop
      vertex   -2.232621e+01 -2.209235e+01 1.342000e+01
      vertex   -2.232621e+01 -2.209235e+01 7.000000e+00
      vertex   -2.197936e+01 -2.252729e+01 1.342000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 3.450282e-16
    outer loop
      vertex   -2.197936e+01 -2.252729e+01 1.342000e+01
      vertex   -2.232621e+01 -2.209235e+01 7.000000e+00
      vertex   -2.197936e+01 -2.252729e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 4.985801e-16
    outer loop
      vertex   -2.197936e+01 -2.252729e+01 1.342000e+01
      vertex   -2.197936e+01 -2.252729e+01 7.000000e+00
      vertex   -2.147815e+01 -2.276866e+01 1.342000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 4.985801e-16
    outer loop
      vertex   -2.147815e+01 -2.276866e+01 1.342000e+01
      vertex   -2.197936e+01 -2.252729e+01 7.000000e+00
      vertex   -2.147815e+01 -2.276866e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 5.533822e-16
    outer loop
      vertex   -2.147815e+01 -2.276866e+01 1.342000e+01
      vertex   -2.147815e+01 -2.276866e+01 7.000000e+00
      vertex   -2.092185e+01 -2.276866e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 5.533822e-16
    outer loop
      vertex   -2.092185e+01 -2.276866e+01 1.342000e+01
      vertex   -2.147815e+01 -2.276866e+01 7.000000e+00
      vertex   -2.092185e+01 -2.276866e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 4.985801e-16
    outer loop
      vertex   -2.092185e+01 -2.276866e+01 1.342000e+01
      vertex   -2.092185e+01 -2.276866e+01 7.000000e+00
      vertex   -2.042063e+01 -2.252729e+01 1.342000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 4.985801e-16
    outer loop
      vertex   -2.042063e+01 -2.252729e+01 1.342000e+01
      vertex   -2.092185e+01 -2.276866e+01 7.000000e+00
      vertex   -2.042063e+01 -2.252729e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 3.450282e-16
    outer loop
      vertex   -2.042063e+01 -2.252729e+01 1.342000e+01
      vertex   -2.042063e+01 -2.252729e+01 7.000000e+00
      vertex   -2.007379e+01 -2.209235e+01 1.342000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 3.450282e-16
    outer loop
      vertex   -2.007379e+01 -2.209235e+01 1.342000e+01
      vertex   -2.042063e+01 -2.252729e+01 7.000000e+00
      vertex   -2.007379e+01 -2.209235e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.100764e+01 1.342000e+01
      vertex   -2.232621e+01 -2.100764e+01 1.342000e+01
      vertex   -1.995000e+01 -2.155000e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.155000e+01 1.342000e+01
      vertex   -2.232621e+01 -2.100764e+01 1.342000e+01
      vertex   -2.245000e+01 -2.155000e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.155000e+01 1.342000e+01
      vertex   -2.245000e+01 -2.155000e+01 1.342000e+01
      vertex   -2.007379e+01 -2.209235e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.209235e+01 1.342000e+01
      vertex   -2.245000e+01 -2.155000e+01 1.342000e+01
      vertex   -2.232621e+01 -2.209235e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.209235e+01 1.342000e+01
      vertex   -2.232621e+01 -2.209235e+01 1.342000e+01
      vertex   -2.042063e+01 -2.252729e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.252729e+01 1.342000e+01
      vertex   -2.232621e+01 -2.209235e+01 1.342000e+01
      vertex   -2.197936e+01 -2.252729e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.252729e+01 1.342000e+01
      vertex   -2.197936e+01 -2.252729e+01 1.342000e+01
      vertex   -2.092185e+01 -2.276866e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.276866e+01 1.342000e+01
      vertex   -2.197936e+01 -2.252729e+01 1.342000e+01
      vertex   -2.147815e+01 -2.276866e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.100764e+01 1.342000e+01
      vertex   -2.007379e+01 -2.100764e+01 1.342000e+01
      vertex   -2.197936e+01 -2.057271e+01 1.342000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.057271e+01 1.342000e+01
      vertex   -2.007379e+01 -2.100764e+01 1.342000e+01
      vertex   -2.042063e+01 -2.057271e+01 1.342000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.057271e+01 1.342000e+01
      vertex   -2.042063e+01 -2.057271e+01 1.342000e+01
      vertex   -2.147815e+01 -2.033134e+01 1.342000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.033134e+01 1.342000e+01
      vertex   -2.042063e+01 -2.057271e+01 1.342000e+01
      vertex   -2.092185e+01 -2.033134e+01 1.342000e+01
    endloop
  endfacet
  facet normal 8.496639e-01 5.268205e-01 -2.305363e-02
    outer loop
      vertex   -1.892941e+01 -2.655384e+01 5.963679e+00
      vertex   -1.882829e+01 -2.680943e+01 3.850000e+00
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
    endloop
  endfacet
  facet normal 8.879085e-01 4.598663e-01 -1.189816e-02
    outer loop
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
      vertex   -1.882829e+01 -2.680943e+01 3.850000e+00
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
    endloop
  endfacet
  facet normal 8.520559e-01 5.234508e-01 -2.237840e-04
    outer loop
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
      vertex   -1.917745e+01 -2.613054e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.116989e-01 7.024845e-01 4.107681e-04
    outer loop
      vertex   -1.917745e+01 -2.613054e+01 8.950000e+00
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.071038e-01 7.071038e-01 2.898902e-03
    outer loop
      vertex   -1.917745e+01 -2.613054e+01 8.950000e+00
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
      vertex   -1.973053e+01 -2.557746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.923602e-01 8.703884e-01 -2.351579e-03
    outer loop
      vertex   -1.973053e+01 -2.557746e+01 8.950000e+00
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.025855e+01 -2.528404e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.539623e-01 8.909512e-01 1.113981e-02
    outer loop
      vertex   -1.973053e+01 -2.557746e+01 8.950000e+00
      vertex   -2.025855e+01 -2.528404e+01 7.000000e+00
      vertex   -2.042745e+01 -2.522236e+01 8.950000e+00
    endloop
  endfacet
  facet normal 2.545582e-01 9.670198e-01 -8.538277e-03
    outer loop
      vertex   -2.042745e+01 -2.522236e+01 8.950000e+00
      vertex   -2.025855e+01 -2.528404e+01 7.000000e+00
      vertex   -2.087913e+01 -2.512067e+01 7.000000e+00
    endloop
  endfacet
  facet normal 1.564162e-01 9.875732e-01 1.526616e-02
    outer loop
      vertex   -2.042745e+01 -2.522236e+01 8.950000e+00
      vertex   -2.087913e+01 -2.512067e+01 7.000000e+00
      vertex   -2.120000e+01 -2.510000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 9.999438e-01 -1.060254e-02
    outer loop
      vertex   -2.120000e+01 -2.510000e+01 8.950000e+00
      vertex   -2.087913e+01 -2.512067e+01 7.000000e+00
      vertex   -2.152086e+01 -2.512067e+01 7.000000e+00
    endloop
  endfacet
  facet normal -1.564162e-01 9.875732e-01 1.526616e-02
    outer loop
      vertex   -2.120000e+01 -2.510000e+01 8.950000e+00
      vertex   -2.152086e+01 -2.512067e+01 7.000000e+00
      vertex   -2.197254e+01 -2.522236e+01 8.950000e+00
    endloop
  endfacet
  facet normal -2.545582e-01 9.670198e-01 -8.538277e-03
    outer loop
      vertex   -2.197254e+01 -2.522236e+01 8.950000e+00
      vertex   -2.152086e+01 -2.512067e+01 7.000000e+00
      vertex   -2.214144e+01 -2.528404e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.539623e-01 8.909512e-01 1.113981e-02
    outer loop
      vertex   -2.197254e+01 -2.522236e+01 8.950000e+00
      vertex   -2.214144e+01 -2.528404e+01 7.000000e+00
      vertex   -2.266946e+01 -2.557746e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.923602e-01 8.703884e-01 -2.351579e-03
    outer loop
      vertex   -2.266946e+01 -2.557746e+01 8.950000e+00
      vertex   -2.214144e+01 -2.528404e+01 7.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -7.071038e-01 7.071038e-01 2.898902e-03
    outer loop
      vertex   -2.266946e+01 -2.557746e+01 8.950000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.116989e-01 7.024845e-01 4.107681e-04
    outer loop
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
    endloop
  endfacet
  facet normal -8.520559e-01 5.234508e-01 -2.237840e-04
    outer loop
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
      vertex   -2.342826e+01 -2.646650e+01 6.400000e+00
    endloop
  endfacet
  facet normal -8.879085e-01 4.598663e-01 -1.189816e-02
    outer loop
      vertex   -2.342826e+01 -2.646650e+01 6.400000e+00
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
      vertex   -2.357171e+01 -2.680943e+01 3.850000e+00
    endloop
  endfacet
  facet normal -8.496639e-01 5.268205e-01 -2.305363e-02
    outer loop
      vertex   -2.342826e+01 -2.646650e+01 6.400000e+00
      vertex   -2.357171e+01 -2.680943e+01 3.850000e+00
      vertex   -2.347058e+01 -2.655384e+01 5.963679e+00
    endloop
  endfacet
  facet normal -9.287751e-01 3.706435e-01 -3.836569e-04
    outer loop
      vertex   -2.347058e+01 -2.655384e+01 5.963679e+00
      vertex   -2.357171e+01 -2.680943e+01 3.850000e+00
      vertex   -2.356785e+01 -2.679797e+01 5.599019e+00
    endloop
  endfacet
  facet normal -9.687083e-01 2.482014e-01 5.065253e-04
    outer loop
      vertex   -2.356785e+01 -2.679797e+01 5.599019e+00
      vertex   -2.357171e+01 -2.680943e+01 3.850000e+00
      vertex   -2.366150e+01 -2.716296e+01 5.354142e+00
    endloop
  endfacet
  facet normal -9.861165e-01 1.648303e-01 -2.012813e-02
    outer loop
      vertex   -2.366150e+01 -2.716296e+01 5.354142e+00
      vertex   -2.357171e+01 -2.680943e+01 3.850000e+00
      vertex   -2.369988e+01 -2.757628e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.961662e-01 8.747025e-02 1.385729e-03
    outer loop
      vertex   -2.366150e+01 -2.716296e+01 5.354142e+00
      vertex   -2.369988e+01 -2.757628e+01 3.850000e+00
      vertex   -2.370000e+01 -2.760000e+01 5.266500e+00
    endloop
  endfacet
  facet normal -9.961144e-01 -8.805517e-02 -1.553828e-03
    outer loop
      vertex   -2.370000e+01 -2.760000e+01 5.266500e+00
      vertex   -2.369988e+01 -2.757628e+01 3.850000e+00
      vertex   -2.366150e+01 -2.803704e+01 5.354142e+00
    endloop
  endfacet
  facet normal -9.890791e-01 -1.460888e-01 -1.951069e-02
    outer loop
      vertex   -2.366150e+01 -2.803704e+01 5.354142e+00
      vertex   -2.369988e+01 -2.757628e+01 3.850000e+00
      vertex   -2.358628e+01 -2.834542e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.689802e-01 -2.471284e-01 2.209679e-03
    outer loop
      vertex   -2.366150e+01 -2.803704e+01 5.354142e+00
      vertex   -2.358628e+01 -2.834542e+01 3.850000e+00
      vertex   -2.356785e+01 -2.840203e+01 5.599019e+00
    endloop
  endfacet
  facet normal -9.277828e-01 -3.731137e-01 -2.302102e-03
    outer loop
      vertex   -2.356785e+01 -2.840203e+01 5.599019e+00
      vertex   -2.358628e+01 -2.834542e+01 3.850000e+00
      vertex   -2.347058e+01 -2.864616e+01 5.963679e+00
    endloop
  endfacet
  facet normal -8.964517e-01 -4.429218e-01 -1.394957e-02
    outer loop
      vertex   -2.347058e+01 -2.864616e+01 5.963679e+00
      vertex   -2.358628e+01 -2.834542e+01 3.850000e+00
      vertex   -2.324188e+01 -2.904246e+01 3.850000e+00
    endloop
  endfacet
  facet normal -8.838121e-01 -4.677749e-01 -7.922091e-03
    outer loop
      vertex   -2.347058e+01 -2.864616e+01 5.963679e+00
      vertex   -2.324188e+01 -2.904246e+01 3.850000e+00
      vertex   -2.342826e+01 -2.873350e+01 6.400000e+00
    endloop
  endfacet
  facet normal -8.544671e-01 -5.195053e-01 4.905736e-04
    outer loop
      vertex   -2.342826e+01 -2.873350e+01 6.400000e+00
      vertex   -2.324188e+01 -2.904246e+01 3.850000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
    endloop
  endfacet
  facet normal -8.909416e-01 -4.539574e-01 1.206916e-02
    outer loop
      vertex   -2.342826e+01 -2.873350e+01 6.400000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.287751e-01 3.706435e-01 -3.836569e-04
    outer loop
      vertex   -1.892941e+01 -2.655384e+01 5.963679e+00
      vertex   -1.883214e+01 -2.679797e+01 5.599019e+00
      vertex   -1.882829e+01 -2.680943e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.687083e-01 2.482014e-01 5.065253e-04
    outer loop
      vertex   -1.882829e+01 -2.680943e+01 3.850000e+00
      vertex   -1.883214e+01 -2.679797e+01 5.599019e+00
      vertex   -1.873849e+01 -2.716296e+01 5.354142e+00
    endloop
  endfacet
  facet normal 9.861165e-01 1.648303e-01 -2.012813e-02
    outer loop
      vertex   -1.882829e+01 -2.680943e+01 3.850000e+00
      vertex   -1.873849e+01 -2.716296e+01 5.354142e+00
      vertex   -1.870011e+01 -2.757628e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.961662e-01 8.747025e-02 1.385729e-03
    outer loop
      vertex   -1.870011e+01 -2.757628e+01 3.850000e+00
      vertex   -1.873849e+01 -2.716296e+01 5.354142e+00
      vertex   -1.870000e+01 -2.760000e+01 5.266500e+00
    endloop
  endfacet
  facet normal 9.892642e-01 -1.461162e-01 -2.525643e-03
    outer loop
      vertex   -1.870011e+01 -2.757628e+01 3.850000e+00
      vertex   -1.870000e+01 -2.760000e+01 5.266500e+00
      vertex   -1.881371e+01 -2.834542e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.951245e-01 -9.378020e-02 -3.053738e-02
    outer loop
      vertex   -1.881371e+01 -2.834542e+01 3.850000e+00
      vertex   -1.870000e+01 -2.760000e+01 5.266500e+00
      vertex   -1.873849e+01 -2.803704e+01 5.354142e+00
    endloop
  endfacet
  facet normal 9.689802e-01 -2.471284e-01 2.209679e-03
    outer loop
      vertex   -1.881371e+01 -2.834542e+01 3.850000e+00
      vertex   -1.873849e+01 -2.803704e+01 5.354142e+00
      vertex   -1.883214e+01 -2.840203e+01 5.599019e+00
    endloop
  endfacet
  facet normal 9.277828e-01 -3.731137e-01 -2.302102e-03
    outer loop
      vertex   -1.883214e+01 -2.840203e+01 5.599019e+00
      vertex   -1.892941e+01 -2.864616e+01 5.963679e+00
      vertex   -1.881371e+01 -2.834542e+01 3.850000e+00
    endloop
  endfacet
  facet normal 8.964517e-01 -4.429218e-01 -1.394957e-02
    outer loop
      vertex   -1.881371e+01 -2.834542e+01 3.850000e+00
      vertex   -1.892941e+01 -2.864616e+01 5.963679e+00
      vertex   -1.915811e+01 -2.904246e+01 3.850000e+00
    endloop
  endfacet
  facet normal 8.838121e-01 -4.677749e-01 -7.922091e-03
    outer loop
      vertex   -1.915811e+01 -2.904246e+01 3.850000e+00
      vertex   -1.892941e+01 -2.864616e+01 5.963679e+00
      vertex   -1.897173e+01 -2.873350e+01 6.400000e+00
    endloop
  endfacet
  facet normal 8.544671e-01 -5.195053e-01 4.905736e-04
    outer loop
      vertex   -1.915811e+01 -2.904246e+01 3.850000e+00
      vertex   -1.897173e+01 -2.873350e+01 6.400000e+00
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
    endloop
  endfacet
  facet normal 8.816187e-01 -4.718774e-01 8.956054e-03
    outer loop
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
      vertex   -1.897173e+01 -2.873350e+01 6.400000e+00
      vertex   -1.892941e+01 -2.864616e+01 6.836321e+00
    endloop
  endfacet
  facet normal 8.909237e-01 -4.539483e-01 1.363866e-02
    outer loop
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
      vertex   -1.892941e+01 -2.864616e+01 6.836321e+00
      vertex   -1.882236e+01 -2.837254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.284212e-01 -3.715278e-01 1.070121e-03
    outer loop
      vertex   -1.882236e+01 -2.837254e+01 8.950000e+00
      vertex   -1.892941e+01 -2.864616e+01 6.836321e+00
      vertex   -1.883214e+01 -2.840203e+01 7.200981e+00
    endloop
  endfacet
  facet normal 9.688264e-01 -2.477376e-01 -1.242888e-03
    outer loop
      vertex   -1.882236e+01 -2.837254e+01 8.950000e+00
      vertex   -1.883214e+01 -2.840203e+01 7.200981e+00
      vertex   -1.873849e+01 -2.803704e+01 7.445858e+00
    endloop
  endfacet
  facet normal 9.874874e-01 -1.564026e-01 2.017014e-02
    outer loop
      vertex   -1.882236e+01 -2.837254e+01 8.950000e+00
      vertex   -1.873849e+01 -2.803704e+01 7.445858e+00
      vertex   -1.870000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.961429e-01 -8.774608e-02 8.803013e-16
    outer loop
      vertex   -1.870000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.873849e+01 -2.803704e+01 7.445858e+00
      vertex   -1.870000e+01 -2.760000e+01 7.533500e+00
    endloop
  endfacet
  facet normal 9.961429e-01 8.774608e-02 -8.803013e-16
    outer loop
      vertex   -1.870000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.870000e+01 -2.760000e+01 7.533500e+00
      vertex   -1.873849e+01 -2.716296e+01 7.445858e+00
    endloop
  endfacet
  facet normal 9.874874e-01 1.564026e-01 2.017014e-02
    outer loop
      vertex   -1.870000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.873849e+01 -2.716296e+01 7.445858e+00
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.688264e-01 2.477376e-01 -1.242888e-03
    outer loop
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
      vertex   -1.873849e+01 -2.716296e+01 7.445858e+00
      vertex   -1.883214e+01 -2.679797e+01 7.200981e+00
    endloop
  endfacet
  facet normal 9.284212e-01 3.715278e-01 1.070121e-03
    outer loop
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
      vertex   -1.883214e+01 -2.679797e+01 7.200981e+00
      vertex   -1.892941e+01 -2.655384e+01 6.836321e+00
    endloop
  endfacet
  facet normal 8.361427e-01 5.477680e-01 2.855823e-02
    outer loop
      vertex   -1.892941e+01 -2.655384e+01 6.836321e+00
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 8.909416e-01 4.539574e-01 1.206916e-02
    outer loop
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
      vertex   -1.897173e+01 -2.646650e+01 6.400000e+00
      vertex   -1.917745e+01 -2.613054e+01 8.950000e+00
    endloop
  endfacet
  facet normal -8.816187e-01 4.718774e-01 8.956054e-03
    outer loop
      vertex   -2.342826e+01 -2.646650e+01 6.400000e+00
      vertex   -2.347058e+01 -2.655384e+01 6.836321e+00
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
    endloop
  endfacet
  facet normal -8.909237e-01 4.539483e-01 1.363866e-02
    outer loop
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
      vertex   -2.347058e+01 -2.655384e+01 6.836321e+00
      vertex   -2.357764e+01 -2.682746e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.284212e-01 3.715278e-01 1.070121e-03
    outer loop
      vertex   -2.357764e+01 -2.682746e+01 8.950000e+00
      vertex   -2.347058e+01 -2.655384e+01 6.836321e+00
      vertex   -2.356785e+01 -2.679797e+01 7.200981e+00
    endloop
  endfacet
  facet normal -9.688264e-01 2.477376e-01 -1.242888e-03
    outer loop
      vertex   -2.357764e+01 -2.682746e+01 8.950000e+00
      vertex   -2.356785e+01 -2.679797e+01 7.200981e+00
      vertex   -2.366150e+01 -2.716296e+01 7.445858e+00
    endloop
  endfacet
  facet normal -9.874874e-01 1.564026e-01 2.017014e-02
    outer loop
      vertex   -2.357764e+01 -2.682746e+01 8.950000e+00
      vertex   -2.366150e+01 -2.716296e+01 7.445858e+00
      vertex   -2.370000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.961429e-01 8.774608e-02 -7.715332e-15
    outer loop
      vertex   -2.370000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.366150e+01 -2.716296e+01 7.445858e+00
      vertex   -2.370000e+01 -2.760000e+01 7.533500e+00
    endloop
  endfacet
  facet normal -9.961429e-01 -8.774608e-02 -7.275181e-15
    outer loop
      vertex   -2.370000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.370000e+01 -2.760000e+01 7.533500e+00
      vertex   -2.366150e+01 -2.803704e+01 7.445858e+00
    endloop
  endfacet
  facet normal -9.874874e-01 -1.564026e-01 2.017014e-02
    outer loop
      vertex   -2.370000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.366150e+01 -2.803704e+01 7.445858e+00
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.688264e-01 -2.477376e-01 -1.242888e-03
    outer loop
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
      vertex   -2.366150e+01 -2.803704e+01 7.445858e+00
      vertex   -2.356785e+01 -2.840203e+01 7.200981e+00
    endloop
  endfacet
  facet normal -9.284212e-01 -3.715278e-01 1.070121e-03
    outer loop
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
      vertex   -2.356785e+01 -2.840203e+01 7.200981e+00
      vertex   -2.347058e+01 -2.864616e+01 6.836321e+00
    endloop
  endfacet
  facet normal -8.361427e-01 -5.477680e-01 2.855823e-02
    outer loop
      vertex   -2.347058e+01 -2.864616e+01 6.836321e+00
      vertex   -2.342826e+01 -2.873350e+01 6.400000e+00
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.124542e-01 -7.017186e-01 0.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 3.850000e+00
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
    endloop
  endfacet
  facet normal 7.070974e-01 -7.070974e-01 5.138916e-03
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
      vertex   -1.973053e+01 -2.962254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.923573e-01 -8.703832e-01 -4.168683e-03
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 7.850000e+00
      vertex   -1.973053e+01 -2.962254e+01 8.950000e+00
      vertex   -2.025855e+01 -2.991596e+01 7.850000e+00
    endloop
  endfacet
  facet normal 4.539020e-01 -8.908328e-01 1.974522e-02
    outer loop
      vertex   -2.025855e+01 -2.991596e+01 7.850000e+00
      vertex   -1.973053e+01 -2.962254e+01 8.950000e+00
      vertex   -2.042745e+01 -2.997764e+01 8.950000e+00
    endloop
  endfacet
  facet normal 2.545383e-01 -9.669442e-01 -1.513485e-02
    outer loop
      vertex   -2.025855e+01 -2.991596e+01 7.850000e+00
      vertex   -2.042745e+01 -2.997764e+01 8.950000e+00
      vertex   -2.087913e+01 -3.007932e+01 7.850000e+00
    endloop
  endfacet
  facet normal 1.563772e-01 -9.873268e-01 2.705598e-02
    outer loop
      vertex   -2.087913e+01 -3.007932e+01 7.850000e+00
      vertex   -2.042745e+01 -2.997764e+01 8.950000e+00
      vertex   -2.120000e+01 -3.010000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -9.998234e-01 -1.879315e-02
    outer loop
      vertex   -2.087913e+01 -3.007932e+01 7.850000e+00
      vertex   -2.120000e+01 -3.010000e+01 8.950000e+00
      vertex   -2.152086e+01 -3.007932e+01 7.850000e+00
    endloop
  endfacet
  facet normal -1.563772e-01 -9.873268e-01 2.705598e-02
    outer loop
      vertex   -2.152086e+01 -3.007932e+01 7.850000e+00
      vertex   -2.120000e+01 -3.010000e+01 8.950000e+00
      vertex   -2.197254e+01 -2.997764e+01 8.950000e+00
    endloop
  endfacet
  facet normal -2.545383e-01 -9.669442e-01 -1.513485e-02
    outer loop
      vertex   -2.152086e+01 -3.007932e+01 7.850000e+00
      vertex   -2.197254e+01 -2.997764e+01 8.950000e+00
      vertex   -2.214144e+01 -2.991596e+01 7.850000e+00
    endloop
  endfacet
  facet normal -4.539020e-01 -8.908328e-01 1.974522e-02
    outer loop
      vertex   -2.214144e+01 -2.991596e+01 7.850000e+00
      vertex   -2.197254e+01 -2.997764e+01 8.950000e+00
      vertex   -2.266946e+01 -2.962254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.923573e-01 -8.703832e-01 -4.168683e-03
    outer loop
      vertex   -2.214144e+01 -2.991596e+01 7.850000e+00
      vertex   -2.266946e+01 -2.962254e+01 8.950000e+00
      vertex   -2.270000e+01 -2.960000e+01 7.850000e+00
    endloop
  endfacet
  facet normal -7.070974e-01 -7.070974e-01 5.138916e-03
    outer loop
      vertex   -2.270000e+01 -2.960000e+01 7.850000e+00
      vertex   -2.266946e+01 -2.962254e+01 8.950000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.124542e-01 -7.017186e-01 -0.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.960000e+01 7.850000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
    endloop
  endfacet
  facet normal -7.170992e-01 -6.969704e-01 -9.698590e-04
    outer loop
      vertex   -2.270000e+01 -2.960000e+01 3.850000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
      vertex   -2.324188e+01 -2.904246e+01 3.850000e+00
    endloop
  endfacet
  facet normal 7.170992e-01 -6.969704e-01 -9.698590e-04
    outer loop
      vertex   -1.970000e+01 -2.960000e+01 3.850000e+00
      vertex   -1.915811e+01 -2.904246e+01 3.850000e+00
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.104537e-01 7.037439e-01 0.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
      vertex   -1.970000e+01 -2.560000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 7.037311e-01 7.104408e-01 -6.028134e-03
    outer loop
      vertex   -1.970000e+01 -2.560000e+01 5.000000e+00
      vertex   -1.918585e+01 -2.611904e+01 3.850000e+00
      vertex   -1.973822e+01 -2.557189e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.923556e-01 8.703803e-01 4.906520e-03
    outer loop
      vertex   -1.970000e+01 -2.560000e+01 5.000000e+00
      vertex   -1.973822e+01 -2.557189e+01 3.850000e+00
      vertex   -2.025855e+01 -2.528404e+01 5.000000e+00
    endloop
  endfacet
  facet normal 4.513700e-01 8.921328e-01 -1.908267e-02
    outer loop
      vertex   -2.025855e+01 -2.528404e+01 5.000000e+00
      vertex   -1.973822e+01 -2.557189e+01 3.850000e+00
      vertex   -2.043197e+01 -2.522090e+01 3.850000e+00
    endloop
  endfacet
  facet normal 2.545400e-01 9.669504e-01 1.470702e-02
    outer loop
      vertex   -2.025855e+01 -2.528404e+01 5.000000e+00
      vertex   -2.043197e+01 -2.522090e+01 3.850000e+00
      vertex   -2.087913e+01 -2.512067e+01 5.000000e+00
    endloop
  endfacet
  facet normal 1.554461e-01 9.875122e-01 -2.561652e-02
    outer loop
      vertex   -2.087913e+01 -2.512067e+01 5.000000e+00
      vertex   -2.043197e+01 -2.522090e+01 3.850000e+00
      vertex   -2.120000e+01 -2.510000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 9.998384e-01 1.797633e-02
    outer loop
      vertex   -2.087913e+01 -2.512067e+01 5.000000e+00
      vertex   -2.120000e+01 -2.510000e+01 3.850000e+00
      vertex   -2.152086e+01 -2.512067e+01 5.000000e+00
    endloop
  endfacet
  facet normal -1.554461e-01 9.875122e-01 -2.561652e-02
    outer loop
      vertex   -2.152086e+01 -2.512067e+01 5.000000e+00
      vertex   -2.120000e+01 -2.510000e+01 3.850000e+00
      vertex   -2.196803e+01 -2.522090e+01 3.850000e+00
    endloop
  endfacet
  facet normal -2.545400e-01 9.669504e-01 1.470702e-02
    outer loop
      vertex   -2.152086e+01 -2.512067e+01 5.000000e+00
      vertex   -2.196803e+01 -2.522090e+01 3.850000e+00
      vertex   -2.214144e+01 -2.528404e+01 5.000000e+00
    endloop
  endfacet
  facet normal -4.513700e-01 8.921328e-01 -1.908267e-02
    outer loop
      vertex   -2.214144e+01 -2.528404e+01 5.000000e+00
      vertex   -2.196803e+01 -2.522090e+01 3.850000e+00
      vertex   -2.266177e+01 -2.557189e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.923556e-01 8.703803e-01 4.906520e-03
    outer loop
      vertex   -2.214144e+01 -2.528404e+01 5.000000e+00
      vertex   -2.266177e+01 -2.557189e+01 3.850000e+00
      vertex   -2.270000e+01 -2.560000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -7.037311e-01 7.104408e-01 -6.028134e-03
    outer loop
      vertex   -2.270000e+01 -2.560000e+01 5.000000e+00
      vertex   -2.266177e+01 -2.557189e+01 3.850000e+00
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
    endloop
  endfacet
  facet normal -7.104537e-01 7.037439e-01 0.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.560000e+01 5.000000e+00
      vertex   -2.321414e+01 -2.611904e+01 3.850000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.882236e+01 -2.837254e+01 8.950000e+00
      vertex   -1.870000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.995000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.870000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
      vertex   -2.007379e+01 -2.705764e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.286626e-16 2.330451e-15 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 8.950000e+00
      vertex   -1.882236e+01 -2.682746e+01 8.950000e+00
      vertex   -1.917745e+01 -2.613054e+01 8.950000e+00
    endloop
  endfacet
  facet normal 1.086008e-15 8.660624e-16 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 8.950000e+00
      vertex   -1.917745e+01 -2.613054e+01 8.950000e+00
      vertex   -2.042063e+01 -2.662271e+01 8.950000e+00
    endloop
  endfacet
  facet normal 1.023628e-15 1.023628e-15 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 8.950000e+00
      vertex   -1.917745e+01 -2.613054e+01 8.950000e+00
      vertex   -1.973053e+01 -2.557746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 6.479446e-16 1.271663e-15 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 8.950000e+00
      vertex   -1.973053e+01 -2.557746e+01 8.950000e+00
      vertex   -2.042745e+01 -2.522236e+01 8.950000e+00
    endloop
  endfacet
  facet normal 6.123170e-16 1.271489e-15 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 8.950000e+00
      vertex   -2.042745e+01 -2.522236e+01 8.950000e+00
      vertex   -2.092185e+01 -2.638134e+01 8.950000e+00
    endloop
  endfacet
  facet normal 2.273908e-16 1.435689e-15 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.638134e+01 8.950000e+00
      vertex   -2.042745e+01 -2.522236e+01 8.950000e+00
      vertex   -2.120000e+01 -2.510000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.386327e-15 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.638134e+01 8.950000e+00
      vertex   -2.120000e+01 -2.510000e+01 8.950000e+00
      vertex   -2.147815e+01 -2.638134e+01 8.950000e+00
    endloop
  endfacet
  facet normal -2.273908e-16 1.435689e-15 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.638134e+01 8.950000e+00
      vertex   -2.120000e+01 -2.510000e+01 8.950000e+00
      vertex   -2.197254e+01 -2.522236e+01 8.950000e+00
    endloop
  endfacet
  facet normal -6.123170e-16 1.271489e-15 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.638134e+01 8.950000e+00
      vertex   -2.197254e+01 -2.522236e+01 8.950000e+00
      vertex   -2.197936e+01 -2.662271e+01 8.950000e+00
    endloop
  endfacet
  facet normal -6.479446e-16 1.271663e-15 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 8.950000e+00
      vertex   -2.197254e+01 -2.522236e+01 8.950000e+00
      vertex   -2.266946e+01 -2.557746e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.023628e-15 1.023628e-15 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 8.950000e+00
      vertex   -2.266946e+01 -2.557746e+01 8.950000e+00
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.086008e-15 8.660624e-16 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 8.950000e+00
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
      vertex   -2.232621e+01 -2.705764e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.286626e-16 2.330451e-15 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.705764e+01 8.950000e+00
      vertex   -2.322254e+01 -2.613054e+01 8.950000e+00
      vertex   -2.357764e+01 -2.682746e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.705764e+01 8.950000e+00
      vertex   -2.357764e+01 -2.682746e+01 8.950000e+00
      vertex   -2.245000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.357764e+01 -2.682746e+01 8.950000e+00
      vertex   -2.370000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.370000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
      vertex   -2.232621e+01 -2.814235e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.286626e-16 2.330451e-15 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.814235e+01 8.950000e+00
      vertex   -2.357764e+01 -2.837254e+01 8.950000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
    endloop
  endfacet
  facet normal 1.086008e-15 8.660624e-16 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.814235e+01 8.950000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
      vertex   -2.197936e+01 -2.857729e+01 8.950000e+00
    endloop
  endfacet
  facet normal 1.023628e-15 1.023628e-15 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 8.950000e+00
      vertex   -2.322254e+01 -2.906946e+01 8.950000e+00
      vertex   -2.266946e+01 -2.962254e+01 8.950000e+00
    endloop
  endfacet
  facet normal 6.479446e-16 1.271663e-15 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 8.950000e+00
      vertex   -2.266946e+01 -2.962254e+01 8.950000e+00
      vertex   -2.197254e+01 -2.997764e+01 8.950000e+00
    endloop
  endfacet
  facet normal 6.123170e-16 1.271489e-15 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 8.950000e+00
      vertex   -2.197254e+01 -2.997764e+01 8.950000e+00
      vertex   -2.147815e+01 -2.881866e+01 8.950000e+00
    endloop
  endfacet
  facet normal 2.273908e-16 1.435689e-15 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.881866e+01 8.950000e+00
      vertex   -2.197254e+01 -2.997764e+01 8.950000e+00
      vertex   -2.120000e+01 -3.010000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.386327e-15 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.881866e+01 8.950000e+00
      vertex   -2.120000e+01 -3.010000e+01 8.950000e+00
      vertex   -2.092185e+01 -2.881866e+01 8.950000e+00
    endloop
  endfacet
  facet normal -2.273908e-16 1.435689e-15 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.881866e+01 8.950000e+00
      vertex   -2.120000e+01 -3.010000e+01 8.950000e+00
      vertex   -2.042745e+01 -2.997764e+01 8.950000e+00
    endloop
  endfacet
  facet normal -6.123170e-16 1.271489e-15 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.881866e+01 8.950000e+00
      vertex   -2.042745e+01 -2.997764e+01 8.950000e+00
      vertex   -2.042063e+01 -2.857729e+01 8.950000e+00
    endloop
  endfacet
  facet normal -6.479446e-16 1.271663e-15 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 8.950000e+00
      vertex   -2.042745e+01 -2.997764e+01 8.950000e+00
      vertex   -1.973053e+01 -2.962254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.023628e-15 1.023628e-15 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 8.950000e+00
      vertex   -1.973053e+01 -2.962254e+01 8.950000e+00
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.086008e-15 8.660624e-16 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 8.950000e+00
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
      vertex   -2.007379e+01 -2.814235e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.286626e-16 2.330451e-15 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 8.950000e+00
      vertex   -1.917745e+01 -2.906946e+01 8.950000e+00
      vertex   -1.882236e+01 -2.837254e+01 8.950000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 8.950000e+00
      vertex   -1.882236e+01 -2.837254e+01 8.950000e+00
      vertex   -1.995000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -1.679862e-15 -1.586033e-16 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.560000e+01 5.000000e+00
      vertex   -2.025855e+01 -2.528404e+01 5.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -8.609867e-17 -3.270730e-16 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.025855e+01 -2.528404e+01 5.000000e+00
      vertex   -2.087913e+01 -2.512067e+01 5.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -3.468989e-16 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.087913e+01 -2.512067e+01 5.000000e+00
      vertex   -2.152086e+01 -2.512067e+01 5.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -3.468989e-16 -1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.152086e+01 -2.512067e+01 5.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 8.609867e-17 -3.270730e-16 -1.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.152086e+01 -2.512067e+01 5.000000e+00
      vertex   -2.214144e+01 -2.528404e+01 5.000000e+00
    endloop
  endfacet
  facet normal 1.679862e-15 -1.586033e-16 -1.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.214144e+01 -2.528404e+01 5.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.560000e+01 5.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 5.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 5.153378e-15
      vertex   -2.007379e+01 -2.705764e+01 3.850000e+00
      vertex   -1.995000e+01 -2.760000e+01 5.795364e-15
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 5.795364e-15
      vertex   -2.007379e+01 -2.705764e+01 3.850000e+00
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 5.795364e-15
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.007379e+01 -2.814235e+01 6.437350e-15
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 6.437350e-15
      vertex   -1.995000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.007379e+01 -2.814235e+01 3.850000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 6.437350e-15
      vertex   -2.007379e+01 -2.814235e+01 3.850000e+00
      vertex   -2.042063e+01 -2.857729e+01 6.952183e-15
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 6.952183e-15
      vertex   -2.007379e+01 -2.814235e+01 3.850000e+00
      vertex   -2.042063e+01 -2.857729e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 6.952183e-15
      vertex   -2.042063e+01 -2.857729e+01 3.850000e+00
      vertex   -2.092185e+01 -2.881866e+01 7.237894e-15
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.881866e+01 7.237894e-15
      vertex   -2.042063e+01 -2.857729e+01 3.850000e+00
      vertex   -2.092185e+01 -2.881866e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.881866e+01 7.237894e-15
      vertex   -2.092185e+01 -2.881866e+01 3.850000e+00
      vertex   -2.147815e+01 -2.881866e+01 7.237894e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.881866e+01 7.237894e-15
      vertex   -2.092185e+01 -2.881866e+01 3.850000e+00
      vertex   -2.147815e+01 -2.881866e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.881866e+01 7.237894e-15
      vertex   -2.147815e+01 -2.881866e+01 3.850000e+00
      vertex   -2.197936e+01 -2.857729e+01 6.952183e-15
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 6.952183e-15
      vertex   -2.147815e+01 -2.881866e+01 3.850000e+00
      vertex   -2.197936e+01 -2.857729e+01 3.850000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 6.952183e-15
      vertex   -2.197936e+01 -2.857729e+01 3.850000e+00
      vertex   -2.232621e+01 -2.814235e+01 6.437350e-15
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.814235e+01 6.437350e-15
      vertex   -2.197936e+01 -2.857729e+01 3.850000e+00
      vertex   -2.232621e+01 -2.814235e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.814235e+01 6.437350e-15
      vertex   -2.232621e+01 -2.814235e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 5.795364e-15
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.760000e+01 5.795364e-15
      vertex   -2.232621e+01 -2.814235e+01 3.850000e+00
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.760000e+01 5.795364e-15
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.232621e+01 -2.705764e+01 5.153378e-15
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.705764e+01 5.153378e-15
      vertex   -2.245000e+01 -2.760000e+01 3.850000e+00
      vertex   -2.232621e+01 -2.705764e+01 3.850000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.705764e+01 5.153378e-15
      vertex   -2.232621e+01 -2.705764e+01 3.850000e+00
      vertex   -2.197936e+01 -2.662271e+01 4.638545e-15
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 4.638545e-15
      vertex   -2.232621e+01 -2.705764e+01 3.850000e+00
      vertex   -2.197936e+01 -2.662271e+01 3.850000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 4.638545e-15
      vertex   -2.197936e+01 -2.662271e+01 3.850000e+00
      vertex   -2.147815e+01 -2.638134e+01 4.352834e-15
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.638134e+01 4.352834e-15
      vertex   -2.197936e+01 -2.662271e+01 3.850000e+00
      vertex   -2.147815e+01 -2.638134e+01 3.850000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.638134e+01 4.352834e-15
      vertex   -2.147815e+01 -2.638134e+01 3.850000e+00
      vertex   -2.092185e+01 -2.638134e+01 4.352834e-15
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.638134e+01 4.352834e-15
      vertex   -2.147815e+01 -2.638134e+01 3.850000e+00
      vertex   -2.092185e+01 -2.638134e+01 3.850000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.638134e+01 4.352834e-15
      vertex   -2.092185e+01 -2.638134e+01 3.850000e+00
      vertex   -2.042063e+01 -2.662271e+01 4.638545e-15
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 4.638545e-15
      vertex   -2.092185e+01 -2.638134e+01 3.850000e+00
      vertex   -2.042063e+01 -2.662271e+01 3.850000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 4.638545e-15
      vertex   -2.042063e+01 -2.662271e+01 3.850000e+00
      vertex   -2.007379e+01 -2.705764e+01 5.153378e-15
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 5.153378e-15
      vertex   -2.042063e+01 -2.662271e+01 3.850000e+00
      vertex   -2.007379e+01 -2.705764e+01 3.850000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 8.950000e+00
      vertex   -2.007379e+01 -2.705764e+01 1.495000e+01
      vertex   -1.995000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 2.225209e-01 -0.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.007379e+01 -2.705764e+01 1.495000e+01
      vertex   -1.995000e+01 -2.760000e+01 1.495000e+01
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 8.950000e+00
      vertex   -1.995000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.007379e+01 -2.814235e+01 8.950000e+00
    endloop
  endfacet
  facet normal 9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 8.950000e+00
      vertex   -1.995000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.007379e+01 -2.814235e+01 1.495000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 8.950000e+00
      vertex   -2.007379e+01 -2.814235e+01 1.495000e+01
      vertex   -2.042063e+01 -2.857729e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 8.950000e+00
      vertex   -2.007379e+01 -2.814235e+01 1.495000e+01
      vertex   -2.042063e+01 -2.857729e+01 1.495000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 8.950000e+00
      vertex   -2.042063e+01 -2.857729e+01 1.495000e+01
      vertex   -2.092185e+01 -2.881866e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.881866e+01 8.950000e+00
      vertex   -2.042063e+01 -2.857729e+01 1.495000e+01
      vertex   -2.092185e+01 -2.881866e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.881866e+01 8.950000e+00
      vertex   -2.092185e+01 -2.881866e+01 1.495000e+01
      vertex   -2.147815e+01 -2.881866e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.881866e+01 8.950000e+00
      vertex   -2.092185e+01 -2.881866e+01 1.495000e+01
      vertex   -2.147815e+01 -2.881866e+01 1.495000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.881866e+01 8.950000e+00
      vertex   -2.147815e+01 -2.881866e+01 1.495000e+01
      vertex   -2.197936e+01 -2.857729e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 -9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 8.950000e+00
      vertex   -2.147815e+01 -2.881866e+01 1.495000e+01
      vertex   -2.197936e+01 -2.857729e+01 1.495000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 8.950000e+00
      vertex   -2.197936e+01 -2.857729e+01 1.495000e+01
      vertex   -2.232621e+01 -2.814235e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 -6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.814235e+01 8.950000e+00
      vertex   -2.197936e+01 -2.857729e+01 1.495000e+01
      vertex   -2.232621e+01 -2.814235e+01 1.495000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.814235e+01 8.950000e+00
      vertex   -2.232621e+01 -2.814235e+01 1.495000e+01
      vertex   -2.245000e+01 -2.760000e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 -2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.232621e+01 -2.814235e+01 1.495000e+01
      vertex   -2.245000e+01 -2.760000e+01 1.495000e+01
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.760000e+01 8.950000e+00
      vertex   -2.245000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.232621e+01 -2.705764e+01 8.950000e+00
    endloop
  endfacet
  facet normal -9.749279e-01 2.225209e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.705764e+01 8.950000e+00
      vertex   -2.245000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.232621e+01 -2.705764e+01 1.495000e+01
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.705764e+01 8.950000e+00
      vertex   -2.232621e+01 -2.705764e+01 1.495000e+01
      vertex   -2.197936e+01 -2.662271e+01 8.950000e+00
    endloop
  endfacet
  facet normal -7.818315e-01 6.234898e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 8.950000e+00
      vertex   -2.232621e+01 -2.705764e+01 1.495000e+01
      vertex   -2.197936e+01 -2.662271e+01 1.495000e+01
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 8.950000e+00
      vertex   -2.197936e+01 -2.662271e+01 1.495000e+01
      vertex   -2.147815e+01 -2.638134e+01 8.950000e+00
    endloop
  endfacet
  facet normal -4.338837e-01 9.009689e-01 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.638134e+01 8.950000e+00
      vertex   -2.197936e+01 -2.662271e+01 1.495000e+01
      vertex   -2.147815e+01 -2.638134e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.638134e+01 8.950000e+00
      vertex   -2.147815e+01 -2.638134e+01 1.495000e+01
      vertex   -2.092185e+01 -2.638134e+01 8.950000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.638134e+01 8.950000e+00
      vertex   -2.147815e+01 -2.638134e+01 1.495000e+01
      vertex   -2.092185e+01 -2.638134e+01 1.495000e+01
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.638134e+01 8.950000e+00
      vertex   -2.092185e+01 -2.638134e+01 1.495000e+01
      vertex   -2.042063e+01 -2.662271e+01 8.950000e+00
    endloop
  endfacet
  facet normal 4.338837e-01 9.009689e-01 -0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 8.950000e+00
      vertex   -2.092185e+01 -2.638134e+01 1.495000e+01
      vertex   -2.042063e+01 -2.662271e+01 1.495000e+01
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 8.950000e+00
      vertex   -2.042063e+01 -2.662271e+01 1.495000e+01
      vertex   -2.007379e+01 -2.705764e+01 8.950000e+00
    endloop
  endfacet
  facet normal 7.818315e-01 6.234898e-01 -0.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 8.950000e+00
      vertex   -2.042063e+01 -2.662271e+01 1.495000e+01
      vertex   -2.007379e+01 -2.705764e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 1.495000e+01
      vertex   -2.232621e+01 -2.705764e+01 1.495000e+01
      vertex   -1.995000e+01 -2.760000e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.232621e+01 -2.705764e+01 1.495000e+01
      vertex   -2.245000e+01 -2.760000e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.245000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.007379e+01 -2.814235e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 1.495000e+01
      vertex   -2.245000e+01 -2.760000e+01 1.495000e+01
      vertex   -2.232621e+01 -2.814235e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 1.495000e+01
      vertex   -2.232621e+01 -2.814235e+01 1.495000e+01
      vertex   -2.042063e+01 -2.857729e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 1.495000e+01
      vertex   -2.232621e+01 -2.814235e+01 1.495000e+01
      vertex   -2.197936e+01 -2.857729e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.857729e+01 1.495000e+01
      vertex   -2.197936e+01 -2.857729e+01 1.495000e+01
      vertex   -2.092185e+01 -2.881866e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.881866e+01 1.495000e+01
      vertex   -2.197936e+01 -2.857729e+01 1.495000e+01
      vertex   -2.147815e+01 -2.881866e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.705764e+01 1.495000e+01
      vertex   -2.007379e+01 -2.705764e+01 1.495000e+01
      vertex   -2.197936e+01 -2.662271e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 4.084192e-15 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 1.495000e+01
      vertex   -2.007379e+01 -2.705764e+01 1.495000e+01
      vertex   -2.042063e+01 -2.662271e+01 1.495000e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.662271e+01 1.495000e+01
      vertex   -2.042063e+01 -2.662271e+01 1.495000e+01
      vertex   -2.147815e+01 -2.638134e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.638134e+01 1.495000e+01
      vertex   -2.042063e+01 -2.662271e+01 1.495000e+01
      vertex   -2.092185e+01 -2.638134e+01 1.495000e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.814235e+01 6.437350e-15
      vertex   -2.232621e+01 -2.814235e+01 6.437350e-15
      vertex   -1.995000e+01 -2.760000e+01 5.795364e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 5.795364e-15
      vertex   -2.232621e+01 -2.814235e+01 6.437350e-15
      vertex   -2.245000e+01 -2.760000e+01 5.795364e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.760000e+01 5.795364e-15
      vertex   -2.245000e+01 -2.760000e+01 5.795364e-15
      vertex   -2.007379e+01 -2.705764e+01 5.153378e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 5.153378e-15
      vertex   -2.245000e+01 -2.760000e+01 5.795364e-15
      vertex   -2.232621e+01 -2.705764e+01 5.153378e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.705764e+01 5.153378e-15
      vertex   -2.232621e+01 -2.705764e+01 5.153378e-15
      vertex   -2.042063e+01 -2.662271e+01 4.638545e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 4.638545e-15
      vertex   -2.232621e+01 -2.705764e+01 5.153378e-15
      vertex   -2.197936e+01 -2.662271e+01 4.638545e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.662271e+01 4.638545e-15
      vertex   -2.197936e+01 -2.662271e+01 4.638545e-15
      vertex   -2.092185e+01 -2.638134e+01 4.352834e-15
    endloop
  endfacet
  facet normal -0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.638134e+01 4.352834e-15
      vertex   -2.197936e+01 -2.662271e+01 4.638545e-15
      vertex   -2.147815e+01 -2.638134e+01 4.352834e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.814235e+01 6.437350e-15
      vertex   -2.007379e+01 -2.814235e+01 6.437350e-15
      vertex   -2.197936e+01 -2.857729e+01 6.952183e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 6.952183e-15
      vertex   -2.007379e+01 -2.814235e+01 6.437350e-15
      vertex   -2.042063e+01 -2.857729e+01 6.952183e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.857729e+01 6.952183e-15
      vertex   -2.042063e+01 -2.857729e+01 6.952183e-15
      vertex   -2.147815e+01 -2.881866e+01 7.237894e-15
    endloop
  endfacet
  facet normal 0.000000e+00 -1.183702e-15 -1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.881866e+01 7.237894e-15
      vertex   -2.042063e+01 -2.857729e+01 6.952183e-15
      vertex   -2.092185e+01 -2.881866e+01 7.237894e-15
    endloop
  endfacet
  facet normal -7.562917e-16 1.726186e-16 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.209235e+01 7.000000e+00
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
      vertex   -1.995000e+01 -2.155000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -9.833404e-16 1.586033e-16 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.155000e+01 7.000000e+00
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.995000e+01 -2.155000e+01 7.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.007379e+01 -2.100764e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.100764e+01 7.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.042063e+01 -2.057271e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.042063e+01 -2.057271e+01 7.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.092185e+01 -2.033134e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.033134e+01 7.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.092185e+01 -2.033134e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.147815e+01 -2.033134e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.033134e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.197936e+01 -2.057271e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.057271e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.232621e+01 -2.100764e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.100764e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.245000e+01 -2.155000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 9.833404e-16 1.586033e-16 1.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.155000e+01 7.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 7.562917e-16 1.726186e-16 1.000000e+00
    outer loop
      vertex   -2.245000e+01 -2.155000e+01 7.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.232621e+01 -2.209235e+01 7.000000e+00
    endloop
  endfacet
  facet normal 2.800903e-16 2.233646e-16 1.000000e+00
    outer loop
      vertex   -2.232621e+01 -2.209235e+01 7.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.197936e+01 -2.252729e+01 7.000000e+00
    endloop
  endfacet
  facet normal -3.855267e-15 1.193223e-15 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.252729e+01 7.000000e+00
      vertex   -2.270000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.214144e+01 -2.528404e+01 7.000000e+00
    endloop
  endfacet
  facet normal 4.526492e-16 9.399358e-16 1.000000e+00
    outer loop
      vertex   -2.197936e+01 -2.252729e+01 7.000000e+00
      vertex   -2.214144e+01 -2.528404e+01 7.000000e+00
      vertex   -2.147815e+01 -2.276866e+01 7.000000e+00
    endloop
  endfacet
  facet normal -2.996498e-16 1.138314e-15 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.276866e+01 7.000000e+00
      vertex   -2.214144e+01 -2.528404e+01 7.000000e+00
      vertex   -2.152086e+01 -2.512067e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.132873e-15 1.000000e+00
    outer loop
      vertex   -2.147815e+01 -2.276866e+01 7.000000e+00
      vertex   -2.152086e+01 -2.512067e+01 7.000000e+00
      vertex   -2.087913e+01 -2.512067e+01 7.000000e+00
    endloop
  endfacet
  facet normal -2.800903e-16 2.233646e-16 1.000000e+00
    outer loop
      vertex   -2.007379e+01 -2.209235e+01 7.000000e+00
      vertex   -2.042063e+01 -2.252729e+01 7.000000e+00
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 3.855267e-15 1.193223e-15 1.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
      vertex   -2.042063e+01 -2.252729e+01 7.000000e+00
      vertex   -2.025855e+01 -2.528404e+01 7.000000e+00
    endloop
  endfacet
  facet normal -4.526492e-16 9.399358e-16 1.000000e+00
    outer loop
      vertex   -2.025855e+01 -2.528404e+01 7.000000e+00
      vertex   -2.042063e+01 -2.252729e+01 7.000000e+00
      vertex   -2.092185e+01 -2.276866e+01 7.000000e+00
    endloop
  endfacet
  facet normal 2.996498e-16 1.138314e-15 1.000000e+00
    outer loop
      vertex   -2.025855e+01 -2.528404e+01 7.000000e+00
      vertex   -2.092185e+01 -2.276866e+01 7.000000e+00
      vertex   -2.087913e+01 -2.512067e+01 7.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.132873e-15 1.000000e+00
    outer loop
      vertex   -2.087913e+01 -2.512067e+01 7.000000e+00
      vertex   -2.092185e+01 -2.276866e+01 7.000000e+00
      vertex   -2.147815e+01 -2.276866e+01 7.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.560000e+01 7.000000e+00
      vertex   -1.970000e+01 -2.560000e+01 5.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
      vertex   -1.970000e+01 -2.560000e+01 5.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.270000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 5.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.970000e+01 -2.000000e+01 5.000000e+00
      vertex   -2.270000e+01 -2.000000e+01 7.000000e+00
      vertex   -1.970000e+01 -2.000000e+01 7.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 -2.895899e-16
    outer loop
      vertex   -2.890433e+00 -3.249194e+01 3.000000e+00
      vertex   -2.890433e+00 -3.249194e+01 3.814698e-07
      vertex   -2.766500e+00 -3.307500e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 -0.000000e+00
    outer loop
      vertex   -2.766500e+00 -3.307500e+01 3.000000e+00
      vertex   -2.890433e+00 -3.249194e+01 3.814698e-07
      vertex   -2.766500e+00 -3.307500e+01 3.814698e-07
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 0.000000e+00
    outer loop
      vertex   -2.766500e+00 -3.307500e+01 3.000000e+00
      vertex   -2.766500e+00 -3.307500e+01 3.814698e-07
      vertex   -2.890433e+00 -3.365806e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 2.895899e-16
    outer loop
      vertex   -2.890433e+00 -3.365806e+01 3.000000e+00
      vertex   -2.766500e+00 -3.307500e+01 3.814698e-07
      vertex   -2.890433e+00 -3.365806e+01 3.814698e-07
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 2.395171e-16
    outer loop
      vertex   -2.890433e+00 -3.365806e+01 3.000000e+00
      vertex   -2.890433e+00 -3.365806e+01 3.814698e-07
      vertex   -3.240801e+00 -3.414030e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 1.197586e-16
    outer loop
      vertex   -3.240801e+00 -3.414030e+01 3.000000e+00
      vertex   -2.890433e+00 -3.365806e+01 3.814698e-07
      vertex   -3.240801e+00 -3.414030e+01 3.814698e-07
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 7.401487e-17
    outer loop
      vertex   -3.240801e+00 -3.414030e+01 3.000000e+00
      vertex   -3.240801e+00 -3.414030e+01 3.814698e-07
      vertex   -3.757024e+00 -3.443834e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 0.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.443834e+01 3.000000e+00
      vertex   -3.240801e+00 -3.414030e+01 3.814698e-07
      vertex   -3.757024e+00 -3.443834e+01 3.814698e-07
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 0.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.443834e+01 3.000000e+00
      vertex   -3.757024e+00 -3.443834e+01 3.814698e-07
      vertex   -4.349842e+00 -3.450065e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 3.094664e-17
    outer loop
      vertex   -4.349842e+00 -3.450065e+01 3.000000e+00
      vertex   -3.757024e+00 -3.443834e+01 3.814698e-07
      vertex   -4.349842e+00 -3.450065e+01 3.814698e-07
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 -9.148741e-17
    outer loop
      vertex   -4.349842e+00 -3.450065e+01 3.000000e+00
      vertex   -4.349842e+00 -3.450065e+01 3.814698e-07
      vertex   -4.916750e+00 -3.431645e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 0.000000e+00
    outer loop
      vertex   -4.916750e+00 -3.431645e+01 3.000000e+00
      vertex   -4.349842e+00 -3.450065e+01 3.814698e-07
      vertex   -4.916750e+00 -3.431645e+01 3.814698e-07
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 0.000000e+00
    outer loop
      vertex   -4.916750e+00 -3.431645e+01 3.000000e+00
      vertex   -4.916750e+00 -3.431645e+01 3.814698e-07
      vertex   -5.359726e+00 -3.391759e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 0.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.391759e+01 3.000000e+00
      vertex   -4.916750e+00 -3.431645e+01 3.814698e-07
      vertex   -5.359726e+00 -3.391759e+01 3.814698e-07
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 0.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.391759e+01 3.000000e+00
      vertex   -5.359726e+00 -3.391759e+01 3.814698e-07
      vertex   -5.602175e+00 -3.337304e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 0.000000e+00
    outer loop
      vertex   -5.602175e+00 -3.337304e+01 3.000000e+00
      vertex   -5.359726e+00 -3.391759e+01 3.814698e-07
      vertex   -5.602175e+00 -3.337304e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.602175e+00 -3.337304e+01 3.000000e+00
      vertex   -5.602175e+00 -3.337304e+01 3.814698e-07
      vertex   -5.602175e+00 -3.277696e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.602175e+00 -3.277696e+01 3.000000e+00
      vertex   -5.602175e+00 -3.337304e+01 3.814698e-07
      vertex   -5.602175e+00 -3.277696e+01 3.814698e-07
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 0.000000e+00
    outer loop
      vertex   -5.602175e+00 -3.277696e+01 3.000000e+00
      vertex   -5.602175e+00 -3.277696e+01 3.814698e-07
      vertex   -5.359726e+00 -3.223241e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 0.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.223241e+01 3.000000e+00
      vertex   -5.602175e+00 -3.277696e+01 3.814698e-07
      vertex   -5.359726e+00 -3.223241e+01 3.814698e-07
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 0.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.223241e+01 3.000000e+00
      vertex   -5.359726e+00 -3.223241e+01 3.814698e-07
      vertex   -4.916750e+00 -3.183355e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 0.000000e+00
    outer loop
      vertex   -4.916750e+00 -3.183355e+01 3.000000e+00
      vertex   -5.359726e+00 -3.223241e+01 3.814698e-07
      vertex   -4.916750e+00 -3.183355e+01 3.814698e-07
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 0.000000e+00
    outer loop
      vertex   -4.916750e+00 -3.183355e+01 3.000000e+00
      vertex   -4.916750e+00 -3.183355e+01 3.814698e-07
      vertex   -4.349842e+00 -3.164935e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 9.148741e-17
    outer loop
      vertex   -4.349842e+00 -3.164935e+01 3.000000e+00
      vertex   -4.916750e+00 -3.183355e+01 3.814698e-07
      vertex   -4.349842e+00 -3.164935e+01 3.814698e-07
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 -3.094664e-17
    outer loop
      vertex   -4.349842e+00 -3.164935e+01 3.000000e+00
      vertex   -4.349842e+00 -3.164935e+01 3.814698e-07
      vertex   -3.757024e+00 -3.171166e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 -0.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.171166e+01 3.000000e+00
      vertex   -4.349842e+00 -3.164935e+01 3.814698e-07
      vertex   -3.757024e+00 -3.171166e+01 3.814698e-07
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 -0.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.171166e+01 3.000000e+00
      vertex   -3.757024e+00 -3.171166e+01 3.814698e-07
      vertex   -3.240801e+00 -3.200970e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 -7.401487e-17
    outer loop
      vertex   -3.240801e+00 -3.200970e+01 3.000000e+00
      vertex   -3.757024e+00 -3.171166e+01 3.814698e-07
      vertex   -3.240801e+00 -3.200970e+01 3.814698e-07
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 -1.197586e-16
    outer loop
      vertex   -3.240801e+00 -3.200970e+01 3.000000e+00
      vertex   -3.240801e+00 -3.200970e+01 3.814698e-07
      vertex   -2.890433e+00 -3.249194e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 -2.395171e-16
    outer loop
      vertex   -2.890433e+00 -3.249194e+01 3.000000e+00
      vertex   -3.240801e+00 -3.200970e+01 3.814698e-07
      vertex   -2.890433e+00 -3.249194e+01 3.814698e-07
    endloop
  endfacet
  facet normal -9.829731e-01 -1.837495e-01 -0.000000e+00
    outer loop
      vertex   7.593129e+00 -3.208282e+01 8.100000e+00
      vertex   7.593129e+00 -3.208282e+01 3.991500e+00
      vertex   7.708500e+00 -3.270000e+01 8.100000e+00
    endloop
  endfacet
  facet normal -9.829731e-01 -1.837495e-01 -0.000000e+00
    outer loop
      vertex   7.708500e+00 -3.270000e+01 8.100000e+00
      vertex   7.593129e+00 -3.208282e+01 3.991500e+00
      vertex   7.708500e+00 -3.270000e+01 3.991500e+00
    endloop
  endfacet
  facet normal -9.829731e-01 1.837495e-01 0.000000e+00
    outer loop
      vertex   7.708500e+00 -3.270000e+01 8.100000e+00
      vertex   7.708500e+00 -3.270000e+01 3.991500e+00
      vertex   7.593129e+00 -3.331718e+01 8.100000e+00
    endloop
  endfacet
  facet normal -9.829731e-01 1.837495e-01 0.000000e+00
    outer loop
      vertex   7.593129e+00 -3.331718e+01 8.100000e+00
      vertex   7.708500e+00 -3.270000e+01 3.991500e+00
      vertex   7.593129e+00 -3.331718e+01 3.991500e+00
    endloop
  endfacet
  facet normal -8.502171e-01 5.264322e-01 0.000000e+00
    outer loop
      vertex   7.593129e+00 -3.331718e+01 8.100000e+00
      vertex   7.593129e+00 -3.331718e+01 3.991500e+00
      vertex   7.262597e+00 -3.385101e+01 8.100000e+00
    endloop
  endfacet
  facet normal -8.502171e-01 5.264322e-01 0.000000e+00
    outer loop
      vertex   7.262597e+00 -3.385101e+01 8.100000e+00
      vertex   7.593129e+00 -3.331718e+01 3.991500e+00
      vertex   7.262597e+00 -3.385101e+01 3.991500e+00
    endloop
  endfacet
  facet normal -6.026346e-01 7.980172e-01 0.000000e+00
    outer loop
      vertex   7.262597e+00 -3.385101e+01 8.100000e+00
      vertex   7.262597e+00 -3.385101e+01 3.991500e+00
      vertex   6.761544e+00 -3.422939e+01 8.100000e+00
    endloop
  endfacet
  facet normal -6.026346e-01 7.980172e-01 0.000000e+00
    outer loop
      vertex   6.761544e+00 -3.422939e+01 8.100000e+00
      vertex   7.262597e+00 -3.385101e+01 3.991500e+00
      vertex   6.761544e+00 -3.422939e+01 3.991500e+00
    endloop
  endfacet
  facet normal -2.736630e-01 9.618256e-01 0.000000e+00
    outer loop
      vertex   6.761544e+00 -3.422939e+01 8.100000e+00
      vertex   6.761544e+00 -3.422939e+01 3.991500e+00
      vertex   6.157640e+00 -3.440121e+01 8.100000e+00
    endloop
  endfacet
  facet normal -2.736630e-01 9.618256e-01 0.000000e+00
    outer loop
      vertex   6.157640e+00 -3.440121e+01 8.100000e+00
      vertex   6.761544e+00 -3.422939e+01 3.991500e+00
      vertex   6.157640e+00 -3.440121e+01 3.991500e+00
    endloop
  endfacet
  facet normal 9.226836e-02 9.957342e-01 0.000000e+00
    outer loop
      vertex   6.157640e+00 -3.440121e+01 8.100000e+00
      vertex   6.157640e+00 -3.440121e+01 3.991500e+00
      vertex   5.532447e+00 -3.434328e+01 8.100000e+00
    endloop
  endfacet
  facet normal 9.226836e-02 9.957342e-01 0.000000e+00
    outer loop
      vertex   5.532447e+00 -3.434328e+01 8.100000e+00
      vertex   6.157640e+00 -3.440121e+01 3.991500e+00
      vertex   5.532447e+00 -3.434328e+01 3.991500e+00
    endloop
  endfacet
  facet normal 4.457384e-01 8.951633e-01 0.000000e+00
    outer loop
      vertex   5.532447e+00 -3.434328e+01 8.100000e+00
      vertex   5.532447e+00 -3.434328e+01 3.991500e+00
      vertex   4.970399e+00 -3.406341e+01 8.100000e+00
    endloop
  endfacet
  facet normal 4.457384e-01 8.951633e-01 0.000000e+00
    outer loop
      vertex   4.970399e+00 -3.406341e+01 8.100000e+00
      vertex   5.532447e+00 -3.434328e+01 3.991500e+00
      vertex   4.970399e+00 -3.406341e+01 3.991500e+00
    endloop
  endfacet
  facet normal 7.390089e-01 6.736956e-01 0.000000e+00
    outer loop
      vertex   4.970399e+00 -3.406341e+01 8.100000e+00
      vertex   4.970399e+00 -3.406341e+01 3.991500e+00
      vertex   4.547404e+00 -3.359941e+01 8.100000e+00
    endloop
  endfacet
  facet normal 7.390089e-01 6.736956e-01 0.000000e+00
    outer loop
      vertex   4.547404e+00 -3.359941e+01 8.100000e+00
      vertex   4.970399e+00 -3.406341e+01 3.991500e+00
      vertex   4.547404e+00 -3.359941e+01 3.991500e+00
    endloop
  endfacet
  facet normal 9.324722e-01 3.612417e-01 0.000000e+00
    outer loop
      vertex   4.547404e+00 -3.359941e+01 8.100000e+00
      vertex   4.547404e+00 -3.359941e+01 3.991500e+00
      vertex   4.320590e+00 -3.301393e+01 8.100000e+00
    endloop
  endfacet
  facet normal 9.324722e-01 3.612417e-01 0.000000e+00
    outer loop
      vertex   4.320590e+00 -3.301393e+01 8.100000e+00
      vertex   4.547404e+00 -3.359941e+01 3.991500e+00
      vertex   4.320590e+00 -3.301393e+01 3.991500e+00
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   4.320590e+00 -3.301393e+01 8.100000e+00
      vertex   4.320590e+00 -3.301393e+01 3.991500e+00
      vertex   4.320590e+00 -3.238606e+01 8.100000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   4.320590e+00 -3.238606e+01 8.100000e+00
      vertex   4.320590e+00 -3.301393e+01 3.991500e+00
      vertex   4.320590e+00 -3.238606e+01 3.991500e+00
    endloop
  endfacet
  facet normal 9.324722e-01 -3.612417e-01 0.000000e+00
    outer loop
      vertex   4.320590e+00 -3.238606e+01 8.100000e+00
      vertex   4.320590e+00 -3.238606e+01 3.991500e+00
      vertex   4.547404e+00 -3.180059e+01 8.100000e+00
    endloop
  endfacet
  facet normal 9.324722e-01 -3.612417e-01 0.000000e+00
    outer loop
      vertex   4.547404e+00 -3.180059e+01 8.100000e+00
      vertex   4.320590e+00 -3.238606e+01 3.991500e+00
      vertex   4.547404e+00 -3.180059e+01 3.991500e+00
    endloop
  endfacet
  facet normal 7.390089e-01 -6.736956e-01 0.000000e+00
    outer loop
      vertex   4.547404e+00 -3.180059e+01 8.100000e+00
      vertex   4.547404e+00 -3.180059e+01 3.991500e+00
      vertex   4.970399e+00 -3.133659e+01 8.100000e+00
    endloop
  endfacet
  facet normal 7.390089e-01 -6.736956e-01 0.000000e+00
    outer loop
      vertex   4.970399e+00 -3.133659e+01 8.100000e+00
      vertex   4.547404e+00 -3.180059e+01 3.991500e+00
      vertex   4.970399e+00 -3.133659e+01 3.991500e+00
    endloop
  endfacet
  facet normal 4.457384e-01 -8.951633e-01 0.000000e+00
    outer loop
      vertex   4.970399e+00 -3.133659e+01 8.100000e+00
      vertex   4.970399e+00 -3.133659e+01 3.991500e+00
      vertex   5.532447e+00 -3.105672e+01 8.100000e+00
    endloop
  endfacet
  facet normal 4.457384e-01 -8.951633e-01 0.000000e+00
    outer loop
      vertex   5.532447e+00 -3.105672e+01 8.100000e+00
      vertex   4.970399e+00 -3.133659e+01 3.991500e+00
      vertex   5.532447e+00 -3.105672e+01 3.991500e+00
    endloop
  endfacet
  facet normal 9.226836e-02 -9.957342e-01 0.000000e+00
    outer loop
      vertex   5.532447e+00 -3.105672e+01 8.100000e+00
      vertex   5.532447e+00 -3.105672e+01 3.991500e+00
      vertex   6.157640e+00 -3.099879e+01 8.100000e+00
    endloop
  endfacet
  facet normal 9.226836e-02 -9.957342e-01 0.000000e+00
    outer loop
      vertex   6.157640e+00 -3.099879e+01 8.100000e+00
      vertex   5.532447e+00 -3.105672e+01 3.991500e+00
      vertex   6.157640e+00 -3.099879e+01 3.991500e+00
    endloop
  endfacet
  facet normal -2.736630e-01 -9.618256e-01 -0.000000e+00
    outer loop
      vertex   6.157640e+00 -3.099879e+01 8.100000e+00
      vertex   6.157640e+00 -3.099879e+01 3.991500e+00
      vertex   6.761544e+00 -3.117061e+01 8.100000e+00
    endloop
  endfacet
  facet normal -2.736630e-01 -9.618256e-01 -0.000000e+00
    outer loop
      vertex   6.761544e+00 -3.117061e+01 8.100000e+00
      vertex   6.157640e+00 -3.099879e+01 3.991500e+00
      vertex   6.761544e+00 -3.117061e+01 3.991500e+00
    endloop
  endfacet
  facet normal -6.026346e-01 -7.980172e-01 -0.000000e+00
    outer loop
      vertex   6.761544e+00 -3.117061e+01 8.100000e+00
      vertex   6.761544e+00 -3.117061e+01 3.991500e+00
      vertex   7.262597e+00 -3.154899e+01 8.100000e+00
    endloop
  endfacet
  facet normal -6.026346e-01 -7.980172e-01 -0.000000e+00
    outer loop
      vertex   7.262597e+00 -3.154899e+01 8.100000e+00
      vertex   6.761544e+00 -3.117061e+01 3.991500e+00
      vertex   7.262597e+00 -3.154899e+01 3.991500e+00
    endloop
  endfacet
  facet normal -8.502171e-01 -5.264322e-01 -0.000000e+00
    outer loop
      vertex   7.262597e+00 -3.154899e+01 8.100000e+00
      vertex   7.262597e+00 -3.154899e+01 3.991500e+00
      vertex   7.593129e+00 -3.208282e+01 8.100000e+00
    endloop
  endfacet
  facet normal -8.502171e-01 -5.264322e-01 -0.000000e+00
    outer loop
      vertex   7.593129e+00 -3.208282e+01 8.100000e+00
      vertex   7.262597e+00 -3.154899e+01 3.991500e+00
      vertex   7.593129e+00 -3.208282e+01 3.991500e+00
    endloop
  endfacet
  facet normal 3.849221e-15 -7.195441e-16 1.000000e+00
    outer loop
      vertex   7.593129e+00 -3.331718e+01 3.991500e+00
      vertex   7.708500e+00 -3.270000e+01 3.991500e+00
      vertex   7.262597e+00 -3.385101e+01 3.991500e+00
    endloop
  endfacet
  facet normal 1.343559e-15 2.511546e-16 1.000000e+00
    outer loop
      vertex   7.262597e+00 -3.385101e+01 3.991500e+00
      vertex   7.708500e+00 -3.270000e+01 3.991500e+00
      vertex   7.593129e+00 -3.208282e+01 3.991500e+00
    endloop
  endfacet
  facet normal -4.088630e-16 5.787390e-16 1.000000e+00
    outer loop
      vertex   7.262597e+00 -3.385101e+01 3.991500e+00
      vertex   7.593129e+00 -3.208282e+01 3.991500e+00
      vertex   7.262597e+00 -3.154899e+01 3.991500e+00
    endloop
  endfacet
  facet normal -4.370434e-16 5.787390e-16 1.000000e+00
    outer loop
      vertex   7.262597e+00 -3.385101e+01 3.991500e+00
      vertex   7.262597e+00 -3.154899e+01 3.991500e+00
      vertex   6.761544e+00 -3.422939e+01 3.991500e+00
    endloop
  endfacet
  facet normal 3.289168e-16 4.355562e-16 1.000000e+00
    outer loop
      vertex   6.761544e+00 -3.422939e+01 3.991500e+00
      vertex   7.262597e+00 -3.154899e+01 3.991500e+00
      vertex   6.761544e+00 -3.117061e+01 3.991500e+00
    endloop
  endfacet
  facet normal -1.239264e-16 4.355562e-16 1.000000e+00
    outer loop
      vertex   6.761544e+00 -3.422939e+01 3.991500e+00
      vertex   6.761544e+00 -3.117061e+01 3.991500e+00
      vertex   6.157640e+00 -3.440121e+01 3.991500e+00
    endloop
  endfacet
  facet normal 1.114096e-16 3.915643e-16 1.000000e+00
    outer loop
      vertex   6.157640e+00 -3.440121e+01 3.991500e+00
      vertex   6.761544e+00 -3.117061e+01 3.991500e+00
      vertex   6.157640e+00 -3.099879e+01 3.991500e+00
    endloop
  endfacet
  facet normal 3.628378e-17 3.915643e-16 1.000000e+00
    outer loop
      vertex   6.157640e+00 -3.440121e+01 3.991500e+00
      vertex   6.157640e+00 -3.099879e+01 3.991500e+00
      vertex   5.532447e+00 -3.434328e+01 3.991500e+00
    endloop
  endfacet
  facet normal -3.756294e-17 4.053686e-16 1.000000e+00
    outer loop
      vertex   5.532447e+00 -3.434328e+01 3.991500e+00
      vertex   6.157640e+00 -3.099879e+01 3.991500e+00
      vertex   5.532447e+00 -3.105672e+01 3.991500e+00
    endloop
  endfacet
  facet normal 2.018496e-16 4.053686e-16 1.000000e+00
    outer loop
      vertex   5.532447e+00 -3.434328e+01 3.991500e+00
      vertex   5.532447e+00 -3.105672e+01 3.991500e+00
      vertex   4.970399e+00 -3.406341e+01 3.991500e+00
    endloop
  endfacet
  facet normal -2.432831e-16 4.885784e-16 1.000000e+00
    outer loop
      vertex   4.970399e+00 -3.406341e+01 3.991500e+00
      vertex   5.532447e+00 -3.105672e+01 3.991500e+00
      vertex   4.970399e+00 -3.133659e+01 3.991500e+00
    endloop
  endfacet
  facet normal 5.359449e-16 4.885784e-16 1.000000e+00
    outer loop
      vertex   4.970399e+00 -3.406341e+01 3.991500e+00
      vertex   4.970399e+00 -3.133659e+01 3.991500e+00
      vertex   4.547404e+00 -3.359941e+01 3.991500e+00
    endloop
  endfacet
  facet normal -8.124376e-16 7.406347e-16 1.000000e+00
    outer loop
      vertex   4.547404e+00 -3.359941e+01 3.991500e+00
      vertex   4.970399e+00 -3.133659e+01 3.991500e+00
      vertex   4.547404e+00 -3.180059e+01 3.991500e+00
    endloop
  endfacet
  facet normal -2.004098e-15 7.406347e-16 1.000000e+00
    outer loop
      vertex   4.547404e+00 -3.359941e+01 3.991500e+00
      vertex   4.547404e+00 -3.180059e+01 3.991500e+00
      vertex   4.320590e+00 -3.301393e+01 3.991500e+00
    endloop
  endfacet
  facet normal 1.957948e-15 -0.000000e+00 1.000000e+00
    outer loop
      vertex   4.320590e+00 -3.301393e+01 3.991500e+00
      vertex   4.547404e+00 -3.180059e+01 3.991500e+00
      vertex   4.320590e+00 -3.238606e+01 3.991500e+00
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 -0.000000e+00
    outer loop
      vertex   1.750957e+01 -3.249194e+01 3.000000e+00
      vertex   1.750957e+01 -3.249194e+01 3.814698e-07
      vertex   1.763350e+01 -3.307500e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 -2.079117e-01 -0.000000e+00
    outer loop
      vertex   1.763350e+01 -3.307500e+01 3.000000e+00
      vertex   1.750957e+01 -3.249194e+01 3.814698e-07
      vertex   1.763350e+01 -3.307500e+01 3.814698e-07
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 0.000000e+00
    outer loop
      vertex   1.763350e+01 -3.307500e+01 3.000000e+00
      vertex   1.763350e+01 -3.307500e+01 3.814698e-07
      vertex   1.750957e+01 -3.365806e+01 3.000000e+00
    endloop
  endfacet
  facet normal -9.781476e-01 2.079117e-01 0.000000e+00
    outer loop
      vertex   1.750957e+01 -3.365806e+01 3.000000e+00
      vertex   1.763350e+01 -3.307500e+01 3.814698e-07
      vertex   1.750957e+01 -3.365806e+01 3.814698e-07
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 0.000000e+00
    outer loop
      vertex   1.750957e+01 -3.365806e+01 3.000000e+00
      vertex   1.750957e+01 -3.365806e+01 3.814698e-07
      vertex   1.715920e+01 -3.414030e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 5.877853e-01 0.000000e+00
    outer loop
      vertex   1.715920e+01 -3.414030e+01 3.000000e+00
      vertex   1.750957e+01 -3.365806e+01 3.814698e-07
      vertex   1.715920e+01 -3.414030e+01 3.814698e-07
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 0.000000e+00
    outer loop
      vertex   1.715920e+01 -3.414030e+01 3.000000e+00
      vertex   1.715920e+01 -3.414030e+01 3.814698e-07
      vertex   1.664298e+01 -3.443834e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 8.660254e-01 0.000000e+00
    outer loop
      vertex   1.664298e+01 -3.443834e+01 3.000000e+00
      vertex   1.715920e+01 -3.414030e+01 3.814698e-07
      vertex   1.664298e+01 -3.443834e+01 3.814698e-07
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 0.000000e+00
    outer loop
      vertex   1.664298e+01 -3.443834e+01 3.000000e+00
      vertex   1.664298e+01 -3.443834e+01 3.814698e-07
      vertex   1.605016e+01 -3.450065e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 9.945219e-01 0.000000e+00
    outer loop
      vertex   1.605016e+01 -3.450065e+01 3.000000e+00
      vertex   1.664298e+01 -3.443834e+01 3.814698e-07
      vertex   1.605016e+01 -3.450065e+01 3.814698e-07
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 0.000000e+00
    outer loop
      vertex   1.605016e+01 -3.450065e+01 3.000000e+00
      vertex   1.605016e+01 -3.450065e+01 3.814698e-07
      vertex   1.548325e+01 -3.431645e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 9.510565e-01 0.000000e+00
    outer loop
      vertex   1.548325e+01 -3.431645e+01 3.000000e+00
      vertex   1.605016e+01 -3.450065e+01 3.814698e-07
      vertex   1.548325e+01 -3.431645e+01 3.814698e-07
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 0.000000e+00
    outer loop
      vertex   1.548325e+01 -3.431645e+01 3.000000e+00
      vertex   1.548325e+01 -3.431645e+01 3.814698e-07
      vertex   1.504027e+01 -3.391759e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 7.431448e-01 0.000000e+00
    outer loop
      vertex   1.504027e+01 -3.391759e+01 3.000000e+00
      vertex   1.548325e+01 -3.431645e+01 3.814698e-07
      vertex   1.504027e+01 -3.391759e+01 3.814698e-07
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 0.000000e+00
    outer loop
      vertex   1.504027e+01 -3.391759e+01 3.000000e+00
      vertex   1.504027e+01 -3.391759e+01 3.814698e-07
      vertex   1.479783e+01 -3.337304e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 4.067366e-01 0.000000e+00
    outer loop
      vertex   1.479783e+01 -3.337304e+01 3.000000e+00
      vertex   1.504027e+01 -3.391759e+01 3.814698e-07
      vertex   1.479783e+01 -3.337304e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   1.479783e+01 -3.337304e+01 3.000000e+00
      vertex   1.479783e+01 -3.337304e+01 3.814698e-07
      vertex   1.479783e+01 -3.277696e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   1.479783e+01 -3.277696e+01 3.000000e+00
      vertex   1.479783e+01 -3.337304e+01 3.814698e-07
      vertex   1.479783e+01 -3.277696e+01 3.814698e-07
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 0.000000e+00
    outer loop
      vertex   1.479783e+01 -3.277696e+01 3.000000e+00
      vertex   1.479783e+01 -3.277696e+01 3.814698e-07
      vertex   1.504027e+01 -3.223241e+01 3.000000e+00
    endloop
  endfacet
  facet normal 9.135455e-01 -4.067366e-01 0.000000e+00
    outer loop
      vertex   1.504027e+01 -3.223241e+01 3.000000e+00
      vertex   1.479783e+01 -3.277696e+01 3.814698e-07
      vertex   1.504027e+01 -3.223241e+01 3.814698e-07
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 0.000000e+00
    outer loop
      vertex   1.504027e+01 -3.223241e+01 3.000000e+00
      vertex   1.504027e+01 -3.223241e+01 3.814698e-07
      vertex   1.548325e+01 -3.183355e+01 3.000000e+00
    endloop
  endfacet
  facet normal 6.691306e-01 -7.431448e-01 0.000000e+00
    outer loop
      vertex   1.548325e+01 -3.183355e+01 3.000000e+00
      vertex   1.504027e+01 -3.223241e+01 3.814698e-07
      vertex   1.548325e+01 -3.183355e+01 3.814698e-07
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 0.000000e+00
    outer loop
      vertex   1.548325e+01 -3.183355e+01 3.000000e+00
      vertex   1.548325e+01 -3.183355e+01 3.814698e-07
      vertex   1.605016e+01 -3.164935e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.090170e-01 -9.510565e-01 0.000000e+00
    outer loop
      vertex   1.605016e+01 -3.164935e+01 3.000000e+00
      vertex   1.548325e+01 -3.183355e+01 3.814698e-07
      vertex   1.605016e+01 -3.164935e+01 3.814698e-07
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 -0.000000e+00
    outer loop
      vertex   1.605016e+01 -3.164935e+01 3.000000e+00
      vertex   1.605016e+01 -3.164935e+01 3.814698e-07
      vertex   1.664298e+01 -3.171166e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.045285e-01 -9.945219e-01 -0.000000e+00
    outer loop
      vertex   1.664298e+01 -3.171166e+01 3.000000e+00
      vertex   1.605016e+01 -3.164935e+01 3.814698e-07
      vertex   1.664298e+01 -3.171166e+01 3.814698e-07
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 -0.000000e+00
    outer loop
      vertex   1.664298e+01 -3.171166e+01 3.000000e+00
      vertex   1.664298e+01 -3.171166e+01 3.814698e-07
      vertex   1.715920e+01 -3.200970e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.000000e-01 -8.660254e-01 -0.000000e+00
    outer loop
      vertex   1.715920e+01 -3.200970e+01 3.000000e+00
      vertex   1.664298e+01 -3.171166e+01 3.814698e-07
      vertex   1.715920e+01 -3.200970e+01 3.814698e-07
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 -0.000000e+00
    outer loop
      vertex   1.715920e+01 -3.200970e+01 3.000000e+00
      vertex   1.715920e+01 -3.200970e+01 3.814698e-07
      vertex   1.750957e+01 -3.249194e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.090170e-01 -5.877853e-01 -0.000000e+00
    outer loop
      vertex   1.750957e+01 -3.249194e+01 3.000000e+00
      vertex   1.715920e+01 -3.200970e+01 3.814698e-07
      vertex   1.750957e+01 -3.249194e+01 3.814698e-07
    endloop
  endfacet
  facet normal -4.915240e-16 1.044767e-16 1.000000e+00
    outer loop
      vertex   -2.890433e+00 -3.365806e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -2.766500e+00 -3.307500e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.380053e-16 2.220446e-16 1.000000e+00
    outer loop
      vertex   -2.766500e+00 -3.307500e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.739652e-16 -5.823309e-17 1.000000e+00
    outer loop
      vertex   -2.766500e+00 -3.307500e+01 3.000000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -2.890433e+00 -3.249194e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.243187e-16 -1.629771e-16 1.000000e+00
    outer loop
      vertex   -2.890433e+00 -3.249194e+01 3.000000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -3.240801e+00 -3.200970e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.734499e-16 -3.004241e-16 1.000000e+00
    outer loop
      vertex   -3.240801e+00 -3.200970e+01 3.000000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -3.757024e+00 -3.171166e+01 3.000000e+00
    endloop
  endfacet
  facet normal -6.859396e-17 -6.526279e-16 1.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.171166e+01 3.000000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -4.349842e+00 -3.164935e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.960595e-16 2.050082e-16 1.000000e+00
    outer loop
      vertex   -4.349842e+00 -3.164935e+01 3.000000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.667996e-16 5.133562e-16 1.000000e+00
    outer loop
      vertex   -4.349842e+00 -3.164935e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -4.916750e+00 -3.183355e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.752846e-16 3.057346e-16 1.000000e+00
    outer loop
      vertex   -4.916750e+00 -3.183355e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -5.359726e+00 -3.223241e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.725756e-16 1.658813e-16 1.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.223241e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -5.602175e+00 -3.277696e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.217160e-16 2.220446e-16 1.000000e+00
    outer loop
      vertex   -5.602175e+00 -3.277696e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.306981e-16 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.602175e+00 -3.277696e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -5.602175e+00 -3.337304e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.001587e-16 -4.459354e-17 1.000000e+00
    outer loop
      vertex   -5.602175e+00 -3.337304e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -5.359726e+00 -3.391759e+01 3.000000e+00
    endloop
  endfacet
  facet normal -7.902167e-17 -8.776246e-17 1.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.391759e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -4.916750e+00 -3.431645e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.323925e-17 -1.638536e-16 1.000000e+00
    outer loop
      vertex   -4.916750e+00 -3.431645e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -4.349842e+00 -3.450065e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.960595e-16 7.776146e-16 1.000000e+00
    outer loop
      vertex   -4.349842e+00 -3.450065e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.656968e-16 1.576500e-15 1.000000e+00
    outer loop
      vertex   -4.349842e+00 -3.450065e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -3.757024e+00 -3.443834e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.476320e-16 6.021163e-16 1.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.443834e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -3.240801e+00 -3.414030e+01 3.000000e+00
    endloop
  endfacet
  facet normal -4.215661e-16 3.062857e-16 1.000000e+00
    outer loop
      vertex   -3.240801e+00 -3.414030e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -2.890433e+00 -3.365806e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.263186e-16 4.810550e-17 1.000000e+00
    outer loop
      vertex   1.750957e+01 -3.365806e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
      vertex   1.763350e+01 -3.307500e+01 3.000000e+00
    endloop
  endfacet
  facet normal -2.061160e-16 7.401487e-17 1.000000e+00
    outer loop
      vertex   1.763350e+01 -3.307500e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.087420e-16 -2.311383e-17 1.000000e+00
    outer loop
      vertex   1.763350e+01 -3.307500e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   1.750957e+01 -3.249194e+01 3.000000e+00
    endloop
  endfacet
  facet normal -8.046701e-17 -5.846270e-17 1.000000e+00
    outer loop
      vertex   1.750957e+01 -3.249194e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   1.715920e+01 -3.200970e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.663448e-17 -9.809380e-17 1.000000e+00
    outer loop
      vertex   1.715920e+01 -3.200970e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   1.664298e+01 -3.171166e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.890007e-17 -1.798222e-16 1.000000e+00
    outer loop
      vertex   1.664298e+01 -3.171166e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   1.605016e+01 -3.164935e+01 3.000000e+00
    endloop
  endfacet
  facet normal -3.700743e-17 -1.338435e-16 1.000000e+00
    outer loop
      vertex   1.605016e+01 -3.164935e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.605016e+01 -3.164935e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   1.548325e+01 -3.183355e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.548325e+01 -3.183355e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   1.504027e+01 -3.223241e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.504027e+01 -3.223241e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   1.479783e+01 -3.277696e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.178411e-16 2.220446e-16 1.000000e+00
    outer loop
      vertex   1.479783e+01 -3.277696e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 2.297875e-16 0.000000e+00 1.000000e+00
    outer loop
      vertex   1.479783e+01 -3.277696e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   1.479783e+01 -3.337304e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.949511e-16 8.679784e-17 1.000000e+00
    outer loop
      vertex   1.479783e+01 -3.337304e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   1.504027e+01 -3.391759e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.661215e-16 1.844966e-16 1.000000e+00
    outer loop
      vertex   1.504027e+01 -3.391759e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   1.548325e+01 -3.431645e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.240312e-16 3.817289e-16 1.000000e+00
    outer loop
      vertex   1.548325e+01 -3.431645e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   1.605016e+01 -3.450065e+01 3.000000e+00
    endloop
  endfacet
  facet normal 3.700743e-17 8.932812e-16 1.000000e+00
    outer loop
      vertex   1.605016e+01 -3.450065e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -5.428658e-17 5.165023e-16 1.000000e+00
    outer loop
      vertex   1.605016e+01 -3.450065e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
      vertex   1.664298e+01 -3.443834e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.357597e-16 2.351427e-16 1.000000e+00
    outer loop
      vertex   1.664298e+01 -3.443834e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
      vertex   1.715920e+01 -3.414030e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.785645e-16 1.297347e-16 1.000000e+00
    outer loop
      vertex   1.715920e+01 -3.414030e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
      vertex   1.750957e+01 -3.365806e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.813672e-16 -1.801441e-16 -1.000000e+00
    outer loop
      vertex   -2.890433e+00 -3.249194e+01 3.814698e-07
      vertex   1.479783e+01 -3.277696e+01 3.814698e-07
      vertex   -2.766500e+00 -3.307500e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.815586e-16 -1.914285e-16 -1.000000e+00
    outer loop
      vertex   -2.766500e+00 -3.307500e+01 3.814698e-07
      vertex   1.479783e+01 -3.277696e+01 3.814698e-07
      vertex   1.479783e+01 -3.337304e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.813672e-16 -2.027127e-16 -1.000000e+00
    outer loop
      vertex   -2.766500e+00 -3.307500e+01 3.814698e-07
      vertex   1.479783e+01 -3.337304e+01 3.814698e-07
      vertex   -2.890433e+00 -3.365806e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.808085e-16 -1.680410e-16 -1.000000e+00
    outer loop
      vertex   -2.890433e+00 -3.365806e+01 3.814698e-07
      vertex   1.479783e+01 -3.337304e+01 3.814698e-07
      vertex   1.504027e+01 -3.391759e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.799268e-16 -2.289525e-16 -1.000000e+00
    outer loop
      vertex   -2.890433e+00 -3.365806e+01 3.814698e-07
      vertex   1.504027e+01 -3.391759e+01 3.814698e-07
      vertex   -3.240801e+00 -3.414030e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.787864e-16 -1.353346e-16 -1.000000e+00
    outer loop
      vertex   -3.240801e+00 -3.414030e+01 3.814698e-07
      vertex   1.504027e+01 -3.391759e+01 3.814698e-07
      vertex   1.548325e+01 -3.431645e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.774573e-16 -2.766070e-16 -1.000000e+00
    outer loop
      vertex   -3.240801e+00 -3.414030e+01 3.814698e-07
      vertex   1.548325e+01 -3.431645e+01 3.814698e-07
      vertex   -3.757024e+00 -3.443834e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.759875e-16 -4.459802e-17 -1.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.443834e+01 3.814698e-07
      vertex   1.548325e+01 -3.431645e+01 3.814698e-07
      vertex   1.605016e+01 -3.450065e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.735443e-16 -8.212756e-16 -1.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.443834e+01 3.814698e-07
      vertex   1.605016e+01 -3.450065e+01 3.814698e-07
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.184238e-16 3.299927e-16 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
      vertex   1.605016e+01 -3.450065e+01 3.814698e-07
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 8.707441e-17 2.006108e-16 -1.000000e+00
    outer loop
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
      vertex   1.605016e+01 -3.450065e+01 3.814698e-07
      vertex   1.664298e+01 -3.443834e+01 3.814698e-07
    endloop
  endfacet
  facet normal 2.523412e-17 -1.294872e-17 -1.000000e+00
    outer loop
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
      vertex   1.664298e+01 -3.443834e+01 3.814698e-07
      vertex   1.715920e+01 -3.414030e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.808085e-16 -2.148158e-16 -1.000000e+00
    outer loop
      vertex   1.479783e+01 -3.277696e+01 3.814698e-07
      vertex   -2.890433e+00 -3.249194e+01 3.814698e-07
      vertex   1.504027e+01 -3.223241e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.799268e-16 -1.539044e-16 -1.000000e+00
    outer loop
      vertex   1.504027e+01 -3.223241e+01 3.814698e-07
      vertex   -2.890433e+00 -3.249194e+01 3.814698e-07
      vertex   -3.240801e+00 -3.200970e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.787864e-16 -2.475222e-16 -1.000000e+00
    outer loop
      vertex   1.504027e+01 -3.223241e+01 3.814698e-07
      vertex   -3.240801e+00 -3.200970e+01 3.814698e-07
      vertex   1.548325e+01 -3.183355e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.774573e-16 -1.062500e-16 -1.000000e+00
    outer loop
      vertex   1.548325e+01 -3.183355e+01 3.814698e-07
      vertex   -3.240801e+00 -3.200970e+01 3.814698e-07
      vertex   -3.757024e+00 -3.171166e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.759875e-16 -3.382588e-16 -1.000000e+00
    outer loop
      vertex   1.548325e+01 -3.183355e+01 3.814698e-07
      vertex   -3.757024e+00 -3.171166e+01 3.814698e-07
      vertex   1.605016e+01 -3.164935e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.738731e-16 3.338935e-16 -1.000000e+00
    outer loop
      vertex   1.605016e+01 -3.164935e+01 3.814698e-07
      vertex   -3.757024e+00 -3.171166e+01 3.814698e-07
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.184238e-16 -3.786576e-16 -1.000000e+00
    outer loop
      vertex   1.605016e+01 -3.164935e+01 3.814698e-07
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.871221e-16 3.684243e-16 -1.000000e+00
    outer loop
      vertex   -3.757024e+00 -3.171166e+01 3.814698e-07
      vertex   -4.349842e+00 -3.164935e+01 3.814698e-07
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 5.149540e-17 4.488846e-17 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
      vertex   -4.349842e+00 -3.164935e+01 3.814698e-07
      vertex   -4.916750e+00 -3.183355e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.555710e-18 -5.068746e-17 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
      vertex   -4.916750e+00 -3.183355e+01 3.814698e-07
      vertex   -5.359726e+00 -3.223241e+01 3.814698e-07
    endloop
  endfacet
  facet normal -4.323087e-17 -1.150670e-16 -1.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.223241e+01 3.814698e-07
      vertex   -5.602175e+00 -3.277696e+01 3.814698e-07
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
    endloop
  endfacet
  facet normal -7.308082e-17 -1.480297e-16 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
      vertex   -5.602175e+00 -3.277696e+01 3.814698e-07
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal -3.574615e-17 -1.914286e-16 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
      vertex   -5.602175e+00 -3.277696e+01 3.814698e-07
      vertex   -5.602175e+00 -3.337304e+01 3.814698e-07
    endloop
  endfacet
  facet normal 2.580480e-18 -2.473932e-16 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
      vertex   -5.602175e+00 -3.337304e+01 3.814698e-07
      vertex   -5.359726e+00 -3.391759e+01 3.814698e-07
    endloop
  endfacet
  facet normal 2.910745e-17 -3.015700e-16 -1.000000e+00
    outer loop
      vertex   -5.359726e+00 -3.391759e+01 3.814698e-07
      vertex   -4.916750e+00 -3.431645e+01 3.814698e-07
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 6.146425e-17 -3.970640e-16 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
      vertex   -4.916750e+00 -3.431645e+01 3.814698e-07
      vertex   -4.349842e+00 -3.450065e+01 3.814698e-07
    endloop
  endfacet
  facet normal 2.107135e-16 -9.757367e-16 -1.000000e+00
    outer loop
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
      vertex   -4.349842e+00 -3.450065e+01 3.814698e-07
      vertex   -3.757024e+00 -3.443834e+01 3.814698e-07
    endloop
  endfacet
  facet normal 2.696404e-17 -2.129636e-16 -1.000000e+00
    outer loop
      vertex   1.750957e+01 -3.249194e+01 3.814698e-07
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
      vertex   1.763350e+01 -3.307500e+01 3.814698e-07
    endloop
  endfacet
  facet normal -3.813380e-17 -1.480297e-16 -1.000000e+00
    outer loop
      vertex   1.763350e+01 -3.307500e+01 3.814698e-07
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal -4.350249e-17 -1.549150e-16 -1.000000e+00
    outer loop
      vertex   1.763350e+01 -3.307500e+01 3.814698e-07
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
      vertex   1.750957e+01 -3.365806e+01 3.814698e-07
    endloop
  endfacet
  facet normal -7.255978e-18 -9.295632e-17 -1.000000e+00
    outer loop
      vertex   1.750957e+01 -3.365806e+01 3.814698e-07
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
      vertex   1.715920e+01 -3.414030e+01 3.814698e-07
    endloop
  endfacet
  facet normal 5.330804e-17 -2.458984e-16 -1.000000e+00
    outer loop
      vertex   1.750957e+01 -3.249194e+01 3.814698e-07
      vertex   1.715920e+01 -3.200970e+01 3.814698e-07
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 7.551293e-17 -2.828229e-16 -1.000000e+00
    outer loop
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
      vertex   1.715920e+01 -3.200970e+01 3.814698e-07
      vertex   1.664298e+01 -3.171166e+01 3.814698e-07
    endloop
  endfacet
  facet normal 1.106700e-16 -3.589691e-16 -1.000000e+00
    outer loop
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
      vertex   1.664298e+01 -3.171166e+01 3.814698e-07
      vertex   1.605016e+01 -3.164935e+01 3.814698e-07
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 -0.000000e+00
    outer loop
      vertex   -9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   -9.000000e+00 -3.570000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   2.100000e+01 -3.570000e+01 3.814698e-07
      vertex   2.100000e+01 -3.570000e+01 3.000000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
      vertex   -9.000000e+00 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.100000e+01 -2.970000e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   2.100000e+01 -2.970000e+01 3.814698e-07
      vertex   -9.000000e+00 -2.970000e+01 3.814698e-07
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   7.593129e+00 -3.331718e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
      vertex   7.708500e+00 -3.270000e+01 8.100000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   7.708500e+00 -3.270000e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   7.708500e+00 -3.270000e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   7.593129e+00 -3.208282e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   7.593129e+00 -3.208282e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   7.262597e+00 -3.154899e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   7.262597e+00 -3.154899e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   6.761544e+00 -3.117061e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   6.761544e+00 -3.117061e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   6.157640e+00 -3.099879e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   6.157640e+00 -3.099879e+01 8.100000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   6.157640e+00 -3.099879e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   5.532447e+00 -3.105672e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   5.532447e+00 -3.105672e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   4.970399e+00 -3.133659e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   4.970399e+00 -3.133659e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   4.547404e+00 -3.180059e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   4.547404e+00 -3.180059e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   4.320590e+00 -3.238606e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   4.320590e+00 -3.238606e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   4.320590e+00 -3.238606e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   4.320590e+00 -3.301393e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   4.320590e+00 -3.301393e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   4.547404e+00 -3.359941e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   4.547404e+00 -3.359941e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   4.970399e+00 -3.406341e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   4.970399e+00 -3.406341e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   5.532447e+00 -3.434328e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   5.532447e+00 -3.434328e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   6.157640e+00 -3.440121e+01 8.100000e+00
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   6.157640e+00 -3.440121e+01 8.100000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   6.157640e+00 -3.440121e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
      vertex   6.761544e+00 -3.422939e+01 8.100000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   6.761544e+00 -3.422939e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
      vertex   7.262597e+00 -3.385101e+01 8.100000e+00
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   7.262597e+00 -3.385101e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
      vertex   7.593129e+00 -3.331718e+01 8.100000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   9.000000e+00 -2.970000e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   9.000000e+00 -3.570000e+01 3.000000e+00
      vertex   9.000000e+00 -2.970000e+01 8.100000e+00
      vertex   9.000000e+00 -3.570000e+01 8.100000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 -8.707632e-17
    outer loop
      vertex   3.000000e+00 -2.970000e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
    endloop
  endfacet
  facet normal -1.000000e+00 -1.288988e-32 -8.707632e-17
    outer loop
      vertex   3.000000e+00 -3.570000e+01 8.100000e+00
      vertex   3.000000e+00 -2.970000e+01 3.000000e+00
      vertex   3.000000e+00 -3.570000e+01 3.000000e+00
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