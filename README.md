# RC Car Project
This project is a custom-built remote-controlled (RC) car, designed from the ground up with custom electronics, PCB design, and embedded software for both the car and its remote. The project demonstrates:

<div align="center">
	<img src="images/completed-car.JPEG" width="500" alt="Completed RC Car Side Profile">
</div>

## Table of Contents
1. [Software Used](#software)
2. [Drive and Steering Control Circuit](#driveandsteeringcircuit)
    - [Steering Servo Control](#steering)
    - [Drive Motor Control](#drivemotor)
    - [RF Receiver](#rfreceiver)
    - [Power Consumption](#powerconsumption)
    - [Parts List](#carpartslist)
3. [Remote Control Circuit](#remotecontrolcircuit)
    - [Control Input Processing](#controlinputprocessing)
        - [Throttle Input (Left Analog Stick)](#throttleprocessing)
        - [Steering Input (Right Analog Stick)](#steeringprocessing)
    - [RF Transmitter](#rftransmitter)
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

### Steering Servo Control<a name="steering"></a>
Steering is driven by an SG90 servo using Timer1 hardware PWM on OC1B (PD4). The car firmware configures Timer1 in Fast PWM mode 14 with ICR1 as TOP, prescaler = 8, and ICR1 = 20000. With an 8 MHz clock, that gives a 1 us timer tick and about a 50 Hz control period (20 ms), which matches standard hobby servo timing.

The commanded steering pulse is written directly to OCR1B. In this project, steering values are transmitted as absolute pulse widths in microseconds, centered at 1780 us. On the controller side, joystick input is mapped and clamped to approximately 1400 us to 2160 us (with a center trim offset), then sent over RF. On the car side, OCR1B is updated from the payload each loop:

- Neutral: ~1780 us
- Left/Right command range: ~1400 us to ~2160 us
- PWM period: ~20 ms (50 Hz)

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

That delay prevents hard instant reversals and reduces shoot-through/braking stress during forward/reverse transitions.

Additional control logic from the car firmware:

- Speed deadband: commands between -15 and +15 stop the motor to avoid joystick/noise creep
- Direction is only changed when needed (or from stopped state)
- A watchdog is enabled after first RF packet and reset on each valid packet; if RF traffic is lost, the MCU resets after the watchdog timeout (2 s)

### Why RF reception is harder with this PWM approach
Using Timer2 compare interrupts for every PWM cycle means the MCU is servicing frequent interrupts continuously while also polling and reading nRF24 payloads over SPI. That interrupt load can steal time from RF handling and make robust payload reception harder than a pure hardware-PWM motor path.

In practice, the current tuning (about 5 kHz PWM, deadband around zero, staged direction changes) was a workable sweet spot between motor smoothness and reliable radio updates. A strictly hardware PWM implementation for both drive channels would reduce ISR overhead and leave more CPU time for RF processing.

### RF Receiver<a name="rfreceiver"></a>

### Power Consumption<a name="powerconsumption"></a>
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
|<a href="https://www.amazon.com/Micro-Servos-Helicopter-Airplane-Controls/dp/B07MLR1498/ref=sr_1_6?crid=2U7YPHQQPWA72&dib=eyJ2IjoiMSJ9.DO_8huDXG-WCdEl_xxmMGOc4m-SOLZHCxXbtX_tdiH3QEGU5P18WICxQSL6xcYWhQqOPLEHUu9sa71Q64UcwL7neHYD6CUQnu9wvT2wwK4ZGkDFNOfYnxbijpqOdZKKxCmOtE4j3XZz6xOX_f63TSt9JLIiMxN3DG0seC5RyzpEwr_yxKqqjQwaiqKzfF0LLlJ5IEMt1Yvu7OU-pBllXw2NXHtA4Nk9yhzDBZ44nW3KEf250HyQT_alUX4nBnvgKgZQTpK2CvP6yfaxlUb5at3ONkOKPIqikAtiE-KERbLM._KaHWO6YfP1st1Ba5PZS305amc16ENFTPJT_9hZmXpg&dib_tag=se&keywords=sg90%2Bservo&qid=1776825044&sprefix=sg90%2Caps%2C143&sr=8-6&th=1">SG90 servo motor</a>| x1 |
|<a href="https://www.amazon.com/uxcell-Micro-11500-12000RPM-Remote-Control/dp/B07M99JK6Y/ref=sr_1_6_pp?crid=15ZR5I0IA9JJI&dib=eyJ2IjoiMSJ9.0dTLxilAb6xq-6TUjWUHu4LHY67tPVw3OW_p63iFn0nxHTF8tJOEU3O_Cb8oOwE6WxJZW0BjBXIf0G8GfpGx34Xib0qnggH2QTWBRLxwTrt1dVHrFBr4m3qlymM1dp_HGhddBT5o2u24L-f4An4mHsWxuHhcwAWkuWetTyXovK-j6vw5tc8bw8An9Jwehol9gVoWqmTKBzJJhRpDE5yd1FT2j8fYInDLiEd9pn63in9N35NYOYHjUvUYXkuJJkr2DH7aythkyAL53UUCrgou9nhxGCkFBAQQLPBe2CI7QtI.hByQYkM03NWvQAiFiiVJo0fbgudInFzwV0ohKh8QIlI&dib_tag=se&keywords=9v+motor&qid=1778356523&sprefix=9v+motor%2Caps%2C163&sr=8-6">9V DC Motor</a>| x1 |
|<a href="https://www.amazon.com/KiNSMART-Jaguar-Project-Metal-Model/dp/B0GXDP3B83/ref=sr_1_2?crid=HY25IPNV0BHN&dib=eyJ2IjoiMSJ9.kNHmPeKKto7l4mN5kOvqBU7R0Xlsnn2LHCeKZya09DTJalbcDjd4JoTcYEG4oBi7Ljllb7X6_TGZSaIGPy1Xm1sebzbI1yp2mHZeyxYMDKpjf4Jqk94GJpw4gnU1N_5tkqw1yrXIqMnX5ZyADOEFBC7UcSktNJJQAYE5tSmkBo9atjGda3yxCk-vgOEnLoZFFx2acgDjpZFOOCdXU92NyMzapczCn7FhwygMaJ_Cei5oG747OONrkMHwEvzIolQ_2xz7kVeILcwJkr2IywmRw9VX98jR5jMxBVIL_H83mhw.naunVJhW78f1ztCTSTYnX7RhDgAuPRFgKA2PJmHRRKo&dib_tag=se&keywords=kinsmart%2B1%3A38&qid=1779063547&sprefix=kinsmart%2B1%2B38%2Caps%2C216&sr=8-2&th=1">Kinsmart Jaguar 1:38</a>| x1 |
|<a href="https://www.amazon.com/HiLetgo-NRF24L01-Wireless-Transceiver-Module/dp/B00LX47OCY/ref=sr_1_1_sspa?crid=1USO9UOFDEZL5&dib=eyJ2IjoiMSJ9.HpGu4TebgrLEY6IjfmGnCKONGE1zifAy342llWfR4vcsJ4_OTj71wcfjuLFi42g9LnfOsZybnBvz4HCtPFZh7IoO0VCtoV4SHTwJkmzj3SyTmBWTWfYEwK0bZ-6KAhnJqpXuroU3ExNMIQ_0sb6zAw01BAymwhYK7jVncUvl8YxZV7HAVItE-ISceLL5caDSPRu-nl4dzw8eF-t1VvKSdHE_Pz68YolGVn5D4_TDIAE.6OIyGVxp3k88Grsb6QKkfbgzYiEAVdqjkrvRE4yyM6M&dib_tag=se&keywords=nrf24l01&qid=1778356552&sprefix=nrf%2Caps%2C232&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1">NRF24L01+ Module</a>| x1 |
|<a href="https://www.amazon.com/dp/B07MW2L96L?ref=ppx_yo2ov_dt_b_fed_asin_title">7.4V battery</a>| x1 |
|<a href="https://www.amazon.com/200PCS-Module-Plastic-Single-Spindle/dp/B0DKHG96PL/ref=sr_1_6?crid=W0RZLTTF36ZV&dib=eyJ2IjoiMSJ9.rd3Q4coQN-xK_iVME0RsSWQTjAsKta_V4h4vm12weDUJC74fnjatzucJO07yHtkLeIzfxeMC2dxmPiht58xboKdyTAKydnddtAAmDD_aARQfSeJf72BiNVGOLvJEh9z4o6wPQQIsW7IFXJZAsVFzRvb3aBc2lONKzi84gw8UxRRc2rQ9rWUQvz_yelBZoDNSvcFoSqvusfdJFjuKQ47WFcnAPOwQnc2LdEzRRlFGYalz5QPsVB-hWhAqh67OA6riZCc13JSazX8bvwV50yC13BdBVPq8QEGZLQo3CGB_GXM.rLefd6TJ3JQPyoYEO4myNas1mQIUsuXzyBHIw13Vpyw&dib_tag=se&keywords=dc%2Bmotor%2Bgears&qid=1778804461&sprefix=dc%2Bmotor%2Bgear%2Caps%2C142&sr=8-6&th=1">Gears </a>| x1 |
|<a href="https://www.amazon.com/ruthex-Threaded-Insert-pieces-ultrasound/dp/B088QJG676/ref=sr_1_3?crid=327WC4A3U357H&dib=eyJ2IjoiMSJ9.v_Af1Sv2DMXyynp1rO3uN2cjI_LkHpW2Xd3NuL9RUn1yz2Ym6BTHo47Bnvh-mF3rih3MIxPZyDCFRwN3f8bgTgfOqS-2FD8o-WWmIpuq7XJIrvCSKP6iwhtXO1FB6g2J8vFhgMjuaMVbjKPUpn3U8iuk4FPTnzt9IQcKtjUuxKiJM3PQJf3MPDf15V62fIp3_oibwlJtUKJ7oyWbckn9BPgwhhw0z9gTb7ylLCQjULA.KUBGIt58ag9IJXW2aWAnvHSlQ59y6f_kavaLpQD422E&dib_tag=se&keywords=inserts%2Bm2&qid=1778806075&sprefix=inserts%2Bm%2Caps%2C197&sr=8-3&th=1">M2 Inserts</a>| x3 |
|<a href="https://www.amazon.com/Phillips-Countersunk-Electronic-Accessories-Samsung/dp/B07HC3LQYS/ref=sr_1_8?crid=2ISFEC45EBS5Q&dib=eyJ2IjoiMSJ9.sToeJ_cHiwrPYQ_C9rq2gwq_BqFxCk_dAqNz8qbKlTKQla66SuHvAVoMEMQE3FrKbT_cXuKk3EDQL7eTiH6WYWt4xdVsIXdoV99uXBCs7qfK_HdB1wotMUmIz4MZM-fYqvCkvNTV6tpBtMOlWfMOOT3xG69H9dmbPd9TCbIKidT_fAgNMZEY2BR0qPmHW3JfhP0KSYKgUQ9dyiHrulr41WxGtIHFVkrdDqeaHCOoUuo.r_V_du0eyowAqg1oQ__QllHDwAeUxV1av7Z4HFjVBKw&dib_tag=se&keywords=m2%2Bscrews&qid=1778806119&sprefix=m2%2Bscrew%2Caps%2C149&sr=8-8&th=1">M2 screws</a>| x3 |

## 3. Remote Control Circuit<a name="remotecontrolcircuit"></a>
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

### Control Input Processing<a name="controlinputprocessing"></a>
#### Throttle Input (Left Analog Stick)<a name="throttleprocessing"></a>
#### Steering Input (Right Analog Stick)<a name="steeringprocessing"></a>

### RF Transmitter<a name="rftransmitter"></a>
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

### Remote Control <a name="casebase"></a>

### Schematics & PCB
Schematic and PCB files are available in the `eda/rc-car-kicad/` directory. Example files:
- `rc-car-kicad.kicad_sch` – Main schematic
- `rc-car-kicad.kicad_pcb` – Main PCB layout
- `rc-car-panel.kicad_pcb` – Panelized PCB for manufacturing

---

## License
See [LICENSE](LICENSE) for details.