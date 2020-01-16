## contains 3 projects

* **passthru**: for checking [stm32f4-discovery](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) hardware, microphone and speaker interface. Requires only one discovery board. Useful for anybody wishing to use this board for audio purpose.
* **vocoder_passthru**: adds vocoder to passthru test using single discovery board: For checking codec2 from microphone encoding directly decoding out to speaker.  This doubles CPU power required since both encoding and decoduing must be done for each frame.
* **lora_codec2**: adds SX1262 radio, half duplex transceiver.  Requires two discovery boards, each with its own sx1262.  PTT is blue button (push to talk)

## QuickStart
After cloning this repository:
```
$ cd codec2
$ git submodule init
$ git submodule update
```
Follow tool install instructions here: [steps 2 and 4 for toolchain and peripheral](https://github.com/drowe67/codec2/tree/master/stm32) library
```
(from root directory of this project)
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_TOOLCHAIN_FILE=../codec2/stm32/cmake/STM32_Toolchain.cmake -DPERIPHLIBDIR=/opt/STM32F4xx_DSP_StdPeriph_Lib_V1.8.0/
$ make
```

Select codec2 bit-rate: uncomment the line in CMakeLists.txt:
``#target_compile_definitions(lora_codec2 PRIVATE -DCODEC2_MODE=CODEC2_MODE_<bitrate>)``
Select desired LoRa bandwidth on the line defining ``LORA_BW_KHZ`` in CMakelists.txt.  Spreading factor will be selected appropriate for the vocoder bit-rate and LoRa bandwidth selected.  Spreading factor functional range is 5 to 12.  See file loRa_codec2/main.h where spreading factor is defined according to codec2 bitrate vs LoRa bandwidth.

Flashing stm32f4-discovery
https://github.com/texane/stlink
providing address isnt needed when using st-flash with .hex files
i.e: 
``st-flash --format ihex write lora_codec2.hex``



codec2 mode   | samples per frame    |  encoded bytes per frame    | frames per lora packet | LoRa packet length bytes | LoRa BW KHz | LoRa SF   | air-time used percent | packet duration (ms)
------ | ------------- | --------- | ----------- | -----------  | ------ | ----- | -----------  | ---------
3200   |    160        |      8    |       16    |  128         |  500   |  10   |  96          |  
2400   |    160        |      6    |       16    |   96         |  500   |  10   |  77          |  237
1600   |    320        |      8    |        8    |   64         |  500   |  10   |  56          |
1600   |    320        |      8    |       16    |  128         |  500   |  11   |  86          |  555
1400   |    320        |      7    |       16    |  112         |  500   |  11   |  77          |  493
1300   |    320        |     6.5   |        8    |   52         |  500   |  11   |  90          |  288
1200   |    320        |      6    |       16    |   96         |  500   |  11   |  71          |  452
1200   |    320        |      6    |       32    |  192         |  500   |  11   |  62          |  1281
700C   |    320        |     3.5   |       16    |   56         |  250   |  11   |  96          |  576
450    | assert e == 0 postfilter()

LoRa data rate selection is only possible in steps by a factor of two.  Codec2 bit-rate change will affect LoRa packet duty cycle.  When duty cycle is under 50%, the LoRa data-rate can be reduced (bandwidth reduced or SF increased).  If packet duty cycle is over 100%, then LoRa data-rate must be increased to faster.  Typical 2.5 to 2.7dB change in link budget for each step of LoRa data-rate.

Latency across the radio link is due to LoRa packet duration.

SX1272 or SX1276 shouldn't be used for this, because SX1261/SX1262 receives large LoRa packets with less errors.

## Wiring to LoRa transceiver
pin function |  port/pin |  discovery connector | [sx1262 shield](https://os.mbed.com/components/SX126xMB2xAS/) pin
------------ | --------- | -------------------- | ----------- | 
Vdd          |           |    P1-3        |       J3-4  |           
Gnd          |           |    P1-1        |       J2-7
NSS          | PA10    |     P2-41        |       J1-8
MOSI         | PA7     |     P1-17       |        J2-4
MISO          |PA6     |     P1-18       |        J2-5
SCLK         | PA5    |      P1-15        |       J2-6
DIO1          |PA2     |     P1-14        |       J1-6
BUSY         | PB1     |     P1-21         |      J1-4
AntSwPwr     | PB3     |     P2-28         |      J2-1
Other modules could be also used, such as [dorji module](http://www.dorji.com/products-detail.php?ProId=63)

## implementation details
DMA is used for microphone interface because PDM microphone operates at 16x the required sample rate.  In the case of codec2, 8Ksps is required, with the PDM microphone operating at 64ksps.  16 samples are collected using DMA, needing an interrupt only when at least 16 samples are ready to send to PDM filter.

## credits
* [PDM filter](https://github.com/olegv142/pdm_firx) for on-board microphone.
* USB [CDC device for stm32f4-discovery](https://github.com/xenovacivus/STM32DiscoveryVCP).
