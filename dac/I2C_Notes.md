---> Extra packages to be installed

apt install libi2c-dev i2c-tools gpiod


[    6.091214] i2c i2c-0: Added multiplexed i2c bus 2
[    6.096136] i2c i2c-0: Added multiplexed i2c bus 3
[    6.101060] i2c i2c-0: Added multiplexed i2c bus 4
[    6.105982] i2c i2c-0: Added multiplexed i2c bus 5
[    6.110907] i2c i2c-0: Added multiplexed i2c bus 6
[    6.116469] at24 7-0050: 1024 byte 24c08 EEPROM, writable, 1 bytes/write
[    6.123193] i2c i2c-0: Added multiplexed i2c bus 7
[    6.128117] i2c i2c-0: Added multiplexed i2c bus 8
[    6.133035] i2c i2c-0: Added multiplexed i2c bus 9
[    6.137830] pca954x 0-0073: registered 8 multiplexed busses for I2C switch pca9548
[    6.139629] ata2: SATA link down (SStatus 0 SControl 330)
[    6.150812] ata1: SATA link down (SStatus 0 SControl 330)
[    6.156287] i2c i2c-0: Added multiplexed i2c bus 10
[    6.161313] i2c i2c-0: Added multiplexed i2c bus 11
[    6.166338] i2c i2c-0: Added multiplexed i2c bus 12
[    6.171352] i2c i2c-0: Added multiplexed i2c bus 13
[    6.176360] i2c i2c-0: Added multiplexed i2c bus 14
[    6.181369] i2c i2c-0: Added multiplexed i2c bus 15
[    6.186377] i2c i2c-0: Added multiplexed i2c bus 16
[    6.191389] i2c i2c-0: Added multiplexed i2c bus 17
[    6.196271] pca954x 0-0077: registered 8 multiplexed busses for I2C switch pca9548
[    6.203859] cdns-i2c ff020000.i2c: 400 kHz mmio ff020000 irq 33




This is changing the DAC values...
0xD is the DAC address

i2cset -y 8 0x0d 0x1 0xDDDD


Visible devices in the FMC:

root@zynqmp-cib-20:~# i2cdetect -y -r 8
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- 0d -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- 57 -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- 6a -- -- -- -- --
70: -- -- -- UU -- -- -- UU


Unsure of what is the A6



All read/write sequences start with the device address (0xD) followed by a pointer byte and then data, if needed

Pointer byte:
-------------

The structure of the pointer byte is :

X X 0 0 0 0 DAC_B DAC_A

Example, for sending a command just to DAC_A one uses 0x1, or to send both one sends 0x3

---------------------------------------------------------------------
---------------------------------------------------------------------

Setting the DAC:
----------------

Two bytes are necesary. The DAC in question is a AD5339:

MSByte:

PD1 PD0 NCLR NLDAC D11 D10 D9 D8

0 0 1 0  

LSByte:

D7 D6 D5 D4 D3 D2 D1 D0

Description of bits:

* PD1, PD0 : Define the power mode. 
    * 00 : normal operation
    * 10 : power down with 10k load
    * 01 : power down with 100k load
    * 11 : power down (three state output)

* NCLR : Clear all the registers 
    * 0: clear
    * 1: normal operation

* NLDAC: commit whatever is in the input register into the DAC register
    * 0 : Set the DAC
    * 1 : Write only to the input register

* D11..D0 : Data bits. Set the DAC value


Examples:

1. Set the DAC 1 to 0xAAA (note the 0x2 - 0010 to set the dac)

i2ctransfer -v -y 8 w3@0xd 0x1 0x2A 0xaa

2. Read the DAC 1

i2ctransfer -v -y 8 w1@0xd 0x1 r2


3. Write into both DACs the value 0xBBB

i2ctransfer -v -y 8 w3@0xd 0x3 0x2B 0xBB


---------------------------------------------------

PLL : 0xD0 is a 8-bit address. The *real* address is 0x6A (shift one bit right)

---------------------------------------------------

CDR : Address 0x60 does have contents. So one should be able to get information out of it

----------------------------------------------------

SFP : Both addresses 0x50 and 0x51 show contents

----------------------------------------------------

TEBF0808 PLL : Meant to be in the channel 1 of 0x77 bus at address 0x70. Need confirmation
