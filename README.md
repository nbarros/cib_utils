# cib_utils
CIB Utilities. Various tools and libraries used in the CIB code base.

# Changelog

* 30/06/2024 : Nuno Barros <barros@lip.pt>
				Implemented working version of the cib_daq_server, along with the respective service.
				Too be deployed in the production CIBs soon.

# Module descrition
## axi_fifo

This is the chosen method for data transmission. This folder contains a simple example of a receiver using the readout of suck a FIFO. 


## cib_debug

Old test applications are located here.

### test_lbls

 A `test_lbls` that sets a pulser to the lbls system and returns a set of data through an `axi_fifo` that is used to ship data to the LBLS system, that in its turn returns its readou that must then be sent to the slow control.

## common

This implements a basic library with common aspects on teh CIB. This includes mapped memory addresses, thread managers, infrastructure to read and write into registers and perform miscelania operations. 
The module produces the `libcib_common` library.

## contrib

Various modules used in the current code base. The `spdlog` module is going to be dropped in a near future and instead made a system requirement. 

## dac

DAC readout methods. 

## daq

This module implements the daq server on the CIB side. There is a testing application in the `apps` module.
The module produces the `libcib_daq` library.

## i2c

This module implements the methods and applications to read and interact with the various i2c peripherals. 
The module produces the `libcib_i2c` library.

## pdts (deprecated)

This module implements a deprecated monitor for the pdts system

## proto (deprecated)

Former message definition to be used with protobuf. Not currently in use. 

## tests

Various simpletests to verify simple functionality. These tend to be standalone and not depend on other components of the package.

## versaclock

Versaclock configuration applications and register definitions. Not currently being used...but could be.

## apps

Various applications for multiple use

### cib_manager

The cib_manager application 