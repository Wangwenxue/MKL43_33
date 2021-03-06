Overview
========
The flexio_spi_master_dma_spi_slave example shows how to use flexio spi master driver in dma way:

In this example, a flexio simulated master connect to a spi slave .

Toolchain supported
===================
- IAR embedded Workbench 7.50.1
- Keil MDK 5.17
- GCC ARM Embedded 2015-4.9-q3
- Kinetis Development Studio IDE 3.0.0
- Atollic TrueSTUDIO 5.4.0

Hardware requirements
=====================
- Mini USB cable
- TWR-KL43Z48M board
- Primary Elevator
- Personal Computer

Board settings
==============
The flexio_spi_master_dma_spi_slave example is requires connecting between FlexIO pins with SPI pins
Insert TWR-KL43Z48M board into Primary Elevator. The connection should be set as following:
- B46-Elevator, B9-Elevator connected
- B48-Elevator, B7-Elevator connected
- B45-Elevator, B10-Elevator connected
- B44-Elevator, B11-Elevator connected

Prepare the Demo
================
1.  Connect a mini USB cable between the PC host and the OpenSDA USB port on the board.
2.  Open a serial terminal on PC for OpenSDA serial device with these settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
When the demo runs successfully, the log would be seen on the OpenSDA terminal like:
~~~~~~~~~~~~~~~~~~~~~
FLEXIO Master dma - SPI Slave interrupt example start.
This example use one flexio spi as master and one spi instance as slave on one board.
Master uses dma and slave uses interrupt way.
Please make sure you make the correct line connection. Basically, the connection is:
FLEXIO_SPI_master -- SPI_slave
   SCK      --    SCK
   PCS0     --    PCS0
   MOSI     --    MOSI
   MISO     --    MISO
This is SPI slave call back.
FLEXIO SPI master <-> SPI slave transfer all data matched!
~~~~~~~~~~~~~~~~~~~~~

Customization options
=====================


