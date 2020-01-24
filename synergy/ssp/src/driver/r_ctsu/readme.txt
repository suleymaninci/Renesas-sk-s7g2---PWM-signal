PLEASE REFER TO THE APPLICATION NOTE FOR THIS MODULE FOR MORE INFORMATION

r_touch
========

Document Number 
---------------
R01ANxxxxEUxxxx

Version
-------
v1.00

Overview
--------
Implements a Capacitive Touch Sensing Unit driver. Allows users to easily setup the CTSU registers for their device
in a given configuration and provide as an output the results of channel measurement (raw/filtered), and a string of 
binary bits which indicate whether a channel is being touched or not.

Features
--------
*	Load from multiple configurations into the CTSU one at a time. Multiple configurations can exist in ROM, 
	however only one configuration will be active at a given time.
* 	Enable peripheral interrupts.	
*	Dynamically figure out which channels are active as receive (primary) and (secondary).
*	Allow use of dynamic memory to optimize volatile memory usage.
*	Work with user defined buffers for storing data.
*	Provide a binary bit-string of data where each bit represents a channel (ascending order) being touched 
	(logic 1) or not (logic 0).
*	Provides for an Open function that loads a configuration and performs optional tasks.
*	Provides a Close function which deactivates the CTSU and closes the configuration.
*	Provides ability to perform filtering, auto tuning, drift compensation, Calculation of delta counts through the 
	Process function in the correct/prescribed order. Allows user to override/disable some functionality.
*	Provides a function to read the results of a scan cycle through Read Results function.
*	Provides Control function to issue advanced commands and query operation.
*	Provision for a callback to indicate that a scan cycle has completed.
*	The R_Touch layer makes available to the user a function to check if a channel is enabled.
*	Permits ignoring channels in operations beyond raw data acquisition.


Supported MCUs
--------------
* RX113 Group
* RX231 Group

Boards Tested On
----------------
* Renesas RX113 CAPTouch MCU board.

Limitations
-----------
* Currently this module does not check if a pin can support the input configuration. The user should ensure the 
  configuration they are writing to the pin is supported by the pin.

Peripherals Used Directly
-------------------------
* CTSU
* DTC (optional, recommended)

Required Packages
-----------------
* r_bsp     v2.80 (or higher) for RX113

How to add to your project
--------------------------
* Add src\r_touch.c to your project.
* Add an include path to the 'r_touch' directory. 
* Copy the reference configuration file 'r_touch_config_reference.h' to your project and rename it r_touch_config.h.
* Configure middleware for your system through just copied r_touch_config.h. Check how it affects the source file 
	r_touch.c
* Add a #include for r_touch_if.h to any source files that need to use the API functions.
* Add a tuning configuration following steps in xxx.

Toolchain(s) Used
-----------------
* Renesas RX v2.02

File Structure
--------------
r_touch
|   readme.txt
|   r_touch_if.h
|
+---doc
|       r01anxxxxeuxx_rx.pdf
|
+---ref
|       r_touch_config_reference.h
|
\---src
        r_touch.c
