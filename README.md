# LS1
LS1 Snap multi-card audio recorder


## Updating Firmware

1.	Install Teensyduino from https://www.pjrc.com/teensy/loader.html
2.	Get latest hex file from appropriate repository
	https://github.com/loggerhead-instruments/LS1/blob/96-kHz/hex/LS1.2018-10-16.hex
	To download the file, right click on the Raw button and select Save As link.
3.	Connect microUSB cable to small board on bottom of main LS1 board. You may need to unscrew it. The microUSB cable will provide power to the LS1.
4.	Run the Teensy Loader program.
5.	From File Menu, select Open HEX File. Note that you may get an error saying the hex file is too large. You can ignore this message.
6.	From Operation menu, select Program (you may have to turn off automatic mode). Note you may need to do a physical reset by removing the board and shorting (connect with a paper clip or forceps) the pins.
![alt text](https://github.com/loggerhead-instruments/LS1/tree/96-kHz/images/pinShort.png "Teensy reset")

To check whether the firmware has been updated, collect some data, and check the log files for the version date of the firmware.
