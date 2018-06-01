# LS1
LS1 Snap multi-card audio recorder


## Updating Firmware

1.	Install Teensyduino from https://www.pjrc.com/teensy/loader.html
2.	Get latest hex file from appropriate repository
	https://github.com/loggerhead-instruments/LS1/tree/master/hex
	To download the file, left-click on the file name, and then right click on the Raw button and select Save As link.
3.	Connect microUSB cable to small board on bottom of main LS1 board. You may need to unscrew it. The microUSB cable will provide power to the LS1.
4.	Run the Teensy Loader program.
5.	From File Menu, select Open HEX File
6.	From Operation menu, select Program (you may have to turn off automatic mode). Note that the hard reset button is underneath the board and difficult to access.

To check whether the firmware has been updated, collect some data, and check the log files for the version date of the firmware.
