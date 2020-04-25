# TWC
C Code to talk to the Tesla Wall Connector Gen2 (EVW2T32HL) over the RS-485 port used for load sharing communication.

This is based on works from https://github.com/dracoventions/TWCManager and the Wall Connector load sharing protocol thread at https://teslamotorsclub.com/tmc/threads/new-wall-connector-load-sharing-protocol.72830/

TWCManager is written in python and is not as suitable for porting to embedded microcontroller systems. My aim is to eventually port this code to the ESP32/ESP8266.

Currently the code runs on Linux (Tested on ubuntu 19.10) and the Raspberry PI using an FTDI based RS-232 adapter.


