# charge-controller
STM32 Test MCU Project
Write a program in C;
1) Monitoring the voltage on each battery and when the voltage goes below 3.6 V on any of them disconnect the load;
2) When the discharge current exceeds 10 A, it is necessary to disconnect the load;
3) If current is below 10 A 10 seconds after diconnection than connect load again.

Features:
Test shcematics was provided with uart-usb converter so I added some kind of protocol (on 115200 bps VCP):
1) Outcoming messages by default are sent each second, they include current and all battery voltages and also state of relay-controll pin;
"Current: %f A\r\nB1 Voltage: %f V\r\nB2 Voltage: %f V\r\nB3 Voltage: %f V\r\nB4 Voltage: %f V\r\nRelay State: %d\r\n\r\n"
2) Incoming messages desrided below:
SC - Set Max Current, example "SC 15.5" - set Max Current to 15.5 A;
SV1 - Set Mimimum voltage of battery with number (1 in this case), example "SV1 3.2" - set Min Voltage to 3.2 V;
ST - Set new timeout in milliseconds (default is 10 seconds as described above), example "ST 2500" - set timeout to 2,5 seconds;
SL - Sel new log period in milliseconds (default is 1 second as descrived above), example "SL 100" - set log period to 100 ms.
If messages is received correctly and value is changed device gives back  "OK!" message, else "ERROR".
