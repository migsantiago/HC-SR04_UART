# HC-SR04_UART
Short example on how to communicate a Raspberry Pi with the HC-SR04 via UART GPIOs.
The example allows you to take measurements and to listen to them by using the Pi speakers.

# UART mode
The HC-SR04 has a way to communicate with a controller by using its UART mode.
My specific HC-SR04 only asks to have:
M1 -> OPEN
M2 -> SHORTED

# Connection to the Raspberry Pi
The specifics on how to setup the serial port of the Pi are in the source code.

The connections are as follows:


HC-SR04 -> Raspberry Pi

VCC     -> 3.3V

Trig/Rx -> Pin 8 AKA TXD (220R in the middle to avoid accidents)

Echo/Tx -> Pin 10 AKA RXD (220R in the middle to avoid accidents)

GND     -> GND

www.migsantiago.com

HC-SR04 front

<img width="451" height="240" alt="HC-SR04_front" src="https://github.com/user-attachments/assets/acc01284-c4a3-4add-9024-657556c0ce04" />

HC-SR04 back

<img width="357" height="247" alt="HC-SR04_back" src="https://github.com/user-attachments/assets/12e06c39-2594-4e97-a510-3be91c3e3154" />
