//============================================================================
// Name        : hc_sr04_pi.cpp
// Author      : Santiago Villafuerte (migsantiago.com)
// Version     : 1
// Copyright   : (c) Copyright 2025 MigSantiago.com
// Description : Use an HC-SR04 to measure distances via UART
//============================================================================
#include <cstdint>
#include <cstdlib>
#include <errno.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <system_error>
#include <termios.h>
#include <thread>
#include <unistd.h>

/*
 * CONNECTIONS
 *
 * HC-SR04 -> Raspberry Pi
 *
 * VCC     -> 3.3V
 * Trig/Rx -> Pin 8 AKA TXD (220R in the middle to avoid accidents)
 * Echo/Tx -> Pin 10 AKA RXD (220R in the middle to avoid accidents)
 * GND     -> GND
 */

int setupSerialPort(const std::string &portPath)
{
    // SOURCE: https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

    int serial_port = open(portPath.c_str(), O_RDWR);
    auto errnoCpy = errno;
    if (serial_port < 0)
    {
        std::cout << "Cannot open " << portPath
                << " " << std::make_error_code(static_cast<std::errc>(errnoCpy)) << "\n";
    }

    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0)
    {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        std::exit(EXIT_FAILURE);
    }
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        std::exit(EXIT_FAILURE);
    }

    std::cout << "Port: " << portPath
            << "File descriptor: " << serial_port << "\n";

    return serial_port;
}

int main(int argc, char **argv)
{
    std::cout << "HC-SR04 via UART\n";
    std::cout << "Remember to enable your serial port and to remove it from cmdline\n";

    // sudo nano /boot/firmware/config.txt
    // # Enable UART
    // enable_uart=1

    // sudo nano /boot/firmware/cmdline.txt
    // Remove the entry "console=serial0,115200"

    // sync
    // sudo reboot

    if (2 != argc)
    {
        std::cout << "Argument must be serial port dev path\n";
        std::cout << "Example: " << program_invocation_short_name << " /dev/serial0\n";
        std::exit(EXIT_FAILURE);
    }

    const std::string PORT_PATH = argv[1];

    std::cout << "Setting serial port " << PORT_PATH << "\n";
    const int serialPort = setupSerialPort(PORT_PATH);

    uint8_t readBuffer[3] {};
    while (true)
    {
        static const uint8_t CMD_DISTANCE = 0xA0U;

//        std::cout << "Requesting distance... 0x"
//                << std::hex << static_cast<uint16_t>(CMD_DISTANCE) << "\n";

        int writtenBytes = write(serialPort, &CMD_DISTANCE, 1U);
        auto errnoCpy = errno;
        if (writtenBytes < 0)
        {
            std::cout << "Cannot write serial port "
                    << std::make_error_code(static_cast<std::errc>(errnoCpy)) << "\n";
            std::exit(EXIT_FAILURE);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // HC-SR04 answers with 3 bytes
        int readBytes = read(serialPort, &readBuffer[0], 3U);
        errnoCpy = errno;
        if (readBytes < 0)
        {
            std::cout << "Cannot read serial port "
                    << std::make_error_code(static_cast<std::errc>(errnoCpy)) << "\n";
            std::exit(EXIT_FAILURE);
        }

//        const uint32_t littleEndianValue =
//                (readBuffer[2] << 16) | (readBuffer[1] << 8) | readBuffer[0];
        const uint32_t bigEndianValue =
                        readBuffer[2] | (readBuffer[1] << 8) | (readBuffer[0] << 16);

//        std::cout << std::setfill('0') << std::setw(2)
//                << "Response: LE 0x" << std::hex << littleEndianValue
//                << " " << std::dec << littleEndianValue
//                << std::setfill('0') << std::setw(2)
//                << " BE 0x" << std::hex << bigEndianValue
//                << " " << std::dec << bigEndianValue
//                << "\n";

        // The correct endianness to use is big endian
        const double cm = bigEndianValue / 10000.0;
        std::string textToSpeech;

        // Discard erroneous reads
        if (cm < 1199.0)
        {
            std::cout << "Distance = " << cm << "cm\n";
            textToSpeech = "espeak \" " + std::to_string(static_cast<int>(cm)) + " centimeters\" 2>/dev/null";
        }
        else
        {
            std::cout << "Distance is UNKNOWN!\n";
        }

        // sudo apt install espeak
        if (!textToSpeech.empty())
        {
            // This is where the Pi speaks and reads the measurement by using its audio output
            // You should have configured it with raspi-config :)
            system(textToSpeech.c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    }

    return 0;
}
