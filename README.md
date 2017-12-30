SelfBalancingRobot-AVR
======================

Robot code for the Atmega 168 MCU

Requirement
===========
CrossPack-AVR (http://www.obdev.at/products/crosspack/download.html)

QuickStart
==========


Compilation
===========

Pin Configuration
===========
    PD0 - UART RX
    PD1 - UART TX
    PD2 - Encoder 1 Interrupt
    PD3 - Encoder 2 Interrupt
    PD4 - Motor1 B
    PD5 - PWM 1
    PD6 - PWM 2
    PB1 - Motor2 A
    PB2 - Motor2 B
    PB3 - Motor1 A
    PB5 - Led 1
    PB6 - Encoder 1 B
    PB7 - Encoder 2 B
 
    PC0 - Encoder 1 A
    PC1 - Encoder 1 B
    PC2 - Encoder 2 A
    PC3 - Encoder 2 B
 
    PC4 - I2C SCL
    PC5 - I2C SDA

License
=======

Copyright (c) 2014 Arvid Björkqvist

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal 
in the Software without restriction, including without limitation the rights 
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
