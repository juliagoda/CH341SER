# CH341SER driver

1. [About driver](#about-driver)  
2. [Changes](#changes)  
3. [Tests](#tests)  
4. [Installation](#installation)  
5. [Official website](#official-website)  
6. [Tutorial on Arch Linux](#tutorial-on-arch-linux)  

<br/>
<br/>

## About driver

It's a manufacturer software of standard serial to usb chip marked CH340

<br/>
<br/>

## Changes

Added line  
`#include <linux/sched/signal.h>`  
<br/>
which helps to fix the problem below:  
`error: implicit declaration of function ‘signal_pending’; did you mean ‘timer_pending’? [-Werror=implicit-function-declaration]`

<br/>
<br/>

## Tests

Tested on:
* Arch Linux 4.11.3-1-hardened
* Arch Linux 4.11.3-1-ARCH

<br/>
<br/>

## Installation

See original readme.txt

<br/>
<br/>

## Official website

[http://www.wch.cn/download/CH341SER_LINUX_ZIP.html](http://www.wch.cn/download/CH341SER_LINUX_ZIP.html)

<br/>
<br/>

## Tutorial on Arch Linux

Tested for Arduino UNO R3 Clone  
<br/>


install required packages:  
`sudo pacman -S arduino arduino-docs avr-binutils avr-gcc avr-libc avrdude`

<br/>

if your system detects the package below:  
`pacman -Qs arduino-avr-core`

<br/>

you should to remove it:  
`sudo pacman -R arduino-avr-core`

<br/>

we add current user to uucp and lock groups:  
```
gpasswd -a $USER uucp
gpasswd -a $USER lock
```
<br/>

it's possible that you have to load module:  
`modprobe cdc_acm`

<br/>

clone fixed driver:  
`git clone https://github.com/juliagoda/CH341SER.git`

<br/>

according to the original readme.txt, we use the commands below:  
```
make
sudo make load
```
<br/>

to be sure, that module will be loaded after reboot, you can change file extension from *.ko to *.ko.gz and add it to drivers path:  
```
find . -name *.ko | xargs gzip
sudo cp ch34x.ko.gz /usr/lib/modules/$(uname -r)/kernel/drivers/usb/serial
```
<br/>

if the command:  
`lsmod | grep ch341` 

<br/>

shows some result, then:  
```
sudo rmmod ch341
sudo mv /usr/lib/modules/$(uname -r)/kernel/drivers/usb/serial/ch341.ko.gz /lib/modules/$(uname -r)/kernel/drivers/usb/serial/ch341.ko.gz~
sudo depmod -a
```

<br/>

let's connect arduino uno R3 clone to USB input and check our results:  
`dmesg | grep ch34x`

<br/>

that's example of my command's output:  
```
[  492.836159] ch34x 3-1:1.0: ch34x converter detected
[  492.846265] usb 3-1: ch34x converter now attached to ttyUSB0
```

<br/>

so our driver ch34x was successfully loaded and our port's name is ttyUSB0  
<br/>
Let's start our installed Arduino IDE.  
<br/>
First we should to install package for Arduino AVR Boards from Boards Manager:  

<br/>

![Choose of Boards Manager](images/arduino_change1.png)

<br/>

![Installation of packages for Arduino AVR Boards from Boards Manager](images/arduino_change2.png)

<br/>

And now we must to choose our port's name. My port's name is ttyUSB0.  

<br/>

![Choose of port's name](images/arduino_change3.png)



