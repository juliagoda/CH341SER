# CH341SER driver

1. [About driver](#about-driver)  
2. [Changes](#changes)  
3. [Tests](#tests)  
4. [Installation](#installation)  
5. [Official website](#official-website)  

<br/>
<br/>

## About driver

It's a manufacturer software of standard serial to usb chip marked CH340

<br/>
<br/>

## Changes

Added line 
`#include <linux/sched/signal.h>`
which helps fix the problem below:
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


