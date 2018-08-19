# Kello
> A simple USB powered Nixie Clock

![Version][Ver-image]
![Status][Stat-image]

USB powered clock made on four classic Z567M Nixie tubes. Pretty simplistic, without unnecessary gadgets: displays time, can be set up both from PC app or via optional capacitive buttons. Refresh frequency, brightness, time format, time and date (for adjusting winter/ summer time) can be configured as needed. Settings and current time are stored in EEprom and RTC. If the clock is connected to the PC it will update its time every minute (using the provided link).

![](header.png)

For the prototype version I've dropped the idea of capacitive buttons integrated into the case, as the clock will stay connected to the PC and it's much more ergonomic to use the app.
The first idea for the clock opted for bluetooth/wifi integration, so that a smartphone could be used as interface for quick setup and the clock itself could synchronize time via Internet.  Since there already is a USB socket used to power the device it seemed silly not to connect it to the PC, which itself simplified the project rendering Wifi and BT pointless. 

## Construction advices

The Nixie standoffs can be made from D-Sub connectors (such as MH CONNECTORS MHDD25-F-T-B-M). They are pricey, but work wonders and look good. For this clock i used 3 connectors, and ended up with some leftover pins. To extract them from the socket itself cut the metal enclosure to release the black plastic holding the pins.

The dots (Neon bulbs) are soldered on 3 pieces of 2mm piano wire. Sanding the wire down before soldering will ease the process. 


## Release History

* 0.1.0
    * First Prototype
* 0.1.1
    * Placed GND Pour keepout under L1
    * Fixed RTC reseting at powerup

## Meta

Krzysztof Belewicz – belewicz@gmail.com

Distributed under the Beerware license. If you decide to build your own feel free to send me a pic.

[https://github.com/CannaCardo/github-link](https://github.com/CannaCardo/)


<!-- Markdown link & img dfn's -->
[Ver-image]: https://img.shields.io/badge/Version-1.0-orange.svg?longCache=true&style=flat-square
[Stat-image]: https://img.shields.io/badge/Status-Finished-green.svg?longCache=true&style=flat-square
[wiki]: https://github.com/CannaCardo/Kello/wiki
