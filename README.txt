The Scratch visual programming language brought to the Raspberry Pi and RPiSoC!

Read more about it here: http://scratch.mit.edu/

This is an extension of Scratch GPIO 5: http://cymplecy.wordpress.com/scratchgpio/scratch-raspberrypi-gpio/

To use this follow these installation steps:

1. First you need our psoc_2_pi Python API set up. (SPI support only for now) Instructions here: http://embeditelectronics.github.io/psoc_2_pi/getting_started.html#configure-spi-and-download-spidev

2. Next you need to install Scratch GPIO: http://cymplecy.wordpress.com/scratchgpio/scratch-raspberrypi-gpio/

3. Finally download this repository and put the files into the scratchgpio5 folder, overwriting any existing files. The path will should be /home/pi/scratchgpio5

4. Open up the ScratchGPIO 5 icon from the desktop and enjoy. A short tutorial for using it can be found here: http://www.embeditelectronics.com/blog/rpisoc/creating-your-first-scratch-program/

This scratch extension currently supports (with lots more to come):

analog input
digital i/o
servo motor control

The examples folder shows how to incorporate some of these new commands into a scratch program.