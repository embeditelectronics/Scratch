Example projects for scratch with the Raspberry Pi and RPiSoC

You can open these opening ScratchGPIO on the desktop, then clicking "File" and then "open" and navigate to the projects in this folder. Video example of these projects here: https://www.youtube.com/watch?v=tbu-IjGkmms

1. rpisoc_analog_ex moves scratch the cat back and forth according to the voltage reading (from 0 to 5V) on RPiSoC pin 3[0]. In my video example I plug a potentiometer into that pin to use as a knob.

2. analog_game builds on the analog example and makes a simple two player game out of it. Now the cat has to dodge cannonballs! 

The player controlling scratch can use a potentiometer to move as before. Player two controls the cannon with the left and right arrow keys on the keyboard to move it, and space to shoot.

A great learning excercise would be to give the cat velocity instead of just 1-to-1 movement with the potentiometer.

3. servo_test builds on the analog example by using the analog input to control a servo motor. The servo motor is on RPiSoC pin 6[4]. 