#!/usr/bin/python
#-*- coding: utf-8

import future
from future import *
from blockext import *
from pisoc import *
from serial import SerialException

class scratch_SoC:
    def __init__(self):
        #print('initing')
       
        self.joystick_l = dict()
        self.analog_values = dict()
        self.select_l = dict()
        self.bitmap = 0
        self.neopixels = NeoPixelShield()
        self.open = False
        self.capsense_is_running = False
        self.cap_result = 0
        self.__neopixelcolor = 0x00
        self.__failed = False
        self.__connecting = False
        self.__lost_connection = False
        self.__port_failure = False
        self.__lists_filled = False

        self.gpio_l = [None]+[j for i in [[digitalPin(k, v, None) for v in PiSoC.GPIO[k]] for k in sorted(PiSoC.GPIO)] for j in i]
        self.ranger_l = [[None, None] for x in self.gpio_l]
        self.capsense_l = [None] + [CapSense(i, threshold = 6) for i in range(PiSoC.CAPSENSE_SENSOR_NUM)]
        self.analog_l = [None] + [analogPin(i) for i in range(PiSoC.ANALOG_IN_NUM)]
        self.pwm_l = [None for x in range(PiSoC.PWM_NUM + 1)]
        self.servo_l = [x for x in self.pwm_l]

        self.__busy = False
        self.open = True

    def _problem(self):
        if self.__connecting:
            self.open = False
            return "Trying to connect..."
        elif self.__lost_connection:
            self.open = False
            return "connection was lost..."
        elif self.__port_failure:
            self.open = False
            return "Port is closed; can't write to it"
        elif self.__failed:
            self.open = False
            return "Couldn't open serial port... Verify connection and try again"
        else:
            return False

    def reconnect_pisoc(self):
        try:
            print('trying to reconnect')
            self.open = False
            self.__connecting = True
            time.sleep(0.25)
            if PiSoC.commChannel.reconnect():

                '''
                self.gpio_l = [None]
                self.capsense_l = [None]
                self.analog_l = [None]
                self.pwm_l = [None]
                self.neopixels = NeoPixelShield()
                for key in sorted(PiSoC.GPIO.keys()):
                    for elem in sorted(PiSoC.GPIO[key]):
                        self.gpio_l.append(digitalPin(int(key), int(elem)))
                        self.ranger_l.append(None)
                        self.trigger.append(None)
                for i in range(PiSoC.CAPSENSE_SENSOR_NUM):
                    self.capsense_l.append(CapSense(i, THRESHOLD = 4))
                for i in range(PiSoC.ANALOG_IN_NUM):
                    self.analog_l.append(analogPin(i))
                for i in range(PiSoC.PWM_NUM):
                    self.pwm_l.append(None)
                    self.servo_l.append(None)
                '''
                self.bitmap = 0
                self.__failed = False
                self.__lost_connection = False
                self.__port_failure = False
                self.__connecting = False
                self.__busy = False
                print('reconnected successfully.')
                self.open = True
        except SerialException:
                self.__failed = True
                print("Couldn't open port...")
        except LostConnection:
            print("Connection was lost..")
            self.__lost_connection = True
        except:
            print("Connection was lost..")
            self.__lost_connection = True

        self.__connecting = False
        if not self.open:
            print("\nA problem occured: \nCouldn't open port. Verify connection and try again")
            self.__failed = True

    """
    @command("open PiSoC on COM %n with %d.baud baud")
    def pisocOpen(self, com, baudr):
        if not self.open:
            self.com = com
            self.baudr = int(baudr)
            self.__connecting = True
            for i in range(3):
                try:
                    if not self.open:
                        if not self.__lost_connection:
                            print("Trying to open COM%s" %com)
                            PiSoC('COM'+str(com), baud = baudr, DEBUG = True)
                            if PiSoC.DEBUG:
                                err_str = ('commChannel attribute not found', 'commChannel attribute found')[hasattr(PiSoC, 'commChannel')]
                                print err_str
                            self.gpio_l = [None]
                            self.capsense_l = [None]
                            self.analog_l = [None]
                            self.pwm_l = [None]
                            for key in sorted(PiSoC.GPIO.keys()):
                                    for elem in sorted(PiSoC.GPIO[key]):
                                        self.gpio_l.append(digitalPin(int(key), int(elem)))
                                        self.ranger_l.append(None)
                                        self.trigger.append(None)
                            for i in range(PiSoC.CAPSENSE_SENSOR_NUM):
                                self.capsense_l.append(CapSense(i, THRESHOLD = 4))
                            for i in range(PiSoC.ANALOG_IN_NUM):
                                self.analog_l.append(analogPin(i))
                            for i in range(PiSoC.PWM_NUM):
                                self.pwm_l.append(None)
                                self.servo_l.append(None)
                            self.__failed = False
                            self.__lost_connection = False
                            self.__port_failure = False
                            self.__connecting = False
                            self.open = True
                            break
                        else:
                            print('trying to reconnect')
                            if PiSoC.commChannel.reconnect():
                                self.gpio_l = [None]
                                self.capsense_l = [None]
                                self.analog_l = [None]
                                self.pwm_l = [None]
                                print('filling lists..')
                                for key in sorted(PiSoC.GPIO.keys()):
                                    for elem in sorted(PiSoC.GPIO[key]):
                                        self.gpio_l.append(digitalPin(int(key), int(elem)))
                                        self.ranger_l.append(None)
                                        self.trigger.append(None)
                                for i in range(PiSoC.CAPSENSE_SENSOR_NUM):
                                    self.capsense_l.append(CapSense(i, THRESHOLD = 4))
                                for i in range(PiSoC.ANALOG_IN_NUM):
                                    self.analog_l.append(analogPin(i))
                                for i in range(PiSoC.PWM_NUM):
                                    self.pwm_l.append(None)
                                    self.servo_l.append(None)
                                print('lists filled')
                                self.__failed = False
                                self.__lost_connection = False
                                self.__port_failure = False
                                self.__connecting = False
                                print('reconnected successfully.')
                                self.open = True
                                break
                except SerialException:
                    self.__failed = True
                    print("Couldn't open COM%s... %d tries remaining\n"%(com, (3-i)))
                    time.sleep(1)
                except LostConnection:
                    print("Connection was lost..")
                    self.__lost_connection = True
                except:
                    print("Connection was lost..")
                    self.__lost_connection = True


        self.__connecting = False
        if not self.open:
            print("\nA problem occured: \nCouldn't open COM%s... Verify connection and COM number and try again"%com)
            self.__failed = True
        """


    @command("reset PiSoC")
    def pisocClose(self):
        try:
            if self.open:
                while self.__busy:
                    pass
                self.__busy = True
                PiSoC.commChannel.cleanup()
                if not self.__connecting:
                    self.reconnect_pisoc()
                self.__busy = False
        except LostConnection:
            self.__lost_connection = True
        except ClosedPortException:
            self.__port_failure = True

    @predicate("touch sensor %d.num touched?")
    def capsenseTouched(self, num):
        if self.open:
            num = int(num)
            try:
                while self.__busy:
                    pass
                self.__busy = True
                num = int(num)
                if not self.capsense_l[num].isRunning():
                    self.capsense_l[num].Start()
                if num == 1:
                    self.cap_result = self.capsense_l[1].get_register()

                result = (self.cap_result>>(num - 1))&0x01
                self.__busy = False
                return result
            except LostConnection:
                self.__lost_connection = True
            except ClosedPortException:
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
            return False

    @command("set pin %n to %m.state")
    def setPin(self, pin, state):
        print("Setting pin %d to %s" %(pin, state))
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if self.gpio_l[pin].config is not None:
                    self.gpio_l[pin].Write((1, 0)[state == 'off'])
                self.__busy = False
            except LostConnection:
                self.__lost_connection = True
            except ClosedPortException:
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @command("configure Joystick %d.num with X:%d.num Y:%d.num button:%d.dig_num")
    def joystickConfig(self, num, x_axis, y_axis, pin):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            num = int(num)
            x_axis = int(x_axis)
            y_axis = int(y_axis)
            pin = int(pin)
            self.joystick_l[num] = [x_axis, y_axis, pin]
            self.analog_values[x_axis] = [None, None, None, None]
            self.analog_values[y_axis] = [None, None, None, None]
            self.select_l[pin] = None
            try:
                self.gpio_l[pin].Configure('PULL_UP')
                self.gpio_l[pin].Write(1)
                print self.gpio_l[pin]
                self.select_l[pin] = self.gpio_l[pin].state
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @reporter("Joystick %d.num X %m.analog")
    def joystickRead_x_axis(self, num, choice):
        if self.open:
            num = int(num)
            try:
                if choice == 'counts':
                    return self.analog_values[self.joystick_l[num][0]][0]
                elif choice == 'volts':
                    return self.analog_values[self.joystick_l[num][0]][1]
                elif choice == 'screen_x_axis':
                    return self.analog_values[self.joystick_l[num][0]][2]
                elif choice == 'screen_y_axis':
                    return self.analog_values[self.joystick_l[num][0]][3]
            except KeyError:
                return "Joystick %s not configured yet!"%num
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @reporter("Joystick %d.num Y %m.analog")
    def joystickRead_y_axis(self, num, choice):
        if self.open:
            num = int(num)
            try:
                if choice == 'counts':
                    return self.analog_values[self.joystick_l[num][1]][0]
                elif choice == 'volts':
                    return self.analog_values[self.joystick_l[num][1]][1]
                elif choice == 'screen_x_axis':
                    return self.analog_values[self.joystick_l[num][1]][2]
                elif choice == 'screen_y_axis':
                    return self.analog_values[self.joystick_l[num][1]][3]
            except KeyError:
                return "Joystick %s not configured yet!"%num
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @predicate("Joystick %d.dig_num button pressed?")
    def joystickRead_select(self, num):
        if self.open:
            num = int(num)
            try:
                return not self.select_l[self.joystick_l[num][2]]
            except KeyError:
                return False
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @command("set pin %n as %m.config")
    def pinConfig(self, pin, config):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                print("Setting pin %d as %s" %(pin, config))
                self.gpio_l[pin].Configure(config)
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @command('blink pin %n')
    def pinBlink(self, pin):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                print("blinking pin %d" %pin)
                if self.gpio_l[pin].config is not None:
                    self.gpio_l[pin].Toggle()
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @predicate("pin %d.dig_num is on?")
    def pinState(self, num):
        #print("reading pin %s" %num)
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            num = int(num)
            try:
                if (num == 1):
                    self.bitmap = self.gpio_l[1].get_gpio_bitmap()

                result = (self.bitmap>>(num - 1))&0x01
                if num in self.select_l:
                    self.select_l[num] = result
                self.__busy = False
                return result
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
            return False

    @reporter("pin %d.dig_num value")
    def pinVal(self, num):
        #print("reading pin %s" %num)
        if self.open:
            num = int(num)
            try:
                result = (self.bitmap>>num)&0x01
                return result
            except LostConnection:
                self.__lost_connection = True
            except ClosedPortException:
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
            return 0

    @reporter("analog pin %d.num %m.analog")
    def analogRead(self, num, analog):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            num = int(num)
            try:
                counts = self.analog_l[num].Read()
                volts = self.analog_l[num].ReadVolts(COUNTS = counts)
                screen_y = 55*volts - 140
                screen_x = 75*volts - 180
                self.__busy = False
                if num in self.analog_values:
                    self.analog_values[num] = [counts, volts, screen_x, screen_y]
                if analog == 'counts':
                    return counts
                elif analog == 'volts':
                    return volts
                elif analog == 'screen_x_axis':
                    return screen_x
                elif analog == 'screen_y_axis':
                    return screen_y
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()


    @command("set PWM %d.num %m.pwm_c to %n")
    def setPWMval(self, num, pwm_c, val):
        num = int(num)
        while self.__busy:
            pass
        self.__busy = True
        try:
            if self.open:
                if self.pwm_l[num] is None:
                    self.pwm_l[num] = PWM(num - 1)
                status = self.pwm_l[num].isRunning()
                if not status:
                    self.pwm_l[num].Start()
                if pwm_c == 'period':
                    val = int(val)
                    self.pwm_l[num].WritePeriod(val, safety = True)
                elif pwm_c == 'compare':
                    val = int(val)
                    self.pwm_l[num].WriteCompare(val, safety = True)
                elif pwm_c == 'onpercentage':
                    val = float(val)
                    self.pwm_l[num].SetDutyCycle(val, safety = True)
                elif pwm_c == 'frequency':
                    val = float(val)
                    self.pwm_l[num].SetFrequency(val)
                elif pwm_c == 'midinote':
                    val = int(val)
                    self.pwm_l[num].SetMIDI(val)
                if not status:
                    self.pwm_l[num].Stop()
                self.__busy = False
            else:
                if not self.__connecting:
                    self.reconnect_pisoc()
        except LostConnection:
            self.__busy = False
            self.__lost_connection = True
        except ClosedPortException:
            self.__busy = False
            self.__port_failure = True

    @command("tell PWM %d.num to %m.pwm")
    def setPWM(self, num, pwm):
        num = int(num)
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if self.pwm_l[num] is None:
                    self.pwm_l[num] = PWM(num -1)
                if pwm == 'start':
                    self.pwm_l[num].Start()
                elif pwm == 'stop':
                    self.pwm_l[num].Stop()
                status = self.pwm_l[num].isRunning()
                if not status:
                    self.pwm_l[num].Start()
                if pwm == 'sleep':
                    self.pwm_l[num].Sleep()
                elif pwm == 'wakeup':
                    self.pwm_l[num].Wakeup()
                elif pwm == 'refresh':
                    self.pwm_l[num].ReadPeriod()
                    self.pwm_l[num].ReadCompare()
                if not status:
                    self.pwm_l[num].Stop()
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()


    @reporter("PWM %d.num %m.pwm_c value")
    def getPWM(self, num, pwm_c):
        num = int(num)
        if self.open:
            try:
                if self.pwm_l[num] is not None:
                    if pwm_c == 'period':
                        return self.pwm_l[num].period
                    elif pwm_c == 'compare':
                        return self.pwm_l[num].cmp
                    elif pwm_c == 'onpercentage':
                        return self.pwm_l[num].GetDutyCycle()
                    elif pwm_c == 'frequency':
                        return self.pwm_l[num].GetFrequency()
                    elif pwm_c == 'midinote':
                        return self.pwm_l[num].GetMIDI()
                else:
                    return ("PWM %d hasn't been started!")%num
            except LostConnection:
                self.__lost_connection = True
            except ClosedPortException:
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @command("tell %d.num to %d.servo_c")
    def setServo(self, num, servo_c):
        num = int(num)
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if self.servo_l[num] is None:
                    self.servo_l[num] = Servo(num -1)
                if servo_c == 'start':
                    self.servo_l[num].Start()
                elif servo_c == 'stop':
                    self.servo_l[num].Stop()
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @command("set %d.num %m.servo to %n")
    def setServoVal(self, num, servo, val):
        num = int(num)
        val = int(val)

        if self.open:
            while self.__busy:
                pass
            self.__busy = True
            try:
                if self.servo_l[num] is None:
                    self.servo_l[num] = PWM(num -1)
                if servo == 'minimumAngle':
                    self.servo_l[num].changeAngles(val, self.servo_l[num].max_angle)
                elif servo == 'maximumAngle':
                    self.servo_l[num].changeAngles(self.servo_l[num].min_angle, val)
                elif servo == 'angle':
                    self.servo_l[num].SetAngle(val)
                elif servo == 'pulse':
                    self.servo_l[num].SetPulse(val)
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @command("set %d.num angle range from %n to %n")
    def setServoAngles(self, num, minangle, maxangle):
        num = int(num)
        minangle = float(minangle)
        maxangle = float(maxangle)

        if self.open:
            while self.__busy:
                pass
            self.__busy = True
            try:
                if self.servo_l[num] is None:
                    self.servo_l[num] = PWM(num -1)
                self.servo_l[num].changeAngles(minangle, maxangle)
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @reporter("servo %d.num %m.servo value")
    def getServo(self, num, servo):
        num = int(num)
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if self.servo_l[num] is not None:
                    if servo == 'minimumAngle':
                        val = self.servo_l[num].min_angle
                    elif servo == 'maximumAngle':
                        val = self.servo_l[num].max_angle
                    elif servo == 'angle':
                        val = self.servo_l[num].ReadAngle()
                    elif servo == 'pulse':
                        val = self.servo_l[num].ReadPulse()
                    self.__busy = False
                    return val
                else:
                    self.__busy = False
                    return ("Servo %d hasn't been started!"%num)
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @command("set NeoPixel %d.rows %d.columns")
    def SetNeoPixel(self, rows, columns):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if not self.neopixels.isRunning():
                    self.neopixels.Start()
                self.neopixels.SetPixel(int(rows), int(columns), self.__neopixelcolor)
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @command("draw NeoPixel row %d.rows")
    def NeoPixelRow(self, rows):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if not self.neopixels.isRunning():
                    self.neopixels.Start()
                    print('starting neopixels')
                print('drawing row %s to %s'%(rows, self.__neopixelcolor))
                self.neopixels.DrawRow(int(rows), self.__neopixelcolor)
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
        return True
    @command("draw NeoPixel column %d.columns")
    def NeoPixelColumn(self, columns):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if not self.neopixels.isRunning():
                    self.neopixels.Start()
                self.neopixels.DrawColumn(int(columns), self.__neopixelcolor)
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @command("set NeoPixel brightness to %d.rows")
    def NeoPixelDim(self, rows):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if not self.neopixels.isRunning():
                    self.neopixels.Start()
                self.neopixels.Dim(int(4 - int(rows)))
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @command("draw %n NeoPixels")
    def NeoPixelStripe(self, pixels):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                if not self.neopixels.isRunning():
                    self.neopixels.Start()
                self.neopixels.Stripe(int(pixels), self.__neopixelcolor, safety = True)
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @command("set NeoPixel color to %s")
    def setNeoPixelcolor1(self, color):
        if self.open:
            if hasattr(self.neopixels, color.capitalize()):
                self.__neopixelcolor = getattr(self.neopixels, color.capitalize())
            else:
                self.__neopixelcolor = 0
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @command("set NeoPixel color to %c")
    def setNeoPixelcolor2(self, color):
        if self.open:
            self.__neopixelcolor = self.neopixels.RGB_toHex(color)
        else:
            if not self.__connecting:
                self.reconnect_pisoc()


    @command("ask for range finder reading from pin %d.num")
    def rangeFinderRead(self, num):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                num = int(num)
                if num<=len(self.gpio_l):
                    if self.ranger_l[num][0] is None:
                        if self.ranger_l[num][1] is None:
                            self.ranger_l[num][0] = RangeFinder([self.gpio_l[num].port, self.gpio_l[num].pin], poll_frequency = 50)
                        else:
                            self.ranger_l[num] = RangeFinder([self.gpio_l[num].port, self.gpio_l[num].pin], [self.ranger_l[num][1][0], self.ranger_l[num][1][1]], poll_frequency = 50)

                    reading = self.ranger_l[num][0].ReadInches()
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()
    @command("set range finder trigger pin to %d.num")
    def rangeFinderSet(self, num):
        if self.open:
            while self.__busy:
                    pass
            self.__busy = True
            try:
                num = int(num)
                if num<=len(self.gpio_l):
                    self.ranger_l[num][1] = [self.gpio_l[num].port, self.gpio_l[num].pin]
                self.__busy = False
            except LostConnection:
                self.__busy = False
                self.__lost_connection = True
            except ClosedPortException:
                self.__busy = False
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    @reporter("%m.dist from range finder %d.num")
    def rangeFinderReport(self, dist, num):
        if self.open:
            try:
                num = int(num)
                if self.ranger_l[num][0] is not None:
                    raw_reading = self.ranger_l[num][0].raw
                    if raw_reading == PiSoC.BAD_PARAM or raw_reading == self.ranger_l[num][0].timeout:
                        return "I don't think the range finder is connected!!"
                    if dist == 'raw':
                        return raw_reading
                    elif dist == 'inches':
                        return self.ranger_l[num][0].inches
                    elif dist == 'centimeters':
                        return self.ranger_l[num][0].centimeters
                    elif dist == 'meters':
                        return self.ranger_l[num][0].meters
                else:
                    return ("ranger finder %d is not set up yet!"%num)
            except LostConnection:
                self.__lost_connection = True
            except ClosedPortException:
                self.__port_failure = True
        else:
            if not self.__connecting:
                self.reconnect_pisoc()

    def _isInt(self,test):
        try:
            return test%1 == 0
        except:
            return False

descriptor = Descriptor(
    name = "PiSoC",
    port = 42001,
    blocks = [
        Block('pisocClose','command', 'reset PiSoC'),
        Block('pinVal', 'reporter', 'pin %d.dig_num value'),
        Block('analogRead', 'reporter', 'analog pin %d.num %m.analog'),
        Block('joystickConfig', 'command', 'configure Joystick %d.num with X:%d.num Y:%d.num button:%d.dig_num'),
        Block('joystickRead_x_axis', 'reporter', 'Joystick %d.num X %m.analog'),
        Block('joystickRead_y_axis', 'reporter', 'Joystick %d.num Y %m.analog'),
        Block('joystickRead_select', 'predicate', 'Joystick %d.dig_num button pressed?'),
        Block('getPWM', 'reporter', 'PWM %d.num %m.pwm_c value'),
        Block('getServo', 'reporter', 'servo %d.num %m.servo value'),
        Block('pinState', 'predicate', 'pin %d.dig_num is on?'),
        Block('capsenseTouched', 'predicate', 'touch sensor %d.num touched?'),
        Block('pinBlink', 'command', 'blink pin %n'),
        Block('setPin', 'command', 'set pin %n to %m.state'),
        Block('pinConfig', 'command', 'set pin %n as %m.config'),
        Block('setPWMval', 'command', 'set PWM %d.num %m.pwm_c to %n'),
        Block('setPWM', 'command', 'tell PWM %d.num to %m.pwm'),
        Block('setServo', 'command', 'tell servo %d.num to %m.servo_c'),
        Block('setServoVal','command', 'set servo %d.num %m.servo to %n'),
        Block('setServoAngles', 'command', 'set servo %d.num angle range from %n to %n'),
        Block('SetNeoPixel', 'command', 'set NeoPixel %d.rows %d.columns'),
        Block('NeoPixelRow', 'command', 'draw NeoPixel row %d.rows'),
        Block('NeoPixelColumn', 'command', 'draw NeoPixel column %d.columns'),
        Block('NeoPixelDim', 'command', 'set NeoPixel brightness to %d.rows'),
        Block('NeoPixelStripe', 'command', 'draw %n NeoPixels'),
        Block('setNeoPixelcolor1', 'command', 'set NeoPixel color to %s'),
        Block('setNeoPixelcolor2', 'command', 'set NeoPixel color to %c'),
        Block('rangeFinderRead', 'command', 'ask for range finder reading from pin %d.num'),
        Block('rangeFinderSet', 'command', 'set range finder trigger pin to %d.num'),
        Block('rangeFinderReport', 'reporter', '%m.dist from range finder %d.num')


    ],
    menus = dict(
        dig_num = range(1, 23),
        num = [1, 2, 3, 4],
        config = ["output", "input", "pull up", "open drain"],
        servo = ["angle", "pulse", "minimumAngle", "maximumAngle"],
        servo_c = ["start", "stop"],
        dist = ["inches", "centimeters", "meters", "raw"],
        rows = [0, 1, 2, 3, 4],
        columns = [0, 1, 2, 3, 4, 5, 6, 7],
        state = ["on", "off"],
        baudr = [9600, 14400, 19200, 38400, 57600, 115200],
        analog = ["counts", "volts", "screen_x_axis", "screen_y_axis"],
        pwm =["start", "stop", "sleep", "wakeup", "refresh"],
        pwm_c = ["period", "compare", "onpercentage", "frequency", "midinote"]

    ),
)

extension = Extension(scratch_SoC, descriptor)

if __name__ == '__main__':
    PiSoC('PC', log_level = 'debug')
    extension.run_forever(debug=False)