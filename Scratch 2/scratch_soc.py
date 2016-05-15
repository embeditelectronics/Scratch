#!/usr/bin/python
#-*- coding: utf-8

import future
from future import *
from blockext import *
from pisoc import *
from serial import SerialException
import functools
import threading
import Queue

def catch_exception(f):
        @functools.wraps(f)
        def func(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            except Exception as e:
                __unknown_failure = True
                print ('Caught an exception in %r'%f.__name__)
        return func

class ErrorCatcher(type):
    def __new__(cls, name, bases, dct):
        for m in dct:
            if hasattr(dct[m], '__call__'):
                dct[m] = catch_exception(dct[m])
        return type.__new__(cls, name, bases, dct)


class scratch_SoC:
    #__metaclass__ = ErrorCatcher
    def __init__(self):
        pass

    def _problem(self):
        if not pisoc_data.connected_:
            return "Can't connect to PiSoC! Cycling power should fix the problem!"
        return False

    @predicate("touch sensor %m.num touched?")
    def capsenseTouched(self, num):
        return pisoc_data.capsense_values[int(num)]

    @predicate("pin %m.dig_num is on?")
    def pinState(self, num):
       return pisoc_data.gpio_values[num]

    @command('blink pin %m.dig_num')
    def pinBlink(self, pin):
        command = lambda: DigitalPin.Toggle(pisoc_data.gpio_objects[pin])
        commands.put(command)

    @command("set pin %m.dig_num as %m.config")
    def pinConfig(self, pin, config):
        command = lambda: DigitalPin.Configure(pisoc_data.gpio_objects[pin], config)
        commands.put(command)

    @command("turn pin %m.dig_num %m.state")
    def setPin(self, pin, state):
        command = lambda: DigitalPin.Write(pisoc_data.gpio_objects[pin], state == 'on')
        commands.put(command)

    @reporter("pin %m.dig_num value")
    def pinVal(self, num):
        return int(pisoc_data.gpio_values[num])

    @reporter("analog pin %m.num %m.analog")
    def analogRead(self, num, analog):
        return pisoc_data.analog_values[str(analog)][int(num)]

    @command('%m.neo_cmd NeoPixel screen')
    def NeoPixelcmd(self, cmd):
        color = pisoc_data.color_ if cmd == 'Fill' else 0
        pisoc_data.shield_data.update({k : [color for i in range(8)] for k in pisoc_data.shield_data.iterkeys()})
        command = lambda: NeoPixelShield.Fill(pisoc_data.shield, color)
        commands.put(command)

    @command("%m.neo_cmd NeoPixel %m.rows %m.columns")
    def SetNeoPixel(self, cmd, rows, columns):
        color = pisoc_data.color_ if cmd == 'Fill' else 0
        pisoc_data.shield_data[int(rows)][int(columns)] = color
        command = lambda: NeoPixelShield.SetPixel(pisoc_data.shield, int(rows), int(columns), color)
        commands.put(command)

    @command("%m.neo_cmd NeoPixel row %m.rows")
    def NeoPixelRow(self, cmd, rows):
        color = pisoc_data.color_ if cmd == 'Fill' else 0
        pisoc_data.shield_data[int(rows)] = [color for i in range(8)]
        command = lambda: NeoPixelShield.DrawRow(pisoc_data.shield, int(rows), color)
        commands.put(command)

    @command("%m.neo_cmd NeoPixel column %m.columns")
    def NeoPixelColumn(self, cmd, columns):
        color = pisoc_data.color_ if cmd == 'Fill' else 0
        for row in range(5):
            pisoc_data.shield_data[row][int(columns)] = color
        command = lambda: NeoPixelShield.DrawColumn(pisoc_data.shield, int(columns), color)
        commands.put(command)

    @command("set NeoPixel brightness to %m.brightness")
    def NeoPixelDim(self, level):
        pisoc_data.brightness = int(level)
        command = lambda: NeoPixelShield.SetBrightness(pisoc_data.shield, pisoc_data.brightness)
        commands.put(command)

    @command("set NeoPixel color to %c")
    def setNeoPixelcolor2(self, color):
        pisoc_data.color_ = pisoc_data.shield.RGB_to_hex(color)

    @reporter("Controller joystick x %m.analog")
    def joystickRead_x_axis(self, choice):
        return pisoc_data.analog_values[choice][1]

    @reporter("Controller joystick y %m.analog")
    def joystickRead_y_axis(self, choice):
        return pisoc_data.analog_values[choice][2]

    @predicate("Controller button %m.controller %m.controller_action ?")
    def controller_buttons_read(self, choice, action):
        result = pisoc_data.controller_digital['pins'][choice]['value']
        return result if action == 'pressed' else not result

    @reporter("tone %m.num %m.tone value")
    def getTone(self, num, tone):
        return pisoc_data.pwms[int(num)][tone]

    @command("tell tone %m.num to %m.ranger")
    def setTone(self, num, servo_c):
        command = lambda: pisoc_data._tone_service(num = int(num), cmd = servo_c)
        commands.put(command)

    @command("set tone %m.num note to %m.notes")
    def setToneVal(self, num, val):
        command = lambda: pisoc_data._tone_service(cmd = 'note', num = int(num), note = str(val))
        commands.put(command)

    @command("set tone %m.num octave to %m.octaves")
    def setToneOctave(self, num, val):
        command = lambda: pisoc_data._tone_service(cmd = 'octave', num = int(num), octave = str(val))
        commands.put(command)

    @command("set tone %m.num volume to %m.volume")
    def setToneVolume(self, num, val):
        command = lambda: pisoc_data._tone_service(cmd = 'volume', num = int(num), volume = int(val))
        commands.put(command)

    @reporter("servo %m.num %m.servo value")
    def getServo(self, num, servo):
       return pisoc_data.pwms[int(num)][servo]

    @command("tell servo %m.num to %m.servo_c")
    def setServo(self, num, servo_c):
        command = lambda: pisoc_data._servo_service(num = int(num), cmd = servo_c)
        commands.put(command)

    @command("set servo %m.num %m.servo to %n")
    def setServoVal(self, num, servo, val):
        try:
            int(val)
            command = lambda: pisoc_data._servo_service(num = int(num), cmd = servo, val = int(val))
            commands.put(command)
        except:
            pass
        

    @command("set servo %m.num angle range from %n to %n")
    def setServoAngles(self, num, minangle, maxangle):
        try:
            int(minangle)
            int(maxangle)
            command = lambda: pisoc_data._servo_service(num = int(num), cmd = 'change_angles', minangle = int(minangle), maxangle = int(maxangle))
            commands.put(command)
        except:
            pass

    @command("%m.ranger distance measurement")
    def rangeFinderSet(self, ranger):
       pisoc_data.is_range_finding = ranger == "start"

    @reporter("ranger finder %m.dist")
    def rangeFinderReport(self, dist):
        return pisoc_data.range_finding[dist]

class PiSoC_Data(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.connected_ = True

        self.color_ = 0xff0000

        self.is_range_finding = False
        self.range_finding = {key: 0 for key in menu_items['dist']}
        #A list of GPIO in sequential order from [0, 0], [0, 1] ... [15, 6], [15, 7]. Only includes GPIO available in firmware.
        #gpio_objects = [j for i in [[DigitalPin(k, v, None) for v in PiSoC.GPIO[k]] for k in sorted(PiSoC.GPIO)] for j in i] 
        self.gpio_objects = {p: DigitalPin(5, int(p) - 1, None) if p!='LED' else DigitalPin(12,0,'output') for p in menu_items['dig_num']}

        #A dictionary of those GPIO that can be easily hashed into
        #self.gpio_objects = {key: gpio_objects[key - 1] if key<=len(gpio_objects) else None for key in menu_items['dig_num']}
        self.analog_objects = {i+1: AnalogPin(i) for i in range(PiSoC.ANALOG_IN_NUM)}

        self.analog_values = {choice_key: {key: 0 for key in menu_items['num']} for choice_key in menu_items['analog']}
        self.gpio_values = {key: False for key in menu_items['dig_num']}
        self.capsense_values = {key: False for key in menu_items['num']}

        self.shield_data = {k: [0 for i in range(8)] for k in range(5)}
        self.shield_data = {key: [0 for i in range(8)] for key in range(5)}
        self.brightness = 1

        #A list of PWM objects that can be PWMs, Tones, or Servos. 
        self.pwms = { key: {k: 0 for k in menu_items['pwm_c'] + menu_items['servo']} for key in range(1, PiSoC.PWM_NUM + 1)}

        tone_notes = {'note': "C"}
        tone_octave = {'octave': 6}
        tone_volume = {'volume': 5}

        for chan in self.pwms:
            self.pwms[chan]["Type"] = None
            self.pwms[chan].update(tone_notes)
            self.pwms[chan].update(tone_octave)
            self.pwms[chan].update(tone_volume)

        self.pwm_l = [None for i in range(PiSoC.PWM_NUM)]

        self.controller_digital = {
            'drive_mode':'pull_up',
            'initial_state': 1,
            'pins': {
                'joystick': {'pin': DigitalPin(6, 2, None), 'value': 0 },
                'select': {'pin': DigitalPin(6, 1, None), 'value': 0 },
                'A': {'pin': DigitalPin(12, 6, None), 'value': 0 },
                'B': {'pin': DigitalPin(12, 7, None), 'value': 0 },
                'C': {'pin': DigitalPin(15, 4, None), 'value': 0 },
                'D': {'pin': DigitalPin(15, 5, None), 'value': 0 } 
                }
            }        

        for button in self.controller_digital['pins']:
            self.controller_digital['pins'][button]['value'] = self.controller_digital['initial_state']
            self.controller_digital['pins'][button]['pin'].Configure(self.controller_digital['drive_mode'])
            self.controller_digital['pins'][button]['pin'].Write(self.controller_digital['pins'][button]['value'])

        self.initialize_data()

    def initialize_data(self):

        #Restart range finder.
        self.ranger = RangeFinder([6, 6], trigger = [6, 7], timeout_us= 50000, poll_frequency = 10)

        #Reconfigure JoyStick shield
        for button in self.controller_digital['pins']:
            self.controller_digital['pins'][button]['value'] = self.controller_digital['initial_state']
            self.controller_digital['pins'][button]['pin'].Configure(self.controller_digital['drive_mode'])
            self.controller_digital['pins'][button]['pin'].Write(self.controller_digital['pins'][button]['value'])

        #Restart and Recalibrate CapSense
        self.capsense_objects = {i+1: CapSense(i, threshold = 6) for i in range(PiSoC.CAPSENSE_SENSOR_NUM)}
        for pin in self.capsense_objects:
            self.capsense_objects[pin].Start()

        #Reset NeoPixelShield
        self.shield = NeoPixelShield()
        self.shield.Start()
        self.shield.SetBrightness(self.brightness)

        #Redraw Previous state of the NeoPixels
        for row in self.shield_data:
            column = 0
            for colors in self.shield_data[row]:
                if colors>0:
                    self.shield.SetPixel(row, column, colors)
                column+=1

        #reconfigure GPIO
        for gpio in self.gpio_objects:
            if self.gpio_objects[gpio].config_str is not None:
                self.gpio_objects[gpio].Configure(self.gpio_objects[gpio].config_str)
                if self.gpio_objects[gpio].config_str != 'input':
                    self.gpio_objects[gpio].Write(self.gpio_values[gpio])

        #Need to reconfigure PWMs here... TODO
        
        for pwm in self.pwms:
            if self.pwms[pwm]["Type"] == "Servo":

                #this isnt working right. No idea why....
                self.pwm_l[pwm - 1] = Servo(pwm - 1)
                self.pwm_l[pwm - 1].Start()
                self.pwms_l[pwm - 1].ChangeAngles(self.pwms[pwm]["minimumAngle"], self.pwms[pwm]["maximumAngle"])#this call appears to be the problem
                self.pwms_l[pwm - 1].SetAngle(self.pwms[pwm]["angle"])
            elif self.pwms[pwm]["Type"] == "Tone":
                self.pwm_l[pwm - 1].SetVolume(self.pwms[num]['volume'])
                self.pwm_l[pwm - 1].SetNote(self.pwms[num]['note'], self.pwms[num]['octave'])
                self.pwm_l[pwm - 1].Start()
        

    def run(self):
        while True:
            if self.connected_:
                try:
                    if not commands.empty():
                        item = commands.get()
                        item()
                        commands.task_done()
                    else:
                        self.update_data()
                except (SerialException, LostConnection, ClosedPortException):
                    self.connected_ = False
                    PiSoC.commChannel.ser.close()
                    time.sleep(0.1)
            else:
                try:
                    result = PiSoC.commChannel.reconnect()
                    if result:
                        self.connected_ = True
                        self.initialize_data()
                except:
                    pass
                time.sleep(1)
                
    def update_data(self):
        for pin in self.analog_objects:
            self.analog_values['counts'][pin] = self.analog_objects[pin].Read()
            volts = self.analog_objects[pin].ReadVolts(counts = self.analog_values['counts'][pin])
            screen_y = 55*volts - 140
            screen_x = 75*volts - 180
            self.analog_values['volts'][pin] = volts
            self.analog_values['screen_x'][pin] = screen_x
            self.analog_values['screen_y'][pin] = screen_y
        

        gpio_bitmap = self.gpio_objects[menu_items['dig_num'][0]].get_gpio_bitmap()

        for button in self.controller_digital['pins']:
            self.controller_digital['pins'][button]['value'] = not self.controller_digital['pins'][button]['pin'].Read(bitmap = gpio_bitmap)

        for pin in self.gpio_values:
            self.gpio_values[pin] = bool(self.gpio_objects[pin].Read(bitmap = gpio_bitmap))

        capsense_bitmap = self.capsense_objects[menu_items['num'][0]].get_register()
        for pin in self.capsense_values:
            self.capsense_values[pin] = bool(self.capsense_objects[pin].is_touched(bitmap = capsense_bitmap))

        if self.is_range_finding:
            if self.ranger.is_ready():
                inches = self.ranger.ReadInches()
                if self.ranger.raw == PiSoC.BAD_PARAM:
                    self.range_finding.update({k : "Distance sensor not connected!" for k in self.range_finding.iterkeys()})
                else:
                    self.range_finding['inches'] = inches
                    self.range_finding['centimeters'] = self.ranger.centimeters
                    self.range_finding['meters'] = self.ranger.meters
                    self.range_finding['raw'] = self.ranger.raw
        else:
            self.range_finding.update({k : "Distance sensor not started!" for k in self.range_finding.iterkeys()})




    def _servo_service(self, *args, **kwargs):

        num = kwargs.get("num", 1)
        cmd = kwargs.get("cmd", None)


        if cmd == 'start':
            if self.pwms[num]["Type"] == None:
                self.pwm_l[num - 1] = Servo(num - 1)
                self.pwm_l[num - 1].Start()
                self.pwms[num]["Type"] = "Servo"
                self.pwms[num]["angle"] = self.pwm_l[num - 1].ReadAngle()
                self.pwms[num]['maximumAngle'] = self.pwm_l[num - 1].max_angle
                self.pwms[num]['minimumAngle'] = self.pwm_l[num - 1].min_angle
        elif cmd == 'stop':
            if self.pwms[num]["Type"] == "Servo":
                self.pwm_l[num - 1].Stop()
                self.pwm_l[num - 1] = None
                self.pwms[num]["Type"] = None
        else:
            if self.pwms[num]["Type"] == "Servo":
                if cmd == 'angle':
                    angle = kwargs.get("val")
                    if angle >= self.pwms[num]['minimumAngle'] and angle <= self.pwms[num]['maximumAngle']:
                        self.pwms[num]['angle'] = angle
                        self.pwm_l[num - 1].SetAngle(angle)
                elif cmd == 'minimumAngle':
                    angle = kwargs.get("val")
                    self.pwm_l[num - 1].ChangeAngles(angle, self.pwm_l[num - 1].max_angle)
                    self.pwms[num]['minimumAngle'] = angle
                elif cmd == 'maximumAngle':
                    angle = kwargs.get("val")
                    self.pwm_l[num - 1].ChangeAngles(self.pwm_l[num - 1].min_angle, angle)
                    self.pwms[num]['maximumAngle'] = angle
                elif cmd == 'change_angles':
                    minangle = kwargs.get("minangle")
                    maxangle = kwargs.get("maxangle")
                    self.pwms[num]['maximumAngle'] = maxangle
                    self.pwms[num]['minimumAngle'] = minangle
                    self.pwm_l[num - 1].ChangeAngles(minangle, maxangle)
                


    def _tone_service(self, *args, **kwargs):

        num = kwargs.get("num", 1)
        cmd = kwargs.get("cmd", None)

        if cmd == 'start':
            if self.pwms[num]["Type"] == None:
                self.pwm_l[num - 1] = Tone(num - 1)
                self.pwm_l[num - 1].SetVolume(self.pwms[num]['volume'])
                self.pwm_l[num - 1].SetNote(self.pwms[num]['note'], self.pwms[num]['octave'])
                self.pwm_l[num - 1].Start()
                self.pwms[num]["Type"] = "Tone"
        elif cmd == 'stop':
            if self.pwms[num]["Type"] == "Tone":
                self.pwm_l[num - 1].Stop()
                self.pwm_l[num - 1] = None
                self.pwms[num]["Type"] = None
        else:
            if self.pwms[num]["Type"] == "Tone":
                val = kwargs.get(cmd)
                self.pwms[num][cmd] = val
                if cmd == 'volume':
                    self.pwm_l[num - 1].SetVolume(val)
                elif cmd == 'note':
                    self.pwm_l[num - 1].SetNote(val, self.pwms[num]['octave'])
        


class Scratch_Extension(threading.Thread):
    def __init__(self, extension):
        threading.Thread.__init__(self)
        self.extension = extension
    def run(self):
        self.extension.run_forever(debug = False)


menu_items = dict(
        dig_num = ["LED"] + [str(c) for c in range(1, 9)],
        num = list(range(1, 9)),
        octaves = list(range(0, 10)), 
        volume = list(range(0, 11)),
        config = ["output", "input", "pull_up", "pull_down"],
        servo = ["angle", "minimumAngle", "maximumAngle"],
        servo_c = ["start", "stop"],
        notes = ['A', 'A#', 'B', 'B#', 'C', 'C#', 'D', 'D#', 'E', 'E#', 'F', 'F#', 'G', 'G#'],
        dist = ["inches", "centimeters", "meters", "raw"],
        controller_action = ["pressed", "released"],
        rows = [0, 1, 2, 3, 4],
        brightness = [1, 2, 3, 4, 5],
        columns = [0, 1, 2, 3, 4, 5, 6, 7],
        state = ["on", "off"],
        ranger = ["start", "stop"],
        analog = ["counts", "volts", "screen_x", "screen_y"],
        pwm =["start", "stop", "sleep", "wakeup", "refresh"],
        pwm_c = ["period", "compare", "onpercentage", "frequency"],
        tone = ["note", "volume", "octave"],
        controller = ["joystick", "A", "B", "C", "D", "select"], 
        neo_cmd = ["Fill", "Clear"]

    )


if __name__ == '__main__':
    PiSoC('PC', log_level = 'debug')

    descriptor = Descriptor(
    name = "PiSoC",
    port = 42001,
    blocks = [

    Block('capsenseTouched', 'predicate', 'touch sensor %m.num touched?'),
    Block('pinState', 'predicate', 'pin %m.dig_num is on?'),
    Block('pinBlink', 'command', 'blink pin %m.dig_num'),
    Block('pinConfig', 'command', 'set pin %m.dig_num as %m.config'),
    Block('setPin', 'command', 'turn pin %m.dig_num %m.state'),
    Block('pinVal', 'reporter', 'pin %m.dig_num value'),
    Block('analogRead', 'reporter', 'analog pin %m.num %m.analog'), 
    Block('NeoPixelcmd', 'command', '%m.neo_cmd NeoPixel screen'),
    Block('SetNeoPixel', 'command', '%m.neo_cmd NeoPixel %m.rows %m.columns'),
    Block('NeoPixelRow', 'command', '%m.neo_cmd NeoPixel row %m.rows'),
    Block('NeoPixelColumn', 'command', '%m.neo_cmd NeoPixel column %m.columns'),
    Block('NeoPixelDim', 'command', 'set NeoPixel brightness to %m.brightness'),
    Block('setNeoPixelcolor2', 'command', 'set NeoPixel color to %c'),
    Block('joystickRead_x_axis', 'reporter', 'Controller joystick x %m.analog'),
    Block('joystickRead_y_axis', 'reporter', 'Controller joystick y %m.analog'),
    Block('controller_buttons_read', 'predicate', 'Controller button %m.controller %m.controller_action ?'),
    Block('getTone', 'reporter', 'tone %m.num %m.tone value'),
    Block('setTone', 'command', 'tell tone %m.num to %m.ranger'),
    Block('setToneVal', 'command', 'set tone %m.num note to %m.notes'),
    Block('setToneOctave', 'command', 'set tone %m.num octave to %m.octaves'),
    Block('setToneVolume', 'command', 'set tone %m.num volume to %m.volume'),
    Block('getServo', 'reporter', 'servo %m.num %m.servo value'),
    Block('setServo', 'command', 'tell servo %m.num to %m.servo_c'),
    Block('setServoVal', 'command', 'set servo %m.num %m.servo to %n'),
    Block('setServoAngles', 'command', 'set servo %m.num angle range from %n to %n'),
    Block('rangeFinderSet', 'command', '%m.ranger distance measurement'),
    Block('rangeFinderReport', 'reporter', 'ranger finder %m.dist')

    ],
    menus = menu_items
)
    

    extension = Extension(scratch_SoC, descriptor)
    commands = Queue.Queue(maxsize=100)
    scratch_handler = Scratch_Extension(extension)
    pisoc_data = PiSoC_Data()


    pisoc_data.daemon = True
    scratch_handler.daemon = True

    scratch_handler.start()
    pisoc_data.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        exit()


    