#!/usr/bin/env python3

"""
 This is the Python Banyan GUI that communicates with
 the Raspberry Pi Banyan Gateway

 Copyright (c) 2019 Alan Yorinks All right reserved.

 Python Banyan is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
 along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

"""
import argparse
import signal
from subprocess import run
import sys
import time

import pigpio
from python_banyan.gateway_base import GatewayBase

from s3_extend.gateways.MyGatewayBase import MyGatewayBase
from .sonar import Sonar
from .stepper import StepperMotor
import traceback

import zmq

f = open("/home/pi/rpi_gateway_log.txt", "w")


def logToFile(message, data=None):
    print(message, data)
    f.write(message + "\n")
    if data is not None:
        f.write(str(data))
    f.flush()


# see https://www.waveshare.com/wiki/AlphaBot2-Pi --> ws2812.py example
from rpi_ws281x import Adafruit_NeoPixel, Color

# LED strip configuration:
LED_COUNT = 4  # Number of LED pixels.
LED_PIN = 18  # GPIO pin connected to the pixels (must support PWM!).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 5  # DMA channel to use for generating signal (try 5)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False  # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL = 0


# noinspection PyAbstractClass
class RpiGateway(MyGatewayBase):
    """
    This class implements a Banyan gateway for the Raspberry Pi
    GPIO pins. It implements the Common Unified GPIO Message
    Specification.

    If pipgiod is not currently running, it will start it, and
    no backplane ip address was specified, a local backplane
    will be automatically started. If you specify a backplane
    ip address, you will need to start that backplane manually.
    """

    def __init__(self, *subscriber_list, **kwargs):
        """
        Initialize the class for operation
        :param subscriber_list: A list of subscription topics
        :param kwargs: Command line arguments - see bg4rpi()
                       at the bottom of this file.
        """

        logToFile('test RpiGateway')
        # attempt to instantiate pigpio
        try:
            logToFile('self.pi = pigpio.pi()')
            self.pi = pigpio.pi()
        except Exception as e:
            logToFile("error rpi_gate 0", traceback.format_exc())

        # if pigpiod is not running, try to start it
        #
        if not self.pi.connected:
            logToFile('\nAttempting to start pigpiod...')
            run(['sudo', 'pigpiod'])
            time.sleep(.5)
            self.pi = pigpio.pi()
            if not self.pi.connected:
                logToFile('pigpiod did not start - goodbye.')
                sys.exit()
            else:
                logToFile('pigpiod successfully started!')

        logToFile('pigpiod is running version: ', self.pi.get_pigpio_version())

        # variables to hold instances of sonar and stepper
        self.sonar = None
        self.stepper = None

        # a list of valid pin numbers
        # using a list comprehension to create the list
        self.gpio_pins = [pin for pin in range(2, 28)]

        # i2c handle supplied by pigpio when i2c open is called
        self.i2c_handle = None

        try:
            self.led_strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
            # Create NeoPixel object with appropriate configuration.
            # Intialize the library (must be called once before other functions).
            self.led_strip.begin()
        except Exception as e:
            logToFile("error could not start led strip", traceback.format_exc())
            self.pi.stop()
            self.clean_up()
            sys.exit(0)

        # initialize the parent
        super(RpiGateway, self).__init__(subscriber_list=subscriber_list,
                                         back_plane_ip_address=kwargs['back_plane_ip_address'],
                                         subscriber_port=kwargs['subscriber_port'],
                                         publisher_port=kwargs['publisher_port'],
                                         process_name=kwargs['process_name'],
                                         )

        # start the banyan receive loop
        try:
            self.receive_loop()
        except Exception as e:
            logToFile("error rpi_gate 1", traceback.format_exc())
            self.pi.stop()
            self.clean_up()
            sys.exit(0)
        except KeyboardInterrupt:
            self.pi.stop()
            self.clean_up()
            sys.exit(0)

    def init_pins_dictionary(self):
        """
        This method is called by the gateway_base parent class
        when it is initializing.
        """

        # build a status table for the pins
        for x in self.gpio_pins:
            # pull_state: one of ['pull_high', 'pull_low', 'pull_none']
            entry = {'mode': None, 'duty': None, 'freq': None, 'value': 0, 'pull_state': 'pull_none'}
            self.pins_dictionary[x] = entry

    def digital_write(self, topic, payload):
        """
        This method performs a digital write
        :param topic: message topic
        :param payload: {"command": "digital_write", "pin": “PIN”, "value": “VALUE”}
        """
        logToFile(f"setting pin {payload['pin']} value to {payload['value']}")

        self.pi.write(payload['pin'], payload['value'])

    def disable_digital_reporting(self, topic, payload):
        """
        This method disables digital input reporting for the selected pin.

        :param topic: message topic
        :param payload: {"command": "disable_digital_reporting", "pin": “PIN”, "tag": "TAG"}
        """
        entry = self.pins_dictionary[payload['pin']]
        entry['mode'] = None

    def i2c_read(self, topic, payload):
        """
        This method will perform an i2c read by specifying the i2c
        device address, i2c device register and the number of bytes
        to read.

        Call set_mode_i2c first to establish the pins for i2c operation.

        :param topic: message topic
        :param payload: {"command": "i2c_read", "pin": “PIN”, "tag": "TAG",
                         "addr": “I2C ADDRESS, "register": “I2C REGISTER”,
                         "number_of_bytes": “NUMBER OF BYTES”}
        :return via the i2c_callback method
        """

        # if no i2c handle has been assigned, get one
        if self.i2c_handle is None:
            self.i2c_handle = self.pi.i2c_open(1, payload['addr'], 0)

        # using the handle do the i2c read and retrieve the data
        value = self.pi.i2c_read_i2c_block_data(self.i2c_handle,
                                                payload['register'],
                                                payload['number_of_bytes'])
        # value index 0 is the number of bytes read.
        # the remainder of value is the a byte array of data returned
        # convert the data to a list
        value = list(value[1])

        # format the data for report to be published containing
        # the data received
        report = ', '.join([str(elem) for elem in value])
        payload = {'report': 'i2c_data', 'value': report}
        self.publish_payload(payload, 'from_rpi_gateway')

    def i2c_write(self, topic, payload):
        """
        This method will perform an i2c write for the i2c device with
        the specified i2c device address, i2c register and a list of byte
        to write.

        Call set_mode_i2c first to establish the pins for i2c operation.

        :param topic: message topic
        :param payload: {"command": "i2c_write", "pin": “PIN”, "tag": "TAG",
                         "addr": “I2C ADDRESS, "register": “I2C REGISTER”,
                         "data": [“DATA IN LIST FORM”]}
        """

        if self.i2c_handle is None:
            self.i2c_handle = self.pi.i2c_open(1, payload['addr'], 0)

        data = payload['data']

        self.pi.i2c_write_device(self.i2c_handle, data)

        # give the i2c device some time to process the write request
        time.sleep(.4)

    def set_mode_tone(self, topic, payload):
        pass

    def play_tone(self, topic, payload):
        """
        This method plays a tone on a piezo device connected to the selected
        pin at the frequency and duration requested.
        Frequency is in hz and duration in milliseconds.

        Call set_mode_tone before using this method.
        :param topic: message topic
        :param payload: {"command": "play_tone", "pin": “PIN”, "tag": "TAG",
                         “freq”: ”FREQUENCY”, duration: “DURATION”}
        """
        pin = int(payload['pin'])
        self.pi.set_mode(pin, pigpio.OUTPUT)

        frequency = int(payload['freq'])
        if frequency < 0:
            frequency = 0
        if frequency != 0:
            frequency = int((1000 / frequency) * 1000)
        tone = [pigpio.pulse(1 << pin, 0, frequency // 2),
                pigpio.pulse(0, 1 << pin, frequency // 2)]

        self.pi.wave_add_generic(tone)
        wid = self.pi.wave_create()

        if wid >= 0:
            self.pi.wave_send_repeat(wid)
            time.sleep(payload['duration'] / 1000)
            self.pi.wave_tx_stop()
            self.pi.wave_delete(wid)

    def set_pwm_frequency(self, topic, payload):
        """
        This method sets the pwm value for the selected pin.
        Call set_mode_pwm before calling this method.
        :param topic: message topic
        :param payload: {“command”: “set_pwm_frequency”, "pin": “PIN”,
                         "tag":”TAG”,
                          “frequency”: “VALUE in Hz”}
        """
        value = payload['frequency']
        if value < 0:
            value = 0

        logToFile(f"setting pin {payload['pin']} pwa frequency to {value}")

        # see https://abyz.me.uk/rpi/pigpio/python.html#set_PWM_frequency
        # only these allowed? (for sample rate 5:
        # maybe only allow these??
        # 8000  4000  2000 1600 1000  800  500  400  320
        #  250   200   160  100   80   50   40   20   10
        self.pi.set_PWM_frequency(payload['pin'], value)

    def pwm_write(self, topic, payload):
        """
        This method sets the pwm value for the selected pin.
        Call set_mode_pwm before calling this method.
        :param topic: message topic
        :param payload: {“command”: “pwm_write”, "pin": “PIN”,
                         "tag":”TAG”,
                          “value”: “VALUE”}
        """
        value = payload['value']
        if value < 0:
            value = 0
        elif payload['value'] > 255:
            value = 255

        self.pi.set_PWM_dutycycle(payload['pin'], value)

    def servo_position(self, topic, payload):
        """
        This method will set a servo's position in degrees.
        Call set_mode_servo first to activate the pin for
        servo operation.

        :param topic: message topic
        :param payload: {'command': 'servo_position',
                         "pin": “PIN”,'tag': 'servo',
                        “position”: “POSITION”}
        """
        pin = payload['pin']
        pulse_width = (payload['position'] * 10) + 600
        self.pi.set_servo_pulsewidth(pin, pulse_width)

    def set_mode_analog_input(self, topic, payload):
        """
        This method programs a PCF8591 AD/DA for analog input.
        :param topic: message topic
        :param payload: {"command": "set_mode_analog_input",
                         "pin": “PIN”, "tag":”TAG” }
        """

        # pin is used as channel number

        value = None
        i2c_handle = self.pi.i2c_open(1, 72, 0)
        pin = payload['pin']

        self.pi.i2c_write_byte_data(i2c_handle, 64 | (pin & 0x03), 0)
        time.sleep(0.1)
        for i in range(3):
            value = self.pi.i2c_read_byte(i2c_handle)

        self.pi.i2c_close(i2c_handle)

        # publish an analog input report
        payload = {'report': 'analog_input', 'pin': pin,
                   'value': value}
        self.publish_payload(payload, 'from_rpi_gateway')

    def set_mode_digital_input(self, topic, payload):
        """
        This method sets a pin as digital input.
        :param topic: message topic
        :param payload: {"command": "set_mode_digital_input", "pin": “PIN”, "tag":”TAG” }
        """
        pin = payload['pin']
        entry = self.pins_dictionary[pin]
        entry['mode'] = self.DIGITAL_INPUT_MODE

        self.pi.set_glitch_filter(pin, 20000)
        self.pi.set_mode(pin, pigpio.INPUT)
        # self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN) # we added an additional command to control this

        logToFile(f"setting up chang listener for pin {pin}")
        self.pi.callback(pin, pigpio.EITHER_EDGE, self.input_callback)

    def set_mode_digital_input_pull_state(self, topic, payload):
        """
        This method sets a pin as digital input.
        :param topic: message topic
        :param payload: {"command": "digital_read_pull_state", "pin": “PIN”, "pull_state": "pull_high | pull_low | pull_none", "tag":”TAG” }
        """
        pin = payload['pin']
        entry = self.pins_dictionary[pin]

        # TODO set mode input?
        entry['mode'] = self.DIGITAL_INPUT_MODE

        self.pi.set_glitch_filter(pin, 20000)
        self.pi.set_mode(pin, pigpio.INPUT)

        logToFile(f"setting pin {pin} to pull state {payload['pull_state']}")

        if payload['pull_state'] == 'pull_high':
            entry['pull_state'] = 'pull_high'
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            logToFile(f"setting pin {pin} to pull state 'high'")

        elif payload['pull_state'] == 'pull_low':
            entry['pull_state'] = 'pull_low'
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)

        else:
            entry['pull_state'] = 'pull_none'
            self.pi.set_pull_up_down(pin, pigpio.PUD_OFF)

    def set_mode_digital_output(self, topic, payload):
        """
        This method sets a pin as a digital output pin.
        :param topic: message topic
        :param payload: {"command": "set_mode_digital_output",
                         "pin": PIN, "tag":”TAG” }
        """
        self.pi.set_mode(payload['pin'], pigpio.OUTPUT)

    def set_mode_pwm(self, topic, payload):
        """
         This method sets a GPIO pin capable of PWM for PWM operation.
         :param topic: message topic
         :param payload: {"command": "set_mode_pwm", "pin": “PIN”, "tag":”TAG” }
         """
        self.pi.set_mode(payload['pin'], pigpio.OUTPUT)

    def set_rgb_led_color(self, topic, payload):
        """
        This method sets a GPIO pin capable of PWM for PWM operation.
        :param topic: message topic
        :param payload: {"command": "set_led_color", "led":0-3", r":0-255, "g":0-255, "b":0-255 }
        """
        _led_index = payload['led']
        _r = payload['r']
        _g = payload['g']
        _b = payload['b']

        logToFile(f"setting led {_led_index} to r: {_r}, g: {_g}, b: {_b}")

        self.led_strip.setPixelColor(_led_index, Color(_r, _g, _b))
        self.led_strip.show()

    def clear_rgb_led_color(self, topic, payload):
        """
        This method sets a GPIO pin capable of PWM for PWM operation.
        :param topic: message topic
        :param payload: {"command": "clear_rgb_led_color", "led":0-3", "r":0-255, "g":0-255, "b":0-255 }
        """
        _led_index = payload['led']
        self.led_strip.setPixelColor(_led_index, Color(0, 0, 0))
        self.led_strip.show()

    def read_ir_key(self, topic, payload):
        """
        This method reads the current pressed IR key
        :param topic: message topic
        :param payload: {"command": "read_ir_key"}
        """
        keyNum = get_IR_key()
        payload = {'report': 'ir_key', 'value': keyNum, 'timestamp': time.time()}
        self.publish_payload(payload, 'from_rpi_gateway')

    def set_mode_servo(self, topic, payload):
        """
        This method establishes a GPIO pin for servo operation.
        :param topic: message topic
        :param payload: {"command": "set_mode_servo", "pin": “PIN”, "tag":”TAG” }
        """
        pass

    def set_mode_sonar(self, topic, payload):
        """
        This method sets the trigger and echo pins for sonar operation.
        :param topic: message topic
        :param payload: {"command": "set_mode_sonar", "trigger_pin": “PIN”, "tag":”TAG”
                         "echo_pin": “PIN”"tag":”TAG” }
        """
        trigger = payload['trigger_pin']
        echo = payload['echo_pin']
        self.sonar = Sonar(self.pi, trigger, echo)
        self.receive_loop_idle_addition = self.read_sonar

    def read_sonar(self):
        """
        Read the sonar device and convert value to
        centimeters. The value is then published as a report.
        """
        sonar_time = self.sonar.read()
        distance = sonar_time / 29 / 2
        distance = round(distance, 2)
        payload = {'report': 'sonar_data', 'value': distance}
        self.publish_payload(payload, 'from_rpi_gateway')

    def set_mode_stepper(self, topic, payload):
        """
        This method establishes either 2 or 4 GPIO pins to be used in stepper
        motor operation.
        :param topic:
        :param payload:{"command": "set_mode_stepper", "pins": [“PINS”],
                        "steps_per_revolution": “NUMBER OF STEPS”}
        """
        self.stepper = StepperMotor(self.pi, payload['pins'][0],
                                    payload['pins'][1],
                                    payload['pins'][2],
                                    payload['pins'][3])

    def stepper_write(self, topic, payload):
        """
        Move a stepper motor for the specified number of steps.
        :param topic:
        :param payload: {"command": "stepper_write", "motor_speed": “SPEED”,
                         "number_of_steps":”NUMBER OF STEPS” }
        """
        if not self.stepper:
            raise RuntimeError('Stepper was not initialized')

        number_of_steps = payload['number_of_steps']
        if number_of_steps >= 0:
            for i in range(number_of_steps):
                self.stepper.do_clockwise_step()
        else:
            for i in range(abs(number_of_steps)):
                self.stepper.do_counterclockwise_step()

    def input_callback(self, pin, level, tick):
        """
        This method is called by pigpio when it detects a change for
        a digital input pin. A report is published reflecting
        the change of pin state for the pin.
        :param pin:
        :param level:
        :param tick:
        :return:
        """
        logToFile(f"setting pin {pin} read state: {level}")
        # payload = {'report': 'digital_input_change', 'pin': str(pin), 'level': str(level)}
        entry = self.pins_dictionary[pin]
        if entry['mode'] == self.DIGITAL_INPUT_MODE:
            payload = {'report': 'digital_input', 'pin': pin,
                       'value': level, 'timestamp': time.time()}
            self.publish_payload(payload, 'from_rpi_gateway')


def rpi_gateway():
    parser = argparse.ArgumentParser()
    parser.add_argument("-b", dest="back_plane_ip_address", default="None",
                        help="None or IP address used by Back Plane")
    parser.add_argument("-m", dest="subscriber_list", default="to_rpi_gateway", nargs='+',
                        help="Banyan topics space delimited: topic1 topic2 topic3")
    parser.add_argument("-n", dest="process_name", default="RaspberryPiGateway",
                        help="Set process name in banner")
    parser.add_argument("-p", dest="publisher_port", default='43124',
                        help="Publisher IP port")
    parser.add_argument("-s", dest="subscriber_port", default='43125',
                        help="Subscriber IP port")
    parser.add_argument("-t", dest="loop_time", default=".1",
                        help="Event Loop Timer in seconds")

    args = parser.parse_args()
    if args.back_plane_ip_address == 'None':
        args.back_plane_ip_address = None
    kw_options = {
        'back_plane_ip_address': args.back_plane_ip_address,
        'publisher_port': args.publisher_port,
        'subscriber_port': args.subscriber_port,
        'process_name': args.process_name,
        'loop_time': float(args.loop_time)}

    try:
        RpiGateway(args.subscriber_list, **kw_options)
    except Exception as e:
        logToFile("error rpi_gate 2", traceback.format_exc())
        sys.exit()
    except KeyboardInterrupt:
        sys.exit()


def signal_handler(sig, frame):
    logToFile('Exiting Through Signal Handler')
    raise KeyboardInterrupt


def get_IR_key():
    IR = 17
    if pigpio.read(IR) == 0:
        count = 0
        while pigpio.read(IR) == 0 and count < 200:  # 9ms
            count += 1
            time.sleep(0.00006)
        if (count < 10):
            return;
        count = 0
        while pigpio.read(IR) == 1 and count < 80:  # 4.5ms
            count += 1
            time.sleep(0.00006)

        idx = 0
        cnt = 0
        data = [0, 0, 0, 0]
        for i in range(0, 32):
            count = 0
            while pigpio.read(IR) == 0 and count < 15:  # 0.56ms
                count += 1
                time.sleep(0.00006)

            count = 0
            while pigpio.read(IR) == 1 and count < 40:  # 0: 0.56mx
                count += 1  # 1: 1.69ms
                time.sleep(0.00006)

            if count > 7:
                data[idx] |= 1 << cnt
            if cnt == 7:
                cnt = 0
                idx += 1
            else:
                cnt += 1
        #		print data
        if data[0] + data[1] == 0xFF and data[2] + data[3] == 0xFF:  # check
            logToFile(f"IR read: {data[2]}")
            return data[2]
        else:
            logToFile(f"IR invalid read, setting to -1")
            return -1

# listen for SIGINT
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

if __name__ == '__main__':
    # replace with name of function you defined above
    rpi_gateway()
