#! python
#
# Backend for Silicon Labs CP2110/4 HID-to-UART devices.
#
# This file is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2001-2015 Chris Liechti <cliechti@gmx.net>
# (C) 2019 Diego Elio Pettenò <flameeyes@flameeyes.eu>
#
# SPDX-License-Identifier:    BSD-3-Clause

import struct
import threading

try:
    import Queue
except ImportError:
    import queue as Queue

import hid  # hidapi

import serial
from serial.serialutil import SerialBase, SerialException, portNotOpenError, to_bytes, Timeout


class Serial(SerialBase):
    # This is not quite correct. AN343 specifies that the minimum
    # baudrate is different between CP2110 and CP2114, and it's halved
    # when using non-8-bit symbols.
    BAUDRATES = (300, 375, 600, 1200, 1800, 2400, 4800, 9600, 19200,
                 38400, 57600, 115200, 230400, 460800, 500000, 576000,
                 921600, 1000000)

    def __init__(self, *args, **kwargs):
        self._read_buffer = None
        self.logger = None
        super(Serial, self).__init__(*args, **kwargs)

    def open(self):
        self.logger = None
        if self._port is None:
            raise SerialException("Port must be configured before it can be used.")
        if self.is_open:
            raise SerialException("Port is already open.")

        self._read_buffer = Queue.Queue()

        self._hid_handle = hid.device()
        self._hid_handle.open(0x10c4, 0xea80)

        try:
            self._reconfigure_port(force_update=True)
        except:
            try:
                self._hid_handle.close()
            except:
                pass
            self._hid_handle = None
            raise
        else:
            self.is_open = True
            self._thread = threading.Thread(target=self._hid_read_loop)
            self._thread.setDaemon(True)
            self._thread.setName('pySerial CP2110 reader thread for {}'.format(self._port))
            self._thread.start()

    def close(self):
        self.is_open = False
        if self._thread:
            self._thread.join(7)  # XXX more than socket timeout
            self._thread = None
            # in case of quick reconnects, give the server some time
            time.sleep(0.3)
        self._hid_handle = None

    def _reconfigure_port(self, force_update=False):
        parity_value = None
        if self._parity == serial.PARITY_NONE:
            parity_value = 0x00
        elif self._parity == serial.PARITY_ODD:
            parity_value = 0x01
        elif self._parity == serial.PARITY_EVEN:
            parity_value = 0x02
        elif self._parity == serial.PARITY_MARK:
            parity_value = 0x03
        elif self._parity == serial.PARITY_SPACE:
            parity_value = 0x04
        else:
            raise ValueError('Invalid parity: {!r}'.format(self._parity))

        if self.rtscts:
            flow_control_value = 0x01
        else:
            flow_control_value = 0x00

        data_bits_value = None
        if self._bytesize == 5:
            data_bits_value = 0x00
        elif self._bytesize == 6:
            data_bits_value = 0x01
        elif self._bytesize == 7:
            data_bits_value = 0x02
        elif self._bytesize == 8:
            data_bits_value = 0x03
        else:
            raise ValueError('Invalid char len: {!r}'.format(self._bytesize))

        stop_bits_value = None
        if self._stopbits == serial.STOPBITS_ONE:
            stop_bits_value = 0x00
        elif self._stopbits == serial.STOPBITS_ONE_POINT_FIVE:
            stop_bits_value = 0x01
        elif self._stopbits == serial.STOPBITS_TWO:
            stop_bits_value = 0x01
        else:
            raise ValueError('Invalid stop bit specification: {!r}'.format(self._stopbits))
            
        configuration_report = struct.pack(
            '>BLBBBB',
            0x50,
            self._baudrate,
            parity_value,
            flow_control_value,
            data_bits_value,
            stop_bits_value)

        self._hid_handle.send_feature_report(configuration_report)

        enable_report = struct.pack('>BB', 0x41, 0x01)
        self._hid_handle.send_feature_report(enable_report)

    @property
    def in_waiting(self):
        return self._read_buffer.qsize()

    def reset_input_buffer(self):
        """Clear input buffer, discarding all that is in the buffer."""
        if not self.is_open:
            raise portNotOpenError
        self._hid_handle.write(b'\x43\x02')
        # empty read buffer
        while self._read_buffer.qsize():
            self._read_buffer.get(False)

    def read(self, size=1):
        if not self.is_open:
            raise portNotOpenError

        data = bytearray()
        try:
            timeout = Timeout(self._timeout)
            while len(data) < size:
                if self._thread is None:
                    raise SerialException('connection failed (reader thread died)')
                buf = self._read_buffer.get(True, timeout.time_left())
                if buf is None:
                    return bytes(data)
                data += buf
                if timeout.expired():
                    break
        except Queue.Empty:  # -> timeout
            pass
        return bytes(data)

    def write(self, data):
        if not self.is_open:
            raise portNotOpenError
        d = to_bytes(data)
        tx_len = length = len(d)
        while tx_len > 0:
            to_be_sent = min(tx_len, 0x3F)
            report = to_bytes([to_be_sent]) + d[:to_be_sent]
            self._hid_handle.write(report)
            
            d = d[to_be_sent:]
            tx_len = len(d)

    def _hid_read_loop(self):
        try:
            while self.is_open:
                try:
                    data = self._hid_handle.read(64)
                    data_len = data.pop(0)
                    assert data_len == len(data)
                    self._read_buffer.put(bytearray(data))
                except:
                    break
                if not data:
                    self._read_buffer.put(None)
        finally:
            self._thread = None
            if self.logger:
                self.logger.debug("read thread terminated")
