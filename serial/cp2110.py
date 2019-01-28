#! python
#
# Backend for Silicon Labs CP2110/4 HID-to-UART devices.
#
# This file is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2019 Diego Elio Pettenò <flameeyes@flameeyes.eu>
#
# SPDX-License-Identifier:    BSD-3-Clause

import struct

import hid  # hidapi

import serial
from serial.serialutil import SerialBase, SerialException, portNotOpenError, to_bytes


class Serial(SerialBase):

    def __init__(self, *args, **kwargs):
        super(Serial, self).__init__(*args, **kwargs)

    def open(self):
        if self._port is None:
            raise SerialException("Port must be configured before it can be used.")
        if self.is_open:
            raise SerialException("Port is already open.")

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
        uart_status_report = self._hid_handle.get_feature_report(0x42, 7)
        _, tx_fifo, rx_fifo, error_status, break_status = struct.unpack(
            '>BHHBB', bytes(uart_status_report))

        return rx_fifo

    def read(self, size=1):
        if not self.is_open:
            raise portNotOpenError
        read = self._hid_handle.read(size + 1)

        return bytes(read[1:])

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
