#!/usr/bin/env python3
'''
@file list_ports.py
@brief Lists active COM ports
@details From google search: vs code tasks.json input com ports extension "Command Variable"

@author Gerald Manweiler
@copyright @showdate "%Y" GWN Software. All rights reserved.
'''

# standard modules
import serial.tools.list_ports
import sys


def list_com_ports():
    '''
    @brief Lists active COM ports
    @details Called by tasks.json input rule to supply list of com ports for selection
    '''

    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No COM ports found")
        sys.exit(1)
    for port in ports:
        # The task will use each line as a selection option
        if port.vid and port.vid:
            print(port.device)


if __name__ == "__main__":
    list_com_ports()
