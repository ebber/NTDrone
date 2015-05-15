#!/usr/bin/python
__author__ = 'erikbeitel'

import serial;
from time import sleep;
import glob;

class Arduino:
    bRate = 9600;
    port = None;
    ser = None;

    isConnected=False;

    def __init__(self, skip=False):
        text = "a";
        if (skip):
            self.connect();
            return;
        while (text != "exit"):
            text = raw_input("What do you want to do \n").lower();
            if (text == "connect"):
                self.connect();
                return;
                #cool shits we are connected, don't want ppl changing the buad rate or the device without disconneting
            elif (text == "set device"):
                self.setPort();
            elif (text == "set brate"):
                self.setBaudRate();
            elif (text == "exit"):
                break;
            else:
                print("Options are \n set device \n set bRate \n connect \n exit");

    def getAttachedDevices(self):  # Fix self to actually retrieve valid devices
        devices = dict();
        dList=glob.glob("/dev/tty.*")
        for i in range(0, len(dList)):
            devices.update({str(i+1):dList[i]});
        return devices;

    def getDeviceName(self):
        return (self.port) if (self.isConnected) else ("No device connected");


    def setPort(self):
        devices = self.getAttachedDevices()
        print("Avaliable devices \n")
        for key in devices:
            print(key + ":  " + devices[key] + "\n");
        try:
            choice = raw_input("Which device do you want? ");
            self.port = devices[choice];
        except KeyError:
            print("That is not a valid device number \n");

    def setBaudRate(self):
        self.bRate = input("Baud Rate? 9600 works well: ");

    def connect(self):
        if (self.port is None):
            self.setPort();
        if (self.bRate is None):
            self.setBaudRate();
        self.ser = serial.Serial(self.port, self.bRate);
        sleep(2);
        print("connected to " + self.port + " at a rate of " + str(self.bRate));
        self.isConnected=True;


    def write(self, toWrite):
        self.ser.write(toWrite);
        print self.ser.read(1);

    def read(self, lines=100):
        for i in range(0,lines):
            line = self.ser.readline();
            yield line;




arduino = Arduino();
choice = "a";
while (choice != "exit"):
       choice = str.lower(raw_input("What do you want to with "+ arduino.getDeviceName()+" \n"));
       if ("read" in choice):
            print arduino.read(choice[5:]);
       elif ("write" in choice):
           arduino.write(choice[6:]);
           print choice[6:];
       else:
           print(choice);
           print("Avialiable options are: \n read \n read [lines to read] \n write [String to write] \n exit \n");

