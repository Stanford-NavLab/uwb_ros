#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from uwb_interface.msg import RangeEvent

from enum import Enum
from datetime import datetime
import sympy as sym
import numpy as np
import csv
import sys

# ANSI escape code for terminal output
CURSOR_UP_ONE = "\033[A"
ERASE_IN_LINE = "\033[K"

class State(Enum):
    COLLECT = 0,
    CALC = 1

class CalibrationNode():

    DWT_TIME_UNITS = 1.0/499.2e6/128.0 # = 15.65e-12 s
    SPEED_OF_LIGHT = 299702547.0 # m/s 

    def __init__(self):
        self.state = State.COLLECT

        #############################################################################################################################
        # User defined input - Begin
        #############################################################################################################################

        self.num_measurements = 1000

        #                     "UWB1 ADDRESS    ", "UWB2 ADDRESS    ", "UWB3 ADDRESS    ", ...
        self.uwb_addresses = ["000000000000198C", "0000000000001639", "0000000000000CD6"]

        # TX ANTENNA DELAY
        self.uwb_txdelay = { #units are in DW1000 device time
            self.uwb_addresses[0]:16449,  # UWB1 
            self.uwb_addresses[1]:16456,  # UWB2 
            self.uwb_addresses[2]:16460   # UWB3
        }

        # RX ANTENNA DELAY
        self.uwb_rxdelay = { #units are in DW1000 device time
            self.uwb_addresses[0]:16449,  # UWB1
            self.uwb_addresses[1]:16456,  # UWB2
            self.uwb_addresses[2]:16460   # UWB3
        }

        # Measured distance between UWBs
        self.uwb_distance = 50000   #units are in mm

        #############################################################################################################################
        # User defined input - End
        #############################################################################################################################

        self.uwb_ranges = {}
        self.anchor_range_reports = {}
        for i in range(0, len(self.uwb_addresses)):
            for j in range(i+1, len(self.uwb_addresses)):
                key = (self.uwb_addresses[i], self.uwb_addresses[j])
                # initialize empty arrays to store the UWB ranging measurements
                self.uwb_ranges[key] = []
                # Store number of measurements saved from each uwb acting as the anchor
                self.anchor_range_reports[key] = [0,0]

        rospy.init_node('uwb_calibration_node', anonymous=True)
        rospy.Subscriber("uwb/range", RangeEvent, self.range_callback)
        self.rate = rospy.Rate(10) # 10hz

        print("Initializing")


    def range_callback(self, data):
        keyv1 = (data.tag_address, data.anchor_address)
        keyv2 = (data.anchor_address, data.tag_address)

        key = None

        if keyv1 in self.uwb_ranges.keys():
            key = keyv1 
        elif keyv2 in self.uwb_ranges.keys(): 
            key = keyv2
            
        if key != None:
            if len(self.uwb_ranges[key]) < self.num_measurements:
                if data.anchor_address == key[0] and self.anchor_range_reports[key][0] < self.num_measurements/2:
                    self.anchor_range_reports[key][0] = self.anchor_range_reports[key][0] + 1
                    self.uwb_ranges[key].append(data.range_rsl)
                elif data.anchor_address == key[1] and self.anchor_range_reports[key][1] < self.num_measurements/2:
                    self.anchor_range_reports[key][1] = self.anchor_range_reports[key][1] + 1
                    self.uwb_ranges[key].append(data.range_rsl)
                

    # convert seconds to device time
    def convertsectodevicetime (self, seconds):
        return seconds/CalibrationNode.DWT_TIME_UNITS
        
    # convert device time to seconds
    def convertdevicetimetosec(self, device_time):
        return device_time * CalibrationNode.DWT_TIME_UNITS

    def run(self):

        while not rospy.is_shutdown():

            if(self.state == State.COLLECT):
                collection_complete = True
                
                for key, value in self.uwb_ranges.items():
                    if(len(value) < self.num_measurements):
                        collection_complete = False
                    
                    message = key[0] + "<=>" + key[1] + " rangings complete: " + str(len(self.uwb_ranges[(key[0], key[1])])) + "/" + str(self.num_measurements)
                    print(message)

                if collection_complete:  
                    self.state = State.CALC
                    print("Data Colleciton Complete, Calculating RX/TX delays")  
                else:
                    for i in range(0, len(self.uwb_ranges)):
                        sys.stdout.write(CURSOR_UP_ONE)
                        sys.stdout.write(ERASE_IN_LINE)

            if(self.state == State.CALC):

                average_range = {}
                distance_error = {} # mm
                time_error = {}     # seconds
                for key in self.uwb_ranges.keys():    
                    num_ranges = len(self.uwb_ranges[key])
                    avg_range = 0
                    for r in self.uwb_ranges[key]:
                        avg_range = avg_range + r
                    avg_range = avg_range/num_ranges            
                    average_range[key] = avg_range

                    distance_error[key] = avg_range - self.uwb_distance #self.uwb_distances[key] # mm
                    time_error[key] = distance_error[key]/1000.0/CalibrationNode.SPEED_OF_LIGHT

                #start construction A matrix
                m = len(self.uwb_ranges)
                n = len(self.uwb_addresses)
                A = np.zeros((m,n))
                b = np.zeros((m,1))
                row = 0
                for key in self.uwb_ranges.keys():
                    for k in range(0, len(key)):
                        index = np.where(np.array(self.uwb_addresses) == key[k])[0][0]
                        A[row][index] = 1

                    b[row] = time_error[key]

                    row = row + 1

                x = np.matmul(np.linalg.pinv(A),b)
                
                trx_error = []
                for i in range(0, len(self.uwb_addresses)):
                    trx_error.append(round(self.convertsectodevicetime(x[i]))) 

                key12 = (self.uwb_addresses[0], self.uwb_addresses[1])
                key13 = (self.uwb_addresses[0], self.uwb_addresses[2])
                key23 = (self.uwb_addresses[1], self.uwb_addresses[2])

                # Difference between calculated distance and measured distance between each UWB pair is equal to the sum of their antenna errors.
                # tx and rx antenna delay assumed to be identical for each UWB 
                e2 = -0.5*(time_error[key13] - time_error[key12] - time_error[key23])
                e1 = time_error[key12] - e2
                e3 = time_error[key23] - e2

                # convert error from standard time to DW1000 device time
                d1 = round(self.convertsectodevicetime(e1)) 
                d2 = round(self.convertsectodevicetime(e2))
                d3 = round(self.convertsectodevicetime(e3))

                rx_tuple = (d1, d2, d3)
                tx_tuple = (d1, d2, d3)  

                print("Calibration Results:")
                for i in range(0,len(self.uwb_addresses)):
                    print("UWB: " + self.uwb_addresses[i])
                    print("    RX DELAY: " + str(int(trx_error[i] + self.uwb_rxdelay[self.uwb_addresses[i]])))
                    print("    TX DELAY: " + str(int(trx_error[i] + self.uwb_txdelay[self.uwb_addresses[i]])))   

                date = str(datetime.date(datetime.now()))
                time = str(datetime.time(datetime.now()))

                #log results
                with open('calibration-' + date + '-' + time + '.csv' , 'w') as file:
                    writer = csv.writer(file)
                    writer.writerow(["UWB CALIBRATION RESULTS", "", "Date", date, "Time",  time])
                    row1 = ["UWB"]
                    row2 = ["RX DELAY"]
                    row3 = ["TX DELAY"]
                    row4 = [" "]
                    row5 = ["RX DELAY (PREVIOUS)"]
                    row6 = ["TX DELAY (PREVIOUS)"]
                    row7 = [" "]
                    for i in range(0,len(self.uwb_addresses)):
                        row1.append(self.uwb_addresses[i])
                        row2.append(trx_error[i] + self.uwb_rxdelay[self.uwb_addresses[i]])
                        row3.append(trx_error[i] + self.uwb_txdelay[self.uwb_addresses[i]])
                        row5.append(self.uwb_rxdelay[self.uwb_addresses[i]])
                        row6.append(self.uwb_txdelay[self.uwb_addresses[i]])
                    
                    row_list = [row1, row2, row3, row4, row5, row6, row7]
                    writer.writerows(row_list)

                    writer.writerow(["CALIBRATION DATA"])
                    writer.writerow(["Speed of light [m/s]", CalibrationNode.SPEED_OF_LIGHT])
                    row = ["UWB PAIR", "Measured Range [mm]", "Average Range [mm]", "Distance Error [mm]", "Time Error [s]", "Time Error [device time]"]
                    writer.writerow(row)
                    for key in self.uwb_ranges.keys():
                        row = []
                        row.append(','.join(key))
                        row.append(str(self.uwb_distance))
                        row.append(str(average_range[key]))
                        row.append(str(distance_error[key]))
                        row.append(str(time_error[key]))
                        row.append(str(self.convertsectodevicetime(time_error[key])))
                        writer.writerow(row)

                    writer.writerow([" "])
                    row_string = "UWB Pair Range Measurements (" + str(self.num_measurements) + " measurements each)"    
                    writer.writerow([row_string])    
                    row1 = []
                    row2 = []
                    for key in self.uwb_ranges.keys():
                        row1.append(key[0])
                        row2.append(key[1])
                    writer.writerow(row1)
                    writer.writerow(row2)
                    for i in range(0,self.num_measurements):
                        row = []
                        
                        for key in self.uwb_ranges.keys():
                            row.append(str(self.uwb_ranges[key][i]))
                        
                        writer.writerow(row)       

                break

            self.rate.sleep()
       

if __name__ == '__main__':
    try:
        #initialize the controller class
        calNode = CalibrationNode()
        #run the controller class
        calNode.run()
    except rospy.ROSInterruptException:
        pass
