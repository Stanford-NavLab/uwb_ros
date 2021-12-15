# UWB Calibration Package

The UWB Calibration Package is used to calibrate the EWB1000 RX and TX delays. The calibration is performed by collecting data from several ranging events between 3 or more UWBs at a known distance from each other. The difference between the averaged ranges and the known distances are then used to determine the position (and therefore timing) errors. The timing errors are used to find the RX and TX delays for each UWB. Finally, the calibration is completed by saving the RX and TX delays in the DW1000 firmware. 

# Calibration Procedure

The UWB calibration is performed by the steps outlined in the [Setup](##Setup) and [Execute](##Execute) sections.

## Setup

**_NOTE:_** This procedure assumes you have 3 at least UWBs loaded with the proper firmware. See the [uwb-firmware](https://github.com/Stanford-NavLab/uwb-firmware) repository for instructions on building/flashing the UWB firmware as well as S1 switch configuration.  

### Locate current calibration definitions

Locate the `calibration` directory in the [uwb-firmware](https://github.com/Stanford-NavLab/uwb-firmware) repository. Look for a file that matches the 4 digit address or the last four digits of the 16 digit address of each UWB. If none exist for the address, create a new one.

The values in the calibration files correspond to the [uwb-firmware](https://github.com/Stanford-NavLab/uwb-firmware) definitions found under `src/application/application_definitions.h`. When building and flashing the firmware be sure to copy/paste all of the calibration settings from the calibration file. After final calibration, make sure any changes are saved to the calibration files and uploaded to the repository.

```
//Antenna delay per S1 channel config
//S1 6-7 : OFF-OFF
#define TX_ANT_DELAY_00            16458
#define RX_ANT_DELAY_00            16458
//S1 6-7 : OFF-ON
#define TX_ANT_DELAY_01            16459
#define RX_ANT_DELAY_01            16459
//S1 6-7 : ON-OFF
#define TX_ANT_DELAY_10            16462
#define RX_ANT_DELAY_10            16462
//S1 6-7 : ON-ON
#define TX_ANT_DELAY_11            16460
#define RX_ANT_DELAY_11            16460
```

When building the [uwb-firmware](https://github.com/Stanford-NavLab/uwb-firmware), make sure the following definition is set to 1 so the calibration values will be applied.

```
#define SET_TXRX_DELAY 			1
```

### Prepare UWBs for calibration

On each UWB, turn on S1-4, apply power, and note the `TX DELAY` and `RX DELAY` antenna delay values that will be displayed on the LCD screen after several seconds. Also note the 4 or 16 digit address that will be displayed on the first line of the LCD after showing the antenna delay.

Configure the switches S1 6-7 as desired on all UWBs and setup two of them a fixed distance apart, antenna faces toward each other. For best results, and for all switch configurations, use the distance at which the UWB displays an RSL of -85dB. 

**_NOTE:_** The RSL value on the LCD display is the average value of the last 50 estimations. Give it a few seconds to settle after moving the UWB. 

Connect one of the UWBs to a linux machine via USB. Only one UWB needs to be connected to the linux machine so the other UWB can be powered elsewhere.

Download the repository containing this readme, open [catkin_ws/src/uwb_delay_calibration/calibration_node.py](catkin_ws/src/uwb_delay_calibration/calibration_node.py), and edit the data between `# User defined input - Begin` and `# User defined input - End` as follows:

Set the number of ranging measurements to take between each UWB. 

```
####################################################################################
# User defined input - Begin
####################################################################################

self.num_measurments = 1000
```

Enter the addresses of the UWBs. Each UWB's address will be displayed on its LCD screen. If only 4 characters are displayed for the address, pad the entry with 12 leading zeros.  

```
//16 character address
#                     "UWB1 ADDRESS    ", "UWB2 ADDRESS    ", "UWB3 ADDRESS    ", ...
self.uwb_addresses = ["109862205F8B5950", "112767325102118C", "1124103251021B20"]
```

OR 

```
//4 charcater address
#                     "UWB1 ADDRESS    ", "UWB2 ADDRESS    ", "UWB3 ADDRESS    ", ...
self.uwb_addresses = ["0000000000005950", "000000000000118C", "0000000000001B20"]
```

Enter the `TX_DELAY` and `RX_DELAY` displayed on each UWB's screen. This number should match the `TX DELAY` and `RX DELAY` values that will be displayed on the LCD screen at UWB powerup.

```
# TX ANTENNA DELAY
self.uwb_txdelay = { #units are in DW1000 device time
    self.uwb_addresses[0]:16455,  # UWB1
    self.uwb_addresses[1]:16453,  # UWB2
    self.uwb_addresses[2]:16456   # UWB3
}

# RX ANTENNA DELAY
self.uwb_rxdelay = { #units are in DW1000 device time
    self.uwb_addresses[0]:16455,  # UWB1
    self.uwb_addresses[1]:16453,  # UWB2
    self.uwb_addresses[2]:16456   # UWB3
}
```

Enter the measured distance (in millimeters) between the two UWB locations.

```
# Measured distance between UWBs
self.uwb_distance = 50000 #units are in mm

####################################################################################
# User defined input - End
####################################################################################

```


Next, data colleciton needs to be set up. On the linux machine, open a terminal, navigate to this repository's [catkin_ws/](catkin_ws/) folder, build the code, and source it.

```
catkin_make
```

```
source devel/setup.bash
```

Open two more terminals, navigate to the same directory and enter the same source command. In one terminal start `rosore`.

```
roscore
```

In another terminal, start the uwb_interface_node using a connected UWB's serial path.

```
rosrun uwb_interface uwb_interface_node -p /dev/ttyS28
```

**_NOTE:_** The `/dev/ttyS28` value may be differnt for different machines. See [catkin_ws/src/uwb_interface/README.md](catkin_ws/src/uwb_interface/README.md) for more information.

## Execute

In the last terminal, start the calibration_node.

```
rosrun uwb_delay_calibration calibration_node.py
```

As the uwb_interface node publishes ranging measurements, the calibration node will report the data collection progress for each pair. 

>>>
109862205F8B5950<=>1124103251021B20 rangings complete: 0/1000  
109862205F8B5950<=>112767325102118C rangings complete: 15/1000  
1124103251021B20<=>112767325102118C rangings complete: 0/1000            
>>>

Once the first pair has completed ranging, swap one of the active UWBs for the third UWB.

>>>
109862205F8B5950<=>1124103251021B20 rangings complete: 342/1000  
109862205F8B5950<=>112767325102118C rangings complete: 1000/1000    
1124103251021B20<=>112767325102118C rangings complete: 0/1000            
>>>

After data collection is complete for the second pair, swap UWBs to gather data for the final UWB pair.

>>>
109862205F8B5950<=>1124103251021B20 rangings complete: 1000/1000  
109862205F8B5950<=>112767325102118C rangings complete: 1000/1000    
1124103251021B20<=>112767325102118C rangings complete: 837/1000            
>>>

After data collection completes, the program will output calibration values for each UWB. More detailed calibration information will also be output to a `calibration-date-time.csv` file in the directory that the calibration_node was started in. 

>>>
Data Colleciton Complete, Calculating RX/TX delays           
Calibration Results:                                         
UWB: 109862205F8B5950                                          
    RX DELAY: 16455                                              
    TX DELAY: 16455     
UWB: 112767325102118C                                              
    RX DELAY: 16455                                             
    TX DELAY: 16455                                            
UWB: 1124103251021B20                                            
    RX DELAY: 16456                                           
    TX DELAY: 16456                                             
>>>

Program these values to each of the UWBs and verify the calibration by returning to the [Setup Section](##Setup) and using the output `RX DELAY` and `TX DELAY` as the values for `TX_ANT_DELAY` and `RX_ANT_DELAY`. 

For example: For S1 6-7 set to ON-ON, and UWB address `109862205F8B5950`, in the uwb-firmware `src/application/application_definitions.h` file set

```
//S1 6-7 ON-ON
#define TX_ANT_DELAY_111            16455
#define RX_ANT_DELAY_111            16455
```

and later in [catkin_ws/src/uwb_delay_calibration/calibration_node.py](catkin_ws/src/uwb_delay_calibration/calibration_node.py) set:

```
self.uwb_txdelay = { #units are in DW1000 device time
    self.uwb_addresses[0]:16455,  # UWB1
    self.uwb_addresses[1]:16455,  # UWB2
    self.uwb_addresses[2]:16456   # UWB3
}

self.uwb_rxdelay = { #units are in DW1000 device time
    self.uwb_addresses[0]:16455,  # UWB1
    self.uwb_addresses[1]:16455,  # UWB2
    self.uwb_addresses[2]:16456   # UWB3
}
```

The process should be repeated a few times for the best results. Once calibrated, the absolute difference between the programmed delays and calibration output should be 3 or less. If the numbers do not converge well after several attempts, you can look at the `calibration-date-time.csv` files generated to see which calibration values performed the best. 