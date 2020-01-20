# Printy

Printy is an Open Source SLA 3D printer. 
Visit us at www.alch3my.com

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/logo.jpg" width="400" />

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F21.jpg" width="400" />

## Description
Alch3my is an open-source initiative for making the latest 3D printing technology accessible to everyone. Printy is an open-source SLA printer. We aim to democratize SLA technology with this fun and easy to build kit. Assembly takes 4 hours.

## Software Installation
### Arduino IDE
Get the latest Arduino IDE from https://www.arduino.cc/en/main/software and install.

### Printy Firmware
Get the latest Printy firmware from https://github.com/alch3my/printy/tree/master/Software/Firmware

Connect an Arduino Mega or equivalent microcontroller board via USB to your computer.

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/F1.jpg" width="400" />

Figure F1. Connect Arduino Mega

Load the firmware project "Marlin.ino" into the Arduino IDE

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/F2.jpg" width="400" />

Figure F2. Load Firmware

Ensure the "board", "processor", and "port" options reflect the connected Arduino Mega or equivalent. Press upload.

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/F3.jpg" width="400" />

Figure F3. Arduino IDE Configuration and Upload

Exit Arduino IDE once the upload is successful.

### Repetier Host
Get Repetier Host from https://www.repetier.com and install.

Use the following settings in "Printer Settings" to setup Printy.

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/R1.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/R2.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/R3.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/R4.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/R5.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/R6.jpg" width="400" />
Figures R1-6. Printer Settings

Note the "Port Number" should be the one your computer assign to the connected Arduino Mega or equivalent microcontroller.

### Slic3r Settings

Slic3r is a slicing tool typically used in FDM printers. Printy mimics the operation of an FDM by steering a laser beam across the print surface. Use the following settings to setup Slic3r. These settings will need to be tweaked for specific resins. Maker Juice G+ was extensively tested with these settings.

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S1.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S2.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S3.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S4.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S5.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S6.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S7.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S8.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S6.jpg" width="400" />
Figures S1-8. Slic3r Print Setting

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S9.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S10.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S11.jpg" width="400" />
Figures S9-11. Slic3r Filament Setting

<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S12.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S13.jpg" width="400" />
<img src="https://github.com/alch3my/printy/raw/master/Software/Instructions/Images/S14.jpg" width="400" />
Figures S12-14. Slic3r Printer Setting


Use the following "Start G-Code" in "Slic3r Printer Setting":

~~~~
G28 ; home all axes
G91 ; relative position
M155 S10 ; report temp every 10 seconds
G1 F300
G1 Z4.2 F300
G1 Z-4 F300
G1 Z4 F300
G1 Z-4 F300
G1 Z4 F300
G1 Z-4 F300
G1 Z4 F300
G1 Z-4 F300
G1 Z4 F300
G1 Z-4.1 F300
G1 Z10 F300
G1 Z-10 F20
G90 ; absolute position
G4 P1000 ; dwell in milliseconds
; end initial lift code
~~~~


Use the following "End G-Code":
~~~~
M104 S0 ; turn off temperature
M84     ; disable motors
~~~~

Use the following "Before Layer Change G-Code":
~~~~
; start lift code between layers
G91 ; relative position
G1 Z5 ; layer lift code
G4 P100 ; dwell in milliseconds
G1 Z-5
G90 ; absolute position
; end lift code between layers
~~~~


### Published by Alch3my LLC 

### CERN OHL v1.2 license whenever applicable.
