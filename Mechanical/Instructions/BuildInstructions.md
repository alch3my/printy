# Printy Build Instructions

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/logo.jpg" width="400" />

[product photo]

## Description
Alch3my is an open-source initiative for making the latest 3D printing technology accessible to everyone. Printy is an open-source SLA printer. We aim to democratize SLA technology with this fun and easy to build kit. Assembly takes 4 hours.

## Sourcing the Components
### Option 1: Everything from Alch3my
Purchase a complete kit. This takes all the work out of qualifying and sourcing. This option is perfect for a weekend project.
### Option 2: Only the Essentials
Do you have access to a laser cutter? Do you already have Arduino Mega, NEMA17 motor, stepper board, switches, etc from other projects? We will provide an essentials kit that includes Alch3my created hardware: the ResinShield and the aluminum sheet metal pieces.
### Option 3: From Scratch
Are you a hard core maker? Do you like to make, improve, and redistribute hardware? We will provide schematics, gerbers, firmware, CAD files and documentation at Alch3my.com or our GitHub page so you can base your next design on ours. Please observe the license and give credit to Alch3my.

## Build Instructions
### Overview
The Printy kit contains many laser cut pieces. Pay close attention to the orientation of pieces, since some of the pieces only fit together in the orientations described. All orientation directions assumes the printer is facing you. The __front__ is the side closest to you.; __left__ is on your left hand side, etc.
#### Setup
Begin by spreading all the pieces out for easy identification and to ensure you have all the necessary pieces. Please refer to the bill of materials (BOM).
#### Peel
Peel the protective tape from all pieces except for clear reservoir bottom. We do this to keep the reservoir protected as we build Printy. Note: the clear film on the reservoir has a pre-applied FEP sheet. 

![Figure G1](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/G1.jpg)

Figure G1. Peel the Protective Tape

#### T-Slot Joints
Printy is held together by T-slot joints. A bolt is inserted through one interface and a nut is inserted into a slot on the receiving end. __Do not overtighten!__

![Figure G2](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/G2.jpg)

Figure G2. Proper Installation for T-Slot Joint

### Building the Crane
The crane holds the print platform, all holes are symmetric for your convenience. The two holes are for attaching the endstop switch, use the holes on the __front-right__.

![Figure C1](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C1.jpg)

Figure C1. Assembly Orientation for the Crane

Attach the front and back piece of the crane to the top of the crane with 4 M3 nuts and 4 M3x14 screws each. A total of 8 M3 nuts and 8 M3x14 screws will be used.

![Figure C2](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C2.jpg)

Figure C2. Assemble the Crane

Attach the NEMA17 motor to top of the crane with 4 M3x10 screws. Wires should come out on the __right__ side.

![Figure C3](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C3.jpg)

Figure C3. Install NEMA 17 Motor

Press in 8 M3 brass press-fit nuts into the crane from the __bottom__, 2 more from the __top__

![Figure C4](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C4.jpg)

Figure C4. Brass Press-Fit Nuts

Insert the LMK8LUU bearings through top of the crane and attach with 4 M3x14 screws.

![Figure C5](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C5.jpg)

Figure C5. LMK8LUU Bearings

Attach the endstop switch with 2 M3x16 screw and 2 M3 nuts. The wires should points towards the __right__ side.

![Figure C6](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C6.jpg)

Figure C6. Endstop Switch

Enclose the motor and endstop wires in a sleeve, zip tie the ends

![Figure C7](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C7.jpg)

Figure C7. Wire Management

### Building the Front Drawer
The front drawer is designed to be easily removed. For your safety, there is a endstop switch halts the printer’s operation when either drawer is removed from the printer.

Press in 2 M3 brass press-fit nuts into drawer support. 

![Figure D1](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D1.jpg)

Figure D1. Drawer Support

Attach drawer support to the front piece with 2 M3x14 screws. Make sure the brass fit nut are on facing out and not sandwiched between the two pieces.

![Figure D2](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D1.jpg)

Figure D2. Install Drawer Support

Attach the drawer bottom to front with 1 M3x14 screw and 1 M3 nut.

![Figure D3](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D3.jpg)

Figure D3. Drawer Bottom

Attach the sides of the drawers with 2 M3x14 screws and 2 M3 nuts each. A total of 4 M3x14 screws and 4 M3 nuts will be used.

![Figure D4](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D4.jpg)

Figure D4. Drawer Sides

Secure the endstop to drawer front with 2 M3x16 screws. The endstop body and not the PCB should be flush against the wood. Note the end stop switch location.

![Figure D5](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D5.jpg)

Figure D5. End Stop

### Building the Back Drawer
The back drawer differs slightly from the front. The back drawer holds all the electronics. The openings for the Arduino USB and power ports.

![Figure D6](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D6.jpg)

Figure D6. Back Drawer __Check the wire colors before publishing__

Use 1 M3x14 screw and 1 M3 nut to secure galvanometer driver in place.

![Figure D7](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D7.jpg)

Figure D7. Galvo Driver

Install the stepper driver board onto Resin Shield. __Line up the EN pins__. Install the Resin Shield onto Arduino Mega and adhere the Arduino Mega to drawer with double sided foam tape. Stacking 2 layers of tape may be necessary depending on height of through hole components. Route end stop wire, galvo power cable, galvo signal cable. Finally, add 2 adhesive pads for future wire management.

![Figure D8](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D8.jpg)

Figure D8. Mounted Electronics

### Building the Frame
The frame holds everything together. Although most parts in this printer kit are built with symmetry in mind, some of the pieces of the frame are direction dependent. 

There are two slots on the carrier, they are for the end stop switches on the drawers from the previous step. In the top view, the slots should be __front-left__ and __rear-right__. 

![Figure F1](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F1.jpg)

Figure F1. Top View

Attach the 2 SHF8 (8mm linear rail holders) on to the carrier using 4 M5x20 screws and 4 M5 nuts. The flat on the SHF8 should point inside, it enables easier access to the clamping screws.

![Figure F2](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F2.jpg)

Figure F2. SHF8 Installed

Attach the Z end stop's adjustment M3x50 screw by pressing a M3 brass press-fit nut from the bottom in the __front-right__ hole, thread a M3 nut onto the adjustment screw first, and then thread the screw onto the nut. The nut is used to secure the adjustment screw. 

![Figure F3](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F3.jpg)

Figure F3. Z End-Stop Adjustment Installed

Install the two 8mm linear rails onto the SHF8 and install the crane into the rails. 

![Figure F4](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F4.jpg)

Figure F4. Carrier and Linear Rails Installed

Slide the frame back piece onto the carrier, use it as a guide to set the linear rail's height. Note there is a raised laser holder feature on the __left__ side of the frame back piece. Tighten the SHF8 locking screws, this can be adjusted later to make everything flush.

![Figure F5](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F5.jpg)

Figure F5. Carrier and Linear Rails Installed

Slide the frame front piece onto the carrier, make sure to orient the laser holder feature on the __left__ side. Now install 4 M3x14 screws and 4 M3 nuts to secure both frame pieces onto the carrier. Keep these screws slightly loose for now.

![Figure F6](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F6.jpg)

Figure F6. Frame Front and Back secured

Place the 8mm to 5mm flex coupler onto the 8mm ACME screw, adjust until the 8mm ACME does not cover any of the spring portion of the flex coupler (approximately 8mm of insertion depth). Tighten the 2 set screws on the flex coupler to secure. Install the assembly onto the frame top piece with 4 M3x14 and 4 M3 nuts.

![Figure F7](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F7.jpg)

Figure F7. Frame Top and ACME Assembly

Place frame top onto the main frame assembly. Install frame sides as well. Tighten the previously loose 4 frame screws.

![Figure F8](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F8.jpg)

Figure F8. Frame Held Together

Install 10 M3x14 screws and 10 M3 nuts to secure frame top. Adjust 8mm rails if necessary to make them flush at the top.

![Figure F9](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F9.jpg)

Figure F9. Frame Top

Install 8 M3x14 screws and 8 M3 nuts to secure the frame sides.

![Figure F10](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F10.jpg)

Figure F10. Frame Sides

Install 10 M3x14 screws and 10 M3 nuts to secrure the frame bottom. Note the frame bottom has a slot, it goes towards the __back__.

![Figure F11](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F11.jpg)

Figure F11. Frame Bottom

Secure the NEMA17 stepper motor's shaft onto the M5 to M8 flex coupler, use about 8mm of overlap. Make sure to put one set screw on the D portion of the shaft. 

![Figure F12](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F12.jpg)

Figure F12. Flex Coupler

Install both frame feet with 4 M3x14 and 4 M3 nuts. Also press in 4 M3 brass press-fit nuts onto the feet, the flange should be on the __inside__.

![Figure F13](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F13.jpg)

Figure 13. Feet

Install laser onto the laser holder piece with 4 M3x14 screws. Note the laser is pointing at the wider side of this piece.

![Figure F14](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F14.jpg)

Figure 14. Laser Holder

Press 2 M3 brass press fit nuts onto the galvo holder, then install the galvo using 3 M4x10 screws. Do your best to keep the galvanometer parallel to this piece. This sets the initial X and Y offset of the printer, we will calibrate this later. __Be careful with the mirrors.__

![Figure F15](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F15.jpg)

Figure 15. Galvanometers

Install the laser holder onto the frame using 2 M3x14 screws and 2 M3 nuts, then the galvo onto the frame using 2 M3x14 screws. Tidy up the wires using the provided wiring channels.


![Figure F16](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F16.jpg)

Figure 16. Galvanometers and Laser












