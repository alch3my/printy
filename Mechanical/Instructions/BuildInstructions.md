# Printy Build Instructions

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/logo.jpg" width="400" />

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F21.jpg" width="400" />

## Description
Alch3my is an open-source initiative for making the latest 3D printing technology accessible to everyone. Printy is an open-source SLA printer. We aim to democratize SLA technology with this fun and easy to build kit. Assembly takes 4 hours.

## Sourcing the Components
### Option 1: Everything from Alch3my
Purchase a complete kit. This takes all the work out of qualifying and sourcing. This option is perfect for a weekend project.
### Option 2: Only the Essentials
Do you have access to a laser cutter? Do you already have Arduino Mega, NEMA17 motor, stepper board, switches, etc from other projects? We will provide an essentials kit that includes Alch3my created hardware: the ResinShield and the aluminum sheet metal pieces. Download bill of materials and all the laser cutting DXF files from our website or GitHub account.
### Option 3: From Scratch
Are you a hard core maker? Do you like to make, improve, and redistribute hardware? We will provide schematics, gerbers, firmware, CAD files and documentation at Alch3my.com or our GitHub page so you can base your next design on ours. Please observe the license and give credit to Alch3my.

## Build Instructions
### Overview
The Printy kit contains many laser cut pieces. Pay close attention to the orientation of pieces, since some of the pieces only fit together in the orientations described. All orientation directions assumes the printer is facing you. The __front__ is the side closest to you.; __left__ is on your left hand side, etc.
#### Setup
Begin by spreading all the pieces out for easy identification and to ensure you have all the necessary pieces. Please refer to the Bill of Materials (BOM).
#### Tools Needed
Metric Allen wrenchs including 2mm, 2.5mm, 3mm. A small wrench. Clear tape. Rough sand paper (20-40 grit).
#### Spares
Ample spare screws and nuts are shipped with the kit. If more are needed, refer to the BOM for hardware specifications.
#### Peel
Peel the protective tape from all laser cut pieces.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/G1.jpg" width="400" />

Figure G1. Peel the Protective Tape

#### T-Slot Joints
Printy is held together by T-slot joints. A bolt is inserted through one interface and a nut is inserted into a slot on the receiving end. __Do not overtighten!__

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/G2.jpg" width="400" />

Figure G2. Proper Installation for T-Slot Joint

### Building the Crane
The crane holds the print platform, all holes are symmetric for your convenience. The two holes are for attaching the endstop switch, use the holes on the __front-right__.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C1.jpg" width="400" />

Figure C1. Assembly Orientation for the Crane

Attach the front and back piece of the crane to the top of the crane with 4 M3 nuts and 4 M3x14 screws each. A total of 8 M3 nuts and 8 M3x14 screws will be used.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C2.jpg" width="400" />

Figure C2. Assemble the Crane

Attach the NEMA17 motor to top of the crane with 4 M3x10 screws. Wires should come out on the __right__ side.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C3.jpg" width="400" />

Figure C3. Install NEMA 17 Motor

Press in 8 M3 brass press-fit nuts into the crane from the __bottom__, 2 more from the __top__

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C4.jpg" width="400" />

Figure C4. Brass Press-Fit Nuts

Insert the LMK8LUU bearings through top of the crane and attach with 4 M3x14 screws.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C5.jpg" width="400" />

Figure C5. LMK8LUU Bearings

Attach the endstop switch with 2 M3x16 screw and 2 M3 nuts. The wires should points towards the __right__ side.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C6.jpg" width="400" />

Figure C6. Endstop Switch

Enclose the motor and endstop wires in a sleeve, zip tie the ends

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/C7.jpg" width="400" />

Figure C7. Wire Management

### Building the Front Drawer
The front drawer is designed to be easily removed. For your safety, there is a endstop switch halts the printer’s operation when either drawer is removed from the printer.

Press in 2 M3 brass press-fit nuts into drawer support. 

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D1.jpg" width="400" />

Figure D1. Drawer Support

Attach drawer support to the front piece with 2 M3x14 screws. Make sure the brass fit nut are on facing out and not sandwiched between the two pieces.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D2.jpg" width="400" />

Figure D2. Install Drawer Support

Attach the drawer bottom to front with 1 M3x14 screw and 1 M3 nut.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D3.jpg" width="400" />

Figure D3. Drawer Bottom

Attach the sides of the drawers with 2 M3x14 screws and 2 M3 nuts each. A total of 4 M3x14 screws and 4 M3 nuts will be used.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D4.jpg" width="400" />

Figure D4. Drawer Sides

Secure the endstop to drawer front with 2 M3x16 screws. The endstop body and not the PCB should be flush against the wood. Note the end stop switch location.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D5.jpg" width="400" />

Figure D5. End Stop

### Building the Back Drawer
The back drawer differs slightly from the front. The back drawer holds all the electronics. The openings for the Arduino USB and power ports.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D6.jpg" width="400" />

Figure D6. Back Drawer

Use 1 M3x14 screw and 1 M3 nut to secure galvanometer driver in place.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D7.jpg" width="400" />

Figure D7. Galvo Driver

Install the stepper driver board onto Resin Shield. __Line up the EN pins__. Install the Resin Shield onto Arduino Mega and adhere the Arduino Mega to drawer with double sided foam tape. Stacking 2 layers of tape may be necessary depending on height of through hole components. Routing end stop wire, galvo power cable, galvo signal cables. Finally, add 2 adhesive pads for future wire management. A full wiring diagram can be found in a later section.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/D8.jpg" width="400" />

Figure D8. Mounted Electronics

### Building the Frame
The frame holds everything together. Although most parts in this printer kit are built with symmetry in mind, some of the pieces of the frame are direction dependent. 

There are two slots on the carrier, they are for the end stop switches on the drawers from the previous step. In the top view, the slots should be __front-left__ and __rear-right__. 

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F1.jpg" width="400" />

Figure F1. Top View

Attach the 2 SHF8 (8mm linear rail holders) on to the carrier using 4 M5x20 screws and 4 M5 nuts. The flat on the SHF8 should point inside, it enables easier access to the clamping screws.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F2.jpg" width="400" />

Figure F2. SHF8 Installed

Attach the Z end stop's adjustment M3x50 screw by pressing a M3 brass press-fit nut from the bottom in the __front-right__ hole, thread a M3 nut onto the adjustment screw first, and then thread the screw onto the nut. The nut is used to secure the adjustment screw. 

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F3.jpg" width="400" />

Figure F3. Z End-Stop Adjustment Installed

Install the two 8mm linear rails onto the SHF8 and install the crane into the rails. 

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F4.jpg" width="400" />

Figure F4. Carrier and Linear Rails Installed

Slide the frame back piece onto the carrier, use it as a guide to set the linear rail's height. Note there is a raised laser holder feature on the __left__ side of the frame back piece. Tighten the SHF8 locking screws, this can be adjusted later to make everything flush.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F5.jpg" width="400" />

Figure F5. Carrier and Linear Rails Installed

Slide the frame front piece onto the carrier, make sure to orient the laser holder feature on the __left__ side. Now install 4 M3x14 screws and 4 M3 nuts to secure both frame pieces onto the carrier. Keep these screws slightly loose for now.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F6.jpg" width="400" />

Figure F6. Frame Front and Back secured

Place the 8mm to 5mm flex coupler onto the 8mm ACME screw, adjust until the 8mm ACME does not cover any of the spring portion of the flex coupler (approximately 8mm of insertion depth). Tighten the 2 set screws on the flex coupler to secure. Install the assembly onto the frame top piece with 4 M3x14 and 4 M3 nuts.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F7.jpg" width="400" />

Figure F7. Frame Top and ACME Assembly

Place frame top onto the main frame assembly. Install frame sides as well. Tighten the previously loose 4 frame screws.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F8.jpg" width="400" />

Figure F8. Frame Held Together

Install 10 M3x14 screws and 10 M3 nuts to secure frame top. Adjust 8mm rails if necessary to make them flush at the top.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F9.jpg" width="400" />

Figure F9. Frame Top

Install 8 M3x14 screws and 8 M3 nuts to secure the frame sides.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F10.jpg" width="400" />

Figure F10. Frame Sides

Install 10 M3x14 screws and 10 M3 nuts to secrure the frame bottom. Note the frame bottom has a slot, it goes towards the __back__.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F11.jpg" width="400" />

Figure F11. Frame Bottom

Secure the NEMA17 stepper motor's shaft onto the M5 to M8 flex coupler, use about 8mm of overlap. Make sure to put one set screw on the D portion of the shaft. 

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F12.jpg" width="400" />

Figure F12. Flex Coupler

Install both frame feet with 4 M3x14 and 4 M3 nuts. Also press in 4 M3 brass press-fit nuts onto the feet, the flange should be on the __inside__.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F13.jpg" width="400" />

Figure 13. Feet

Install laser onto the laser holder piece with 4 M3x14 screws. Note the laser is pointing at the wider side of this piece.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F14.jpg" width="400" />

Figure 14. Laser Holder

Press 2 M3 brass press fit nuts onto the galvo holder, then install the galvo using 3 M4x10 screws. Do your best to keep the galvanometer parallel to this piece. This sets the initial X and Y offset of the printer, we will calibrate this later. __Be careful with the mirrors.__

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F15.jpg" width="400" />

Figure 15. Galvanometers

Install the laser holder onto the frame using 2 M3x14 screws and 2 M3 nuts, then the galvo onto the frame using 2 M3x14 screws. Tidy up the wires using the provided wiring channels.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F16.jpg" width="400" />

Figure 16. Galvanometers and Laser

Install the orange acrylic accent pieces onto the crane using 2x M3x10.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F17.jpg" width="400" />

Figure 17. Accent Piece

Install 2 M5 standoffs and 2 M5 nuts on the carrier. Next use clear tape to hold down the removable clear print plate onto the carrier. Note the print plate is not square, please make sure the wider side goes over the left and right edge of the hole in the carrier. This print plate also gives additional tension to the FEP film (future steps). 

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F18.jpg" width="400" />

Figure 18. Standoffs and print plate

### The Build Platform
Sand the bottom of the build platform with rough sandpaper for better print adhesion. All the holes on the sides are for drainage since the build platform is partially submerged in resin at the beginning of the print. Use 4 M5x20 screws to install the build platform on the crane. For future removal, simply loosen the 4 screws and pull the print and build platform away from the crane.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/P1.jpg" width="400" />

Figure P1. Build Platform

### Reusable Resin Reservoir
This step is crucial for the quality of print, make sure to proceed slowly and ensure a water-tight finish in the end.

1. Place a ring of double sided adhesive (provided) as high as possible along the vertical reservoir exterior wall.
2. Remove FEP film from backing. FEP is clear and slightly stretchy, the backing is made of PTFE and is not stretchy.
3. Place the aluminum resin reservoir on top of the FEP film and fold the FEP around the resevoir. Make sure the FEP film is tightly stretched and well adhered to the double sided tape.
4. Cover the FEP's vertical surface with single sided tape (provided).

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/R1.jpg" width="400" />

Figure R1. Completed Reservoir

Install the reservoir onto the frame using 2 M5x20 screws. The slightly thick print plate provides additional tension to the FEP film, make sure there are no bubbles or gaps between the FEP film and the print plate.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/R2.jpg" width="400" />

Figure R2. Installed Reservoir

The reservoir can be removed easily for cleaning. If worn out, the FEP can be replaced by repeating the above steps.

### Electrical Hookup Diagram
Connect all the electrical cables. Pay special attention to the following:
1. Switches: Signal (S) is green, Ground (-) is black, 5V (+) is red. Z end stop switch connects to the ZMIN port. The other door switches connect to the SAFE ports.
2. NEMA17 stepper motor: they come with 4 wires of various colors depending on manufacturer, if you later find the Z direction is reversed, just flip this connection backwards.
3. The laser module: the fan and laser power use identical connectors. The fan cable is longer and thinner.
4. The galvos: x-axis is the galvo installed vertically, it's responsible for the laser going left and right. Y-axis is the galvo installed horizontally, it's responsible for the laser going back and forth.

![Figure E1](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/E1.jpg)

Figure E1. Electrical Hookup Diagram

Use zip ties, anchors, and tape to neatly arrange the wires.

### Finishing Touches

Install the drawers onto the frame and secure using 4 M3x14 screws.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F19.jpg" width="400" />

Figure 19. Installed Drawers

Install the side accent windows and secure with clear tape.

<img src="https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F20.jpg" width="400" />

Figure 20. Windows

Slot in the front and back doors. Make sure it engages the safety switches with a satisfying click. With proper installation, if a user removes either door or drawers mid print, the print will halt and the laser will be immediately extinguished.

![Figure F21](https://github.com/alch3my/printy/raw/master/Mechanical/Instructions/Images/F21.jpg)

Figure 21. Fully Built

### Published by Alch3my LLC 

### CERN OHL v1.2 license whenever applicable.




