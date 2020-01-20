
/**
 * Arduino Mega with ResinShield
 * Alch3my LLC
 * Open Hardware License OHL 1.3
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */


/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */




#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH true



#ifndef RESIN
	#define RESIN
#endif


  //#define RESIN_SEGMENTS_PER_SECOND 10




#define X_STEP_PIN         41
#define X_DIR_PIN          5
#define X_ENABLE_PIN       4
#define X_MIN_PIN          -1
#define X_MAX_PIN          -1

#define Y_STEP_PIN         39
#define Y_DIR_PIN          5
#define Y_ENABLE_PIN       4
#define Y_MIN_PIN          -1
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         24 
#define Z_DIR_PIN          22
#define Z_ENABLE_PIN       32
#define Z_MIN_PIN          25
#define Z_MAX_PIN          -1



#define E0_STEP_PIN        37
#define E0_DIR_PIN         5
#define E0_ENABLE_PIN      4


#define LASER_FIRING_PIN   2 
#define LASER_ENABLE_PIN   3
#define GALVO_SS_PIN       48 
#define CASE_OPEN_PIN      29 
#define CASE_OPEN2_PIN     27


#define DEBUG4				4
#define DEBUG5				5
#define DEBUG6				6
#define DEBUG7				7

#define HEATER_0_PIN     11   // EXTRUDER 1



#define TEMP_0_PIN         11   // A12
#define TEMP_1_PIN         -1   // ANALOG NUMBERING
#define TEMP_2_PIN         -1   // ANALOG NUMBERING


#define HEATER_BED_PIN   8   

#define TEMP_BED_PIN      14 


