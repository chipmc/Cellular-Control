/**********************************************************************************************************************
 * Copyright © 2016 by Geert Van Hecke
 *
 * Particle Electron pinout source code documentation
 *
 *                                                             ┌─────┐
 *                                             ┌───────┬─────┬─┤ USB ├───────┐
 *                                             │ ● Li+ │ • • │ └─────┘VUSB ● │
 *                                            ─┤ ○ VIN └─────┘         3V3 ○ ├─
 *                                            ─┤ ○ GND                 RST ○ ├─
 *                                            ─┤ ○ TX                 VBAT ○ ├─
 *                                            ─┤ ○ RX                  GND ○ ├─
 *                                            ─┤ ○ WKP  ◙     □     ◙   D7 ○ ├─
 *                                            ─┤ ○ DAC  MODE    RESET   D6 ○ ├─
 *                                            ─┤ ○ A5 ┌───────────────┐ D5 ○ ├─
 *                                            ─┤ ○ A4 │    Electron   │ D4 ○ ├─
 *                                            ─┤ ○ A3 │               │ D3 ○ ├─
 *                                            ─┤ ○ A2 │               │ D2 ○ ├─
 *                                            ─┤ ○ A1 │               │ D1 ○ ├─
 *                                            ─┤ ○ A0 │               │ D0 ○ ├─
 *                                            ─┤ ○ B5 │               │ C5 ○ ├─
 *                                            ─┤ ○ B4 │               │ C4 ○ ├─
 *                                            ─┤ ○ B3 │               │ C3 ○ ├─
 *                                            ─┤ ○ B2 │ µblox         │ C2 ○ ├─
 *                                            ─┤ ○ B1 └───────────────┘ C1 ○ ├─
 *                                            ─┤ ○ B0    ┌───┐          C0 ○ ├─
 *                                             │         │ ○ │               │
 *                                             │         └───┘           *   │
 *                                              \___________________________/
 *
 *
 * A0 - tmp36Pin              // Simple Analog temperature sensor
 * A2 - Pump Current Pin      // Pin for Voltage Sensor interrupt - Brown-White
 * A4 - Pump Contol Pin       // Pin for to turn the pump on - Brown
 * A7 - wakeUpPin             // This is the Particle Electron WKP pin
 * B1 - controlPowerPin       // Power to the contol panel - Orange-White
 * B2 - pumpOnPin             // Is the pump running - Green-White
 * B3 - lowLevelPin           // Low Level warning light - Green
 * B4 - AnyOnPin              // Not used on this board - Blue
 * B5 - tmp36Shutdwn          // Can turn off the TMP-36 to save energy
 * D4 - hardResetPin          // Power Cycles the Electron and the Carrier Board
 * D5 - userSwitch            // User switch with a pull-up resistor
 * D6 - donePin               // Pin the Electron uses to "pet" the watchdog
 * D7 - blueLED               // This LED is on the Electron itself

 * On the headers, GND is Blue-White and 3.3V is Orange

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
***********************************************************************************************************************/


/**
 * @file   doc.h
 * @author Geert Van Hecke
 * @date   12 April 2016
 * @brief  File containing the pinout documentation of a Particle Electron.
 *
 * Here typically goes a more extensive explanation of what the header defines.
 */
