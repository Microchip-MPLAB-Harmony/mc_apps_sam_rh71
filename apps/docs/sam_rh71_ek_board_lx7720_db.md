---
parent: Hardware Setup
title: Setup for quadrature encoder mode
has_children: false
has_toc: false
---

# SAM RH71 Evaluation Kit with LX7720 Daughter Board
## Setting up the hardware

| Project Name| Hardware |
|:---------|:---------:|
| sam_rh71_ek.X |<br>[SAM RH71 Evaluation Kit](https://ww1.microchip.com/downloads/en/DeviceDoc/SAMRH71F20-EK-Evaluation-Kit-User-Guide-DS50002910A.pdf)<br>[LX7720 Daughter Board](https://www.microsemi.com/product-directory/space-system-managers/3708-position-motor-controller-ic#resources)<br>[Hurst Motor with encoder](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNo/AC300022) |
|||
	
### Setting up [SAM RH71 Evaluation Kit](https://ww1.microchip.com/downloads/en/DeviceDoc/SAMRH71F20-EK-Evaluation-Kit-User-Guide-DS50002910A.pdf) and [LX7720 Daughter Board](https://www.microsemi.com/product-directory/space-system-managers/3708-position-motor-controller-ic#resources)

* Hardware Setup
    * Connect the debugger probe to J33
    * Connect the USB port on the board to the computer using a mini USB cable.
    * Connect the 3 motors phases to the LX board.
    * Connect the motor encoders A and B respectively to LX7720 BLI1 and BLI2. Connecter encoder VCC to LX7720 5V and encoder GND to LX7720 SGND.
    * LX7720 jumpers configuration
        * SM_EN to ON with J39
        * SCP forced high with J38
        * Connect the followings signals between SAMRH71-EK board and LX7720 board.

        | SAMRH71F20-EK        | LX7720                |
        |:-------------:       |:-------------:        |
        | PA4 (J24.8)          |   LD_IN_A_C   (J20.2) | 
        | PA0 (J24.7)          |   UD_IN_A_C   (J20.1) | 
        | PA5 (J24.6)          |   LD_IN_B_C   (J20.6) | 
        | PA1 (J24.9)          |   UD_IN_B_C   (J20.5) | 
        | PA6 (J24.17)         |   LD_IN_C_C   (J20.10)|
        | PA2 (J24.16)         |   UD_IN_C_C   (J20.9) |
        | PA09 (J24.3)         |   OC_FAULT    (J20.32)|
        | PC21 (J30.16) (TCLK9)|   SNS_OUT_A   (J20.3) |
        | PC28 (J30.9) (TCLK10)|   SNS_OUT_B   (J20.7) |
        | PB9 (J30.6) (TIOB9)  |   BLO2        (J20.36)|
        | PB10 (J30.7) (TIOA9) |   BLO1        (J20.35)|
        | PA11 (J24.5) (PCK0)  |   MOD_CLK     (J20.18)|
        | PA10 (J24.4) (PCK2)  |   CP_CLK      (J20.17)|
    * Add external buttons with pullup and debounce filter
        * Start/Stop switch : PA22 (J25.6)
        * Increase speed switch : PA23 (J25.3)
        * Decrease speed switch : PA24 (J25.4)

## Running The Application

1. Build and Program the application using the MPLAB X IDE.
2. Press Start/Stop switch (PA22) to start the motor, it will set LED0 indicating the control loop is running.
3. Increase speed with switch (PA23) and decrease speed with switch (PA24).
4. Stop the motor by pressing again PB0.
