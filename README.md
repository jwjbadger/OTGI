# OTGI

This simple project acts as a bluetooth low-energy On-Board Diagnostics (OBD-2) reader capable of being read on mobile devices via the [app](https://github.com/jwjbadger/otgi-app).

The board is designed around the MCP-2561 CAN Tranceiver and ESP-32 WROOM platform. A simple buck converter powers the board from the car's battery accesible through OBD-2, which is then hooked to the MCP-2561, which interacts with the ESP32 via a level shifter and the CAN (TWAI) protocol. Circuit diagram to be included though the important bits are visible in the following image:

![otgi](https://github.com/user-attachments/assets/30d613f0-d4eb-4205-9fae-597af944ee3e)

The MCP-2561 should have power and ground hooked up to 5V with the STBY pin pulled low (to prevent the device from sleeping), the CANL and CANH pins connected to the OBD-2 port, and the TX and RX pins connected to the respective ESP32 pins. The SPLIT pin is not used for end termination due to the exposed automotive CAN bus already being very well terminated.

## Current Status

Currently, this project uses probes the mass air flow sensor multiple times per second with more infrequent probing of the Long/Short Term Fuel Trims (LTFT/STFT) to determine the approximate usage of fuel through the estimated current fuel ratio using the fuel trims and stoichiometric ratio of combustion of gas. Using numerical integration of this value, we determine the trip fuel usage and send this via BLE. Currently, the otgi-app project stores this data persistently to estimate remaining fuel, which is particularly useful when the car is on empty though it still has at least 3 gallons of gas remaining while also providing an estimated range using the mileage of a Tacoma 2011. 

The stored diagnostic's codes (DTC) are probed on startup though they are not currently sent over BLE (this is the next step in the construction of this project). The calculation of estimated fuel usage will also be shifted in the future to ignore the STFT/LTFT to prevent double counting correction and instead focus on tracking current vehicle speed to numerically integrate and more accurately calculate mileage.

Though the current state of this project is relatively simple, the associated library contained in the lib.rs and obd.rs files provides relatively extensive and expandable support for OBD-2 functions for any future endevours.
