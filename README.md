# Arduino-Uno-Power-Router-for-Load-Shedding-home-made-electricity-power-in-exceed
Or how to make a Power Router to optimize homemade electricity consumption and to prevent against public grid injection.

Are your tired to offer to your electricity grid provider any surplus of your own electricity you produce? 
Would you prefer either to switch on a machine to consume this surplus, or to switch off a (micro) invertor so that exceeded energy will only charge batteries?
And so this would be done automatically?

A Power Router is a device that will detect any injection in public power grid and help to prevent against it.
This manual explains step by step how to build this device, with an Arduino Uno rev.3.

The device detects the current at the immediate entrance of the public power grid, and the voltage on the line.
Any injection in the public power grid is detected by the shift between current and voltage:
When “in same way” we are consuming. When “in opposite way” we are injecting.

Then the power consumption is calculated and the Triac is driven by the device in order to prevent any injection. Parameters allow to fix limit values.

In addition a load shedding is possible that will switch on or off any device, depending of the consumption. For instance a SSR can cut off the wind turbine injection (and then it will charge batteries only) if consumption is near 0 (near injection in grid) and will be switched on again once consumption raises the power delivered by the wind turbine invertor.


