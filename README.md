## Dynamixel SDK
The ROBOTIS Dynamixel SDK is a software development kit that provides Dynamixel control functions using packet communication. The API is designed for Dynamixel actuators and Dynamixel-based platforms. For more information on Dynamixel SDK, please refer to the e-manual below.

## Supported Programming Languages
DynamixelSDK supports various programming languages.
- **C**: *Dynamic library and source code of this library and examples
- **C++**: *Dynamic library and source code of this library and examples
- **Python**: Python module and examples
`.so: shared object on Linux`

## This SDK can be used with the BeagleBone Black and BeagleBone AI-64 under Linux armhf and aarch64, respectively.

```
in the Makefile files, add -fcommon to the end of the flags section(s)...
```

I currently am using the AX-12A but plan on using others as time passes. The AX-12A servos work with the U2D2, USB, and a 12v PSU. 

You, with the power `OFF`, can plug the U2D2 into the USB port on the BBB and/or the BBAI-64, compile on board, and then run their /c/example/protocol1.0/ files.

For the AX-12A servos, go to https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table and then use the table to enter in the text needed to support them.

...

### You can change these values in text (C) to alter the proximity of your movements...

```
#define DXL_MINIMUM_POSITION_VALUE      0
#define DXL_MAXIMUM_POSITION_VALUE      1023
```

Enjoy...
