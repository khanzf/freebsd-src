# athn driver

This driver was ported from OpenBSD, the exact copy of the code is from XXX

## Supported Drivers

The OpenBSD driver supports a wide range of chips, this only supports USB.

## Changes from OpenBSD

* OpenBSD assigns the card bus in athn_get_chipid, seemingly incorrectly. Modified to assign bus during attachment.
* Removing PCMCIA (Cardbus)

## Todo

Write function that polls for the NF, seems to be ath9k_hw_loadnf and/or related functions
AR_WMI_CMD_VAP_CREATE into vap creation
