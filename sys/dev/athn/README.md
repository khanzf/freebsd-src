# athn driver

This driver was ported from OpenBSD, the exact copy of the code is from XXX

## Supported Drivers

The OpenBSD driver supports a wide range of chips, this only supports USB.

## Changes from OpenBSD

* OpenBSD assigns the card bus in athn_get_chipid, seemingly incorrectly. Modified to assign bus during attachment.
* Removing PCMCIA (Cardbus)
* Reworked shutdown code
* Replace msleep/wake with cv_wait
* Removed constant reallocation of Tx/Rx buffers in athn_usb_init
* Enabled 2-VAP limit, shifted WMI VAP code from init to per-vap start

## Bugs
Device HTC initialization does not consistently start for VMs.
Device shutdown also inconsistent, may require a look at how Linux does this.

## Todo

Write function that polls for the NF, seems to be ath9k_hw_loadnf and/or related functions
AR_WMI_CMD_VAP_CREATE into vap creation
Rx mbuf handling
Device VAP management
USB drain functionality
The raw_xmit and athn_transmit should not take the data off usc->usc_inactive, it should be done in athn_usb_tx
It might be possible to read the real device noise floor (NF), check the Linux driver implementation
