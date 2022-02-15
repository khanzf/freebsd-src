# $FreeBSD$

.PATH: ${SRCTOP}/sys/dev/athn/usb

SYSDIR?=${SRCTOP}/sys
.include "${SYSDIR}/conf/kern.opts.mk"

KMOD     = if_athn_usb
SRCS	 = if_athn_usb.c if_athn_usb.h \
	   bus_if.h device_if.h \
	   opt_bus.h opt_athn.h opt_usb.h opt_wlan.h usb_if.h usbdevs.h

.PATH: ${SRCTOP}/sys/dev/athn/ar9271
SRCS	+= ar9271u.c

#opt_athn.h:
#	@echo "#define RTWN_DEBUG 1" > ${.TARGET}
#.if ${MK_SOURCELESS_UCODE} == "no"
#	@echo "#define RTWN_WITHOUT_UCODE 1" >> ${.TARGET}
#.endif

.include <bsd.kmod.mk>
