# $FreeBSD$

.PATH: ${SRCTOP}/sys/dev/athn

SYSDIR?=${SRCTOP}/sys
.include "${SYSDIR}/conf/kern.opts.mk"

KMOD     = athn
SRCS     = athn.c athnreg.h athnvar.h \
	   bus_if.h device_if.h \
	   opt_bus.h opt_rtwn.h opt_wlan.h

.PATH:	   ${SRCTOP}/sys/dev/athn/ar9271
SRCS	+= ar9271u.c ar9271u.h

#opt_rtwn.h:
#	@echo "#define RTWN_DEBUG 1" > ${.TARGET}
#.if ${MK_SOURCELESS_UCODE} == "no"
#	@echo "#define RTWN_WITHOUT_UCODE 1" >> ${.TARGET}
#.endif

EXPORT_SYMS=	YES

.include <bsd.kmod.mk>
