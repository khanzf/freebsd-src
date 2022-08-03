/*	$OpenBSD: if_athn_usb.c,v 1.63 2021/11/22 10:17:14 mglocker Exp $	*/

/*-
 * Copyright (c) 2022 Farhan Khan <khanzf@gmail.com>
 * Copyright (c) 2011 Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2018 Stefan Sperling <stsp@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * USB front-end for Atheros AR9271 and AR7010 chipsets.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/linker.h>
#include <sys/kdb.h>
#include <sys/firmware.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>
#include <net/if_media.h>

#include <net80211/ieee80211_var.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include "usbdevs.h"

#include <dev/athn/athnreg.h>
#include <dev/athn/athnvar.h>

#define ECDA_NUM_AC 4
#include <dev/athn/usb/if_athn_usb.h>

#include <sys/sockio.h> // Delete this

MALLOC_DEFINE(M_ATHN_USB, "athn_usb", "athn usb private state");

#if 0
static const struct athn_usb_type {
	struct usb_devno	devno;
	u_int			flags;
} athn_usb_devs[] = {
	{{ USB_VENDOR_ACCTON, USB_PRODUCT_ACCTON_AR9280 },
	   ATHN_USB_FLAG_AR7010 },
	{{ USB_VENDOR_ACTIONTEC, USB_PRODUCT_ACTIONTEC_AR9287 },
	   ATHN_USB_FLAG_AR7010 },
	{{ USB_VENDOR_ATHEROS2, USB_PRODUCT_ATHEROS2_AR9271_1 }},
	{{ USB_VENDOR_ATHEROS2, USB_PRODUCT_ATHEROS2_AR9271_2 }},
	{{ USB_VENDOR_ATHEROS2, USB_PRODUCT_ATHEROS2_AR9271_3 }},
	{{ USB_VENDOR_ATHEROS2, USB_PRODUCT_ATHEROS2_AR9280 },
	   ATHN_USB_FLAG_AR7010 },
	{{ USB_VENDOR_ATHEROS2, USB_PRODUCT_ATHEROS2_AR9287 },
	   ATHN_USB_FLAG_AR7010 },
	{{ USB_VENDOR_AZUREWAVE, USB_PRODUCT_AZUREWAVE_AR9271_1 }},
	{{ USB_VENDOR_AZUREWAVE, USB_PRODUCT_AZUREWAVE_AR9271_2 }},
	{{ USB_VENDOR_AZUREWAVE, USB_PRODUCT_AZUREWAVE_AR9271_3 }},
	{{ USB_VENDOR_AZUREWAVE, USB_PRODUCT_AZUREWAVE_AR9271_4 }},
	{{ USB_VENDOR_AZUREWAVE, USB_PRODUCT_AZUREWAVE_AR9271_5 }},
	{{ USB_VENDOR_AZUREWAVE, USB_PRODUCT_AZUREWAVE_AR9271_6 }},
	{{ USB_VENDOR_DLINK2, USB_PRODUCT_DLINK2_AR9271 }},
	{{ USB_VENDOR_LITEON, USB_PRODUCT_LITEON_AR9271 }},
	{{ USB_VENDOR_NETGEAR, USB_PRODUCT_NETGEAR_WNA1100 }},
	{{ USB_VENDOR_NETGEAR, USB_PRODUCT_NETGEAR_WNDA3200 },
	   ATHN_USB_FLAG_AR7010 },
	{{ USB_VENDOR_PANASONIC, USB_PRODUCT_PANASONIC_N5HBZ0000055 },
	   ATHN_USB_FLAG_AR7010 },
	{{ USB_VENDOR_MELCO, USB_PRODUCT_MELCO_UWABR100 },
	   ATHN_USB_FLAG_AR7010 },
	{{ USB_VENDOR_VIA, USB_PRODUCT_VIA_AR9271 }}
};
#define athn_usb_lookup(v, p)	\
	((const struct athn_usb_type *)usb_lookup(athn_usb_devs, v, p))
#endif

static int		athn_usb_match(device_t);
static int		athn_usb_attach(device_t);
int		athn_usb_detach(device_t);
static int		athn_usb_attachhook(device_t);
int		athn_usb_open_pipes(struct athn_usb_softc *, device_t);
void		athn_usb_close_pipes(struct athn_usb_softc *);
int		athn_usb_alloc_rx_list(struct athn_usb_softc *);
void		athn_usb_free_rx_list(struct athn_usb_softc *);
int		athn_usb_alloc_tx_list(struct athn_usb_softc *);
void		athn_usb_free_tx_list(struct athn_usb_softc *);
int		athn_usb_alloc_tx_cmd(struct athn_usb_softc *);
void		athn_usb_free_tx_cmd(struct athn_usb_softc *);
//void		athn_usb_task(void *);
static void	athn_cmdq_cb(void *, int);
void		athn_usb_do_async(struct athn_usb_softc *,
		    void (*)(struct athn_usb_softc *, void *), void *, int);
void		athn_usb_wait_async(struct athn_usb_softc *);
int		athn_usb_load_firmware(struct athn_usb_softc *);
int		athn_usb_htc_msg(struct athn_usb_softc *, uint16_t, void *,
		    int);
int		athn_usb_htc_setup(struct athn_usb_softc *);
int		athn_usb_htc_connect_svc(struct athn_usb_softc *, uint16_t,
		    uint8_t, uint8_t, uint8_t *);
int		athn_usb_wmi_xcmd(struct athn_usb_softc *, uint16_t, void *,
		    int, void *);
int		athn_usb_read_rom(struct athn_softc *);
uint32_t	athn_usb_read(struct athn_softc *, uint32_t);
void		athn_usb_write(struct athn_softc *, uint32_t, uint32_t);
void		athn_usb_write_barrier(struct athn_softc *);
int		athn_usb_media_change(struct ifnet *);
void		athn_usb_next_scan(void *);
int		athn_usb_newstate(struct ieee80211com *, enum ieee80211_state,
		    int);
void		athn_usb_newstate_cb(struct athn_usb_softc *, void *);
void		athn_usb_newassoc(struct ieee80211_node *, int);
void		athn_usb_newassoc_cb(struct athn_usb_softc *, void *);
struct ieee80211_node *athn_usb_node_alloc(struct ieee80211vap *, const uint8_t mac[IEEE80211_ADDR_LEN]);
void		athn_usb_count_active_sta(void *, struct ieee80211_node *);
void		athn_usb_newauth_cb(struct athn_usb_softc *, void *);
int		athn_usb_newauth(struct ieee80211com *,
		    struct ieee80211_node *, int, uint16_t);
void		athn_usb_node_free(struct ieee80211com *,
		    struct ieee80211_node *);
void		athn_usb_node_free_cb(struct athn_usb_softc *, void *);
int		athn_usb_ampdu_tx_start(struct ieee80211com *,
		    struct ieee80211_node *, uint8_t);
void		athn_usb_ampdu_tx_start_cb(struct athn_usb_softc *, void *);
void		athn_usb_ampdu_tx_stop(struct ieee80211com *,
		    struct ieee80211_node *, uint8_t);
void		athn_usb_ampdu_tx_stop_cb(struct athn_usb_softc *, void *);
void		athn_usb_clean_nodes(void *, struct ieee80211_node *);
int		athn_usb_create_node(struct athn_usb_softc *,
		    struct ieee80211_node *);
int		athn_usb_node_set_rates(struct athn_usb_softc *,
		    struct ieee80211_node *);
int		athn_usb_remove_node(struct athn_usb_softc *,
		    struct ieee80211_node *);
void		athn_usb_rx_enable(struct athn_softc *);
int		athn_set_chan(struct athn_softc *, struct ieee80211_channel *,
		    struct ieee80211_channel *);
int		athn_usb_switch_chan(struct athn_softc *,
		    struct ieee80211_channel *, struct ieee80211_channel *);
void		athn_usb_updateedca(struct ieee80211com *);
void		athn_usb_updateedca_cb(struct athn_usb_softc *, void *);
void		athn_usb_updateslot(struct ieee80211com *);
void		athn_usb_updateslot_cb(struct athn_usb_softc *, void *);
int		athn_usb_set_key(struct ieee80211vap *, const struct ieee80211_key *);
void		athn_usb_set_key_cb(struct athn_usb_softc *, void *);
int			athn_usb_delete_key(struct ieee80211vap *, const struct ieee80211_key *);
void		athn_usb_delete_key_cb(struct athn_usb_softc *, void *);
void		athn_usb_bcneof(struct usbd_xfer *, void *);
void		athn_usb_swba(struct athn_usb_softc *);
void		athn_usb_tx_status(void *, struct ieee80211_node *);
void		athn_usb_rx_wmi_ctrl(struct athn_usb_softc *, uint8_t *, int);
void		athn_usb_intr(struct usb_xfer *, usb_error_t);
void		athn_usb_rx_radiotap(struct athn_softc *, struct mbuf *,
		    struct ar_rx_status *);
void		athn_usb_rx_frame(struct athn_usb_softc *, struct mbuf *);
//		    struct mbuf_list *);
void		athn_usb_rxeof(struct usbd_xfer *, void *);
void		athn_usb_txeof(struct usbd_xfer *, void *);
int		athn_usb_tx(struct athn_softc *, struct mbuf *,
		    struct ieee80211_node *);
void		athn_usb_start(struct ifnet *);
void		athn_usb_watchdog(struct ifnet *);
int		athn_usb_ioctl(struct ieee80211com *, u_long, void *);
int		athn_usb_init(struct athn_softc *);
static int		athn_usb_stop(device_t);
void		ar9271_load_ani(struct athn_softc *);
int		ar5008_ccmp_decap(struct athn_softc *, struct mbuf *,
		    struct ieee80211_node *);
int		ar5008_ccmp_encap(struct mbuf *, u_int, struct ieee80211_key *);

/* Shortcut. */
#define athn_usb_wmi_cmd(sc, cmd_id)	\
	athn_usb_wmi_xcmd(sc, cmd_id, NULL, 0, NULL)

/* Extern functions. */
void		athn_led_init(struct athn_softc *);
void		athn_set_led(struct athn_softc *, int);
void		athn_btcoex_init(struct athn_softc *);
void		athn_set_rxfilter(struct athn_softc *, uint32_t);
int		athn_reset(struct athn_softc *, int);
void		athn_init_pll(struct athn_softc *,
		    const struct ieee80211_channel *);
int		athn_set_power_awake(struct athn_softc *);
void		athn_set_power_sleep(struct athn_softc *);
void		athn_reset_key(struct athn_softc *, int);
int		athn_set_key(struct ieee80211com *, struct ieee80211_node *,
		    struct ieee80211_key *);
void		athn_delete_key(struct ieee80211com *, struct ieee80211_node *,
		    struct ieee80211_key *);
void		athn_rx_start(struct athn_softc *);
void		athn_set_sta_timers(struct athn_softc *);
void		athn_set_hostap_timers(struct athn_softc *);
void		athn_set_opmode(struct athn_softc *);
void		athn_set_bss(struct athn_softc *, struct ieee80211_node *);
int		athn_hw_reset(struct athn_softc *, struct ieee80211_channel *,
		    struct ieee80211_channel *, int);
void		athn_updateedca(struct ieee80211com *);
void		athn_updateslot(struct ieee80211com *);


/* FreeBSD additions */
//void athn_intr_rx_callback(struct usb_xfer *, usb_error_t);
void athn_intr_tx_callback(struct usb_xfer *, usb_error_t);
void athn_data_rx_callback(struct usb_xfer *, usb_error_t);
void athn_data_tx_callback(struct usb_xfer *, usb_error_t);

#define ATHN_USB_DEV(v, p) { USB_VPI(v, p, 0) }
static const STRUCT_USB_HOST_ID athn_devs[] = {
	ATHN_USB_DEV(USB_VENDOR_ATHEROS2, USB_PRODUCT_ATHEROS2_AR9271U)
};


#if 0
const struct cfattach athn_usb_ca = {
	sizeof(struct athn_usb_softc),
	athn_usb_match,
	athn_usb_attach,
	athn_usb_detach
};
#endif

#define ATHN_CONFIG_INDEX	0

static const struct usb_config athn_config_common[ATHN_N_TRANSFERS] = {
	[ATHN_TX_DATA] = {
		.type = UE_BULK,
		.endpoint = 0x01, // AR_PIPE_TX_DATA,
		.direction = UE_DIR_TX,
		.flags = {
			.short_xfer_ok = 1,
//			.force_short_xfer = 1,
			.pipe_bof = 1
		},
		.callback = athn_data_tx_callback,
		.bufsize = 0x200,
	},
	[ATHN_RX_DATA] = {
		.type = UE_BULK,
		.endpoint = 0x82, //AR_PIPE_RX_DATA,
		.direction = UE_DIR_RX,
		.flags = {
			.short_xfer_ok = 1,
//			.force_short_xfer = 1,
			.pipe_bof = 1
		},
		.callback = athn_data_rx_callback,
		.bufsize = 0x200,
	},
	[ATHN_RX_INTR] = {
		.type = UE_INTERRUPT,
		.endpoint = 0x83, // AR_PIPE_RX_INTR,
		.direction = UE_DIR_RX,
		.flags = {
			.short_xfer_ok = 1,
//			.force_short_xfer = 1,
			.pipe_bof = 1
		},
		.callback = athn_usb_intr,
		.bufsize = 0x40,
//		.callback = athn_intr_rx_callback,
//		.interval = USB_DEFAULT_INTERVAL,
	},
	[ATHN_TX_INTR] = {
		.type = UE_INTERRUPT,
		.endpoint = 0x04, //AR_PIPE_TX_INTR,
		.direction = UE_DIR_TX,
		.flags = {
//			.short_xfer_ok = 1,
//			.force_short_xfer = 1,
			.pipe_bof = 1
		},
		.callback= athn_intr_tx_callback,
		.bufsize = 512, // 200, //40,
		.timeout = ATHN_USB_CMD_TIMEOUT,
//		.interval = 1,
	}
};

/* FreeBSD additions */
void
athn_data_rx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	int actlen;
//	struct usb_page_cache *pc;
	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch(USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		/* XXX Fall through */
	case USB_ST_SETUP:
		//pc = usbd_xfer_get_frame(xfer, 0);
		usbd_xfer_get_frame(xfer, 0);
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);
		break;
	default: /* Error */
		break;
	}

	return;
}


/* FreeBSD additions */
void
athn_data_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	int actlen;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch(USB_GET_STATE(xfer)) {
	case USB_ST_SETUP:
		usbd_transfer_submit(xfer);
		break;
	case USB_ST_TRANSFERRED:
		break;
	default: /* Error */
		break;
	}

	return;
}

/* FreeBSD additions */
void
athn_intr_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	int actlen;
	struct athn_usb_softc *usc = usbd_xfer_softc(xfer);
	struct athn_usb_tx_data *data = &usc->tx_cmd;

//	struct athn_data *data;
	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch(USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:

		/* It seems like something else should go here, but not certain */
		/* Not implementing fallthrough for this */
//	msg->msg_id = htobe16(msg_id);
		break;
	case USB_ST_SETUP:
/*
		if (data == NULL) {
			printf("Empty pending queue?\n");
			// DPRINTF(SC, ATHN_DEBUG_XMIT,
			//	"%s: empty pending queue\n", __func__);
			return;
		}

		STAILQ_REMOVE_HEAD(&usc->tx_intr_queue, next);

*/

		usbd_xfer_set_frame_data(xfer, 0, data->buf, data->len);
//		usbd_xfer_set_frames(xfer, 2);
//		usbd_xfer_set_stall(xfer);
		usbd_transfer_submit(xfer);
	//	STAILQ_FOREACH(cur_data, &sc_tx_intr_active, next) {


	//	}



//		usbd_xfer_set_frame_data(xfer, 0, data->buf,
//			usbd_xfer_max_len(xfer));
//		usbd_transfer_submit(xfer);
		break;
	default: /* Error */
		break;
	}

	return;
}
/* End of FreeBSD additions */

static int
athn_usb_match(device_t self)
{
	struct usb_attach_arg *uaa = device_get_ivars(self);

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);
	if (uaa->info.bConfigIndex != ATHN_CONFIG_INDEX)
		return (ENXIO);
	if (uaa->info.bIfaceIndex != ATHN_IFACE_INDEX)
		return (ENXIO);

	return (usbd_lookup_id_by_uaa(athn_devs, sizeof(athn_devs), uaa));
}

static int
athn_usb_resume(device_t self)
{
	return 0;
}

static int
athn_usb_attach(device_t self)
{
	printf("athn_usb_attach\n");
	struct usb_attach_arg *uaa = device_get_ivars(self);
	struct athn_usb_softc *usc = device_get_softc(self);
	struct athn_softc *sc = &usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	int error;

	ic->ic_name = device_get_nameunit(self);

	usc->sc_udev = uaa->device;
	usc->sc_iface = uaa->iface;
	sc->sc_dev = self;

	//usc->flags = athn_usb_lookup(uaa->vendor, uaa->product)->flags; // OpenBSD
	usc->flags = 0x0;
	sc->flags |= ATHN_FLAG_USB;
#ifdef notyet
	/* Check if it is a combo WiFi+Bluetooth (WB193) device. */
	if (strncmp(product, "wb193", 5) == 0)
		sc->flags |= ATHN_FLAG_BTCOEX3WIRE;
#endif

	sc->ops.read = athn_usb_read;
	sc->ops.write = athn_usb_write;
	sc->ops.write_barrier = athn_usb_write_barrier;

	/* I think OpenBSD's athn_init is for PCI only */
	sc->sc_init = athn_usb_init;

	athn_usb_attach_private(usc, USB_GET_DRIVER_INFO(uaa));

	// OpenBSD below
	//usb_init_task(&usc->sc_task, athn_usb_task, sc, USB_TASK_TYPE_GENERIC);
	// FreeBSD side
	mtx_init(&sc->sc_mtx, ic->ic_name, MTX_NETWORK_LOCK, MTX_DEF);
	ATHN_CMDQ_LOCK_INIT(sc);
	TASK_INIT(&sc->cmdq_task, 0, athn_cmdq_cb, sc);

	if (athn_usb_open_pipes(usc, self) != 0)
		goto fail;

//	STAILQ_INIT(&usc->tx_intr_queue);

	/* Allocate xfer for firmware commands. */
	error = athn_usb_alloc_tx_cmd(usc);
	if (error)
		goto fail;

//	config_mountroot(self, athn_usb_attachhook);
	error = athn_usb_attachhook(self);
	if (error) {
		goto fail;
	}
	printf("Disabling auto-delete!!\n");
//	goto fail;
	return 0;

fail:
	printf("Detaching...\n");
	athn_usb_detach(self);
	return (ENXIO);
}

int
athn_usb_detach(device_t self)
{
	struct athn_usb_softc *usc = device_get_softc(self);
	struct athn_softc *sc = &usc->sc_sc;

	usbd_transfer_unsetup(usc->usc_xfer, ATHN_N_TRANSFERS);
	mtx_destroy(&sc->sc_mtx);
	printf("Destroy\n");
	return 0;
#if 0
	struct athn_usb_softc *usc = (struct athn_usb_softc *)self;
	struct athn_softc *sc = &usc->sc_sc;

	if (usc->sc_athn_attached)
		athn_detach(sc);

	/* Wait for all async commands to complete. */
	athn_usb_wait_async(usc);

	usbd_ref_wait(usc->sc_udev);

	/* Abort and close Tx/Rx pipes. */
	athn_usb_close_pipes(usc);

	/* Free Tx/Rx buffers. */
	athn_usb_free_tx_cmd(usc);
	athn_usb_free_tx_list(usc);
	athn_usb_free_rx_list(usc);

	return (0);
#endif
}

static void
athn_usb_vap_delete(struct ieee80211vap *vap)
{
	printf("athn_usb_vap_delete Unimplemented! Memory leak\n");
}

static struct ieee80211vap *
athn_usb_vap_create(struct ieee80211com *ic, const char name[IFNAMSIZ], int unit,
	enum ieee80211_opmode opmode, int flags,
	const uint8_t bssid[IEEE80211_ADDR_LEN],
	const uint8_t mac[IEEE80211_ADDR_LEN])
{
//	struct athn_softc *sc = ic->ic_softc;
	struct athn_vap *avp;
	struct ieee80211vap *vap;
	printf("athn_usb_vap_create\n");
//	struct ifnet *ifp;

	/* From zyd and rsu, not sure if this applies to athn */
//	if (!TAILQ_EMPTY(&ic->ic_vaps)) {
//		printf("VAP create returns null\n");
//		return (NULL);
//	}

	if (opmode == IEEE80211_M_MONITOR) {
		printf("monitor mode\n");
	}

	avp = malloc(sizeof(struct athn_vap), M_80211_VAP, M_WAITOK | M_ZERO);
	vap = &avp->vap;

	if(ieee80211_vap_setup(ic, vap, name, unit, opmode, flags, bssid) != 0) {
		printf("ieee80211_vap_setup failed\n");
		free(avp, M_80211_VAP);
		return (NULL);
	}

	/* override state transition machine */
	avp->newstate = vap->iv_newstate;
	vap->iv_newstate = athn_newstate;

//	vap->vp_set_key = athn_usb_set_key;
//	vap->vp_delete_key = athn_usb_delete_key;

/*
	vap->iv_update_beacon = ??
	vap->iv_reset = ??
	vap->iv_key_alloc = ??
	vap->iv_key_set = athn_usb_set_key;
	vap->iv_key_delete = athn_usb_delete_key;
	vap->iv_max_aid = ??

	vap->iv_ampdu_density = ??
	vap->iv_ampdu_rxmax = ??

	vap->iv_recv_mgmt = ??
*/

	ieee80211_vap_attach(vap, athn_usb_media_change, //ieee80211_media_change,
		ieee80211_media_status, mac);
	ic->ic_opmode = opmode;

	/* BUS-specific additions */
	//vap->iv_key_delete = sc->sc_key_delete;
	//vap->iv_key_set = sc->sc_key_set;

	return(vap);

	//ifp = vap->iv_ifp;
	//ifp->if_capabilities = ???;
}

int
athn_usb_attachhook(device_t self)
{
//	struct usb_attach_arg *uaa = device_get_ivars(self);
	struct athn_usb_softc *usc = device_get_softc(self);
	struct athn_softc *sc = &usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
//	struct athn_ops *ops = &sc->ops;
//	struct ieee80211com *ic = &sc->sc_ic;
//	struct ifnet *ifp = &ic->ic_if;
//	int s, i, error;
	int error;

	device_printf(sc->sc_dev, "Start of device!\n");

	ic->ic_softc = sc;

	/* Load firmware. */
	error = athn_usb_load_firmware(usc);
	if (error != 0) {
		device_printf(sc->sc_dev, "could not load firmware\n");
		return(ENXIO);
	}

	/* Setup the host transport communication interface. */
	error = athn_usb_htc_setup(usc);
	if (error != 0) {
		return(ENXIO);
	}

	/* We're now ready to attach the bus agnostic driver. */
	sc->sc_key_delete = athn_usb_delete_key;
	sc->sc_key_set = athn_usb_set_key;

	error = athn_attach(sc);
	if (error != 0) {
		printf("returning from the athn_attach...\n");
		return (ENXIO);
	}
	printf("=============================================== DONT FORGET TO DO THIS\n");
	usc->sc_athn_attached = 1;
	/* Override some operations for USB. */

	// Attach VAP-specific "stuff"
	ic->ic_vap_create = athn_usb_vap_create;
	ic->ic_vap_delete = athn_usb_vap_delete; // Not finished
	ic->ic_ioctl = athn_usb_ioctl;
//	ifp->if_start = athn_usb_start;
//	ifp->if_watchdog = athn_usb_watchdog;
	ic->ic_node_alloc = athn_usb_node_alloc;
//	ic->ic_newauth = athn_usb_newauth;
	ic->ic_newassoc = athn_usb_newassoc;
#ifndef IEEE80211_STA_ONLY
//	usc->sc_node_free = ic->ic_node_free;		// No OpenBSD equivalent?
//	ic->ic_node_free = athn_usb_node_free;		// No OpenBSD equivalent?
#endif
//	ic->ic_updateslot = athn_usb_updateslot;	
//	ic->ic_updateedca = athn_usb_updateedca;



//	vp->vp_set_key = athn_usb_set_key;
//	vp->vp_delete_key = athn_usb_delete_key;	// For VAP
//	ic->ic_set_key = athn_usb_set_key;			// For VAP
//	ic->ic_delete_key = athn_usb_delete_key;	// For VAP
#ifdef notyet
	ic->ic_ampdu_tx_start = athn_usb_ampdu_tx_start;	// For VAP
	ic->ic_ampdu_tx_stop = athn_usb_ampdu_tx_stop;		// For VAP
#endif
//	ic->ic_newstate = athn_usb_newstate;
#if 0
	ic->ic_media.ifm_change = athn_usb_media_change;
	timeout_set(&sc->scan_to, athn_usb_next_scan, usc);

	ops->rx_enable = athn_usb_rx_enable;
#endif

	/* Reset HW key cache entries. */
	int i; // XXX In the future move this integer back up
	for (i = 0; i < sc->kc_entries; i++)
		athn_reset_key(sc, i);

#if 0
	ops->enable_antenna_diversity(sc);
#endif

#ifdef ATHN_BT_COEXISTENCE
	/* Configure bluetooth coexistence for combo chips. */
	if (sc->flags & ATHN_FLAG_BTCOEX)
		athn_btcoex_init(sc);
#endif
	/* Configure LED. */
	athn_led_init(sc);

	if (bootverbose)
		ieee80211_announce(ic);
	return 0;
}

int
athn_usb_open_pipes(struct athn_usb_softc *usc, device_t dev)
{
//	struct usb_config *rtwn_config;
	struct athn_softc *sc = &usc->sc_sc;
//	const uint8_t iface_index = ATHN_IFACE_INDEX;
//	struct usb_endpoint *ep, *ep_end;
//#define ATHN_MAX_EPOUT 4 // Move to header file
//	uint8_t addr[ATHN_MAX_EPOUT];
	struct usb_attach_arg *uaa = device_get_ivars(dev);
	int error;
	int isize;
	uint8_t iface_index = ATHN_IFACE_INDEX;
	int ret = ENXIO;

	error = usbd_transfer_setup(uaa->device, &iface_index, usc->usc_xfer,
		athn_config_common, ATHN_N_TRANSFERS, sc, &sc->sc_mtx);
	if (error) {
		device_printf(dev, "could not allocate USB transfers, "
			"err=%s\n", usbd_errstr(error));
			ret = ENXIO;
			return (ret);
	}

	// OpenBSD side has this getting the max size manually
	//
	//ed = usbd_get_endpoint_descriptor(usc->sc_iface, AR_PIPE_RX_INTR);
	//isize = UGETW(ed->wMaxPacketSize);

	//
	//

	isize = 1 * 64; // Currently hard-coding this value
	usc->ibuflen = isize;
	usc->ibuf = malloc(isize, M_USBDEV, M_WAITOK);

//	ATHN_LOCK(sc);
	// Commenting out after wireshark analysis
//	usbd_transfer_start(usc->usc_xfer[ATHN_RX_INTR]);
//	usbd_transfer_start(usc->usc_xfer[ATHN_RX_DATA]);
//	usbd_transfer_start(usc->usc_xfer[ATHN_TX_INTR]);
//	usbd_transfer_start(usc->usc_xfer[ATHN_TX_DATA]);
//	ATHN_UNLOCK(sc);

	return 0;
#if 0
	usb_endpoint_descriptor_t *ed;
	int isize, error;

	error = usbd_open_pipe(usc->sc_iface, AR_PIPE_TX_DATA, 0,
	    &usc->tx_data_pipe);
	if (error != 0) {
		printf("%s: could not open Tx bulk pipe\n",
		    usc->usb_dev.dv_xname);
		goto fail;
	}

	error = usbd_open_pipe(usc->sc_iface, AR_PIPE_RX_DATA, 0,
	    &usc->rx_data_pipe);
	if (error != 0) {
		printf("%s: could not open Rx bulk pipe\n",
		    usc->usb_dev.dv_xname);
		goto fail;
	}

	ed = usbd_get_endpoint_descriptor(usc->sc_iface, AR_PIPE_RX_INTR);
	if (ed == NULL) {
		printf("%s: could not retrieve Rx intr pipe descriptor\n",
		    usc->usb_dev.dv_xname);
		goto fail;
	}
	isize = UGETW(ed->wMaxPacketSize);
	if (isize == 0) {
		printf("%s: invalid Rx intr pipe descriptor\n",
		    usc->usb_dev.dv_xname);
		goto fail;
	}
	usc->ibuf = malloc(isize, M_USBDEV, M_NOWAIT);
	if (usc->ibuf == NULL) {
		printf("%s: could not allocate Rx intr buffer\n",
		    usc->usb_dev.dv_xname);
		goto fail;
	}
	usc->ibuflen = isize;
	error = usbd_open_pipe_intr(usc->sc_iface, AR_PIPE_RX_INTR,
	    USBD_SHORT_XFER_OK, &usc->rx_intr_pipe, usc, usc->ibuf, isize,
	    athn_usb_intr, USBD_DEFAULT_INTERVAL);
	if (error != 0) {
		printf("%s: could not open Rx intr pipe\n",
		    usc->usb_dev.dv_xname);
		goto fail;
	}

	error = usbd_open_pipe(usc->sc_iface, AR_PIPE_TX_INTR, 0,
	    &usc->tx_intr_pipe);
	if (error != 0) {
		printf("%s: could not open Tx intr pipe\n",
		    usc->usb_dev.dv_xname);
		goto fail;
	}
 fail:
	if (error != 0)
		athn_usb_close_pipes(usc);
	return (error);
#endif
}

void
athn_usb_close_pipes(struct athn_usb_softc *usc)
{
#if 0
	if (usc->tx_data_pipe != NULL) {
		usbd_close_pipe(usc->tx_data_pipe);
		usc->tx_data_pipe = NULL;
	}
	if (usc->rx_data_pipe != NULL) {
		usbd_close_pipe(usc->rx_data_pipe);
		usc->rx_data_pipe = NULL;
	}
	if (usc->tx_intr_pipe != NULL) {
		usbd_close_pipe(usc->tx_intr_pipe);
		usc->tx_intr_pipe = NULL;
	}
	if (usc->rx_intr_pipe != NULL) {
		usbd_close_pipe(usc->rx_intr_pipe);
		usc->rx_intr_pipe = NULL;
	}
	if (usc->ibuf != NULL) {
		free(usc->ibuf, M_USBDEV, usc->ibuflen);
		usc->ibuf = NULL;
	}
#endif
}

int
athn_usb_alloc_rx_list(struct athn_usb_softc *usc)
{
	return 0;
#if 0
	struct athn_usb_rx_data *data;
	int i, error = 0;

	for (i = 0; i < ATHN_USB_RX_LIST_COUNT; i++) {
		data = &usc->rx_data[i];

		data->sc = usc;	/* Backpointer for callbacks. */

		data->xfer = usbd_alloc_xfer(usc->sc_udev);
		if (data->xfer == NULL) {
			printf("%s: could not allocate xfer\n",
			    usc->usb_dev.dv_xname);
			error = ENOMEM;
			break;
		}
		data->buf = usbd_alloc_buffer(data->xfer, ATHN_USB_RXBUFSZ);
		if (data->buf == NULL) {
			printf("%s: could not allocate xfer buffer\n",
			    usc->usb_dev.dv_xname);
			error = ENOMEM;
			break;
		}
	}
	if (error != 0)
		athn_usb_free_rx_list(usc);
	return (error);
#endif
}

void
athn_usb_free_rx_list(struct athn_usb_softc *usc)
{
#if 0
	int i;

	/* NB: Caller must abort pipe first. */
	for (i = 0; i < ATHN_USB_RX_LIST_COUNT; i++) {
		if (usc->rx_data[i].xfer != NULL)
			usbd_free_xfer(usc->rx_data[i].xfer);
		usc->rx_data[i].xfer = NULL;
	}
#endif
}

int
athn_usb_alloc_tx_list(struct athn_usb_softc *usc)
{
	return 0;
#if 0
	struct athn_usb_tx_data *data;
	int i, error = 0;

	TAILQ_INIT(&usc->tx_free_list);
	for (i = 0; i < ATHN_USB_TX_LIST_COUNT; i++) {
		data = &usc->tx_data[i];

		data->sc = usc;	/* Backpointer for callbacks. */

		data->xfer = usbd_alloc_xfer(usc->sc_udev);
		if (data->xfer == NULL) {
			printf("%s: could not allocate xfer\n",
			    usc->usb_dev.dv_xname);
			error = ENOMEM;
			break;
		}
		data->buf = usbd_alloc_buffer(data->xfer, ATHN_USB_TXBUFSZ);
		if (data->buf == NULL) {
			printf("%s: could not allocate xfer buffer\n",
			    usc->usb_dev.dv_xname);
			error = ENOMEM;
			break;
		}
		/* Append this Tx buffer to our free list. */
		TAILQ_INSERT_TAIL(&usc->tx_free_list, data, next);
	}
	if (error != 0)
		athn_usb_free_tx_list(usc);
	return (error);
#endif
}

void
athn_usb_free_tx_list(struct athn_usb_softc *usc)
{
#if 0
	int i;

	/* NB: Caller must abort pipe first. */
	for (i = 0; i < ATHN_USB_TX_LIST_COUNT; i++) {
		if (usc->tx_data[i].xfer != NULL)
			usbd_free_xfer(usc->tx_data[i].xfer);
		usc->tx_data[i].xfer = NULL;
	}
#endif
}

int
athn_usb_alloc_tx_cmd(struct athn_usb_softc *usc)
{
//	struct athn_softc *sc = &usc->sc_sc;
//	int error;

//	error = athn_usb_alloc_list(sc, usc->usc_cmd,
//		ATHN_USB_CMD_LIST_COUNT, ATHN_USB_TXCMDSZ);
//	if (error)
//		return (error);

	// STAILQ stuff

	struct athn_usb_tx_data *data = &usc->tx_cmd;

	data->sc = usc;	/* Backpointer for callbacks. */

	data->xfer = malloc(ATHN_USB_TXBUFSZ, M_USBDEV, M_NOWAIT | M_ZERO);
	if (data->xfer == NULL) {
		printf(": could not allocate xfer\n");
		return (ENOMEM);
	}

	data->buf = malloc(ATHN_USB_TXCMDSZ, M_USBDEV, M_NOWAIT | M_ZERO);
	if (data->buf == NULL) {
		printf(": could not allocate xfer buffer\n");
		return (ENOMEM);
	}
	return (0);
}

void
athn_usb_free_tx_cmd(struct athn_usb_softc *usc)
{
#if 0
	if (usc->tx_cmd.xfer != NULL)
		usbd_free_xfer(usc->tx_cmd.xfer);
	usc->tx_cmd.xfer = NULL;
#endif
}

//athn_usb_task(void *arg)
void
athn_cmdq_cb(void *arg, int pending)
{
	// Based on rtwn_cmdq_cb from if_rtwn_task.c
	printf("athn_cmdq_cb unimplemented\n");
#if 0
	struct athn_usb_softc *usc = arg;
	struct athn_usb_host_cmd_ring *ring = &usc->cmdq;
	struct athn_usb_host_cmd *cmd;
	int s;

	/* Process host commands. */
	s = splusb();
	while (ring->next != ring->cur) {
		cmd = &ring->cmd[ring->next];
		splx(s);
		/* Invoke callback. */
		cmd->cb(usc, cmd->data);
		s = splusb();
		ring->queued--;
		ring->next = (ring->next + 1) % ATHN_USB_HOST_CMD_RING_COUNT;
	}
	splx(s);
#endif
}

void
athn_usb_do_async(struct athn_usb_softc *usc,
    void (*cb)(struct athn_usb_softc *, void *), void *arg, int len)
{
#if 0
	struct athn_usb_host_cmd_ring *ring = &usc->cmdq;
	struct athn_usb_host_cmd *cmd;
	int s;

	if (ring->queued == ATHN_USB_HOST_CMD_RING_COUNT) {
		printf("%s: host cmd queue overrun\n", usc->usb_dev.dv_xname);
		return;	/* XXX */
	}
	
	s = splusb();
	cmd = &ring->cmd[ring->cur];
	cmd->cb = cb;
	KASSERT(len <= sizeof(cmd->data));
	memcpy(cmd->data, arg, len);
	ring->cur = (ring->cur + 1) % ATHN_USB_HOST_CMD_RING_COUNT;

	/* If there is no pending command already, schedule a task. */
	if (++ring->queued == 1)
		usb_add_task(usc->sc_udev, &usc->sc_task);
	splx(s);
#endif
}

void
athn_usb_wait_async(struct athn_usb_softc *usc)
{
#if 0
	/* Wait for all queued asynchronous commands to complete. */
	usb_wait_task(usc->sc_udev, &usc->sc_task);
#endif
}

int
athn_usb_load_firmware(struct athn_usb_softc *usc)
{
	struct athn_softc *sc = &usc->sc_sc;
	usb_device_request_t req;
	char *ptr;
	const struct firmware *fw;
	int mlen, error, size;
	uint32_t addr;

	error = 0;

	/* Determine which firmware image to load. */
	/*
	if (usc->flags & ATHN_USB_FLAG_AR7010) {
		dd = usbd_get_device_descriptor(usc->sc_udev);
		name = "athn-open-ar7010";
	} else {
		name = "athn-open-ar9271";
	}
	*/

	/* Read firmware image from the filesystem */
	ATHN_LOCK(sc);
	fw = firmware_get(sc->fwname);
	printf("The size of the fw is: %zu\n", fw->datasize);
	ATHN_UNLOCK(sc);
	if (fw == NULL) {
		device_printf(sc->sc_dev, "failed to load of file %s\n", sc->fwname);
		return (ENOENT);
	}

	ptr = __DECONST(char *, fw->data);
	addr = AR9271_FIRMWARE >> 8;
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD;
	USETW(req.wIndex, 0);
	size = fw->datasize;
	ATHN_LOCK(sc);
	while (size > 0) {
		mlen = MIN(size, 4096);

		USETW(req.wValue, addr);
		USETW(req.wLength, mlen);
		if (usbd_do_request_flags(usc->sc_udev, &sc->sc_mtx,
			&req, ptr, 0, NULL, 250) != 0) {
			error = EIO;
			break;
		}
		addr += mlen >> 8;
		ptr += mlen;
		size -= mlen;
	}
	ATHN_UNLOCK(sc);

	addr = AR9271_FIRMWARE_TEXT >> 8;

	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD_COMP;
	USETW(req.wIndex, 0);
	USETW(req.wValue, addr);
	USETW(req.wLength, 0);
	usc->wait_msg_id = AR_HTC_MSG_READY;
	ATHN_LOCK(sc);
	error = usbd_do_request(usc->sc_udev, &sc->sc_mtx, &req, NULL);
//	usbd_transfer_start(RX_INTR); //////////////////////////////
	usbd_transfer_start(usc->usc_xfer[ATHN_RX_INTR]);
	int xxx = 10;
	while (usbd_transfer_pending(usc->usc_xfer[ATHN_RX_INTR]) && xxx--) {
		ATHN_UNLOCK(sc);
		pause("W", hz / 16);
		ATHN_LOCK(sc);
	}
	if (error == 0 && usc->wait_msg_id != 0) {
		printf("Error is %d\n", error);
//		error = tsleep(&usc->wait_msg_id, 0, "athnfw", hz); /* Wait 1 second at most */
		error = msleep(&usc->wait_msg_id, &sc->sc_mtx, 0, "athnfw", 2 * hz); /* Wait 1 second at most */

//		msleep(const void *chan, struct mtx *mtx, int priority, const char *wmesg, int timo);

		if (error) {
			ATHN_UNLOCK(sc);
			return error;
		}
	}
	ATHN_UNLOCK(sc);

	usc->wait_msg_id = 0;

	firmware_put(fw, FIRMWARE_UNLOAD);
	if (error != 0)
		printf("Bad: %s: error=%d\n", __func__, error);
//		device_printf(sc->sc_dev, "%s: %s: error=%d\n", __func__, name, error);
	return error;
}

#if 0
int
athn_usb_load_firmware(struct athn_usb_softc *usc)
{
	struct athn_softc *sc = &usc->sc_sc;
	const struct firmware *fw;
	size_t fwsize, size;
	int error = 0;
	int mlen;
	void *fw_copy_head;
	unsigned char *ptr;
	uint32_t addr;
	usb_device_request_t req;
	printf("athn_usb_load_firmware start\n");

	ATHN_LOCK(sc);
	fw = firmware_get(sc->fwname);
	ATHN_UNLOCK(sc);
	if (fw == NULL) {
		device_printf(sc->sc_dev, "failed to load of file %s\n", sc->fwname);
		return (ENOENT);
	}

#ifdef CHECK_THE_FIRMWARE
	size_t len;
	len = fw->datasize;
	if (len < 999999 || len > sc->fwsize_limit) {
		device_printf(sc->sc_dev, "wrong firmware size (%zu)\n", len);
		error = EINVAL;
		return error;
	}
#endif
	fwsize = fw->datasize;
#if 0
	usb_device_descriptor_t *dd;
	usb_device_request_t req;
	const char *name;
	u_char *fw, *ptr;
	 fwsize, size;
	uint32_t addr;
	int s, mlen, error;

	/* Determine which firmware image to load. */
	if (usc->flags & ATHN_USB_FLAG_AR7010) {
		dd = usbd_get_device_descriptor(usc->sc_udev);
		name = "athn-open-ar7010";
	} else
		name = "athn-open-ar9271";
	/* Read firmware image from the filesystem. */
	if ((error = loadfirmware(name, &fw, &fwsize)) != 0) {
		printf("%s: failed loadfirmware of file %s (error %d)\n",
		    usc->usb_dev.dv_xname, name, error);
		return (error);
	}
#endif

	fw_copy_head = malloc(fw->datasize, M_ATHN_USB, M_WAITOK);
	memcpy(fw_copy_head, fw->data, fw->datasize);
	ptr = fw_copy_head;

	addr = AR9271_FIRMWARE >> 8;
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD;
	USETW(req.wIndex, 0);
	size = fwsize;
	while (size > 0) {
		mlen = MIN(size, 4096);

		USETW(req.wValue, addr);
		USETW(req.wLength, mlen);
		error = usbd_do_request(usc->sc_udev, NULL, &req, (void *)ptr);
		if (error != 0) {
			printf("usbd_do_request error %d at %s:%d\n", error, __FILE__, __LINE__);
			return (error);
		}
		addr += mlen >> 8;
		ptr  += mlen;
		size -= mlen;
	}
	free(fw_copy_head, M_DEVBUF);

	/* Start firmware. */
	if (usc->flags & ATHN_USB_FLAG_AR7010) {
		printf("AR7010_FIRMWARE_TEXT\n");
		addr = AR7010_FIRMWARE_TEXT >> 8;
	}
	else {
		printf("AR9271_FIRMWARE_TEXT\n");
		addr = AR9271_FIRMWARE_TEXT >> 8;
	}
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD_COMP;
	USETW(req.wIndex, 0);
	USETW(req.wValue, addr);
	USETW(req.wLength, 0);
	usc->wait_msg_id = AR_HTC_MSG_READY;
	error = usbd_do_request(usc->sc_udev, NULL, &req, NULL);
	/* Wait at most 1 second for firmware to boot. */
	if (error == 0 && usc->wait_msg_id != 0) {
		printf("Latter error! %d %d\n", error, usc->wait_msg_id);
//		ATHN_LOCK(sc);
	// XXX Update this in the future to check wakeup() value
		error = tsleep(sc, 0, "athnfw", hz);
//		error = mtx_sleep(sc, &sc->sc_mtx, 0 , "athnfw", hz);
//		if (error == EINTR)
//			printf("EINTR on line %d\n", __LINE__);
//		else if (error == ERESTART)
//			printf("ERESTART on line %d\n", __LINE__);
//		else if (error == EWOULDBLOCK)
//			printf("EWOULDBLOCK on %d\n", __LINE__);
//		else
//			printf("Clear on %d\n", __LINE__);
//		ATHN_UNLOCK(sc);
	}
	usc->wait_msg_id = 0;
	printf("sending back %d\n", error);
	return (error);
}
#endif

int
athn_usb_htc_msg(struct athn_usb_softc *usc, uint16_t msg_id, void *buf,
    int len)
{
	struct athn_softc *sc = &usc->sc_sc;
	struct athn_usb_tx_data *data = &usc->tx_cmd;
	struct ar_htc_frame_hdr *htc;
	struct ar_htc_msg_hdr *msg;

	htc = (struct ar_htc_frame_hdr *)data->buf;
	memset(htc, 0, sizeof(*htc));
	htc->endpoint_id = 0;
	htc->payload_len = htobe16(sizeof(*msg) + len);

	msg = (struct ar_htc_msg_hdr *)&htc[1];
	msg->msg_id = htobe16(msg_id);

	memcpy(&msg[1], buf, len);

	/*
	 * FreeBSD addition, required because OpenBSD's xfer mechanism
	 * specifies the length during transfer, whereas FreeBSD's callback
	 * mechanism does not.
	 */
	data->len = sizeof(*htc) + sizeof(*msg) + len;

//	ATHN_LOCK(sc);
	usbd_transfer_start(usc->usc_xfer[ATHN_RX_INTR]);
	usbd_transfer_start(usc->usc_xfer[ATHN_TX_INTR]);
	while (usbd_transfer_pending(usc->usc_xfer[ATHN_TX_INTR])) {
		ATHN_UNLOCK(sc);
		pause("Farhan was here", hz / 16);
		ATHN_LOCK(sc);
	}
//	ATHN_UNLOCK(sc);

	return 0;
#if 0

	usbd_setup_xfer(
		data->xfer,				// xfer
		usc->tx_intr_pipe,		// pipe
		NULL,					// priv data?
		data->buf,				// buffer data?
	    sizeof(*htc) + sizeof(*msg) + len,	// length of data
	    USBD_SHORT_XFER_OK | USBD_NO_COPY | USBD_SYNCHRONOUS, // flags
	    ATHN_USB_CMD_TIMEOUT,	// timeout time
		NULL); 					// callback function?
	return (usbd_transfer(data->xfer));
#endif
}

int
athn_usb_htc_setup(struct athn_usb_softc *usc)
{
	struct athn_softc *sc = &usc->sc_sc;
	struct ar_htc_msg_config_pipe cfg;
	int error;

	/*
	 * Connect WMI services to USB pipes.
	 */
	 printf("++Load Step 1\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_CONTROL,
	    AR_PIPE_TX_INTR, AR_PIPE_RX_INTR, &usc->ep_ctrl);
	if (error != 0) {
		printf("Step 1 error %d\n", error);
		return (error);
	}
	 printf("++Load Step 2\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_BEACON,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_bcn);
	if (error != 0)
		return (error);
	 printf("++Load Step 3\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_CAB,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_cab);
	if (error != 0)
		return (error);
	 printf("++Load Step 4\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_UAPSD,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_uapsd);
	if (error != 0)
		return (error);
	 printf("++Load Step 5\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_MGMT,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_mgmt);
	if (error != 0)
		return (error);
	 printf("++Load Step 6\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_BE,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[WME_AC_BE]);
	if (error != 0)
		return (error);
	 printf("++Load Step 7\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_BK,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[WME_AC_BK]);
	if (error != 0)
		return (error);
	 printf("++Load Step 8\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_VI,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[WME_AC_VI]);
	if (error != 0)
		return (error);
	 printf("++Load Step 9\n");
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_VO,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[WME_AC_VO]);
	if (error != 0)
		return (error);

	/* Set credits for WLAN Tx pipe. */
	memset(&cfg, 0, sizeof(cfg));
	cfg.pipe_id = UE_GET_ADDR(AR_PIPE_TX_DATA);
	cfg.credits = (usc->flags & ATHN_USB_FLAG_AR7010) ? 45 : 33;
	usc->wait_msg_id = AR_HTC_MSG_CONF_PIPE_RSP;
	// XXX Come back to this maybe?
	ATHN_LOCK(sc);
	error = athn_usb_htc_msg(usc, AR_HTC_MSG_CONF_PIPE, &cfg, sizeof(cfg));
	if (error == 0 && usc->wait_msg_id != 0) {
		//error = tsleep(sc, 0, "athnhtc", hz);
		error = msleep(&usc->wait_msg_id, &sc->sc_mtx, 0, "athnhtc", hz);
	}
	ATHN_UNLOCK(sc);
	usc->wait_msg_id = 0;
	if (error != 0) {
		printf("%s: could not configure pipe\n", sc->sc_ic.ic_name);
		return (error);
	}

	ATHN_LOCK(sc);
	error = athn_usb_htc_msg(usc, AR_HTC_MSG_SETUP_COMPLETE, NULL, 0);
	ATHN_UNLOCK(sc);
	if (error != 0) {
		printf("%s: could not complete setup\n", sc->sc_ic.ic_name);
		return (error);
	}
	return (0);
}

int
athn_usb_htc_connect_svc(struct athn_usb_softc *usc, uint16_t svc_id,
    uint8_t ul_pipe, uint8_t dl_pipe, uint8_t *endpoint_id)
{
	struct athn_softc *sc = &usc->sc_sc;
	struct ar_htc_msg_conn_svc msg;
	struct ar_htc_msg_conn_svc_rsp rsp;
	int error;

	memset(&msg, 0, sizeof(msg));
	msg.svc_id = htobe16(svc_id);
	msg.dl_pipeid = UE_GET_ADDR(dl_pipe);
	msg.ul_pipeid = UE_GET_ADDR(ul_pipe);
	usc->msg_conn_svc_rsp = &rsp;
	usc->wait_msg_id = AR_HTC_MSG_CONN_SVC_RSP;

	ATHN_LOCK(sc);
	error = athn_usb_htc_msg(usc, AR_HTC_MSG_CONN_SVC, &msg, sizeof(msg));

	/* Wait at most 1 second for response. */
	if (error == 0 && usc->wait_msg_id != 0) {
		printf("Sleep here Line: %d\n", __LINE__);
		error = msleep(&usc->wait_msg_id, &sc->sc_mtx, 0, "athnhtc", 10 * hz);
		printf("Working with Hans, msg_id = %d\n", usc->wait_msg_id);
		/* Wait 1 second at most */
	}
	ATHN_UNLOCK(sc);
	usc->wait_msg_id = 0;
//	splx(s);
	if (error != 0) {
		printf("error waiting for service %d connection\n", svc_id);
//		printf("%s: error waiting for service %d connection\n",
//		    usc->usb_dev.dv_xname, svc_id);
		return (error);
	}
	if (rsp.status != AR_HTC_SVC_SUCCESS) {
		printf(": service %d connection failed, error %d\n",  svc_id, rsp.status);
		return (EIO);
	}
//	DPRINTF(("service %d successfully connected to endpoint %d\n",
//	    svc_id, rsp.endpoint_id));

	/* Return endpoint id. */
	*endpoint_id = rsp.endpoint_id;
	return (0);
#if 0
#endif
}

int
athn_usb_wmi_xcmd(struct athn_usb_softc *usc, uint16_t cmd_id, void *ibuf,
    int ilen, void *obuf)
{
	struct athn_usb_tx_data *data = &usc->tx_cmd;
	struct athn_softc *sc = &usc->sc_sc;
	struct ar_htc_frame_hdr *htc;
	struct ar_wmi_cmd_hdr *wmi;
	int error;

//	if (usbd_is_dying(usc->sc_udev))
//		return ENXIO;

//	s = splusb();
	// REVISIT THIS CODE BELOW. WHY THE LOOP?!?!?!?
	while (usc->wait_cmd_id) {
		/*
		 * The previous USB transfer is not done yet. We can't use
		 * data->xfer until it is done or we'll cause major confusion
		 * in the USB stack.
		 */
		//tsleep(&usc->wait_cmd_id, 0, "athnwmx", ATHN_USB_CMD_TIMEOUT);
		ATHN_LOCK(sc);
		printf("This one probably needs some sort of wakeup equivalent...\n");
		msleep(&usc->wait_cmd_id, &sc->sc_mtx, 0, "athnwmx", ATHN_USB_CMD_TIMEOUT); /* Wait 1 second at most */
		ATHN_UNLOCK(sc);
//		tsleep_nsec(&usc->wait_cmd_id, 0, "athnwmx",
//		    MSEC_TO_NSEC(ATHN_USB_CMD_TIMEOUT));
//		if (usbd_is_dying(usc->sc_udev)) {
//			splx(s);
//			return ENXIO;
	}
//	splx(s);

	htc = (struct ar_htc_frame_hdr *)data->buf;
	memset(htc, 0, sizeof(*htc));
	htc->endpoint_id = usc->ep_ctrl;
	htc->payload_len = htobe16(sizeof(*wmi) + ilen);

	wmi = (struct ar_wmi_cmd_hdr *)&htc[1];
	wmi->cmd_id = htobe16(cmd_id);
	usc->wmi_seq_no++;
	wmi->seq_no = htobe16(usc->wmi_seq_no);

	memcpy(&wmi[1], ibuf, ilen);

	data->len = sizeof(*htc) + sizeof(*wmi) + ilen;

	ATHN_LOCK(sc);
	usbd_transfer_start(usc->usc_xfer[ATHN_TX_INTR]);
	ATHN_UNLOCK(sc);

//	usbd_setup_xfer(data->xfer, usc->tx_intr_pipe, NULL, data->buf,
//	    sizeof(*htc) + sizeof(*wmi) + ilen,
//	    USBD_SHORT_XFER_OK | USBD_NO_COPY, ATHN_USB_CMD_TIMEOUT,
//	    NULL);
//	s = splusb();
//	error = usbd_transfer(data->xfer);
//	if (__predict_false(error != USBD_IN_PROGRESS && error != 0)) {
//		splx(s);
//		return (error);
//	}
	usc->obuf = obuf;
	usc->wait_cmd_id = cmd_id;
	/* 
	 * Wait for WMI command complete interrupt. In case it does not fire
	 * wait until the USB transfer times out to avoid racing the transfer.
	 */
//	error = tsleep(&usc->wait_cmd_id, 0, "athnwmi", ATHN_USB_CMD_TIMEOUT);
	ATHN_LOCK(sc);
	error = msleep(&usc->wait_cmd_id, &sc->sc_mtx, 0, "athnwmi", ATHN_USB_CMD_TIMEOUT);
	ATHN_UNLOCK(sc);
//	error = tsleep_nsec(&usc->wait_cmd_id, 0, "athnwmi",
//	    MSEC_TO_NSEC(ATHN_USB_CMD_TIMEOUT));
	if (error) {
		if (error == EWOULDBLOCK) {
			if (cmd_id != 0x15)
				device_printf(sc->sc_dev, "firmware command 0x%x timed out\n", cmd_id);
			error = ETIMEDOUT;
		}
	}

	/* 
	 * Both the WMI command and transfer are done or have timed out.
	 * Allow other threads to enter this function and use data->xfer.
	 */
	usc->wait_cmd_id = 0;
	wakeup(&usc->wait_cmd_id);

//	splx(s);
	return (error);
}

int
athn_usb_read_rom(struct athn_softc *sc)
{
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	uint32_t addrs[8], vals[8], addr;
	uint16_t *eep;
	int i, j, error;

	/* Read EEPROM by blocks of 16 bytes. */
	eep = sc->eep;
	addr = AR_EEPROM_OFFSET(sc->eep_base);
	for (i = 0; i < sc->eep_size / 16; i++) {
		for (j = 0; j < 8; j++, addr += 4)
			addrs[j] = htobe32(addr);
		error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_REG_READ,
		    addrs, sizeof(addrs), vals);
		if (error != 0)
			break;
		for (j = 0; j < 8; j++)
			*eep++ = htobe32(vals[j]);
	}
	return (error);
}

uint32_t
athn_usb_read(struct athn_softc *sc, uint32_t addr)
{
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	uint32_t val;
	int error;

	/* Flush pending writes for strict consistency. */
	athn_usb_write_barrier(sc);

	addr = htobe32(addr);
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_REG_READ,
	    &addr, sizeof(addr), &val);
	if (error != 0)
		return (0xdeadbeef);
	return (htobe32(val));
}

void
athn_usb_write(struct athn_softc *sc, uint32_t addr, uint32_t val)
{
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;

	usc->wbuf[usc->wcount].addr = htobe32(addr);
	usc->wbuf[usc->wcount].val  = htobe32(val);
	if (++usc->wcount == AR_MAX_WRITE_COUNT)
		athn_usb_write_barrier(sc);
}

void
athn_usb_write_barrier(struct athn_softc *sc)
{
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;

	if (usc->wcount == 0)
		return;	/* Nothing to write. */

	(void)athn_usb_wmi_xcmd(usc, AR_WMI_CMD_REG_WRITE,
	    usc->wbuf, usc->wcount * sizeof(usc->wbuf[0]), NULL);
	usc->wcount = 0;	/* Always flush buffer. */
}

int
athn_usb_media_change(struct ifnet *ifp)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_usb_softc *usc = (struct athn_usb_softc *)ifp->if_softc;
	int error;

	if (usbd_is_dying(usc->sc_udev))
		return ENXIO;

	error = ieee80211_media_change(ifp);
	if (error != ENETRESET)
		return (error);

	if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
	    (IFF_UP | IFF_RUNNING)) {
		athn_usb_stop(ifp);
		error = athn_usb_init(ifp);
	}
	return (error);
#endif
}

void
athn_usb_next_scan(void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_softc *usc = arg;
	struct athn_softc *sc = &usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	int s;

	if (usbd_is_dying(usc->sc_udev))
		return;

	usbd_ref_incr(usc->sc_udev);

	s = splnet();
	if (ic->ic_state == IEEE80211_S_SCAN)
		ieee80211_next_scan(&ic->ic_if);
	splx(s);

	usbd_ref_decr(usc->sc_udev);
#endif
}

int
athn_usb_newstate(struct ieee80211com *ic, enum ieee80211_state nstate,
    int arg)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_usb_cmd_newstate cmd;

	/* Do it in a process context. */
	cmd.state = nstate;
	cmd.arg = arg;
	athn_usb_do_async(usc, athn_usb_newstate_cb, &cmd, sizeof(cmd));
	return (0);
#endif
}

void
athn_usb_newstate_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_cmd_newstate *cmd = arg;
	struct athn_softc *sc = &usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	enum ieee80211_state ostate;
	uint32_t reg, imask;
	int s, error;

	timeout_del(&sc->calib_to);

	s = splnet();
	ostate = ic->ic_state;

	if (ostate == IEEE80211_S_RUN && ic->ic_opmode == IEEE80211_M_STA) {
		athn_usb_remove_node(usc, ic->ic_bss);
		reg = AR_READ(sc, AR_RX_FILTER);
		reg = (reg & ~AR_RX_FILTER_MYBEACON) |
		    AR_RX_FILTER_BEACON;
		AR_WRITE(sc, AR_RX_FILTER, reg);
		AR_WRITE_BARRIER(sc);
	}
	switch (cmd->state) {
	case IEEE80211_S_INIT:
		athn_set_led(sc, 0);
		break;
	case IEEE80211_S_SCAN:
		/* Make the LED blink while scanning. */
		athn_set_led(sc, !sc->led_state);
		error = athn_usb_switch_chan(sc, ic->ic_bss->ni_chan, NULL);
		if (error)
			printf("%s: could not switch to channel %d\n",
			    usc->usb_dev.dv_xname,
			    ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan));
		if (!usbd_is_dying(usc->sc_udev))
			timeout_add_msec(&sc->scan_to, 200);
		break;
	case IEEE80211_S_AUTH:
		athn_set_led(sc, 0);
		error = athn_usb_switch_chan(sc, ic->ic_bss->ni_chan, NULL);
		if (error)
			printf("%s: could not switch to channel %d\n",
			    usc->usb_dev.dv_xname,
			    ieee80211_chan2ieee(ic, ic->ic_bss->ni_chan));
		break;
	case IEEE80211_S_ASSOC:
		break;
	case IEEE80211_S_RUN:
		athn_set_led(sc, 1);

		if (ic->ic_opmode == IEEE80211_M_MONITOR)
			break;

		if (ic->ic_opmode == IEEE80211_M_STA) {
			/* Create node entry for our BSS */
			error = athn_usb_create_node(usc, ic->ic_bss);
			if (error)
				printf("%s: could not update firmware station "
				    "table\n", usc->usb_dev.dv_xname);
		}
		athn_set_bss(sc, ic->ic_bss);
		athn_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
#ifndef IEEE80211_STA_ONLY
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			athn_usb_switch_chan(sc, ic->ic_bss->ni_chan, NULL);
			athn_set_hostap_timers(sc);
			/* Enable software beacon alert interrupts. */
			imask = htobe32(AR_IMR_SWBA);
		} else
#endif
		{
			athn_set_sta_timers(sc);
			/* Enable beacon miss interrupts. */
			imask = htobe32(AR_IMR_BMISS);

			/* Stop receiving beacons from other BSS. */
			reg = AR_READ(sc, AR_RX_FILTER);
			reg = (reg & ~AR_RX_FILTER_BEACON) |
			    AR_RX_FILTER_MYBEACON;
			AR_WRITE(sc, AR_RX_FILTER, reg);
			AR_WRITE_BARRIER(sc);
		}
		athn_usb_wmi_xcmd(usc, AR_WMI_CMD_ENABLE_INTR,
		    &imask, sizeof(imask), NULL);
		break;
	}
	(void)sc->sc_newstate(ic, cmd->state, cmd->arg);
	splx(s);
#endif
}

void
athn_usb_newassoc(struct ieee80211_node *ni, int isnew)
{
	printf("%s unimplemented...\n", __func__);
#if 0
	printf("%s unimplemented.\n", __func__);
//#ifndef IEEE80211_STA_ONLY
	struct athn_usb_softc *usc = ic->ic_softc;

	if (ic->ic_opmode != IEEE80211_M_HOSTAP &&
	    ic->ic_state != IEEE80211_S_RUN)
		return;

	/* Update the node's supported rates in a process context. */
	ieee80211_ref_node(ni);
	athn_usb_do_async(usc, athn_usb_newassoc_cb, &ni, sizeof(ni));
#endif
}

#ifndef IEEE80211_STA_ONLY
void
athn_usb_newassoc_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct ieee80211_node *ni = *(void **)arg;
	struct athn_node *an = (struct athn_node *)ni;
	int s;

	if (ic->ic_state != IEEE80211_S_RUN)
		return;

	s = splnet();
	/* NB: Node may have left before we got scheduled. */
	if (an->sta_index != 0)
		(void)athn_usb_node_set_rates(usc, ni);
	ieee80211_release_node(ic, ni);
	splx(s);
#endif
}
#endif

struct ieee80211_node *
athn_usb_node_alloc(struct ieee80211vap *vap, const uint8_t mac[IEEE80211_ADDR_LEN])
{
//	printf("%s unimplemented.\n", __func__);
//	return NULL;
//#if 0
	struct athn_node *an;

	an = malloc(sizeof(struct athn_node), M_80211_NODE, M_NOWAIT | M_ZERO);
	return (struct ieee80211_node *)an;
//#endif
}


#ifndef IEEE80211_STA_ONLY
void
athn_usb_count_active_sta(void *arg, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	int *nsta = arg;
	struct athn_node *an = (struct athn_node *)ni;

	if (an->sta_index == 0)
		return;

	if ((ni->ni_state == IEEE80211_STA_AUTH ||
	    ni->ni_state == IEEE80211_STA_ASSOC) &&
	    ni->ni_inact < IEEE80211_INACT_MAX)
		(*nsta)++;
#endif
}

struct athn_usb_newauth_cb_arg {
	struct ieee80211_node *ni;
	uint16_t seq;
};

void
athn_usb_newauth_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct athn_usb_newauth_cb_arg *a = arg;
	struct ieee80211_node *ni = a->ni;
	uint16_t seq = a->seq;
	struct athn_node *an = (struct athn_node *)ni;
	int s, error = 0;

	if (ic->ic_state != IEEE80211_S_RUN)
		return;

	s = splnet();
	if (an->sta_index == 0) {
		error = athn_usb_create_node(usc, ni);
		if (error)
			printf("%s: could not add station %s to firmware "
			    "table\n", usc->usb_dev.dv_xname,
			    ether_sprintf(ni->ni_macaddr));
	}
	if (error == 0)
		ieee80211_auth_open_confirm(ic, ni, seq);
	ieee80211_unref_node(&ni);
	splx(s);
#endif
}
#endif

int
athn_usb_newauth(struct ieee80211com *ic, struct ieee80211_node *ni,
    int isnew, uint16_t seq)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
#ifndef IEEE80211_STA_ONLY
	struct athn_usb_softc *usc = ic->ic_softc;
	struct ifnet *ifp = &ic->ic_if;
	struct athn_node *an = (struct athn_node *)ni;
	int nsta;
	struct athn_usb_newauth_cb_arg arg;

	if (ic->ic_opmode != IEEE80211_M_HOSTAP)
		return 0;

	if (!isnew && an->sta_index != 0) /* already in firmware table */
		return 0;

	/* Check if we have room in the firmware table. */
	nsta = 1; /* Account for default node. */
	ieee80211_iterate_nodes(ic, athn_usb_count_active_sta, &nsta);
	if (nsta >= AR_USB_MAX_STA) {
		if (ifp->if_flags & IFF_DEBUG)
			printf("%s: cannot authenticate station %s: firmware "
			    "table is full\n", usc->usb_dev.dv_xname,
			    ether_sprintf(ni->ni_macaddr));
		return ENOSPC;
	}

	/* 
	 * In a process context, try to add this node to the
	 * firmware table and confirm the AUTH request.
	 */
	arg.ni = ieee80211_ref_node(ni);
	arg.seq = seq;
	athn_usb_do_async(usc, athn_usb_newauth_cb, &arg, sizeof(arg));
	return EBUSY;
#else
	return 0;
#endif /* IEEE80211_STA_ONLY */
#endif
}

#ifndef IEEE80211_STA_ONLY
void
athn_usb_node_free(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_node *an = (struct athn_node *)ni;

	/* 
	 * Remove the node from the firmware table in a process context.
	 * Pass an index rather than the pointer which we will free.
	 */
	if (an->sta_index != 0)
		athn_usb_do_async(usc, athn_usb_node_free_cb,
		    &an->sta_index, sizeof(an->sta_index));
	usc->sc_node_free(ic, ni);
#endif
}

void
athn_usb_node_free_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct ifnet *ifp = &ic->ic_if;
	uint8_t sta_index = *(uint8_t *)arg;
	int error;

	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_REMOVE,
	    &sta_index, sizeof(sta_index), NULL);
	if (error) {
		printf("%s: could not remove station %u from firmware table\n",
		    usc->usb_dev.dv_xname, sta_index);
		return;
	}
	usc->free_node_slots |= (1 << sta_index);
	if (ifp->if_flags & IFF_DEBUG)
		printf("%s: station %u removed from firmware table\n",
		    usc->usb_dev.dv_xname, sta_index);
#endif
}
#endif /* IEEE80211_STA_ONLY */

int
athn_usb_ampdu_tx_start(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_node *an = (struct athn_node *)ni;
	struct athn_usb_aggr_cmd cmd;

	/* Do it in a process context. */
	cmd.sta_index = an->sta_index;
	cmd.tid = tid;
	athn_usb_do_async(usc, athn_usb_ampdu_tx_start_cb, &cmd, sizeof(cmd));
	return (0);
#endif
}

void
athn_usb_ampdu_tx_start_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_aggr_cmd *cmd = arg;
	struct ar_htc_target_aggr aggr;

	memset(&aggr, 0, sizeof(aggr));
	aggr.sta_index = cmd->sta_index;
	aggr.tidno = cmd->tid;
	aggr.aggr_enable = 1;
	(void)athn_usb_wmi_xcmd(usc, AR_WMI_CMD_TX_AGGR_ENABLE,
	    &aggr, sizeof(aggr), NULL);
#endif
}

void
athn_usb_ampdu_tx_stop(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_node *an = (struct athn_node *)ni;
	struct athn_usb_aggr_cmd cmd;

	/* Do it in a process context. */
	cmd.sta_index = an->sta_index;
	cmd.tid = tid;
	athn_usb_do_async(usc, athn_usb_ampdu_tx_stop_cb, &cmd, sizeof(cmd));
#endif
}

void
athn_usb_ampdu_tx_stop_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_aggr_cmd *cmd = arg;
	struct ar_htc_target_aggr aggr;

	memset(&aggr, 0, sizeof(aggr));
	aggr.sta_index = cmd->sta_index;
	aggr.tidno = cmd->tid;
	aggr.aggr_enable = 0;
	(void)athn_usb_wmi_xcmd(usc, AR_WMI_CMD_TX_AGGR_ENABLE,
	    &aggr, sizeof(aggr), NULL);
#endif
}

#ifndef IEEE80211_STA_ONLY
/* Try to find a node we can evict to make room in the firmware table. */
void
athn_usb_clean_nodes(void *arg, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_softc *usc = arg;
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct athn_node *an = (struct athn_node *)ni;

	/* 
	 * Don't remove the default node (used for management frames).
	 * Nodes which are not in the firmware table also have index zero.
	 */
	if (an->sta_index == 0)
		return;

	/* Remove non-associated nodes. */
	if (ni->ni_state != IEEE80211_STA_AUTH &&
	    ni->ni_state != IEEE80211_STA_ASSOC) {
		athn_usb_remove_node(usc, ni);
		return;
	}

	/* 
	 * Kick off inactive associated nodes. This won't help
	 * immediately but will help if the new STA retries later.
	 */
	if (ni->ni_inact >= IEEE80211_INACT_MAX) {
		IEEE80211_SEND_MGMT(ic, ni, IEEE80211_FC0_SUBTYPE_DEAUTH,
		    IEEE80211_REASON_AUTH_EXPIRE);
		ieee80211_node_leave(ic, ni);
	}
#endif // End of the FreeBSD endif
}
#endif

int
athn_usb_create_node(struct athn_usb_softc *usc, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_node *an = (struct athn_node *)ni;
	struct ar_htc_target_sta sta;
	int error, sta_index;
#ifndef IEEE80211_STA_ONLY
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct ifnet *ifp = &ic->ic_if;

	/* Firmware cannot handle more than 8 STAs. Try to make room first. */
	if (ic->ic_opmode == IEEE80211_M_HOSTAP)
		ieee80211_iterate_nodes(ic, athn_usb_clean_nodes, usc);
#endif
	if (usc->free_node_slots == 0x00)
		return ENOBUFS;

	sta_index = ffs(usc->free_node_slots) - 1;
	if (sta_index < 0 || sta_index >= AR_USB_MAX_STA)
		return ENOSPC;

	/* Create node entry on target. */
	memset(&sta, 0, sizeof(sta));
	IEEE80211_ADDR_COPY(sta.macaddr, ni->ni_macaddr);
	IEEE80211_ADDR_COPY(sta.bssid, ni->ni_bssid);
	sta.sta_index = sta_index;
	sta.maxampdu = 0xffff;
	if (ni->ni_flags & IEEE80211_NODE_HT)
		sta.flags |= htobe16(AR_HTC_STA_HT);
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_CREATE,
	    &sta, sizeof(sta), NULL);
	if (error != 0)
		return (error);
	an->sta_index = sta_index;
	usc->free_node_slots &= ~(1 << an->sta_index);

#ifndef IEEE80211_STA_ONLY
	if (ifp->if_flags & IFF_DEBUG)
		printf("%s: station %u (%s) added to firmware table\n",
		    usc->usb_dev.dv_xname, sta_index,
		    ether_sprintf(ni->ni_macaddr));
#endif
	return athn_usb_node_set_rates(usc, ni);
#endif
}

int
athn_usb_node_set_rates(struct athn_usb_softc *usc, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_node *an = (struct athn_node *)ni;
	struct ar_htc_target_rate rate;
	int i, j;

	/* Setup supported rates. */
	memset(&rate, 0, sizeof(rate));
	rate.sta_index = an->sta_index;
	rate.isnew = 1;
	rate.lg_rates.rs_nrates = ni->ni_rates.rs_nrates;
	memcpy(rate.lg_rates.rs_rates, ni->ni_rates.rs_rates,
	    ni->ni_rates.rs_nrates);
	if (ni->ni_flags & IEEE80211_NODE_HT) {
		rate.capflags |= htobe32(AR_RC_HT_FLAG);
		/* Setup HT rates. */
		for (i = 0, j = 0; i < IEEE80211_HT_NUM_MCS; i++) {
			if (!isset(ni->ni_rxmcs, i))
				continue;
			if (j >= AR_HTC_RATE_MAX)
				break;
			rate.ht_rates.rs_rates[j++] = i;
		}
		rate.ht_rates.rs_nrates = j;

		if (ni->ni_rxmcs[1]) /* dual-stream MIMO rates */
			rate.capflags |= htobe32(AR_RC_DS_FLAG);
#ifdef notyet
		if (ni->ni_htcaps & IEEE80211_HTCAP_CBW20_40)
			rate.capflags |= htobe32(AR_RC_40_FLAG);
		if (ni->ni_htcaps & IEEE80211_HTCAP_SGI40)
			rate.capflags |= htobe32(AR_RC_SGI_FLAG);
		if (ni->ni_htcaps & IEEE80211_HTCAP_SGI20)
			rate.capflags |= htobe32(AR_RC_SGI_FLAG);
#endif
	}

	return athn_usb_wmi_xcmd(usc, AR_WMI_CMD_RC_RATE_UPDATE,
	    &rate, sizeof(rate), NULL);
#endif
}

int
athn_usb_remove_node(struct athn_usb_softc *usc, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_node *an = (struct athn_node *)ni;
	int error;
#ifndef IEEE80211_STA_ONLY
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct ifnet *ifp = &ic->ic_if;
#endif

	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_REMOVE,
	    &an->sta_index, sizeof(an->sta_index), NULL);
	if (error) {
		printf("%s: could not remove station %u (%s) from "
		    "firmware table\n", usc->usb_dev.dv_xname, an->sta_index,
		    ether_sprintf(ni->ni_macaddr));
		return error;
	}

#ifndef IEEE80211_STA_ONLY
	if (ifp->if_flags & IFF_DEBUG)
		printf("%s: station %u (%s) removed from firmware table\n",
		    usc->usb_dev.dv_xname, an->sta_index,
		    ether_sprintf(ni->ni_macaddr));
#endif

	usc->free_node_slots |= (1 << an->sta_index);
	an->sta_index = 0;
	return 0;
#endif
}

void
athn_usb_rx_enable(struct athn_softc *sc)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	AR_WRITE(sc, AR_CR, AR_CR_RXE);
	AR_WRITE_BARRIER(sc);
#endif
}

int
athn_usb_switch_chan(struct athn_softc *sc, struct ieee80211_channel *c,
    struct ieee80211_channel *extc)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	uint16_t mode;
	int error;

	/* Disable interrupts. */
	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
	if (error != 0)
		goto reset;
	/* Stop all Tx queues. */
	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_DRAIN_TXQ_ALL);
	if (error != 0)
		goto reset;
	/* Stop Rx. */
	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_STOP_RECV);
	if (error != 0)
		goto reset;

	/* If band or bandwidth changes, we need to do a full reset. */
	if (c->ic_flags != sc->curchan->ic_flags ||
	    ((extc != NULL) ^ (sc->curchanext != NULL))) {
		DPRINTFN(2, ("channel band switch\n"));
		goto reset;
	}

	error = athn_set_chan(sc, c, extc);
	if (AR_SREV_9271(sc) && error == 0)
		ar9271_load_ani(sc);
	if (error != 0) {
 reset:		/* Error found, try a full reset. */
		DPRINTFN(3, ("needs a full reset\n"));
		error = athn_hw_reset(sc, c, extc, 0);
		if (error != 0)	/* Hopeless case. */
			return (error);

		error = athn_set_chan(sc, c, extc);
		if (AR_SREV_9271(sc) && error == 0)
			ar9271_load_ani(sc);
		if (error != 0)
			return (error);
	}

	sc->ops.set_txpower(sc, c, extc);

	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_START_RECV);
	if (error != 0)
		return (error);
	athn_rx_start(sc);

	mode = htobe16(IEEE80211_IS_CHAN_2GHZ(c) ?
	    AR_HTC_MODE_11NG : AR_HTC_MODE_11NA);
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_SET_MODE,
	    &mode, sizeof(mode), NULL);
	if (error != 0)
		return (error);

	/* Re-enable interrupts. */
	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_ENABLE_INTR);
	return (error);
#endif
}

void
athn_usb_updateedca(struct ieee80211com *ic)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;

	/* Do it in a process context. */
	athn_usb_do_async(usc, athn_usb_updateedca_cb, NULL, 0);
#endif
}

void
athn_usb_updateedca_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	int s;

	s = splnet();
	athn_updateedca(&usc->sc_sc.sc_ic);
	splx(s);
#endif
}

void
athn_usb_updateslot(struct ieee80211com *ic)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;

	return;	/* XXX */
	/* Do it in a process context. */
	athn_usb_do_async(usc, athn_usb_updateslot_cb, NULL, 0);
#endif
}

void
athn_usb_updateslot_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	int s;

	s = splnet();
	athn_updateslot(&usc->sc_sc.sc_ic);
	splx(s);
#endif
}

int
athn_usb_set_key(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_usb_cmd_key cmd;

	/* Defer setting of WEP keys until interface is brought up. */
	if ((ic->ic_if.if_flags & (IFF_UP | IFF_RUNNING)) !=
	    (IFF_UP | IFF_RUNNING))
		return (0);

	/* Do it in a process context. */
	cmd.ni = (ni != NULL) ? ieee80211_ref_node(ni) : NULL;
	cmd.key = k;
	athn_usb_do_async(usc, athn_usb_set_key_cb, &cmd, sizeof(cmd));
	usc->sc_key_tasks++;
	return EBUSY;
#endif
}

void
athn_usb_set_key_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct athn_usb_cmd_key *cmd = arg;
	int s;

	usc->sc_key_tasks--;

	s = splnet();
	athn_usb_write_barrier(&usc->sc_sc);
	athn_set_key(ic, cmd->ni, cmd->key);
	if (usc->sc_key_tasks == 0) {
		DPRINTF(("marking port %s valid\n",
		    ether_sprintf(cmd->ni->ni_macaddr)));
		cmd->ni->ni_port_valid = 1;
		ieee80211_set_link_state(ic, LINK_STATE_UP);
	}
	if (cmd->ni != NULL)
		ieee80211_release_node(ic, cmd->ni);
	splx(s);
#endif
}

int
athn_usb_delete_key(struct ieee80211vap *vap, const struct ieee80211_key *k)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_usb_cmd_key cmd;

	if (!(ic->ic_if.if_flags & IFF_RUNNING) ||
	    ic->ic_state != IEEE80211_S_RUN)
		return;	/* Nothing to do. */

	/* Do it in a process context. */
	cmd.ni = (ni != NULL) ? ieee80211_ref_node(ni) : NULL;
	cmd.key = k;
	athn_usb_do_async(usc, athn_usb_delete_key_cb, &cmd, sizeof(cmd));
#endif
}

void
athn_usb_delete_key_cb(struct athn_usb_softc *usc, void *arg)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct ieee80211com *ic = &usc->sc_sc.sc_ic;
	struct athn_usb_cmd_key *cmd = arg;
	int s;

	s = splnet();
	athn_delete_key(ic, cmd->ni, cmd->key);
	if (cmd->ni != NULL)
		ieee80211_release_node(ic, cmd->ni);
	splx(s);
#endif
}

#ifndef IEEE80211_STA_ONLY
void
athn_usb_bcneof(struct usbd_xfer *xfer, void *priv)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_tx_data *data = priv;
	struct athn_usb_softc *usc = data->sc;

	if (__predict_false(status == USBD_STALLED))
		usbd_clear_endpoint_stall_async(usc->tx_data_pipe);
	usc->tx_bcn = data;
#endif
}

/*
 * Process Software Beacon Alert interrupts.
 */
void
athn_usb_swba(struct athn_usb_softc *usc)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_softc *sc = &usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct athn_usb_tx_data *data;
	struct ieee80211_frame *wh;
	struct ar_stream_hdr *hdr;
	struct ar_htc_frame_hdr *htc;
	struct ar_tx_bcn *bcn;
	struct mbuf *m;
	int error;

	if (ic->ic_dtim_count == 0)
		ic->ic_dtim_count = ic->ic_dtim_period - 1;
	else
		ic->ic_dtim_count--;

	/* Make sure previous beacon has been sent. */
	if (usc->tx_bcn == NULL)
		return;
	data = usc->tx_bcn;

	/* Get new beacon. */
	m = ieee80211_beacon_alloc(ic, ic->ic_bss);
	if (__predict_false(m == NULL))
		return;
	/* Assign sequence number. */
	wh = mtod(m, struct ieee80211_frame *);
	*(uint16_t *)&wh->i_seq[0] =
	    htole16(ic->ic_bss->ni_txseq << IEEE80211_SEQ_SEQ_SHIFT);
	ic->ic_bss->ni_txseq++;

	hdr = (struct ar_stream_hdr *)data->buf;
	hdr->tag = htole16(AR_USB_TX_STREAM_TAG);
	hdr->len = htole16(sizeof(*htc) + sizeof(*bcn) + m->m_pkthdr.len);

	htc = (struct ar_htc_frame_hdr *)&hdr[1];
	memset(htc, 0, sizeof(*htc));
	htc->endpoint_id = usc->ep_bcn;
	htc->payload_len = htobe16(sizeof(*bcn) + m->m_pkthdr.len);

	bcn = (struct ar_tx_bcn *)&htc[1];
	memset(bcn, 0, sizeof(*bcn));
	bcn->vif_idx = 0;

	m_copydata(m, 0, m->m_pkthdr.len, &bcn[1]);

	usbd_setup_xfer(data->xfer, usc->tx_data_pipe, data, data->buf,
	    sizeof(*hdr) + sizeof(*htc) + sizeof(*bcn) + m->m_pkthdr.len,
	    USBD_SHORT_XFER_OK | USBD_NO_COPY, ATHN_USB_TX_TIMEOUT,
	    athn_usb_bcneof);

	m_freem(m);
	usc->tx_bcn = NULL;
	error = usbd_transfer(data->xfer);
	if (__predict_false(error != USBD_IN_PROGRESS && error != 0))
		usc->tx_bcn = data;
#endif
}
#endif

/* Update current transmit rate for a node based on firmware Tx status. */
void
athn_usb_tx_status(void *arg, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct ar_wmi_evt_txstatus *ts = arg;
	struct athn_node *an = (struct athn_node *)ni;
	uint8_t rate_index = (ts->rate & AR_HTC_TXSTAT_RATE);

	if (an->sta_index != ts->cookie) /* Tx report for a different node */
		return;

	if (ts->flags & AR_HTC_TXSTAT_MCS) {
		if (isset(ni->ni_rxmcs, rate_index))
			ni->ni_txmcs = rate_index;
	} else if (rate_index < ni->ni_rates.rs_nrates)
		ni->ni_txrate = rate_index;
#endif
}

void
athn_usb_rx_wmi_ctrl(struct athn_usb_softc *usc, uint8_t *buf, int len)
{
	struct ar_wmi_cmd_hdr *wmi;
	uint16_t cmd_id;

	if (__predict_false(len < sizeof(*wmi)))
		return;
	wmi = (struct ar_wmi_cmd_hdr *)buf;
//	cmd_id = betoh16(wmi->cmd_id);
	cmd_id = be16toh(wmi->cmd_id);

	if (!(cmd_id & AR_WMI_EVT_FLAG)) {
		if (usc->wait_cmd_id != cmd_id) {
			printf("errir, fiz me\n");
			return;	/* Unexpected reply. */
		}
		if (usc->obuf != NULL) {
			/* Copy answer into caller supplied buffer. */
			memcpy(usc->obuf, &wmi[1], len - sizeof(*wmi));
		}
		/* Notify caller of completion. */
		wakeup(&usc->wait_cmd_id);
		return;
	}
	switch (cmd_id & 0xfff) {
#ifndef IEEE80211_STA_ONLY
	case AR_WMI_EVT_SWBA:
		athn_usb_swba(usc);
		break;
#endif
	case AR_WMI_EVT_TXSTATUS: {
#if 0 // Temporary commenting this out
		struct ar_wmi_evt_txstatus_list *tsl;
		int i;

		tsl = (struct ar_wmi_evt_txstatus_list *)&wmi[1];
		for (i = 0; i < tsl->count && i < nitems(tsl->ts); i++) {
			struct ieee80211com *ic = &usc->sc_sc.sc_ic;
			struct athn_node *an = (struct athn_node *)ic->ic_bss;
			struct ar_wmi_evt_txstatus *ts = &tsl->ts[i];
			uint8_t qid;

			/* Skip the node we use to send management frames. */
			if (ts->cookie == 0)
				continue;

			/* Skip Tx reports for non-data frame endpoints. */
			qid = (ts->rate & AR_HTC_TXSTAT_EPID) >>
				AR_HTC_TXSTAT_EPID_SHIFT;
			if (qid != usc->ep_data[EDCA_AC_BE] &&
			    qid != usc->ep_data[EDCA_AC_BK] &&
			    qid != usc->ep_data[EDCA_AC_VI] &&
			    qid != usc->ep_data[EDCA_AC_VO])
				continue;

			if (ts->cookie == an->sta_index)
				athn_usb_tx_status(ts, ic->ic_bss);
			else
				ieee80211_iterate_nodes(ic, athn_usb_tx_status,
				    ts);
		}
		break;
#endif // End of the temp comment out
	printf("AR_WMI_EVT_TXSTATUS The above code is temporary removed, this should be enabled.\n");
	}
	case AR_WMI_EVT_FATAL:
		device_printf(usc->sc_sc.sc_dev, "fatal firmware error\n");
		break;
	default:
		DPRINTF(("WMI event %d ignored\n", cmd_id));
		break;
	}
}

void
athn_usb_intr(struct usb_xfer *xfer, usb_error_t usb_error)
{
	int actlen;
	struct athn_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ar_htc_frame_hdr *htc;
	struct ar_htc_msg_hdr *msg;
	uint8_t *buf = usc->ibuf;
	struct usb_page_cache *pc = NULL;
	int len;
	uint16_t msg_id;
//	struct athn_usb_tx_data *data = &usc->tx_cmd;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch(USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
//		printf("======= USB_ST_TRANSFERRED\n"); 
		pc = usbd_xfer_get_frame(xfer, 0);
		usbd_copy_out(pc, 0, usc->ibuf, actlen);
		len = actlen;

		/* Skip watchdog pattern if present. */
		if (len >= 4 && *(uint32_t *)buf == htobe32(0x00c60000)) {
			buf += 4;
			len -= 4;
		}

		htc = (struct ar_htc_frame_hdr *)buf;
		buf += sizeof(*htc);
		len -= sizeof(*htc);

		if (htc->endpoint_id != 0) {
//			if (__predict_false(htc->endpoint_id != usc->ep_ctrl))
//				return;
			/* Remove trailer if present .*/
			if (htc->flags & AR_HTC_FLAG_TRAILER) {
				if (__predict_false(len < htc->control[0]))
					goto TR_SETUP;
		//			return;
				len -= htc->control[0];
			}
			athn_usb_rx_wmi_ctrl(usc, buf, len);
			goto TR_SETUP;
//			return;
		}

		// XXX put this back in
		if (__predict_false(len < sizeof(*msg))) {
			goto TR_SETUP;
		//	return;
		}
		msg = (struct ar_htc_msg_hdr *)buf;
		msg_id = be16toh(msg->msg_id);

		switch (msg_id) {
		case AR_HTC_MSG_READY:
			printf("======= AR_HTC_MSG_READY\n"); 
			if (usc->wait_msg_id != msg_id) {
				break;
			}
			usc->wait_msg_id = 0;
			wakeup(&usc->wait_msg_id);
			break;
		case AR_HTC_MSG_CONN_SVC_RSP:
			printf("======= AR_HTC_MSG_CONN_SVC_RSP\n"); 
			if (usc->wait_msg_id != msg_id) {
				break;
			}
			if (usc->msg_conn_svc_rsp != NULL) {
				memcpy(usc->msg_conn_svc_rsp, &msg[1],
					sizeof(struct ar_htc_msg_conn_svc_rsp));
			}
			usc->wait_msg_id = 0;
			wakeup(&usc->wait_msg_id);
			break;
		case AR_HTC_MSG_CONF_PIPE_RSP:
			printf("======= AR_HTC_MSG_CONF_PIPE_RSP\n"); 
			if (usc->wait_msg_id != msg_id)
				break;
			usc->wait_msg_id = 0;
			wakeup(&usc->wait_msg_id);
			break;
		default:
			printf("====HTC message %d ignored\n", msg_id); // This should be a debug message?
			break;
		}


//		break; /* No fallthrough */
		/* XXX Fallthrough */
	case USB_ST_SETUP:
	TR_SETUP:
		usbd_xfer_set_frame_len(xfer, 0, usbd_xfer_max_len(xfer));
		usbd_xfer_set_frames(xfer, 1);
		usbd_transfer_submit(xfer);
		break;
	case USB_ST_ERROR:
		break;
	default: /* Error */
		printf("===default condition  athn_usb_intr\n");
		break;
		// XXX Based on other drivers, there should be a verification for USB_ERR_CANCELLED
	}

	return;
}

void
athn_usb_rx_radiotap(struct athn_softc *sc, struct mbuf *m,
    struct ar_rx_status *rs)
{
	printf("%s unimplemented.\n", __func__);
#if 0

#define IEEE80211_RADIOTAP_F_SHORTGI	0x80	/* XXX from FBSD */

	struct athn_rx_radiotap_header *tap = &sc->sc_rxtap;
	struct ieee80211com *ic = &sc->sc_ic;
	struct mbuf mb;
	uint8_t rate;

	tap->wr_flags = IEEE80211_RADIOTAP_F_FCS;
	tap->wr_tsft = htole64(betoh64(rs->rs_tstamp));
	tap->wr_chan_freq = htole16(ic->ic_bss->ni_chan->ic_freq);
	tap->wr_chan_flags = htole16(ic->ic_bss->ni_chan->ic_flags);
	tap->wr_dbm_antsignal = rs->rs_rssi;
	/* XXX noise. */
	tap->wr_antenna = rs->rs_antenna;
	tap->wr_rate = 0;	/* In case it can't be found below. */
	rate = rs->rs_rate;
	if (rate & 0x80) {		/* HT. */
		/* Bit 7 set means HT MCS instead of rate. */
		tap->wr_rate = rate;
		if (!(rs->rs_flags & AR_RXS_FLAG_GI))
			tap->wr_flags |= IEEE80211_RADIOTAP_F_SHORTGI;

	} else if (rate & 0x10) {	/* CCK. */
		if (rate & 0x04)
			tap->wr_flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
		switch (rate & ~0x14) {
		case 0xb: tap->wr_rate =   2; break;
		case 0xa: tap->wr_rate =   4; break;
		case 0x9: tap->wr_rate =  11; break;
		case 0x8: tap->wr_rate =  22; break;
		}
	} else {			/* OFDM. */
		switch (rate) {
		case 0xb: tap->wr_rate =  12; break;
		case 0xf: tap->wr_rate =  18; break;
		case 0xa: tap->wr_rate =  24; break;
		case 0xe: tap->wr_rate =  36; break;
		case 0x9: tap->wr_rate =  48; break;
		case 0xd: tap->wr_rate =  72; break;
		case 0x8: tap->wr_rate =  96; break;
		case 0xc: tap->wr_rate = 108; break;
		}
	}
	mb.m_data = (caddr_t)tap;
	mb.m_len = sc->sc_rxtap_len;
	mb.m_next = m;
	mb.m_nextpkt = NULL;
	mb.m_type = 0;
	mb.m_flags = 0;
	bpf_mtap(sc->sc_drvbpf, &mb, BPF_DIRECTION_IN);
#endif
}

void
athn_usb_rx_frame(struct athn_usb_softc *usc, struct mbuf *m)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_softc *sc = &usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_if;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	struct ieee80211_rxinfo rxi;
	struct ar_htc_frame_hdr *htc;
	struct ar_rx_status *rs;
	uint16_t datalen;
	int s;

	if (__predict_false(m->m_len < sizeof(*htc)))
		goto skip;
	htc = mtod(m, struct ar_htc_frame_hdr *);
	if (__predict_false(htc->endpoint_id == 0)) {
		DPRINTF(("bad endpoint %d\n", htc->endpoint_id));
		goto skip;
	}
	if (htc->flags & AR_HTC_FLAG_TRAILER) {
		if (m->m_len < htc->control[0])
			goto skip;
		m_adj(m, -(int)htc->control[0]);
	}
	m_adj(m, sizeof(*htc));	/* Strip HTC header. */

	if (__predict_false(m->m_len < sizeof(*rs)))
		goto skip;
	rs = mtod(m, struct ar_rx_status *);

	/* Make sure that payload fits. */
	datalen = betoh16(rs->rs_datalen);
	if (__predict_false(m->m_len < sizeof(*rs) + datalen))
		goto skip;

	if (__predict_false(datalen < sizeof(*wh) + IEEE80211_CRC_LEN))
		goto skip;

	if (rs->rs_status != 0) {
		if (rs->rs_status & AR_RXS_RXERR_DECRYPT)
			ic->ic_stats.is_ccmp_dec_errs++;
		ifp->if_ierrors++;
		goto skip;
	}
	m_adj(m, sizeof(*rs));	/* Strip Rx status. */

	s = splnet();

	/* Grab a reference to the source node. */
	wh = mtod(m, struct ieee80211_frame *);
	ni = ieee80211_find_rxnode(ic, wh);

	/* Remove any HW padding after the 802.11 header. */
	if (!(wh->i_fc[0] & IEEE80211_FC0_TYPE_CTL)) {
		u_int hdrlen = ieee80211_get_hdrlen(wh);
		if (hdrlen & 3) {
			memmove((caddr_t)wh + 2, wh, hdrlen);
			m_adj(m, 2);
		}
		wh = mtod(m, struct ieee80211_frame *);
	}
#if NBPFILTER > 0
	if (__predict_false(sc->sc_drvbpf != NULL))
		athn_usb_rx_radiotap(sc, m, rs);
#endif
	/* Trim 802.11 FCS after radiotap. */
	m_adj(m, -IEEE80211_CRC_LEN);

	/* Send the frame to the 802.11 layer. */
	rxi.rxi_flags = 0;
	rxi.rxi_rssi = rs->rs_rssi + AR_USB_DEFAULT_NF;
	rxi.rxi_tstamp = betoh64(rs->rs_tstamp);
	if (!(wh->i_fc[0] & IEEE80211_FC0_TYPE_CTL) &&
	    (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) &&
	    (ic->ic_flags & IEEE80211_F_RSNON) &&
	    (ni->ni_flags & IEEE80211_NODE_RXPROT) &&
	    (ni->ni_rsncipher == IEEE80211_CIPHER_CCMP ||
	    (IEEE80211_IS_MULTICAST(wh->i_addr1) &&
	    ni->ni_rsngroupcipher == IEEE80211_CIPHER_CCMP))) {
		if (ar5008_ccmp_decap(sc, m, ni) != 0) {
			ifp->if_ierrors++;
			ieee80211_release_node(ic, ni);
			splx(s);
			goto skip;
		}
		rxi.rxi_flags |= IEEE80211_RXI_HWDEC;
	}
	ieee80211_inputm(ifp, m, ni, &rxi, ml);

	/* Node is no longer needed. */
	ieee80211_release_node(ic, ni);
	splx(s);
	return;
 skip:
	m_freem(m);
#endif
}

void
athn_usb_rxeof(struct usbd_xfer *xfer, void *priv)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct mbuf_list ml = MBUF_LIST_INITIALIZER();
	struct athn_usb_rx_data *data = priv;
	struct athn_usb_softc *usc = data->sc;
	struct athn_softc *sc = &usc->sc_sc;
	struct ifnet *ifp = &sc->sc_ic.ic_if;
	struct athn_usb_rx_stream *stream = &usc->rx_stream;
	uint8_t *buf = data->buf;
	struct ar_stream_hdr *hdr;
	struct mbuf *m;
	uint16_t pktlen;
	int off, len;

	if (__predict_false(status != USBD_NORMAL_COMPLETION)) {
		DPRINTF(("RX status=%d\n", status));
		if (status == USBD_STALLED)
			usbd_clear_endpoint_stall_async(usc->rx_data_pipe);
		if (status != USBD_CANCELLED)
			goto resubmit;
		return;
	}
	usbd_get_xfer_status(xfer, NULL, NULL, &len, NULL);

	if (stream->left > 0) {
		if (len >= stream->left) {
			/* We have all our pktlen bytes now. */
			if (__predict_true(stream->m != NULL)) {
				memcpy(mtod(stream->m, uint8_t *) +
				    stream->moff, buf, stream->left);
				athn_usb_rx_frame(usc, stream->m, &ml);
				stream->m = NULL;
			}
			/* Next header is 32-bit aligned. */
			off = (stream->left + 3) & ~3;
			buf += off;
			len -= off;
			stream->left = 0;
		} else {
			/* Still need more bytes, save what we have. */
			if (__predict_true(stream->m != NULL)) {
				memcpy(mtod(stream->m, uint8_t *) +
				    stream->moff, buf, len);
				stream->moff += len;
			}
			stream->left -= len;
			goto resubmit;
		}
	}
	KASSERT(stream->left == 0);
	while (len >= sizeof(*hdr)) {
		hdr = (struct ar_stream_hdr *)buf;
		if (hdr->tag != htole16(AR_USB_RX_STREAM_TAG)) {
			DPRINTF(("invalid tag 0x%x\n", hdr->tag));
			break;
		}
		pktlen = letoh16(hdr->len);
		buf += sizeof(*hdr);
		len -= sizeof(*hdr);

		if (__predict_true(pktlen <= MCLBYTES)) {
			/* Allocate an mbuf to store the next pktlen bytes. */
			MGETHDR(m, M_DONTWAIT, MT_DATA);
			if (__predict_true(m != NULL)) {
				m->m_pkthdr.len = m->m_len = pktlen;
				if (pktlen > MHLEN) {
					MCLGET(m, M_DONTWAIT);
					if (!(m->m_flags & M_EXT)) {
						m_free(m);
						m = NULL;
					}
				}
			}
		} else	/* Drop frames larger than MCLBYTES. */
			m = NULL;

		if (m == NULL)
			ifp->if_ierrors++;

		/*
		 * NB: m can be NULL, in which case the next pktlen bytes
		 * will be discarded from the Rx stream.
		 */
		if (pktlen > len) {
			/* Need more bytes, save what we have. */
			stream->m = m;	/* NB: m can be NULL. */
			if (__predict_true(stream->m != NULL)) {
				memcpy(mtod(stream->m, uint8_t *), buf, len);
				stream->moff = len;
			}
			stream->left = pktlen - len;
			goto resubmit;
		}
		if (__predict_true(m != NULL)) {
			/* We have all the pktlen bytes in this xfer. */
			memcpy(mtod(m, uint8_t *), buf, pktlen);
			athn_usb_rx_frame(usc, m, &ml);
		}

		/* Next header is 32-bit aligned. */
		off = (pktlen + 3) & ~3;
		buf += off;
		len -= off;
	}
	if_input(ifp, &ml);

 resubmit:
	/* Setup a new transfer. */
	usbd_setup_xfer(xfer, usc->rx_data_pipe, data, data->buf,
	    ATHN_USB_RXBUFSZ, USBD_SHORT_XFER_OK | USBD_NO_COPY,
	    USBD_NO_TIMEOUT, athn_usb_rxeof);
	(void)usbd_transfer(xfer);
#endif
}

void
athn_usb_txeof(struct usbd_xfer *xfer, void *priv)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_usb_tx_data *data = priv;
	struct athn_usb_softc *usc = data->sc;
	struct athn_softc *sc = &usc->sc_sc;
	struct ifnet *ifp = &sc->sc_ic.ic_if;
	int s;

	s = splnet();
	/* Put this Tx buffer back to our free list. */
	TAILQ_INSERT_TAIL(&usc->tx_free_list, data, next);

	if (__predict_false(status != USBD_NORMAL_COMPLETION)) {
		DPRINTF(("TX status=%d\n", status));
		if (status == USBD_STALLED)
			usbd_clear_endpoint_stall_async(usc->tx_data_pipe);
		ifp->if_oerrors++;
		splx(s);
		/* XXX Why return? */
		return;
	}
	sc->sc_tx_timer = 0;

	/* We just released a Tx buffer, notify Tx. */
	if (ifq_is_oactive(&ifp->if_snd)) {
		ifq_clr_oactive(&ifp->if_snd);
		ifp->if_start(ifp);
	}
	splx(s);
#endif
}

int
athn_usb_tx(struct athn_softc *sc, struct mbuf *m, struct ieee80211_node *ni)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	struct athn_node *an = (struct athn_node *)ni;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	struct ieee80211_key *k = NULL;
	struct athn_usb_tx_data *data;
	struct ar_stream_hdr *hdr;
	struct ar_htc_frame_hdr *htc;
	struct ar_tx_frame *txf;
	struct ar_tx_mgmt *txm;
	uint8_t *frm;
	uint16_t qos;
	uint8_t qid, tid = 0;
	int hasqos, xferlen, error;

	wh = mtod(m, struct ieee80211_frame *);
	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
		k = ieee80211_get_txkey(ic, wh, ni);
		if (k->k_cipher == IEEE80211_CIPHER_CCMP) {
			u_int hdrlen = ieee80211_get_hdrlen(wh);
			if (ar5008_ccmp_encap(m, hdrlen, k) != 0)
				return (ENOBUFS);
		} else {
			if ((m = ieee80211_encrypt(ic, m, k)) == NULL)
				return (ENOBUFS);
			k = NULL; /* skip hardware crypto further below */
		}
		wh = mtod(m, struct ieee80211_frame *);
	}
	if ((hasqos = ieee80211_has_qos(wh))) {
		qos = ieee80211_get_qos(wh);
		tid = qos & IEEE80211_QOS_TID;
		qid = ieee80211_up_to_ac(ic, tid);
	} else
		qid = EDCA_AC_BE;

	/* Grab a Tx buffer from our free list. */
	data = TAILQ_FIRST(&usc->tx_free_list);
	TAILQ_REMOVE(&usc->tx_free_list, data, next);

#if NBPFILTER > 0
	/* XXX Change radiotap Tx header for USB (no txrate). */
	if (__predict_false(sc->sc_drvbpf != NULL)) {
		struct athn_tx_radiotap_header *tap = &sc->sc_txtap;
		struct mbuf mb;

		tap->wt_flags = 0;
		tap->wt_chan_freq = htole16(ic->ic_bss->ni_chan->ic_freq);
		tap->wt_chan_flags = htole16(ic->ic_bss->ni_chan->ic_flags);
		mb.m_data = (caddr_t)tap;
		mb.m_len = sc->sc_txtap_len;
		mb.m_next = m;
		mb.m_nextpkt = NULL;
		mb.m_type = 0;
		mb.m_flags = 0;
		bpf_mtap(sc->sc_drvbpf, &mb, BPF_DIRECTION_OUT);
	}
#endif

	/* NB: We don't take advantage of USB Tx stream mode for now. */
	hdr = (struct ar_stream_hdr *)data->buf;
	hdr->tag = htole16(AR_USB_TX_STREAM_TAG);

	htc = (struct ar_htc_frame_hdr *)&hdr[1];
	memset(htc, 0, sizeof(*htc));
	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
	    IEEE80211_FC0_TYPE_DATA) {
		htc->endpoint_id = usc->ep_data[qid];

		txf = (struct ar_tx_frame *)&htc[1];
		memset(txf, 0, sizeof(*txf));
		txf->data_type = AR_HTC_NORMAL;
		txf->node_idx = an->sta_index;
		txf->vif_idx = 0;
		txf->tid = tid;
		if (m->m_pkthdr.len + IEEE80211_CRC_LEN > ic->ic_rtsthreshold)
			txf->flags |= htobe32(AR_HTC_TX_RTSCTS);
		else if (ic->ic_flags & IEEE80211_F_USEPROT) {
			if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
				txf->flags |= htobe32(AR_HTC_TX_CTSONLY);
			else if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
				txf->flags |= htobe32(AR_HTC_TX_RTSCTS);
		}

		if (k != NULL) {
			/* Map 802.11 cipher to hardware encryption type. */
			if (k->k_cipher == IEEE80211_CIPHER_CCMP) {
				txf->key_type = AR_ENCR_TYPE_AES;
			} else
				panic("unsupported cipher");
			/*
			 * NB: The key cache entry index is stored in the key
			 * private field when the key is installed.
			 */
			txf->key_idx = (uintptr_t)k->k_priv;
		} else
			txf->key_idx = 0xff;

		txf->cookie = an->sta_index;
		frm = (uint8_t *)&txf[1];
	} else {
		htc->endpoint_id = usc->ep_mgmt;

		txm = (struct ar_tx_mgmt *)&htc[1];
		memset(txm, 0, sizeof(*txm));
		txm->node_idx = an->sta_index;
		txm->vif_idx = 0;
		txm->key_idx = 0xff;
		txm->cookie = an->sta_index;
		frm = (uint8_t *)&txm[1];
	}
	/* Copy payload. */
	m_copydata(m, 0, m->m_pkthdr.len, frm);
	frm += m->m_pkthdr.len;
	m_freem(m);

	/* Finalize headers. */
	htc->payload_len = htobe16(frm - (uint8_t *)&htc[1]);
	hdr->len = htole16(frm - (uint8_t *)&hdr[1]);
	xferlen = frm - data->buf;

	usbd_setup_xfer(data->xfer, usc->tx_data_pipe, data, data->buf,
	    xferlen, USBD_FORCE_SHORT_XFER | USBD_NO_COPY, ATHN_USB_TX_TIMEOUT,
	    athn_usb_txeof);
	error = usbd_transfer(data->xfer);
	if (__predict_false(error != USBD_IN_PROGRESS && error != 0)) {
		/* Put this Tx buffer back to our free list. */
		TAILQ_INSERT_TAIL(&usc->tx_free_list, data, next);
		return (error);
	}
	ieee80211_release_node(ic, ni);
	return (0);
#endif
}

void
athn_usb_start(struct ifnet *ifp)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_softc *sc = ifp->if_softc;
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct mbuf *m;

	if (!(ifp->if_flags & IFF_RUNNING) || ifq_is_oactive(&ifp->if_snd))
		return;

	for (;;) {
		if (TAILQ_EMPTY(&usc->tx_free_list)) {
			ifq_set_oactive(&ifp->if_snd);
			break;
		}
		/* Send pending management frames first. */
		m = mq_dequeue(&ic->ic_mgtq);
		if (m != NULL) {
			ni = m->m_pkthdr.ph_cookie;
			goto sendit;
		}
		if (ic->ic_state != IEEE80211_S_RUN)
			break;

		/* Encapsulate and send data frames. */
		m = ifq_dequeue(&ifp->if_snd);
		if (m == NULL)
			break;
#if NBPFILTER > 0
		if (ifp->if_bpf != NULL)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_OUT);
#endif
		if ((m = ieee80211_encap(ifp, m, &ni)) == NULL)
			continue;
 sendit:
#if NBPFILTER > 0
		if (ic->ic_rawbpf != NULL)
			bpf_mtap(ic->ic_rawbpf, m, BPF_DIRECTION_OUT);
#endif
		if (athn_usb_tx(sc, m, ni) != 0) {
			ieee80211_release_node(ic, ni);
			ifp->if_oerrors++;
			continue;
		}

		sc->sc_tx_timer = 5;
		ifp->if_timer = 1;
	}
#endif
}

void
athn_usb_watchdog(struct ifnet *ifp)
{
	printf("%s unimplemented.\n", __func__);
#if 0
	struct athn_softc *sc = ifp->if_softc;

	ifp->if_timer = 0;

	if (sc->sc_tx_timer > 0) {
		if (--sc->sc_tx_timer == 0) {
			printf("%s: device timeout\n", sc->sc_dev.dv_xname);
			/* athn_usb_init(ifp); XXX needs a process context! */
			ifp->if_oerrors++;
			return;
		}
		ifp->if_timer = 1;
	}
	ieee80211_watchdog(ifp);
#endif
}

int
athn_usb_ioctl(struct ieee80211com *ic, u_long cmd, void *data)
{
//	struct athn_softc *sc = ic->ic_softc;
///	struct ifreq *ifr = (struct ifreq *)data;
//	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
//	struct ieee80211com *ic = &sc->sc_ic;
	int error = 0;



//	usbd_ref_incr(usc->sc_udev);

	switch (cmd) {
	case SIOCSIFADDR:
		printf("SIOCIFADDR not implemented!\n");
//		ifp->if_flags |= IFF_UP;
		/* FALLTHROUGH */
	case SIOCSIFFLAGS:
		printf("SIOCSIFFLAGS not implemented!\n");
/*
		if (ifp->if_flags & IFF_UP) {
			if (!(ifp->if_flags & IFF_RUNNING))
				error = athn_usb_init(ifp);
		} else {
			if (ifp->if_flags & IFF_RUNNING)
				athn_usb_stop(ifp);
		}
*/
		break;
	case IEEE80211_IOC_CHANNEL:
//	case SIOCS80211CHANNEL: // OpenBSD version
		printf("SIOCS80211CHANNEL not implemented!\n");
/*
		error = ieee80211_ioctl(ifp, cmd, data);
		if (error == ENETRESET &&
		    ic->ic_opmode == IEEE80211_M_MONITOR) {
			if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
			    (IFF_UP | IFF_RUNNING)) {
				athn_usb_switch_chan(sc, ic->ic_ibss_chan,
				    NULL);
			}
			error = 0;
		}
*/
		break;
	default: {
		struct ieee80211vap *vap;
		struct ifnet *ifp;
		printf("default not implemented! cmd = %lu\n", cmd);
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
			ifp = vap->iv_ifp;
			printf("ifp->if_xname: %s\n", ifp->if_xname);
		}
		error = ENOTTY;
//		error = ieee80211_ioctl(ifp, cmd, data);
	}
	}

	/*
	if (error == ENETRESET) {
		error = 0;
		if ((ifp->if_flags & (IFF_UP | IFF_RUNNING)) ==
		    (IFF_UP | IFF_RUNNING)) {
			athn_usb_stop(ifp);
			error = athn_usb_init(ifp);
		}
	}
	*/

//	usbd_ref_decr(usc->sc_udev);

	return (error);
}

int
athn_usb_init(struct athn_softc *sc)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
//	struct athn_softc *sc = ifp->if_softc;
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	struct athn_ops *ops = &sc->ops;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_channel *c, *extc;
	struct athn_usb_rx_data *data;
	struct ar_htc_target_vif hvif;
	struct ar_htc_target_sta sta;
	struct ar_htc_cap_target hic;
	uint16_t mode;
	int i, error;

	/* Init host async commands ring. */
	usc->cmdq.cur = usc->cmdq.next = usc->cmdq.queued = 0;

	/* Allocate Tx/Rx buffers. */
	error = athn_usb_alloc_rx_list(usc);
	if (error != 0)
		goto fail;
	error = athn_usb_alloc_tx_list(usc);
	if (error != 0)
		goto fail;
	/* Steal one buffer for beacons. */
	usc->tx_bcn = TAILQ_FIRST(&usc->tx_free_list);
	TAILQ_REMOVE(&usc->tx_free_list, usc->tx_bcn, next);

	c = ic->ic_bss->ni_chan = ic->ic_ibss_chan;
	extc = NULL;

	/* In case a new MAC address has been configured. */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, LLADDR(ifp->if_sadl));

	error = athn_set_power_awake(sc);
	if (error != 0)
		goto fail;

	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_FLUSH_RECV);
	if (error != 0)
		goto fail;

	error = athn_hw_reset(sc, c, extc, 1);
	if (error != 0)
		goto fail;

	ops->set_txpower(sc, c, extc);

	mode = htobe16(IEEE80211_IS_CHAN_2GHZ(c) ?
	    AR_HTC_MODE_11NG : AR_HTC_MODE_11NA);
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_SET_MODE,
	    &mode, sizeof(mode), NULL);
	if (error != 0)
		goto fail;

	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_ATH_INIT);
	if (error != 0)
		goto fail;

	error = athn_usb_wmi_cmd(usc, AR_WMI_CMD_START_RECV);
	if (error != 0)
		goto fail;

	athn_rx_start(sc);

	/* Create main interface on target. */
	memset(&hvif, 0, sizeof(hvif));
	hvif.index = 0;
	IEEE80211_ADDR_COPY(hvif.myaddr, ic->ic_myaddr);
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		hvif.opmode = htobe32(AR_HTC_M_STA);
		break;
	case IEEE80211_M_MONITOR:
		hvif.opmode = htobe32(AR_HTC_M_MONITOR);
		break;
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_IBSS:
		hvif.opmode = htobe32(AR_HTC_M_IBSS);
		break;
	case IEEE80211_M_AHDEMO:
		hvif.opmode = htobe32(AR_HTC_M_AHDEMO);
		break;
	case IEEE80211_M_HOSTAP:
		hvif.opmode = htobe32(AR_HTC_M_HOSTAP);
		break;
#endif
	}
	hvif.rtsthreshold = htobe16(ic->ic_rtsthreshold);
	DPRINTF(("creating VAP\n"));
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_VAP_CREATE,
	    &hvif, sizeof(hvif), NULL);
	if (error != 0)
		goto fail;

	/* Create a fake node to send management frames before assoc. */
	memset(&sta, 0, sizeof(sta));
	IEEE80211_ADDR_COPY(sta.macaddr, ic->ic_myaddr);
	sta.sta_index = 0;
	sta.is_vif_sta = 1;
	sta.vif_index = hvif.index;
	sta.maxampdu = 0xffff;
	DPRINTF(("creating default node\n"));
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_CREATE,
	    &sta, sizeof(sta), NULL);
	if (error != 0)
		goto fail;
	usc->free_node_slots = ~(1 << sta.sta_index);

	/* Update target capabilities. */
	memset(&hic, 0, sizeof(hic));
	hic.ampdu_limit = htobe32(0x0000ffff);
	hic.ampdu_subframes = 20;
	hic.txchainmask = sc->txchainmask;
	DPRINTF(("updating target configuration\n"));
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_TARGET_IC_UPDATE,
	    &hic, sizeof(hic), NULL);
	if (error != 0)
		goto fail;

	/* Queue Rx xfers. */
	for (i = 0; i < ATHN_USB_RX_LIST_COUNT; i++) {
		data = &usc->rx_data[i];

		usbd_setup_xfer(data->xfer, usc->rx_data_pipe, data, data->buf,
		    ATHN_USB_RXBUFSZ, USBD_SHORT_XFER_OK | USBD_NO_COPY,
		    USBD_NO_TIMEOUT, athn_usb_rxeof);
		error = usbd_transfer(data->xfer);
		if (error != 0 && error != USBD_IN_PROGRESS)
			goto fail;
	}
	/* We're ready to go. */
	ifp->if_flags |= IFF_RUNNING;
	ifq_clr_oactive(&ifp->if_snd);

#ifdef notyet
	if (ic->ic_flags & IEEE80211_F_WEPON) {
		/* Install WEP keys. */
		for (i = 0; i < IEEE80211_WEP_NKID; i++)
			athn_usb_set_key(ic, NULL, &ic->ic_nw_keys[i]);
	}
#endif
	if (ic->ic_opmode == IEEE80211_M_MONITOR)
		ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
	else
		ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);
	athn_usb_wait_async(usc);
	return (0);
 fail:
	athn_usb_stop(ifp);
	return (error);
#endif
}

static int
athn_usb_stop(device_t self)
{
	printf("%s unimplemented.\n", __func__);
	return 0;
#if 0
	struct athn_softc *sc = ifp->if_softc;
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ar_htc_target_vif hvif;
	uint8_t sta_index;
	int s;

	sc->sc_tx_timer = 0;
	ifp->if_timer = 0;
	ifp->if_flags &= ~IFF_RUNNING;
	ifq_clr_oactive(&ifp->if_snd);

	s = splusb();
	ieee80211_new_state(ic, IEEE80211_S_INIT, -1);

	/* Wait for all async commands to complete. */
	athn_usb_wait_async(usc);

	timeout_del(&sc->scan_to);
	timeout_del(&sc->calib_to);

	/* Remove all non-default nodes. */
	for (sta_index = 1; sta_index < AR_USB_MAX_STA; sta_index++) {
		if (usc->free_node_slots & (1 << sta_index))
			continue;
		(void)athn_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_REMOVE,
		    &sta_index, sizeof(sta_index), NULL);
	}

	/* Remove main interface. This also invalidates our default node. */
	memset(&hvif, 0, sizeof(hvif));
	hvif.index = 0;
	IEEE80211_ADDR_COPY(hvif.myaddr, ic->ic_myaddr);
	(void)athn_usb_wmi_xcmd(usc, AR_WMI_CMD_VAP_REMOVE,
	    &hvif, sizeof(hvif), NULL);

	usc->free_node_slots = 0xff;

	(void)athn_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
	(void)athn_usb_wmi_cmd(usc, AR_WMI_CMD_DRAIN_TXQ_ALL);
	(void)athn_usb_wmi_cmd(usc, AR_WMI_CMD_STOP_RECV);

	athn_reset(sc, 0);
	athn_init_pll(sc, NULL);
	athn_set_power_awake(sc);
	athn_reset(sc, 1);
	athn_init_pll(sc, NULL);
	athn_set_power_sleep(sc);

	/* Abort Tx/Rx. */
	usbd_abort_pipe(usc->tx_data_pipe);
	usbd_abort_pipe(usc->rx_data_pipe);

	/* Free Tx/Rx buffers. */
	athn_usb_free_tx_list(usc);
	athn_usb_free_rx_list(usc);
	splx(s);

	/* Flush Rx stream. */
	m_freem(usc->rx_stream.m);
	usc->rx_stream.m = NULL;
	usc->rx_stream.left = 0;
#endif
}




static device_method_t athn_usb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		athn_usb_match),
	DEVMETHOD(device_attach,	athn_usb_attach),
	DEVMETHOD(device_detach,	athn_usb_detach),
	DEVMETHOD(device_suspend,	athn_usb_stop),
	DEVMETHOD(device_resume,	athn_usb_resume),

	DEVMETHOD_END
};

static driver_t athn_usb_driver = {
	.name = "athn",
	.methods = athn_usb_methods,
	.size = sizeof(struct athn_usb_softc)
};

static devclass_t athn_usb_devclass;

DRIVER_MODULE(athn_usb, uhub, athn_usb_driver, athn_usb_devclass, NULL, NULL);
MODULE_VERSION(athn_usb, 1);
MODULE_DEPEND(athn_usb, usb, 1, 1, 1);
MODULE_DEPEND(athn_usb, wlan, 1, 1, 1);
MODULE_DEPEND(athn_usb, athn, 1, 1, 1);
USB_PNP_HOST_INFO(athn_devs);
