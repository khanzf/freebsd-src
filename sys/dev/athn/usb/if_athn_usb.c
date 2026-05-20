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

#pragma clang diagnostic ignored "-Wunused-but-set-variable"


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
#include <net/bpf.h>

#include <net80211/ieee80211_var.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include "usbdevs.h"

#include <dev/athn/athnreg.h>
#include <dev/athn/athnvar.h>

#define ECDA_NUM_AC 4
#include <dev/athn/usb/if_athn_usb.h>

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
int		athn_usb_newstate(struct ieee80211vap *, enum ieee80211_state,
		    int);
void		athn_usb_newassoc(struct ieee80211_node *, int);
void		athn_usb_newassoc_cb(struct athn_usb_softc *, void *);
struct ieee80211_node *athn_usb_node_alloc(struct ieee80211vap *, const uint8_t mac[IEEE80211_ADDR_LEN]);
void		athn_usb_count_active_sta(void *, struct ieee80211_node *);
void		athn_usb_newauth_cb(struct athn_usb_softc *, void *);
int		athn_usb_newauth(struct ieee80211com *,
		    struct ieee80211_node *, int, uint16_t);
void		athn_usb_node_free(struct ieee80211_node *);
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
void		athn_usb_rx_radiotap(struct athn_softc *, struct mbuf *,
		    struct ar_rx_status *);
void		athn_usb_rx_frame(struct athn_usb_softc *, struct mbuf *,
		    struct mbufq *);
void		athn_usb_rxeof(struct athn_usb_bulk_rx_data *, int, struct mbufq *);
void		athn_usb_txeof(struct usbd_xfer *, void *);
int		athn_usb_tx(struct athn_usb_softc *, struct ieee80211_node *, struct mbuf *,
		    struct athn_usb_tx_data *, const struct ieee80211_bpf_params *);
void		athn_usb_start(struct athn_softc *);
void		athn_usb_watchdog(struct ifnet *);
int		athn_usb_ioctl(struct ieee80211com *, u_long, void *);
int		athn_usb_init(struct athn_softc *);
void		athn_usb_stop(struct athn_softc *);
void		ar9271_load_ani(struct athn_softc *);
int		ar5008_ccmp_decap(struct athn_softc *, struct mbuf *,
		    struct ieee80211_node *);
int		ar5008_ccmp_encap(struct mbuf *, u_int, struct ieee80211_key *);
static char * athn_usb_wmi_cmd_str(uint16_t cmd_id);

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
void		athn_usb_intr_tx_callback(struct usb_xfer *, usb_error_t);
void		athn_usb_data_rx_callback(struct usb_xfer *, usb_error_t);
void		athn_usb_data_tx_callback(struct usb_xfer *, usb_error_t);
void		athn_usb_intr_rx_callback(struct usb_xfer *, usb_error_t);
int		athn_usb_raw_xmit(struct ieee80211_node *, struct mbuf *, const struct ieee80211_bpf_params *);
static void	athn_usb_shutdown(struct athn_usb_softc *);
static void	athn_usb_reboot(struct athn_usb_softc *);
int		athn_reset_power_on(struct athn_softc *sc);
void		athn_stop_tx_dma(struct athn_softc *sc, int qid);
void		athn_usb_tx_freebuf(struct athn_usb_softc *, struct athn_usb_tx_data *);
struct		athn_usb_tx_data *athn_usb_tx_getbuf(struct athn_usb_softc *);

int wassent = 0;

static void print_hex(const void *buffer, size_t length) {
    const uint8_t *buf = (const uint8_t *)buffer;

    for (size_t i = 0; i < length; i += 16) {
        printf("00%04zX: ", i);  // Print offset starting with "00"
        for (size_t j = 0; j < 16 && (i + j) < length; j++) {
            printf("%02X ", buf[i + j]);  // Print each byte in hex
        }
        printf("\n");
    }
}

// Also a debug function below, delete this later
static void
athn_dump_node_cb(struct ieee80211_node *ni, void *arg)
{
    static int count = 0;

    count++;
    printf("Node %d: %6D  state=%d  associd=0x%04x\n",
        count, ni->ni_macaddr, ":",
        ni->ni_vap ? ni->ni_vap->iv_state : -1,   /* use vap state instead of missing ni_state */
        ni->ni_associd);
}

static void viewframe(struct mbuf *m) {
	int i;
	struct ieee80211_frame *wh;
	wh = mtod(m, struct ieee80211_frame *);

	printf("wh->i_addr1: ");
	for (i = 0; i < IEEE80211_ADDR_LEN; i++)
		printf("%02x%s", (unsigned char)wh->i_addr1[i], (i < IEEE80211_ADDR_LEN - 1) ? ":" : "\n");
	printf("wh->i_addr2: ");
	for (i = 0; i < IEEE80211_ADDR_LEN; i++)
		printf("%02x%s", (unsigned char)wh->i_addr2[i], (i < IEEE80211_ADDR_LEN - 1) ? ":" : "\n");
	printf("wh->i_addr3: ");
	for (i = 0; i < IEEE80211_ADDR_LEN; i++)
		printf("%02x%s", (unsigned char)wh->i_addr3[i], (i < IEEE80211_ADDR_LEN - 1) ? ":" : "\n");
}


#define ATHN_USB_DEV(v, p) { USB_VPI(v, p, 0) }
static const STRUCT_USB_HOST_ID athn_devs[] = {
	ATHN_USB_DEV(USB_VENDOR_ATHEROS2, USB_PRODUCT_ATHEROS2_AR9271U)
};

#define ATHN_CONFIG_INDEX	0

/* Some devices allow 2 extra Rx bulk outputs */
static const struct usb_config athn_config_common[ATHN_N_TRANSFERS] = {
	[ATHN_TX_DATA] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.flags = {
			.short_xfer_ok = 1,
			.pipe_bof = 1
		},
		.callback = athn_usb_data_tx_callback,
		.bufsize = 0x200,	// 512 bytes
		.interval = USB_DEFAULT_INTERVAL,
	},
	[ATHN_RX_DATA] = {
		.type = UE_BULK,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.flags = {
			.short_xfer_ok = 1,
			.pipe_bof = 1
		},
		.callback = athn_usb_data_rx_callback,
		.bufsize = ATHN_USB_RXBUFSZ,	// 512 bytes
		.interval = USB_DEFAULT_INTERVAL,
	},
	[ATHN_RX_INTR] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_IN,
		.flags = {
			.short_xfer_ok = 1,
			.pipe_bof = 1
		},
		.callback = athn_usb_intr_rx_callback,
		.bufsize = 0x40,	// 64 bytes
		.interval = 1
	},
	[ATHN_TX_INTR] = {
		.type = UE_INTERRUPT,
		.endpoint = UE_ADDR_ANY,
		.direction = UE_DIR_OUT,
		.flags = {
			.short_xfer_ok = 1,
			.pipe_bof = 1
		},
		.callback= athn_usb_intr_tx_callback,
		.bufsize = 0x200,	// 512 bytes
		.timeout = ATHN_USB_CMD_TIMEOUT,
		.interval = 1
	}
};


/* FreeBSD additions */
/*
 * Mutex entry state: Locked
 */
void
athn_usb_data_rx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	struct athn_usb_softc *usc = usbd_xfer_softc(xfer);
	struct athn_softc *sc = &usc->sc_sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame_min *wh;
	struct ieee80211_node *ni;
//	struct usb_page_cache *pc;
	struct athn_usb_bulk_rx_data *data;
	struct mbufq ml;
	struct mbuf *m;
	int actlen;
	int err;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	mbufq_init(&ml, 1024); // correct?
	switch(USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		DPRINTF(("TEST DPRINTF\n"));
		data = STAILQ_FIRST(&usc->usc_rx_active);
		if (data == NULL) {
			printf("goto tr_setup\n");
			goto tr_setup;
		}
		STAILQ_REMOVE_HEAD(&usc->usc_rx_active, next);
		// Here we already have data->buf populated
		// So I will print it out here so we can look at the contents.
		// There are magic bytes that we should see after the first 2 bytes.
		athn_usb_rxeof(data, actlen, &ml);
		STAILQ_INSERT_TAIL(&usc->usc_rx_inactive, data, next);

		/* XXX Fall through */
	case USB_ST_SETUP:
tr_setup:
		data = STAILQ_FIRST(&usc->usc_rx_inactive);
		if (data == NULL) {
			printf("early drop 1\n");
			return;
		}
		STAILQ_REMOVE_HEAD(&usc->usc_rx_inactive, next);
		STAILQ_INSERT_TAIL(&usc->usc_rx_active, data, next);

		/* This will usbd_copy_out */
		usbd_xfer_set_frame_data(xfer, 0, data->buf,
			usbd_xfer_max_len(xfer));
		usbd_transfer_submit(xfer);

		// TODO for Farhan: Get rssi and nf

		while ((m=mbufq_dequeue(&ml)) != NULL) {
			ATHN_UNLOCK(sc);

			wh = mtod(m, struct ieee80211_frame_min *);
			
			if (m->m_len >= sizeof(struct ieee80211_frame_min))
				ni = ieee80211_find_rxnode(ic, wh);
			else
				ni = NULL;

//			viewframe(m);

			if (ni != NULL) {
				err = ieee80211_input_mimo(ni, m);
				ieee80211_free_node(ni);
			}
			else
				err = ieee80211_input_mimo_all(ic, m);
			ATHN_LOCK(sc);
		}
		break;
	default: /* Error */
		/* Copied from ural */
		if (error != USB_ERR_CANCELLED) {
		usbd_xfer_set_stall(xfer);
			printf("Error, but setting up TR setup\n");
			goto tr_setup;
		}
		break;
	}

	return;
}


/* FreeBSD additions */
void
athn_usb_data_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	int actlen;
	struct athn_usb_softc *usc = usbd_xfer_softc(xfer);
	struct athn_usb_tx_data *data;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch(USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
		data = STAILQ_FIRST(&usc->usc_tx_active);
		if (data == NULL)
			goto tr_setup;
		STAILQ_REMOVE_HEAD(&usc->usc_tx_active, next);
		STAILQ_INSERT_TAIL(&usc->usc_tx_inactive, data, next);
	case USB_ST_SETUP:
tr_setup:
		data = STAILQ_FIRST(&usc->usc_tx_pending);
		if (data == NULL) {
			printf("usc_tx_pending is NULL at %d\n", __LINE__);
			break;
		}
		STAILQ_REMOVE_HEAD(&usc->usc_tx_pending, next);
		STAILQ_INSERT_TAIL(&usc->usc_tx_active, data, next);
		/*
		if (wassent == 0) {
			printf("--- BEFORE XFER --\n");
			printf("Length: %d\n", data->buflen);
			print_hex(data->buf, data->buflen);
			printf("--- AFTER  XFER --\n");
			wassent++;
		}
		*/
		usbd_xfer_set_frame_data(xfer, 0, data->buf, data->buflen);
		usbd_transfer_submit(xfer);
		break;
	default: /* Error */
		printf("Error occured!\n");
		break;
	}

	return;
}

/* FreeBSD additions */
void
athn_usb_intr_tx_callback(struct usb_xfer *xfer, usb_error_t error)
{
	int actlen;
	struct athn_usb_softc *usc = usbd_xfer_softc(xfer);
	struct athn_usb_tx_data *data = &usc->tx_cmd;

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

		usbd_xfer_set_frame_data(xfer, 0, data->buf, data->buflen);
		usbd_transfer_submit(xfer);

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

	sc->sc_init = athn_usb_init;

	athn_usb_attach_private(usc, USB_GET_DRIVER_INFO(uaa));

	/* Initialize Conditional Variables */
	cv_init(&usc->cv_cmd, "put something");
	cv_init(&usc->cv_msg, "clever here");

	// OpenBSD below
	//usb_init_task(&usc->sc_task, athn_usb_task, sc, USB_TASK_TYPE_GENERIC);
	// FreeBSD side
	mtx_init(&sc->sc_mtx, ic->ic_name, MTX_NETWORK_LOCK, MTX_DEF);
	ATHN_CMDQ_LOCK_INIT(sc);
//	TASK_INIT(&usc->cmdq_task, 0, athn_cmdq_cb, sc);
	TASK_INIT(&usc->usc_task, 0, athn_cmdq_cb, usc);

	if (athn_usb_open_pipes(usc, self) != 0)
		goto fail;

//	STAILQ_INIT(&usc->tx_intr_queue);

	/* Allocate Tx/Rx buffers. */
	error = athn_usb_alloc_rx_list(usc);
	if (error != 0)
		goto fail;
	error = athn_usb_alloc_tx_list(usc);
	if (error != 0)
		goto fail;

	/* Steal one buffer for beacons. */
//	usc->tx_bcn = STAILQ_FIRST(&usc->usc_tx_inactive);
//	printf("steal addr 1: %p\n", usc->tx_bcn);
//	STAILQ_REMOVE(&usc->usc_tx_inactive, usc->tx_bcn, athn_usb_tx_data, next);

	/* Allocate xfer for firmware commands. */
	error = athn_usb_alloc_tx_cmd(usc);
	if (error)
		goto fail;

//	config_mountroot(self, athn_usb_attachhook);
	error = athn_usb_attachhook(self);
	if (error) {
		goto fail;
	}
//	goto fail;
	return 0;

fail:
	athn_usb_detach(self);
	return (ENXIO);
}

/* Very tailored to AR9271 */
static void
athn_usb_shutdown(struct athn_usb_softc *usc)
{
	struct athn_softc *sc = &usc->sc_sc;
//	struct ieee80211com *ic = &sc->sc_ic;
	int ntries;
//	uint32_t val;
//	uint32_t id1;
	int qid;

	/*
	 * Disable interrupts at the MAC
	 * Mask all MAC interrupts
	 * Clear pending causes
	 * Prevent firmware -> Host signaling
	 */
	// Mask all interrupts
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_IER, AR_IER_DISABLE);
	AR_WRITE(sc, AR_INTR_ASYNC_ENABLE, 0);
	AR_WRITE(sc, AR_INTR_SYNC_ENABLE, 0);

	AR_WRITE(sc, AR_INTR_ASYNC_CAUSE, 0xffffffff);
	AR_WRITE(sc, AR_INTR_SYNC_CAUSE, 0xffffffff);

	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);

	/*
	 * Clear Rx enable bit
	 * Waits for Rx DMA idle
	 */
	/* Disable RX */
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_CR, AR_CR_RXD);
	ATHN_UNLOCK(sc);

	/* Wait for RX DMA to go idle */
	for (ntries = 0; ntries < 1000; ntries++) {
		ATHN_LOCK(sc);
		if (!(AR_READ(sc, AR_CR) & AR_CR_RXE)) {
			printf("Good, exiting loop!\n");
			ATHN_UNLOCK(sc);
			break;
		}
		ATHN_UNLOCK(sc);
		DELAY(10);
	}
	if (ntries == 1000) {
		printf("Didn't break from loop, bad\n");
	}
	/* Firmware cannot push frames or descriptors. */
//////////////////////////

	// Abort and fully shutdown Tx DMA
	// Linux equivalent is ath9k_hw_abort_tx_dma

	ATHN_LOCK(sc);
	for(qid=0;qid < ATHN_QID_COUNT;qid++) {
		athn_stop_tx_dma(sc, qid);
	}
	ATHN_UNLOCK(sc);


///

	// Step 2.5 - Disable PLL
//	ATHN_LOCK(sc);
//	AR_WRITE(sc, AR_RTC_PLL_CONTROL, 0x0);
//	AR_WRITE_BARRIER(sc);
//	ATHN_UNLOCK(sc);

//////////////////////////
	// Step 3

	// Now full sleep - ath9k_hw_setpower with ATH9k_PM_FULL_SLEEP
	// Force wake I guess
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);

	// Disable Mac Explicitly
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_CR, AR_CR_RXD);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);

	// Full sleep
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_RTC_RC, AR_RTC_RC_MAC_WARM);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);
	DELAY(50);

	// Commit sleep
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_RTC_RC, 0);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);

	// Release force wake
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_RTC_FORCE_WAKE, 0);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);

	for(ntries = 0; ntries < 1000; ntries++) {
		ATHN_LOCK(sc);
		if (!(AR_READ(sc, AR_RTC_STATUS) & AR_RTC_STATUS_ON)) {
			printf("Break at Rx poll loop, good!\n");
			ATHN_UNLOCK(sc);
			break;
		}
		ATHN_UNLOCK(sc);
		DELAY(10);
	}
	if (ntries == 1000)
		printf("RTC failed to enter full sleep, device needs power cycle\n");


//// Step 4
	// Equivalent of ATH9k_RESET_COLD
	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_RC, AR_RC_AHB | AR_RC_HOSTIF);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);
	DELAY(50);

	ATHN_LOCK(sc);
	AR_WRITE(sc, AR_RC, AR_RC_AHB);
	AR_WRITE_BARRIER(sc);
	ATHN_UNLOCK(sc);

	// Flush posted writes
	ATHN_LOCK(sc);
	(void)AR_READ(sc, AR_RC);
	ATHN_UNLOCK(sc);

	return;

/*
	// Turn the device on (needed for state changes)
	ATHN_LOCK(sc);
	if (athn_reset_power_on(sc)) {
		printf("athn_reset_power_on failure, exiting.\n");
		goto error;
	}
	for(ntries=0 ; ntries < 1000 ; ntries++) {
		val = AR_READ(sc, AR_RTC_STATUS);
		printf("Val %d 0x%x\n", ntries, val);
		if (val & AR_RTC_STATUS_ON)
			break;
	}

	if (ntries == 1000) {
		printf("Unable to power on device during shutdown, exiting.\n");
		goto error;
	}

	// Write the original back address back to the device
	printf("Writing the Mac address + friends\n");
	printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	   ic->ic_macaddr[0], ic->ic_macaddr[1], ic->ic_macaddr[2],
	   ic->ic_macaddr[3], ic->ic_macaddr[4], ic->ic_macaddr[5]);

	printf("Writing STA0 0x%08x\n", *(uint32_t *)ic->ic_macaddr);
	AR_WRITE(sc, AR_STA_ID0, LE_READ_4(&ic->ic_macaddr[0]));
	printf("Reading STA1\n");
	id1 = AR_READ(sc, AR_STA_ID1);
	printf("Read STA1 0x%08x\n", id1);
	printf("Writing STA1 0x%08x\n", *(uint32_t *)(ic->ic_macaddr+4) |
	   id1 | AR_STA_ID1_RTS_USE_DEF | AR_STA_ID1_CRPT_MIC_ENABLE);
	AR_WRITE(sc, AR_STA_ID1, LE_READ_2(&ic->ic_macaddr[4]) |
	   id1 | AR_STA_ID1_RTS_USE_DEF | AR_STA_ID1_CRPT_MIC_ENABLE);

	// Set BSSID mask.
	AR_WRITE(sc, AR_BSSMSKL, 0xffffffff);
	AR_WRITE(sc, AR_BSSMSKU, 0xffff);

	val = AR_READ(sc, AR_OBS_BUS_1);
	printf("Value: 0x%08x\n", val);

	// Disable debug, from ath9k_hw_stopdmarecv
	AR_WRITE(sc, AR_MACMISC, ((AR_MACMISC_DMA_OBS_LINE_8 << AR_MACMISC_DMA_OBS_S) |
	   (AR_MACMISC_MISC_OBS_BUS_1 <<
	   AR_MACMISC_MISC_OBS_BUS_MSB_S)));

	AR_WRITE(sc, AR_CR, AR_CR_RXD);
	val = AR_READ(sc, AR_CR);
	printf("AR_CR 0x%08x = 0x%08x\n", AR_CR, val);


	printf("Writing AR_RC: 0x%08x\n", AR_RC_AHB | AR_RC_HOSTIF);
	AR_WRITE(sc, AR_RC, AR_RC_AHB | AR_RC_HOSTIF);

	// Setting AR_RTC_STATUS to Sleep
	for(ntries = 0; ntries < 1000; ntries++) {
		if (AR_READ(sc, AR_RTC_STATUS) == AR_RTC_STATUS_ON)
			break;
	}
	if (ntries == 1000) {
		printf("Unable to Set ar_rtc_status to on, exiting.\n");
		goto error;
	}

	// Force Wake
	AR_WRITE(sc, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT);
	AR_WRITE(sc, AR_RTC_FORCE_WAKE, AR_RTC_FORCE_WAKE_EN | AR_RTC_FORCE_WAKE_ON_INT);
	AR_WRITE_BARRIER(sc);

//	// From ath9k_hw_set_reset
	val = AR_READ(sc, AR_INTR_SYNC_CAUSE);
	printf("Read AR_INTR_SYNC_CAUSE (0x%04x) = 0x%04x\n", AR_INTR_SYNC_CAUSE, val);
	val &= AR_INTR_SYNC_LOCAL_TIMEOUT | AR_INTR_SYNC_RADM_CPL_TIMEOUT;
	printf("Adjusted AR_INTR_SYNC_CAUSE (0x%04x) = 0x%04x\n", AR_INTR_SYNC_CAUSE, val);

	AR_WRITE(sc, AR_RC, AR_RC_AHB);
	AR_WRITE(sc, AR_RTC_RC, AR_RTC_RC_MAC_WARM);
	AR_WRITE(sc, AR_RTC_RC, 0x0);
	val = AR_READ(sc, AR_RTC_RC);
	printf("AR_RTC_RC: 0x%04x\n", val);
	AR_WRITE_BARRIER(sc);

	athn_init_pll(sc, NULL);
	printf("Exiting correctly!\n");

error:
	ATHN_UNLOCK(sc);
	if (usc->usc_firmware != NULL) {
		firmware_put(usc->usc_firmware, FIRMWARE_UNLOAD);
		usc->usc_firmware = NULL;
	}
	return;
*/
}

/*
 * Send a reboot command to the device to return it to its first-stage
 * bootloader, making it re-enumerable without a power cycle.
 *
 * Linux: ath9k_hif_usb_reboot() sends 0xffffffff via usb_interrupt_msg()
 * to USB_REG_OUT_PIPE (the interrupt OUT endpoint, AR_PIPE_TX_INTR).
 * We use the existing ATHN_TX_INTR xfer and tx_cmd buffer; the transfer
 * will be drained by athn_usb_close_pipes() in Phase 3c.
 */
static void
athn_usb_reboot(struct athn_usb_softc *usc)
{
	struct athn_softc *sc = &usc->sc_sc;
	uint32_t reboot_cmd = 0xffffffff;

	memcpy(usc->tx_cmd.buf, &reboot_cmd, sizeof(reboot_cmd));
	usc->tx_cmd.buflen = sizeof(reboot_cmd);

	ATHN_LOCK(sc);
	usbd_transfer_start(usc->usc_xfer[ATHN_TX_INTR]);
	ATHN_UNLOCK(sc);
}

int
athn_usb_detach(device_t self)
{
	struct athn_usb_softc *usc = device_get_softc(self);
	struct athn_softc     *sc  = &usc->sc_sc;
	struct ieee80211com   *ic  = &sc->sc_ic;
	bool                   unplugged;

	/*
	 * Phase 1: Detect whether the device was physically unplugged or
	 * cleanly detached, and wait for any in-progress firmware load to
	 * complete before proceeding.
	 *
	 * Linux: bool unplugged = (udev->state == USB_STATE_NOTATTACHED);
	 *        wait_for_completion(&hif_dev->fw_done);
	 *
	 *   --> bool unplugged = (usbd_get_state(usc->sc_udev) == USB_STATE_DETACHED);
	 *       taskqueue_drain(taskqueue_thread, &usc->usc_task);
	 */
	unplugged = (usb_get_device_state(usc->sc_udev) == USB_STATE_DETACHED);
	taskqueue_drain(taskqueue_thread, &usc->usc_task);

	/*
	 * Phase 2: Mark device as gone so in-flight WMI commands bail out
	 * early rather than waiting for a response from a detached device.
	 *
	 * Linux: if (hotunplug)
	 *            htc_handle->drv_priv->ah->ah_flags |= AH_UNPLUGGED;
	 */
	if (unplugged)
		usc->usc_unplugged = true;

	/*
	 * Phase 3a: Deinit the 802.11 stack and hardware.  TX/RX buffer frees
	 * are deferred to the end of detach so the buffers remain valid while
	 * pipes are still being torn down in later phases.
	 *
	 * Linux: ath9k_deinit_device(priv):
	 *   wiphy_rfkill_stop_polling(hw->wiphy)  -- no FreeBSD equivalent
	 *   ath9k_deinit_leds(priv)               --> athn_set_led(sc, 0)
	 *   ath9k_htc_deinit_debug(priv)          -- no FreeBSD equivalent
	 *   ieee80211_unregister_hw(hw)           --> ieee80211_ifdetach(ic)
	 *   ath9k_rx_cleanup(priv)                --> athn_usb_free_rx_list(usc)  [deferred]
	 *   ath9k_tx_cleanup(priv)                --> athn_usb_free_tx_list(usc)  [deferred]
	 *                                             athn_usb_free_tx_cmd(usc)   [deferred]
	 *   ath9k_deinit_priv(priv)               --> athn_detach(sc)  [stub, not yet implemented]
	 */
	/* Turn off the LED before stopping the interface, matching the Linux
	 * ordering where ath9k_deinit_leds() is called before
	 * ieee80211_unregister_hw() triggers .stop.  The GPIO write goes
	 * through WMI so the firmware must still be running at this point.
	 */
	if (usc->sc_athn_attached)
		athn_set_led(sc, 0);

	/* Stop the interface if it is still running before unregistering.
	 * In Linux, ieee80211_unregister_hw() triggers .stop internally;
	 * in FreeBSD we must call it explicitly first.
	 * Guard with sc_athn_attached: athn_usb_stop sends WMI commands to
	 * the device and must not be called if firmware was never loaded.
	 */
	if (usc->sc_athn_attached && sc->sc_running)
		athn_usb_stop(sc);
	ieee80211_ifdetach(ic);
	athn_detach(sc);

	/* Release the firmware image; no longer needed once hardware is down. */
	if (usc->usc_firmware != NULL) {
		firmware_put(usc->usc_firmware, FIRMWARE_UNLOAD);
		usc->usc_firmware = NULL;
	}

	/*
	 * Phase 3b: Stop WMI — wake any threads sleeping in athn_usb_wmi_xcmd
	 * waiting for a response that will never arrive.
	 *
	 * Linux: ath9k_stop_wmi(priv)             --> ATHN_LOCK(sc)
	 *   sets wmi->stopped = true under mutex      sc->sc_attached = 0
	 *   so waiters see it and bail out            cv_broadcast(&usc->cv_cmd)
	 *                                             cv_broadcast(&usc->cv_msg)
	 *                                             ATHN_UNLOCK(sc)
	 */
	ATHN_LOCK(sc);
	sc->sc_attached = 0;
	cv_broadcast(&usc->cv_cmd);
	cv_broadcast(&usc->cv_msg);
	ATHN_UNLOCK(sc);

	/*
	 * Phase 4: If this was a clean detach (not a physical unplug), send
	 * a reboot command to return the device to its first-stage bootloader,
	 * making it re-enumerable without a power cycle.
	 *
	 * Linux: if (!unplugged && (hif_dev->flags & HIF_USB_READY))
	 *            ath9k_hif_usb_reboot(udev);
	 *   which sends 0xffffffff as a 4-byte interrupt OUT message to
	 *   USB_REG_OUT_PIPE (the TX interrupt endpoint, AR_PIPE_TX_INTR).
	 *
	 * Note: placed before Phase 3c so the TX interrupt pipe is still open.
	 * athn_usb_close_pipes() in Phase 3c will drain the in-flight transfer.
	 *
	 *   --> if (!unplugged)
	 *           athn_usb_reboot(usc);
	 */
	if (!unplugged && usc->sc_athn_attached)
		athn_usb_reboot(usc);

	/*
	 * Phase 3c: Tear down USB transfers, destroy WMI state, and release
	 * the hardware allocation.
	 *
	 * Linux: ath9k_hif_usb_dealloc_urbs(hif_dev) --> athn_usb_close_pipes(usc)
	 *        ath9k_destroy_wmi(priv)               --> cv_destroy(&usc->cv_cmd)
	 *                                                  cv_destroy(&usc->cv_msg)
	 *        ieee80211_free_hw(hw)                 --> mtx_destroy(&sc->sc_mtx)
	 *        ath9k_htc_hw_free(hif_dev->htc_handle) -- softc freed by USB bus driver
	 */
	athn_usb_close_pipes(usc);
	cv_destroy(&usc->cv_cmd);
	cv_destroy(&usc->cv_msg);
	ATHN_CMDQ_LOCK_DESTROY(sc);
	mtx_destroy(&sc->sc_mtx);

	/*
	 * Phase 5: Release the hif_device_usb allocation.
	 *
	 * Linux: kfree(hif_dev);
	 *        dev_info(&udev->dev, "ath9k_htc: USB layer deinitialized\n");
	 *
	 *   -- softc is freed by the USB bus driver; no explicit free needed.
	 *   --> device_printf(sc->sc_dev, "USB layer deinitialized\n");
	 */
	device_printf(sc->sc_dev, "USB layer deinitialized\n");

	/* Free TX/RX buffers last, after all pipes are closed and no further
	 * DMA activity can reference them.
	 *
	 * Linux: ath9k_rx_cleanup(priv) / ath9k_tx_cleanup(priv)
	 */
	athn_usb_free_rx_list(usc);
	athn_usb_free_tx_list(usc);
	athn_usb_free_tx_cmd(usc);

	return (0);
}

static void
athn_usb_vap_delete(struct ieee80211vap *vap)
{
	struct athn_vap *avp = (struct athn_vap *)vap;

	printf("Running %s %d\n", __func__, __LINE__);
	ieee80211_vap_detach(vap);
	printf("Post %s %d\n", __func__, __LINE__);
	free(avp, M_80211_VAP);
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
//	struct athn_usb_softc *usc = ic->ic_softc;
//	struct ifnet *ifp;

	/* From zyd and rsu, not sure if this applies to athn */
//	if (!TAILQ_EMPTY(&ic->ic_vaps)) {
//		DEBUG_PRINF("VAP create returns null\n");
//		return (NULL);
//	}

	if (opmode == IEEE80211_M_MONITOR) {
		printf("Device is monitor mode\n");
	}

	avp = malloc(sizeof(struct athn_vap), M_80211_VAP, M_WAITOK | M_ZERO);
	vap = &avp->vap;

	if(ieee80211_vap_setup(ic, vap, name, unit, opmode, flags, bssid) != 0) {
		free(avp, M_80211_VAP);
		printf("Exiting VAP early, bad!\n");
		return (NULL);
	}

	/* override state transition machine */
	avp->newstate = vap->iv_newstate;
	vap->iv_newstate = athn_usb_newstate;
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
	struct athn_ops *ops = &sc->ops;
//	struct ieee80211com *ic = &sc->sc_ic;
//	struct ifnet *ifp = &ic->ic_if;
//	int s, i, error;
	int error;

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
	sc->sc_stop = athn_usb_stop;
	sc->sc_start = athn_usb_start;

	error = athn_attach(sc);
	if (error != 0) {
		return (ENXIO);
	}
	usc->sc_athn_attached = 1;
	/* Override some operations for USB. */


	ic->ic_raw_xmit = athn_usb_raw_xmit;
	// Attach VAP-specific "stuff"
	ic->ic_vap_create = athn_usb_vap_create;
	ic->ic_vap_delete = athn_usb_vap_delete; // Not finished
//	ic->ic_ioctl = athn_usb_ioctl;
//	ifp->if_start = athn_usb_start;
//	ifp->if_watchdog = athn_usb_watchdog;
//	ic->ic_newauth = athn_usb_newauth;
	ic->ic_newassoc = athn_usb_newassoc;
//	ic->ic_updateslot = athn_usb_updateslot;	
//	ic->ic_updateedca = athn_usb_updateedca;

	ic->ic_node_alloc = athn_usb_node_alloc;

	usc->node_free = ic->ic_node_free;
	ic->ic_node_free = athn_usb_node_free;

//	vp->vp_set_key = athn_usb_set_key;
//	vp->vp_delete_key = athn_usb_delete_key;	// For VAP
//	ic->ic_set_key = athn_usb_set_key;			// For VAP
//	ic->ic_delete_key = athn_usb_delete_key;	// For VAP
#ifdef notyet
	ic->ic_ampdu_tx_start = athn_usb_ampdu_tx_start;	// For VAP
	ic->ic_ampdu_tx_stop = athn_usb_ampdu_tx_stop;		// For VAP
	sc->sc_stop = athn_usb_stop;
#endif
//	ic->ic_newstate = athn_usb_newstate;
#if 0
	ic->ic_media.ifm_change = athn_usb_media_change;
	timeout_set(&sc->scan_to, athn_usb_next_scan, usc);

#endif
	ops->rx_enable = athn_usb_rx_enable;

	/* Reset HW key cache entries. */
	int i; // XXX In the future move this integer back up
//	sc->kc_entries = 10;
	for (i = 0; i < sc->kc_entries; i++) {
		ATHN_LOCK(sc); // Should the lock be outside of the loop?
		athn_reset_key(sc, i);
		ATHN_UNLOCK(sc);
	}

	ops->enable_antenna_diversity(sc);

#ifdef ATHN_BT_COEXISTENCE
	/* Configure bluetooth coexistence for combo chips. */
	if (sc->flags & ATHN_FLAG_BTCOEX)
		athn_btcoex_init(sc);
#endif
	/* Configure LED. */
	ATHN_LOCK(sc);
	athn_led_init(sc);
	ATHN_UNLOCK(sc);

//	if (bootverbose)
		ieee80211_announce(ic);

	return 0;
}

void
athn_usb_tx_freebuf(struct athn_usb_softc *usc, struct athn_usb_tx_data *bf)
{
//	struct athn_softc *sc = &usc->sc_sc;
//	ATHN_LOCK_ASSERT(sc);
	STAILQ_INSERT_TAIL(&usc->usc_tx_inactive, bf, next);
}

struct athn_usb_tx_data *
athn_usb_tx_getbuf(struct athn_usb_softc *usc)
{
//	struct athn_softc *sc = &usc->sc_sc;
	struct athn_usb_tx_data *bf;

	bf = STAILQ_FIRST(&usc->usc_tx_inactive);
	if (bf != NULL)
		STAILQ_REMOVE_HEAD(&usc->usc_tx_inactive, next);
	else
		bf = NULL;
	return (bf);
}

/*
 * Otus gets and frees the buffer from usc_tx_active, but I'm making this more of a passthrough
 */
int
athn_usb_raw_xmit(struct ieee80211_node *ni, struct mbuf *m, const struct ieee80211_bpf_params *params)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_softc *sc = &usc->sc_sc;
	struct athn_usb_tx_data *bf;
	int error = 0;

	printf("raw xmit\n");

	ATHN_LOCK(sc);
	// XXX printf("The is-running step should be here\n");

	/*
	 * Taken off usc->usc_tx_inactive
	 * Added into usc->usc_tx_pending in athn_usb_tx which does the USB transfer
	 * Consumed and and placed back into inactive in athn_usb_data_tx_callback
	 */
	bf = athn_usb_tx_getbuf(usc);
	if (bf == NULL) {
		error = ENOBUFS;
		goto error;
	}
	if (athn_usb_tx(usc, ni, m, bf, params) != 0) {
		printf("athn_usb_tx fails!\n");
		error = EIO;
		goto error;
	}
	ATHN_UNLOCK(sc);
	return (0);

error:
	if (bf)
		athn_usb_tx_freebuf(usc, bf);
	ATHN_UNLOCK(sc);
	m_free(m);
	return error;
}

int
athn_usb_open_pipes(struct athn_usb_softc *usc, device_t dev)
{
	struct athn_softc *sc = &usc->sc_sc;
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
}

void
athn_usb_close_pipes(struct athn_usb_softc *usc)
{
	usbd_transfer_unsetup(usc->usc_xfer, ATHN_N_TRANSFERS);
}

int
athn_usb_alloc_rx_list(struct athn_usb_softc *usc)
{
	struct athn_softc *sc = &usc->sc_sc;
	struct athn_usb_bulk_rx_data *data;
	int i, error = 0;

	for (i = 0; i < ATHN_USB_RX_LIST_COUNT; i++) {
		data = &usc->rx_data[i];

		data->usc = usc;	/* Backpointer for callbacks. */
		data->m = NULL;
		data->ni = NULL;
		data->buf = malloc(ATHN_USB_RXBUFSZ, M_USBDEV, M_NOWAIT);
		if (data->buf == NULL) {
			device_printf(sc->sc_dev, "could not allocate xfer buffer\n");
			error = ENOMEM;
			break;
		}
	}

	if (error != 0)
		athn_usb_free_rx_list(usc);

	STAILQ_INIT(&usc->usc_rx_active);
	STAILQ_INIT(&usc->usc_rx_inactive);

	for (i = 0; i < ATHN_USB_RX_LIST_COUNT; i++) {
		STAILQ_INSERT_HEAD(&usc->usc_rx_inactive, &usc->rx_data[i], next);
	}
	return (error);
}

void
athn_usb_free_rx_list(struct athn_usb_softc *usc)
{
	struct athn_usb_bulk_rx_data *data;
	int i;

	/* NB: Caller must abort pipe first. */
	for (i = 0; i < ATHN_USB_RX_LIST_COUNT; i++) {
		data = &usc->rx_data[i];
		if (data->buf != NULL)
			free(data->buf, M_USBDEV);
		if (data->ni != NULL)
			ieee80211_free_node(data->ni);
	}
}

int
athn_usb_alloc_tx_list(struct athn_usb_softc *usc)
{
	struct athn_softc *sc = &usc->sc_sc;
	struct athn_usb_tx_data *data;
	int i, error = 0;

	for (i = 0; i < ATHN_USB_TX_LIST_COUNT; i++) {
		data = &usc->tx_data[i];

		data->usc = usc;	/* Backpointer for callbacks. */
		data->m = NULL;
		data->ni = NULL;
		data->buf = malloc(ATHN_USB_TXBUFSZ, M_USBDEV, M_NOWAIT);
		if (data->buf == NULL) {
			device_printf(sc->sc_dev, "could not allocate xfer buffer\n");
			error = ENOMEM;
			break;
		}
	}

	if (error != 0)
		athn_usb_free_tx_list(usc);

	STAILQ_INIT(&usc->usc_tx_active);
	STAILQ_INIT(&usc->usc_tx_pending);
	STAILQ_INIT(&usc->usc_tx_inactive);

	for (i = 0; i < ATHN_USB_TX_LIST_COUNT; i++) {
		STAILQ_INSERT_HEAD(&usc->usc_tx_inactive, &usc->tx_data[i], next);
	}
	return (error);
}

void
athn_usb_free_tx_list(struct athn_usb_softc *usc)
{
	struct athn_usb_tx_data *data;
	int i;

	/* NB: Caller must abort pipe first. */
	for (i = 0; i < ATHN_USB_TX_LIST_COUNT; i++) {
		data = &usc->tx_data[i];
		if (data->buf != NULL)
			free(data->buf, M_USBDEV);
	}
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
	struct athn_usb_tx_data *data = &usc->tx_cmd;

	data->usc = usc;	/* Backpointer for callbacks. */

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
	printf("Why haven't I done this yet?\n");
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

/* Might move this to athn_do_async */
void
athn_usb_do_async(struct athn_usb_softc *usc,
    void (*cb)(struct athn_usb_softc *, void *), void *arg, int len)
{
	struct athn_softc *sc = (struct athn_softc *)usc;
	struct athn_usb_host_cmd_ring *ring = &usc->cmdq;
	struct athn_usb_host_cmd *cmd;

	ATHN_CMDQ_LOCK(sc);

	printf("Comes to athn_usb_do_async...verify\n");

	if (ring->queued == ATHN_USB_HOST_CMD_RING_COUNT) {
		device_printf(sc->sc_dev, "host cmd queue overrun\n");
		return;	/* XXX */
	}
	
	cmd = &ring->cmd[ring->cur];
	cmd->cb = cb;
	KASSERT(len <= sizeof(cmd->data), ("athn_usb_do_async received overflow data size"));
	memcpy(cmd->data, arg, len);
	ring->cur = (ring->cur + 1) % ATHN_USB_HOST_CMD_RING_COUNT;

	ATHN_CMDQ_UNLOCK(sc);

	/* If there is no pending command already, schedule a task. */
	if (++ring->queued == 1)
		ieee80211_runtask(&sc->sc_ic, &usc->usc_task);

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
//	const struct firmware *fw;
	int mlen, size, error = 0;
	uint32_t addr;
	int retries;

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
	usc->usc_firmware = firmware_get(sc->fwname);
	ATHN_UNLOCK(sc);
	if (usc->usc_firmware == NULL) {
		device_printf(sc->sc_dev, "failed to load of file %s\n", sc->fwname);
		error = ENOENT;
		goto error;
//		return (ENOENT);
	}

	ptr = __DECONST(char *, usc->usc_firmware->data);
	addr = AR9271_FIRMWARE >> 8;
	req.bmRequestType = UT_WRITE_VENDOR_DEVICE;
	req.bRequest = AR_FW_DOWNLOAD;
	USETW(req.wIndex, 0);
	size = usc->usc_firmware->datasize;
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
	usbd_transfer_start(usc->usc_xfer[ATHN_RX_INTR]);
	retries = 10;
	while (usbd_transfer_pending(usc->usc_xfer[ATHN_RX_INTR]) && retries--) {
		ATHN_UNLOCK(sc);
		pause("W", hz / 16);
		ATHN_LOCK(sc);
	}
	if (error == 0 && usc->wait_msg_id != 0) {
		error = cv_timedwait(&usc->cv_msg, &sc->sc_mtx, 2 * hz);
		if (error) {
			ATHN_UNLOCK(sc);
			goto error;
		}
	}
	usc->wait_msg_id = 0;
	ATHN_UNLOCK(sc);

error:
	if (usc->usc_firmware) {
		firmware_put(usc->usc_firmware, FIRMWARE_UNLOAD);
		usc->usc_firmware = NULL;
	}
//	if (error != 0)
//		device_printf(sc->sc_dev, "%s: %s: error=%d\n", __func__, name, error);
	return error;
}

int
athn_usb_htc_msg(struct athn_usb_softc *usc, uint16_t msg_id, void *buf,
    int len)
{
	struct athn_softc *sc = &usc->sc_sc;
	struct athn_usb_tx_data *data = &usc->tx_cmd;
	struct ar_htc_frame_hdr *htc;
	struct ar_htc_msg_hdr *msg;

	/* Lock done in athn_usb_htc_connect_svc and athn_usb_htc_setup */
	ATHN_LOCK_ASSERT(sc);

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
	data->buflen = sizeof(*htc) + sizeof(*msg) + len;

	usbd_transfer_start(usc->usc_xfer[ATHN_RX_INTR]);
	usbd_transfer_start(usc->usc_xfer[ATHN_TX_INTR]);
	while (usbd_transfer_pending(usc->usc_xfer[ATHN_TX_INTR])) {
		ATHN_UNLOCK(sc);
		pause("Farhan was here", hz / 16);
		ATHN_LOCK(sc);
	}

	return 0;
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
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_CONTROL,
	    AR_PIPE_TX_INTR, AR_PIPE_RX_INTR, &usc->ep_ctrl);
	if (error != 0) {
		return (error);
	}
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_BEACON,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_bcn);
	if (error != 0)
		return (error);
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_CAB,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_cab);
	if (error != 0)
		return (error);
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_UAPSD,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_uapsd);
	if (error != 0)
		return (error);
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_MGMT,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_mgmt);
	if (error != 0)
		return (error);
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_BE,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[WME_AC_BE]);
	if (error != 0)
		return (error);
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_BK,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[WME_AC_BK]);
	if (error != 0)
		return (error);
	error = athn_usb_htc_connect_svc(usc, AR_SVC_WMI_DATA_VI,
	    AR_PIPE_TX_DATA, AR_PIPE_RX_DATA, &usc->ep_data[WME_AC_VI]);
	if (error != 0)
		return (error);
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
	if (error == 0 && usc->wait_msg_id != 0)
		error = cv_timedwait(&usc->cv_msg, &sc->sc_mtx, hz);
	ATHN_UNLOCK(sc);
	usc->wait_msg_id = 0;
	if (error != 0) {
		return (error);
	}

	ATHN_LOCK(sc);
	error = athn_usb_htc_msg(usc, AR_HTC_MSG_SETUP_COMPLETE, NULL, 0);
	ATHN_UNLOCK(sc);
	if (error != 0) {
		device_printf(sc->sc_dev, "could not complete setup\n");
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
		/* Wait 1 second at most */
		error = cv_timedwait(&usc->cv_msg, &sc->sc_mtx, 10 * hz);
	}
	ATHN_UNLOCK(sc);
	usc->wait_msg_id = 0;
	if (error != 0) {
//		printf("%s: error waiting for service %d connection\n",
//		    usc->usb_dev.dv_xname, svc_id);
		return (error);
	}
	if (rsp.status != AR_HTC_SVC_SUCCESS) {
		device_printf(sc->sc_dev, "service %d connection failed, error %d\n",  svc_id, rsp.status);
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

/*
 * Return WMI command string
 */
static char *
athn_usb_wmi_cmd_str(uint16_t cmd_id)
{
	char *cmd_str = NULL;

	switch(cmd_id) {
	case AR_WMI_CMD_ECHO:
		cmd_str = "AR_WMI_CMD_ECHO";
		break;
	case AR_WMI_CMD_ACCESS_MEMORY:
		cmd_str = "AR_WMI_CMD_ACCESS_MEMORY";
		break;
	case AR_WMI_GET_FW_VERSION:
		cmd_str = "AR_WMI_GET_FW_VERSION";
		break;
	case AR_WMI_CMD_DISABLE_INTR:
		cmd_str = "AR_WMI_CMD_DISABLE_INTR";
		break;
	case AR_WMI_CMD_ENABLE_INTR:
		cmd_str = "AR_WMI_CMD_ENABLE_INTR";
		break;
	case AR_WMI_CMD_ATH_INIT:
		cmd_str = "AR_WMI_CMD_ATH_INIT";
		break;
	case AR_WMI_CMD_ABORT_TXQ:
		cmd_str = "AR_WMI_CMD_ABORT_TXQ";
		break;
	case AR_WMI_CMD_STOP_TX_DMA:
		cmd_str = "AR_WMI_CMD_STOP_TX_DMA";
		break;
	case AR_WMI_CMD_ABORT_TX_DMA:
		cmd_str = "AR_WMI_CMD_ABORT_TX_DMA";
		break;
	case AR_WMI_CMD_DRAIN_TXQ:
		cmd_str = "AR_WMI_CMD_DRAIN_TXQ";
		break;
	case AR_WMI_CMD_DRAIN_TXQ_ALL:
		cmd_str = "AR_WMI_CMD_DRAIN_TXQ_ALL";
		break;
	case AR_WMI_CMD_START_RECV:
		cmd_str = "AR_WMI_CMD_START_RECV";
		break;
	case AR_WMI_CMD_STOP_RECV:
		cmd_str = "AR_WMI_CMD_STOP_RECV";
		break;
	case AR_WMI_CMD_FLUSH_RECV:
		cmd_str = "AR_WMI_CMD_FLUSH_RECV";
		break;
	case AR_WMI_CMD_SET_MODE:
		cmd_str = "AR_WMI_CMD_SET_MODE";
		break;
	case AR_WMI_CMD_NODE_CREATE:
		cmd_str = "AR_WMI_CMD_NODE_CREATE";
		break;
	case AR_WMI_CMD_NODE_REMOVE:
		cmd_str = "AR_WMI_CMD_NODE_REMOVE";
		break;
	case AR_WMI_CMD_VAP_REMOVE:
		cmd_str = "AR_WMI_CMD_VAP_REMOVE";
		break;
	case AR_WMI_CMD_VAP_CREATE:
		cmd_str = "AR_WMI_CMD_VAP_CREATE";
		break;
	case AR_WMI_CMD_REG_READ:
		cmd_str = "AR_WMI_CMD_REG_READ";
		break;
	case AR_WMI_CMD_REG_WRITE:
		cmd_str = "AR_WMI_CMD_REG_WRITE";
		break;
	case AR_WMI_CMD_RC_STATE_CHANGE:
		cmd_str = "AR_WMI_CMD_RC_STATE_CHANGE";
		break;
	case AR_WMI_CMD_RC_RATE_UPDATE:
		cmd_str = "AR_WMI_CMD_RC_RATE_UPDATE";
		break;
	case AR_WMI_CMD_TARGET_IC_UPDATE:
		cmd_str = "AR_WMI_CMD_TARGET_IC_UPDATE";
		break;
	case AR_WMI_CMD_TX_AGGR_ENABLE:
		cmd_str = "AR_WMI_CMD_TX_AGGR_ENABLE";
		break;
	case AR_WMI_CMD_TGT_DETACH:
		cmd_str = "AR_WMI_CMD_TGT_DETACH";
		break;
	case AR_WMI_CMD_NODE_UPDATE:
		cmd_str = "AR_WMI_CMD_NODE_UPDATE";
		break;
	case AR_WMI_CMD_INT_STATS:
		cmd_str = "AR_WMI_CMD_INT_STATS";
		break;
	case AR_WMI_CMD_TX_STATS:
		cmd_str = "AR_WMI_CMD_TX_STATS";
		break;
	case AR_WMI_CMD_RX_STATS:
		cmd_str = "AR_WMI_CMD_RX_STATS";
		break;
	case AR_WMI_CMD_BITRATE_MASK:
		cmd_str = "AR_WMI_CMD_BITRATE_MASK";
		break;
	case AR_WMI_CMD_REG_RMW:
		cmd_str = "AR_WMI_CMD_REG_RMW";
		break;
	default:
		cmd_str = "unknown";
		break;
	}

	return cmd_str;
}

/*
 * Mutex entry state: Locked
 */
int
athn_usb_wmi_xcmd(struct athn_usb_softc *usc, uint16_t cmd_id, void *ibuf,
    int ilen, void *obuf)
{
	struct athn_usb_tx_data *data = &usc->tx_cmd;
	struct athn_softc *sc = &usc->sc_sc;
	struct ar_htc_frame_hdr *htc;
	struct ar_wmi_cmd_hdr *wmi;
	int error;

	if (!sc->sc_attached) {
		return (ENXIO);
	}
	while (usc->wait_cmd_id) {
		/*
		 * The previous USB transfer is not done yet. We can't use
		 * data->xfer until it is done or we'll cause major confusion
		 * in the USB stack.
		 */
		error = cv_timedwait(&usc->cv_msg, &sc->sc_mtx, ATHN_USB_CMD_TIMEOUT); /* Wait 1 second at most */

		if (!sc->sc_attached) {
			return (ENXIO);
		}
	}

	htc = (struct ar_htc_frame_hdr *)data->buf;
	memset(htc, 0, sizeof(*htc));
	htc->endpoint_id = usc->ep_ctrl;
	htc->payload_len = htobe16(sizeof(*wmi) + ilen);

	wmi = (struct ar_wmi_cmd_hdr *)&htc[1];
	wmi->cmd_id = htobe16(cmd_id);
	usc->wmi_seq_no++;
	wmi->seq_no = htobe16(usc->wmi_seq_no);

	memcpy(&wmi[1], ibuf, ilen);

	data->buflen = sizeof(*htc) + sizeof(*wmi) + ilen;

	// Already locked
	usbd_transfer_start(usc->usc_xfer[ATHN_TX_INTR]);

	usc->obuf = obuf;
	usc->wait_cmd_id = cmd_id;
	/*
	 * Wait for WMI command complete interrupt. In case it does not fire
	 * wait until the USB transfer times out to avoid racing the transfer.
	 */
//	printf("msleep wait_cmd_id %d\n", __LINE__);
	error = cv_timedwait(&usc->cv_cmd, &sc->sc_mtx, ATHN_USB_CMD_TIMEOUT);
//	error = msleep(&usc->wait_cmd_id, &sc->sc_mtx, 0, "athnwmi", ATHN_USB_CMD_TIMEOUT);

	if (error == EWOULDBLOCK) {
		device_printf(sc->sc_dev, "firmware command time out: %s (0x%x)\n",
		    athn_usb_wmi_cmd_str(cmd_id),
		    cmd_id);
		error = ETIMEDOUT;
	}

	/*
	 * Both the WMI command and transfer are done or have timed out.
	 * Allow other threads to enter this function and use data->xfer.
	 */
	usc->wait_cmd_id = 0;
	cv_broadcast(&usc->cv_cmd);

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

#if 0
int
athn_usb_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate,
    int arg)
{
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_usb_cmd_newstate cmd;

	/* Do it in a process context. */
	//cmd.state = nstate;
	cmd.arg = arg;
	//athn_usb_do_async(usc, athn_usb_newstate_cb, &cmd, sizeof(cmd));
	return (0);
}
#endif

int
athn_usb_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate,
    int arg)
{
	struct athn_vap *avp = (struct athn_vap *)vap;
	struct ieee80211com *ic = vap->iv_ic;
	struct athn_softc *sc = ic->ic_softc;
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	enum ieee80211_state ostate;
	uint32_t reg, imask;
	int error = 0;

//	timeout_del(&sc->calib_to);

	IEEE80211_UNLOCK(ic);
	ATHN_LOCK(sc);
	ostate = vap->iv_state;

	switch(ostate) {
	case IEEE80211_S_RUN:
		if (ic->ic_opmode == IEEE80211_M_STA) {
			athn_usb_remove_node(usc, vap->iv_bss);
			reg = AR_READ(sc, AR_RX_FILTER);
			reg = (reg & ~AR_RX_FILTER_MYBEACON) |
			    AR_RX_FILTER_BEACON;
			AR_WRITE(sc, AR_RX_FILTER, reg);
			AR_WRITE_BARRIER(sc);
		}
		break;
	case IEEE80211_S_INIT:
	case IEEE80211_S_SCAN:
	case IEEE80211_S_AUTH:
	case IEEE80211_S_ASSOC:
	case IEEE80211_S_CAC:
	case IEEE80211_S_SLEEP:
	default:
		break;
	}

	switch (nstate) {
	case IEEE80211_S_INIT:
		printf("nstate is IEEE80211_S_INIT\n");
		athn_set_led(sc, 0);
		break;
	case IEEE80211_S_SCAN:
		printf("nstate is IEEE80211_S_SCAN\n");
		/* Make the LED blink while scanning. */
		athn_set_led(sc, !sc->led_state);
		printf("This is not complete IEEE80211_S_SCAN\n");
		error = athn_usb_switch_chan(sc, ic->ic_curchan, NULL);
		if (error)
			device_printf(sc->sc_dev, "could not switch to channel %d\n",
			    ieee80211_chan2ieee(ic, ic->ic_curchan));
		break;
	case IEEE80211_S_AUTH:
		printf("This is not complete IEEE80211_S_AUTH, but should at least trigger\n");
		athn_set_led(sc, 0);
		error = athn_usb_switch_chan(sc, ic->ic_curchan, NULL);
		if (error)
			device_printf(sc->sc_dev, "could not switch to channel %d\n",
			    ieee80211_chan2ieee(ic, ic->ic_curchan));
		break;
	case IEEE80211_S_ASSOC:
		break;
	case IEEE80211_S_RUN:

        	if (vap->iv_bss)
			printf("  Entering RUN with AP %6D\n", vap->iv_bss->ni_macaddr, ":");


		athn_set_led(sc, 1);

		if (ic->ic_opmode == IEEE80211_M_MONITOR)
			break;

		if (ic->ic_opmode == IEEE80211_M_STA) {
			/* Create node entry for our BSS */
			error = athn_usb_create_node(usc, vap->iv_bss);
			if (error)
				device_printf(sc->sc_dev, "could not update firmware station table\n");
		}
		athn_set_bss(sc, vap->iv_bss);
		athn_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
//#ifndef IEEE80211_STA_ONLY
		if (ic->ic_opmode == IEEE80211_M_HOSTAP) {
			printf("Not certain that this should be done here.\n");
			athn_usb_switch_chan(sc, ic->ic_curchan, NULL);
			athn_set_hostap_timers(sc);
			/* Enable software beacon alert interrupts. */
			imask = htobe32(AR_IMR_SWBA);
		} else
//#endif
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
	case IEEE80211_S_CSA:
	case IEEE80211_S_SLEEP:
	default:
		break;
	}

	ATHN_UNLOCK(sc);
	IEEE80211_LOCK(ic);

	return (avp->newstate(vap, nstate, arg));
}

void
athn_usb_newassoc(struct ieee80211_node *ni, int isnew)
{
	struct athn_node *an = (struct athn_node *)ni;
	struct ieee80211com *ic = ni->ni_ic;
	struct athn_usb_softc *usc = ic->ic_softc;
	struct ieee80211vap *vap = an->vap;
	printf("Working through %s...\n", __func__);

	if (vap->iv_opmode != IEEE80211_M_HOSTAP &&
	    vap->iv_state != IEEE80211_S_RUN)
		return;

	/* Update the node's supported rates in a process context. */
	ieee80211_ref_node(ni);
	athn_usb_do_async(usc, athn_usb_newassoc_cb, &ni, sizeof(ni));
}

void
athn_usb_newassoc_cb(struct athn_usb_softc *usc, void *arg)
{
	struct ieee80211_node *ni = *(void **)arg;
	struct athn_node *an = (struct athn_node *)ni;
	struct ieee80211vap *vap = an->vap;

	if (vap->iv_state != IEEE80211_S_RUN)
		return;

	/* NB: Node may have left before we got scheduled. */
	if (an->sta_index != 0)
		(void)athn_usb_node_set_rates(usc, ni);
//	ieee80211_release_node(ic, ni);
}

struct ieee80211_node *
athn_usb_node_alloc(struct ieee80211vap *vap, const uint8_t mac[IEEE80211_ADDR_LEN])
{
	struct ieee80211com *ic = vap->iv_ic;
	struct athn_softc *sc = ic->ic_softc;
	struct athn_node *an;

	an = malloc(sizeof(struct athn_node), M_80211_NODE, M_NOWAIT | M_ZERO);
	if (an == NULL) {
		device_printf(sc->sc_dev, "unable to allocate node\n");
		return NULL;
	}
	an->vap = vap;
	return (struct ieee80211_node *)an;
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

void
athn_usb_node_free(struct ieee80211_node *ni)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct athn_usb_softc *usc = ic->ic_softc;
	struct athn_node *an = (struct athn_node *)ni;

	/*
	 * Remove the node from the firmware table in a process context.
	 * Pass an index rather than the pointer which we will free.
	 */
	if (an->sta_index != 0)
		athn_usb_do_async(usc, athn_usb_node_free_cb,
		    &an->sta_index, sizeof(an->sta_index));
	usc->node_free(ni);
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

	// I think I need to put a lock here, but check flow graph

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
	AR_WRITE(sc, AR_CR, AR_CR_RXE);
	AR_WRITE_BARRIER(sc);
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
	cmd_id = be16toh(wmi->cmd_id);

	if (!(cmd_id & AR_WMI_EVT_FLAG)) {
		if (usc->wait_cmd_id != cmd_id) {
			printf("error, fix me\n");
			return;	/* Unexpected reply. */
		}
		if (usc->obuf != NULL) {
			/* Copy answer into caller supplied buffer. */
			memcpy(usc->obuf, &wmi[1], len - sizeof(*wmi));
		}
		/* Notify caller of completion. */
		cv_broadcast(&usc->cv_cmd);
		return;
	}
	switch (cmd_id & 0xfff) {
#ifndef IEEE80211_STA_ONLY
	case AR_WMI_EVT_SWBA:
		athn_usb_swba(usc);
		break;
#endif
	case AR_WMI_EVT_TXSTATUS: {
		/*
		 * Not able to figure out how to replace ic_bss.
		 * OpenBSD uses ic_bss for "this node". I need a
		 * way to map that onto the right VAP and node.
		 */
		struct ar_wmi_evt_txstatus_list *tsl;
		int i;

		tsl = (struct ar_wmi_evt_txstatus_list *)&wmi[1];
		for (i = 0; i < tsl->count && i < nitems(tsl->ts); i++) {
//			struct ieee80211com *ic = &usc->sc_sc.sc_ic;
//			struct ieee80211_node *ni;
			//struct athn_node *an = (struct athn_node *)ic->ic_bss;
//			struct athn_node *an;// = (struct athn_node *)ic->ic_bss;
			struct ar_wmi_evt_txstatus *ts = &tsl->ts[i];
			uint8_t qid;

			/* Skip the node we use to send management frames. */
//			if (ts->cookie == 0)
//				continue;

			/* Skip Tx reports for non-data frame endpoints. */
			qid = (ts->rate & AR_HTC_TXSTAT_EPID) >>
				AR_HTC_TXSTAT_EPID_SHIFT;
			if (qid != usc->ep_data[WME_AC_BE] &&
			    qid != usc->ep_data[WME_AC_BK] &&
			    qid != usc->ep_data[WME_AC_VI] &&
			    qid != usc->ep_data[WME_AC_VO])
				continue;

//			if (ts->cookie == an->sta_index)
//				athn_usb_tx_status(ts, ic->ic_bss);
//			else
//				ieee80211_iterate_nodes(ic, athn_usb_tx_status, ts);
		}
		break;
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
athn_usb_intr_rx_callback(struct usb_xfer *xfer, usb_error_t usb_error)
{
	int actlen;
	struct athn_usb_softc *usc = usbd_xfer_softc(xfer);
	struct ar_htc_frame_hdr *htc;
	struct ar_htc_msg_hdr *msg;
	uint8_t *buf = usc->ibuf;
	struct usb_page_cache *pc = NULL;
	int len;
	uint16_t msg_id;

	usbd_xfer_status(xfer, &actlen, NULL, NULL, NULL);

	switch(USB_GET_STATE(xfer)) {
	case USB_ST_TRANSFERRED:
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
			if (usc->wait_msg_id != msg_id) {
				break;
			}
			usc->wait_msg_id = 0;
			cv_broadcast(&usc->cv_msg);
			break;
		case AR_HTC_MSG_CONN_SVC_RSP:
			if (usc->wait_msg_id != msg_id) {
				break;
			}
			if (usc->msg_conn_svc_rsp != NULL) {
				memcpy(usc->msg_conn_svc_rsp, &msg[1],
					sizeof(struct ar_htc_msg_conn_svc_rsp));
			}
			usc->wait_msg_id = 0;
			cv_broadcast(&usc->cv_msg);
			break;
		case AR_HTC_MSG_CONF_PIPE_RSP:
			if (usc->wait_msg_id != msg_id)
				break;
			usc->wait_msg_id = 0;
			cv_broadcast(&usc->cv_msg);
			break;
		default:
			printf("====HTC message %d ignored\n", msg_id); // This should be a debug message?
			cv_broadcast(&usc->cv_msg); // Should this happen if its truly ignored?
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
	default:
		printf("Rx Interrupt Error... %s 0x%x\n",
		    athn_usb_wmi_cmd_str(usc->wait_msg_id), usc->wait_msg_id);
		wakeup(&usc->wait_msg_id);
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
athn_usb_rx_frame(struct athn_usb_softc *usc, struct mbuf *m, struct mbufq *ml)
{
	struct ieee80211_frame *wh;
	struct ieee80211_rx_stats rxi;
	struct ar_htc_frame_hdr *htc;
	struct ar_rx_status *rs;
	uint16_t datalen;

	if (__predict_false(m->m_len < sizeof(*htc)))
		goto skip;
	htc = mtod(m, struct ar_htc_frame_hdr *);
	printf("htc: endpoint_id=%d, flags=%d, payload_len=%d\n",
	    htc->endpoint_id, htc->flags, be16toh(htc->payload_len));

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
	datalen = htobe16(rs->rs_datalen);
	if (__predict_false(m->m_len < sizeof(*rs) + datalen))
		goto skip;

	if (__predict_false(datalen < sizeof(*wh) + IEEE80211_CRC_LEN))
		goto skip;

#if 0
	if (rs->rs_status != 0) {
		if (rs->rs_status & AR_RXS_RXERR_DECRYPT)
			ic->ic_stats.is_ccmp_dec_errs++;
		ifp->if_ierrors++;
		goto skip;
	}
#else
//	printf("Need to complete this\n");
#endif
	m_adj(m, sizeof(*rs));	/* Strip Rx status. */

	/* Grab a reference to the source node. */
	wh = mtod(m, struct ieee80211_frame *);
//	ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)wh);

	/* Remove any HW padding after the 802.11 header. */
	if (!(wh->i_fc[0] & IEEE80211_FC0_TYPE_CTL)) {
		u_int hdrlen = ieee80211_hdrsize(wh);
		if (hdrlen & 3) {
			memmove((caddr_t)wh + 2, wh, hdrlen);
			m_adj(m, 2);
		}
		wh = mtod(m, struct ieee80211_frame *);
	}
#if 0
#if NBPFILTER > 0
	if (__predict_false(sc->sc_drvbpf != NULL))
		athn_usb_rx_radiotap(sc, m, rs);
#endif
#else
//	printf("Re-implement this later\n");
#endif
	/* Trim 802.11 FCS after radiotap. */
	m_adj(m, -IEEE80211_CRC_LEN);

	/* Send the frame to the 802.11 layer. */
	rxi.r_flags = IEEE80211_R_NF | IEEE80211_R_RSSI;
	rxi.c_nf = AR_USB_DEFAULT_NF; // XXX Apparently Linux might do this correctly
	rxi.c_rssi = rs->rs_rssi;

//	rxi.rxi_rssi = rs->rs_rssi + AR_USB_DEFAULT_NF;
//	rxi.rxi_tstamp = betoh64(rs->rs_tstamp);
/*
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
*/

	if (!ieee80211_add_rx_params(m, &rxi)) {
		printf("Failure to add add rx params\n");
		goto skip;
	}

	printf("ieee80211_add_rx_params succeeded, rssi=%d, nf=%d\n", rxi.c_rssi, rxi.c_nf);

	if (mbufq_enqueue(ml, m)) {
		printf("Unable to queue mbuf\n");
		goto skip;
	}

	return;
 skip:
	m_freem(m);
}

/*
 * I found this function confusing after going through it, so here are some notes.
 * The first time this function is run, stream->left is 0, so the first if-condition
 * will not be run, so you go to the while-statement.
 * `len` is is the bytes we received from the raw wire
 * The while loop will:
 * 1) Verify that the raw frame is set to AR_USB_RX_STREAM_TAG or die
 * 2) Set some lengths
 * 3) If the pktlen (from the device) is <= to MCLBYTES, then
 *    - Allocate a new mbuf and assign pkthdr values
 *    - If pktlen exceeds MHLEN (160), increase its size
 *    Or else set m to NULL
 * 4) If pktlen is above the length of bytes we have (len, the bytes over USB)
 *    this means we need more data and our packet is incomplete (more data coming).
 *    So stream's m (mbuf) is set to m;
 *    If the value is null, then...
 *    Update stream->left (bytes remaining) to bytes size of the total packet minus
 *    bytes received.
 *    Exit early and 'goto submit', to setup another xfer.
 * 5) Otherwise, update the offsets:
 *    Update the pktlen by 0-3 bytes (not sure why)
 *    Update the buffer by that offset and subtract `len` (USB length) by that
 *
 */
void
athn_usb_rxeof(struct athn_usb_bulk_rx_data *data, int len, struct mbufq *ml)
{
	struct athn_usb_softc *usc = data->usc;
	struct athn_usb_rx_stream *stream = &usc->rx_stream;
	uint8_t *buf = data->buf;
	struct ar_stream_hdr *hdr;
	struct mbuf *m;
	uint16_t pktlen;
	int off;

/*
	if (__predict_false(status != USBD_NORMAL_COMPLETION)) {
		DPRINTF(("RX status=%d\n", status));
		if (status == USBD_STALLED)
			usbd_clear_endpoint_stall_async(usc->rx_data_pipe);
		if (status != USBD_CANCELLED)
			goto resubmit;
		return;
	}
*/
//	usbd_get_xfer_status(xfer, NULL, NULL, &len, NULL);

	if (stream->left > 0) {
		if (len >= stream->left) {
			/* We have all our pktlen bytes now. */
			if (__predict_true(stream->m != NULL)) {
				memcpy(mtod(stream->m, uint8_t *) +
				    stream->moff, buf, stream->left);
				athn_usb_rx_frame(usc, stream->m, ml);
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
	KASSERT(stream->left == 0, ("stream->left is not 0"));
	while (len >= sizeof(*hdr)) {
		hdr = (struct ar_stream_hdr *)buf;


		// XXX Debug: print stream header values
		DPRINTF(("tag : %.8x htole16(AR_USB_RX_STREAM_TAG):%.8x hdr->len:%d\n",hdr->tag ,htole16(AR_USB_RX_STREAM_TAG),hdr->len));
		if (hdr->tag != htole16(AR_USB_RX_STREAM_TAG)) {
			DPRINTF(("invalid tag 0x%x\n", hdr->tag));
			break;
		}
		pktlen = htole16(hdr->len);
		buf += sizeof(*hdr);
		len -= sizeof(*hdr);

		if (__predict_true(pktlen <= MCLBYTES)) {
			/* Allocate an mbuf to store the next pktlen bytes. */
			m = m_get2(pktlen, M_NOWAIT, MT_DATA, M_PKTHDR);
			if (__predict_true(m != NULL)) {
				m->m_pkthdr.len = pktlen;
				m->m_len = pktlen;
			}
		} else {
			m = NULL;
		}



//		if (m == NULL)
//			ifp->if_ierrors++;

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

			printf("Frame received, pktlen=%d, first 32 bytes: ", pktlen);
			uint8_t *dbg = mtod(m, uint8_t *);
			for(int i=0;i<min(32, pktlen);i++)
				printf("%02x ", dbg[i]);
			printf("\n");

			athn_usb_rx_frame(usc, m, ml);
		}

		/* Next header is 32-bit aligned. */
		off = (pktlen + 3) & ~3;
		buf += off;
		len -= off;
	} // end of while loop
//	if_input(ifp, &ml);

 resubmit:
	/* Setup a new transfer. */
//	usbd_setup_xfer(xfer, usc->rx_data_pipe, data, data->buf,
//	    ATHN_USB_RXBUFSZ, USBD_SHORT_XFER_OK | USBD_NO_COPY,
//	    USBD_NO_TIMEOUT, athn_usb_rxeof);
//	(void)usbd_transfer(xfer);
	return;
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
	STAILQ_INSERT_TAIL(&usc->tx_free_list, data, next);

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
athn_usb_tx(struct athn_usb_softc *usc, struct ieee80211_node *ni, struct mbuf *m,
    struct athn_usb_tx_data *data, const struct ieee80211_bpf_params *params)
{
	struct athn_softc *sc = (struct athn_softc *)usc;
	struct athn_node *an = (struct athn_node *)ni;
	struct ieee80211vap *vap = an->vap;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_frame *wh;
	struct ieee80211_key *k = NULL;
	struct ar_stream_hdr *hdr;
	struct ar_htc_frame_hdr *htc;
	struct ar_tx_frame *txf;
	struct ar_tx_mgmt *txm;
	uint8_t *frm;
	uint16_t qos;
	uint8_t qid, tid = 0;
//	int hasqos, xferlen, error;

	ATHN_LOCK_ASSERT(sc);

	wh = mtod(m, struct ieee80211_frame *);

	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
		printf("Condition 1, this is not unimplemented, needs to be un-if 0\n");
#if 0
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
#endif
	}
	if (IEEE80211_QOS_HAS_SEQ(wh)) {
		printf("Has QOS\n");
		qos = ((const struct ieee80211_qosframe *)wh)->i_qos[0];
		tid = qos & IEEE80211_QOS_TID;
		qid = WME_AC_TO_TID(tid);
	} else {
		qid = WME_AC_BE;
	}

	/* Grab a Tx buffer from our free list. */
//	data = STAILQ_FIRST(&usc->usc_tx_inactive);
//	STAILQ_REMOVE(&usc->usc_tx_inactive, data, athn_usb_tx_data, next);

//#if NBPFILTER > 0
#if 0
	/* XXX Change radiotap Tx header for USB (no txrate). */
	if (__predict_false(sc->sc_drvbpf != NULL)) {
		struct athn_tx_radiotap_header *tap = &sc->sc_txtap;
		struct mbuf mb;

		tap->wt_flags = 0;
		//tap->wt_chan_freq = htole16(ic->ic_bss->ni_chan->ic_freq);
		tap->wt_chan_freq = htole16(ic->ic_curchan->ic_freq);
		//tap->wt_chan_flags = htole16(ic->ic_bss->ni_chan->ic_flags);
		tap->wt_chan_flags = htole16(ic->ic_curchan->ic_flags);
		mb.m_data = (caddr_t)tap;
		mb.m_len = sc->sc_txtap_len;
		mb.m_next = m;
		mb.m_nextpkt = NULL;
		mb.m_type = 0;
		mb.m_flags = 0;
		bpf_mtap(sc->sc_drvbpf, &mb, BPF_DIRECTION_OUT);
	}
#endif
//#endif

	/* NB: We don't take advantage of USB Tx stream mode for now. */
	hdr = (struct ar_stream_hdr *)data->buf;
	hdr->tag = htole16(AR_USB_TX_STREAM_TAG);

	htc = (struct ar_htc_frame_hdr *)&hdr[1];
	memset(htc, 0, sizeof(*htc));
	if ((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA) {
		printf("Enters condition 1\n");
		htc->endpoint_id = usc->ep_data[qid];

		txf = (struct ar_tx_frame *)&htc[1];
		memset(txf, 0, sizeof(*txf));
		txf->data_type = AR_HTC_NORMAL;
		txf->node_idx = an->sta_index;
		txf->vif_idx = 0;
		txf->tid = tid;
		if (m->m_pkthdr.len + IEEE80211_CRC_LEN > vap->iv_rtsthreshold) {
			printf("Enters condition 2\n");
			txf->flags |= htobe32(AR_HTC_TX_RTSCTS);
		}
		else if (ic->ic_flags & IEEE80211_F_USEPROT) {
			if (ic->ic_protmode == IEEE80211_PROT_CTSONLY) {
				printf("Enters condition 3\n");
				txf->flags |= htobe32(AR_HTC_TX_CTSONLY);
			}
			else if (ic->ic_protmode == IEEE80211_PROT_RTSCTS) {
				printf("Enters condition 4\n");
				txf->flags |= htobe32(AR_HTC_TX_RTSCTS);
			}
		}

		if (k != NULL) {
			printf("Condition %d UNCOMMENT ME!!!! \n", __LINE__);
#if 0
			/* Map 802.11 cipher to hardware encryption type. */
			if (k->k_cipher == IEEE80211_CIPHER_AES_CCM) {
				txf->key_type = AR_ENCR_TYPE_AES;
			} else
				panic("unsupported cipher");
			/*
			 * NB: The key cache entry index is stored in the key
			 * private field when the key is installed.
			 */
			txf->key_idx = (uintptr_t)k->k_priv;
#endif
		} else {
			printf("Condition %d\n", __LINE__);
			txf->key_idx = 0xff;
		}

		txf->cookie = an->sta_index;
		frm = (uint8_t *)&txf[1];
	} else {
//		printf("Last condition\n");
		htc->endpoint_id = usc->ep_mgmt;

		if (wassent == 0) {
			printf("usc->ep_mgmt: %x\n", usc->ep_mgmt);
		}

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
//	m_freem(m); // Done in athn_usb_raw_xmit <-- Wait, do I need this??

	/* Finalize headers. */
	htc->payload_len = htobe16(frm - (uint8_t *)&htc[1]);
	hdr->len = htole16(frm - (uint8_t *)&hdr[1]);

//	xferlen = frm - data->buf;
	data->buflen = frm - data->buf;

	printf("Before Tx Transfer\n");
	STAILQ_INSERT_TAIL(&usc->usc_tx_pending, data, next);
	usbd_transfer_start(usc->usc_xfer[ATHN_TX_DATA]);

#if 0
	usbd_setup_xfer(data->xfer, usc->tx_data_pipe, data, data->buf,
	    xferlen, USBD_FORCE_SHORT_XFER | USBD_NO_COPY, ATHN_USB_TX_TIMEOUT,
	    athn_usb_txeof);
	error = usbd_transfer(data->xfer);
	if (__predict_false(error != USBD_IN_PROGRESS && error != 0)) {
		/* Put this Tx buffer back to our free list. */
		STAILQ_INSERT_TAIL(&usc->usc_tx_inactive, data, next);
		return (error);
	}
	ieee80211_release_node(ic, ni);
#endif
	return (0);
}

void
athn_usb_start(struct athn_softc *sc)
{
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	struct athn_usb_tx_data *bf;
//	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_node *ni;
	struct mbuf *m;

//	if (!(ifp->if_flags & IFF_RUNNING) || ifq_is_oactive(&ifp->if_snd))
//		return;

	if (sc->sc_running == 0) {
		printf("sc_running is 0, not transmitting\n");
		return;
	}

	while ((m = mbufq_dequeue(&sc->sc_snd)) != NULL) {

		/* Buffer taken out here */
		bf = athn_usb_tx_getbuf(usc);
		if (bf == NULL) {
			// Failed to start buffer, prepending
			mbufq_prepend(&sc->sc_snd, m);
			printf("unable to get buffer, breaking in athn_usb_start\n");
			printf("tx_inactive count before loop:\n");
			int cnt = 0;
			struct athn_usb_tx_data *iter;
			STAILQ_FOREACH(iter, &usc->usc_tx_inactive, next) cnt++;
			printf("Inactive: %d\n", cnt);
			cnt = 0;
			STAILQ_FOREACH(iter, &usc->usc_tx_active, next) cnt++;
			printf("Active: %d\n", cnt);
			cnt = 0;
			STAILQ_FOREACH(iter, &usc->usc_tx_pending, next) cnt++;
			printf("Pending: %d\n", cnt);
			break;
		}

		ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;

		if (athn_usb_tx(usc, ni, m, bf, NULL) != 0) {
			athn_usb_tx_freebuf(usc, bf);
//			ieeee80211_free_node(ni);
			m_freem(m);
			break;
		}
	}

#if 0
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

#if 0
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
//		struct ifnet *ifp;
		printf("default not implemented! cmd = %lu\n", cmd);
		TAILQ_FOREACH(vap, &ic->ic_vaps, iv_next) {
			printf("A VAP failed\n");
//			ifp = vap->iv_ifp;
//			printf("ifp->if_xname: %s\n", ic->ic_name);
		}
		error = ENOTTY;
//		error = ieee80211_ioctl(ifp, cmd, data);
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
#endif

int
athn_usb_init(struct athn_softc *sc)
{
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	struct athn_ops *ops = &sc->ops;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_channel *c, *extc;
	struct ar_htc_target_vif hvif;
	struct ar_htc_target_sta sta;
	struct ar_htc_cap_target hic;
	uint16_t mode;
	int error;

	/* Init host async commands ring. */
	ATHN_LOCK(sc);
	usc->cmdq.cur = usc->cmdq.next = usc->cmdq.queued = 0;

	/* Reset RX queue state. */
/*
	STAILQ_INIT(&usc->usc_rx_active);
	STAILQ_INIT(&usc->usc_rx_inactive);
	for (i = 0; i < ATHN_USB_RX_LIST_COUNT; i++) {
	    STAILQ_INSERT_HEAD(&usc->usc_rx_inactive, &usc->rx_data[i], next);
	}
*/
	STAILQ_CONCAT(&usc->usc_rx_inactive, &usc->usc_rx_active);

	/* Steal one buffer for beacons. */
	usc->tx_bcn = STAILQ_FIRST(&usc->usc_tx_inactive);
	STAILQ_REMOVE(&usc->usc_tx_inactive, usc->tx_bcn, athn_usb_tx_data, next);

	c = ic->ic_curchan;
	extc = NULL;
//	DEBUG_PRINTF("temporarily cutting out iv_bss->ni_chan, DEFINITELY do not ignore this\n");

	/* In case a new MAC address has been configured. */
//	IEEE80211_ADDR_COPY(ic->ic_myaddr, LLADDR(ifp->if_sadl));

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
	IEEE80211_ADDR_COPY(hvif.myaddr, ic->ic_macaddr);
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		printf("IEEE80211_M_STA condition\n");
		hvif.opmode = htobe32(AR_HTC_M_STA);
		break;
	case IEEE80211_M_MONITOR:
		printf("IEEE80211_M_MONITOR condition\n");
		hvif.opmode = htobe32(AR_HTC_M_MONITOR);
		break;
#ifndef IEEE80211_STA_ONLY
	case IEEE80211_M_IBSS:
		printf("IEEE80211_M_IBSS condition\n");
		hvif.opmode = htobe32(AR_HTC_M_IBSS);
		break;
	case IEEE80211_M_AHDEMO:
		printf("IEEE80211_M_AHDEMO condition\n");
		hvif.opmode = htobe32(AR_HTC_M_AHDEMO);
		break;
	case IEEE80211_M_HOSTAP:
		printf("IEEE80211_M_HOSTAP condition\n");
		hvif.opmode = htobe32(AR_HTC_M_HOSTAP);
		break;
	default:
		printf("Go to failure condition\n");
		goto fail;
#endif
	}
//	hvif.rtsthreshold = htobe16(ic->ic_rtsthreshold);
	DPRINTF(("creating VAP\n"));
	error = athn_usb_wmi_xcmd(usc, AR_WMI_CMD_VAP_CREATE,
	    &hvif, sizeof(hvif), NULL);
	if (error != 0)
		goto fail;

	/* Create a fake node to send management frames before assoc. */
	memset(&sta, 0, sizeof(sta));
	IEEE80211_ADDR_COPY(sta.macaddr, ic->ic_macaddr);
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
	usbd_transfer_start(usc->usc_xfer[ATHN_RX_DATA]);

	/* We're ready to go. */
	sc->sc_running = 1;
	printf("Setting sc->sc_running = 1\n");

	printf("Stuff below this needs to be updated...\n");
	ATHN_UNLOCK(sc);
	return (0);
#if 0
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
#endif
//	athn_usb_wait_async(usc);
 fail:
	ATHN_UNLOCK(sc);
	printf("Device failed to load, athn_usb_init\n");
//	athn_usb_stop(ifp);
	return (error);
}

static int
athn_usb_suspend(device_t self)
{
#if 0
	int error;
	struct athn_usb_softc *usc = device_get_softc(self);
	// Transparent handler
	error = athn_usb_stop(sc);

	return (error);
#endif
	printf("Currently athn_usb_suspend disabled\n");
	return 1;
}


void
athn_usb_stop(struct athn_softc *sc)
{
	struct athn_usb_softc *usc = (struct athn_usb_softc *)sc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ar_htc_target_vif hvif;
	uint8_t sta_index;

	/* Linux: test_bit(ATH_OP_INVALID) guard */
	if (sc->sc_attached == 0)
		return;

	/* Wake the hardware before issuing any WMI commands.
	 * Linux: ath9k_htc_ps_wakeup(priv) -> ath9k_hw_setpower(ATH9K_PM_AWAKE)
	 */
	ATHN_LOCK(sc);
	athn_set_power_awake(sc);
	ATHN_UNLOCK(sc);

	/* Wait for all async commands to complete.
	 * Linux: implicit via mutex ordering before WMI commands.
	 */
	ATHN_LOCK(sc);
	athn_usb_wait_async(usc);
	ATHN_UNLOCK(sc);

	/* Stop scan and calibration timers.
	 * Linux: ath9k_htc_stop_ani() -> cancel_delayed_work_sync(&ani_work)
	 *        cancel_work_sync(fatal_work, ps_work, led_work) -- no FreeBSD equivalent
	 */
	ATHN_LOCK(sc);
	callout_stop(&sc->scan_to);
	callout_stop(&sc->calib_to);
	ATHN_UNLOCK(sc);

	/* Disable Bluetooth coexistence if active.
	 * Linux: ath9k_htc_stop_btcoex(priv) -> ath9k_hw_btcoex_disable()
	 */
	if (sc->flags & ATHN_FLAG_BTCOEX)
		athn_btcoex_disable(sc);

	/* Remove monitor interface if present.
	 * Linux: if (priv->ah->is_monitoring) ath9k_htc_remove_monitor_interface()
	 * Not yet implemented in this driver.
	 */

	/* Remove all non-default nodes. */
	ATHN_LOCK(sc);
	for (sta_index = 1; sta_index < AR_USB_MAX_STA; sta_index++) {
		if (usc->free_node_slots & (1 << sta_index))
			continue;
		(void)athn_usb_wmi_xcmd(usc, AR_WMI_CMD_NODE_REMOVE,
		    &sta_index, sizeof(sta_index), NULL);
	}
	ATHN_UNLOCK(sc);

	/* Remove main interface. This also invalidates our default node. */
	memset(&hvif, 0, sizeof(hvif));
	hvif.index = 0;
	IEEE80211_ADDR_COPY(hvif.myaddr, ic->ic_macaddr);
	ATHN_LOCK(sc);
	(void)athn_usb_wmi_xcmd(usc, AR_WMI_CMD_VAP_REMOVE,
	    &hvif, sizeof(hvif), NULL);
	ATHN_UNLOCK(sc);

	usc->free_node_slots = 0xff;

	/* Send WMI shutdown commands to the firmware.
	 * Linux: WMI_CMD(WMI_DISABLE_INTR_CMDID)
	 *        WMI_CMD(WMI_DRAIN_TXQ_ALL_CMDID)
	 *        WMI_CMD(WMI_STOP_RECV_CMDID)
	 */
	ATHN_LOCK(sc);
	(void)athn_usb_wmi_cmd(usc, AR_WMI_CMD_DISABLE_INTR);
	(void)athn_usb_wmi_cmd(usc, AR_WMI_CMD_DRAIN_TXQ_ALL);
	(void)athn_usb_wmi_cmd(usc, AR_WMI_CMD_STOP_RECV);
	ATHN_UNLOCK(sc);

	/* Drain all USB transfers — bulk and interrupt endpoints.
	 * Linux: ath9k_htc_tx_drain() drains SW queues and kills tasklets;
	 *        ath9k_wmi_event_drain() kills the WMI event tasklet and purges
	 *        the event queue.  FreeBSD drains at the USB transfer level.
	 */
	usbd_transfer_drain(usc->usc_xfer[ATHN_TX_DATA]);
	usbd_transfer_drain(usc->usc_xfer[ATHN_RX_DATA]);
	usbd_transfer_drain(usc->usc_xfer[ATHN_TX_INTR]);
	usbd_transfer_drain(usc->usc_xfer[ATHN_RX_INTR]);

	/* Reset and power down the hardware.
	 * Linux: ath9k_hw_phy_disable() + ath9k_hw_disable()
	 *        ath9k_htc_ps_restore() + ath9k_htc_setpower(ATH9K_PM_FULL_SLEEP)
	 */
	ATHN_LOCK(sc);
	athn_reset(sc, 0);
	athn_init_pll(sc, NULL);
	athn_set_power_awake(sc);
	athn_reset(sc, 1);
	athn_init_pll(sc, NULL);
	athn_set_power_sleep(sc);
	ATHN_UNLOCK(sc);

	/* Flush the Rx stream reassembly buffer. */
	m_freem(usc->rx_stream.m);
	usc->rx_stream.m = NULL;
	usc->rx_stream.left = 0;

	/* Linux: set_bit(ATH_OP_INVALID) */
	sc->sc_running = 0;
}

static device_method_t athn_usb_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		athn_usb_match),
	DEVMETHOD(device_attach,	athn_usb_attach),
	DEVMETHOD(device_detach,	athn_usb_detach),
	DEVMETHOD(device_suspend,	athn_usb_suspend),
	DEVMETHOD(device_resume,	athn_usb_resume),

	DEVMETHOD_END
};

static driver_t athn_usb_driver = {
	.name = "athn",
	.methods = athn_usb_methods,
	.size = sizeof(struct athn_usb_softc)
};

DRIVER_MODULE(athn_usb, uhub, athn_usb_driver, NULL, NULL);
MODULE_VERSION(athn_usb, 1);
MODULE_DEPEND(athn_usb, usb, 1, 1, 1);
MODULE_DEPEND(athn_usb, wlan, 1, 1, 1);
MODULE_DEPEND(athn_usb, athn, 1, 1, 1);
//MODULE_DEPEND(athn_usb, athn-ar9271fw, 0f, 1, 1);

USB_PNP_HOST_INFO(athn_devs);
