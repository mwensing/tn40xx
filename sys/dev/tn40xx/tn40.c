/*
 * Copyright (c) 2019 Tehuti Networks Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "tn40.h"

static tn40_devid_t  g_tn40DevIds[] =
{
    {TEHUTI_VID,0x4010,TEHUTI_VID, 0x4010, "TN4010 Clean SROM"										},
    {TEHUTI_VID,0x4020,TEHUTI_VID, 0x3015, "TN9030 10GbE CX4 Ethernet Adapter"						},
#ifdef PHY_QT2025
    {TEHUTI_VID,0x4022,TEHUTI_VID, 0x3015, "TN9310 10GbE SFP+ Ethernet Adapter"						},
    {TEHUTI_VID,0x4022,DLINK_VID,  0x4d00, "D-Link DXE-810S 10GbE SFP+ Ethernet Adapter"			},
    {TEHUTI_VID,0x4022,ASUS_VID,   0x8709, "ASUS XG-C100F 10GbE SFP+ Ethernet Adapter"				},
	{TEHUTI_VID,0x4022,EDIMAX_VID, 0x8103, "Edimax 10 Gigabit Ethernet SFP+ PCI Express Adapter"	},
#endif
#ifdef PHY_MV88X3120
	{TEHUTI_VID,0x4024,TEHUTI_VID,  0x3015, "TN9210 10GBase-T Ethernet Adapter"						},
#endif
#ifdef PHY_MV88X3310
	{TEHUTI_VID,0x4027,TEHUTI_VID, 0x3015, "TN9710P 10GBase-T/NBASE-T Ethernet Adapter"				},
	{TEHUTI_VID,0x4027,EDIMAX_VID, 0x8104, "Edimax 10 Gigabit Ethernet PCI Express Adapter"			},
	{TEHUTI_VID,0x4027,BUFFALO_VID,0x0368, "Buffalo LGY-PCIE-MG Ethernet Adapter"},
	{TEHUTI_VID,0x4027,0x1546, 	   0x4027, "IOI9710P 10Gbase-T/NBASE-T Ethernet Adapter"},
#endif
#ifdef PHY_MV88E2010
	{TEHUTI_VID,0x4527,TEHUTI_VID, 0x3015, "TN9710Q 5GBase-T/NBASE-T Ethernet Adapter"				},
#endif
#ifdef PHY_TLK10232
	{TEHUTI_VID,0x4026,TEHUTI_VID, 0x3015, "TN9610 10GbE SFP+ Ethernet Adapter"						},
#endif
#ifdef PHY_AQR105
	{TEHUTI_VID,0x4025,DLINK_VID,  0x2900, "D-Link DXE-810T 10GBase-T Ethernet Adapter"				},
    {TEHUTI_VID,0x4025,TEHUTI_VID, 0x3015, "TN9510 10GBase-T/NBASE-T Ethernet Adapter"				},
	{TEHUTI_VID,0x4025,EDIMAX_VID, 0x8102, "Edimax 10 Gigabit Ethernet PCI Express Adapter"			},
#endif
	{0}
};

/*********************************************************************
 *  Function prototypes
 *********************************************************************/
static int  tn40_probe(device_t);
static int  tn40_attach(device_t);
static int  tn40_detach(device_t);
static int  tn40_shutdown(device_t);
static int  tn40_if_ioctl(struct ifnet *, u_long, caddr_t);
static void	tn40_transmit_deferred(void *context, int pending);


/*********************************************************************
 *  FreeBSD Device Interface Entry Points
 *********************************************************************/

static device_method_t tn40_methods[] =
{
	/* Device interface */
	DEVMETHOD(device_probe, 	tn40_probe),
	DEVMETHOD(device_attach, 	tn40_attach),
	DEVMETHOD(device_detach, 	tn40_detach),
	DEVMETHOD(device_shutdown, 	tn40_shutdown),
	DEVMETHOD_END
};
//-------------------------------------------------------------------------------------------------

static driver_t tn40_driver =
{
	"tn40",
	tn40_methods,
	sizeof(tn40_priv_t),
};
//-------------------------------------------------------------------------------------------------

devclass_t tn40_devclass;
DRIVER_MODULE(tn40, pci, tn40_driver, tn40_devclass, 0, 0);
MODULE_DEPEND(tn40, pci, 1, 1, 1);
MODULE_DEPEND(tn40, ether, 1, 1, 1);

//-------------------------------------------------------------------------------------------------

static void tn40_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{

	if (error == 0)
	{
		*(bus_addr_t *) arg = segs->ds_addr;
	}

} // tn40_dmamap_cb()

//-------------------------------------------------------------------------------------------------

int tn40_dma_tag_create(struct bdx_priv *priv, u32 size, u32 seg_size, int nsegments, bus_dma_tag_t *dma_tag)
{
	int 			err;
	tn40_priv_t		*tn40_priv = CONTAINING_RECORD(priv, tn40_priv_t, bdx_priv);

	ENTER;
	err = bus_dma_tag_create(bus_get_dma_tag(tn40_priv->dev),		/* parent 			 	*/
			       	   	   1, 0,									/* alignment, bounds 	*/
						   BUS_SPACE_MAXADDR,						/* lowaddr 			 	*/
						   BUS_SPACE_MAXADDR,						/* highaddr 		 	*/
						   NULL, NULL,								/* filter, filterarg 	*/
						   size,									/* maxsize 			 	*/
						   nsegments,								/* nsegments 			*/
						   seg_size,								/* maxsegsize 			*/
						   BUS_DMA_ALLOCNOW,						/* flags 				*/
						   NULL,									/* lockfunc 			*/
						   NULL,									/* lockfuncarg 			*/
						   dma_tag);
	RET(err);

} // tn40_dma_tag_create()

//-------------------------------------------------------------------------------------------------

int tn40_alloc_dma(struct bdx_priv *priv, u32 size, tn40_dma_t *dma)
{
	int 			err;

	ENTER;
	do
	{
		err = tn40_dma_tag_create(priv, size, size, 1, &dma->dma_tag);

		if (err != 0)
		{
			ERR("bus_dma_tag_create FAILED !\n");
			break;
		}
		err = bus_dmamem_alloc(dma->dma_tag, &dma->tn40_dmamap.va, BUS_DMA_NOWAIT, &dma->tn40_dmamap.dma_map);
		if (err != 0)
		{
			ERR("tn40_alloc() bus_dmamem_alloc FAILED !\n");
			bus_dma_tag_destroy(dma->dma_tag);
			break;
		}
		err = bus_dmamap_load(dma->dma_tag, dma->tn40_dmamap.dma_map, dma->tn40_dmamap.va, size, tn40_dmamap_cb, (void *)&dma->tn40_dmamap.pa,	BUS_DMA_ZERO | BUS_DMA_NOWAIT);
		if (err != 0)
		{
			ERR("bus_dmamap_load FAILED !\n");
			bus_dmamem_free(dma->dma_tag, dma->tn40_dmamap.va, dma->tn40_dmamap.dma_map);
			bus_dma_tag_destroy(dma->dma_tag);
			break;
		}

	} while (0);

	RET(err);

} // tn40_alloc_dma()

//-------------------------------------------------------------------------------------------------

void tn40_free_dma(tn40_dma_t *dma)
{
	ENTER;
	DBG("dma %p\n", dma);
	if (dma && dma->tn40_dmamap.va != NULL)
	{
		DBG("dma_tag %p dma_map %p\n", dma->dma_tag, dma->tn40_dmamap.dma_map);
		bus_dmamap_sync(dma->dma_tag, dma->tn40_dmamap.dma_map, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(dma->dma_tag, dma->tn40_dmamap.dma_map);
		bus_dmamem_free(dma->dma_tag, dma->tn40_dmamap.va, dma->tn40_dmamap.dma_map);
		bus_dma_tag_destroy(dma->dma_tag);
		dma->tn40_dmamap.va = NULL;
	}
	EXIT;
} // tn40_free_dma()

//-------------------------------------------------------------------------------------------------

u32 tn40_read_reg(struct bdx_priv *priv, u32 reg)
{
    tn40_priv_t		*tn40_priv = CONTAINING_RECORD(priv, tn40_priv_t, bdx_priv);
	return bus_space_read_4(tn40_priv->bus_tag, tn40_priv->bus_handle, reg);

} // tn40_read_reg()


//-------------------------------------------------------------------------------------------------

void tn40_write_reg(struct bdx_priv *priv, u32 reg, u32 value)
{
    tn40_priv_t		*tn40_priv = CONTAINING_RECORD(priv, tn40_priv_t, bdx_priv);

    bus_space_write_4(tn40_priv->bus_tag, tn40_priv->bus_handle, reg, value);

} // tn40_write_reg()

//-------------------------------------------------------------------------------------------------

void tn40_checksum_results(struct mbuf *mBuf, uint32_t pkt_type)
{

    ENTER;
    switch (pkt_type)
    {
		case PKT_TYPE_UDP:
		case PKT_TYPE_TCP:
			mBuf->m_pkthdr.csum_flags = CSUM_L3_CALC | CSUM_L3_VALID | CSUM_L4_CALC | CSUM_L4_VALID;
			mBuf->m_pkthdr.csum_data  = htons(0xffff);
			DBG("%s checksum OK\n",(pkt_type == PKT_TYPE_UDP) ? "UDP" : "TCP");
			break;

		case PKT_TYPE_IP:
			mBuf->m_pkthdr.csum_flags = CSUM_L3_CALC | CSUM_L3_VALID;
			DBG("IP non TCP/UDP checksum OK\n");
			break;

		default:
			mBuf->m_pkthdr.csum_flags = 0;
			DBG("unknown packet type %d !\n",(int)pkt_type);
			break;
    }
    EXIT;

} // tn40_checksum_results

//-------------------------------------------------------------------------------------------------

static int tn40_my_device(uint16_t vendor,uint16_t device, uint16_t sub_vendor,uint16_t sub_device)
{
	uint32_t 	j;
	int 		bFound 	= -1;

	ENTER;
	DBG("v 0x%x d 0x%x\n", vendor, device);
    for(j = 0; g_tn40DevIds[j].vid; j++)
    {
    	DBG("%d. v 0x%x d 0x%x\n", j, g_tn40DevIds[j].vid, g_tn40DevIds[j].pid);
        if((g_tn40DevIds[j].vid 	  == vendor) 	 &&
           (g_tn40DevIds[j].pid 	  == device) 	 &&
		   (g_tn40DevIds[j].subvendor == sub_vendor) &&
		   (g_tn40DevIds[j].subdev 	  == sub_device))
        {
        	bFound = j;
        	DBG("found %d\n", bFound);
        	break;
        }
    }
    RET(bFound);

} // tn40_my_device

//-------------------------------------------------------------------------------------------------

static int tn40_probe(device_t dev)
{
	int	tn40Dev;
	u16	pci_vendor_id 	 = pci_get_vendor(dev);
	u16	pci_device_id 	 = pci_get_device(dev);
	u16	pci_subvendor_id = pci_get_subvendor(dev);
	u16	pci_subdevice_id = pci_get_subdevice(dev);
	int	rVal 			 = ENXIO;

	ENTER;
	if ((tn40Dev = tn40_my_device(pci_vendor_id, pci_device_id, pci_subvendor_id, pci_subdevice_id)) >= 0)
	{
		MSG("Detected 0x%x 0x%x 0x%x 0x%x %s\n", pci_vendor_id,pci_device_id,pci_subvendor_id, pci_subdevice_id, g_tn40DevIds[tn40Dev].name);
		device_set_desc_copy(dev, g_tn40DevIds[tn40Dev].name);
		rVal = BUS_PROBE_DEFAULT;
	}
	else
	{

		DBG("PCI id 0x%x 0x%x 0x%x 0x%x\n", pci_vendor_id,pci_device_id,pci_subvendor_id, pci_subdevice_id);

	}

	RET(rVal);

} // tn40_probe()

static void tn40_vlan(tn40_priv_t *tn40_priv, u16 vtag, int enable)
{
	struct bdx_priv	*bdx_priv 	= &tn40_priv->bdx_priv;

	mtx_lock(&tn40_priv->mtx);
	bdx_vlan_rx_vid(bdx_priv, vtag , enable);
	mtx_unlock(&tn40_priv->mtx);

} // tn40_vlan()
//-------------------------------------------------------------------------------------------------

static void tn40_add_vlan(void *arg, struct ifnet *ifp, u16 vtag)
{
	tn40_priv_t 	*tn40_priv 	= ifp->if_softc;

	tn40_vlan(tn40_priv, vtag, 1);

} // tn40_add_vlan()

//-------------------------------------------------------------------------------------------------

static void tn40_remove_vlan(void *arg, struct ifnet *ifp, u16 vtag)
{
	tn40_priv_t 	*tn40_priv 	= ifp->if_softc;

	tn40_vlan(tn40_priv, vtag, 0);

} // tn40_add_vlan()

//-------------------------------------------------------------------------------------------------

static void tn40_if_init(void *xsc)
{
	tn40_priv_t		*tn40_priv = xsc;
	struct ifnet	*ifp = tn40_priv->ifp;

	ENTER;

	mtx_lock(&tn40_priv->mtx);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	ifp->if_hwassist  = 0;
	if (ifp->if_capenable & IFCAP_TSO4)
	{
		ifp->if_hwassist |= CSUM_TSO;
	}
	if (ifp->if_capenable & IFCAP_TXCSUM)
	{
		ifp->if_hwassist |= (CSUM_TCP | CSUM_UDP);
	}
	mtx_unlock(&tn40_priv->mtx);

} // tn40_if_init()

//-------------------------------------------------------------------------------------------------

static int tn40_transmit_locked(struct ifnet *ifp)
{
	struct mbuf		*mBuf;
	tn40_priv_t		*tn40_priv 	= ifp->if_softc;
	int				err 		= 0;

	while (1)
	{
		if (ifp->if_drv_flags & IFF_DRV_OACTIVE)
		{
			break;;
		}
		if ((mBuf = drbr_peek(ifp, tn40_priv->br)) == NULL)
		{
			break;
		}
		COUNT_INC(tn40_priv->counters, tx_locked_pkts);
		err = bdx_tx_transmit(&tn40_priv->bdx_priv, mBuf);
		if (err != 0)
		{
			drbr_putback(ifp, tn40_priv->br, mBuf);
			DBG2("drbr_putback --> m %p len %d\n", mBuf, mBuf->m_pkthdr.len);
			break;
		}
		drbr_advance(ifp, tn40_priv->br);
		ETHER_BPF_MTAP(ifp, mBuf);
	}

	return err;

} // tn40_transmit_locked()

//-------------------------------------------------------------------------------------------------
/*
static int tn40_transmit_lockedX(struct ifnet *ifp)
{
	struct mbuf	*mBuf;
	int			err = 0;

	tn40_priv_t	*tn40_priv =  ifp->if_softc;

	while ((mBuf = drbr_peek(ifp, tn40_priv->br)) != NULL)
	{
		//DBG2("m %p\n", mBuf);
		err = bdx_tx_transmit(&tn40_priv->bdx_priv, mBuf);
		if (err != 0)
		{
			drbr_putback(ifp, tn40_priv->br, mBuf);
			break;
		}
		drbr_advance(ifp, tn40_priv->br);
		ETHER_BPF_MTAP(ifp, mBuf);
	}

	return err;

} // tn40_transmit_locked()
*/
//-------------------------------------------------------------------------------------------------

static void tn40_transmit_deferred(void *context, int pending)
{
	struct ifnet 	*ifp 		= (struct ifnet *)context;
	tn40_priv_t		*tn40_priv 	=  ifp->if_softc;

	//DBG2("\n");
	mtx_lock(&tn40_priv->tx_lock);
	COUNT_INC(tn40_priv->counters, tx_deferred_pkts);
	if (!drbr_empty(ifp, tn40_priv->br))
	{
		tn40_transmit_locked(ifp);
	}
	mtx_unlock(&tn40_priv->tx_lock);

} // tn40_transmit_deferred()

//-------------------------------------------------------------------------------------------------

static int tn40_transmit(struct ifnet *ifp, struct mbuf *m)
{
	int 		err;
	tn40_priv_t	*tn40_priv =  ifp->if_softc;

	ENTER;

	DBG("m %p\n", m);
	COUNT_INC(tn40_priv->counters, tx_pkts);
	err = drbr_enqueue(ifp, tn40_priv->br, m);
	if (!err)
	{
		if (mtx_trylock(&tn40_priv->tx_lock))
		{
			err = tn40_transmit_locked(ifp);
			mtx_unlock(&tn40_priv->tx_lock);
		}
		else
		{
			taskqueue_enqueue(tn40_priv->tq, &tn40_priv->txq_task);
			DBG("deferred --> m %p len %d\n", m, m->m_pkthdr.len);
		}
	}

	RET(err);

} // tn40_transmit()

//-------------------------------------------------------------------------------------------------
/*
static void tn40_start_locked(struct ifnet *ifp)
{
	tn40_priv_t	*tn40_priv 	=  ifp->if_softc;
	struct mbuf	*mBuf 		= NULL;
	int			err 		= 0;


	while (!IFQ_DRV_IS_EMPTY(&ifp->if_snd))
	{
		IFQ_DRV_DEQUEUE(&ifp->if_snd, mBuf);
		if (mBuf == NULL)
		{
			break;
		}
		err = bdx_tx_transmit(&tn40_priv->bdx_priv, mBuf);
		if (err != 0)
		{
			IFQ_DRV_PREPEND(&ifp->if_snd, mBuf);
			break;
		}

		ETHER_BPF_MTAP(ifp, mBuf);
	}

} // tn40_start_locked()

//-------------------------------------------------------------------------------------------------

static void tn40_start(struct ifnet *ifp)
{
	tn40_priv_t	*tn40_priv =  ifp->if_softc;

	mtx_lock(&tn40_priv->tx_lock);
	tn40_start_locked(ifp);
	mtx_unlock(&tn40_priv->tx_lock);

} // tn40_start()
*/
//-------------------------------------------------------------------------------------------------

static void tn40_if_qflush(struct ifnet *ifp)
{

	ENTER;

	if_qflush(ifp);

	EXIT;

} // tn40_if_qflush()

//-------------------------------------------------------------------------------------------------

static void tn40_media_status(struct ifnet * ifp, struct ifmediareq * ifmr)
{
	tn40_priv_t 	*tn40_priv 	= ifp->if_softc;
	struct bdx_priv	*priv 		= &tn40_priv->bdx_priv;

	mtx_lock(&tn40_priv->mtx);

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER | IFM_FDX;

	if (priv->link_speed)
	{
		ifmr->ifm_status |= IFM_ACTIVE;
		switch (priv->link_speed)
		{
		case  SPEED_10000:
			ifmr->ifm_active |= IFM_10G_T;
			break;

		case  SPEED_5000:
			ifmr->ifm_active |= IFM_5000_T;
			break;

		case  SPEED_2500:
			ifmr->ifm_active |= IFM_2500_T;
			break;

		case  SPEED_1000X:
		case  SPEED_1000:
			ifmr->ifm_active |= IFM_1000_T;
			break;

		case  SPEED_100X:
		case  SPEED_100:
			ifmr->ifm_active |= IFM_100_T;
			break;

		}
	}

	mtx_unlock(&tn40_priv->mtx);

} // tn40_media_status()

//-------------------------------------------------------------------------------------------------
//  ifconfig  for media options

static int tn40_media_change(struct ifnet * ifp)
{
	tn40_priv_t 	*tn40_priv = ifp->if_softc;
	struct bdx_priv	*bdx_priv  = &tn40_priv->bdx_priv;
	struct ifmedia	*ifm = &tn40_priv->media;

	DBG("type 0x%x subtype 0x%x\n", IFM_TYPE(ifm->ifm_media), IFM_SUBTYPE(ifm->ifm_media));
	return bdx_priv->phy_ops.set_settings(bdx_priv, IFM_SUBTYPE(ifm->ifm_media));

} // tn40_media_change()

//-------------------------------------------------------------------------------------------------

static void tn40_isr(void *arg)
{
    u32 isr;
    tn40_priv_t 	*tn40_priv = (tn40_priv_t *)arg;
    struct bdx_priv	*bdx_priv  = &tn40_priv->bdx_priv;

/***
	struct thread 	*td;
	uintptr_t 		stackStart, stackEnd;
	unsigned int	stackSize;
	int				stackPages;

	td = curthread;
	stackStart = (uintptr_t) td->td_kstack;
	stackEnd   = (uintptr_t) td->td_kstack + td->td_kstack_pages * PAGE_SIZE;
	stackPages = td->td_kstack_pages;
	stackSize  = stackEnd - stackStart;

***/



    isr = bdx_isr(bdx_priv);

	if (isr & IR_RX_DESC_0)
	{
		bdx_rx_receive(bdx_priv);
	}
	if (isr & IR_TX_FREE_0)
	{
		mtx_lock(&tn40_priv->tx_lock);
		bdx_tx_cleanup(bdx_priv);
		if (!drbr_empty(tn40_priv->ifp, tn40_priv->br))
		{
			tn40_transmit_locked(tn40_priv->ifp);
		}
		mtx_unlock(&tn40_priv->tx_lock);
	}
	if (isr & (IR_LNKCHG0|IR_LNKCHG1|IR_TMR0))
	{
		bdx_link_changed(bdx_priv);
	}

	bdx_enable_interrupts(bdx_priv);

} // tn40_isr()

//-------------------------------------------------------------------------------------------------

static int tn40_attach(device_t dev)
{
	int						count;
	struct ifnet   			*ifp;
	tn40_priv_t 			*tn40_priv 	= device_get_softc(dev);
	struct bdx_priv			*priv 		= &tn40_priv->bdx_priv;
	int             		rid			= PCIR_BAR(0);
	int             		err 		= ENXIO;

	ENTER;
	do
	{
		bzero(tn40_priv, sizeof(tn40_priv_t));
		tn40_priv->dev 			= dev;
		priv->deviceId 			= pci_get_device(dev);
		priv->subsystem_device	= pci_get_subdevice(dev);
		priv->subsystem_vendor	= pci_get_subvendor(dev);
		mtx_init(&tn40_priv->mtx, device_get_nameunit(dev), MTX_NETWORK_LOCK, MTX_DEF);
		callout_init_mtx(&tn40_priv->callout, &tn40_priv->mtx, 0);
		mtx_init(&tn40_priv->tx_lock, device_get_nameunit(dev), "TN40xx TX", MTX_DEF);
		pci_enable_busmaster(dev);
		tn40_priv->pci_mem 		= bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
		if (tn40_priv->pci_mem == NULL)
		{
			ERR("Failed to allocate PCI memory !\n");
			break;
		}
		tn40_priv->bus_tag 		= rman_get_bustag(tn40_priv->pci_mem);
		tn40_priv->bus_handle 	= rman_get_bushandle(tn40_priv->pci_mem);
		count 					= pci_msi_count(dev);
		if (pci_alloc_msi(dev, &count) != 0)
		{
			ERR("Failed to allocate MSI !\n");
			break;
		}
		tn40_priv->state |= TN40_STATE_ALLOC_MSI;
		rid = 1;
		tn40_priv->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
		if (tn40_priv->irq == NULL)
		{
			ERR("Failed to allocate IRQ !\n");
			break;
		}
		err = tn40_dma_tag_create(priv, TN40_MAX_TSO_PACKET_SIZE, MJUM16BYTES, MAX_SEGMENTS, &priv->tx_dma_tag);
		if (err != 0)
		{
			ERR("tn40_dma_tag_create tx FAILED !\n");
			break;
		}
		err = BUS_SETUP_INTR(device_get_parent(dev), dev, tn40_priv->irq, INTR_TYPE_NET | INTR_MPSAFE,
			    NULL, tn40_isr, tn40_priv, &tn40_priv->intr_handle);
		if (err != 0)
		{
			ERR("bus_setup_intr FAILED !\n");
			break;
		}
		ifmedia_init(&tn40_priv->media, IFM_IMASK, tn40_media_change, tn40_media_status);
		ifp = tn40_priv->ifp = if_alloc(IFT_ETHER);
		if (ifp == NULL)
		{
			ERR("if_alloc FAILED !\n");
			break;
		}
		tn40_priv->br = buf_ring_alloc(TN40_BR_SIZE, M_DEVBUF, M_WAITOK, &tn40_priv->tx_lock);
		if (tn40_priv->br == NULL)
		{
			ERR("buf_ring_alloc FAILED !\n");
			break;
		}
		TASK_INIT(&tn40_priv->txq_task, 0, tn40_transmit_deferred, ifp);
		tn40_priv->tq = taskqueue_create_fast("tn40_que", M_NOWAIT, taskqueue_thread_enqueue, &tn40_priv->tq);
		taskqueue_start_threads(&tn40_priv->tq, 1, PI_NET, "%s que",  device_get_nameunit(tn40_priv->dev));
		ifp->if_softc 		= tn40_priv;
		ifp->if_baudrate 	= 1000000000;
		ifp->if_flags 		= IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
		ifp->if_mtu 		= ETHERMTU;
		ifp->if_init 		= tn40_if_init;
		ifp->if_ioctl 		= tn40_if_ioctl;
		ifp->if_transmit	= tn40_transmit;
//		ifp->if_start 		= tn40_start;
		ifp->if_qflush 		= tn40_if_qflush;
		ifp->if_capabilities= IFCAP_HWCSUM 			| IFCAP_TSO4 		| IFCAP_VLAN_HWCSUM | IFCAP_JUMBO_MTU 	|
							  IFCAP_VLAN_HWTAGGING 	| IFCAP_VLAN_HWTSO 	| IFCAP_VLAN_MTU 	/*| IFCAP_TOE4*/	|
							  IFCAP_VLAN_HWFILTER 	| IFCAP_LINKSTATE 	| IFCAP_HWSTATS;
		if_initname(ifp, device_get_name(dev), device_get_unit(dev));
		ifp->if_capenable = ifp->if_capabilities;
		if (bdx_probe(priv) != 0)
		{
			ERR("bdx_probe FAILED !\n");
			break;
		}
		tn40_priv->vlan_attach = EVENTHANDLER_REGISTER(vlan_config,   tn40_add_vlan,    tn40_priv, EVENTHANDLER_PRI_FIRST);
		tn40_priv->vlan_detach = EVENTHANDLER_REGISTER(vlan_unconfig, tn40_remove_vlan, tn40_priv, EVENTHANDLER_PRI_FIRST);
		ether_ifattach(ifp, priv->mac_address);
		if (bdx_open(priv) != 0)
		{
			ERR("bdx_probe FAILED !\n");
			break;
		}
		err = 0;

		tn40_priv->state |= TN40_STATE_ATTACH;
	} while (0);

	if (err != 0)
	{
		tn40_detach(dev);
	}

	RET(err);

} // tn40_attach()

//-------------------------------------------------------------------------------------------------

static int tn40_detach(device_t dev)
{
	tn40_priv_t 			*tn40_priv 	= device_get_softc(dev);
	struct bdx_priv			*priv 		= &tn40_priv->bdx_priv;
	int						rid = 1;

	ENTER;
	if (tn40_priv->ifp && (tn40_priv->ifp->if_vlantrunk != NULL))
	{
		MSG("Vlan in use, can't detach\n");
		return (EBUSY);
	}
	if (tn40_priv->state & TN40_STATE_ATTACH)
	{
		mtx_lock(&tn40_priv->mtx);
		callout_stop(&tn40_priv->callout);
		mtx_unlock(&tn40_priv->mtx);
		callout_drain(&tn40_priv->callout);
	}
	if (tn40_priv->br != NULL)
	{
		buf_ring_free(tn40_priv->br, M_DEVBUF);
	}
	if (tn40_priv->vlan_attach != NULL)
	{
		EVENTHANDLER_DEREGISTER(vlan_config, tn40_priv->vlan_attach);
	}
	if (tn40_priv->vlan_detach != NULL)
	{
		EVENTHANDLER_DEREGISTER(vlan_unconfig, tn40_priv->vlan_detach);
	}
	if (tn40_priv->ifp)
	{
		ether_ifdetach(tn40_priv->ifp);
		if_free(tn40_priv->ifp);
	}
	bdx_close(priv);
	if (tn40_priv->tq)
	{
		taskqueue_drain(tn40_priv->tq, &tn40_priv->txq_task);
		taskqueue_free(tn40_priv->tq);
	}
	if (tn40_priv->intr_handle)
	{
		BUS_TEARDOWN_INTR(device_get_parent(dev), dev, tn40_priv->irq, tn40_priv->intr_handle);
	}
	if (tn40_priv->irq)
	{
		bus_release_resource(dev, SYS_RES_IRQ, rid, tn40_priv->irq);
	}
	if (tn40_priv->state & TN40_STATE_ALLOC_MSI)
	{
		pci_release_msi(dev);
		tn40_priv->state &= ~TN40_STATE_ALLOC_MSI;
	}
	if (tn40_priv->pci_mem)
	{
		rid	= PCIR_BAR(0);
		bus_release_resource(dev, SYS_RES_MEMORY, rid, tn40_priv->pci_mem);
	}
	if (tn40_priv->state & TN40_STATE_ATTACH)
	{
		mtx_destroy(&tn40_priv->tx_lock);
		mtx_destroy(&tn40_priv->mtx);
	}
	tn40_priv->state &= ~TN40_STATE_ATTACH;

	RET(0);

} // tn40_detach()

///-------------------------------------------------------------------------------------------------

static int tn40_shutdown(device_t dev)
{
	tn40_priv_t 			*tn40_priv 	= device_get_softc(dev);
	struct bdx_priv			*priv 		= &tn40_priv->bdx_priv;

	ENTER;
	mtx_lock(&tn40_priv->mtx);
	callout_stop(&tn40_priv->callout);
	mtx_unlock(&tn40_priv->mtx);

	bdx_remove(priv);

	RET(0);

} // tn40_shutdown()

//-------------------------------------------------------------------------------------------------

static int tn40_if_ioctl(struct ifnet * ifp, u_long command, caddr_t data)
{
	tn40_priv_t 			*tn40_priv 	= ifp->if_softc;
	struct bdx_priv			*priv 		= &tn40_priv->bdx_priv;
	struct ifreq			*ifr 		= (struct ifreq *)data;
	int             		err			= 0;
	int						flags		= 0;

	ENTER;

	switch (command) {
	case SIOCSIFMTU:
		DBG("SIOCSIFMTU %d\n", ifr->ifr_mtu);
		mtx_lock(&tn40_priv->mtx);
		if (bdx_change_mtu(priv, ifr->ifr_mtu) == 0)
		{
			ifp->if_mtu = ifr->ifr_mtu;
		}
		else
		{
			err = EINVAL;
		}
		mtx_unlock(&tn40_priv->mtx);
		break;

	case SIOCSIFFLAGS:
		DBG("SIOCSIFFLAGS 0x%x\n", ifp->if_flags);
		mtx_lock(&tn40_priv->mtx);
		if (ifp->if_flags & IFF_PROMISC)
		{
			flags |= IFF_PROMISC;
		}
		if (ifp->if_flags & IFF_ALLMULTI)
		{
			flags |= IFF_ALLMULTI;
		}
		if (ifp->if_flags & IFF_BROADCAST)
		{
			flags |= IFF_BROADCAST;
		}
		if (memcmp(if_getlladdr(tn40_priv->ifp), priv->mac_address, ETHER_ADDR_LEN))
		{
			DBG("%s MAC change\n", tn40_priv->ifp->if_xname);
			bdx_set_mac(priv, if_getlladdr(tn40_priv->ifp));
		}
		bdx_setbroadcast(priv, flags);
		mtx_unlock(&tn40_priv->mtx);
		break;

	case SIOCADDMULTI:
		DBG("SIOCADDMULTI 0x%x\n", ifp->if_flags);
		mtx_lock(&tn40_priv->mtx);
		bdx_setmulti(priv);
		mtx_unlock(&tn40_priv->mtx);
		break;

	case SIOCDELMULTI:
		DBG("SIOCDELMULTI not implemented\n");
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
	case SIOCGIFXMEDIA:
		DBG("%s\n", (command == SIOCSIFMEDIA) ? "SIOCSIFMEDIA" :
				    (command == SIOCGIFMEDIA) ? "SIOCGIFMEDIA" : "SIOCGIFXMEDIA");
		err = ifmedia_ioctl(ifp, ifr, &tn40_priv->media, command);
		break;

	case SIOCSIFCAP:
	{
		int mask = ifr->ifr_reqcap ^ ifp->if_capenable;
		DBG("SIOCSIFCAP 0x%x\n", mask);
		break;
	}
	case SIOCGPRIVATE_0:
//		err = priv_check(curthread, PRIV_DRIVER);
//		if (err != 0)
//			break;
		err = bdx_ioctl_priv(priv, ifr);
		break;

	case SIOCGIFMAC:
	{
		DBG("SIOCGIFMAC\n");
		break;
	}

	case SIOCSIFMAC:
	{
		DBG("SIOCSIFMAC\n");
		break;
	}

	default:
		err = ether_ioctl(ifp, command, data);
		DBG("Unknown command %lu (0x%X) return %d\n", command & 0xFF, (int)command, err);
		break;
	}

	RET(err);

} // tn40_if_ioctl()



