/* **************** Eth_Driver.c **************** */
/*
 * The code herein is: Copyright Pavel Teixeira, 2014
 *
 * This Copyright is retained for the purpose of protecting free
 * redistribution of source.
 *
 *     email:  gal[dot]prime[dot]kr[at]gmail[dot]com
 *
 * The primary maintainer for this code is Pavel Teixeira
 * The CONTRIBUTORS file (distributed with this
 * file) lists those known to have contributed to the source.
 *
 * This code is distributed under Version 2 of the GNU General Public
 * License, which you should have received with the source.
 *
 */
/*
 * Building a Transmitting Network Driver skeleton
 *
 * This skeleton handles with the emission of packets 
 * function, which means that supplys a method for
 * ndo_start_xmit().
 *
 * While you are at it, you may want to add other entry points to see
 * how you may exercise them.
 *
 * Once again, you should be able to exercise it with:
 *
 *   insmod Eth_Driver.ko
 *   ifconfig mynet0 up 192.168.3.197
 *   ping -I mynet0 localhost
 *       or
 *   ping -bI mynet0 192.168.3
 *
 * Make sure your chosen address is not being used by anything else.
 *
 @*/
#include <linux/module.h>			// Recognizes that it's a module.
#include <linux/netdevice.h>		// To use the net features.
#include <linux/init.h>				// Initialize the module.
#include <linux/kernel.h>			// Uses the kernel functions.
#include <linux/skbuff.h>			// Packet manipulations.
#include <linux/etherdevice.h>		// For the device.
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/irqreturn.h>
#include <linux/pci.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <net/genetlink.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/if_ether.h>
#include <linux/fs.h>


/* Driver specific definitions. */
#include "NG10.h"
#include "ocdp.h"

/* Length definitions */
//#define mac_addr_len                    6
#define RX_POLL_WEIGHT					64
#define NUM_NETDEVS						4

 /* Driver version. */
#define ETH_DRIVER_VERSION     "1.0.0"

 char driver_name[] = "Eth_driver";

/* These are the flags in the statusword */
#define ETH_RX_INTR 0x0001
#define ETH_TX_INTR 0x0002

 /* MMIO */
/* Like the DMA variables, probe() and remove() both use bar0_base_va, so need
* to make global. FIXME: explore more elegant way of doing this. */
void __iomem *bar0_base_va;
unsigned int bar0_size;

/* DMA */
/* *_dma_reg_* need these to be global for now since probe() and remobe() both use them.
* FIXME: possible that we can do something more elegant. */
void *tx_dma_reg_va; /* TX DMA region kernel virtual address. */
dma_addr_t tx_dma_reg_pa; /* TX DMA region physical address. */
void *rx_dma_reg_va; /* RX DMA region kernel virtual address. */
dma_addr_t rx_dma_reg_pa; /* RX DMA region physical address. */
struct dma_stream tx_dma_stream; /* To device. */
struct dma_stream rx_dma_stream; /* From device. */

/* Need a locking mechanism to control concurrent access to each DMA region. */
static DEFINE_SPINLOCK(tx_dma_region_spinlock);
static DEFINE_SPINLOCK(rx_dma_region_spinlock);

/* DMA parameters. */
#define     DMA_BUF_SIZE        2048    /* Size of buffer for each DMA transfer. Property of the hardware. */
#define     DMA_FPGA_BUFS       4       /* Number of buffers on the FPGA side. Property of the hardware. */
#define     DMA_CPU_BUFS        32768   /* Number of buffers on the CPU side. */
#define     MIN_DMA_CPU_BUFS    1       /* Minimum number of buffers on the CPU side. */

/* Total size of a DMA region (1 region for TX, 1 region for RX). */
#define     DMA_REGION_SIZE     ((DMA_BUF_SIZE + OCDP_METADATA_SIZE + sizeof(uint32_t)) * (DMA_CPU_BUFS))

/* Proc variables */
struct proc_dir_entry *proc;
int len,temp;
char *msg;

uint32_t dma_cpu_bufs;
uint32_t dma_region_size;


/* Bundle of variables to keep track of a unidirectional DMA stream. */
struct dma_stream {
	uint8_t *buffers;
	OcdpMetadata *metadata;
	volatile uint32_t *flags;
	volatile uint32_t *doorbell;
	uint32_t buf_index;
};


/* net_device referencing */
/* Network devices to be registered with the kernel. */
static struct net_device *device[NUM_NETDEVS];
static struct net_device_stats *stats;

/* priv structure that holds the informations about the device. */
struct eth_priv {
	struct net_device_stats stats;
	struct napi_struct napi;
	int status;
	struct eth_packet* ppool;
	struct eth_packet* rx_queue; /* List of incoming packets */
	int rx_int_enabled;
	int tx_packetlen;
	u8* tx_packetdata;
	struct sk_buff* skb;
	spinlock_t lock;
	struct net_device *dev;
};

/* Structure that holds the informations about the packets. */
struct eth_packet {
	struct eth_packet* next;
	struct net_device* dev;
	int datalen;
	u8 data[1500];
};

// static struct pci_driver eth_pci_driver = {
//     .name       = "eth_driver: pci_driver",
//     .id_table   = id_table,
//     .probe      = probe,
//     .remove     = remove,
// };

/* These are the IDs of the PCI devices that this Ethernet driver supports. */
static struct pci_device_id id_table[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_ETH, PCI_DEVICE_ID_ETH_REF_NIC), }, /* NetFPGA-10G Reference NIC. */
    { 0, }
};
/* Creates a symbol used by depmod to tell the kernel that these devices
* are associated with this driver module. This is communicated via the
* /lib/modules/KVERSION/modules.pcimap file, written to by depmod. */
MODULE_DEVICE_TABLE(pci, id_table);

/* Functions prototypes up here.*/
void Eth_teardown_pool (struct net_device* dev);
__be16 eth_type_trans(struct sk_buff *skb, struct net_device *dev);
int Eth_start_xmit(struct sk_buff *skb, struct net_device *dev);
void Eth_teardown_pool (struct net_device* dev);
static int Eth_napi_struct_poll(struct napi_struct *napi, int budget);
void eth_release_buffer(struct eth_packet *pkt);
void Eth_tx_timeout(struct net_device *dev);
static void Eth_rx_ints(struct net_device *dev, int enable);
static int probe(struct pci_dev *pdev, const struct pci_device_id *id);


/************************************************************/



/* DMA */
/* *_dma_reg_* need these to be global for now since probe() and remove() both use them.
 * FIXME: possible that we can do something more elegant. */
void                *tx_dma_reg_va; /* TX DMA region kernel virtual address. */
dma_addr_t          tx_dma_reg_pa;  /* TX DMA region physical address. */
void                *rx_dma_reg_va; /* RX DMA region kernel virtual address. */
dma_addr_t          rx_dma_reg_pa;  /* RX DMA region physical address. */
//struct dma_stream   tx_dma_stream;  /* To device. */
//struct dma_stream   rx_dma_stream;  /* From device. */

/* MMIO */
/* Like the DMA variables, probe() and remove() both use bar0_base_va, so need
 * to make global. FIXME: explore more elegant way of doing this. */
void __iomem        *bar0_base_va;
unsigned int        bar0_size;

/* Ading the NAPI interruption structure
 * to the code so the driver can handle
 * the high velocity transmission and 
 * packages.
 */	
static struct napi_struct Eth_napi_struct;


/* Function to print the status. */
void printline(unsigned char *data, int n) {
	char line[256], entry[16];
	int j;
	strcpy(line,"");
	for (j=0; j < n; j++){
		sprintf(entry, " %2x", data[j]);
		strcat(line, entry);
	}
	pr_info("%s\n", line);
}

/* Called to fill @buf when user reads our file in /proc. */
int read_proc(struct file *filp,char *buf,size_t count,loff_t *offp ) {
	char *data;
	data=PDE_DATA(file_inode(filp));

	if(!(data)){
		printk(KERN_INFO "Null data");
		return 0;
	}

	if(count>temp){
		count=temp;
	}

	temp=temp-count;

	copy_to_user(buf,data, count);
	
	if(count==0)
		temp=len;

	return count;
}



struct file_operations proc_fops = {
	read:   read_proc
};

/*
 * Eth_do_ioctl allows the driver to have Input/Output commands.
 * Missing implementation
 */
static int Eth_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd) {
	pr_info("Eth_do_ioctl(%s)\n", dev->name);
	return -1;
}

static struct net_device_stats *Eth_get_stats(struct net_device *dev)
{
	pr_info("Eth_get_stats(%s)\n", dev->name);
	return stats;
}

/*
 * This is where ifconfig comes down and tells us who we are, etc.
 * We can just ignore this.
 */
static int Eth_config(struct net_device *dev, struct ifmap *map)
{
	pr_info("Eth_config(%s)\n", dev->name);
	if (dev->flags & IFF_UP) {
		return -EBUSY;
	}
	return 0;
}

/*
 * This will allow us to change the device mtu size.
 */
static int Eth_change_mtu(struct net_device *dev, int new_mtu)
{
	unsigned long flags = 0;
	struct eth_priv *priv = netdev_priv(dev);
	spinlock_t *lock = &priv->lock;

	pr_info("Eth_change_mtu(%s)\n", dev->name);

	/* Check ranges */
	if ((new_mtu < 68) || (new_mtu > 10000))	//Remember to see at the hardware documentation the right especification
		return -EINVAL;

	/*
	 * Do anything you need, and accept the value
	 */
	spin_unlock_irqrestore(lock, flags);
	dev->mtu = new_mtu;
	spin_unlock_irqrestore(lock, flags);
	printk (KERN_INFO "New mtu: (%d)", dev->mtu);
	return 0; /* Sucess */
}

/*
 * The open function is called on every time we use the "ifconfig" command
 * and it's allways opened by the kernel and then assign an address to it
 * before the interface can carry packets.
 */
static int Eth_open(struct net_device *dev)
{
	pr_info("Hit: Eth_open(%s)\n", dev->name);

	/* start up the transmission queue */

	netif_start_queue(dev);
	return 0;
}

/*
 * Opposit of Eth_open function
 */
static int Eth_close(struct net_device *dev)
{
	pr_info("Hit: Eth_close(%s)\n", dev->name);

	/* shutdown the transmission queue */

	netif_stop_queue(dev);
	return 0;
}

/*
 * Structure that holds all the options supported by the driver.
 */
static struct net_device_ops ndo = {
	.ndo_open 		= Eth_open,
	.ndo_stop 		= Eth_close,
	.ndo_start_xmit = Eth_start_xmit,
	.ndo_do_ioctl 	= Eth_do_ioctl,
	.ndo_get_stats 	= Eth_get_stats,
	.ndo_set_config = Eth_config,
	.ndo_change_mtu = Eth_change_mtu,
	.ndo_tx_timeout = Eth_tx_timeout,
};

int ng_header(struct sk_buff *skb, struct net_device *dev,
                unsigned short type, const void *daddr, const void *saddr,
                unsigned len) {

	struct ethhdr *eth = (struct ethhdr *)skb_push(skb,ETH_HLEN);

	eth->h_proto = htons(type);
	memcpy(eth->h_source, saddr ? saddr : dev->dev_addr, dev->addr_len);
	memcpy(eth->h_dest, daddr ? daddr : dev->dev_addr, dev->addr_len);
	eth->h_dest[ETH_ALEN-1] ^= 0x01; /* dest is us xor 1 */
	return (dev->hard_header_len);
}


static void Eth_setup(struct net_device *dev)
{
	//char mac_addr[mac_addr_len+1];
	pr_info("Eth_setup(%s)\n", dev->name);

	/* Fill in the MAC address with a phoney */

	//for (j = 0; j < ETH_ALEN; ++j) {
	//	dev->dev_addr[j] = (char)j;
	//}

	/* request_region(), request_irq(),...
	 *
	 * Assign the hardware address of the board: use "\oSNULx", where
	 * x is  0 or 1. The first byte is '\0' to avoid being a multicast
	 * address (the first byte of multicast addrs is odd).
	 */

    	//mac_addr_len = device->addr_len;

        //memset(mac_addr, 0, mac_addr_len+1);
	//snprintf(&mac_addr, mac_addr_len, "NF%d", 0);
        //memcpy(device->dev_addr, mac_addr, mac_addr_len);



	//memcpy(dev->dev_addr, "\0SNUL0", ETH_LEN);
	//if (dev == device)
	//	dev->dev_addr[ETH_LEN-1]++; /* \OSNUL1 */

	ether_setup(dev);

	dev->netdev_ops = &ndo;
	dev->flags |= IFF_NOARP;
	stats = &dev->stats;

	/*
	 * Just for laughs, let's claim that we've seen 50 collisions.
	 */
	stats->collisions = 50;
}

void Eth_netdev_init(struct net_device *netdev) {
	printk(KERN_DEBUG "Eth_netdev_init(): Initializing Eth_netdev.\n");
	ether_setup(netdev);
	netdev->netdev_ops  = &ndo;;
    netdev->watchdog_timeo = 5 * HZ;
}

static int __init Eth_driver_init(void) {
	int 		result;
	int 		i;
	int 		err;
	uint32_t 	mac_addr_len;
	
	pr_info("Loading Ethernet network module:....");

	//priv = 0; // TODO: GET priv

	/* Allocating the net devices. */
	for (i = 0; i <NUM_NETDEVS; i++) {
		device[i] = alloc_netdev(0, "Eth%d", Eth_setup);
		if(device[i] == NULL) {
			printk(KERN_ERR "Eth_Driver: ERROR: Eth_driver_init(): failed to allocate net_device %d... unloading driver\n", i);

			for (i = i-1; i >= 0; i--)
				free_netdev(device[i]);

			//pci_unregister_driver(&eth_pci_driver);
			return -ENOMEM;
		}
	}

	printk(KERN_DEBUG "Eth_driver_init(): Succeeded allocating netdevs...\n");

	mac_addr_len = device[0]->addr_len;
	char mac_addr[mac_addr_len+1];

	/* Set network interface MAC addresses. */
    for(i = 0; i < NUM_NETDEVS; i++) {
        memset(mac_addr, 0, mac_addr_len+1);
        snprintf(&mac_addr[1], mac_addr_len, "NG%d", i);
        memcpy(device[i]->dev_addr, mac_addr, mac_addr_len);
    }

	printk(KERN_INFO "Eth_driver_init(): Succeeded in setting netdev MAC addresses...\n\n");
	
	/* Add NAPI structure to the device. */
    /* Since we have NUM_NETDEVS net_devices, we just use the 1st one for implementing polling. */
    netif_napi_add(device[0], &Eth_napi_struct, Eth_napi_struct_poll, RX_POLL_WEIGHT);

    printk(KERN_INFO "Eth_driver_init(): Succeeded in adding NAPI structure...\n");

    /* Register the network interfaces. */
    for (i = 0; i < NUM_NETDEVS; i++) {
    	err = register_netdev(device[i]);

    	/* Verification... */
		if(err != 0) {
			printk(KERN_ERR "Eth_driver: ERROR: Eth_driver_init(): failed to register net_device %d... unloading driver\n", i);
			
			for (i = i-1; i >= 0; i--)
    			unregister_netdev(device[i]);

    		netif_napi_del(&Eth_napi_struct);

    		for (i = 0; i < NUM_NETDEVS; i++)
    			free_netdev(device[i]);

    		//pci_unregister_driver(&eth_pci_driver);
    		return err;
		}
    }

    printk(KERN_DEBUG "Eth_driver_init(): Succeeded in registering netdevs...\n");

     /* FIXME: Do we need to check for error on create_proc_read_entry? */
	//create_proc_read_entry("driver/Eth_driver", 0, NULL, read_proc, NULL);		THIS IS OBSOLETE NOW!
	proc_create_data("driver/Eth_driver",0,NULL,&proc_fops,NULL);


    /* Enabling NAPI. */
    napi_enable(&Eth_napi_struct);


	return 0;
}

static void __exit Eth_driver_exit(void) {
	int err;
	int i;

	/* Disable NAPI */
	napi_disable(&Eth_napi_struct);

	/* Strop the polling timer for receiving packets. */
	/* TODO */

	// err = genl_unregister_family(&eth_genl_family);
	// if(err != 0)
	// 	printk (KERN_ERR "Eth_Driver: ERROR: Eth_driver_exit(): failed to unregister GENL family\n");

	for(i = 0; i < NUM_NETDEVS; i++)
		unregister_netdev(device[i]);

	printk(KERN_INFO "Device Unregistered...");

	netif_napi_del(&Eth_napi_struct);

	for(i = 0; i < NUM_NETDEVS; i++)
		free_netdev(device[i]);

	printk(KERN_INFO "Device's memory fully cleaned...");

	return;
}

void Eth_teardown_pool (struct net_device* dev) {
	struct eth_priv *priv = netdev_priv(dev);
    struct eth_packet *pkt;

    while ((pkt = priv->ppool)) {
		priv->ppool = pkt->next;
        kfree (pkt);
       /* FIXME - in-flight packets ? */
    }
}

/* Structure to manage the pool buffer */
struct eth_packet *Eth_get_tx_buffer(struct net_device *dev) {
	struct eth_priv *priv = netdev_priv(dev);
	unsigned long flags = 0;
	spinlock_t *lock = &priv->lock;
	struct eth_packet *pkt;

	spin_lock_irqsave(lock, flags);
	pkt = priv->ppool;
	priv->ppool = pkt->next;
	if (priv->ppool == NULL) {
		printk(KERN_INFO "Pool empty\n");
		netif_stop_queue(dev);
	}
	spin_unlock_irqrestore(lock, flags);
	return pkt;
} 

/*
* Transmit a packet (low level interface)
*/


int Eth_start_xmit(struct sk_buff *skb, struct net_device *dev) {
	int len;
	char *data, shortpkt[ETH_ZLEN];
	struct eth_priv *priv = netdev_priv(dev);

	data = skb->data;
	len = skb->len;
	if (len < ETH_ZLEN) {
		memset(shortpkt, 0 , ETH_ZLEN);
		memcpy(shortpkt, skb->data, skb->len);
		len = ETH_ZLEN;
		data = shortpkt;
	}
	dev->trans_start = jiffies; /* save the timestamp */

	/* Remember the skb, so we can free it at interrupt time */
	priv->skb = skb;

	/* actual deliver of data is device-specific, and not shown here */
	//nf10_hw_tx(data, len, dev);

	return 0; /* Our simple device can not fail */
}

struct eth_packet *eth_dequeue_buf(struct net_device *dev) {

	struct eth_priv *priv = netdev_priv(dev);
	struct eth_packet *pkt;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	pkt = priv->rx_queue;
	if (pkt != NULL)
		priv->rx_queue = pkt->next;
	spin_unlock_irqrestore(&priv->lock, flags);
	return pkt;
}

 static int Eth_napi_struct_poll(struct napi_struct *napi, int budget) {
	int npackets = 0;
	struct sk_buff *skb;
	struct eth_priv *priv = container_of(napi, struct eth_priv, napi);
	struct net_device *dev = priv->dev;
	struct eth_packet *pkt;

	while (npackets < budget && priv->rx_queue) {
		pkt = eth_dequeue_buf(dev);
		skb = dev_alloc_skb(pkt->datalen + 2);
		if (!skb) {
			if (printk_ratelimit())
				printk(KERN_NOTICE "Eth: packet dropped\n");
			priv->stats.rx_dropped++;
			eth_release_buffer(pkt);
			continue;
		}
		skb_reserve(skb, 2);  //align IP on 16B boundary 
		memcpy(skb_put(skb, pkt->datalen), pkt->data, pkt->datalen);
		skb->dev = dev;
		skb->protocol = eth_type_trans(skb, dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY; // don't check it
		netif_receive_skb(skb);
		// Maintain stats
		npackets++;
		priv->stats.rx_packets++;
		priv->stats.rx_bytes += pkt->datalen;
		eth_release_buffer(pkt);
	}
	//If we processed all packets, we're done; tell the kernel and re-enable interruptions
	if (npackets < budget) {
		napi_complete(napi);
		Eth_rx_ints(dev, 1);
	}
	return npackets;
}


void Eth_rx(struct net_device *dev, struct eth_packet *pkt) {
	struct sk_buff *skb;
	struct eth_priv * priv = netdev_priv(dev);

	/*
	 * The packet has been retrieved from the transmission
	 * medium. Build ans skb around it, so upper layers can handle it
	 */
	skb = dev_alloc_skb(pkt->datalen + 2); //alocating the buffer for the packet

	/* Checking if the packet allocation process went wrong */
	if (!skb) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "Eth rx: low on mem - packet dropped\n");
		priv->stats.rx_dropped++;
		goto out;
	}
	memcpy(skb_put(skb, pkt->datalen), pkt->data, pkt->datalen);	//No problems, so we can copy the packet to the buffer.

	/* Write metadata, and then pass to the receive level */
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;	/* don't check it */
	priv->stats.rx_packets++;
	priv->stats.rx_bytes += pkt->datalen;
	netif_rx(skb);
	out:
		return;
}

void eth_release_buffer(struct eth_packet *pkt) {
	unsigned long flags;
	struct eth_priv *priv = netdev_priv(pkt->dev);

	spin_lock_irqsave(&priv->lock, flags);
	pkt->next = priv->ppool;
	priv->ppool = pkt;
	spin_unlock_irqrestore(&priv->lock, flags);
	if(netif_queue_stopped(pkt->dev) && pkt->next == NULL)
		netif_wake_queue(pkt->dev);
}

static void Eth_rx_ints(struct net_device *dev, int enable) {
	struct eth_priv *priv = netdev_priv(dev);
	priv->rx_int_enabled = enable;
}

static irqreturn_t Eth_interruption(int irq, void *dev_id, struct pt_regs *regs) {
	int statusword;
	struct eth_priv *priv;
	struct eth_packet *pkt = NULL;

	/*
	 * As usual, check the "device" pointer to be sure it is
	 * really interrupting.
	 * Then assign "struct device *dev".
	 */
	 struct net_device *dev = (struct net_device *)dev_id;
	 /* ... and check with hw if it's really ours */

	 /* paranoid */
	 if(!dev)
	 	return IRQ_HANDLED;

	/* Lock the device */
	priv = netdev_priv(dev);
	spin_lock(&priv->lock);

	/* retrieve statusword: real netdevices use I/O instructions */
	statusword = priv->status;
	priv->status = 0;
	if(statusword & ETH_RX_INTR) {
		/* This will disinable any further "packet available" 
		 * interrupts and tells networking subsystem to poll 
		 * the driver shortly to pick up all available packets.
		 */
		Eth_rx_ints(dev, 0);
		if (napi_schedule_prep(&priv->napi)) {
			/* Disinable reception interrupts */
       			__napi_schedule(&priv->napi);
		}
		
	}
	if (statusword & ETH_TX_INTR) {
		/* a transmission is over: free the skb */
		priv->stats.tx_packets++;
		priv->stats.tx_bytes += priv->tx_packetlen;
		dev_kfree_skb(priv->skb);
	}

	/* Unlock the device and we are done */
	spin_unlock(&priv->lock);
	if (pkt)
		eth_release_buffer(pkt); /* Do this outside the lock! */
		return IRQ_HANDLED;
}

void Eth_tx_timeout(struct net_device *dev) {
	struct eth_priv *priv = netdev_priv(dev);

	//PDEBUG ("Transmit timeout at %ld, latency %ls'n", jiffies,
	//			jiffies - dev->trans_start);
	printk(KERN_DEBUG "Transmit timeout at %ld, latency %ld \n", jiffies, 
				jiffies - dev->trans_start);
	/* Simulate a transmission interrupt to get things moving */
	priv->status = ETH_TX_INTR;
	Eth_interruption(0, dev, NULL);
	priv->stats.tx_errors++;
	netif_wake_queue(dev);
	return;
}


module_init(Eth_driver_init);
module_exit(Eth_driver_exit);

MODULE_AUTHOR("Pavel Teixeira");
MODULE_DESCRIPTION("Ethernet driver");
MODULE_LICENSE("GPL v2");

