/* **************** NG10.c **************** */
/*
 * The code herein is: Copyright Pavel Teixeira, 2014
 *
 * This Copyright is retained for the purpose of protecting free
 * redistribution of source.
 *
 *     URL:    http://www.coopj.com
 *     email:  gal.prime.kr@gmail.com
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
 * Building a Transmitting Network Driver
 *
 * Extend your stub network device driver to include a transmission
 * function, which means supplying a method for
 * ndo_start_xmit().
 *
 * While you are at it, you may want to add other entry points to see
 * how you may exercise them.
 *
 * Once again, you should be able to exercise it with:
 *
 *   insmod lab_network.ko
 *   ifconfig mynet0 up 192.168.3.197
 *   ping -I mynet0 localhost
 *       or
 *   ping -bI mynet0 192.168.3
 *
 * Make sure your chosen address is not being used by anything else.
 *
 @*/
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>

/* Length definitions */
#define mac_addr_len                    6

static struct net_device *device;
static struct net_device_stats *stats;
void nf10_teardown_pool (struct net_device* dev);
__be16 eth_type_trans(struct sk_buff *skb, struct net_device *dev);
int nf10_tx(struct sk_buff *skb, struct net_device *dev);

struct nf10_priv {
	struct net_device_stats stats;
	int status;
	struct nf10_packet* ppool;
	struct nf10_packet* rx_queue; /* List of incoming packets */
	int rx_int_enabled;
	int tx_packetlen;
	u8* tx_packetdata;
	struct sk_buff* skb;
	spinlock_t lock;
};

struct nf10_packet {
	struct nf10_packet* next;
	struct net_device* dev;
	int datalen;
	u8 data[1500];
};

void printline(unsigned char *data, int n){
	char line[256], entry[16];
	int j;
	strcpy(line,"");
	for (j=0; j < n; j++){
		sprintf(entry, " %2x", data[j]);
		strcat(line, entry);
	}
	pr_info("%s\n", line);
}

/*
* Deal with a transmit timeout.
*/

//void nf10_tx_timeout (struct net_device *dev) {
//	struct nf10_priv *priv = netdev_priv(dev);
//	PDEBUG("Transmit timeout at %ld, latency %ld\n", jiffies,
//		jiffies - dev->trans_start);
	/* Simulates a transmission interrupt to get things moving */
//	priv->status = NF10_TX_INTR;
//	nf10_interrupt(0, dev, NULL);
//	priv->stats.tx_errors++;
//	netif_wake_queue(dev);
//	return;
//}

static int my_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	pr_info("my_do_ioctl(%s)\n", dev->name);
	return -1;
}

static struct net_device_stats *my_get_stats(struct net_device *dev)
{
	pr_info("my_get_stats(%s)\n", dev->name);
	return stats;
}

/*
 * This is where ifconfig comes down and tells us who we are, etc.
 * We can just ignore this.
 */
static int my_config(struct net_device *dev, struct ifmap *map)
{
	pr_info("my_config(%s)\n", dev->name);
	if (dev->flags & IFF_UP) {
		return -EBUSY;
	}
	return 0;
}

static int my_change_mtu(struct net_device *dev, int new_mtu)
{
	unsigned long flags = 0;
	struct nf10_priv *priv = netdev_priv(dev);
	spinlock_t *lock = &priv->lock;

	pr_info("my_change_mtu(%s)\n", dev->name);

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

static int my_open(struct net_device *dev)
{
	pr_info("Hit: my_open(%s)\n", dev->name);

	/* start up the transmission queue */

	netif_start_queue(dev);
	return 0;
}

static int my_close(struct net_device *dev)
{
	pr_info("Hit: my_close(%s)\n", dev->name);

	/* shutdown the transmission queue */

	netif_stop_queue(dev);
	return 0;
}

static struct net_device_ops ndo = {
	.ndo_open = my_open,
	.ndo_stop = my_close,
	.ndo_start_xmit = nf10_tx,
	.ndo_do_ioctl = my_do_ioctl,
	.ndo_get_stats = my_get_stats,
	.ndo_set_config = my_config,
	.ndo_change_mtu = my_change_mtu,
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


static void my_setup(struct net_device *dev)
{
	//char mac_addr[mac_addr_len+1];
	pr_info("my_setup(%s)\n", dev->name);

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

static int __init my_init(void)
{
	int result;
	pr_info("Loading NG network module:....");

	/* Allocating each net device. */
	device = alloc_netdev(0, "nf%d", my_setup);

	if ((result = register_netdev(device))) {
		printk(KERN_EMERG "NG: error %i registering  device \"%s\"\n", result, device->name);
		free_netdev(device);
		return -1;
	}
	printk(KERN_INFO "Succeeded in loading %s!\n\n", dev_name(&device->dev));
	return 0;
}

static void __exit my_exit(void)
{
	pr_info("Unloading transmitting network module\n\n");
	if (device) {
		unregister_netdev(device);
		printk(KERN_INFO "Device Unregistered...");
		nf10_teardown_pool(device);
		free_netdev(device);
		printk(KERN_INFO "Device's memory fully cleaned...");
	}
	return;
}

void nf10_teardown_pool (struct net_device* dev) {
	struct nf10_priv *priv = netdev_priv(dev);
        struct nf10_packet *pkt;

        while ((pkt = priv->ppool)) {
		priv->ppool = pkt->next;
                kfree (pkt);
                /* FIXME - in-flight packets ? */
         }
}

/*
* Transmit a packet (low level interface)
*/
static void snull_hw_tx(char *buf, int len, struct net_device *dev) {
 	/*
  	 * This function deals with hw details. This interface loops
	 * back the packet to the other snull interface (if any).
	 * In other words, this function implements the snull behaviour,
	 * while all other procedures are rather device-independent
	 */
	struct iphdr *ih;
	struct net_device *dest;
	struct snull_priv *priv;
	u32 *saddr, *daddr;
	struct snull_packet *tx_buffer;

	/* I am paranoid. Ain't I? */
	if (len < sizeof(struct ethhdr) + sizeof(struct iphdr)) {
		printk("snull: Hmm... packet too short (%i octets)\n", len);
		return;
	}

	if (0) { /* enable this conditional to look at the data */
	int i;
	PDEBUG("len is %i\n" KERN_DEBUG "data:",len);
	for (i=14 ; i<len; i++)
		printk(" %02x",buf[i]&0xff);
	printk("\n");
	}
	/*
	 * Ethhdr is 14 bytes, but the kernel arranges for iphdr
	 * to be aligned (i.e., ethhdr is unaligned)
	 */
	ih = (struct iphdr *)(buf+sizeof(struct ethhdr));
	saddr = &ih->saddr;
	daddr = &ih->daddr;

	((u8 *)saddr)[2] ^= 1; /* change the third octet (class C) */
	((u8 *)daddr)[2] ^= 1;

	ih->check = 0; /* and rebuild the checksum (ip needs it) */
	ih->check = ip_fast_csum((unsigned char *)ih,ih->ihl);

	if (dev == snull_devs[0])
	PDEBUGG("%08x:%05i --> %08x:%05i\n",
	ntohl(ih->saddr),ntohs(((struct tcphdr *)(ih+1))->source),
	ntohl(ih->daddr),ntohs(((struct tcphdr *)(ih+1))->dest));
	else
	PDEBUGG("%08x:%05i <-- %08x:%05i\n",
	ntohl(ih->daddr),ntohs(((struct tcphdr *)(ih+1))->dest),
	ntohl(ih->saddr),ntohs(((struct tcphdr *)(ih+1))->source));

	/*
	 * Ok, now the packet is ready for transmission: first simulate a
	 * receive interrupt on the twin device, then a
	 * transmission-done on the transmitting device
	 */
	dest = snull_devs[dev == snull_devs[0] ? 1 : 0];
	priv = netdev_priv(dest);
	tx_buffer = snull_get_tx_buffer(dev);
	tx_buffer->datalen = len;
	memcpy(tx_buffer->data, buf, len);
	snull_enqueue_buf(dest, tx_buffer);
	if (priv->rx_int_enabled) {
		priv->status |= SNULL_RX_INTR;
		snull_interrupt(0, dest, NULL);
	}

	priv = netdev_priv(dev);
	priv->tx_packetlen = len;
	priv->tx_packetdata = buf;
	priv->status |= SNULL_TX_INTR;
	if (lockup && ((priv->stats.tx_packets + 1) % lockup) == 0) {
         	/* Simulate a dropped transmit interrupt */
		netif_stop_queue(dev);
		PDEBUG("Simulate lockup at %ld, txp %ld\n", jiffies,
		(unsigned long) priv->stats.tx_packets);
	}
	else
		snull_interrupt(0, dev, NULL);
}

int nf10_tx(struct sk_buff *skb, struct net_device *dev) {
	int len;
	char *data, shortpkt[ETH_ZLEN];
	struct nf10_priv *priv = netdev_priv(dev);

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

void nf10_rx(struct net_device *dev, struct nf10_packet *pkt) {
	struct sk_buff *skb;
	struct nf10_priv * priv = netdev_priv(dev);

	/*
	 * The packet has been retrieved from the transmission
	 * medium. Build ans skb around it, so upper layers can handle it
	 */
	skb = dev_alloc_skb(pkt->datalen + 2); //alocating the buffer for the packet

	/* Checking if the packet allocation process went wrong */
	if (!skb) {
		if (printk_ratelimit())
			printk(KERN_NOTICE "NF10 rx: low on mem - packet dropped\n");
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


/**
150  * eth_type_trans - determine the packet's protocol ID.
151  * @skb: received socket data
152  * @dev: receiving network device
153  *
154  * The rule here is that we
155  * assume 802.3 if the type field is short enough to be a length.
156  * This is normal practice and works for any 'now in use' protocol.
157  */
__be16 eth_type_trans(struct sk_buff *skb, struct net_device *dev) {
	struct ethhdr* eth;
	unsigned short _service_access_point;
	const unsigned short *sap;

	skb->dev = dev;
	skb_reset_mac_header(skb);
	skb_pull_inline(skb, ETH_HLEN);
	eth = eth_hdr(skb);

	 if (unlikely(is_multicast_ether_addr(eth->h_dest))) {
         	if (ether_addr_equal_64bits(eth->h_dest, dev->broadcast))
			skb->pkt_type = PACKET_BROADCAST;
		else
			skb->pkt_type = PACKET_MULTICAST;
	}

	else if (unlikely(!ether_addr_equal_64bits(eth->h_dest, dev->dev_addr)))
        	skb->pkt_type = PACKET_OTHERHOST;

	if (unlikely(netdev_uses_dsa_tags(dev)))
		return htons(ETH_P_DSA);
	if (unlikely(netdev_uses_trailer_tags(dev)))
		return htons(ETH_P_TRAILER);

	if (likely(ntohs(eth->h_proto) >= 1536))
		return eth->h_proto;

	sap = skb_header_pointer(skb, 0, sizeof(*sap), &_service_access_point);
	if (sap && *sap == 0xFFFF)
		return htons(ETH_P_802_3);

	return htons(ETH_P_802_3);
}

module_init(my_init);
module_exit(my_exit);

MODULE_AUTHOR("Pavel Teixeira");
MODULE_DESCRIPTION("NG Ethernet driver");
MODULE_LICENSE("GPL v2");
