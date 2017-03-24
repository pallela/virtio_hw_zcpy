#include"virtio.h"
#include <pcap.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/eventfd.h>
#include<semaphore.h>
#include"dma_rxtx.h"

extern uint64_t guestphyddr_to_hostphysaddr(uint64_t gpaddr);

static bpf_u_int32 net;		/* Our IP */
pcap_t *pcap_init(char *iname)
{
	pcap_t *handle;			/* Session handle */
	char *dev;			/* The device to sniff on */
	char errbuf[PCAP_ERRBUF_SIZE];	/* Error string */
	bpf_u_int32 mask;		/* Our netmask */

	/* Define the device */

	printf("vvdn debug :opening pcap on interface %s\n",iname);
	dev = iname;

	/* Find the properties for the device */
	if (pcap_lookupnet(dev, &net, &mask, errbuf) == -1) {
		fprintf(stderr, "Couldn't get netmask for device %s: %s\n", dev, errbuf);
		net = 0;
		mask = 0;
	}
	/* Open the session in promiscuous mode */
	handle = pcap_open_live(dev, BUFSIZ, 1, 1, errbuf);
	if (handle == NULL) {
		fprintf(stderr, "Couldn't open device %s: %s\n", dev, errbuf);
		return;
	}

	return handle;
}

extern sem_t tx_start_wait_sem,rx_start_wait_sem;
extern sem_t tx_clean_wait_sem,rx_clean_wait_sem;

extern volatile struct vring_desc *rx_desc_base; 
extern volatile struct vring_used  *rx_used; 
extern volatile struct vring_avail *rx_avail;
extern volatile int rxirqfd;
extern int rx_desc_count;
extern volatile connected_to_guest;
extern int vhost_hlen;


uint64_t guestphyddr_to_vhostvadd(uint64_t gpaddr);

unsigned char mac_address[6] = {0xb8,0x2a,0x72,0xc4,0x26,0x45};
unsigned char broadcast_mac_address[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
void *pcap_rx_thread(void *arg)
{
//	struct bpf_program fp;		/* The compiled filter */
	//char filter_exp[] = "ether dst 00:00:00:00:00:01 ";	/* The filter expression */
	//char filter_exp[] = "ether dst b8:2a:72:c4:26:45 or ether dst ff:ff:ff:ff:ff:ff  or  arp";	/* The filter expression */
	//char filter_exp[] = "ether dst b8:2a:72:c4:26:45 or  arp";	/* The filter expression */
//	char filter_exp[] = "ether dst 00:00:00:00:00:01 or  arp";	/* The filter expression */
	//char filter_exp[] = "";	/* The filter expression */
//	struct pcap_pkthdr header;	/* The header that pcap gives us */
	const u_char *packet;		/* The actual packet */
//	pcap_t *handle;
	void  *tmp;
//	uint16_t *nbuffs;
	int rx_len;
	uint16_t rx_desc_num = 0,rx_header_desc_num = 0,rx_avail_ring_no = 0,rx_used_ring_no = 0;
	unsigned char  *packet_addr;
	uint32_t packet_len;
	uint16_t avail_idx,used_idx;
	struct virtio_net_hdr_mrg_rxbuf *tmpheader;
	int rx_cleanup_required;
	uint64_t rxbuff_paddr;


#if 0
	handle = (pcap_t *) arg;

	printf("starting rx thread with pcap handle : %p\n",handle);

	/* Compile and apply the filter */
	if (pcap_compile(handle, &fp, filter_exp, 0, net) == -1) {
		fprintf(stderr, "Couldn't parse filter %s: %s\n", filter_exp, pcap_geterr(handle));
		return;
	}

	if (pcap_setfilter(handle, &fp) == -1) {
		fprintf(stderr, "Couldn't install filter %s: %s\n", filter_exp, pcap_geterr(handle));
		return;
	}
#endif

	printf("starting rx thread\n");
	while(1) {

		/* Grab a packet */
		//packet = pcap_next(handle, &header);
		if(connected_to_guest) {
			avail_idx = rx_avail->idx;
			used_idx = rx_used->idx;

			if(avail_idx != used_idx) {

				//printf("before rx_len : %d and addr : %p\n",rx_len,packet);
				if( VHOST_SUPPORTED_FEATURES &( 1ULL << VIRTIO_NET_F_MRG_RXBUF)) {
					rx_desc_num = rx_avail->ring[rx_avail_ring_no];
					tmp = (void *) rx_desc_base[rx_desc_num].addr;
					rxbuff_paddr = guestphyddr_to_hostphysaddr((uint64_t)tmp);
					rxbuff_paddr += (uint64_t)vhost_hlen;
					rx_len = rx_desc_base[rx_desc_num].len - vhost_hlen; 
					//printf("func : %s line : %u rxbuff_paddr : %llx and max recv len : %d\n",__func__,__LINE__,rxbuff_paddr,rx_len);
					packet = dma_rx((void *)rxbuff_paddr,&rx_len);
					//printf("func : %s line : %u rx pkt len  : %d\n",__func__,__LINE__,rx_len);

				}
				else {
					rx_desc_num = rx_avail->ring[rx_avail_ring_no];
					rx_desc_num = rx_desc_base[rx_desc_num].next;
					tmp = (void *) rx_desc_base[rx_desc_num].addr;
					rxbuff_paddr = guestphyddr_to_hostphysaddr((uint64_t)tmp);
					//printf("rxbuff_paddr  : %llx (actual: %llx)\n",rxbuff_paddr, guestphyddr_to_hostphysaddr((uint64_t)tmp));
					rx_len = rx_desc_base[rx_desc_num].len; 
					//printf("func : %s line : %u rxbuff_paddr : %llx and max recv len : %d\n",__func__,__LINE__,rxbuff_paddr,rx_len);
					packet = dma_rx((void *)rxbuff_paddr,&rx_len);
					//printf("func : %s line : %u rx pkt len  : %d\n",__func__,__LINE__,rx_len);
				}
				//packet = dma_rx((void *)rxbuff_paddr,&rx_len);
				//printf("later rx_len : %d and addr : %p\n",rx_len,packet);
				//packet_addr = (unsigned char *) packet;
			}
			else {
				usleep(1);
				continue;
			}
		}
		/* Print its length */


#if 1
		//if(header.len > 0 && connected_to_guest) {
		if(rx_len > 0 && connected_to_guest) {

			//printf("received a packet with length of [%d]\n", header.len);

			if(packet ) {


			
				//printf("vhost rx packet at address : %p len : %d\n",(void *) packet,header.len);
				//printf("vhost rx packet len : %d\n",header.len);

				avail_idx = rx_avail->idx;
				used_idx = rx_used->idx;

				if((avail_idx - used_idx) == 0) {
					printf("Dropping packet\n");
					continue;
				}
				
				//printf("avail_idx : %d and used_idx : %d diff  : %d\n",avail_idx,used_idx,avail_idx-used_idx);


				if( VHOST_SUPPORTED_FEATURES &( 1ULL << VIRTIO_NET_F_MRG_RXBUF) ) {
					rx_desc_num = rx_avail->ring[rx_avail_ring_no];
					rx_header_desc_num = rx_desc_num;
					tmp = (void *)guestphyddr_to_vhostvadd(rx_desc_base[rx_desc_num].addr);

					if(rx_desc_base[rx_desc_num].len < (vhost_hlen + rx_len)) {
						//printf("receive desc buff len : %d and packet len : %d ,so dropping packet\n"
						//		,rx_desc_base[rx_desc_num].len,header.len);
						printf("receive desc buff len : %d and packet len : %d ,so dropping packet\n"
								,rx_desc_base[rx_desc_num].len,rx_len);
						continue;
					}
					packet_len = rx_len;
					memset(tmp,0,vhost_hlen);
					//memcpy(tmp+vhost_hlen,packet_addr,packet_len);
					//printf("recv packet : %d bytes\n",packet_len);
	
				}
				else {
					rx_desc_num = rx_avail->ring[rx_avail_ring_no];
					rx_header_desc_num = rx_desc_num;
					tmp = (void *)guestphyddr_to_vhostvadd(rx_desc_base[rx_desc_num].addr);
					memset(tmp,0,vhost_hlen);
					rx_desc_num = rx_desc_base[rx_desc_num].next;

					if(rx_desc_base[rx_desc_num].len < rx_len) {
						//printf("receive desc buff len : %d and packet len : %d ,so dropping packet\n"
						//		,rx_desc_base[rx_desc_num].len,header.len);
						printf("receive desc buff len : %d and packet len : %d ,so dropping packet\n"
								,rx_desc_base[rx_desc_num].len,rx_len);
						continue;
					}
					//printf("receive desc buff len : %d and packet len : %d\n",rx_desc_base[rx_desc_num].len,header.len);

					//tmp = (void *)guestphyddr_to_vhostvadd(rx_desc_base[rx_desc_num].addr);
					//printf("tmp ( packet data ): %p \n",tmp);
					//packet_len = header.len;
					packet_len = rx_len;
					//memcpy(tmp,packet_addr,packet_len);
					//printf("packet copied to VM memory\n");
					//rx_len = 0;
				}

				rx_avail_ring_no = (rx_avail_ring_no + 1)%rx_desc_count;
				wmb();

				rx_used->ring[rx_used_ring_no].id = rx_header_desc_num;
				rx_used->ring[rx_used_ring_no].len = vhost_hlen + packet_len;
				rx_used_ring_no = (rx_used_ring_no +1)%rx_desc_count;
				wmb();
				rx_used->idx++;
				wmb();
				eventfd_write(rxirqfd, (eventfd_t)1);
				wmb();
				//printf("packets received : %d\n",rx_used->idx);
			}
			else {
				//printf("packet address is NULL\n");
			}
		}
		else if(!connected_to_guest) {
			rx_avail_ring_no = 0;
			rx_used_ring_no = 0;
			if(rx_cleanup_required) {
				rx_cleanup_required = 0;
				printf("rx thread , cleanup done\n");
				sem_post(&rx_clean_wait_sem);
			}
			printf("rx  thread , waiting for connection\n");
			sem_wait(&rx_start_wait_sem);
			rx_cleanup_required = 1;
			printf("rx thread , starting processing now\n");
			//usleep(10000);
		}
#endif
	}
	/* And close the session */
}

char pcap_tx_err_str[1024];
void pcap_tx(pcap_t *handle, void *packet,int size)
{
	int ret;
	//printf("to tx : %d  bytes\n",size);
	ret = pcap_inject(handle,packet,size);

	if(ret == -1) {
		printf("tx packet failed : %s\n",pcap_geterr(handle));
	}
	
}
