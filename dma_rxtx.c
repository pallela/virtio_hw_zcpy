#include <stdio.h>
#include <stdlib.h>
#include "xdma-core.h"

#include<string.h>
#include<fcntl.h>
#include<sys/mman.h>
#include<stdint.h>

#define PAGE_SIZE 4096
#define DESC_CNT 1
#define LENGTH 4*1024
#define NUM_BUFFERS 64
#define LOGGING 0

extern volatile connected_to_guest;
int channel;
struct mydata *data;
unsigned int * bar_base_user;
unsigned int  *rd_data, *wr_data;
struct mydata
{
        //struct device dev;
        void *tx_queue;
        uint64_t tx_queue_dma_addr;
        void *rx_queue;
        uint64_t rx_queue_dma_addr;
        void *rx_result;
	uint64_t rx_result_dma_addr;
	void *bar_virt;
	uint64_t bar_base_addr_phy;
        void *coherent_mem_tx[NUM_BUFFERS];
        uint64_t coherent_mem_tx_dma_addr[NUM_BUFFERS];
        void *coherent_mem_rx[NUM_BUFFERS];
        uint64_t coherent_mem_rx_dma_addr[NUM_BUFFERS];
} __attribute__((packed)) ;

int simple_read(/*struct packet *pkt*/);
int simple_write(/*struct packet *pkt*/);
struct xdma_desc *tx_desc_virt;
struct xdma_desc *rx_desc_virt;
struct xdma_result *rx_result_virt;
extern unsigned char *tx_packet_buff;
/*
 * xdma_engine_stop() - stop an SG DMA engine
 */

void xdma_engine_stop(struct xdma_engine *engine)
{
	unsigned int w;

	w = 0;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
	w |= (unsigned int) XDMA_CTRL_POLL_MODE_WB;

	engine->regs->control = w;
}

static inline __attribute__((always_inline)) uint64_t current_clock_cycles()

{

	unsigned int eax_reg_rdtsc, edx_reg_rdtsc;

	unsigned long long int uptime;

	asm volatile  ("rdtsc\n\t" 
			"mov %%eax, %0\n\t"
			"mov %%edx, %1\n\t"

			:       "=r" (eax_reg_rdtsc) , "=r" (edx_reg_rdtsc)

			:

			: "eax" , "edx"

			);



	uptime =  ((unsigned long long int)edx_reg_rdtsc << 32) | eax_reg_rdtsc;

	return uptime;

}



static inline __attribute__((always_inline)) void delay_clock_cycles(uint64_t clock_cycles)

{

	register uint64_t start, end;



	start = current_clock_cycles();

	do {

		end = current_clock_cycles();

	} while ((end - start) < clock_cycles);



}
void *dma_rx(void* rxbuff,int *pkt_len)
{
	int extra_adj = DESC_CNT - 1, next_adj, j = 0, offset, sgdma_offset;
	int length, i;
	//volatile unsigned int read_p, rx_desc_start;
	uint64_t read_p, rx_desc_start;
	unsigned int control, w;
	struct xdma_engine *engine;
	int dir_from_dev = 0;
	volatile struct xdma_result *result;

	rx_desc_start = data->rx_queue_dma_addr;
	//printf("trace :  func : %s line : %u rx_desc_start : %x\n",__func__,__LINE__,rx_desc_start);

	//read_p = data->coherent_mem_rx_dma_addr[0];
	read_p = (uint64_t) rxbuff;
	//length = LENGTH; /*max pkt size recv is 4096*/
	length = *pkt_len; /*max pkt size recv */

	engine = malloc(sizeof(struct xdma_engine));
	if (!engine) {
		printf("engine allocation failed");
		return NULL;
	}

	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	offset = H2C_CHANNEL_OFFSET + (channel * CHANNEL_SPACING);
	sgdma_offset = offset + SGDMA_OFFSET_FROM_CHANNEL;
	//printf("offset : %x sgdma_offset : %x\n",offset ,sgdma_offset);

	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	engine->regs = (struct engine_regs *)((uintptr_t)bar_base_user + offset);
	engine->sgdma_regs = (struct engine_sgdma_regs *)((uintptr_t)bar_base_user + sgdma_offset);
	engine->rx_result_buffer_bus = data->rx_result_dma_addr;
	result = rx_result_virt;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	while (j < DESC_CNT) {
		rx_desc_virt[j].dst_addr_lo = (PCI_DMA_L(read_p + (j * length)));
		rx_desc_virt[j].dst_addr_hi = (PCI_DMA_H(read_p + j * length));
		rx_desc_virt[j].src_addr_lo = (PCI_DMA_L(engine->rx_result_buffer_bus + j * sizeof(struct xdma_result)));
		rx_desc_virt[j].src_addr_hi = (PCI_DMA_H(engine->rx_result_buffer_bus + j * sizeof(struct xdma_result)));
		//rx_desc_virt[j].src_addr_lo = 0;
		//rx_desc_virt[j].src_addr_hi = 0;
			rx_desc_virt[j].next_lo = 0;// end of desc chain
			rx_desc_virt[j].next_hi = 0;
/*
		if (j == DESC_CNT - 1) {
			rx_desc_virt[j].next_lo = (PCI_DMA_L(rx_desc_start));
			rx_desc_virt[j].next_hi = (PCI_DMA_H(rx_desc_start));
		} else {
			rx_desc_virt[j].next_lo = (PCI_DMA_L(rx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
			rx_desc_virt[j].next_hi = (PCI_DMA_H(rx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
		}*/
		rx_desc_virt[j].bytes = (length);
		control = (DESC_MAGIC);
		if (j == DESC_CNT - 1)   /* if last packet */
			control |= XDMA_DESC_STOPPED_1; /* set to 1 to stop fetching descriptors */
		else
			control |= XDMA_DESC_STOPPED_0;

		control |= XDMA_DESC_EOP;
		control |= XDMA_DESC_COMPLETED;

		next_adj = extra_adj - j - 1;
		if (next_adj < 1)
			next_adj = 0;
		rx_desc_virt[j].control = (control) | ((next_adj << 8) & 0x00003f00);
		j++;
	}
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	result->length = 0;
	engine->regs->completed_desc_count = 0;
	w = (PCI_DMA_L(rx_desc_start));
	//printf("result->len : %d\n", result->length);
	/*printf("After accessing 1\n");
	printf("engine->sgdma_regs->first_desc_lo  : %x\n",engine->sgdma_regs->first_desc_lo);
	printf("After accessing 2\n");*/
	engine->sgdma_regs->first_desc_lo = PCI_DMA_L(rx_desc_start);
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	w = (PCI_DMA_H(rx_desc_start));
	engine->sgdma_regs->first_desc_hi = PCI_DMA_H(rx_desc_start);
	engine->sgdma_regs->first_desc_adjacent = extra_adj;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	w = (unsigned int)XDMA_CTRL_RUN_STOP;
	w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (unsigned int)XDMA_CTRL_POLL_MODE_WB;
	engine->regs->control = w;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	//printf("waiting for rx dma to complete on channel %d\n",channel);
	while (engine->regs->completed_desc_count < (DESC_CNT)) {
		if(!connected_to_guest) {
			printf("returning from func : %s because of guest disconnection\n",__func__);
			return;
		}
		//usleep(1);
		//delay_clock_cycles(3400);
		delay_clock_cycles(400);
	}
	//printf("read() : completed_desc_count = %d\n", engine->regs->completed_desc_count);
	//for(i=0; i < LENGTH / 4; i++)
	//	printf("%x", rd_data[i]);
	//*pkt_len = 0;	
	//printf("b4 packet received len : %d\n",*pkt_len);
	*pkt_len = result->length;
	//printf("aftr packet received len : %d\n",*pkt_len);
	xdma_engine_stop(engine);
	free(engine);
	return (rd_data);
}


int dma_tx(char * pkt, int pkt_len, int pkt_offset)
{
	int extra_adj = DESC_CNT - 1, next_adj, j = 0, offset, sgdma_offset;
        int length;
	unsigned int control, control_field, w;
	uint64_t write_p, tx_desc_start;
	//unsigned int write_p, tx_desc_start;
	struct xdma_engine *engine;
	int dir_from_dev = 0;

	//printf("pkt : %p pkt_len: %d pkt_offset : %d\n",pkt,pkt_len,pkt_offset);
//	pkt_offset = 0; //overridden

	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);

        tx_desc_start = data->tx_queue_dma_addr;
	
	//write_p = (uintptr_t)data->coherent_mem_tx_dma_addr[0] + pkt_offset;
	write_p = (uint64_t) pkt;
	//printf("trace :  func : %s line : %u\n",__func__,__LINE__);

	engine = malloc(sizeof(struct xdma_engine));
	if (!engine) {
		printf("engine allocation failed");
		return -1;
	}
//	printf("trace :  func : %s line : %u\n",__func__,__LINE__);

	offset = (channel * CHANNEL_SPACING);
	sgdma_offset = offset + SGDMA_OFFSET_FROM_CHANNEL;
	engine->regs = (struct engine_regs *)((uintptr_t)bar_base_user + offset);
	engine->sgdma_regs = (struct engine_sgdma_regs *)((uintptr_t)bar_base_user + sgdma_offset);
//	printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	while (j < DESC_CNT) {
		tx_desc_virt[j].src_addr_lo = (PCI_DMA_L(write_p + j * length));
		tx_desc_virt[j].src_addr_hi = (PCI_DMA_H(write_p + j * length));
		tx_desc_virt[j].dst_addr_lo = (0);
		tx_desc_virt[j].dst_addr_hi = (0);
		if (j == DESC_CNT - 1) {
			tx_desc_virt[j].next_lo = (0);
			tx_desc_virt[j].next_hi = (0);
		} else {
			tx_desc_virt[j].next_lo = (PCI_DMA_L(tx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
			tx_desc_virt[j].next_hi = (PCI_DMA_H(tx_desc_start + sizeof(struct xdma_desc) * (j + 1)));
		}
		tx_desc_virt[j].bytes = pkt_len;
		control = 0;
		control |= XDMA_DESC_STOPPED_0;
		control |= XDMA_DESC_EOP;
		control |= XDMA_DESC_COMPLETED;
		tx_desc_virt[j].control = control | DESC_MAGIC;
		
		j++;
	}
//	printf("trace :  func : %s line : %u\n",__func__,__LINE__);

	w = (PCI_DMA_L((uint64_t)tx_desc_start));
	engine->sgdma_regs->first_desc_lo = w;
//	printf("trace :  func : %s line : %u\n",__func__,__LINE__);
	w = (PCI_DMA_H(tx_desc_start));
	engine->sgdma_regs->first_desc_hi = w;
	engine->sgdma_regs->first_desc_adjacent = extra_adj;
	w = (unsigned int)XDMA_CTRL_RUN_STOP;
	w |= (unsigned int)XDMA_CTRL_IE_READ_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ERROR;
	w |= (unsigned int)XDMA_CTRL_IE_DESC_ALIGN_MISMATCH;
	w |= (unsigned int)XDMA_CTRL_IE_MAGIC_STOPPED;
	w |= (unsigned int)XDMA_CTRL_POLL_MODE_WB;
	engine->regs->control = w;
	//printf("waiting for tx dma to complete on channel %d\n",channel);
	while (engine->regs->completed_desc_count < (DESC_CNT)) {

		if(!connected_to_guest) {
			printf("returning from func : %s because of guest disconnection\n",__func__);
			return;
		}
		//delay_clock_cycles(3400);
		delay_clock_cycles(400);
		//usleep(1);
		
	}
//	printf("simple_write() : engine->regs->completed_desc_count = %d\n", engine->regs->completed_desc_count);
	xdma_engine_stop(engine);
	free(engine);
}

int configfd;

int init(int ch)
{
        int waitint;
        int i;
        char *address = NULL;
        void *tx_queue;

	channel = ch;
        configfd = open("/sys/kernel/debug/coherent_buffers",O_RDWR);

        if(configfd < 0) {
                perror("Open call failed\n");
                return -1;
        }

        data = malloc(sizeof(struct mydata));

        read(configfd,data,sizeof(struct mydata));

        for(i = 0; i < NUM_BUFFERS; i++) {
  //              printf("i : %d tx phys : %p rx phys : %p\n",i,
                //i//(void *)data->coherent_mem_tx_dma_addr[i],(void *)data->coherent_mem_rx_dma_addr[i]);
        }


        tx_desc_virt =  mmap(NULL,PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 0*PAGE_SIZE);
	
        if(tx_desc_virt == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
                printf("tx_desc_virt at vaddr : %p\n",tx_desc_virt);
        }
	
	rx_desc_virt =  mmap(NULL,PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 1*PAGE_SIZE);

        if(rx_desc_virt == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
                printf("rx_desc_virt at vaddr : %p\n",rx_desc_virt);
        }

	rx_result_virt =  mmap(NULL,PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 2*PAGE_SIZE);

	if(rx_result_virt == MAP_FAILED) {
		perror("mmap operation failed\n");
		return -1;
	} else {
		printf("rx_result_virt at vaddr : %p\n", rx_result_virt);
	}


	bar_base_user =  mmap(NULL, 64*1024, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 3*PAGE_SIZE);

        if(bar_base_user == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
                printf("bar_base_user at vaddr : %p\n", bar_base_user);
        }

	rd_data =  mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 68*PAGE_SIZE);

        if(rd_data == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
                printf("rd_data at vaddr : %p\n", rd_data);
        }

	wr_data =  mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, configfd, 4*PAGE_SIZE);
        if(wr_data == MAP_FAILED) {
                perror("mmap operation failed\n");
                return -1;
        } else {
		tx_packet_buff = (unsigned char *)wr_data;
                printf("wr_data : %p\n", wr_data);
        }
	
//	printf(" bar 0 -> h2c eng -> %x\n", *bar_base_user);
        
	//close(configfd);
        return 0;
}

void uinit()
{
  close(configfd);
}



