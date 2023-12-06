/* @license The MIT License
 * Copyright (c) 2011 mux, simon
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *  
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *  
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "usbcore.h"

#include "mbed.h"

// Serial Interface Engine
#define SIE_SET_ADDR    (0xD0)
#define SIE_SET_STATUS  (0xFE)
#define SIE_GET_STATUS  (0xFE)
#define SIE_SET_MODE    (0xF3)
#define SIE_CLR_BUFFER  (0xF2)
#define SIE_VAL_BUFFER  (0xFA)
#define SIE_SEL_EP      (0x00)
#define SIE_SEL_CLR_EP  (0x28)
#define SIE_SET_EP_STAT (0x40)
#define SIE_READ_ERROR  (0xFB)
#define SIE_CONFIG_DEVICE (0xD8)

// EP status
#define EP_FE           (1<<0) // Full/Empty
#define EP_ST           (1<<1) // Stalled endpoint
#define EP_STP          (1<<2) // Setup packet
#define EP_PO           (1<<3) // packet overwritten
#define EP_EPN          (1<<4) // EP NAKed
#define B_1_FULL        (1<<5) // buffer 1 status
#define B_2_FULL        (1<<6) // buffer 2 status

// USB device interrupts
#define DEV_FRAME       (1<<0)
#define EP_FAST         (1<<1)
#define EP_SLOW         (1<<2)
#define DEV_STAT        (1<<3)
#define RxENDPKT        (1<<6)
#define TxENDPKT        (1<<7)
#define EP_RLZED        (1<<8)
#define CCEMPTY         (0x10)
#define CDFULL          (0x20)

// USB device status bits
#define STAT_CON        (1<<0)
#define STAT_CON_CH     (1<<1)
#define STAT_SUS        (1<<2)
#define STAT_SUS_CH     (1<<3)
#define STAT_RST        (1<<4)

// end points interrupts
#define EP0RX_INT       (1<<0)
#define EP0TX_INT       (1<<1)
#define EP1RX_INT       (1<<2)
#define EP1TX_INT       (1<<3)
#define EP2RX_INT       (1<<4)
#define EP2TX_INT       (1<<5)

// USB control register
#define RD_EN           (1<<0)
#define WR_EN           (1<<1)
#define PKT_RDY         (1<<11)
#define LOG_ENDPOINT(ep) ((ep>>1)<<2)

// configure state
static int configured = 0;

// USB interrupt handler
void USB_IRQHandler(void);

// Serial Interface Engine functions
void sie_command(uint32_t code) {
    LPC_USB->USBDevIntClr = CCEMPTY;                // clear CCEMPTY     
    LPC_USB->USBCmdCode   = ((code<<16)|(0x05<<8)); // CMD_PHASE=Command 
    while (!(LPC_USB->USBDevIntSt & CCEMPTY));      // wait for CCEMPTY 
}

void sie_write(uint32_t data) {
    LPC_USB->USBDevIntClr = CCEMPTY;                // clear CCEMPTY     
    LPC_USB->USBCmdCode   = ((data<<16)|(0x01<<8)); // CMD_PHASE=Write   
    while (!(LPC_USB->USBDevIntSt & CCEMPTY));      // wait for CCEMPTY  
}

uint8_t sie_read(uint32_t code) {
    LPC_USB->USBDevIntClr = CDFULL;                 // clear CCEMPTY     
    LPC_USB->USBCmdCode   = ((code<<16)|(0x02<<8)); // CMD_PHASE=Read    
    while (!(LPC_USB->USBDevIntSt & CDFULL));       // wait for CDFULL   
    return (uint8_t) LPC_USB->USBCmdData;

}

// end point functions
void ep_realize(uint8_t ep, uint32_t size) {
    LPC_USB->USBDevIntClr = EP_RLZED;           // clear EP_RLZED
    LPC_USB->USBReEp    |= (1<<ep);
    LPC_USB->USBEpInd    = ep;                  // set USBEpIn
    LPC_USB->USBMaxPSize = size;                // writing to EPn pointed to by USBEpInd
    while (!(LPC_USB->USBDevIntSt & EP_RLZED)); // wait for EP_RLZED
    LPC_USB->USBDevIntClr = EP_RLZED;           // clear EP_RLZED
}

void ep_stall(uint8_t ep) {
    sie_command(SIE_SET_EP_STAT+ep);
    sie_write(1);
}

void ep_unstall(uint8_t ep) {
    sie_command(SIE_SET_EP_STAT+ep);
    sie_write(0);
}

// initializes a pointer to the endpoint buffer
uint8_t ep_select(uint8_t ep) {
    sie_command(SIE_SEL_EP+ep);
    return sie_read(SIE_SEL_EP+ep);
}

uint8_t ep_select_clear(uint8_t ep) {
    LPC_USB->USBEpIntClr |= ep;                 // clear ep interrupt   
    while (!(LPC_USB->USBDevIntSt & CDFULL));   // wait for cmd finish 
    return LPC_USB->USBCmdData;
}

int ep_readable(uint8_t ep) {
    uint8_t st = ep_select(ep);
    return (st & EP_FE);
}

int ep_writable(uint8_t ep) {
    uint8_t st = ep_select(ep);
    return !(st & EP_FE);
}

int ep_read(uint8_t ep, uint8_t *tbuf) {
    uint32_t *buf = (uint32_t*) tbuf;
    LPC_USB->USBCtrl = LOG_ENDPOINT(ep)|RD_EN;  // RD_EN bit and LOG_ENDPOINT   
    while (!(LPC_USB->USBRxPLen & PKT_RDY));    // wait for packet to be fetched
    int len = LPC_USB->USBRxPLen & 0x3FF;       // read and mask packet length  
    while (!(LPC_USB->USBDevIntSt & RxENDPKT)) {
        *buf++ = LPC_USB->USBRxData;
    }
    LPC_USB->USBCtrl = 0;
    LPC_USB->USBDevIntClr |= RxENDPKT;
    sie_command(SIE_SEL_EP+ep);     // select endpoint   
    sie_command(SIE_CLR_BUFFER);    // clear RX buffer   
    return len;
}

void ep_write(uint8_t ep, uint8_t *tbuf, uint32_t len) {
    uint32_t *buf = (uint32_t*) tbuf;
    LPC_USB->USBCtrl   = LOG_ENDPOINT(ep)|WR_EN; // RD_EN bit and LOG_ENDPOINT  
    LPC_USB->USBTxPLen |= (len & 0x3FF);         // write and mask packet length
    while (!(LPC_USB->USBDevIntSt & TxENDPKT)) {
        LPC_USB->USBTxData = *buf++;
    }
    LPC_USB->USBCtrl = 0;
    LPC_USB->USBDevIntClr |= TxENDPKT;
    sie_command(SIE_SEL_EP+ep);     // select endpoint   
    sie_command(SIE_VAL_BUFFER);    // validate TX buffer
}

// USB device controller initialization
void usb_init() {
    // USB D+/D- pinsel functions
    LPC_PINCON->PINSEL1 &=   0xC3FFFFFF;
    LPC_PINCON->PINSEL1 |=   0x14000000;

#if USB_UP_DEBUG
    // USB_UP_LED pinsel function
    LPC_PINCON->PINSEL3 &=   0xFFFFFFCF;
    LPC_PINCON->PINSEL3 |=   0x00000010;
#endif    

    // USB connect pinsel function
    LPC_PINCON->PINSEL4 &=   0xFFFCFFFF;
    LPC_PINCON->PINSEL4 |=   0x00040000;
    LPC_SC->PCONP       |=   (1UL<<31);         // enable the USB controller
    LPC_USB->USBClkCtrl |=   ((1<<1)|(1<<4));   // enable the AHB and DEV clocks
    while ((LPC_USB->USBClkSt & 0x12) != 0x12); // wait for the clocks to init

    NVIC_SetVector(USB_IRQn, (uint32_t)&USB_IRQHandler);
    NVIC_EnableIRQ(USB_IRQn);    // enable USB interrupts

    usb_reset();
    usb_set_address(0);          // default address
}

void usb_reset() {
    ep_realize(EP0, MAX_EP0_PSIZE);
    ep_realize(EP1, MAX_EP0_PSIZE);
    LPC_USB->USBEpIntClr  = 0xFFFFFFFF;         // clear end points interrupts
    LPC_USB->USBEpIntEn   = 0xFFFFFFFF;         // enable end points interrupts
    LPC_USB->USBEpIntPri  = 0x0;                // route to EP_SLOW
    LPC_USB->USBDevIntClr = 0xFFFFFFFF;         // clear USB device interrupts 
    LPC_USB->USBDevIntEn  = (EP_SLOW|DEV_STAT); // enable USB device interrupts
}

void usb_configure(uint8_t conf) {
    sie_command(SIE_CONFIG_DEVICE);
    sie_write(conf);
    configured = 1;
}

int usb_configured() {
    return configured;
}

void usb_set_address(uint8_t addr) {
    sie_command(SIE_SET_ADDR);
    sie_write(addr|0x80);       // DEV_EN = 1
}

uint8_t usb_get_status() {
    sie_command(SIE_GET_STATUS);
    return sie_read(SIE_GET_STATUS);
}

void usb_connect() {
    sie_command(SIE_GET_STATUS); // read current status
    uint8_t st = sie_read(SIE_GET_STATUS);

    sie_command(SIE_SET_STATUS); // set STAT_CON bit
    sie_write(st|STAT_CON);
}

void USB_IRQHandler(void) {
    if (LPC_USB->USBDevIntSt & DEV_STAT) { // DEV_STAT interrupt
        LPC_USB->USBDevIntClr |= DEV_STAT;
        if (usb_get_status() & STAT_RST) { // bus reset
            usb_reset();
        }
        return;
    }

    if (LPC_USB->USBDevIntSt & EP_SLOW) {  // EP_SLOW interrupt
        if (LPC_USB->USBEpIntSt & EP0RX_INT) {
            if (ep_select_clear(EP0RX_INT) & EP_STP) { // setup transfer
                ep0_setup();
            } else {
                ep0_out();
            }
        }

        if (LPC_USB->USBEpIntSt & EP0TX_INT) {
            ep_select_clear(EP0TX_INT);
            ep0_in();
        }

        if (LPC_USB->USBEpIntSt & EP2RX_INT) {
            ep_select_clear(EP2TX_INT);
            ep2_out();
        }

        if (LPC_USB->USBEpIntSt & EP2TX_INT) {
            ep_select_clear(EP2TX_INT);
            ep2_in();
        }

        // EP_SLOW should be cleared after clearing EPs interrupts
        LPC_USB->USBDevIntClr |= EP_SLOW;
    }
}
