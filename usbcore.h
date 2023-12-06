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

#ifndef USB_CORE_H
#define USB_CORE_H

#include "mbed.h"

#define MAX_EP0_PSIZE   (8)
#define MAX_EPn_PSIZE   (64)

// physical endpoint numbers
#define EP0             (0)
#define EP1             (1)
#define EP2             (2)
#define EP3             (3)
#define EP4             (4)
#define EP5             (5)

void usb_init();
void usb_reset();
void usb_connect();
void usb_set_address(uint8_t);
void usb_configure(uint8_t);
int usb_configured();
void ep_realize(uint8_t ep, uint32_t psize);
int  ep_read(uint8_t ep, uint8_t *buf);
void ep_write(uint8_t ep, uint8_t *buf, uint32_t len);
int ep_readable(uint8_t ep);
int ep_writable(uint8_t ep);
void ep0_setup();
void ep0_in();
void ep0_out();
void ep2_in();
void ep2_out();

#endif
