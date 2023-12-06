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

#define STANDARD_DEVICE_REQUEST     (0x00)
#define STANDARD_INTERFACE_REQUEST  (0x01)
#define STANDARD_ENDPOINT_REQUEST   (0x02)
#define CLASS_DEVICE_REQUEST        (0x20)
#define CLASS_INTERFACE_REQUEST     (0x21)
#define CLASS_ENDPOINT_REQUEST      (0x22)
#define VENDOR_DEVICE_REQUEST       (0x40)
#define VENDOR_INTERFACE_REQUEST    (0x41)
#define VENDOR_ENDPOINT_REQUEST     (0x42)
#define GET_STATUS                  (0x00)
#define CLEAR_FEATURE               (0x01)
#define SET_FEATURE                 (0x03)
#define SET_ADDRESS                 (0x05)
#define GET_DESCRIPTOR              (0x06)
#define SET_DESCRIPTOR              (0x07)
#define GET_CONFIGURATION           (0x08)
#define SET_CONFIGURATION           (0x09)
#define DEVICE_DESCRIPTOR           (0x01)
#define CONFIG_DESCRIPTOR           (0x02)
#define STRING_DESCRIPTOR           (0x03)
#define INTERFACE_DESCRIPTOR        (0x04)
#define ENDPOINT_DESCRIPTOR         (0x05)
#define QUALIFIER_DESCRIPTOR        (0x06)
#define unpack(x) (x & 0xFF),((x >> 8) & 0xFF)

// setup packet
struct {
    uint8_t   bmRequestType;
    uint8_t   bRequest;
    uint16_t  wValue;
    uint16_t  wIndex;
    uint16_t  wLength;
} __attribute__((packed)) setup = {0};

// data packet
struct { 
    uint8_t *data;
    uint8_t size;
    uint8_t sent;
} transfer = {0};

uint8_t device_descriptor[] = {
    0x12,                       // Descriptor size in bytes (12h)                  
    DEVICE_DESCRIPTOR,          // The constant DEVICE (01h)                       
    unpack(0x0200),             // US2B specification release number (BCD)         
    0x00,                       // Class code                                      
    0x00,                       // Subclass code                                   
    0x00,                       // Protocol Code                                   
    MAX_EP0_PSIZE,              // Maximum packet size for endpoint zero           
    unpack(0x0763),             // Vendor ID                                       
    unpack(0x0198),             // Product ID                                      
    unpack(0x0001),             // Device release number (BCD)                     
    0x00,                       // Index of string descriptor for the manufacturer 
    0x00,                       // Index of string descriptor for the product      
    0x00,                       // Index of string descriptor for the serial number
    0x01,                       // Number of possible configurations               
};

uint8_t config_descriptor[]={
    0x09, 0x02, 0x65, 0x00, 0x02, 0x01, 0x00, 0xc0, 0x50, // configuration descriptor
    // The Audio Interface Collection
    0x09, 0x04, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, // Standard AC Interface Descriptor
    0x09, 0x24, 0x01, 0x00, 0x01, 0x09, 0x00, 0x01, 0x01, // Class-specific AC Interface Descriptor
    0x09, 0x04, 0x01, 0x00, 0x02, 0x01, 0x03, 0x00, 0x00, // MIDIStreaming Interface Descriptors
    0x07, 0x24, 0x01, 0x00, 0x01, 0x41, 0x00,             // Class-Specific MS Interface Header Descriptor

    // MIDI IN JACKS
    0x06, 0x24, 0x02, 0x01, 0x01, 0x00,
    0x06, 0x24, 0x02, 0x02, 0x02, 0x00,

    // MIDI OUT JACKS
    0x09, 0x24, 0x03, 0x01, 0x03, 0x01, 0x02, 0x01, 0x00,
    0x09, 0x24, 0x03, 0x02, 0x06, 0x01, 0x01, 0x01, 0x00,

    // OUT endpoint descriptor
    0x09, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x05, 0x25, 0x01, 0x01, 0x01,

    // IN endpoint descriptor
    0x09, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x05, 0x25, 0x01, 0x01, 0x03,
};

void ep0_in();

void data_in_stage(uint8_t *desc, uint8_t length) {
    transfer.sent = 0;
    transfer.data = desc;
    transfer.size = length;
    ep0_in();
}

void status_in_stage() {
    ep_write(EP1, 0, 0); // ZLEP for status stage
}

void ep0_setup() {
    ep_read(EP0,(uint8_t*) &setup);

    switch (setup.bmRequestType & 0x7f) { // mask direction
        case STANDARD_DEVICE_REQUEST:
            switch (setup.bRequest) {
                case GET_DESCRIPTOR:
                    switch ((setup.wValue>>8)) {
                        case DEVICE_DESCRIPTOR:     // device descriptor request
                        case QUALIFIER_DESCRIPTOR:  // device qualifier descriptor
                            data_in_stage(device_descriptor, sizeof(device_descriptor));
                            break;
                        case CONFIG_DESCRIPTOR:     // configuration descriptor
                            data_in_stage(config_descriptor, setup.wLength);
                            break;
                        case STRING_DESCRIPTOR:
                            break;
                        default:
                            break;
                    }
                    break;
                case SET_ADDRESS:
                    usb_set_address((uint8_t) (setup.wValue & 0xFF));
                    status_in_stage();
                    break;
                case SET_CONFIGURATION:
                    if (!setup.wValue) {
                        break;
                    }
                    ep_realize(EP4, MAX_EPn_PSIZE);
                    ep_realize(EP5, MAX_EPn_PSIZE);
                    status_in_stage();
                    usb_configure(1);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void ep0_in() {
    if ((setup.bmRequestType & 0x80) && transfer.size) { // device to host
        if (transfer.size > MAX_EP0_PSIZE) {
            transfer.sent = MAX_EP0_PSIZE;
        } else {
            transfer.sent = transfer.size;
        }
        ep_write(EP1, transfer.data, transfer.sent);
        transfer.data += transfer.sent;
        transfer.size -= transfer.sent;
    }
}

void ep0_out() {
    uint8_t buf[64];
    ep_read(EP0, buf);
}
