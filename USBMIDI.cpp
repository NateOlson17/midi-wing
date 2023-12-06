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

#include "USBMIDI.h"

#include "mbed.h"
#include "usbcore.h"

static void (*midi_evt)(MIDIMessage) = NULL;

USBMIDI::USBMIDI() {
    usb_init();
    usb_connect();
    while (!usb_configured());  
}

void USBMIDI::write(MIDIMessage m) {
    while (!writeable());
    ep_write(EP5, m.data, 4);
}

bool USBMIDI::writeable() {
    return (bool)ep_writable(EP5);
}

void USBMIDI::attach(void (*fptr)(MIDIMessage)) {
    midi_evt = fptr;
}

void ep2_in() {}

void ep2_out() {    
    uint8_t buf[MAX_EPn_PSIZE];
    int len = ep_read(EP4, buf);
    
    if (midi_evt != NULL) {
        for(int i=0; i<len; i+=4){
            midi_evt(MIDIMessage(buf+i));
        }
    }
}
