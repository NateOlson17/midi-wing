#include "mbed.h"
#include "USBMIDI.h"
#include "usbcore.h"

DigitalOut MUX_0(p21);
DigitalOut MUX_1(p22);
DigitalOut MUX_2(p23);
DigitalOut MUX_3(p24);

DigitalIn INT_0A(p13);
DigitalIn INT_1A(p14);
DigitalIn INT_2A(p25);
DigitalIn INT_3A(p26);
AnalogIn FADER_IN(p20, 3.3);

I2C bus(p28, p27);

USBMIDI midicon;


//FIND BETTER WAY TO DETECT FADER VAL CHANGE
//WIRE INPUTS FOR B SIDE INTERRUPTS AT 11/12/30/29 (maybe move them around for clean wiring)

/*
Takes device address, register to read, buffer to store value in.
Reads from a given GPIO expander register for all slaves.
Returns 0 on success, -n on ACK fail, where nth sent byte was NACK by slave.
*/
int readReg(uint8_t addr, uint8_t reg, uint8_t *buf) {
    uint8_t opcode = (addr << 1) | 0x40; //format opcode as 0100AAA0, last bit writes
    bus.start();
    if (bus.write(opcode) == 1) { //if control byte ack by slave, send reg byte (2 on timeout, 0 on nack)
        if (bus.write(reg) == 1) { //if register address ack, restart and send next control byte
            bus.start();
            if (bus.write(opcode | 1) == 1) {//send second control byte (opcode with read bit high)
                *buf = bus.read(1); //read reg into buf and send ack
                bus.stop();
                return 0;
            } else {bus.stop(); return -3;}
        } else {bus.stop(); return -2;}
    } else {bus.stop(); return -1;}
}

/*
Takes device address, register to write to, write data buffer.
Writes to a given GPIO expander register for all slaves.
Returns 0 on success, -n on ACK fail, where nth sent byte was NACK by slave.
*/
int writeReg(uint8_t addr, uint8_t reg, uint8_t *buf) {
    uint8_t opcode = (addr << 1) | 0x40; //format opcode as 0100AAA0, last bit writes
    bus.start();
    if (bus.write(opcode) == 1) { //if control byte ack by slave, send reg byte (2 on timeout, 0 on nack)
        if (bus.write(reg) == 1) { //if register address ack, send write data
            if (bus.write(*buf) == 1) {
                bus.stop();
                return 0;
            } else {bus.stop(); return -3;}
        } else {bus.stop(); return -2;}
    } else {bus.stop(); return -1;}
}


/*
Init I2C network.
Configures:
-All pins as inputs
-Pin logic state to match input
-All pins enabled for interrupt on change
-Interrupt comparison to previous value, not DEFVAL
-Bank addressing
-Non mirrored interrupt pins
-Automatic pointer incrementation
-Interrupts as active LOW
-Pull up resistors enabled on all pins (will idle high)
Returns 0 on success, -1 on write fail.
*/
int busInit() {
    bus.frequency(400000);
    uint8_t configbuf = 0xff;
    for (uint8_t addr = 0; addr < 4; addr++) { //write config to each I2C slave
        if (writeReg(addr, 0x04, &configbuf) || //GPINTENA reg
        writeReg(addr, 0x05, &configbuf) ||
        writeReg(addr, 0x0C, &configbuf) || //GPPUA reg
        writeReg(addr, 0x0D, &configbuf)) {
            return -1; //if any write fails
        }
    }
    return 0;
}

/*
Takes command byte (0x06 for set), data pointer, and data length.
Sends a formatted MSC message over serial.
Returns 0 on success.
*/
int sendMSC(uint8_t command, uint8_t *data, int datalen) {
    uint8_t msc_packet[] = {0xF0, 0x7F, 0x7F, 0x02, 0x7F, command};
    /*[F07F] header, marks univ sys ex/realtime
      [7F] target device (broadcast)
      [02] MSC specifier
      [7F] command format (all)*/
    memcpy(msc_packet + 6, data, datalen); //copy data to end of msc packet
    uint8_t msc_close = 0xF7; //closing octet
    memcpy(msc_packet + 6 + datalen, &msc_close, 1);
    
    ep_write(EP5, msc_packet, 7 + datalen);
    return 0;
}

int main()
{
    printf("================\n");
    busInit();
    double faderval[15] = { }; //init fader values at 0
    
    while (true) {
        for (uint8_t mux_addr = 0; mux_addr < 15; mux_addr++) { //iterate over fader address (0-15)
            MUX_0 = mux_addr & 0x01; //send on A0-A3 pins, set mux switch to given address
            MUX_1 = mux_addr & 0x02;
            MUX_2 = mux_addr & 0x04;
            MUX_3 = mux_addr & 0x08;
            double newfaderval = FADER_IN.read() * 100;
            if (abs(newfaderval - faderval[mux_addr]) > .3) { //if current val differs from prev by more than .3%, update
                faderval[mux_addr] = newfaderval;
                uint8_t fader_coarse = newfaderval * 1.28; //get discrete 8 bit values
                uint8_t fader_fine = (newfaderval - trunc(newfaderval)) * 128;
                //printf("[A] %i [C] %i [F] %i\n", mux_addr, fader_coarse, fader_fine);
                uint8_t mscmsg[] = {mux_addr, 1, fader_fine, fader_coarse}; //fader num, page num, fine, coarse
                sendMSC(0x06, mscmsg, 4);
            }
        }
    }
}