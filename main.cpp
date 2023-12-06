#include "mbed.h"
#include "USBMIDI.h"

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


//WIRE INPUTS FOR B SIDE INTERRUPTS AT 11/12/30/29 (maybe move them around for clean wiring)

/*
Takes device address, register to read, buffer to store value in.
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
    for (uint8_t addr = 0; addr < 4; addr++) { //write config to each slave
        if (writeReg(addr, 0x04, &configbuf) || //GPINTENA reg
        writeReg(addr, 0x05, &configbuf) ||
        writeReg(addr, 0x0C, &configbuf) || //GPPUA reg
        writeReg(addr, 0x0D, &configbuf)) {
            return -1; //if any write fails
        }
    }
    return 0;
}

int main()
{
    printf("================\n");
    busInit();
    
    while (true) {
        //check interrupt pins

        printf("\n\n----------------\n\n");
        for (uint8_t mux_addr = 0; mux_addr < 16; mux_addr++) {
            MUX_0 = mux_addr & 0x01;
            MUX_1 = mux_addr & 0x02;
            MUX_2 = mux_addr & 0x04;
            MUX_3 = mux_addr & 0x08;
            printf("A: %i: %.4f\n", mux_addr, FADER_IN.read());
        }
    }
}