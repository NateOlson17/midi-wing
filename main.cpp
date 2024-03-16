#include "mbed.h"
#include <vector>
#include "USBMIDI.h"

#define DEBUG 1 //ENABLE/DISABLE DEBUG OVER SERIAL - PRINTF IS ALIASED, WILL BLOCK UNTIL COM PORT IS AVAILABLE. TURN OFF DEBUG WHEN RUNNING WITHOUT COM!
#define VERBOSE 1 //Disabling debug will override verbose option

#if DEBUG 
  #define debug(a) printf a
#else
  #define debug(a) (void)0
#endif

DigitalOut MUX_0(p21);
DigitalOut MUX_1(p22);
DigitalOut MUX_2(p23);
DigitalOut MUX_3(p24);

DigitalIn INT_0(p13);
DigitalIn INT_1(p14);
DigitalIn INT_2(p25);
DigitalIn INT_3(p26);

AnalogIn FADER_IN(p20, 3.3);

I2C bus(p28, p27);

USBMIDI usb_con;


//change to HID
//Set up battery harness for LEDs
//Implement sendMSC properly
//Figure out better fader change detection

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
                *buf = bus.read(0); //read reg into buf and send nack
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
Init/config I2C devices and USB HID.
MCP23017 configuration:
-All pins as inputs (except GPA7/GPB7, in datasheet errata they are output only on MCP23017 after 2022, bidirectional on MCP23S17!)
-Pin logic state opposite input level
-All pins enabled for interrupt on change (except GPA7/GPB7)
-Interrupt comparison to previous value, not DEFVAL
-Non-banked addressing
-Internally linked interrupt pins
-Automatic pointer incrementation disabled
-INT as active driver output
-Interrupts as active LOW
-100k pull up resistors enabled on all pins (except GPA7/GPB7)
Returns 0 on success, -n on write fail. n = 1 for IOCON, 2 for IODIR, 3 for IPOL, 4 for GPINTEN, 5 for GPPU. -6 for USB init fail.
*/
int busInit() {
    usb_con.init();
    if (usb_con.ready()) {
        debug(("USB config success!\n"));
    } else {
        debug(("USB config failed!\n"));
        return -6;
    }

    bus.frequency(400000); //I2C bus
    uint8_t IOCON_cfg = 0x40; //MCP register configs
    uint8_t IODIR_cfg = 0x7f;
    uint8_t GPINTEN_cfg = 0x7f;
    uint8_t GPPU_cfg = 0x7f;
    uint8_t IPOL_cfg = 0x7f;

    for (uint8_t addr = 0; addr < 4; addr++) { //write config to each I2C slave
        debug(("Writing config to slave %i\n", addr));
        if (writeReg(addr, 0x0A, &IOCON_cfg)) { //IOCON reg
            debug(("IOCON write failed!\n"));
            return -1;
        } else if (writeReg(addr, 0x00, &IODIR_cfg) ||
                    writeReg(addr, 0x01, &IODIR_cfg)) { //IODIR A/B
            debug(("IODIR write failed!\n"));
            return -2;
        } else if (writeReg(addr, 0x02, &IPOL_cfg) ||
                    writeReg(addr, 0x03, &IPOL_cfg)) { //IPOL A/B
            debug(("IPOL write failed!\n"));
            return -3;
        } else if (writeReg(addr, 0x04, &GPINTEN_cfg) ||
                    writeReg(addr, 0x05, &GPINTEN_cfg)) { //GPINTEN A/B
            debug(("GPINTEN write failed!\n"));
            return -4;
        } else if (writeReg(addr, 0x0C, &GPPU_cfg) ||
                    writeReg(addr, 0x0D, &GPPU_cfg)) { //GPPU A/B
            debug(("GPPU write failed!\n"));
            return -5;
        } 
    }
    return 0;
}

/*
Takes command byte (0x06 for set), data pointer, and data length.
Sends a formatted MSC message over serial.
Returns 0 on success.
*/
int sendMSC(uint8_t command, uint8_t *mscdata, int datalen) {
    vector<uint8_t> msc_packet{0xF0, 0x7F, 0x7F, 0x02, 0x7F, command};
    /*[F07F] header, marks univ sys ex/realtime
      [7F] target device (broadcast)
      [02] MSC specifier
      [7F] command format (all)*/
    for (int i = 0; i < datalen; ++i) {
        msc_packet.push_back(mscdata[i]);
    }
    msc_packet.push_back(0xF7); //closing octet
    
    if (VERBOSE) {
        debug(("Sending MSC packet => "));
        for (int i = 0; i < 7 + datalen; ++i) {
            debug(("%x ", msc_packet[i]));
        }
        debug(("\n"));
    }
    usb_con.write(MIDIMessage::SysEx(&msc_packet.front(), 7 + datalen));
    return 0;
}

//Reads and prints MCP23017 register values of interest for debug purposes.
void reportRegisterStatus() {
    uint8_t rbuf;
    readReg(0x00, 0x0A, &rbuf);
    debug(("IOCON=%x | ", rbuf));
    readReg(0x00, 0x00, &rbuf);
    debug(("IODIRA=%x | ", rbuf));
    readReg(0x00, 0x01, &rbuf);
    debug(("IODIRB=%x | ", rbuf));
    readReg(0x00, 0x04, &rbuf);
    debug(("GPINTENA=%x | ", rbuf));
    readReg(0x00, 0x05, &rbuf);
    debug(("GPINTENB=%x | ", rbuf));
    readReg(0x00, 0x08, &rbuf);
    debug(("INTCONA=%x | ", rbuf));
    readReg(0x00, 0x09, &rbuf);
    debug(("INTCONB=%x | ", rbuf));
    readReg(0x00, 0x0C, &rbuf);
    debug(("GPPUA=%x | ", rbuf));
    readReg(0x00, 0x0D, &rbuf);
    debug(("GPPUB=%x | ", rbuf));
    readReg(0x00, 0x12, &rbuf);
    debug(("GPIOA=%x | ", rbuf));
    readReg(0x00, 0x13, &rbuf);
    debug(("GPIOB=%x | ", rbuf));
    readReg(0x00, 0x0E, &rbuf);
    debug(("INTFA=%x | ", rbuf));
    readReg(0x00, 0x0F, &rbuf);
    debug(("INTFB=%x | ", rbuf));
    if (INT_0) {
        debug(("INT HIGH\n"));
    } else {
        debug(("INT LOW\n"));
    }
}

/*
Takes MCP23017 device address.
Reads interrupt flags on given IC and finds value of corresponding key. For future-proofing, supports detection of multiple simultaneous/waiting interrupt flags, although MCP23017 flags one port exclusively.
Returns 0 on success. 
*/
int MCPIntHandler(uint8_t dev_addr) {
    uint8_t intfa_buf;
    uint8_t intfb_buf;
    uint8_t intcapa_buf = 0;
    uint8_t intcapb_buf = 0; 
    readReg(dev_addr, 0x0E, &intfa_buf); //fetch INTF registers
    readReg(dev_addr, 0x0F, &intfb_buf);
    if (intfa_buf) {readReg(dev_addr, 0x10, &intcapa_buf);} //fetch INTCAP registers for nonzero INTF banks
    if (intfb_buf) {readReg(dev_addr, 0x11, &intcapb_buf);}

    vector<uint8_t> intaflags;
    vector<uint8_t> intavals;
    vector<uint8_t> intbflags;
    vector<uint8_t> intbvals;
    for (int i = 0; i < 5; ++i) { //iterate through active bits of flag bytes
        if (intfa_buf && intfa_buf & (1 << i)) { //if nonzero flag byte and current bit is set
            intaflags.push_back(i); //add index of flagged bit to vector
            intavals.push_back((intcapa_buf & (1 << i)) ? 1:0); //find value of flagged input and add to value vector
        }
        if (intfb_buf && intfb_buf & (1 << i)) { //if nonzero flag byte and current bit is set
            intaflags.push_back(i); //add index of flagged bit to vector
            intbvals.push_back((intcapb_buf & (1 << i)) ? 1:0); //find value of flagged input and add to value vector
        }
    }

    for (unsigned int i = 0; i < intaflags.size(); ++i) {
        debug(("INTERRUPT: BTN A%i BANK %i AT %i\n", intaflags[i], dev_addr, intavals[i]));
    }
    for (unsigned int i = 0; i < intbflags.size(); ++i) {
        debug(("INTERRUPT: BTN B%i BANK %i AT %i\n", intbflags[i], dev_addr, intbvals[i]));
    }
    return 0;
}

int main()
{
    debug(("========MAIN========\n"));
    if (busInit()) {
        debug(("I2C bus init failed!\n"));
    } else {
        debug(("I2C bus init success!\n"));
    }

    double faderval[15] = { }; //init fader values at 0
    while (true) {
        if (VERBOSE) {reportRegisterStatus();}

        for (uint8_t mux_addr = 0; mux_addr < 15; ++mux_addr) { //iterate over fader address (0-15)
            MUX_0 = mux_addr & 0x01; //send on A0-A3 pins, set mux switch to given address
            MUX_1 = mux_addr & 0x02;
            MUX_2 = mux_addr & 0x04;
            MUX_3 = mux_addr & 0x08;
            double newfaderval = FADER_IN.read() * 100;
            if (abs(newfaderval - faderval[mux_addr]) > .3) { //if current val differs from prev by more than .3%, update
                faderval[mux_addr] = newfaderval;
                uint8_t fader_coarse = newfaderval * 1.28; //get discrete 8 bit values
                uint8_t fader_fine = (newfaderval - trunc(newfaderval)) * 128;
                debug(("Fader update: [A] %x [C] %i [F] %i\n", mux_addr, fader_coarse, fader_fine));
                uint8_t mscmsg[] = {mux_addr, 1, fader_fine, fader_coarse}; //fader num, page num, fine, coarse
                sendMSC(0x06, mscmsg, 4);
            }
        }

        if (!INT_0) {MCPIntHandler(0);}
        if (!INT_1) {MCPIntHandler(1);}
        if (!INT_2) {MCPIntHandler(2);}
        if (!INT_3) {MCPIntHandler(3);}
    }
}