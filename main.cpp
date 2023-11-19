#include "mbed.h"

DigitalOut MUX_0(p21);
DigitalOut MUX_1(p22);
DigitalOut MUX_2(p23);
DigitalOut MUX_3(p24);

DigitalIn INT_0(p13);
DigitalIn INT_1(p14);
DigitalIn INT_2(p25);
DigitalIn INT_3(p26);
AnalogIn FADER_IN(p20);

I2C bus(p28, p27);

/*
Takes device address, register to read, buffer to store value in.
Returns 0 on success, 1 on ACK fail.
*/
int readReg(uint8_t addr, uint8_t reg, uint8_t *buf) {
    uint8_t opcode = (addr << 1) | 0x40; //format opcode as 0100AAA0, last bit writes
    uint8_t regbyte = (reg << 1) | 0x01; //leftshift and add read bit
    bus.start();
    if (bus.write(opcode) == 1) { //if control byte ack by slave, send next
        bus.start();
        bus.write(regbyte) == 1; //send next control byte
        *buf = bus.read(1); //read reg into buf and send ack
        bus.stop();
        return 0;
    } else return 1;
}

int main()
{
    printf("================\n");
    FADER_IN.set_reference_voltage(5);

    bus.frequency(400000);

    uint8_t readval;
    readReg(0x00, 0x00, &readval);
    printf("%x", readval);
    return 1;

    // while (true) {
    //     if (INT_0) {
    //         printf("INTERRUPT ON CHIP 0\n");
    //     }

    //     if (INT_1) {
    //         printf("INTERRUPT ON CHIP 1\n");
    //     }

    //     if (INT_2) {
    //         printf("INTERRUPT ON CHIP 2\n");
    //     }

    //     if (INT_3) {
    //         printf("INTERRUPT ON CHIP 3\n");
    //     }


    //     printf("\n\n----------------\n\n");
    //     for (uint8_t mux_addr = 0x00; mux_addr < 0x0f; mux_addr++) {
    //         MUX_0 = mux_addr & 0x01;
    //         MUX_1 = mux_addr & 0x02;
    //         MUX_2 = mux_addr & 0x04;
    //         MUX_3 = mux_addr & 0x08;
    //         printf("A: %x, V: %.4f\n", mux_addr, FADER_IN.read_voltage());
    //     }
    // }
}