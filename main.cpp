#include "mbed.h"
#include "MCP23017.h"

DigitalOut MUX_0(p21);
DigitalOut MUX_1(p22);
DigitalOut MUX_2(p23);
DigitalOut MUX_3(p24);

DigitalIn INT_0(p13);
DigitalIn INT_1(p14);
DigitalIn INT_2(p25);
DigitalIn INT_3(p26);
AnalogIn FADER_IN(p20);

MCP23017 i2c(p28, p27, 0x20);


int main()
{
    printf("================\n");
    FADER_IN.set_reference_voltage(5);
    while (true) {
        if (INT_0) {
            printf("INTERRUPT ON CHIP 0\n");
        }

        if (INT_1) {
            printf("INTERRUPT ON CHIP 1\n");
        }

        if (INT_2) {
            printf("INTERRUPT ON CHIP 2\n");
        }

        if (INT_3) {
            printf("INTERRUPT ON CHIP 3\n");
        }


        printf("\n\n----------------\n\n");
        for (uint8_t mux_addr = 0x00; mux_addr < 0x0f; mux_addr++) {
            MUX_0 = mux_addr & 0x01;
            MUX_1 = mux_addr & 0x02;
            MUX_2 = mux_addr & 0x04;
            MUX_3 = mux_addr & 0x08;
            printf("A: %x, V: %.4f\n", mux_addr, FADER_IN.read_voltage());
        }
    }
}