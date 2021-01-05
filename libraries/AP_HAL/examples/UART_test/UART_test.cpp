/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}


void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

    setup_uart(hal.uartA, "uartA");  // console
    setup_uart(hal.uartB, "uartB");  // 1st GPS
    setup_uart(hal.uartC, "uartC");  // Telem1, FTDI
    setup_uart(hal.uartD, "uartD");  // Telem2, FM_ECU1
    setup_uart(hal.uartE, "uartE");  // UART & I2C B, FM_ECU2
}

static void test_uart(AP_HAL::UARTDriver *uart1, AP_HAL::UARTDriver *uart2, AP_HAL::UARTDriver *uart3, const char *name)
{    
    if (uart1 == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    if (uart2 == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    if (uart3 == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    int16_t nb1 = uart2->available();
    hal.scheduler->delay(20);
    uart1->printf("%d data ready to read\n\r", nb1);
    //int16_t nb2 = uart3->available();
    //hal.scheduler->delay(20);
    int16_t dataM1[511];
    //int16_t dataM2[48];
    
    //uart1->printf("%d data ready to read", nb);
    //while( nb > 0){
      for(int i=0;i<511;i++){
		  //uart1->printf("%d data ready to read\n\r", nb1);
          //uart1->printf("dataM1[%d] = %d\n\r", i, dataM1[i]);
          //uart1->printf("%d data ready to read\n\r", nb2);
		  dataM1[i]=uart2->read();
		  //dataM2[i]=uart3->read();
		  if(dataM1[i]== 0xA5 && dataM1[i+47] == 0x0D && dataM1[i+1]== 0x5A && dataM1[i+2]== 0x01 && dataM1[i+3]== 0x00)
		  { 
		    uart1->printf("Moteur 1, Temperature 1: %d à la position %d \t", dataM1[i+5]+ (dataM1[i+4]<<8), i+4);
		   // uart1->printf("Moteur 2, Temperature 1: %d à la position %d \n\r", dataM2[i+4]+ (dataM2[i+5]<<8), i+4);
		    uart1->printf("Moteur 1, Temperature 2: %d à la position %d \t", dataM1[i+7]+ (dataM1[i+6]<<8), i+6);
		    //uart1->printf("Moteur 2, Temperature 2: %d à la position %d \n\r", dataM2[i+6]+ (dataM2[i+7]<<8), i+6);
		    uart1->printf("Moteur 1, RPM: %d à la position %d \t", dataM1[i+19]+ (dataM1[i+18]<<8), i+18);
		    //uart1->printf("Moteur 2, RPM: %d à la position %d \n\r", dataM2[i+18]+ (dataM2[i+19]<<8), i+18);
		    uart1->printf("Moteur 1, Input voltage: %d à la position %d \t", dataM1[i+21]+ (dataM1[i+20]<<8), i+20);
		    //uart1->printf("Moteur 2, Input voltage: %d à la position %d \n\r", dataM2[i+20]+ (dataM2[i+21]<<8), i+20);
		    uart1->printf("Moteur 1: Servo voltage: %d à la position %d \t", dataM1[i+23]+ (dataM1[i+22]<<8), i+22);
		    //uart1->printf("Moteur 2, Servo voltage: %d à la position %d \n\r", dataM2[i+22]+ (dataM2[i+23]<<8), i+22);
		    uart1->printf("Moteur 1, Throttle position: %d à la position %d \t", dataM1[i+27]+ (dataM1[i+26]<<8), i+26);
		    //uart1->printf("Moteur 2, Throttle position: %d à la position %d \n\r", dataM2[i+26]+ (dataM2[i+27]<<8), i+26);
		    uart1->printf("Moteur 1, Fuel pressure: %d à la position %d \t", dataM1[i+33]+ (dataM1[i+32]<<8), i+16);
		    //uart1->printf("Moteur 2, Fuel pressure: %d à la position %d \n\r", dataM2[i+32]+ (dataM2[i+33]<<8), i+16);
		    uart1->printf("Moteur 1, Fuel consumption : %d à la position %d \t", dataM1[i+35]+ (dataM1[i+34]<<8), i+16);
		    ///uart1->printf("Moteur 2, Fuel consumption: %d à la position %d \n\r", dataM2[i+34]+ (dataM2[i+35]<<8), i+16);
		    uart1->printf("Moteur 1: Injection length: %d à la position %d \t", dataM1[i+45]+ (dataM1[i+44]<<8), i+44);
		    //uart1->printf("Moteur 2: Injection length: %d à la position %d \n\r", dataM2[i+44]+ (dataM2[i+45]<<8), i+44);

		} 
	  }
    //}   
}

void loop(void)
{
    //test_uart(hal.uartA, "uartA");
    //test_uart(hal.uartB, "uartB");
    test_uart(hal.uartD, hal.uartE, hal.uartE, "uartD");
    //test_uart(hal.uartD, "uartD");
    //test_uart(hal.uartE, hal.uartC, "uartE");

    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));
#endif

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
