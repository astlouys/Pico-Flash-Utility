/* ================================================================== *\
   Pico-Flash-Utility.c
   St-Louys Andre - November 2022
   astlouys@gmail.com
   Revision 03-JAN-2023
   Compiler: GNU 7.3.1
   Version 2.00

   Raspberry Pi Pico Utility to work with Pico's flash memory.
   An external monitor (or a PC running a terminal emulator program like TeraTerm)
   must be connected to the USB connector for CDC communication.

   NOTES:
   - This Firmware has been built to run from RAM, so that all FLASH memory space may be erased if needed.
   - Even if it runs from RAM, the firmware is saved to flash in order to be available on next power-up.
     (if user doesn't press the BootSel button).
   - The Firmware will never overwrite the Pico's manufacturing test results saved at 0x1007F000 (length: 107 bytes).
   - If you erase all the flash, the Pico will be seen as a "USB drive" automatically on next power-up.
   - Pico's "Unique number" is taken from the flash memory IC (and is never erased).
   - Flash memory space goes from 0x10000000 to 0x101FFFFF (2MBytes).
   - RAM   memory space goes from 0x20000000 to 0x20041FFF (264KBytes)
  
    - When including required system libraries to support the Pico W, firmware becomes far too big to be run from RAM (around 575 kBypes),
      whereas the firmware without Pico W support code is more like 100 kBytes (which could be run entirelly from RAM address space - 264 kBytes).
      The only noticeable impact of not adding support libraries for Pico W is that Pico W's LED will not blink to indicate that firmware
      has started and is waiting for connection with USB CDC terminal software.
    - Some code for Pico W support has been left in the firmware (but commented out), so that those interested in exploring the Pico W can do it.


   REVISION HISTORY:
   =================
   26-NOV-2022 1.00 - Initial release.
   01-JAN_2023 2.00 - Add flash memory test function.
                    - Some code restructuring and cleanup.
\* ================================================================== */



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                                Include files.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
/// #include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/unique_id.h"
/// #include "pico_w.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                                Definitions.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
typedef unsigned int  UINT;   // processor-optimized.
typedef uint8_t       UINT8;
typedef uint16_t      UINT16;
typedef uint32_t      UINT32;
typedef uint64_t      UINT64;
typedef unsigned char UCHAR;


/// #define RESTORE

#define ADC_VCC  29

#define FLAG_OFF 0x00
#define FLAG_ON  0xFF

/* Current software mode definitions. */
#define MODE_UNDEFINED                0
#define MODE_DISPLAY_TEST_RESULT      1
#define MODE_DISPLAY_SECTOR           2
#define MODE_DISPLAY_COMPLETE_FLASH   3
#define MODE_DISPLAY_COMPLETE_RAM     4
#define MODE_DISPLAY_FUNCTIONS        5
#define MODE_COMPLETE_LOG             6
#define MODE_ERASE_SECTOR             7
#define MODE_ERASE_WHOLE_FLASH        8
#define MODE_BLANK_CHECK              9
#define MODE_FLASH_TEST              10


#define PICO_LED 25  // for Pico only (Pico W's LED must go through cyw43 library).

#define RAM_BASE_ADDRESS 0x20000000

#define TEST_RESULT_SIZE  107  // size of Pico's manufacturing test result in flash memory.
#define TOTAL_CYCLES      5    // total number of write cycles to do for the complete flash test.
#define TYPE_PICO    1
#define TYPE_PICO_W  2

/* GPIO assignations. */
#define UART_TX_PIN  1  // optional serial line to transmit data to   an external VT101-type monitor.
#define UART_RX_PIN  2  // optional serial line to receive  data from an external VT101-type monitor.



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                              Global variables.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* NOTE: XIP_BASE ("eXecute-In-Place") is the base address of the flash memory in Pico's address space (memory map). */
UINT8 *FlashBaseAddress = (UINT8 *)XIP_BASE;  // base address of flash memory.
UINT8 *FlashOldData;                          // pointer to an allocated RAM memory space used for flash operations.
UINT8 *FlashNewData;                          // pointer to an allocated RAM memory space used for flash operations.
UINT8  SoftwareMode;                          // indicate current software mode.
UINT8  WriteCycle = 0;                        // keep track of current memory write cycle.

struct repeating_timer TimerMSec;     // time keeping and overall supervision callback



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                             Function prototypes.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/* Blink Pico's LED the specified number of times. */
void blink_pico_led(UINT8 NumberOfTimes);

/* Display Pico's complete flash address space. */
void display_all_flash(void);

/* Display Pico's complete RAM address space. */
void display_all_ram(void);

/* Erase all flash memory and display complete log for this Raspberry Pi Pico. */
void display_complete_log(void);

/* Display address of functions. They should be in RAM, somewhere between 0x20000000 and 0x20041FFF. */
void display_function_addresses(void);

/* Display Pico's manufacturing test results. */
void display_manufacturing_test(void);

/* Display memory content through external monitor. */
void display_memory(UINT32 BaseAddress, UINT32 StartOffset, UINT32 Length);

/* Determine if the microcontroller is a Pico or a Pico W and display Pico's Unique Number. */
UINT8 display_microcontroller_id(void);

/* Display Pico's specific flash memory sector. */
void display_specific_sector(void);

/* Erase Pico's whole flash address space except Pico's manufacturing test results. */
void erase_all_flash(UINT8 FlagUnattended);

/* Erase a specific sector of Pico's flash memory. */
void erase_specific_sector(void);

/* Check if flash area is blank (0xFF). */
UINT64 flash_blank_check(void);

/* Erase data in Pico flash memory. */
void flash_erase(UINT32 FlashMemoryOffset);

/* Flash memory test. */
void flash_test(void);

/* Write data to Pico's flash memory. */
UINT flash_write(UINT32 FlashMemoryOffset, UINT8 NewData[], UINT16 NewDataSize);

/* Read a single character from stdin (external PC running TeraTerm or other terminal software). */
void input_string(UCHAR *String);

/* One second period callback function. */
bool timer_callback_ms(struct repeating_timer *TimerMSec);

/* Send a string to VT101 monitor through Pico UART. */
void uart_send(UINT16 LineNumber, UCHAR *String);



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- *\
                                                                          Main program entry point.
\* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
int main(void)
{
  UCHAR Menu;
  UCHAR String[256];

  UINT8 Loop1UInt8;
  UINT8 PicoType;

  UINT16 DataInput;
  
  UINT32 Loop1UInt32;
  UINT32 Loop2UInt32;
  UINT32 SectorAddress;
  UINT32 StartAddress;

  uart_inst_t *Uart;  // Pico's UART used to serially transfer data to an external monitor or to a PC.


  /* Initialize UART0 used for bi-directional communication with a PC running TeraTermn (or other) terminal emulation software. */
  stdio_init_all();
  uart_init(uart0, 921600);

  /* Initialize Pico's analog-to-digital (ADC) converter. */
  adc_init();

  /* Used to determine if we are running on a Pico or a Pico W. */
  adc_gpio_init(ADC_VCC);    // power supply voltage.

  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_format(uart0, 8, 1, UART_PARITY_NONE);

  
  /* Initialize gpio for Pico's LED pin. */
  gpio_init(PICO_LED);
  gpio_set_dir(PICO_LED, GPIO_OUT);


  /* Check if we are running on a Pico or Pico W. */
  PicoType = display_microcontroller_id();

 
  /* Initialize gpio for Pico W's LED pin. */
  if (PicoType == TYPE_PICO_W)
  {
    /* See comments at the beginning of the source code above. */
    // cyw43_arch_init();
  }



  /* ---------------------------------------------------------------- *\
               Wait for CDC USB to establish connection.
  \* ---------------------------------------------------------------- */
  while (!stdio_usb_connected())
  {
    if (PicoType == TYPE_PICO)
    {
      blink_pico_led(2);
    }

    sleep_ms(1200);
  }


  /* ---------------------------------------------------------------- *\
           Separate this logging session from the previous one.
  \* ---------------------------------------------------------------- */
  for (Loop1UInt8 = 0; Loop1UInt8 < 30; ++Loop1UInt8)
    printf("\r");



  /* ---------------------------------------------------------------- *\
               Reserve RAM space area for flash operations.
  \* ---------------------------------------------------------------- */
  FlashOldData = (UINT8 *)malloc(FLASH_SECTOR_SIZE);
  FlashNewData = (UINT8 *)malloc(FLASH_SECTOR_SIZE);



  /* Initialize callback function for 100 millisecond timer (to flash current on-going cycle to user during processing). */
  add_repeating_timer_ms(100, timer_callback_ms, NULL, &TimerMSec);

  
  
  /* ================================================================ *\
                 Main program loop. Display menu to user.
  \* ================================================================ */
  while (true)
  {
    display_microcontroller_id();  // display information about Pico.


    /* ---------------------------------------------------------------- *\
              Check if Firmware is run from flash as expected.
    \* ---------------------------------------------------------------- */
    if (((void *)main < (void *)0x20000000) || ((void *)main > (void *)0x20041FFF))
    {
      sprintf(String, "-     <<<<< WARNING >>>>> APPLICATION SHOULD BE RUN FROM RAM, NOT FROM FLASH (%p)!!!\r\r", main);
      uart_send(__LINE__, String);
    }
    // build flag instruction to be executed from the "build" directory (including the double-dots):  
    //     cmake -DPICO_COPY_TO_RAM=1 ..
    // or other build flag instruction to be executed from the "build" directory (including the double-dots):
    //     cmake -DPICO_NO_FLASH=1 ..



    /* ---------------------------------------------------------------- *\
                              Display menu to user.
    \* ---------------------------------------------------------------- */
    printf("                    1) Display Pico's manufacturing test results.\r");
    printf("                    2) Display Pico's flash memory specific sector.\r");
    printf("                    3) Display Pico's complete flash address space.\r");
    printf("                    4) Display Pico's complete RAM address space.\r");
    printf("                    5) Display firmware functions address.\r");
    printf("                    6) Erase all flash and generate Pico's complete log.\r");
    printf("                    7) Erase a specific sector of Pico's flash.\r");
    printf("                    8) Erase Pico's whole flash address space.\r");
    printf("                    9) Flash memory blank check.\r");
    printf("                   10) Flash memory test.\r");
    printf("                   11) Clear screen.\r");
    printf("\r");

    
    printf("                    Enter an option: ");
    input_string(String);
    if (String[0] == 0x0D) continue;
    Menu = atoi(String);
    
    
    switch(Menu)
    {
      case (1):
        /* Display Pico's manufacturing test results. */
        printf("\r\r");
        SoftwareMode = MODE_DISPLAY_TEST_RESULT;
        display_manufacturing_test();
        printf("\r\r");
      break;
      
      case (2):
        /* Display a Pico's flash memory specific sector. */
        printf("\r\r");
        SoftwareMode = MODE_DISPLAY_SECTOR;
        display_specific_sector();
        printf("\r\r");
      break;

      case (3):
        /* Display Pico's complete flash address space. */
        printf("\r\r");
        SoftwareMode = MODE_DISPLAY_COMPLETE_FLASH;
        display_all_flash();
        printf("\r\r");
      break;
      
      case (4):
        /* Display Pico's complete RAM address space. */
        printf("\r\r");
        SoftwareMode = MODE_DISPLAY_COMPLETE_RAM;
        display_all_ram();
        printf("\r\r");
      break;

      case (5):
        /* Display software functions' address. */
        printf("\r\r");
        SoftwareMode = MODE_DISPLAY_FUNCTIONS;
        display_function_addresses();
        printf("\r\r");
      break;

      case (6):
        /* Display Pico's complete log book. */
        printf("\r\r");
        SoftwareMode = MODE_COMPLETE_LOG;
        display_complete_log();
        printf("\r\r");
      break;
    
      case (7):
        /* Erase a specific sector of Pico's flash. */
        printf("\r\r");
        SoftwareMode = MODE_ERASE_SECTOR;
        erase_specific_sector();
        printf("\r\r");
      break;

      case (8):
        /* Erase Pico's whole flash address space, except manufacturing test results. */
        printf("\r\r");
        SoftwareMode = MODE_ERASE_WHOLE_FLASH;
        erase_all_flash(FLAG_OFF);
        printf("\r\r");
      break;

      case (9):
        /* Flash memory blank check. */
        printf("\r\r");
        SoftwareMode = MODE_BLANK_CHECK;
        flash_blank_check();
        printf("\r\r");
      break;

      case (10):
        /* Flash memory test. */
        printf("\r\r");
        SoftwareMode = MODE_FLASH_TEST;
        flash_test();
        printf("\r\r");
      break;

      case (11):
        /* Clear screen. */
        for (Loop1UInt8 = 0; Loop1UInt8 < 80; ++Loop1UInt8)
          printf("\r");
      break;

      default:
        printf("\r\r");
        printf("                    Invalid choice... please re-enter [%s]  [%u]\r\r\r\r\r", String, Menu);
        printf("\r\r");
      break;
    }

    SoftwareMode = MODE_UNDEFINED;
  }
}





/* $PAGE */
/* $TITLE=blink_pico_led() */
/* ------------------------------------------------------------------ *\
            Blink Pico's LED the specified number of times.
\* ------------------------------------------------------------------ */
void blink_pico_led(UINT8 NumberOfTimes)
{
  UINT8 Loop1UInt8;


  for (Loop1UInt8 = 0; Loop1UInt8 < NumberOfTimes; ++Loop1UInt8)
  {
    gpio_put(PICO_LED, true);
    sleep_ms(120);
    gpio_put(PICO_LED, false);
    sleep_ms(300);
  }

  return;
}





/* $PAGE */
/* $TITLE=display_all_flash() */
/* ------------------------------------------------------------------ *\
             Display Pico's complete flash address space.
\* ------------------------------------------------------------------ */
void display_all_flash(void)
{
  UCHAR String[256];

  UINT32 StartOffset;
  UINT32 Length;

  
  StartOffset = 0x00000000;
  Length      = 0x00200000;



  printf("=======================================================================================================\r");
  sprintf(String, "Display Pico's complete flash address space:\r");
  uart_send(__LINE__, String);

  sprintf(String, "XIP_BASE: 0x%p   StartOffset: 0x%8.8X   Length: 0x%8.8X (%u)\r", XIP_BASE, StartOffset, Length, Length);
  uart_send(__LINE__, String);

  sprintf(String, "(Note: Pico's flash memory space goes from 0x10000000 to 0x101FFFFF)\r\r");
  uart_send(__LINE__, String);

  display_memory(XIP_BASE, StartOffset, Length);

  printf("\r");
  sprintf(String, "End of display Pico's complete flash address space.\r");
  uart_send(__LINE__, String);
  printf("=======================================================================================================\r\r\r");

  return;
}





/* $PAGE */
/* $TITLE=display_all_ram() */
/* ------------------------------------------------------------------ *\
               Display Pico's complete RAM address space.
\* ------------------------------------------------------------------ */
void display_all_ram(void)
{
  UCHAR String[256];

  UINT32 StartOffset;
  UINT32 Length;

  
  StartOffset = 0x00000000;
  Length      = 0x00042000;


  printf("=======================================================================================================\r");
  sprintf(String, "Display Pico's complete RAM address space:\r");
  uart_send(__LINE__, String);

  sprintf(String, "RAM base address: 0x%p   StartOffset: 0x%6.6X   Length: 0x%X (%u)\r", RAM_BASE_ADDRESS, StartOffset, Length, Length);
  uart_send(__LINE__, String);

  sprintf(String, "(Note: Pico's RAM memory space goes from 0x20000000 to 0x20041FFF)\r\r");
  uart_send(__LINE__, String);

  display_memory(RAM_BASE_ADDRESS, StartOffset, Length);

  printf("\r");
  sprintf(String, "End of Pico's RAM address space.\r");
  uart_send(__LINE__, String);
  printf("=======================================================================================================\r\r\r");

  return;
}





/* $TITLE=display_complete_log() */
/* ------------------------------------------------------------------- *\
                  Erase all flash memory and display
               complete log for this Raspberry Pi Pico.
\* ------------------------------------------------------------------- */
void display_complete_log(void)
{
  UCHAR String[256];


  printf("                    This will erase Pico's whole flash address space except Pico's manufacturing test results.\r");
  printf("                    Are you sure you want to proceed <Y/N>: ");
  input_string(String);
  if ((strcmp(String, "Y") != 0) && (strcmp(String, "y") != 0))
    return;


  display_manufacturing_test();
  erase_all_flash(FLAG_ON);
  flash_blank_check();
  display_all_flash();
  display_function_addresses();

  return;
}





/* $TITLE=display_function_addresses() */
/* ------------------------------------------------------------------- *\
                      Display addresses of functions.
   They should be in RAM, somewhere between 0x20000000 and 0x20041FFF.
\* ------------------------------------------------------------------- */
void display_function_addresses(void)
{
  UCHAR String[256];


  printf("=======================================================================================================\r");
  sprintf(String, "Display functions' address:\r");
  uart_send(__LINE__, String);
  
  sprintf(String, "FLASH_BASE_ADDRESS: 0x%p         RAM_BASE_ADDRESS: 0x%p\r", XIP_BASE, RAM_BASE_ADDRESS);
  uart_send(__LINE__, String);

  sprintf(String, "(Note: Pico's FLASH memory space goes from 0x10000000 to 0x101FFFFF)\r");
  uart_send(__LINE__, String);

  sprintf(String, "(Note: Pico's  RAM  memory space goes from 0x20000000 to 0x20041FFF)\r\r");
  uart_send(__LINE__, String);

  sprintf(String, "main():                             0x%p\r", main);
  uart_send(__LINE__, String);

  sprintf(String, "display_all_flash():                0x%p\r", display_all_flash);
  uart_send(__LINE__, String);

  sprintf(String, "display_all_ram():                  0x%p\r", display_all_ram);
  uart_send(__LINE__, String);

  sprintf(String, "display_complete_log():             0x%p\r", display_complete_log);
  uart_send(__LINE__, String);

  sprintf(String, "display_function_addresses():       0x%p\r", display_function_addresses);
  uart_send(__LINE__, String);

  sprintf(String, "display_manufacturing_test():       0x%p\r", display_manufacturing_test);
  uart_send(__LINE__, String);

  sprintf(String, "display_memory():                   0x%p\r", display_memory);
  uart_send(__LINE__, String);

  sprintf(String, "display_microcontroller_id():       0x%p\r", display_microcontroller_id);
  uart_send(__LINE__, String);

  sprintf(String, "display_specific_sector():          0x%p\r", display_specific_sector);
  uart_send(__LINE__, String);

  sprintf(String, "erase_all_flash()):                 0x%p\r", erase_all_flash);
  uart_send(__LINE__, String);

  sprintf(String, "erase_specific_sector()):           0x%p\r", erase_specific_sector);
  uart_send(__LINE__, String);

  sprintf(String, "flash_blank_check():                0x%p\r", flash_blank_check);
  uart_send(__LINE__, String);

  sprintf(String, "flash_erase():                      0x%p\r", flash_erase);
  uart_send(__LINE__, String);

  sprintf(String, "flash_test():                       0x%p\r", flash_test);
  uart_send(__LINE__, String);

  sprintf(String, "flash_write():                      0x%p\r", flash_write);
  uart_send(__LINE__, String);

  sprintf(String, "input_string():                     0x%p\r", input_string);
  uart_send(__LINE__, String);

  sprintf(String, "uart_send():                        0x%p\r", uart_send);
  uart_send(__LINE__, String);


  printf("\r");
  sprintf(String, "End of functions' address display.\r");
  uart_send(__LINE__, String);
  printf("========================================================================================================\r\r\r");
  printf("\r\r");

  return;
}





/* $PAGE */
/* $TITLE=display_manufacturing_test() */
/* ------------------------------------------------------------------ *\
               Display Pico's manufacturing test results.
     This data has been saved in Pico's flash at address 0x1007F000
                       and it is 107 bytes long.
\* ------------------------------------------------------------------ */
void display_manufacturing_test(void)
{
  UCHAR String[256];

  UINT8 TestResultSize;

  UINT32 TestResultOffset;


  TestResultOffset = 0x7F000;
  TestResultSize   = TEST_RESULT_SIZE;


  printf("=======================================================================================================\r");
  uart_send(__LINE__, "Display Pico's manufacturing test results:\r");

  sprintf(String, "XIP_BASE: 0x%p   Offset: 0x%6.6X   Length: 0x%X (%u)\r", XIP_BASE, TestResultOffset, TestResultSize, TestResultSize);
  uart_send(__LINE__, String);

  sprintf(String, "(Note: Pico's flash memory space goes from 0x10000000 to 0x101FFFFF)\r\r");
  uart_send(__LINE__, String);

  display_memory(XIP_BASE, TestResultOffset, TestResultSize);

  printf("\r");
  sprintf(String, "End of Pico's manufacturing test results.\r");
  uart_send(__LINE__, String);
  printf("=======================================================================================================\r\r\r");

  return;
}





/* $PAGE */
/* $TITLE=display_memory() */
/* ------------------------------------------------------------------ *\
           Display memory content through external monitor.
\* ------------------------------------------------------------------ */
void display_memory(UINT32 BaseAddress, UINT32 Offset, UINT32 Length)
{
  UCHAR String[256];

  UINT8 *MemoryBaseAddress;

  UINT32 Loop1UInt32;
  UINT32 Loop2UInt32;


  /* Point to target memory address (either RAM of flash). */
  MemoryBaseAddress = (UINT8 *)(BaseAddress);

  for (Loop1UInt32 = Offset; Loop1UInt32 < (Offset + Length); Loop1UInt32 += 16)
  {
    /* Display memory address. */
    sprintf(String, "[%p] ", MemoryBaseAddress + Loop1UInt32);

    /* Display memory content in hex. */
    for (Loop2UInt32 = 0; Loop2UInt32 < 16; ++Loop2UInt32)
    {
      if ((Loop1UInt32 + Loop2UInt32) >= (Offset + Length))
        sprintf(&String[strlen(String)], "   ");
      else
        sprintf(&String[strlen(String)], "%2.2X ", MemoryBaseAddress[Loop1UInt32 + Loop2UInt32]);
    }


    /* Display separator. */
    sprintf(&String[strlen(String)], "| ");


    /* Display memory content in ASCII when displayable characters, or <.> if not displayable. */
    for (Loop2UInt32 = 0; Loop2UInt32 < 16; ++Loop2UInt32)
    {
      if ((Loop1UInt32 + Loop2UInt32) >= (Offset + Length))
      {
        sprintf(&String[strlen(String)], " ");
      }
      else
      {
        if ((MemoryBaseAddress[Loop1UInt32 + Loop2UInt32] >= 0x20) && (MemoryBaseAddress[Loop1UInt32 + Loop2UInt32] <= 0x7E) && (MemoryBaseAddress[Loop1UInt32 + Loop2UInt32] != 0x25))
          sprintf(&String[strlen(String)], "%c", MemoryBaseAddress[Loop1UInt32 + Loop2UInt32]);
        else
          sprintf(&String[strlen(String)], ".");
      }
    }
    
    sprintf(&String[strlen(String)], "\r");  // add linefeed.
    uart_send(__LINE__, String);
  }

  return;
}





/* $PAGE */
/* $TITLE=display_microcontroller_id() */
/* ------------------------------------------------------------------ *\
         Determine if the microcontroller is a Pico or a Pico W
                     and display its Unique Number.
\* ------------------------------------------------------------------ */
UINT8 display_microcontroller_id(void)
{
  UCHAR String[256];

  UINT8 Loop1UInt8;
  UINT8 PicoType;
  
  UINT16 AdcValue1;
  UINT16 AdcValue2;

  float Volts1;
  float Volts2;

  pico_unique_board_id_t board_id;


  printf("=======================================================================================================\r");
  printf("                                          Pico-Flash-Utility\r");


  /* Select power supply input. */
  adc_select_input(3);
  gpio_put(PICO_LED, true);

  /* Read ADC converter raw value. */
  AdcValue1 = adc_read();

  /* Convert raw value to voltage value. */
  Volts1 = (3 * (AdcValue1 * 3.3f / 4096));



  /* The important power supply value to consider is when GPIO25 is Low. */
  gpio_put(PICO_LED, false);
    
  /* Read ADC converter raw value. */
  AdcValue2 = adc_read();

  /* Convert raw value to voltage value. */
  Volts2 = (3 * (AdcValue2 * 3.3f / 4096));
  if (Volts2 > 3)
  {
    printf("                                Microcontroller is a Raspberry Pi Pico\r");
    PicoType = TYPE_PICO;
  }
  else
  {
    printf("                               Microcontroller is a Raspberry Pi Pico W\r");
    PicoType = TYPE_PICO_W;
  }



  /* Retrieve Pico Unique Number from flash memory IC. */
  pico_get_unique_board_id(&board_id);
    
  /* Build-up the Unique ID string in hex. */
  sprintf(String, "                                     Pico ID: ");
  for (Loop1UInt8 = 0; Loop1UInt8 < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++Loop1UInt8)
  {
    sprintf(&String[strlen(String)], "%2.2X", board_id.id[Loop1UInt8]);
    if ((Loop1UInt8 % 2) && (Loop1UInt8 != 7))
      sprintf(&String[strlen(String)], " ");
  }
  sprintf(&String[strlen(String)], "\r");
  printf(String);
  
  // printf("                                  AdcValue with PICO_LED true: %4.4u\r", AdcValue1);
  // printf("            Voltage is between 1.8 and 5 volts on a Pico but near 0 volt on a Pico W: %2.2f\r", Volts1);
  
  // printf("                                  AdcValue with PICO_LED false: %4.4u\r", AdcValue2);
  // printf("            Voltage is between 1.8 and 5 volts on a Pico but near 0 volt on a Pico W: %2.2f\r", Volts2);

  printf("=======================================================================================================\r\r\r");

  return PicoType;
}





/* $PAGE */
/* $TITLE=display_specific_sector() */
/* ------------------------------------------------------------------ *\
              Display Pico's specific flash memory sector.
\* ------------------------------------------------------------------ */
void display_specific_sector(void)
{
  UCHAR String[256];

  UINT32 SectorOffset;


  /* Request sector to be displayed. */
  printf("                    Enter sector offset in hex (0x000000 to 0x1FFFFF): ");
  input_string(String);
  SectorOffset = strtol(String, NULL, 16);
  printf("\r\r");
  
  
  /* Validate sector number entered. */
  while (SectorOffset > 0x1FFFFF)
  {
    printf("                    Invalid sector offset entered...[0x%5.5X]\r", SectorOffset);
    printf("                    Sector offset must be a value in hexadecimal between 0 and 1FFFFF\r");
    printf("                    Enter sector offset (or <Enter> to return to menu): ");
    input_string(String);
    if (String[0] == 0x0D) break;  // if user pressed <Enter> only.
    SectorOffset = strtol(String, NULL, 16);
  }
  if (String[0] == 0x0D) return;  // if user pressed <Enter> only.

  
  /* Make sure display begins on a sector boundary. */
  while ((SectorOffset % FLASH_SECTOR_SIZE) != 0)
    --SectorOffset;

  
  printf("=======================================================================================================\r");
  sprintf(String, "Display Pico's flash memory sector:\r");
  uart_send(__LINE__, String);
  
  sprintf(String, "XIP_BASE: 0x%p   Offset: 0x%6.6X   Length: 0x%X (%u)\r", XIP_BASE, SectorOffset, FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  uart_send(__LINE__, String);

  sprintf(String, "(Note: Pico's flash memory space goes from 0x10000000 to 0x101FFFFF)\r\r");
  uart_send(__LINE__, String);

  display_memory(XIP_BASE, SectorOffset, FLASH_SECTOR_SIZE);

  printf("\r");
  sprintf(String, "End of flash specific sector display.\r");
  uart_send(__LINE__, String);
  printf("=======================================================================================================\r\r\r");

  return;
}





/* $PAGE */
/* $TITLE=erase_all_flash() */
/* ------------------------------------------------------------------ *\
                 Erase Pico's whole flash address space
                except Pico's manufacturing test results.
\* ------------------------------------------------------------------ */
void erase_all_flash(UINT8 FlagUnattended)
{
  UCHAR String[256];

  UINT8 Loop1UInt8;

  UINT32 Loop1UInt32;
  UINT32 StartOffset;
  UINT32 EndOffset;
  
  
  /* Initializations. */
  StartOffset = 0x00000000;
  EndOffset   = 0x001FFFFF;


  if (FlagUnattended == FLAG_OFF)
  {
    printf("                    This will erase Pico's whole flash address space except Pico's manufacturing test results.\r");
    printf("                    Are you sure you want to proceed <Y/N>: ");
    input_string(String);
    if ((strcmp(String, "Y") != 0) && (strcmp(String, "y") != 0))
      return;
  }


  if (((void *)main < (void *)0x20000000) || ((void *)main > (void *)0x20041FFF))
  {
    sprintf(String, "<<<<< FATAL >>>>> YOU CAN'T ERASE FLASH MEMORY WHILE YOU RUN THE APPLICATION FROM FLASH.\r");
    uart_send(__LINE__, String);

    sprintf(String, "                  THIS WOULD BE PROGRAM SELF-DESTRUCTION !!!\r\r\r");
    uart_send(__LINE__, String);

    return;
  }


  printf("=======================================================================================================\r");
  sprintf(String, "Erase Pico's whole flash address space.\r");
  uart_send(__LINE__, String);
  
  sprintf(String, "XIP_BASE: 0x%p   StartOffset: 0x%8.8X   EndOffset: 0x%8.8X (%u)\r\r", XIP_BASE, StartOffset, EndOffset, EndOffset);
  uart_send(__LINE__, String);

  printf("Erasing sectors...\r");
  for (Loop1UInt32 = StartOffset; Loop1UInt32 < EndOffset; Loop1UInt32 += 0x1000)
  {
    printf("0x%8.8X   ", Loop1UInt32);
    if (((Loop1UInt32 + 0x1000) % 0x8000) == 0) printf("\r");
    flash_erase(Loop1UInt32);
  }


  printf("\r");
  sprintf(String, "End erasing whole flash address space.\r");
  uart_send(__LINE__, String);
  printf("=======================================================================================================\r\r\r");
  printf("\r\r");

  return;
}





/* $PAGE */
/* $TITLE=erase_specific_sector() */
/* ------------------------------------------------------------------ *\
               Erase a specific sector of Pico's flash.
\* ------------------------------------------------------------------ */
void erase_specific_sector(void)
{
  UCHAR String[256];

  UINT32 SectorOffset;

  
  printf("                    Enter offset of the sector to erase in hex (or <Enter> to return to menu): ");
  input_string(String);
  SectorOffset = strtol(String, NULL, 16);
  printf("\r\r");

  while (SectorOffset % 4096)
  {
    printf("                    Invalid sector offset entered...[0x%5.5X]\r", SectorOffset);
    printf("                    Sector offset must be aligned on a sector boundary (0x3000, 0xB000, 0x7A000, 1EC000)\r");
    printf("                    Enter offset of the sector to erase in hex (or <Enter> to return to menu): ");
    input_string(String);
    if (String[0] == 0x0D) break;  // if user pressed <Enter> only.
    SectorOffset = strtol(String, NULL, 16);
  }
  if (String[0] == 0x0D) return;  // if user pressed <Enter> only.

  
  printf("=======================================================================================================\r");
  sprintf(String, "Erase Pico's specific flash memory sector at offset 0x%8.8X\r", SectorOffset);
  uart_send(__LINE__, String);
  
  sprintf(String, "XIP_BASE: 0x%p   Offset: 0x%6.6X   Length: 0x%X (%u)\r", XIP_BASE, SectorOffset, FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  uart_send(__LINE__, String);

  sprintf(String, "(Note: Pico's flash memory space goes from 0x10000000 to 0x101FFFFF)\r\r");
  uart_send(__LINE__, String);

  display_memory(XIP_BASE, SectorOffset, FLASH_SECTOR_SIZE);
  
  printf("\r\r");
  printf("                    Are you sure you want to erase this sector <Y/N>: ");
  input_string(String);
  if ((strcmp(String, "Y") != 0) && (strcmp(String, "y") != 0))
    return;

  printf("\r\r");
  printf("=======================================================================================================\r");
  sprintf(String, "Erase a specific sector of Pico's flash at offset 0x%8.8X.\r", SectorOffset);
  uart_send(__LINE__, String);
  
  sprintf(String, "XIP_BASE: 0x%p   Offset: 0x%X  Length: 0x%X (%u)\r", XIP_BASE, SectorOffset, FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
  uart_send(__LINE__, String);

  flash_erase(SectorOffset);
  
  printf("\r");
  sprintf(String, "End of erase specific flash sector.\r");
  uart_send(__LINE__, String);
  printf("=======================================================================================================\r\r\r");

  return;
}





/* $PAGE */
/* $TITLE=flash_blank_check() */
/* ------------------------------------------------------------------ *\
                 Check if flash area is blank (0xFF).
\* ------------------------------------------------------------------ */
UINT64 flash_blank_check(void)
{
  UCHAR String[256];

  UINT8  FlagDirt;
  UINT8  FlagSkipLine;
  UINT8  FlagStarted;

  UINT32 StartOffset;
  UINT32 EndOffset;
  UINT32 Loop1UInt32;
  UINT32 Loop2UInt32;

  UINT64 TotalErrors;


  /* Initializations. */
  StartOffset = 0x00000000;
  EndOffset   = 0x001FFFFF;
  FlagStarted = FLAG_OFF;
  TotalErrors = 0;


  /***
  // This block would allow for a restricted flash blank test, but I didn't see a great utility for it.
  // Since this function may be called from the option 6 "macro", asking for a start address from user would require
  // a small change to remain unattended.
  printf("                    Enter start offset for flash blank check (in hex): ");
  input_string(String);
  StartOffset = strtol(String, NULL, 16);

  // Validate sector number entered.
  while (StartOffset > 0x1FFFFF)
  {
    printf("                    Invalid start offset entered...[0x%8.8X]\r", StartOffset);
    printf("                    Start offset must be an hexadecimal value between 000000 and 1FFFFF\r");
    printf("                    Enter start offset (or press <Enter> to return to menu): ");
    input_string(String);
    if (String[0] == 0x0D) break;  // if user pressed <Enter> only.
    StartOffset = strtol(String, NULL, 16);
  }
  if (String[0] == 0x0D) break;  // if user pressed <Enter> only.
  printf("\r\r");
  ***/


  printf("======================================================================================================\r");
  uart_send(__LINE__, "Pico's flash blank check.\r");
   
  sprintf(String, "XIP_BASE: 0x%p   StartOffset: 0x%8.8X   EndOffset: 0x%8.8X\r\r", XIP_BASE, StartOffset, EndOffset);
  uart_send(__LINE__, String);

  
  FlagSkipLine = FLAG_OFF;
  for (Loop1UInt32 = StartOffset; Loop1UInt32 < EndOffset; Loop1UInt32 += 16)
  {
    /* Check if this range is blank. */
    FlagDirt = FLAG_OFF;
    for (Loop2UInt32 = 0; Loop2UInt32 < 16; ++Loop2UInt32)
    {
      if (FlashBaseAddress[Loop1UInt32 + Loop2UInt32] != 0xFF)
      {
        ++TotalErrors;
        FlagDirt     = FLAG_ON;
        FlagSkipLine = FLAG_OFF;
      }
    }
      
    if (FlagDirt == FLAG_OFF)
    {
      if (FlagSkipLine == FLAG_OFF)
      {
        FlagSkipLine = FLAG_ON;
        
        /* Prevent line feed on first pass. */
        if (FlagStarted == FLAG_ON) printf("\r");
      }
    }
    else
    {
      FlagSkipLine = FLAG_OFF;
    }


    /* Range is not blank, display it. */
    if (FlagDirt)
    {
      /* Display start address. */
      sprintf(String, " [%p] ", XIP_BASE + Loop1UInt32);

      /* Display data in hex. */
      for (Loop2UInt32 = 0; Loop2UInt32 < 16; ++Loop2UInt32)
        sprintf(&String[strlen(String)], "%2.2X ", FlashBaseAddress[Loop1UInt32 + Loop2UInt32]);


      /* Display separator. */
      sprintf(&String[strlen(String)], "| ");


      /* Display data in ASCII if displayable character, or <.> if not displayable. */
      for (Loop2UInt32 = 0; Loop2UInt32 < 16; ++Loop2UInt32)
      {
        if ((FlashBaseAddress[Loop1UInt32 + Loop2UInt32] >= 0x20) && (FlashBaseAddress[Loop1UInt32 + Loop2UInt32] <= 0x7E) && (FlashBaseAddress[Loop1UInt32 + Loop2UInt32] != 0x25))
        {
          sprintf(&String[strlen(String)], "%c", FlashBaseAddress[Loop1UInt32 + Loop2UInt32]);
        }
        else
        {
          sprintf(&String[strlen(String)], ".");
        }
      }
      /* Add a final linefeed. */
      sprintf(&String[strlen(String)], "\r");
      uart_send(__LINE__, String);
    }

    FlagStarted = FLAG_ON;
  }

  printf("\r");
  sprintf(String, "End of Pico's flash blank check from offset 0x%8.8X to offset 0x%8.8X\r", StartOffset, EndOffset);
  uart_send(__LINE__, String);
  sprintf(String, "Total errors found: %llu (0x%X) (see documentation)\r", TotalErrors, TotalErrors);
  uart_send(__LINE__, String);
  printf("========================================================================================================\r\r\r");

  return TotalErrors;
}





/* $PAGE */
/* $TITLE=flash_erase() */
/* ------------------------------------------------------------------ *\
                  Erase data in Pico's flash memory.
     NOTES:
     - The way the electronics is done, one sector of the flash
       (4096 bytes) must be erased at a time.
     - This function has been kept simple and one sector (4096 bytes)
       will be erased, beginning at the specified offset, which must
       be aligned on a sector boundary (4096).
\* ------------------------------------------------------------------ */
void flash_erase(UINT32 FlashMemoryOffset)
{
  UCHAR String[256];

  UINT16 Loop1UInt16;

  UINT32 InterruptMask;


  /* Erase a sector of Pico's flash memory.
     FlashMemoryOffset is the offset in flash memory (relative to XIP_BASE) and must be aligned on a sector boundary (a multiple of 4096). */
  if (FlashMemoryOffset % FLASH_SECTOR_SIZE)
  {
    sprintf(String, "Offset specified for flash_erase(0x%8.8X) is not aligned on a sector boundary (multiple of 4096)\r", FlashMemoryOffset);
    uart_send(__LINE__, String);

    while (FlashMemoryOffset % FLASH_SECTOR_SIZE)
      ++FlashMemoryOffset;

    sprintf(String, "Offset has been shifted to 0x%8.8X\r", FlashMemoryOffset);
    uart_send(__LINE__, String);
  }


  /* Special handling of sector 0x7F000 containing Pico's manufacturing test results.
     If flash_erase() is called for sector 0x7F000, it is converted to a flash_write() with all 0xFF.
     flash_write() will take care of keeping Pico's manufacturing test results unchanged. */
  if (FlashMemoryOffset == 0x7F000)
  {
    /* Set all bytes to be written to flash equal to 0xFF. */
    memset(FlashNewData, 0xFF, FLASH_SECTOR_SIZE);
    
    /* Write this sector to flash. (NOTE: flash_write() will take care of keeping Pico's manufacturing test results unchanged). */
    flash_write(0x7F000, FlashNewData, FLASH_SECTOR_SIZE);
  }
  else
  {
    /* Keep track of interrupt mask on entry. */
    InterruptMask = save_and_disable_interrupts();

    /* Erase flash area. */
    flash_range_erase(FlashMemoryOffset, FLASH_SECTOR_SIZE);

    /* Restore original interrupt mask when done. */
    restore_interrupts(InterruptMask);
  }

  return;
}





/* $PAGE */
/* $TITLE=flash_test() */
/* ------------------------------------------------------------------ *\
                 Perform a test of all flash memory.
\* ------------------------------------------------------------------ */
void flash_test(void)
{
  UCHAR  Dum1Str[2];
  UCHAR  String[256];
  
  UINT8  Loop2UInt8;

  UINT16 Loop1UInt16;

  UINT32 Loop1UInt32;
  UINT32 SectorOffset;

  UINT64 EndOffset;
  UINT64 StartOffset;
  UINT64 TotalErrors;
  

  /* ----------------------------------------------------- *\
                Present instructions to user.
  \* ----------------------------------------------------- */
  printf("=======================================================================================================\r");
  uart_send(__LINE__, "Pico's flash memory test will perform five (5) complete memory write cycles\r");
  uart_send(__LINE__, "For each cycle, the following actions will be executed:\r\r")   ;
  uart_send(__LINE__, " 1) Flash memory will be erased.\r");
  uart_send(__LINE__, " 2) A blank check will be done.\r");
  uart_send(__LINE__, " 3) Bytes 0x00 will be written to all flash memory space.\r");
  uart_send(__LINE__, " 4) Flash will be read back to check if all memory content is 0x00.\r\r");
  
  uart_send(__LINE__, " 5) Flash memory will be erased.\r");
  uart_send(__LINE__, " 6) A blank check will be done.\r");
  uart_send(__LINE__, " 7) Bytes 0x55 will be written to all flash memory space.\r");
  uart_send(__LINE__, " 8) Flash will be read back to check if all memory content is 0x55.\r\r");
  
  uart_send(__LINE__, " 9) Flash memory will be erased.\r");
  uart_send(__LINE__, "10) A blank check will be done.\r");
  uart_send(__LINE__, "11) Bytes 0xAA will be written to all flash memory space.\r");
  uart_send(__LINE__, "12) Flash will be read back to check if all memory content is 0xAA.\r\r");
  
  uart_send(__LINE__, "13) Flash memory will be erased.\r");
  uart_send(__LINE__, "14) A blank check will be done.\r");
  uart_send(__LINE__, "15) Consecutive bytes 0x55 and 0xAA will be written to all flash memory space.\r");
  uart_send(__LINE__, "16) Flash will be read back to check if all memory content is 0x55 and 0xAA.\r\r");
  
  uart_send(__LINE__, "17) Flash memory will be erased.\r");
  uart_send(__LINE__, "18) A blank check will be done.\r");
  uart_send(__LINE__, "19) Consecutive bytes 0xAA and 0x55 will be written to all flash memory space.\r");
  uart_send(__LINE__, "20) Flash will be read back to check if all memory content is 0xAA and 0x55.\r\r");
  
  uart_send(__LINE__, "If any error is detected during the process, it will be reported.\r");
  uart_send(__LINE__, "Also, data will be saved to log file at each step of the process.\r\r");
  uart_send(__LINE__, "NOTE: Write to Pico's flash memory is limited to more or less 100,000 cycles.\r");
  uart_send(__LINE__, "      Moreover, no wear leveling algorithm has been implemented in the Pico.\r");
  uart_send(__LINE__, "      You may use this utility as required, but you should not modify it and\r");
  uart_send(__LINE__, "      use it as a <burn-in> test and let it run for hours...\r\r");

  uart_send(__LINE__, "NOTE: The process takes more or less 60 minutes to complete. You may want to\r");
  uart_send(__LINE__, "      let the system go and come back later...\r\r");

  uart_send(__LINE__, "As can be seen from the steps above, for each memory write cycle, five <flash erase> will be done,\r");
  uart_send(__LINE__, "along with five <flash write>. Since the Pico's manufacturing test result will not be overwritten\r");
  uart_send(__LINE__, "by the Pico-Flash-Utility, 107 byte errors will be added and cumulated at every step.\r");
  sprintf(String, "So, %u errors for <flash erase> and %u errors for <flash write> = 214 errors.\r", TEST_RESULT_SIZE, TEST_RESULT_SIZE, (TEST_RESULT_SIZE * 2));
  uart_send(__LINE__, String);
  sprintf(String, "Then, %u errors for each pattern written to flash, times 5 patterns = %u errors. \r", (TEST_RESULT_SIZE * 2), (TEST_RESULT_SIZE * 2 * 5));
  uart_send(__LINE__, String);
  sprintf(String, "Finally, %u errors for each write cycle, times %u write cycles = %u errors.\r", (TEST_RESULT_SIZE * 2 * 5), TOTAL_CYCLES, (TEST_RESULT_SIZE * 2 * 5 * TOTAL_CYCLES));
  uart_send(__LINE__, String);
  uart_send(__LINE__, "Consequently, it is normal to have 5350 errors reported at the end of the procedure.\r\r");

  uart_send(__LINE__, "Are you sure you want to proceed <Y/N>: ");
  input_string(String);
  if ((strcmp(String, "Y") != 0) && (strcmp(String, "y") != 0))
    return;



  /* ----------------------------------------------------- *\
                        Initializations.
  \* ----------------------------------------------------- */
  TotalErrors = 0;
  StartOffset = 0x00000000;
  EndOffset   = 0x001FFFFF;


  /***
  printf("\r\r");
  printf("=======================================================================================================\r");
  sprintf(String, "XIP_BASE: 0x%p   StartOffset: 0x%8.8llX   EndOffset: 0x%8.8llX\r\r", XIP_BASE, StartOffset, EndOffset);
  uart_send(__LINE__, String);
  ***/


  
  /* ----------------------------------------------------- *\
                Loop for <TOTAL_CYCLES> cycles.
  \* ----------------------------------------------------- */
  /* Perform <TOTAL_CYCLES> complete cycles. */
  for (WriteCycle = 0; WriteCycle < TOTAL_CYCLES; ++WriteCycle)
  {
    printf("\r\r\r\r\r= = = = = = = = = = = = = = = = = = = = = = = = CYCLE %u = = = = = = = = = = = = = = = = = = = = = = = =\r", WriteCycle + 1);
    /* For each cycle, write 5 different patterns. */
    for (Loop2UInt8 = 0; Loop2UInt8 < 5; ++Loop2UInt8)
    {
      /* ----------------------------------------------------- *\
                    Erase whole flash memory space.
      \* ----------------------------------------------------- */
      // uart_send(__LINE__, "Erasing flash...\r");
      erase_all_flash(FLAG_ON);
      
      // uart_send(__LINE__, "Flash blank check...\r");
      TotalErrors += flash_blank_check();

      /* ----------------------------------------------------- *\
                      Write data to flash memory.
      \* ----------------------------------------------------- */
      printf("========================================================================================================\r");

      /* For each cycle, write 5 different patterns. */
      switch (Loop2UInt8)
      {
        case (0):
          uart_send(__LINE__, "Writing 0x00 to all flash memory.\r");
          Dum1Str[0] = 0x00;
          Dum1Str[1] = 0x00;
        break;

        case (1):
          uart_send(__LINE__, "Writing 0x55 to all flash memory.\r");
          Dum1Str[0] = 0x55;
          Dum1Str[1] = 0x55;
        break;

        case (2):
          uart_send(__LINE__, "Writing 0xAA to all flash memory.\r");
          Dum1Str[0] = 0xAA;
          Dum1Str[1] = 0xAA;
        break;

        case (3):
          uart_send(__LINE__, "Writing 0x55 / 0xAA to all flash memory.\r");
          Dum1Str[0] = 0x55;
          Dum1Str[1] = 0xAA;
        break;

        case (4):
          uart_send(__LINE__, "Writing 0xAA / 0x55 to all flash memory.\r");
          Dum1Str[0] = 0xAA;
          Dum1Str[1] = 0x55;
        break;
      }

      uart_send(__LINE__, "Please wait...\r");

      /* Initialize data to be saved. */
      for (Loop1UInt16 = 0; Loop1UInt16 < FLASH_SECTOR_SIZE; Loop1UInt16 += 2)
      {
        FlashNewData[Loop1UInt16]     = Dum1Str[0];
        FlashNewData[Loop1UInt16 + 1] = Dum1Str[1];
      }


      /*** Display data to be written to flash. ***
      printf("Data to be written to flash:\r");
      display_memory(RAM_BASE_ADDRESS, (UINT32)(FlashNewData - (UINT8 *)0x20000000), FLASH_SECTOR_SIZE);
      ***/


      /* Overwrite all flash sectors with new data. */
      for (SectorOffset = StartOffset; SectorOffset < EndOffset; SectorOffset += 0x1000)
      {
        flash_write(SectorOffset, FlashNewData, FLASH_SECTOR_SIZE);

        /* If we wrote to sector containing "Pico's manufacturing test results", data has been partly overwritten. */
        if (SectorOffset == 0x7F000)
        {
          for (Loop1UInt16 = 0; Loop1UInt16 < TEST_RESULT_SIZE; Loop1UInt16 += 2)
          {
            FlashNewData[Loop1UInt16]     = Dum1Str[0];
            FlashNewData[Loop1UInt16 + 1] = Dum1Str[1];
          }
        }
      }
  
      uart_send(__LINE__, "Done writing to all flash memory.\r");
      printf("========================================================================================================\r\r\r");


      /* ----------------------------------------------------- *\
             Display whole flash address space to log file.
      \* ----------------------------------------------------- */
      /* When all flash has been written, take a snapshot of it. */
      display_all_flash();



      /* ----------------------------------------------------- *\
               Check all flash memory space for a match.
      \* ----------------------------------------------------- */
      printf("========================================================================================================\r");
      uart_send(__LINE__, "Check all flash memory for a match with data written.\r\r");

      /* Check every flash byte to confirm write has been successful. */
      for (Loop1UInt32 = 0; Loop1UInt32 < EndOffset; Loop1UInt32 += 2)
      {
        /* Check those 2 bytes. */
        if (FlashBaseAddress[Loop1UInt32] != Dum1Str[0])
        {
          sprintf(String, "Offset: 0x%8.8X   Data read: 0x%2.2X instead of 0x%2.2X\r", Loop1UInt32, FlashBaseAddress[Loop1UInt32], Dum1Str[0]);
          uart_send(__LINE__, String);
          ++TotalErrors;
        }

        if (FlashBaseAddress[Loop1UInt32 + 1] != Dum1Str[1])
        {
          sprintf(String, "Offset: 0x%8.8X   Data read: 0x%2.2X instead of 0x%2.2X\r", Loop1UInt32 + 1, FlashBaseAddress[Loop1UInt32 + 1], Dum1Str[1]);
          uart_send(__LINE__, String);
          ++TotalErrors;
        }
      }
      uart_send(__LINE__, "\r");
      
      sprintf(String, "Total errors found so far: %u\r", TotalErrors);
      uart_send(__LINE__, String);

      sprintf(String, "As mentionned in the documentation, %u errors until now is normal because\r", (WriteCycle * 1070) + (TEST_RESULT_SIZE * ((Loop2UInt8 + 1) * 2)));
      uart_send(__LINE__, String);

      uart_send(__LINE__, "Pico's manufacturing test result has been preserved.\r");

      printf("========================================================================================================\r\r\r");
    }
  }
  
  
  
  /* ----------------------------------------------------- *\
                      Final flash erase
          to leave flash memory space clear when done.
  \* ----------------------------------------------------- */
  erase_all_flash(FLAG_ON);



  /* ----------------------------------------------------- *\
          Display final report for flash memory test.
  \* ----------------------------------------------------- */
  printf("========================================================================================================\r");
  sprintf(String, "Flash memory test final report after %u write cycles.\r\r", WriteCycle);
  uart_send(__LINE__, String);

  sprintf(String, "Total errors found: %u\r", TotalErrors);
  uart_send(__LINE__, String);
  
  sprintf(String, "NOTE: %u errors is normal  since Pico's manufacturing results is not overwritten.\r", (((TEST_RESULT_SIZE * 2) * 5) * TOTAL_CYCLES));
  uart_send(__LINE__, String);

  sprintf(String, "      %u bytes for erase, %u bytes for write pattern = %U errors for each pattern written\r", TEST_RESULT_SIZE, TEST_RESULT_SIZE, (TEST_RESULT_SIZE * 2));
  uart_send(__LINE__, String);

  sprintf(String, "      %u for each pattern X 5 patterns = %u errors per write cycle (00, 55, AA, 55+AA, AA+55)\r", (TEST_RESULT_SIZE * 2), (TEST_RESULT_SIZE * 2 * 5));
  uart_send(__LINE__, String);

  sprintf(String, "      %u for each write cycle X %u cycles = %u total errors for the whole process.\r\r", (TEST_RESULT_SIZE * 2 * 5), TOTAL_CYCLES, (TEST_RESULT_SIZE * 2 * 5 * TOTAL_CYCLES));
  uart_send(__LINE__, String);

  sprintf(String, "End of flash memory test\r");
  uart_send(__LINE__, String);
  printf("========================================================================================================\r\r\r");

  return;
}





/* $PAGE */
/* $TITLE=flash_write() */
/* ------------------------------------------------------------------ *\
                    Write data to Pico's flash memory.
                    To keep things simple in the code, 
                always one sector is updated in the flash.
\* ------------------------------------------------------------------ */
UINT flash_write(UINT32 FlashMemoryOffset, UINT8 *Data, UINT16 DataSize)
{
  UCHAR Archive[TEST_RESULT_SIZE];
  UCHAR String[256];

  UINT16 Loop1UInt16;

  UINT32 SectorOffset;
  UINT32 InterruptMask;


  /* Initializations. */
  SectorOffset      = FlashMemoryOffset;  // assume offset specified is on a sector boundary.
  FlashMemoryOffset = 0;                  // assume that new data will be at the beginning of sector.


  /***
  uart_send(__LINE__, "\r");

  sprintf(String, "Entering flash_write()     SectorOffset from XIP_BASE where to write: 0x%8.8X\r", SectorOffset);
  uart_send(__LINE__, String);

  sprintf(String, "Data address: 0x%8.8X   Data size:    0x%4.4X (%u)\r", Data, DataSize, DataSize);
  uart_send(__LINE__, String);

  sprintf(String, "FlashNewData: 0x%p   FlashOldData: 0x%p\r", FlashNewData, FlashOldData);
  uart_send(__LINE__, String);
  ***/


  if (SectorOffset % FLASH_SECTOR_SIZE)
  {
    /* Offset specified is not aligned on a sector boundary. */
    sprintf(String, "FlashMemoryOffset specified (0x%8.8X) is not aligned on a sector boundary (multiple of 4096)\r", SectorOffset);
    uart_send(__LINE__, String);

    sprintf(String, "Phased out by %u (0x%X) bytes.\r", SectorOffset % FLASH_SECTOR_SIZE, SectorOffset % FLASH_SECTOR_SIZE);
    uart_send(__LINE__, String);

    while (SectorOffset % FLASH_SECTOR_SIZE)
    {
      --SectorOffset;
      ++FlashMemoryOffset;
    }

    sprintf(String, "Sector offset has been shifted down to 0x%8.8X...\r", SectorOffset);
    uart_send(__LINE__, String);

    sprintf(String, "...and data offset has been shifted up to 0x%X (%u) from sector start.\r", FlashMemoryOffset, FlashMemoryOffset);
    uart_send(__LINE__, String);
  }


  if (FlashMemoryOffset + DataSize > 4096)
  {
    uart_send(__LINE__, "The arguments given cross a sector boundary which is not allowed...\r");

    sprintf(String, "Sector offset: %X   FlashMemoryOffset: %X  Data size: %X\r\r\r", SectorOffset, FlashMemoryOffset, DataSize);
    uart_send(__LINE__, String);

    return 1;
  }


  /* A wear leveling algorithm has not been implemented.
     Consequently, flash_write() must not be used for intensive data logging without adding a wear leveling algorithm. */
  /* Take a copy of current flash content. */
  for (Loop1UInt16 = 0; Loop1UInt16 < FLASH_SECTOR_SIZE; ++Loop1UInt16)
    FlashOldData[Loop1UInt16] = FlashBaseAddress[SectorOffset + Loop1UInt16];

  
  /***
  printf("=======================================================================================================\r");
  uart_send(__LINE__, "Display original data retrieved from flash:\r");

  sprintf(String, "FlashBaseAddress: 0x%p   FlashOldData: %p   SectorOffset: 0x%4.4X\r\r", FlashBaseAddress, FlashOldData, SectorOffset);
  uart_send(__LINE__, String);

  display_memory(RAM_BASE_ADDRESS, (UINT32)(FlashOldData - RAM_BASE_ADDRESS), FLASH_SECTOR_SIZE);
  
  uart_send(__LINE__, "\r");
  uart_send(__LINE__, "End of display original data retrieved from flash.\r");
  printf("=======================================================================================================\r\r\r");
  ***/


  /* Special handling of sector 0x7F000 containing Pico's manufacturing test results. */
  if (SectorOffset == 0x7F000)
  {
    #ifdef RESTORE
    #include "restore1.c"
    #else
    /* Keep track of Pico's manufacturing test results. */
    for (Loop1UInt16 = 0; Loop1UInt16 < TEST_RESULT_SIZE; ++Loop1UInt16)
      Archive[Loop1UInt16] = FlashBaseAddress[SectorOffset + Loop1UInt16];
    #endif

    /***
    printf("=======================================================================================================\r");
    sprintf(String, "Pico's manufacturing test results read back from flash:\r\r");
    uart_send(__LINE__, String);
    
    display_memory(RAM_BASE_ADDRESS, (UINT32)(Archive - (UINT8 *)0x20000000), TEST_RESULT_SIZE);
    
    printf("\r");
    sprintf(String, "End of Pico's manufacturing test results read back.\r");
    uart_send(__LINE__, String);
    printf("=======================================================================================================\r\r\r");
    ***/
  }


  /* Overwrite target memory area with new data. 
     NOTE: FlashNewData contains data to be written, passed in argument.
           and FlashOldData contains original data (one sector) read from flash. */
  memcpy(&FlashOldData[FlashMemoryOffset], Data, DataSize);


  /* Special handling of sector 0x7F000 containing Pico's manufacturing test results. */
  if (SectorOffset == 0x7F000)
  {
    /* Keep track of Pico's manufacturing test results. */
    for (Loop1UInt16 = 0; Loop1UInt16 < TEST_RESULT_SIZE; ++Loop1UInt16)
      FlashOldData[FlashMemoryOffset + Loop1UInt16] = Archive[Loop1UInt16];
  }

  
  /***
  printf("=======================================================================================================\r");
  sprintf(String, "Data to be written to sector 0x%8.8X at offset 0x%4.4X:\r", SectorOffset, FlashMemoryOffset);
  uart_send(__LINE__, String);
  
  sprintf(String, "Data: 0x%p   DataSize: 0x%4.4X (%u)\r", Data, DataSize, DataSize);
  uart_send(__LINE__, String);

  sprintf(String, "FlashBaseAddress: 0x%p\r\r", FlashBaseAddress);
  uart_send(__LINE__, String);

  display_memory(RAM_BASE_ADDRESS, (UINT32)(FlashOldData - RAM_BASE_ADDRESS), FLASH_SECTOR_SIZE);
  ***/


  /* Keep track of interrupt mask and disable interrupts during flash writing. */
  InterruptMask = save_and_disable_interrupts();

  /* Erase flash before reprogramming. */
  flash_range_erase(SectorOffset, FLASH_SECTOR_SIZE);
  
  /* Save data to flash memory. */
  flash_range_program(SectorOffset, FlashOldData, FLASH_SECTOR_SIZE);

  /* Restore original interrupt mask when done. */
  restore_interrupts(InterruptMask);


  /***
  printf("\r");
  sprintf(String, "End of flash_write(%u, %u)\r", FlashMemoryOffset, DataSize);
  uart_send(__LINE__, String);
  printf("========================================================================================================\r\r\r");
  ***/

  return 0;
}





/* $PAGE */
/* $TITLE=input_string() */
/* ------------------------------------------------------------------ *\
                       Read a string from stdin.
\* ------------------------------------------------------------------ */
void input_string(UCHAR *String)
{
  int8_t DataInput;

  UINT8 Loop1UInt8;


  Loop1UInt8 = 0;
  do
  {
    DataInput = getchar_timeout_us(50000);

    switch (DataInput)
    {
      case (PICO_ERROR_TIMEOUT):
      case (0):
        continue;
      break;

      case (8):
        /* <Backspace> */
        if (Loop1UInt8 > 0)
        {
          --Loop1UInt8;
          String[Loop1UInt8] = 0x00;
          printf("%c %c", 0x08, 0x08);  // erase character under the cursor.
        }
      break;

      case (0x0D):
        /* <Enter> */
        if (Loop1UInt8 == 0)
        {
          String[Loop1UInt8++] = (UCHAR)DataInput;  
          String[Loop1UInt8++] = 0x00;
        }
        printf("\r");
      break;

      default:
        printf("%c", (UCHAR)DataInput);
        String[Loop1UInt8] = (UCHAR)DataInput;
        // printf("Loop1UInt8: %3u   %2.2X - %c\r", Loop1UInt8, DataInput, DataInput);  ///
        ++Loop1UInt8;
      break;
    }
  } while((Loop1UInt8 < 128) && (DataInput != 0x0D));
  
  String[Loop1UInt8] = '\0';  // end-of-string

  /***
  for (Loop1UInt8 = 0; Loop1UInt8 < 10; ++Loop1UInt8)
    printf("%2u:[%2.2X]   ", Loop1UInt8, String[Loop1UInt8]);
  printf("\r");
  ***/

  return;
}





/* $PAGE */
/* $TITLE=timer_callback_s() */
/* ------------------------------------------------------------------ *\
                Hundred millisecond callback function.
     Callback function used to blink Pico's LED to indicate to user
              what is the current on-going write cycle.
\* ------------------------------------------------------------------ */
bool timer_callback_ms(struct repeating_timer *TimerMSec)
{
  static UINT16 MSecCounter;
  static UINT16 CycleCount;
  static UINT8 FlagCurrentBlink;
  static UINT8 FlagLedStatus;


  /* Display current cycle only if we are running a flash test. */
  if (SoftwareMode != MODE_FLASH_TEST) return true;

  ++MSecCounter;  // one more 100 milliseconds period elapsed.

  /* Blink current cycle every 15 seconds. */
  if (FlagCurrentBlink == FLAG_OFF)
  {
    if (MSecCounter >= (15 * 10))
    {
      FlagCurrentBlink = FLAG_ON;
      MSecCounter      = 0;  // reset 100 milliseconds counter.
    }
    else
    {
      return true;
    }
  }


  /* There is a blink in progress. */
  if (FlagLedStatus == FLAG_ON)
  {
    /* LED On for 200 milliseconds. */
    if (MSecCounter >= 2)
    {
      /* Turn Off LED. */
      gpio_put(PICO_LED, false);
      FlagLedStatus = FLAG_OFF;
      MSecCounter   = 0;  // reset 100 milliseconds counter.
      
      /* If we reached total number of blinks required, stop it. */
      if (CycleCount >= (WriteCycle + 1))
      {
        MSecCounter      = 0;         // reset 15 seconds counter for next blink.
        CycleCount       = 0;         // get prepared for next cycle number to blink in 15 seconds.
        FlagCurrentBlink = FLAG_OFF;  // no more blink in progress.
      }
    }
  }
  else
  {
    /* LED Off for 300 milliseconds. */
    if (MSecCounter >= 3)
    {
      /* LED is Off, turn it On. */
      gpio_put(PICO_LED, true);
      FlagLedStatus = FLAG_ON;
      MSecCounter   = 0;  // reset 15 seconds counter for next blink.
      ++CycleCount;
    }
  }

  return true;
}





/* $PAGE */
/* $TITLE=uart_send() */
/* ------------------------------------------------------------------ *\
    Send a string to external monitor through Pico UART (or USB CDC).
\* ------------------------------------------------------------------ */
void uart_send(UINT16 LineNumber, UCHAR *String)
{
  UCHAR LineString[512];

  
  /* Initializations. */
  LineString[0] = 0x00;  // end-of-string.


  /* Trap special control code for <home>. Replace "home" by appropriate escape sequence for "home" on a VT101. */
  if (strcmp(String, "home") == 0)
  {
    String[0] = 0x1B; // ESC code
    String[1] = '[';
    String[2] = 'H';
    String[3] = 0x00;
  }

  /* Trap special control code for <cls>. Replace "cls" by appropriate escape sequence for "clear screen" on a VT101. */
  if (strcmp(String, "cls") == 0)
  {
    /* For some reason, "cls" has side effects on the behaviour of uart_send() for future logging and should be avoided. */
    String[0] = 0x1B;
    String[1] = '[';
    String[2] = '2';
    String[3] = 'J';
    String[4] = 0x00;
  }

  /* "Timer stamp" will not be printed if first character is '-',
     or if first character is a line feed '\r' when we simply want to add line spacing in the debug log,
     or if the string represents a tag for an an escape sequence (for example 'Home' or "Clear screen'). */
  if ((String[0] != '-') && (String[0] != '\r') && (String[0] != 0x1B) && (String[0] != '|'))
  {
    /* Send line number through UART. */
    sprintf(LineString, "[%7lu] ", LineNumber);

    /* Send current timer value to external terminal. */
    sprintf(&LineString[strlen(LineString)], "[%10lu] ", time_us_32());
  }

  /* Send log string through UART. */
  sprintf(&LineString[strlen(LineString)], String);
  printf(LineString);

  return;
}
