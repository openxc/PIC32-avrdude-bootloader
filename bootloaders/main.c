// *** main.c ******************************************************************
//
// this is the main program that is launched by crt0.S; it just
// initializes all of the modules of the os and then runs the main
// application program loop.
//
// This file originated from the cpustick.com skeleton project from
// http://www.cpustick.com/downloads.htm and was originally written
// by Rich Testardi; please preserve this reference and share bug
// fixes with rich@testardi.com.

#define PUT_CONFIG_BITS_HERE
#include "main.h"

#ifdef _BOARD_CROSSCHASM_CELLULAR_C5_
typedef enum {
    None,
    Working,
    Success,
    Failed
}RETVAL;

bool telit_initialize();
__attribute__((nomips16)) RETVAL telit_restorebackup();
RETVAL telit_makebackup();
bool telit_initialized = false;
bool FlashProgramming = false;
RETVAL hexloader();
uint16 crc16Reflected(unsigned char* data, unsigned int len);
unsigned int UpgradeFromModem(unsigned char* data, unsigned int len);
bool telit_socketstatus();
bool telit_readsocket(unsigned char** data, unsigned int* len);
bool telit_setbaud115200();
RETVAL otaloader();
#endif

// the stk500v2 state machine states
// see: http://www.atmel.com/dyn/resources/prod_documents/doc2591.pdf
enum {
    STATE_START,
    STATE_GETSEQ,
    STATE_GETMS1,
    STATE_GETMS2,
    STATE_GETTOK,
    STATE_GETDATA,
    STATE_GETCSUM
};

// the stk500v2 constants
#define CMD_SIGN_ON                         0x01
#define CMD_SET_PARAMETER                   0x02
#define CMD_GET_PARAMETER                   0x03
#define CMD_LOAD_ADDRESS                    0x06
#define CMD_ENTER_PROGMODE_ISP              0x10
#define CMD_LEAVE_PROGMODE_ISP              0x11
#define CMD_CHIP_ERASE_ISP                  0x12
#define CMD_PROGRAM_FLASH_ISP               0x13
#define CMD_READ_FLASH_ISP                  0x14
#define CMD_SPI_MULTI                       0x1D

#define STATUS_CMD_OK                       0x00

#define SIGNATURE_BYTES 0x504943

#define OTA_LOADER_CHECK_WAIT_MS            500

// indicates stk500v2 protocol is active
static volatile bool active;  // bootloader is active
static volatile bool fLoaded = false;  // bootloader has loaded

static uint32 tLoopTime;
static uint32 tLoopStart;
static uint32 tLastBlink;

static FNUSERAPP UserApp = (FNUSERAPP) USER_APP_ADDR;
static bool fLoadProgramFromFlash = false;

// stuff about the bootloader written into the image header
static uint32 bootloaderVer             = BOOTLOADERVER;
static uint32 prodAndVend               =  PROD << 16 | VEND;
static uint32 bootloaderCapabilities    = CAPABILITIES;
static RAM_HEADER_INFO ramHeader;

// indicates flash has been erased
static bool erased;

// stk500v2 request state
static int state = STATE_START;
static byte seq;
static int size;
static byte csum;

// stk500v2 request message
static bool ready;          // request has been received and is ready to process
static int requesti;        // number of request bytes
static byte request[1024];  // request buffer
#define REQUEST_OFFSET  2   // shift request buffer so flash data at byte offset 10 is 32-bit aligned

// stk500v2 reply message
static int replyi;          // number of reply bytes
static byte reply[1024];    // reply buffer

// Just-In-Time-Flash variables.

// Unfortunately FLASH_BYTES is calculated at run time, so we have to just
// an array that is bigger than any part. For now we allow for 2MB or Flash
// static byte rgPageMap[FLASH_BYTES/FLASH_PAGE_SIZE];
static byte pageMap[FLASH_BYTES/FLASH_PAGE_SIZE];
static uint32 addrBase = FLASH_START;
static uint32 avrdudeAddrBase = FLASH_START;
static uint32 cbSkipRam = ((uint32) &_RAM_SKIP_SIZE);

int main()  // we're called directly by Crt0.S
{
    RETVAL otaloader_status = None;

    ASSERT(sizeof(byte) == 1);
    ASSERT(sizeof(uint16) == 2);
    ASSERT(sizeof(uint32) == 4);

    // make sure the modem is ON immediately in case we need to upgrade
    MODEM_ON();

    ramHeader.rcon = RCON;
    InitLEDsAndButtons();

    // sometimes there is a debugger circuit on the board that needs to intialize and will
    // pull the reset line after 6 seconds or so and abruptly abort the loaded applicaiton.
    // this pause is put in for a Power On Reset only and waits for the debug circuit to come up
    // and apply the external reset which this call will ignore.
    WaitForFinalReset();

#ifdef _BOARD_CROSSCHASM_CELLULAR_C5_
    // run ota loader
    switch(otaloader())
    {
        case None:
        case Working:
            break;
        case Success:
        case Failed:
        default:
            break;
    }
#endif

    // Determine if there is anything loaded in flash
    fLoadProgramFromFlash = fIsUserAppLoadedInFlash;

    // we have 3 conditions

    // 1. The program button tells use to wait indefinitely (don't load from flash) for an upload
    fLoadProgramFromFlash &= !fLoadFromAVRDudeViaProgramButton;

    // 2. The virtual program button tells us to wait indefinitly for an upload
    //      but only if this is not from a real reset of the processor and must come as a software reset.
    fLoadProgramFromFlash &= !(fLoadFromAVRDudeViaVirtualProgramButton && !RCONbits.POR && !RCONbits.EXTR && RCONbits.SWR);

    // 3. Otherwise we will either load the program immediately, or wait our timeout time to load
    // this will happen in the for loop below

	// Always clear out RCON after 'using' it's values, so it doesn't screw things up next time
    // This must occur after the WaitForFinalReset() and checking our program buttons states
    ClearVirtualProgramButton();
	RCON = 0;

    // If we are just going to immediately load from flash
    // don't even init the UART or USB, just load the application
    if (fLoadProgramFromFlash && LISTEN_BEFORE_LOAD == 0)
	{
        // launch the application!
        ExecuteApp();
    }

    tLoopStart = _CP0_GET_COUNT();
    tLastBlink = tLoopStart;

    // at this point we know that we either are going to wait
    // for something to be download, or are going to wait indefinitly for
    // for a download, in any case we need to enable the the interface for the download
    InitStk500v2Interface();
    
    // forever...
    for (;;) {
        tLoopTime = _CP0_GET_COUNT();

        // This is the listening heartbeat LED.
        // we blink at twice the speed when downloading, thus the divide by 2 shift with active
        // In reality, when downloading starts the boot LED is erratic; look at the download LED
        // to see if you are downloading.
        if ((tLoopTime - tLastBlink) >= ((CORE_TIMER_TICKS_PER_MILLISECOND  * 250) >> active)) {

            // blink the heartbeat LED
            BootLED_Toggle();

            // set up for the next blink
            tLastBlink = tLoopTime;
        }

        // See if we should jump to the application in flash
        // If we just loaded an application via the Stk500v2Interface, we know there is an applicaiton
        // in flash and we can jump right to it now.
        if( fLoaded ||

            // If a program button is used, fLoadProgramFromFlash will be set false
            // as a program button will instruct to either immediately load from flash
            // and that will happen before this while loop, or wait forever for a program to load
            // and we will jump to the applicaiton only once fLoaded is true
            // This is also be false if there was never an application loaded in flash
            // Also when we are downloading and app, this is set false to block jumping to the application until fLoaded is true
            (fLoadProgramFromFlash &&

            // This is to support the auto-reset feature.
            // After a reset we listen for an stk500v2 download request.
            // However if a new application isn't loaded for LISTEN_BEFORE_LOAD milliseconds
            // then just jump to the loaded application in flash. If there is no application in
            // flash, fLoadProgramFromFlash will be false and we wait until something is downloaded
            // via stk500v2 and fLoaded becomes true.
           ((tLoopTime - tLoopStart) >= (LISTEN_BEFORE_LOAD * CORE_TIMER_TICKS_PER_MILLISECOND))
          )) {
            // launch the application!
            ExecuteApp();
        }

        // There are no interrupts, so we must poll
        // the stk500v2 interface to see if a character came in.
        // This may be from the UART or USB input (depending on what is being used).
        stk500v2_isr();

        // if we've received an stk500v2 request...
        // that is, something come in and we need to process it
        if (ready) {
            // process it
            avrbl_message(request+REQUEST_OFFSET, requesti);
            ready = false;
        }
    }

    ASSERT(0);  // stop!
    return 0;
}

// *** avrbl.c *****************************************************************
//
// this file implements the main program loop of the PIC32 USB CDC/ACM
// avrdude bootloader that lives entirely in bootflash
//
// This file originated from the cpustick.com skeleton project from
// http://www.cpustick.com/downloads.htm and was originally written
// by Rich Testardi; please preserve this reference and share bug
// fixes with rich@testardi.com.
//
// KeithV (Digilent) 2/21/2012 Made code configurable by including BoardConfig.h
//                              Removed all references to plib.h C-runtime libarary

// this function handles the stk500v2 message protocol state machine
void avrbl_state_machine(byte b)
{
    csum ^= b;

    switch (state) {
        case STATE_START:
            if (b == 27) {
                state = STATE_GETSEQ;
            }
            csum = b;
            break;
        case STATE_GETSEQ:
            seq = b;
            state = STATE_GETMS1;
            break;
        case STATE_GETMS1:
            size = b<<8;
            state = STATE_GETMS2;
            break;
        case STATE_GETMS2:
            size |= b;
            state = STATE_GETTOK;
            break;
        case STATE_GETTOK:
            if (b == 14) {
                requesti = 0;
                state = STATE_GETDATA;
            } else {
                state = STATE_START;
            }
            break;
        case STATE_GETDATA:
            request[REQUEST_OFFSET+requesti++] = b;
            if (requesti == size) {
                state = STATE_GETCSUM;
            }
            break;
        case STATE_GETCSUM:
            if (csum) {
                ASSERT(0);
            } else {
                ready = true;
            }
            state = STATE_START;
            break;
        default:
            ASSERT(0);
            break;
    }
}

// this function sends bytes to the CDC/ACM port
static void
avrbl_print(const byte *buffer, int length)
{
    stk500v2_print(buffer, length);
}

// this function handle an stk500v2 message
static void
avrbl_message(byte *request, int size)
{
    static uint32 load_address;  // load address for stk500v2 flash read/write operations
    static byte parameters[256];  // track stk500v2 parameters (we ignore them all)
	static bool fGetBaseAddress = true;

    uint32 i;
    uint32 nbytes;
    uint32 nbytesAligned;
    uint32 endAddr;
    uint32 address;
    int rawi;
    byte raw[64];
    uint32 temp;

    ASSERT(! replyi);

    // our reply message always starts with the message and status bytes
    reply[replyi++] = *request;
    reply[replyi++] = STATUS_CMD_OK;

    // process the request message and generate additional reply message bytes
    switch (*request) {
        case CMD_SIGN_ON:

            // this will block us from loading from flash if our auto-reset timeout occures while we are actually
            // in the process of downloading a new applicaiton. We wil have to wait until fLoaded is true before
            // we will load the application.
            fLoadProgramFromFlash = false;

            active = true;
            erased = false;
            reply[replyi++] = 8;
            ilstrcpy(reply+replyi, "STK500_2");
            replyi += 8;
            DownloadLED_On();
            break;
        case CMD_SET_PARAMETER:
            parameters[request[1]] = request[2];
            break;
        case CMD_GET_PARAMETER:
            reply[replyi++] = parameters[request[1]];
            break;
        case CMD_ENTER_PROGMODE_ISP:
            break;
        case CMD_SPI_MULTI:
            reply[replyi++] = 0;
            reply[replyi++] = request[4];
            reply[replyi++] = 0;
            if (request[4] == 0x30) {
                if (request[6] == 0) {
                    reply[replyi++] = (byte)(SIGNATURE_BYTES>>16);
                } else if ( request[6] == 1 ) {
                    reply[replyi++] = (byte)(SIGNATURE_BYTES>>8);
                } else {
                    reply[replyi++] = (byte)SIGNATURE_BYTES;
                }

           } else if ((request[4] == 0x20) || (request[4] == 0x28)) {

/* this is never called, but lets just lie and say 0xFF
                //* read one byte from flash
                //* 0x20 is read odd byte
                //* 0x28 is read even byte

                //* read the even address
                address = (request[5]<<8)|(request[6]);
                //* the address is in 16 bit words
                address = address<<1;

                if (request[4] == 0x20) {
                    reply[replyi++] = *(uint16 *)(FLASH_START+address);
                } else {
                    reply[replyi++] = (*(uint16 *)(FLASH_START+address))>>8;
                }
*/
				reply[replyi++] = 0xFF;
            } else {
                reply[replyi++] = 0;
            }
            reply[replyi++] = STATUS_CMD_OK;
            break;
        case CMD_CHIP_ERASE_ISP:
			// removed so we can get the base address on the program flash
            //flash_erase_pages((void *)FLASH_START, FLASH_BYTES/FLASH_PAGE_SIZE);
            //erased = true;
            break;
        case CMD_LOAD_ADDRESS:
            load_address = (request[1]<<24)|(request[2]<<16)|(request[3]<<8)|(request[4]);
            //* the address is in 16 bit words
            load_address = load_address<<1;
            ASSERT((load_address&3)==0);
			load_address += avrdudeAddrBase;	// lets get our virtual address
            break;
        case CMD_PROGRAM_FLASH_ISP:

            // start of our buffer needs to be DWORD aligned
            ASSERT(((uintptr)(request+10)&3)==0);

            // according to the spec, the message body can not exceed 275 bytes
            // our buffer is 1024, more than big enough
            // if we have an odd number of bytes, we need to round up and make it DWORD aligned
            // The last thing in this message is data, so we can just put 0xFFs at the end
            // and round up to a word alignment.
            nbytes = ((request[1])<<8)|(request[2]);
            nbytesAligned = (nbytes + 3) & (~(0x3));

            // just put 0xFF at the end of the buffer until we are DWORD aligned
            for(i=nbytes; i<nbytesAligned; i++)
            {
                request[10+i] = 0xFF;  // append some 0xFFs
            }

			if(fGetBaseAddress)
			{
				// here we must know how old and new ones look
				// old images all started at offest 0x180, so the first
				// avrdude load address was 0x100 - 0x1FF
				// new images all start at offset 0xF8, so avrdude will
				// send us the block at 0x000 - 0xFF
				if(nbytesAligned >= 0x100 && (load_address & 0x1FF) == 0x000)		// must not be offset 0x100 - 0x1FF
				{
					uint32 addrBaseT = *((uint32 *)(request+10+offsetBaseAddrInfo));

					// stupid avrdude only knows about 64K of memory and doesn't really know
					// about anything above that, so the upper 16bits are masked. We use the upper
					// 16 bits as our base address to add on. Fortunately, avrdude can program more than 64K.
					if(addrBaseT != ALLF)
					{
						load_address -= avrdudeAddrBase;			// back out the assigned base
						avrdudeAddrBase = addrBaseT & 0xFFFF0000;	// use the new base address
						load_address += avrdudeAddrBase;			// apply the new base address
						addrBase = addrBaseT;
					}
				}
				fGetBaseAddress = false;
			}

            // erase the pages we are about to write to if needed.
            justInTimeFlashErase(load_address, load_address + nbytesAligned)

            // program the words
            ASSERT((load_address & 3) == 0);    // this will assert if we got off DWORD alignment
            flashWriteUint32(load_address, (uint32 *)(request+10), nbytesAligned/4);
            load_address += nbytes;             // we tell the truth even if we are not DWORD aligned
            break;                              // this will cause an assert if we do not get a new load address the next time

        case CMD_READ_FLASH_ISP:

            endAddr = load_address + ((request[1])<<8)|(request[2]);

			// do this page by page as we might have to lie to avrdude.
			while(load_address < endAddr)
			{
				uint32 addrEndPage = nextFlashPage(load_address);
				byte lieMask = wasPageErased(load_address) ? 0 : 0xFF;

				// don't go beyond what was requested; might be before the end of the page
				if(addrEndPage >= endAddr)
				{
					addrEndPage = endAddr;
				}

				// load the data and apply lie.
				while(load_address < addrEndPage)
				{
					reply[replyi++] = *((byte *)(load_address)) | lieMask;
					load_address++;
				}
			}

            reply[replyi++] = STATUS_CMD_OK;
            break;
        case CMD_LEAVE_PROGMODE_ISP:
            finshFlashProcessingAfterLoad();
            fLoaded = true;
            DownloadLED_Off();
            break;
        default:
            ASSERT(0);
            break;
    }

    // send our reply header
    rawi = 0;
    raw[rawi++] = 27;
    raw[rawi++] = seq;
    raw[rawi++] = replyi>>8;
    raw[rawi++] = replyi;
    raw[rawi++] = 14;
    csum = 0;
    for (i = 0; i < rawi; i++) {
        csum ^= raw[i];
    }
    avrbl_print(raw, rawi);

    // send the reply message bytes
    for (i = 0; i < replyi; i++) {
        csum ^= reply[i];
    }
    avrbl_print(reply, replyi);

    // send the reply checksum
    avrbl_print(&csum, 1);

    replyi = 0;
}

/***    void ExecuteApp(void)
**
**    Synopsis:
**      Jumps to the application
*
**    Parameters:
**      None
**
**    Return Values:
**      None
**
**    Errors:
**      None
**
**  Notes:
**
**		If there is a header, it will jump to the location specified by the header
**		Remeber that UserApp is set by default to the jump address assigned in BoardConfig.h
*/
static void ExecuteApp(void)
{
  IMAGE_HEADER_INFO *   pHeaderInfo;

    UninitStk500v2Interface();
    UninitLEDsAndButtons();

	// We are about to jump to the application
	// but lets first check to see if the header info gave me a
	// special jump location
	// see if we have a header
	if((pHeaderInfo = getHeaderStructure(addrBase)) != NULL)
    {
        // if we just programmed, and we are told to jump to the first sketch in flash
        // Don't do this on a simple reset start.
        if(fLoaded && (pHeaderInfo->imageType & imageExecutionJumpToFirstInFlash) == imageExecutionJumpToFirstInFlash)
        {
            if((pHeaderInfo = getHeaderStructure(FLASH_START)) != NULL)
            {
       		    // Set the jump location
    		    UserApp = pHeaderInfo->pJumpAddr;
            }
        }
        else
        {
      		// Set the jump location
    		UserApp = pHeaderInfo->pJumpAddr;
        }

        // now load the RAM HEADER DATA
        // check to see if we have header info? We use the cbHeader as a version number
        if( pHeaderInfo->cbHeader >= OFFSETOF(IMAGE_HEADER_INFO,cbBlPreservedRam) &&
            // is the header in our perserved space? Can we at least save the number of bytes written to the header?
            ((uint32) pHeaderInfo->pRamHeader) <= (((uint32) &_skip_ram_space_end_adder) - sizeof(uint32)) )
        {
            uint32 cb = MIN(pHeaderInfo->cbRamHeader, sizeof(RAM_HEADER_INFO)); // only copy what we the bootloader and sketch both know

            // and make sure we don't walk on our (the bootloaders) own memory
            cb = MIN(cb, (((uint32) &_skip_ram_space_end_adder) - ((uint32) pHeaderInfo->pRamHeader)));

            // store how much the bootloader is going to write.
            ramHeader.cbBlRamHeader = cb;

            // copy, ensuring not to whack the bootloader's own memory, or the sketches persistent data
            ilmemcpy(pHeaderInfo->pRamHeader, &ramHeader, cb);
        }
	}


    // jump to the sketch
    // by default, USER_APP_ADDR as defined in BoardConfig.h is used.
	UserApp();
}

/***    static HEADER_INFO * getHeaderStructure(uint32 imageBaseAddr)
**
**    Synopsis:
**      See if we have a header and gets a pointer to it
*
**    Parameters:
**      imageBaseAddr the base address of the image, like FLASH_START
**
**    Return Values:
**      A pointer to the header or NULL if one does not exist
**
**    Errors:
**      None
**
**  Notes:
**
**
*/
static IMAGE_HEADER_INFO * getHeaderStructure(uint32 imageBaseAddr)
{
    IMAGE_HEADER_INFO *     pHeaderInfo;
  	
	uint32 			        addr 		= imageBaseAddr + offsetHeaderInfo;
    uint32 			        addrHigh 	= FLASH_START + FLASH_BYTES;

	// see if we even wrote the header address in the image
    if(addr+sizeof(uint32) <= addrHigh)
    {
        addr = *((uint32 *) addr);		// dereference and get the header address

		// now lets see if it was an old, pre-header image; if so it will have 0xFFFFFFFF (ALLF) there.
        if( addr != ALLF && ((addr % 4) == 0) && addr+sizeof(IMAGE_HEADER_INFO) <= addrHigh)
        {
            pHeaderInfo = (IMAGE_HEADER_INFO *) addr;		// it is a header, set our header struct to it.

            if(pHeaderInfo->pProgramFlash == imageBaseAddr)
            {
				// all looks good, so we have the header.
				return(pHeaderInfo);
            }
        }
    }

	return(NULL);
}

/***    void eraseFlashViaHeaderInstructions(void)
**
**    Synopsis:
**      When done loading a program, clear the rest of flash as defined by the header info
*
**    Parameters:
**      None
**
**    Return Values:
**      None
**
**    Errors:
**      None
**
**  Notes:
**
**		If no header then all of flash is cleared like the original bootloader did
*/
static void finshFlashProcessingAfterLoad(void)
{
  	IMAGE_HEADER_INFO *   pHeaderInfo;

	// Historically we use to clear all of flash except 4K of EEProm, so set the flash limits
	// to what they use to be, if we have no header this is what will be done.
    uint32 			addrLow		= FLASH_START;
    uint32 			addrHigh 	= FLASH_START + FLASH_BYTES - DEFAULT_EEPROM_SIZE;

	// see if we have a header
	if((pHeaderInfo = getHeaderStructure(addrBase)) != NULL)
    {
        // This is stuff that needs to be put in the header
        flashWriteUint32((uint32) &pHeaderInfo->verBootloader, &bootloaderVer, 1);
        flashWriteUint32((uint32) &pHeaderInfo->vend, &prodAndVend , 1);
        flashWriteUint32((uint32) &pHeaderInfo->bootloaderCapabilities, &bootloaderCapabilities, 1);
        flashWriteUint32((uint32) &pHeaderInfo->cbBlPreservedRam, &cbSkipRam, 1);

		// if we are instructed to only erase pages touched, we are done.
		if((pHeaderInfo->imageType & imageJustInTimeFlashErase) == imageJustInTimeFlashErase)
		{
			return;	// nothing more to do
		}
		// if we are asked to erase the range of pages in the header, erase to those limits
		else if((pHeaderInfo->imageType & imageLinkerSpecifiedFlashErase) == imageLinkerSpecifiedFlashErase)
		{
		            	addrLow 	= pHeaderInfo->pProgramFlash;
		            	addrHigh 	= addrLow + pHeaderInfo->cbProgramFlash;
		}

        // if we are instructed to erase all of flash, do everything, including eeprom
		else if((pHeaderInfo->imageType & imageFullFlashErase) == imageFullFlashErase)
		{
		            	addrHigh 	= FLASH_START + FLASH_BYTES;
		}

        // and if none of the above, we will erase all but the last 4K reserved for EEProm
        // as we did in the past.
 	}

    // Cleared any pages that have not been cleared to the requested limits
	// by default his will be all of flash if we did not have a header.
    justInTimeFlashErase(addrLow, addrHigh);
}

// *** flash.c *****************************************************************
//
// this file implements the low level flash control and access.
//
// This file originated from the cpustick.com skeleton project from
// http://www.cpustick.com/downloads.htm and was originally written
// by Rich Testardi; please preserve this reference and share bug
// fixes with rich@testardi.com.
// KeithV (Digilent) 6/10/2012 Modified for Just-In-Time-Flash-Erase; moved to main.c

/***    void flashOperation(uint32 nvmop, uint32 addr, uint32 data)
**
**    Synopsis:
**      Performs either a page erase, word write, or row write
**
**    Parameters:
**      nvmop	either NVMOP_PAGE_ERASE, NVMOP_WORD_PGM, or NVMOP_ROW_PGM
**		addr	the uint32_t flash address of: the page to erase, word location to write, or row location to write
**		data	a uint32_t of data to write, or the uint32_t of the SRAM address containing the array of data to write to the row
**
**    Return Values:
**      True if successful, false if failed
**
**    Errors:
**      None
**
**  Notes:
**      data has no meaning when page erase is specified and should be set to 0ul
**
*/
static void __attribute__((nomips16)) flashOperation(uint32 nvmop, uint32 addr, uint32 data)
{
    unsigned long   t0;
    unsigned int    status;

    #if defined(_PCACHE)
        unsigned long   K0;
        unsigned long   PFEN = CHECON & _CHECON_PREFEN_MASK;
    #endif

    // Convert Address to Physical Address
    NVMADDR = KVA_2_PA(addr);
    NVMDATA = data;
    NVMSRCADDR = KVA_2_PA(data);

    // Suspend or Disable all Interrupts
// no interrupts in the bootloader
//    SuspendINT(status);

    #if defined(_PCACHE)
        // disable predictive prefetching, see errata
        CHECONCLR = _CHECON_PREFEN_MASK;

        // turn off caching, see errata
        ReadK0(K0);
        WriteK0((K0 & ~0x07) | K0_UNCACHED);
    #endif

 	// Enable Flash Write/Erase Operations
    NVMCON = NVMCON_WREN | nvmop;

    // this is a poorly documented yet very important
    // required delay on newer silicon.
    // If you do not delay, on some silicon it will
    // completely latch up the flash to where you need
    // to cycle power, so wait for at least
    // 6us for LVD start-up, see errata
    t0 = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() - t0 < ((6 * F_CPU) / 2 / 1000000UL));

    // magic unlock sequence
    NVMKEY = 0xAA996655;
    NVMKEY = 0x556699AA;
    NVMCONSET = NVMCON_WR;

    // Wait for WR bit to clear
    while (NVMCON & NVMCON_WR);

    // see errata, wait 500ns before writing to any NVM register
    t0 = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() - t0 < ((F_CPU / 2 / 2000000UL)));

    // Disable Flash Write/Erase operations
    NVMCONCLR = NVMCON_WREN;

    #if defined(_PCACHE)
        // restore predictive prefetching and caching, see errata
        WriteK0(K0);
        CHECONSET = PFEN;
    #endif

    // Restore Interrupts
//no interrupts in the bootloader
//    RestoreINT(status);

    // assert no errors
// no return type in bootloader
//    return(! (NVMCON & (_NVMCON_WRERR_MASK | _NVMCON_LVDERR_MASK)));
}

/***    void flashErasePage(uint32 addrPage)
**
**    Synopsis:
**      Erases the page starting at the page address.
*
**    Parameters:
**      addrPage	The virtual address of the page to erase
**
**    Return Values:
**      None
**
**    Errors:
**      None
**
**  Notes:
**
**      addrPage must be page aligned.
**
*/
static void flashErasePage(uint32 addrPage)
{
    int i;
    int j;
    int32 x = ALLF;
    uint32 *rgUint32 = (uint32 *) addrPage;

    // we learned that just because the flash does not successfully
    // erase on the first attempt, it might on another. We found that
    // we can double the life of flash by attempting to
    // erase the flash up to 5 times before quitting.
    for(j=0; j<5; j++)
    {
    	// first check to see if the page needs to be erased
       	for (i = 0; i < FLASH_PAGE_SIZE/sizeof(uint32); i++)
    	{
    		x &= rgUint32[i];
    	}

        // flash erased, we are done.
        if(x == ALLF)
        {
            break;
        }

        // Unlock and Erase Page
        flashOperation(NVMOP_PAGE_ERASE, addrPage, 0);
    }

    // at this point, we don't care if the flash ever actually erased as
    // as we will catch the failure when we verify the programming.
 }

/***    void flashWriteUint32(uint32 addrUint32, uint32 *rgu32Data, uint32 cu32Data)
**
**    Synopsis:
**      Writes an array to uint32 to flash
*
**    Parameters:
**      rgu32Data	Pointer to an array of uint32
**		cu32Data	The number of uint32 to write
**
**    Return Values:
**      None
**
**    Errors:
**      None
**
**  Notes:
**
**      Assumes the pages have already been erased.
**
*/
static void flashWriteUint32(uint32 addrUint32, uint32 *rgu32Data, uint32 cu32Data)
{
    int  i = 0;

    for(i=0; i < cu32Data; i++)
	{

		// only do this if the data is not what is already in flash
		if(rgu32Data[i] != ALLF)
		{
	        // Write the data
	        flashOperation(NVMOP_WORD_PGM, addrUint32, rgu32Data[i]);
		}

        addrUint32 += sizeof(uint32);
    }
}

static void resetJustInTimeFlashErase() {
    unsigned int i = 0;
    for(i = 0; i < FLASH_BYTES/FLASH_PAGE_SIZE; ++i)
        pageMap[i] = 0;
}

/***    void justInTimeFlashErase(uint32 addrLow, uint32 addrHigh)
**
**    Synopsis:
**      Erases all pages that have not been erased in the address range
*
**    Parameters:
**      addrLow:	low virtual address of where the page needs erased
**		addrHigh	1 + the last high byte to be erased.
**
**    Return Values:
**      None
**
**    Errors:
**      None
**
**  Notes:
**
**      The address do not need to be page aligned.
**
*/
static void justInTimeFlashErase(uint32 addrLow, uint32 addrHigh)
{
	uint32 addrCurPage 		= startOfFlashPage(addrLow);
	uint32 addrLastPage 	= nextFlashPage(addrHigh - 1);
	uint32 iPage 			= getPageIndex(addrCurPage);

	while(addrCurPage < addrLastPage)
	{
		// if this has never been erased, erase it.
		if(pageMap[iPage] == 0)
		{
			flashErasePage(addrCurPage);
			pageMap[iPage] = 1;
		}

		iPage++;
		addrCurPage += FLASH_PAGE_SIZE;
	}
}

#ifdef _BOARD_CROSSCHASM_CELLULAR_C5_

/********************************************************************
 FileName:     SD Bootloader.c
 Dependencies: See INCLUDES section
 Processor:		PIC32 USB Microcontrollers
 Hardware:
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Description
  1.0   Initial release
  2.1   Updated for simplicity and to use common
                     coding style
********************************************************************/

//
// This file has been derived from the Microchip SD bootloader. The following
// functions have been modified from their original versions to integrate
// with this PIC32 bootloader.
 // WriteHexRecord2Flash()
 // ConvertAsciiToHex()
 // UpgradeFromModem() - was UpgradeFromSDCard()
//

#include <GenericTypeDefs.h>

#define DEV_CONFIG_REG_BASE_ADDRESS     0x9FC02FF0
#define DEV_CONFIG_REG_END_ADDRESS      0x9FC02FFF

#define PA_TO_KVA0(pa)  ((pa) | 0x80000000)

#define DATA_RECORD 		0
#define END_OF_FILE_RECORD 	1
#define EXT_SEG_ADRS_RECORD     2
#define EXT_LIN_ADRS_RECORD     4
#define START_ADDRESS_RECORD    0xA1
#define FLASH_LENGTH_RECORD     0xA2
#define FLASH_CRC_RECORD        0xA3

#define REC_FLASHED                 0
#define REC_NOT_FOUND               1
#define REC_FOUND_BUT_NOT_FLASHED   2

typedef struct {
    uint8 RecDataLen;
    DWORD_VAL Address;
    uint8 RecType;
    uint8* Data;
    uint8 CheckSum;
    DWORD_VAL ExtSegAddress;
    DWORD_VAL ExtLinAddress;
}T_HEX_RECORD;

typedef struct {
    UINT8 *start;
    UINT8 len;
    UINT8 status;
}T_REC;

typedef struct {
    unsigned int startAddress;
    unsigned int length;
    unsigned short int crc;
}E_REC;

/********************************************************************
* Function: 	WriteHexRecord2Flash()
*
* Precondition:
*
* Input: 		HexRecord buffer.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview:     Writes hex record to flash.
*
*
* Note:		 	None.
********************************************************************/
int WriteHexRecord2Flash(uint8* HexRecord) {

    static T_HEX_RECORD HexRecordSt;
    static E_REC eRecord;
    UINT8 Checksum = 0;
    UINT8 i;
    UINT WrData;
    void* ProgAddress;
    uint16 localCrc = 0;

    HexRecordSt.RecDataLen = HexRecord[0];
    HexRecordSt.RecType = HexRecord[3];
    HexRecordSt.Data = &HexRecord[4];

    // Hex Record checksum check.
    for(i = 0; i < HexRecordSt.RecDataLen + 5; ++i)
    {
        Checksum += HexRecord[i];
    }

    if(Checksum != 0)
    {
        return 0x00;
    }
    else
    {
        // Hex record checksum OK.
	switch(HexRecordSt.RecType) {

            case DATA_RECORD:  //Record Type 00, data record.

                HexRecordSt.Address.byte.MB = 0;
		HexRecordSt.Address.byte.UB = 0;
		HexRecordSt.Address.byte.HB = HexRecord[1];
		HexRecordSt.Address.byte.LB = HexRecord[2];

		// Derive the address
		HexRecordSt.Address.Val = HexRecordSt.Address.Val + HexRecordSt.ExtLinAddress.Val + HexRecordSt.ExtSegAddress.Val;

                // loop until all bytes are done
		while(HexRecordSt.RecDataLen) {

                    // Convert the Physical address to Virtual address.
                    ProgAddress = (void *)PA_TO_KVA0(HexRecordSt.Address.Val);

                    // Make sure we are not writing boot area and device configuration bits.
                    if(((ProgAddress >= (void *)FLASH_START) && (ProgAddress <= (void *)(FLASH_START + FLASH_BYTES - 1)))
                       && ((ProgAddress < (void*)DEV_CONFIG_REG_BASE_ADDRESS) || (ProgAddress > (void*)DEV_CONFIG_REG_END_ADDRESS))) {

                        if(HexRecordSt.RecDataLen < 4) {
                            // Sometimes record data length will not be in multiples of 4. Appending 0xFF will make sure that..
                            // we don't write junk data in such cases.
                            WrData = 0xFFFFFFFF;
                            ilmemcpy(&WrData, HexRecordSt.Data, HexRecordSt.RecDataLen);
                        }
			else {
                            ilmemcpy(&WrData, HexRecordSt.Data, 4);
                        }

                        // erase flash page (if needed and not already done)
                        justInTimeFlashErase((uint32)ProgAddress, (uint32)ProgAddress+4);
                        // Write the data into flash.
                        flashWriteUint32((uint32)ProgAddress, (uint32*)&WrData, 1);
                    }

                    // Increment the address.
                    HexRecordSt.Address.Val += 4;
                    // Increment the data pointer.
                    HexRecordSt.Data += 4;
                    // Decrement data len.
                    if(HexRecordSt.RecDataLen > 3) {
                        HexRecordSt.RecDataLen -= 4;
                    }
                    else {
                        HexRecordSt.RecDataLen = 0;
                    }

                } // end while(HexRecordSt.RecDataLen)
		break;

            case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4th to 19th bits of the data address.

                HexRecordSt.ExtSegAddress.byte.MB = 0;
                HexRecordSt.ExtSegAddress.byte.UB = HexRecordSt.Data[0];
		HexRecordSt.ExtSegAddress.byte.HB = HexRecordSt.Data[1];
		HexRecordSt.ExtSegAddress.byte.LB = 0;
		// Reset linear address.
		HexRecordSt.ExtLinAddress.Val = 0;
		break;

            case EXT_LIN_ADRS_RECORD:   // Record Type 04, defines 16th to 31st bits of the data address.
                HexRecordSt.ExtLinAddress.byte.MB = HexRecordSt.Data[0];
                HexRecordSt.ExtLinAddress.byte.UB = HexRecordSt.Data[1];
		HexRecordSt.ExtLinAddress.byte.HB = 0;
		HexRecordSt.ExtLinAddress.byte.LB = 0;
		// Reset segment address.
		HexRecordSt.ExtSegAddress.Val = 0;
		break;

            case END_OF_FILE_RECORD:  //Record Type 01, defines the end of file record.
		HexRecordSt.ExtSegAddress.Val = 0;
		HexRecordSt.ExtLinAddress.Val = 0;

                // wait for the extended records (FLASH start, len, crc) before
                // we return
                //return 0xFF;

		break;

            case START_ADDRESS_RECORD:

                eRecord.startAddress = 0;
                for(i = 0; i < HexRecordSt.RecDataLen; ++i)
                {
                    eRecord.startAddress <<= 8;
                    eRecord.startAddress += HexRecordSt.Data[i];
                }

                break;

            case FLASH_LENGTH_RECORD:

                eRecord.length = 0;
                for(i = 0; i < HexRecordSt.RecDataLen; ++i)
                {
                    eRecord.length <<= 8;
                    eRecord.length += HexRecordSt.Data[i];
                }

                break;

            case FLASH_CRC_RECORD:

                eRecord.crc = 0;
                for(i = 0; i < HexRecordSt.RecDataLen; ++i)
                {
                    eRecord.crc <<= 8;
                    eRecord.crc += HexRecordSt.Data[i];
                }

                localCrc = crc16Reflected((unsigned char*)eRecord.startAddress, eRecord.length);
                if(localCrc == eRecord.crc)
                    return 0xFF;
                else
                    return 0x00;

                break;

            default:
		HexRecordSt.ExtSegAddress.Val = 0;
		HexRecordSt.ExtLinAddress.Val = 0;

                break;

        } // end switch(HexRecordSt.RecType)

    } // end if CheckSum == 0

    return 0x01;
}

/********************************************************************
* Function: 	ConvertAsciiToHex()
*
* Precondition:
*
* Input: 		Ascii buffer and hex buffer.
*
* Output:
*
* Side Effects:	No return from here.
*
* Overview: 	Converts ASCII to Hex.
*
*
* Note:		 	None.
********************************************************************/
void ConvertAsciiToHex(uint8* asciiRec, uint8* hexRec, unsigned int ascii_len) {

    uint8 i = 0;
    uint8 k = 0;
    uint8 hexT;

    for(i = 0; i < ascii_len; i++) {
        // Check if the ascii values are in alpha numeric range.
        if(!((asciiRec[i] >= 0x30) && (asciiRec[i] <= 0x66)))
            break;

        if(asciiRec[i] < 0x3A) {
            // Numerical reperesentation in ASCII found.
            hexT = asciiRec[i] & 0x0F;
        }
        else {
            // Alphabetical value.
            hexT = 0x09 + (asciiRec[i] & 0x0F);
        }

        // Following logic converts 2 bytes of ASCII to 1 byte of hex.
        k = i%2;

        if(k) {
            hexRec[i/2] |= hexT;
        }
        else {
            hexRec[i/2] = (hexT << 4) & 0xF0;
        }
    }

    return;

}

/********************************************************************
* Function: 	UpgradeFromModem()
*
* Precondition:
*
* Input: 		None.
*
* Output:		None.
*
* Side Effects:	None.
*
* Overview: 	Main entry function. If there is a trigger or
*				if there is no valid application, the device
*				stays in firmware upgrade mode.
*
*
* Note:		 	None.
********************************************************************/
unsigned int UpgradeFromModem(unsigned char* data, unsigned int len) {

    uint8 hexRec[32];
    T_REC record;
    unsigned int i = 0;
    uint8 HexRecordWriteResult = 0;

    record.status = REC_NOT_FOUND;

    for(i = 0; i < len; ++i) {

        // This state machine seperates-out the valid hex records from the read 512 bytes.
        switch(record.status) {

            case REC_FLASHED:

            case REC_NOT_FOUND:

                if(data[i] == ':' || data[i] == ';') {
                    // We have a record found in the 512 bytes of data in the buffer.
                    record.start = &data[i];
                    record.len = 0;
                    record.status = REC_FOUND_BUT_NOT_FLASHED;
                }

                break;

            case REC_FOUND_BUT_NOT_FLASHED:

                if((data[i] == 0x0A) || (data[i] == 0xFF)) {

                    // We have got a complete record. (0x0A is new line feed and 0xFF is End of file)
                    // Start the hex conversion from element
                    // 1. This will discard the ':' which is
                    // the start of the hex record.
                    ConvertAsciiToHex(&record.start[1], hexRec, len);
                    HexRecordWriteResult = WriteHexRecord2Flash(hexRec);

                    // if the write function returns 0xFF, we hit EOF
                    if(HexRecordWriteResult == 0xFF)
                    {
                        // return to main bootloader
                        return 0xFF;
                    }
                    else if(HexRecordWriteResult == 0x00)
                    {
                        return 0x00;
                    }

                    record.status = REC_FLASHED;

                } // end complete record processing

                break;

        } // end switch case

        // Move to next byte in the buffer.
        record.len++;

    }

    return 0x01;

}

// *** otaloader.c *****************************************************************
//
// code that, in combination with that derived from the Microchip SD bootloader,
// manages OTA upgrades
//

#define STR(s) #s

#define START_TIMER(x)    x = _CP0_GET_COUNT()
#define CHECK_TIMER(x,y)  (_CP0_GET_COUNT()-x) >= y*CORE_TIMER_TICKS_PER_MILLISECOND

// backup file name
static const char* backup_file_name = "flash.bak";
static const uint8 backup_file_name_len = 9;

// commands
static const char* lscript = "AT#LCSCRIPT\r\n";
static const uint8 lscript_len = 13;

static const char* wscript = "AT#WSCRIPT=\"flash.bak\",524288\r\n";
static const uint8 wscript_len = 31;

static const char* rscript = "AT#RSCRIPT=\"flash.bak\"\r\n";
static const uint8 rscript_len = 24;

static const char* cflo = "AT#CFLO=1\r\n";
static const uint8 cflo_len = 11;

static const char* socketstatus = "AT#SS=1\r\n";
static const uint8 socketstatus_len = 9;

static const char* socketread = "AT#SRECV=1,512\r\n";
static const uint8 socketread_len = 16;

static const char* baud115200 = "AT+IPR=115200\r\n";
static const unsigned int baud115200_len= 15;

/*STRING HELPERS*/

static bool isalpha(unsigned char c) {
    return ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'));
}

static bool isupper(unsigned char c) {
    return (c >= 'A' && c <= 'Z');
}

static bool isalphanumeric(unsigned char c) {
    return ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'));
}

// converts a string to int representation
// handles base 16 or base 10, upper and lower case
// used to convert size and crc values reported for telit scripts
static unsigned int string_to_int(unsigned char* s, uint8 base) {

    unsigned int val = 0;

    while(isalphanumeric(*s))
    {
        switch(base)
        {
            case 10:
                val *= 10;
                break;

            case 16:
                val = val << 4;
                break;

            default:
                return 0;
                break;
        }
        if(isalpha(*s))
        {
            val += isupper(*s) ? *s - 'A' + 10 : *s - 'a' + 10;
        }
        else
        {
            val += *s - '0';
        }
        s++;
    }

    return val;

}

/*UART*/

#define UART_RX_SIZE    1024

typedef struct {
    unsigned int head;
    unsigned int tail;
    unsigned int size;
    unsigned char buffer[UART_RX_SIZE];
}_FIFO;

static _FIFO fifo;

unsigned char* uart_find(unsigned char* s, unsigned char* f) {

    unsigned char* sp = s;
    unsigned char* fp = f;
    unsigned int i = 0;

    while(*sp && sp < fifo.buffer+fifo.head)
    {
        if(*sp++ == *fp)
        {
            fp++;
            i++;
        }
        else
        {
            fp = f;
            i = 0;
        }
        if(!*fp)
        {
            return sp - i;
        }
    }

    return NULL;
}

unsigned char* finder(unsigned char* s, unsigned char* f, unsigned char* m) {

    unsigned char* sp = s;
    unsigned char* fp = f;
    unsigned int i = 0;

    while(*sp && sp < m)
    {
        if(*sp++ == *fp)
        {
            fp++;
            i++;
        }
        else
        {
            fp = f;
            i = 0;
        }
        if(!*fp)
        {
            return sp - i;
        }
    }

    return NULL;
}

void uart_tx(unsigned char* data, unsigned int len) {
    int i;
    for (i = 0; i < len; i++)
    {
        while (!USTAbits.TRMT);
        UTXREG = data[i];
    }
}

bool uart_rx_full() {
    return (fifo.size - ((fifo.head - fifo.tail)&(fifo.size - 1))) == 1;
}

bool uart_rx_empty() {
    return fifo.head == fifo.tail;
}

unsigned char uart_rx_pop() {

    unsigned char c;

    if(fifo.head == fifo.tail)
        return 0;

    c = *(fifo.buffer+fifo.tail);
    fifo.tail = (fifo.tail + 1) % fifo.size;

    return c;

}

void inline uart_rx_push(unsigned char c) {
    if(!uart_rx_full())
    {
        *(fifo.buffer+fifo.head) = c;
        fifo.head = (fifo.head + 1) % fifo.size;
    }
}

bool uart_rx() {
    if(USTAbits.URXDA)
    {
        if(!uart_rx_full())
        {
            uart_rx_push(URXREG);
            if(USTAbits.OERR)
                USTAbits.OERR = 0;
            return 1;
        }
    }

    return 0;
}

void uart_clear() {
    unsigned char r;

    fifo.head = 0;
    fifo.tail = 0;
    fifo.size = UART_RX_SIZE;

    USTAbits.OERR = 0;
    while(USTAbits.URXDA)
        r = URXREG;
}

void uart_rx_reset() {
    fifo.head = 0;
    fifo.tail = 0;
    fifo.size = UART_RX_SIZE;
}

/*CRC16-CCITT*/

// byte reflection table (for input-reflected CRC)
static const unsigned char BitReverseTable256[] = {
  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

// CRC16 calculation table
static const uint16 crc_table[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef};

uint16 crc16Reflected(unsigned char* data, unsigned int len) {

    unsigned short int crc = 0xFFFF;
    unsigned int i = 0;
    unsigned char rdata = 0;

    while(len--)
    {
        rdata = BitReverseTable256[*data];
        i = (crc >> 12) ^ (rdata >> 4);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        i = (crc >> 12) ^ (rdata >> 0);
        crc = crc_table[i & 0x0F] ^ (crc << 4);
        data++;
    }

    return BitReverseTable256[(unsigned char)((crc & 0xFF00) >> 8)] + (BitReverseTable256[(unsigned char)(crc & 0x00FF)] << 8);

}

/*TELIT BASIC FUNCTIONS*/

bool telit_socketstatus() {

    static unsigned int state = 0;

    switch(state)
    {
        case 0:

            // check socket status
            uart_clear();
            uart_tx(socketstatus, socketstatus_len);
            state = 1;

            break;

        case 1:

            // wait for reply
            uart_rx();
            if(uart_find(fifo.buffer, "\r\n\r\nOK\r\n"))
            {
                uart_clear();
                state = 2;
            }

            break;

        default:
            state = 0;
            break;
    }

    return state == 2;

}

bool telit_readsocket(unsigned char** data, unsigned int* len) {

    static unsigned int state = 0;
    static unsigned char* p;
    static unsigned char* p2;
    static unsigned int ret_len;
    static unsigned int i = 0;

    switch(state)
    {
        case 0:

            // request socket data
            i = 0;
            uart_clear();
            uart_tx(socketread, socketread_len);
            state = 1;
            //uart_tx("X",1);
            break;

        case 1:

            // get reply
            uart_rx();
            if(p = uart_find(fifo.buffer, "#SRECV:"), p)
            {
                //uart_tx("Y",1);
                p += 10;
                state = 2;
            }

            break;

        case 2:

            uart_rx();
            if(p2 = uart_find(p, "\r\n"), p2)
            {
                //uart_tx("Z",1);
                ret_len = string_to_int(p, 10);
                p2 += 2;
                state = 3;
            }

            break;

        case 3:

            // p2 points to start of socket data
            // read out the reported length
            //uart_tx("J",1);
            if(uart_rx())
                i++;
            if(i >= ret_len)
                state = 4;

            break;

        case 4:

            // find OK
            uart_rx();
            if(uart_find(p2+i, "OK\r\n"))
            {
                *data = p2;
                *len = ret_len;
                state = 5;
            }

            break;

        default:
            state = 0;
            break;
    }

    return state == 5;

}

bool telit_cflo() {

    static unsigned int state = 0;

    switch(state)
    {
        case 0:

            // enable command flow control
            uart_clear();
            uart_tx(cflo, cflo_len);
            state = 1;

        case 1:

            // wait for reply
            uart_rx();
            if(uart_find(fifo.buffer, "\r\n\r\nOK\r\n"))
            {
                uart_clear();
                state = 2;
            }

            break;

        default:
            state = 0;
            break;
    }

    return state == 2;

}

bool telit_initialize() {

    static unsigned int state = 0;
    static unsigned int timer = 0;

    switch(state)
    {
        case 0:

            MODEM_ON();
            START_TIMER(timer);
            state = 1;

            break;

        case 1:
            if(!CHECK_TIMER(timer, 8500) == 0)
                break;
            state = 2;

            break;

        case 2:

            if(telit_cflo())
            {
                START_TIMER(timer);
                state = 3;
            }

            break;

        case 3:

            if(CHECK_TIMER(timer, 500) == 0)
                break;

            if(telit_setbaud115200())
            {
                START_TIMER(timer);
                state = 4;
            }

            break;

        case 4:

            if(CHECK_TIMER(timer, 500) == 1)
            {
                telit_initialized = true;
                state = 5;
            }

            break;

        default:
            state = 0;
            break;
    }

    return state == 5;

}

bool telit_setbaud115200() {

    static unsigned int state = 0;

    switch(state)
    {
        case 0:

            uart_clear();
            uart_tx(baud115200, baud115200_len);
            state = 1;

        case 1:

            uart_rx();
            if(uart_find(fifo.buffer, "\r\n\r\nOK\r\n"))
            {
                uart_clear();
                state = 2;
            }

            break;

        default:
            state = 0;
            break;
    }

    return state == 2;

}

/*TELIT BACKUP/RESTORE*/

enum {
    BACKUP_CALCULATE_FLASH_CRC16,
    BACKUP_FIND_PREVIOUS_BACKUP,
    BACKUP_TRANSFER_MEMORY,

    BACKUP_COMPLETE
};

RETVAL telit_getbackupinfo(unsigned int* script_size, unsigned int* script_crc) {

    static RETVAL ret = None;
    static uint8 state = 0;
    static unsigned char* p = NULL;
    //static unsigned char* p2 = NULL;
    static unsigned int timeout = 0;
    static unsigned int timer = 0;

    switch(state)
    {
        default:
            state = 0;
        case 0:

            // ask for list of stored scripts
            uart_clear();
            uart_tx(lscript, lscript_len);

            ret = Working;
            START_TIMER(timeout);
            START_TIMER(timer);
            
            state = 1;

            break;

        case 1:

            uart_rx();
            if(CHECK_TIMER(timer, 500) == 0)
                break;

            START_TIMER(timer);

            // wait for command to complete
            if(p = uart_find(fifo.buffer, "\r\n\r\nOK\r\n"), p)
            {
                state = 2;
            }

            break;

         case 2:

            // look for the backup file and, if found, read
            if(p = uart_find(fifo.buffer, backup_file_name), p)
            {
                //if(p2 = uart_find(p, "\r\n"), p2)
                //{
                    p += backup_file_name_len + 2;
                    *script_size = string_to_int(p, 10);
                    if(p = uart_find(p, ","), p)
                    {
                        p++;
                        *script_crc = string_to_int(p, 16);
                        ret = Success;
                        state = 3;
                    }
                //}
            }
            else
            {
                state = 3;
                ret = Failed;
            }

            break;
    }

    if(CHECK_TIMER(timeout, 10000) == 1)
    {
        ret = Failed;
    }

    return ret;

}

RETVAL telit_findbackup() {

    static RETVAL ret = None;
    static RETVAL subret = None;
    static uint16 myCrc = 0;
    static uint8 state = 0;
    static unsigned int found_crc = 0;
    static unsigned int found_size = 0;
    static unsigned int timeout = 0;

    switch(state)
    {
        default:
            state = 0;
        case 0:

            ret = Working;
            START_TIMER(timeout);

            myCrc = crc16Reflected((unsigned char*)FLASH_START, FLASH_BYTES);
            state = 1;
            break;

        case 1:

            subret = telit_getbackupinfo(&found_size, &found_crc);
            if(subret == Success)
            {
                #warning "test"
                //uart_tx(&found_crc, 2);
                //uart_tx(&myCrc, 2);
                if(found_size == FLASH_BYTES && found_crc == myCrc)
                {
                    state = 2;
                    ret = Success;
                }
                else
                {
                    state = 2;
                    ret = Failed;
                }
            }
            else if(subret == Failed)
            {
                state = 2;
                ret = Failed;
            }
            
            break;
    }

    if(CHECK_TIMER(timeout, 5000) == 1)
    {
        ret = Failed;
    }

    return ret;
    
}

RETVAL telit_makebackup() {

    static RETVAL ret = None;
    static unsigned int state = 0;
    static unsigned int i = 0;
    static unsigned int myCrc = 0;
    static unsigned int timeout = 0;
    static unsigned int timer = 0;

    switch(state)
    {
        default:
            state = 0;
        case 0:

            ret = Working;
            START_TIMER(timeout);

            // get my crc
            myCrc = crc16Reflected((unsigned char*)FLASH_START, FLASH_BYTES);
            uart_clear();
            state = 1;

            break;

        case 1:

            if(telit_cflo())
            {
                START_TIMER(timer);
                state = 2;
            }

            break;

        case 2:

            if(CHECK_TIMER(timer, 500) == 0)
                break;

            // request to write a script
            uart_clear();
            uart_tx(wscript, wscript_len);
            state = 3;

            break;

        case 3:

            // wait for ready response
            uart_rx();
            if(uart_find(fifo.buffer, ">>>"))
            {
                uart_clear();
                state = 4;
            }

            break;

        case 4:

            // dump FLASH memory to script (in chunks to avoid long blocking)
            uart_tx(FLASH_START + i, 1024);
            i += 1024;
            if(i == FLASH_BYTES)
                state = 5;

            break;

        case 5:

            // wait for OK
            uart_rx();
            if(uart_find(fifo.buffer, "OK"))
            {
                START_TIMER(timer);
                state = 6;
            }

            break;

        case 6:

            if(CHECK_TIMER(timer, 500) == 0)
                break;
            
            // confirm successful backup
            if(telit_findbackup() == Success)
            {
                state = 7;
                ret = Success;
            }

            break;
    }

    if(CHECK_TIMER(timeout, 75000) == true)
        ret = Failed;

    return ret;

}

__attribute__((nomips16)) RETVAL telit_restorebackup() {

    RETVAL ret = None;

    static unsigned int state = 0;
    static unsigned int timer = 0;
    static unsigned int flashunit = 0;
    static unsigned int i = 0;
    static unsigned int pflash = 0;
    static unsigned int myCrc = 0;

    static unsigned int script_size;
    static unsigned int script_crc;

    static unsigned int timeout = 0;

    switch(state)
    {
        case 0:

            ret = Working;
            START_TIMER(timeout);
            resetJustInTimeFlashErase();

            // initialize flash pointer
            pflash = FLASH_START;

            // find backup
            if(telit_getbackupinfo(&script_size, &script_crc))
            {
                if(script_size == FLASH_BYTES)
                {
                    START_TIMER(timer);
                    state = 1;
                }
            }

            break;

        case 1:

            if(CHECK_TIMER(timer, 500) == 0)
                break;

            // request to read a script
            uart_clear();
            uart_tx(rscript, rscript_len);
            state = 2;

            break;

        case 2:

            uart_rx();
            if(uart_find(fifo.buffer, "<<<"))
            {
                uart_clear();
                state = 3;
            }

            break;

        case 3:

            // read the backup to FLASH
            uart_rx();
            if(!uart_rx_empty())
            {
                //flashunit <<= 8;
                flashunit += ((uint32)uart_rx_pop() << 8*i);
                ++i;
                if(i == 4)
                {
                    justInTimeFlashErase(pflash, pflash+sizeof(unsigned int));
                    flashWriteUint32((uint32)pflash, (uint32*)&flashunit, 1);
                    flashunit = 0;
                    i = 0;
                    pflash+=sizeof(unsigned int);
                    if(pflash == FLASH_START + FLASH_BYTES)
                    {
                        uart_clear();
                        state = 4;
                    }
                }
            }

            break;

        case 4:

            // wait for the OK
            uart_rx();
            if(uart_find(fifo.buffer, "\r\nOK\r\n"))
            {
                uart_clear();
                state = 5;
            }

            break;

        case 5:

            // confirm FLASH crc
            myCrc = crc16Reflected((unsigned char*)FLASH_START, FLASH_BYTES);
            if(myCrc == script_crc)
            {
                state = 6;
                ret = Success;
            }
            else
            {
                state = 6;
                ret = Failed;
            }

            break;

        default:

            break;
    }
    
    if(CHECK_TIMER(timeout, 45000) == true)
        ret = Failed;

    return ret;

}

RETVAL hexloader() {

    static RETVAL ret = None;
    static unsigned char record[64];
    static unsigned char* record_index;
    static unsigned char* p_record;
    static unsigned char* p;
    static unsigned int plen;
    static unsigned int hexStatus = 0;
    static unsigned int state = 0;
    static unsigned int timer = 0;

    switch(state)
    {
        default:
            state = 0;
        case 0:

            uart_clear();
            record_index = record;
            ret = Working;
            state = 1;

            break;

        case 1:

            // request a chunk from the socket
            START_TIMER(timer);
            while(!telit_readsocket(&p, &plen))
            {
                if(CHECK_TIMER(timer, 5000) == 1)
                {
                    // failed to read from socket while upgrade is in progress
                    flashErasePage(USER_APP_ADDR); // invalidate the user app
                    return Failed;
                }
            }
            while(plen--)
            {
                *record_index++ = *p++;
                if(p_record = finder(record, "\r\n", record_index), p_record)
                {
                    // we can buffer the bytes here until we reach EOL
                    record_index = record;
                    hexStatus = UpgradeFromModem(record, p_record - record + 2);
                    switch(hexStatus)
                    {
                        case 0x00:
                            flashErasePage(USER_APP_ADDR);
                            ret = Failed;
                            state = 2;
                            break;

                        case 0x01:
                            ret = Working;
                            state = 1;
                            break;

                        case 0xFF:
                            finshFlashProcessingAfterLoad();
                            ret = Success;
                            state = 2;
                            break;
                    }
                }
            }

            break;
    }

    return ret;
}

#define BACKUP_TRIES     2
#define RESTORE_TRIES    2

RETVAL otaloader() {

    RETVAL ret = None;
    unsigned int state = 0;
    unsigned int otaStartTime = 0;
    unsigned int otaLoopTime = 0;
    unsigned int otaLastBlink = 0;
    unsigned int tryCount = 0;

    WDTCONSET = 0x8000; // enable watchdog
    InitUARTInterfaceSpecific(230400, false);
    otaStartTime = _CP0_GET_COUNT();

    while(ret != Success && ret != Failed)
    {
        WDTCONSET = 0x01; // service watchdog
        otaLoopTime = _CP0_GET_COUNT();
        if ((otaLoopTime - otaLastBlink) >= ((CORE_TIMER_TICKS_PER_MILLISECOND  * 63)) && state > 0)
        {
            // blink the heartbeat LED
            BootLED_Toggle();
            // set up for the next blink
            otaLastBlink = otaLoopTime;
        }

        switch(state)
        {
            case 0:
                ret = Working;
                // find out if we need to do an upgrade
                if(telit_socketstatus())
                {
                    UninitUARTInterface();
                    InitUARTInterfaceSpecific(230400, true);
                    state = 1;
                }
                else if(_CP0_GET_COUNT() - otaStartTime > (CORE_TIMER_TICKS_PER_MILLISECOND  * OTA_LOADER_CHECK_WAIT_MS))
                {
                    ret = Failed;
                }
                break;
                
            case 1:
                // find out if we have a backup
                switch(telit_findbackup())
                {
                    case None:
                    case Working:
                        break;
                    case Success:
                        state = 3;
                        break;
                    case Failed:
                    default:
                        state = 2;
                        break;
                }
                break;

            case 2:
                // create a backup if we don't have it
                switch(telit_makebackup())
                {
                    case None:
                    case Working:
                        break;
                    case Success:
                        state = 3;
                        break;
                    case Failed:
                    default:
                        tryCount++;
                        if(tryCount >= BACKUP_TRIES)
                        {
                            tryCount = 0;
                            ret = Failed;
                            state = 0;
                        }
                        else
                        {
                            state = 2;
                        }
                        break;
                }
                break;

            case 3:
                // download and apply the new firmware
                switch(hexloader())
                {
                    case None:
                    case Working:
                        break;
                    case Success:
                        state = 0;
                        ret = Success;
                        break;
                    case Failed:
                    default:
                        state = 4;
                        ret = Working;
                        break;
                }
                break;

            case 4:
                // if the upgrade fails, restore the backup
                switch(telit_restorebackup())
                {
                    case None:
                    case Working:
                        break;
                    case Success:
                        state = 0;
                        ret = Success;
                        break;
                    case Failed:
                    default:
                        tryCount++;
                        if(tryCount >= RESTORE_TRIES)
                        {
                            tryCount = 0;
                            ret = Failed;
                            state = 0;
                        }
                        else
                        {
                            state = 4;
                        }
                        break;
                }
                break;

            default:
                state = 0;
                ret = None;
                break;
        }
    }
    MODEM_OFF();
    UninitUARTInterface();
    WDTCONCLR = 0x8000; // disable watchdog

    return ret;

}

#endif