#include <cstdint>
#include "flash_os.hpp"

#include <klib/klib.hpp>
#include <io/pins.hpp>
#include <io/spi.hpp>
#include <io/system.hpp>

#include <klib/delay.hpp>
#include <klib/hardware/memory/is25lq040b.hpp>

namespace target = klib::target;

using cs = target::io::pin_out<target::pins::package::lqfp_80::p50>;
using spi = target::io::spi<target::io::periph::lqfp_80::spi0>;
using memory = klib::hardware::memory::is25lq040b<spi, cs>;

/**
 * @brief Smallest amount of data that can be programmed
 * 
 * <PageSize> = 2 ^ Shift. Shift = 3 => <PageSize> = 2^3 = 8 bytes
 * 
 */
#define PAGE_SIZE_SHIFT (8)

/**
 * @brief If value is false the device does not support native read. This 
 * makes the loader use the read function instead of using memory mapped 
 * io.
 * 
 */
#define NATIVE_READ (false)

/**
 * @brief Marks if the device supports a chip erase. This can speed up erasing
 * a chip.
 * 
 */
#define CHIP_ERASE (true)

/**
 * @brief If value is true only uniform sectors are allowed on the device. 
 * Speeds up erasing as it can erase multiple sectors at once.
 * 
 */
#define UNIFORM_SECTORS (true)

/**
 * @brief Sector size shift for when using uniform sector erase
 * 
 * <SectorSize> = 2 ^ Shift. Shift = 12 => <SectorSize> = 2 ^ 12 = 4096 bytes
 * 
 */
#define SECTOR_SIZE_SHIFT (12)

/**
 * @brief Use a custom verify. Is optional. Speeds up verifying
 * 
 */
#define CUSTOM_VERIFY (false)

/**
 * @brief Device specific infomation
 * 
 */
extern "C" {
    // declaration for the flash device. If we initialize it here we get
    // a wrong name in the symbol table
    extern const struct flash_device FlashDevice;

    // Mark start of <PrgData> segment. Non-static to make sure linker can keep this 
    // symbol. Dummy needed to make sure that <PrgData> section in resulting ELF file 
    // is present. Needed by open flash loader logic on PC side
    volatile int PRGDATA_StartMarker __attribute__ ((section ("PrgData"), __used__));
}

// definition for the flash device
const flash_device FlashDevice __attribute__ ((section ("DevDscr"), __used__)) = {
    flash_drv_version, // driver version
    "is25lq040b", // device name
    device_type::on_chip, // device type
    0xA0000000, // base address
    0x00080000, // flash size
    256, // page size
    0, // reserved
    0xff, // blank value
    20, // page program timeout
    3000, // sector erase timeout

    // flash sectors
    {
        {0x00001000, 0x00000000},
        end_of_sectors
    }
};

// function overrides when parts are not in use
#if NATIVE_READ
    #define BLANK_CHECK_FUNC nullptr
    #define OPEN_READ_FUNC nullptr
#else
    #define BLANK_CHECK_FUNC BlankCheck
    #define OPEN_READ_FUNC SEGGER_OPEN_Read
#endif

#if CUSTOM_VERIFY
    #define VERIFY_FUNC Verify
#else 
    #define VERIFY_FUNC nullptr
#endif

#if CHIP_ERASE
    #define CHIP_ERASE_FUNC EraseChip
#else
    #define CHIP_ERASE_FUNC nullptr
#endif

#if UNIFORM_SECTORS
    #define UNIFORM_ERASE_FUNC SEGGER_OPEN_Erase
#else
    #define UNIFORM_ERASE_FUNC nullptr
#endif

/**
 * @brief array with all the functions for the segger software
 * 
 */
extern "C" {
    // declaration for the OFL Api. If we initialize it here we get
    // a wrong name in the symbol table
    extern const uint32_t SEGGER_OFL_Api[13];
}

// definition of OFL Api
const uint32_t SEGGER_OFL_Api[13] __attribute__ ((section ("PrgCode"), __used__)) = {
    reinterpret_cast<uint32_t>(FeedWatchdog),
    reinterpret_cast<uint32_t>(Init),
    reinterpret_cast<uint32_t>(UnInit),
    reinterpret_cast<uint32_t>(EraseSector),
    reinterpret_cast<uint32_t>(ProgramPage),
    reinterpret_cast<uint32_t>(BLANK_CHECK_FUNC),
    reinterpret_cast<uint32_t>(CHIP_ERASE_FUNC),
    reinterpret_cast<uint32_t>(VERIFY_FUNC),
    reinterpret_cast<uint32_t>(nullptr), // SEGGER_OPEN_CalcCRC
    reinterpret_cast<uint32_t>(OPEN_READ_FUNC),
    reinterpret_cast<uint32_t>(SEGGER_OPEN_Program),
    reinterpret_cast<uint32_t>(UNIFORM_ERASE_FUNC),
    reinterpret_cast<uint32_t>(nullptr), // SEGGER_OPEN_Start for turbo mode
};

void __attribute__ ((noinline)) FeedWatchdog(void) {
    // TODO: implement something to keep the watchdog happy
    return;
}

int __attribute__ ((noinline)) Init(const uint32_t address, const uint32_t frequency, const uint32_t function) {
    using clock = target::io::system::clock;

    // setup the flash wait state to 4 + 1 CPU clocks
    target::io::system::flash::setup<4>();

    // setup the clock to 96Mhz (this is using a 12Mhz oscillator)
    // (((15 + 1) * 2 * 12Mhz) / (0 + 1) = 384Mhz) / (3 + 1) = 96Mhz
    clock::set_main<clock::source::internal, 96'000'000, 47, 0, 3>();

    // klib::clock::set(4'000'000);

    // init the cs pin
    cs::init();

    // init the spi driver
    spi::init<
        klib::io::spi::mode::mode3, 1'000'000, 
        klib::io::spi::bits::bit_8, true
    >();

    cs::template set<true>();

    // init the memory using the spi and cs
    memory::init();

    // wait until the device is not busy
    while (memory::is_busy()) {
        klib::delay<klib::busy_wait>(klib::time::ms{3});
    }

    return 0;
}

int __attribute__ ((noinline)) UnInit(const uint32_t function) {
    // TODO: implement uninit

    return 0;
}

int __attribute__ ((noinline)) EraseSector(const uint32_t sector_address) {   
    // do a sector erase
    memory::erase(memory::erase_mode::sector, (sector_address & 0xfffffff));

    // wait until the device is not busy
    while (memory::is_busy()) {
        // wait and do nothing
        klib::delay<klib::busy_wait>(klib::time::ms{3});
    }

    return 0;
}

int __attribute__ ((noinline)) ProgramPage(const uint32_t address, const uint32_t size, const uint8_t *const data) {
    // write the data to the memory device
    memory::write((address & 0xfffffff), data, size);

    // wait until the device is not busy
    while (memory::is_busy()) {
        // wait and do nothing
        klib::delay<klib::busy_wait>(klib::time::ms{3});
    }

    return 0;
}

int __attribute__ ((noinline)) SEGGER_OPEN_Program(uint32_t address, uint32_t size, uint8_t *data) {
    // get the amount of pages to write
    const uint32_t pages = size >> PAGE_SIZE_SHIFT;

    for (uint32_t i = 0; i < pages; i++) {
        // program a page
        int r = ProgramPage(address, (0x1 << PAGE_SIZE_SHIFT), data);

        // check if something went wrong
        if (r) {
            // return a error
            return 1;
        }

        address += (0x1 << PAGE_SIZE_SHIFT);
        data += (0x1 << PAGE_SIZE_SHIFT);
    }

    // return everything went oke
    return 0;
}

#if CHIP_ERASE == true
    int __attribute__ ((noinline)) EraseChip(void) {
        // do a chip erase
        memory::chip_erase();

        // wait until the device is not busy
        while (memory::is_busy()) {
            // wait and do nothing
            klib::delay<klib::busy_wait>(klib::time::ms{3});
        }        

        return 0;
    }
#endif

#if UNIFORM_SECTORS
    int __attribute__ ((noinline)) SEGGER_OPEN_Erase(uint32_t SectorAddr, uint32_t SectorIndex, uint32_t NumSectors) {
        // feed the watchdog
        FeedWatchdog();

        for (uint32_t i = 0; i < NumSectors; i++) {
            // erase a sector
            int r = EraseSector((SectorAddr & 0xfffffff));

            // check for errors
            if (r) {
                // return we have a error
                return 1;
            }

            // go to the next sector address
            SectorAddr += (1 << SECTOR_SIZE_SHIFT);
        }

        // return everything went oke
        return 0;
    }
#endif

#if CUSTOM_VERIFY
    uint32_t __attribute__ ((noinline, __used__)) Verify(uint32_t Addr, uint32_t NumBytes, uint8_t *pBuff) {
        // TODO: implement verify

        return (Addr + NumBytes);
    }
#endif

#if !NATIVE_READ
    int __attribute__ ((noinline, __used__)) BlankCheck(const uint32_t address, const uint32_t size, const uint8_t blank_value) {
        static uint8_t buffer[256];
        
        // read all the memory and compare it with the blank value
        for (uint32_t i = 0; i < size; /* do not update i here */) {
            // get the size to read
            const uint32_t s = klib::min(size - i, sizeof(buffer));

            // read memory from device
            memory::read((address & 0xfffffff) + i, buffer, s);

            // check if all the data matches the blank value
            for (uint32_t j = 0; j < s; j++) {
                if (buffer[j] != blank_value) {
                    return 1;
                }
            }

            // update i
            i += s;
        }

        return 0;
    }

    int __attribute__ ((noinline, __used__)) SEGGER_OPEN_Read(const uint32_t address, const uint32_t size, uint8_t *const data) {
        // read memory
        memory::read((address & 0xfffffff), data, size);

        return size;
    }
#endif
