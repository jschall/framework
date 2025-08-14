MEMORY
{
    bl (rx) :             ORIGIN = 0x08000000,            LENGTH = 32K
    param1 (rx) :         ORIGIN = 0x08000000+32K,        LENGTH = 16K
    param2 (rx) :         ORIGIN = 0x08000000+48K,        LENGTH = 16K
    app (rx) :            ORIGIN = 0x08000000+64K,        LENGTH = 2M-64K
    ram (rwx) :           ORIGIN = 0x20000000,            LENGTH = 192K-256
    app_bl_shared (rwx) : ORIGIN = 0x20000000+(192K-256), LENGTH = 256
    /* Dummy regions to satisfy generic linker rules; not used on this MCU. */
    ram1 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram2 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram3 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram4 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram5 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram6 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram7 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
}
