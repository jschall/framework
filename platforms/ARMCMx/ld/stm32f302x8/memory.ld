MEMORY
{
    bl (rx) :             ORIGIN = 0x08000000,            LENGTH = 12K
    app (rx) :            ORIGIN = 0x08000000+12K,        LENGTH = 64K-12K-4K
    param1 (rx) :         ORIGIN = 0x08000000+60K,        LENGTH = 2K
    param2 (rx) :         ORIGIN = 0x08000000+62K,        LENGTH = 2K
    ram (rwx) :           ORIGIN = 0x20000000,            LENGTH = 16K-256
    app_bl_shared (rwx) : ORIGIN = 0x20000000+(16K-256),  LENGTH = 256
    /* Dummy regions to satisfy generic linker rules; unused on F3. */
    ram1 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram2 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram3 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram4 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram5 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram6 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
    ram7 (rwx) :          ORIGIN = 0x00000000,            LENGTH = 0
}
