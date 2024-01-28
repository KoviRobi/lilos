MEMORY {
  /* NOTE K = KiB = 1024 bytes */
  BOOT2  (rx)  : ORIGIN = 0x10000000, LENGTH = 256
  FLASH  (rx)  : ORIGIN = 0x10000100, LENGTH = 2M - 256
  RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 264K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    /* ### Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;
