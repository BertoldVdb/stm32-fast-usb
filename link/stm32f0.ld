OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
ENTRY(ResetHandler)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 0x2000
    RAM (rwx) :  ORIGIN = 0x20000000, LENGTH = 8K
}

SECTIONS {
    __stack_base = ORIGIN(RAM) + LENGTH(RAM);

    . = ORIGIN(FLASH);

    .stack_base : {
        LONG(__stack_base);
    } >FLASH

    .vectors : {
        KEEP(*(.vectors))
    } > FLASH

    .text : {
        . = ALIGN(4);
        *(.text*)
        . = ALIGN(4);
        *(.rodata*)
    } >FLASH

    . = ALIGN(4);
    __data_flash_begin = LOADADDR(.data);

    . = ORIGIN(RAM);
    .data : {
        . = ALIGN(4);
        __data_ram_begin = .;
        *(.data)
        *(.data*)
        *(.ramcode)
        *(.ramcode*)
        . = ALIGN(4);
        __data_ram_end = .;
    } >RAM AT >FLASH

    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_ram_begin = .;
        *(.bss)
        *(.bss*)
        . = ALIGN(4);
        __bss_ram_end = .;
    } >RAM

    .noinit (NOLOAD) : {
        *(.noinit)
        *(.noinit*)
    } >RAM
}

