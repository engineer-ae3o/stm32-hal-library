MEMORY
{
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    FLASH (rx): ORIGIN = 0x08000000, LENGTH = 512K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(reset_handler)

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        *(.isr_vector*)
        KEEP(*(.isr_vector));
    } > FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text*)
        *(.init*)
        *(.fini*)
    } > FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata*)
    } > FLASH

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM AT > FLASH

    _sidata = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM
    
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _sheap = .;
    } > RAM
}
