MEMORY
{
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
    FLASH (rx): ORIGIN = 0x08000000, LENGTH = 512K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(reset_handler)

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector*));
        
        . = ALIGN(4);
        *(.text*)
        
        __init_array_start = .;
        KEEP(*(SORT(.init_array*)))
        __init_array_end = .;

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
        _ebss = .;
    } > RAM
    
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        _sheap = .;
    } > RAM
}
