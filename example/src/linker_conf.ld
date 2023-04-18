MEMORY {
    flash : org = 0x08000000, len = 1024k
    ram : org = 0x20000000, len = 128k
}

SECTIONS {
    . = ORIGIN(flash);
        .text : {
            *(.vectors);
            *(.text);
        } >flash
    . = ORIGIN(ram);
        .data : {
            INIT_DATA_VALUES = LOADADDR(.data);
            INIT_DATA_START = .;
            *(.data);
            INIT_DATA_END = .;
        } >ram AT>flash
    BSS_START = .;
    .bss : {
        *(.bss);
    } >ram
    BSS_END = .;
}