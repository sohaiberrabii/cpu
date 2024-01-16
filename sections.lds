MEMORY {
    mem : ORIGIN = 0x0, LENGTH = 0x20000 /* 32768 words, 128kB */
}

SECTIONS {
    .memory 0x0 : {
        start*(.text)
        *(.text)
        *(*)
        end = .;
        . = ALIGN(4);
    } > mem
}

