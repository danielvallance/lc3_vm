//! This crate contains a Rust implementation of a VM
//! for the LC-3 architecture
//!
//! It follows the tutorial for doing this in C here:
//! https://www.jmeiners.com/lc3-vm/ and I am doing
//! this as a learning exercise

/* The LC-3 architecture contains 65536 memory locations */
const MEMORY_MAX: usize = 1 << 16;

/*
 * The LC-3 architecture has 10 registers (8 general purpose,
 * a program counter, and a condition flag register)
 *
 * They are all 16 bits (the size of a word in this architecture)
 */
/* General purpose registers */
const R0: u16 = 0;
const R1: u16 = 1;
const R2: u16 = 2;
const R3: u16 = 3;
const R4: u16 = 4;
const R5: u16 = 5;
const R6: u16 = 6;
const R7: u16 = 7;
const RPC: u16 = 8; /* Program counter */
const RCOND: u16 = 9; /* Condition flag register */
const RCOUNT: u16 = 10; /* UNUSED: Number of registers */

/* Program Counter's default starting position */
const PC_START: u16 = 0x3000;

/* The LC-3 architecture's instruction set supports the following opcodes */
const OP_BR: u16 = 0; /* Branch */
const OP_ADD: u16 = 1; /* Add  */
const OP_LD: u16 = 2; /* Load */
const OP_ST: u16 = 3; /* Store */
const OP_JSR: u16 = 4; /* Jump register */
const OP_AND: u16 = 5; /* Bitwise and */
const OP_LDR: u16 = 6; /* Load register */
const OP_STR: u16 = 7; /* Store register */
const OP_RTI: u16 = 8; /* Unused */
const OP_NOT: u16 = 9; /* Bitwise not */
const OP_LDI: u16 = 10; /* Load indirect */
const OP_STI: u16 = 11; /* Store indirect */
const OP_JMP: u16 = 12; /* Jump */
const OP_RES: u16 = 13; /* Reserved (unused) */
const OP_LEA: u16 = 14; /* Load effective address */
const OP_TRAP: u16 = 15; /* Execute trap */
const OP_COUNT: u16 = 16; /* UNUSED: Number of opcodes */

/*
 * The LC-3 architecture uses the following flags
 * to represent the sign of the result of the previous
 * calculation
 *
 * These flags will be stored in the condition flag
 * register
 */
const FL_POS: u16 = 1 << 0; /* Positive */
const FL_ZRO: u16 = 1 << 1; /* Zero */
const FL_NEG: u16 = 1 << 2; /* Negative */

fn main() {
    /* Memory is stored in this array */
    let mut memory: [u16; MEMORY_MAX] = [0; MEMORY_MAX];

    /* Register values are stored in this array */
    let mut registers: [u16; RCOUNT as usize] = [0; RCOUNT as usize];

    /*
     * Exactly one condition flag is set in RCOND
     * at any given moment. Initially this will be FL_ZERO
     */
    registers[RCOND as usize] = FL_ZRO;

    /* Set PC to default starting position */
    registers[RPC as usize] = PC_START;

    /* Main fetch-execute cycle loop */
    let mut running = true;
    while running {
        /* Fetch instruction at PC's address */
        let instruction = memory[registers[RPC as usize] as usize];

        /* Get opcode which is stored in first 4 bits of instruction */
        let op = instruction >> 12;

        match op {
            OP_ADD => (),
            OP_AND => (),
            OP_NOT => (),
            OP_BR => (),
            OP_JMP => (),
            OP_JSR => (),
            OP_LD => (),
            OP_LDI => (),
            OP_LDR => (),
            OP_LEA => (),
            OP_ST => (),
            OP_STI => (),
            OP_STR => (),
            OP_TRAP => (),
            OP_RES => (),
            OP_RTI => (),
            _ => (),
        }
        running = false; /* Terminate the loop */
    }
}
