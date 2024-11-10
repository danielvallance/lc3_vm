//! This crate contains a Rust implementation of a VM
//! for the LC-3 architecture
//!
//! It follows the tutorial for doing this in C here:
//! https://www.jmeiners.com/lc3-vm/ and I am doing
//! this as a learning exercise

use std::{env, fs::read, process::exit};

/* The LC-3 architecture contains 65536 memory locations */
const MEMORY_MAX: usize = 1 << 16;

/*
 * The LC-3 architecture has 10 registers (8 general purpose,
 * a program counter, and a condition flag register)
 *
 * They are all 16 bits (the size of a word in this architecture)
 */
/* General purpose registers */
const R0: usize = 0;
const R1: usize = 1;
const R2: usize = 2;
const R3: usize = 3;
const R4: usize = 4;
const R5: usize = 5;
const R6: usize = 6;
const R7: usize = 7;
const RPC: usize = 8; /* Program counter */
const RCOND: usize = 9; /* Condition flag register */
const RCOUNT: usize = 10; /* UNUSED: Number of registers */

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

fn read_image(_image_path: &str) -> bool {
    true
}

fn sign_extend(mut operand: u16, no_of_bits: u8) -> u16 {
    if (operand >> (no_of_bits - 1)) == 1 {
        operand |= 0xFFFF << no_of_bits
    }
    operand
}

fn update_flags(new_reg_value: u16, r_cond: &mut u16) {
    if new_reg_value == 0 {
        *r_cond = FL_ZRO;
    /* MSB being 1 indicates a negative value */
    } else if (new_reg_value >> 15) == 1 {
        *r_cond = FL_NEG;
    } else {
        *r_cond = FL_POS;
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        println!("Usage: lc3_vm <image_file_1> [another_image_file]*");
        exit(1);
    }

    /* Read image file(s) */
    for arg in args {
        if !read_image(&arg) {
            println!("Could not read image: {}", arg);
            exit(1);
        }
    }
    /* Memory is stored in this array */
    let mut memory: [u16; MEMORY_MAX] = [0; MEMORY_MAX];

    /* Register values are stored in this array */
    let mut registers: [u16; RCOUNT] = [0; RCOUNT];

    /*
     * Exactly one condition flag is set in RCOND
     * at any given moment. Initially this will be FL_ZERO
     */
    registers[RCOND] = FL_ZRO;

    /* Set PC to default starting position */
    registers[RPC] = PC_START;

    /* Main fetch-execute cycle loop */
    let mut running = true;
    while running {
        /* Fetch instruction at PC's address */
        let instruction = memory[registers[RPC] as usize];

        /* Get opcode which is stored in first 4 bits of instruction */
        let op = instruction >> 12;

        /*
         * Execute the instruction. Specifications for each instruction
         * can be found here: https://www.jmeiners.com/lc3-vm/supplies/lc3-isa.pdf
         */
        match op {
            OP_ADD => {
                let dst_reg = ((instruction >> 9) & 0x7) as usize; /* Where result will be stored */
                let src_reg_1 = ((instruction >> 6) & 0x7) as usize; /* Where the first operand is */
                let imm_mode = ((instruction >> 5) & 0x1) == 1; /* Whether immediate mode is being used */

                if imm_mode {
                    /* Immediate mode means the second operand is encoded in the instruction itself */
                    let imm_operand = sign_extend(instruction & 0x1f, 5);
                    registers[dst_reg] = registers[src_reg_1] + imm_operand;
                } else {
                    /* If immediate mode is not being used, the instruction refers to a second destination register */
                    let src_reg_2 = (instruction & 0x7) as usize;
                    registers[dst_reg] = registers[src_reg_1] + registers[src_reg_2];
                }

                update_flags(registers[dst_reg], &mut registers[RCOND]);
            }
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
