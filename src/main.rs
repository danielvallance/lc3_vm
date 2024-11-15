//! This crate contains a Rust implementation of a VM
//! for the LC-3 architecture
//!
//! It follows the tutorial for doing this in C here:
//! https://www.jmeiners.com/lc3-vm/ and I am doing
//! this as a learning exercise

use core::ascii;
use std::{
    env,
    error::Error,
    fs::File,
    io::{stdin, stdout, Read, Write},
    process::exit,
};

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

/* Trap codes */
const TRAP_GETC: u16 = 0x20; /* Get character from keyboard, not echoed onto the terminal */
const TRAP_OUT: u16 = 0x21; /* Output a character */
const TRAP_PUTS: u16 = 0x22; /* Output a word string */
const TRAP_IN: u16 = 0x23; /* Get character from keyboard, echoed onto the terminal */
const TRAP_PUTSP: u16 = 0x24; /* Output a byte string */
const TRAP_HALT: u16 = 0x25; /* Halt the program */

const EOF: i16 = -1;

/// Loads encoded LC3 assembly from file to memory
fn read_image(image_path: &str, memory: &mut [u16]) -> Result<(), Box<dyn Error>> {
    /* Read image file */
    let mut image_file = File::open(image_path)?;
    let mut buf = Vec::new();
    let len = image_file.read_to_end(&mut buf)?;

    /* Image must be at least 2 bytes, and an even number of bytes */
    if len < 2 || len % 2 == 1 {
        Err(format!("Image at {} is not valid LC3 format\n", image_path))?
    }

    /* Get origin which indicates memory location to which the instructions in the image should be loaded */
    let origin: u16 = buf_to_little_endian_u16(&buf[..2]);

    /* If image size cannot be loaded into memory, fail */
    if len > MEMORY_MAX - origin as usize {
        Err(format!("Not enough memory for image: {}\n", image_path))?
    }

    for (idx, instruction) in buf[2..].chunks(2).enumerate() {
        memory[origin as usize + idx] = buf_to_little_endian_u16(instruction)
    }

    Ok(())
}

/// Convert big-endian u16 in buffer to little-endian u16
fn buf_to_little_endian_u16(buf: &[u8]) -> u16 {
    buf[1] as u16 | buf[0] as u16
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

/// Gets character from standard input, or returns EOF
fn get_char() -> Result<i16, Box<dyn Error>> {
    match stdin().bytes().next() {
        Some(ch) => Ok(ch? as i16),
        None => Ok(EOF), /* If no character on stdin, return EOF */
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() != 2 {
        println!("Usage: lc3_vm <image_file>\n");
        exit(1);
    }

    /* Memory is stored in this array */
    let mut memory: [u16; MEMORY_MAX] = [0; MEMORY_MAX];

    /* Register values are stored in this array */
    let mut registers: [u16; RCOUNT] = [0; RCOUNT];

    /* Read image file */
    if let Err(e) = read_image(&args[1], &mut memory) {
        println!("Could not read image: {}\n", e);
        exit(1);
    }

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
            OP_AND => {
                let dst_reg = ((instruction >> 9) & 0x7) as usize; /* Where result will be stored */
                let src_reg_1 = ((instruction >> 6) & 0x7) as usize; /* Where the first operand is */
                let imm_mode = ((instruction >> 5) & 0x1) == 1; /* Whether immediate mode is being used */

                if imm_mode {
                    /* Immediate mode means the second operand is encoded in the instruction itself */
                    let imm_operand = sign_extend(instruction & 0x1f, 5);
                    registers[dst_reg] = registers[src_reg_1] & imm_operand;
                } else {
                    /* If immediate mode is not being used, the instruction refers to a second destination register */
                    let src_reg_2 = (instruction & 0x7) as usize;
                    registers[dst_reg] = registers[src_reg_1] & registers[src_reg_2];
                }

                update_flags(registers[dst_reg], &mut registers[RCOND]);
            }
            OP_NOT => {
                let dst_reg = ((instruction >> 9) & 0x7) as usize; /* Where result will be stored */
                let src_reg_1 = ((instruction >> 6) & 0x7) as usize; /* Where the operand is */

                /* Calculate bitwise NOT, store in dst_reg, and update flags */
                registers[dst_reg] = !registers[src_reg_1];
                update_flags(registers[dst_reg], &mut registers[RCOND]);
            }
            OP_BR => {
                /* These bits in the instruction define the condition flags being tested */
                let condition_flags = (instruction >> 9) & 0x7;

                /* The flags are in the same order in the instruction and the register */
                if (condition_flags & registers[RCOND]) != 0 {
                    /* Branch by adding the sign extended PC offset to the PC */
                    let pc_offset = sign_extend(instruction & 0x1ff, 9);
                    registers[RPC] += pc_offset;
                }
            }
            OP_JMP => {
                let base_reg = ((instruction >> 6) & 0x7) as usize; /* Register which contains position to jump to */
                registers[RPC] = registers[base_reg]; /* Jump to specified location */
            }
            OP_JSR => {
                registers[R7] = registers[RPC]; /* Save PC in R7 */

                /* If this bit is 0, jump to address in the register specified in the instruction */
                if (instruction & (1 << 11)) == 0 {
                    let base_reg = ((instruction >> 6) & 0x7) as usize;
                    registers[RPC] = registers[base_reg];
                /* If the bit is 1, jump to the address encoded in the instruction itself */
                } else {
                    let pc_offset = sign_extend(instruction & 0x7ff, 11);
                    registers[RPC] += pc_offset;
                }
            }
            OP_LD => {
                let dst_reg = ((instruction >> 9) & 0x7) as usize; /* Register where value will be loaded */

                /* Get PC offset, add to PC, and load value at the resulting memory location to dst_reg */
                let pc_offset = sign_extend(instruction & 0x1ff, 9);
                registers[dst_reg] = memory[(registers[RPC] + pc_offset) as usize];

                update_flags(registers[dst_reg], &mut registers[RCOND]);
            }
            OP_LDI => {
                let dst_reg = ((instruction >> 9) & 0x7) as usize; /* Register where value will be loaded */
                /* Will be added to PC to find address of address of data to be loaded */
                let pc_offset = sign_extend(instruction & 0x1FF, 9);

                /* Load value at address referred to in address of the PC, combined with pc_offset */
                registers[dst_reg] = memory[memory[(registers[RPC] + pc_offset) as usize] as usize];
                update_flags(registers[dst_reg], &mut registers[RCOND]);
            }
            OP_LDR => {
                let dst_reg = ((instruction >> 9) & 0x7) as usize; /* Register where value will be loaded */

                /* The value in the base register, added to an offset, point to the value to be loaded */
                let base_reg = ((instruction >> 6) & 0x7) as usize;
                let offset = sign_extend(instruction & 0x3f, 6);
                registers[dst_reg] = memory[(registers[base_reg] + offset) as usize];

                update_flags(registers[dst_reg], &mut registers[RCOND]);
            }
            OP_LEA => {
                let dst_reg = ((instruction >> 9) & 0x7) as usize; /* Register where value will be loaded */

                /* Get PC offset from the instruction, and add to PC to get address of value */
                let pc_offset = sign_extend(instruction & 0x1ff, 9);
                registers[dst_reg] = memory[(registers[RPC] + pc_offset) as usize];

                update_flags(registers[dst_reg], &mut registers[RCOND]);
            }
            OP_ST => {
                let src_reg = ((instruction >> 9) & 0x7) as usize; /* Register containing value to be stored */

                /* Destination address calculated by adding pc_offset to PC */
                let pc_offset = sign_extend(instruction & 0x1ff, 9);
                memory[(registers[RPC] + pc_offset) as usize] = registers[src_reg];
            }
            OP_STI => {
                let src_reg = ((instruction >> 9) & 0x7) as usize; /* Register containing value to be stored */

                /* Add pc_offset to PC to get address of address at which value should be stored */
                let pc_offset = sign_extend(instruction & 0x1ff, 9);
                memory[memory[(registers[RPC] + pc_offset) as usize] as usize] = registers[src_reg];
            }
            OP_STR => {
                let src_reg = ((instruction >> 9) & 0x7) as usize; /* Register containing value to be stored */

                /* Destination address calculated by adding offset to base_reg */
                let base_reg = ((instruction >> 6) & 0x7) as usize;
                let offset = sign_extend(instruction & 0x3f, 6);
                memory[(registers[base_reg] + offset) as usize] = registers[src_reg];
            }
            OP_TRAP => {
                registers[R7] = registers[RPC]; /* Store program counter */

                match instruction & 0xff {
                    TRAP_GETC => {
                        /* Get u8 ascii character from standard input */
                        registers[R0] = match get_char() {
                            Ok(EOF) => 0xffff, /* EOF */
                            Ok(ch) => ch as u16,
                            Err(_) => {
                                println!("Could not execute getc trap. Quitting.\n");
                                running = false;
                                break;
                            }
                        };

                        update_flags(registers[R0], &mut registers[RCOND]);
                    }
                    TRAP_OUT => {
                        /* C implementation uses putc, so I decided to only treat ascii characters here */
                        print!("{}", ascii::escape_default(registers[R0] as u8));
                        /* Attempt to flush */
                        if stdout().flush().is_err() {
                            println!("Could not execute out trap. Quitting.\n");
                            running = false;
                        }
                    }
                    TRAP_PUTS => {
                        /*
                         * Iterate though NULL terminated string where the first
                         * character is stored at address in R0
                         */
                        for &ch in memory[registers[R0] as usize..].iter() {
                            if ch == 0 {
                                break;
                            }
                            /*
                             * C implementation uses putc, so I decided to only treat ascii characters here
                             *
                             * Specification of this trap states that there is one ascii character per 16 bit word
                             */
                            print!("{}", ascii::escape_default(ch as u8));
                        }

                        /* Attempt to flush */
                        if stdout().flush().is_err() {
                            println!("Could not execute puts trap. Quitting.\n");
                            running = false;
                        }
                    }
                    TRAP_IN => {
                        println!("Enter a character: ");
                        let ch = match get_char() {
                            Ok(EOF) => break,
                            Ok(ch) => ch as u16,
                            Err(_) => {
                                println!("Could not execute in trap. Quitting.\n");
                                running = false;
                                break;
                            }
                        } as u8;

                        /* C implementation uses putc, so I decided to only treat ascii characters here */
                        print!("{}", ascii::escape_default(registers[R0] as u8));

                        /* Attempt to flush */
                        if stdout().flush().is_err() {
                            println!("Could not execute in trap. Quitting.\n");
                            running = false;
                        }

                        registers[R0] = ch as u16;
                        update_flags(registers[R0], &mut registers[RCOND]);
                    }
                    TRAP_PUTSP => {
                        /*
                         * Iterate though NULL terminated string where the first
                         * character is stored at address in R0
                         */
                        for &word in memory[registers[R0] as usize..].iter() {
                            let char1 = (word & 0xff) as u8;
                            if char1 == 0 {
                                break;
                            }
                            /*
                             * C implementation uses putc, so I decided to only treat ascii characters here
                             *
                             * Specification of this trap states that there are two ascii characters per 16 bit word
                             */
                            print!("{}", ascii::escape_default(char1));

                            let char2 = (word >> 8) as u8;
                            if char2 == 0 {
                                break;
                            }
                            print!("{}", ascii::escape_default(char1));
                        }

                        /* Attempt to flush */
                        if stdout().flush().is_err() {
                            println!("Could not execute putsp trap. Quitting.\n");
                            running = false;
                        }
                    }
                    TRAP_HALT => {
                        println!("Halting.");
                        running = false;
                    }
                    _ => exit(1), /* Trap code not implemented */
                }
            }
            OP_RES => exit(1), /* Not implemented */
            OP_RTI => exit(1), /* Not implemented */
            _ => exit(1),
        }
        running = false; /* Terminate the loop */
    }
}
