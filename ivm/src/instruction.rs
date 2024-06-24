use core::fmt::{self, Debug};

use crate::{
  ivm::IVM,
  port::{Port, Tag},
};

/// A list of instructions to create a net.
#[derive(Debug)]
pub struct Instructions<'ivm> {
  /// The next unused register value; equivalently, the number of used
  /// registers.
  pub(crate) next_register: Register,
  /// The actual list of instructions; see [`Instructions::push`] for the safety
  /// requirements of these instructions.
  pub(crate) instructions: Vec<Instruction<'ivm>>,
}

impl<'ivm> Default for Instructions<'ivm> {
  fn default() -> Self {
    Self { next_register: Register::new(1), instructions: Default::default() }
  }
}

impl<'ivm> Instructions<'ivm> {
  /// Returns a new, unused register.
  #[inline(always)]
  pub fn new_register(&mut self) -> Register {
    let register = self.next_register;
    self.next_register.byte_offset += REGISTER_SIZE;
    register
  }

  /// Appends an instruction to the list.
  ///
  /// ## Safety
  /// All registers used in the instruction must either be `Register::ROOT` or
  /// have been returned by `self.new_register()`.
  #[inline(always)]
  pub unsafe fn push(&mut self, instruction: Instruction<'ivm>) {
    self.instructions.push(instruction);
  }

  #[inline(always)]
  pub fn instructions(&self) -> &[Instruction<'ivm>] {
    &self.instructions
  }

  #[inline(always)]
  pub unsafe fn instructions_mut(&mut self) -> &mut Vec<Instruction<'ivm>> {
    &mut self.instructions
  }
}

/// An instruction to create a small sub-net.
///
/// It is not unsafe to construct invalid an invalid instruction, but it is
/// unsafe to add it to [`Instructions`] (which is assumed to consist only of
/// valid instructions).
#[derive(Debug)]
pub enum Instruction<'ivm> {
  /// Create a nilary node.
  ///
  /// ## Validity
  /// Its port must truly be nilary (and thus safe to clone).
  Nilary(Register, Port<'ivm>),
  /// Create a binary node with a given tag and label.
  ///
  /// ## Validity
  /// For this instruction to be safely pushed to `Instructions`, its tag must
  /// be that of a binary node, and its label must comply with the tag's
  /// requirements.
  Binary(Tag, u16, Register, Register, Register),
}

/// A "register" of an [`Instruction`], used to identify pairs of ports to link.
///
/// Serves as an index into the `registers` field in an [`IVM`].
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Register {
  /// The byte offset corresponding to the index; equivalent to `index *
  /// REGISTER_SIZE`.
  ///
  /// Stored as a `u32` to keep the size of an [`Instruction`] small.
  pub(crate) byte_offset: u32,
}

/// The size of the data referenced by a register.
const REGISTER_SIZE: u32 = 8;

impl Register {
  /// Register 0 is reserved for the root of a net.
  pub const ROOT: Register = Register::new(0);

  /// Creates a new register corresponding to the given index.
  #[inline(always)]
  const fn new(index: usize) -> Self {
    Register { byte_offset: index as u32 * REGISTER_SIZE }
  }

  /// Returns the index of this register.
  #[inline(always)]
  pub fn index(&self) -> usize {
    (self.byte_offset / REGISTER_SIZE) as usize
  }
}

impl Debug for Register {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(f, "Register({})", self.index())
  }
}

impl<'ivm> IVM<'ivm> {
  /// Links the given `port` to the given `register`.
  ///
  /// ## Safety
  /// `register.index() < self.registers.len()`
  #[inline]
  unsafe fn link_register(&mut self, register: Register, port: Port<'ivm>) {
    let register = self.get_register(register);
    if let Some(got) = register.take() {
      self.link(port, got);
    } else {
      *register = Some(port);
    }
  }

  unsafe fn get_register(&mut self, register: Register) -> &mut Option<Port<'ivm>> {
    debug_assert!(register.index() < self.registers.len());
    &mut *(&mut *self.registers as *mut [_] as *mut Option<Port<'ivm>>)
      .byte_offset(register.byte_offset as isize)
  }

  /// Execute [`Instructions`], linking the net's root to `port`.
  pub fn execute(&mut self, instructions: &Instructions<'ivm>, port: Port<'ivm>) {
    let needed_registers = instructions.next_register.index().max(1);
    if needed_registers > self.registers.len() {
      self.registers.resize_with(needed_registers, || None)
    }

    unsafe { self.link_register(Register::ROOT, port) };

    for i in &instructions.instructions {
      unsafe {
        match *i {
          Instruction::Nilary(r, ref p) => self.link_register(r, p.clone()),
          Instruction::Binary(tag, label, p0, p1, p2) => {
            let r0p = &mut *(self.get_register(p0) as *mut Option<Port<'ivm>>);
            if let Some(r0) = r0p {
              *r0 = self.follow(r0.clone());
              if r0.tag() == tag && r0.label() == label {
                let (a, b) = r0.clone().aux();
                self.link_register(p1, Port::new_wire(a));
                self.link_register(p2, Port::new_wire(b));
                *r0p = None;
                self.stats.annihilate += 1;
                self.stats.fast += 1;
                continue;
              }
            }
            let node = self.new_node(tag, label);
            self.link_register(p0, node.0);
            self.link_register(p1, Port::new_wire(node.1));
            self.link_register(p2, Port::new_wire(node.2));
          }
        }
      }
    }

    // Using registers an odd number of times will cause severe logic errors.
    if cfg!(debug_assertions) {
      for r in &self.registers {
        assert!(r.is_none());
      }
    }
  }
}
