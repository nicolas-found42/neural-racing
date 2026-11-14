//! Safe offset-preserving storage for recovered native car state. The immutable
//! car-data allocation is separate. No host pointer or unsafe memory is exposed.

pub const CAR_BYTES: usize = 0x1100;
pub const WHEEL_BASE: usize = 0x164;
pub const WHEEL_STRIDE: usize = 0xec;
pub const WHEEL_COUNT: usize = 4;

#[derive(Clone)]
pub struct NativeCar {
    bytes: [u8; CAR_BYTES],
}
impl Default for NativeCar {
    fn default() -> Self {
        Self {
            bytes: [0; CAR_BYTES],
        }
    }
}

macro_rules! readers {
    () => {
        pub(crate) fn u8(&self, offset: usize) -> u8 {
            self.bytes[offset]
        }
        pub(crate) fn i8(&self, offset: usize) -> i8 {
            self.bytes[offset] as i8
        }
        pub(crate) fn u16(&self, offset: usize) -> u16 {
            u16::from_le_bytes(self.bytes[offset..offset + 2].try_into().unwrap())
        }
        pub(crate) fn i16(&self, offset: usize) -> i16 {
            i16::from_le_bytes(self.bytes[offset..offset + 2].try_into().unwrap())
        }
        pub(crate) fn u32(&self, offset: usize) -> u32 {
            u32::from_le_bytes(self.bytes[offset..offset + 4].try_into().unwrap())
        }
        pub(crate) fn f32(&self, offset: usize) -> f32 {
            f32::from_bits(self.u32(offset))
        }
    };
}
macro_rules! writers {
    () => {
        pub(crate) fn set_u8(&mut self, offset: usize, value: u8) {
            self.bytes[offset] = value;
        }
        pub(crate) fn set_i8(&mut self, offset: usize, value: i8) {
            self.bytes[offset] = value as u8;
        }
        pub(crate) fn set_u16(&mut self, offset: usize, value: u16) {
            self.bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        pub(crate) fn set_i16(&mut self, offset: usize, value: i16) {
            self.bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        pub(crate) fn set_u32(&mut self, offset: usize, value: u32) {
            self.bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        pub(crate) fn set_f32(&mut self, offset: usize, value: f32) {
            self.set_u32(offset, value.to_bits());
        }
    };
}
impl NativeCar {
    readers!();
    writers!();
    pub(crate) fn wheel(&self, index: usize) -> NativeWheel<'_> {
        assert!(index < WHEEL_COUNT);
        let base = WHEEL_BASE + index * WHEEL_STRIDE;
        NativeWheel {
            bytes: &self.bytes[base..base + WHEEL_STRIDE],
        }
    }
    pub(crate) fn wheel_mut(&mut self, index: usize) -> NativeWheelMut<'_> {
        assert!(index < WHEEL_COUNT);
        let base = WHEEL_BASE + index * WHEEL_STRIDE;
        NativeWheelMut {
            bytes: &mut self.bytes[base..base + WHEEL_STRIDE],
        }
    }
    pub(crate) fn bytes(&self) -> &[u8; CAR_BYTES] {
        &self.bytes
    }
    pub(crate) fn from_bytes(bytes: [u8; CAR_BYTES]) -> Self {
        Self { bytes }
    }
}

pub(crate) struct NativeWheel<'a> {
    bytes: &'a [u8],
}
impl NativeWheel<'_> {
    readers!();
}
pub(crate) struct NativeWheelMut<'a> {
    bytes: &'a mut [u8],
}
impl NativeWheelMut<'_> {
    readers!();
    writers!();
}
