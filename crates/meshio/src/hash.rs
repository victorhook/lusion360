//! Tiny fast hasher (FxHash-style with a finalizer) and position-welding helpers.

use std::collections::HashMap;
use std::hash::{BuildHasherDefault, Hash, Hasher};

const SEED: u64 = 0x51_7c_c1_b7_27_22_0a_95;

#[derive(Default, Clone, Copy)]
pub(crate) struct FxHasher {
    hash: u64,
}

impl FxHasher {
    #[inline]
    fn add(&mut self, i: u64) {
        self.hash = (self.hash.rotate_left(5) ^ i).wrapping_mul(SEED);
    }
}

impl Hasher for FxHasher {
    #[inline]
    fn write(&mut self, bytes: &[u8]) {
        let (chunks, rem) = bytes.as_chunks::<8>();
        for c in chunks {
            self.add(u64::from_le_bytes(*c));
        }
        if !rem.is_empty() {
            let mut buf = [0u8; 8];
            buf[..rem.len()].copy_from_slice(rem);
            self.add(u64::from_le_bytes(buf));
        }
    }
    #[inline]
    fn write_u8(&mut self, i: u8) {
        self.add(i as u64)
    }
    #[inline]
    fn write_u32(&mut self, i: u32) {
        self.add(i as u64)
    }
    #[inline]
    fn write_u64(&mut self, i: u64) {
        self.add(i)
    }
    #[inline]
    fn write_usize(&mut self, i: usize) {
        self.add(i as u64)
    }
    #[inline]
    fn finish(&self) -> u64 {
        // fmix64-style finalizer so that both low and high bits are well distributed
        // (float bit patterns often differ only in their high bits).
        let mut h = self.hash;
        h ^= h >> 33;
        h = h.wrapping_mul(0xff51_afd7_ed55_8ccd);
        h ^= h >> 33;
        h
    }
}

pub(crate) type FxBuild = BuildHasherDefault<FxHasher>;
pub(crate) type FxHashMap<K, V> = HashMap<K, V, FxBuild>;

pub(crate) fn map_with_capacity<K, V>(cap: usize) -> FxHashMap<K, V> {
    HashMap::with_capacity_and_hasher(cap, FxBuild::default())
}

/// Exact-position key (bit pattern; -0.0 is folded onto +0.0).
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct PosKey(u64, u32);

impl PosKey {
    #[inline]
    pub(crate) fn new(p: [f32; 3]) -> Self {
        // `x + 0.0` turns -0.0 into +0.0 and leaves everything else unchanged.
        let b = |x: f32| (x + 0.0).to_bits();
        PosKey(((b(p[0]) as u64) << 32) | b(p[1]) as u64, b(p[2]))
    }
}

impl Hash for PosKey {
    #[inline]
    fn hash<H: Hasher>(&self, state: &mut H) {
        state.write_u64(self.0);
        state.write_u32(self.1);
    }
}

/// Incremental vertex welder: maps exact positions to shared indices.
pub(crate) struct Welder {
    map: FxHashMap<PosKey, u32>,
    pub positions: Vec<[f32; 3]>,
}

impl Welder {
    pub(crate) fn with_capacity(vertices: usize) -> Self {
        Welder {
            map: map_with_capacity(vertices),
            positions: Vec::with_capacity(vertices),
        }
    }

    #[inline]
    pub(crate) fn index(&mut self, p: [f32; 3]) -> u32 {
        let next = self.positions.len() as u32;
        let idx = *self.map.entry(PosKey::new(p)).or_insert(next);
        if idx == next {
            self.positions.push(p);
        }
        idx
    }
}
