#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut buffer = [0u8; 16];
    for (i, &b) in data.iter().enumerate() {
        if i < buffer.len() {
            buffer[i] = b;
        }
    }
});