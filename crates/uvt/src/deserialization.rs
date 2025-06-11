#[derive(Debug, Clone, PartialEq)]
pub struct MessageDataBuffer {
    // Buffer Data
    data: Vec<u8>,
    // Position
    position: usize,
}

impl MessageDataBuffer {
    /// Instantiate a new MessageDataBuffer from a Vec<u8>
    pub fn new(data: Vec<u8>) -> Self {
        Self { data, position: 0 }
    }

    /// Read the byte at position `position`
    pub fn seek(&self, position: usize) -> Option<u8> {
        if position > self.data.len() {
            None
        } else {
            Some(self.data[position])
        }
    }

    /// Retrieve a slice of length `length` from the buffer
    pub fn slice(&mut self, length: usize) -> Option<&[u8]> {
        if (self.position + length) > self.data.len() {
            return None;
        }
        let bytes: &[u8] = self.data[self.position..self.position + length]
            .try_into()
            .ok()?;
        self.position += length;
        Some(bytes)
    }

    /// Read a u32 from the buffer
    pub fn read_u32_le(&mut self) -> Option<u32> {
        self.slice(4)
            .and_then(|bytes| Some(u32::from_le_bytes(bytes.try_into().ok()?)))
    }
    /// Read a u16 from the buffer
    pub fn read_u16_le(&mut self) -> Option<u16> {
        self.slice(2)
            .and_then(|bytes| Some(u16::from_le_bytes(bytes.try_into().ok()?)))
    }
    /// Read a i32 from the buffer
    pub fn read_i32_le(&mut self) -> Option<i32> {
        self.slice(4)
            .and_then(|bytes| Some(i32::from_le_bytes(bytes.try_into().ok()?)))
    }
    /// Read a i16 from the buffer
    pub fn read_i16_le(&mut self) -> Option<i16> {
        self.slice(2)
            .and_then(|bytes| Some(i16::from_le_bytes(bytes.try_into().ok()?)))
    }
    /// Read a f64 from the buffer
    pub fn read_f64_le(&mut self) -> Option<f64> {
        self.slice(8)
            .and_then(|bytes| Some(f64::from_le_bytes(bytes.try_into().ok()?)))
    }
    /// Read a f32 from the buffer
    pub fn read_f32_le(&mut self) -> Option<f32> {
        self.slice(2)
            .and_then(|bytes| Some(f32::from_le_bytes(bytes.try_into().ok()?)))
    }

    /// Read a length-prefixed UTF-8 string from the buffer (4-byte LE length + bytes)
    pub fn read_lp_string(&mut self) -> Option<String> {
        let strlen = self.read_u32_le()? as usize;
        let bytes = self.slice(strlen)?;
        Some(str::from_utf8(bytes).ok()?.to_owned())
    }

    /// Read a i16 from the buffer
    pub fn read_byte(&mut self) -> Option<u8> {
        Some(self.slice(1).unwrap()[0])
    }
}
