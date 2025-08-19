use crate::pose;
use std::fs::{File, create_dir_all};
use std::io::{self, Error, ErrorKind, Write};
use std::path::Path;

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

    /// Get total length of buffer
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Get number of remaining bytes in buffer
    pub fn n_remaining(&self) -> usize {
        self.len() - self.position
    }

    /// Dump data to file
    pub fn dump_to_file(&self, path: &str) -> io::Result<()> {
        let path = Path::new(&path);

        // Ensure the parent directory exists
        if let Some(parent) = path.parent() {
            create_dir_all(parent)?;
        }

        let mut file = File::create(path)?;
        let bytes = &self.data;

        for (i, chunk) in bytes.chunks(16).enumerate() {
            // Hex section
            write!(file, "{:08x}:", i * 16)?;
            for b in chunk {
                write!(file, " {:02x}", b)?;
            }

            // Pad to align ASCII section
            for _ in chunk.len()..16 {
                write!(file, "   ")?; // 3 spaces
            }

            // ASCII section
            write!(file, "  |")?;
            for b in chunk {
                let c = *b;
                let printable = if (0x20..=0x7E).contains(&c) {
                    c as char
                } else {
                    '.'
                };
                write!(file, "{}", printable)?;
            }
            writeln!(file, "|")?;
        }

        Ok(())
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
    pub fn read_u32_le(&mut self) -> Result<u32, io::Error> {
        let bytes = self
            .slice(4)
            .ok_or_else(|| Error::new(ErrorKind::UnexpectedEof, "Not enough bytes to read u32"))?;

        let bytes_arr: [u8; 4] = bytes
            .try_into()
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Failed to convert bytes to u32"))?;

        Ok(u32::from_le_bytes(bytes_arr))
    }
    /// Read a u16 from the buffer
    pub fn read_u16_le(&mut self) -> Result<u16, io::Error> {
        let bytes = self.slice(2).ok_or_else(|| {
            Error::new(ErrorKind::UnexpectedEof, "Not enough bytes to read a u16")
        })?;
        let bytes_arr: [u8; 2] = bytes
            .try_into()
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Failed to convert bytes to u16"))?;

        Ok(u16::from_le_bytes(bytes_arr))
    }
    /// Read a i32 from the buffer
    pub fn read_i32_le(&mut self) -> Result<i32, io::Error> {
        let bytes = self.slice(4).ok_or_else(|| {
            Error::new(ErrorKind::UnexpectedEof, "Not enough bytes to read a i32")
        })?;
        let bytes_arr: [u8; 4] = bytes
            .try_into()
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Failed to convert bytes to i32"))?;

        Ok(i32::from_le_bytes(bytes_arr))
    }
    /// Read a i16 from the buffer
    pub fn read_i16_le(&mut self) -> Result<i16, io::Error> {
        let bytes = self.slice(2).ok_or_else(|| {
            Error::new(ErrorKind::UnexpectedEof, "Not enough bytes to read a i16")
        })?;
        let bytes_arr: [u8; 2] = bytes
            .try_into()
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Failed to convert bytes to i16"))?;

        Ok(i16::from_le_bytes(bytes_arr))
    }
    /// Read a f64 from the buffer
    pub fn read_f64_le(&mut self) -> Result<f64, io::Error> {
        let bytes = self.slice(8).ok_or_else(|| {
            Error::new(ErrorKind::UnexpectedEof, "Not enough bytes to read a f64")
        })?;

        let bytes_arr: [u8; 8] = bytes
            .try_into()
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Failed to convert bytes to f64"))?;

        Ok(f64::from_le_bytes(bytes_arr))
    }
    /// Read a f32 from the buffer
    pub fn read_f32_le(&mut self) -> Result<f32, io::Error> {
        let bytes = self.slice(4).ok_or_else(|| {
            Error::new(ErrorKind::UnexpectedEof, "Not enough bytes to read a f32")
        })?;

        let bytes_arr: [u8; 4] = bytes
            .try_into()
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Failed to convert bytes to f32"))?;

        Ok(f32::from_le_bytes(bytes_arr))
    }

    /// Read a byte from the buffer
    pub fn read_byte(&mut self) -> Result<u8, io::Error> {
        let bytes = self.slice(1).ok_or_else(|| {
            Error::new(ErrorKind::UnexpectedEof, "Not enough bytes to read a byte")
        })?;
        Ok(bytes[0])
    }

    /// Read a length-prefixed UTF-8 string from the buffer (4-byte LE length + bytes)
    pub fn read_lp_string(&mut self) -> Result<String, io::Error> {
        let strlen = self.read_u32_le()? as usize;
        let bytes = self.slice(strlen).ok_or_else(|| {
            Error::new(
                ErrorKind::UnexpectedEof,
                "Not enough bytes to read a string defined by the length prefix",
            )
        })?;
        let s = str::from_utf8(bytes)
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Invalid UTF-8 string"))?;

        Ok(s.to_owned())
    }

    pub fn read_null_terminated_string(&mut self) -> Result<String, io::Error> {
        let strlen = self.read_u32_le()? as usize;
        let mut bytes = Vec::new();
        while let b = self.read_byte()? {
            if b == 0 {
                break;
            }
            bytes.push(b);
        }
        println!("{:?}", bytes);
        let s = String::from_utf8(bytes).map_err(|_| {
            Error::new(
                ErrorKind::InvalidData,
                "Bytes cannot be converted to string",
            )
        })?;
        Ok(s)
    }
}

pub trait BufferReader {
    fn read_u32_le(&mut self) -> Result<u32, std::io::Error>;
    fn read_f64_le(&mut self) -> Result<f64, std::io::Error>;
    fn read_byte(&mut self) -> Result<u8, std::io::Error>;
    fn read_lp_string(&mut self) -> Result<String, std::io::Error>;
    fn read_null_terminated_string(&mut self) -> Result<String, std::io::Error>;
    fn read_header(&mut self) -> Result<pose::Header, std::io::Error>;
}
