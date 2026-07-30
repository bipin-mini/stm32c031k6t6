use core::convert::TryInto;

/// Default Modbus slave address.
pub const DEFAULT_ADDRESS: u8 = 127;

/// Supported function codes.
const FC_READ_HOLDING: u8 = 0x03;
const FC_WRITE_SINGLE: u8 = 0x06;
const FC_WRITE_MULTIPLE: u8 = 0x10;

/// Exception codes.
const EX_ILLEGAL_FUNCTION: u8 = 0x01;
const EX_ILLEGAL_ADDRESS: u8 = 0x02;
const EX_ILLEGAL_VALUE: u8 = 0x03;

#[derive(Default, Debug, Clone, Copy)]
pub struct HoldingRegisters {
    pub value_low: u16,
    pub value_high: u16,
    pub node_address: u16,
    pub new_node_address: u16,
}

pub struct Modbus {
    address: u8,
}

impl Modbus {
    pub const fn new(address: u8) -> Self {
        Self { address }
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    pub fn set_address(&mut self, address: u8) {
        self.address = address;
    }

    /// Process one complete Modbus RTU frame using external holding registers.
    pub fn process(
        &mut self,
        request: &[u8],
        response: &mut [u8],
        holding: &mut HoldingRegisters,
    ) -> usize {
        if request.len() < 4 {
            return 0;
        }

        // Verify CRC
        let received_crc = u16::from_le_bytes(request[request.len() - 2..].try_into().unwrap());
        let calculated_crc = crc16(&request[..request.len() - 2]);

        if received_crc != calculated_crc {
            return 0;
        }

        let slave = request[0];

        // Ignore frames for other nodes (unless broadcast)
        if slave != self.address && slave != 0 {
            return 0;
        }

        let broadcast = slave == 0;

        match request[1] {
            FC_READ_HOLDING => self.read_holding(request, response, broadcast, holding),
            FC_WRITE_SINGLE => self.write_single(request, response, broadcast, holding),
            FC_WRITE_MULTIPLE => self.write_multiple(request, response, broadcast, holding),
            function => {
                if broadcast {
                    return 0;
                }
                self.exception(response, function, EX_ILLEGAL_FUNCTION)
            }
        }
    }

    fn exception(&self, response: &mut [u8], function: u8, code: u8) -> usize {
        response[0] = self.address;
        response[1] = function | 0x80;
        response[2] = code;

        let crc = crc16(&response[..3]);
        response[3..5].copy_from_slice(&crc.to_le_bytes());

        5
    }

    fn read_holding(
        &mut self,
        request: &[u8],
        response: &mut [u8],
        broadcast: bool,
        holding: &HoldingRegisters,
    ) -> usize {
        if broadcast {
            return 0;
        }

        if request.len() != 8 {
            return self.exception(response, FC_READ_HOLDING, EX_ILLEGAL_VALUE);
        }

        let start = u16::from_be_bytes([request[2], request[3]]);
        let count = u16::from_be_bytes([request[4], request[5]]);

        if count == 0 {
            return self.exception(response, FC_READ_HOLDING, EX_ILLEGAL_VALUE);
        }

        if start + count > 4 {
            return self.exception(response, FC_READ_HOLDING, EX_ILLEGAL_ADDRESS);
        }

        response[0] = self.address;
        response[1] = FC_READ_HOLDING;
        response[2] = (count * 2) as u8;

        let mut index = 3;

        for reg in start..start + count {
            let value = match self.read_register(holding, reg) {
                Ok(val) => val,
                Err(code) => return self.exception(response, FC_READ_HOLDING, code),
            };

            response[index] = (value >> 8) as u8;
            response[index + 1] = value as u8;
            index += 2;
        }

        let crc = crc16(&response[..index]);
        response[index] = crc as u8;
        response[index + 1] = (crc >> 8) as u8;

        index + 2
    }

    fn write_single(
        &mut self,
        request: &[u8],
        response: &mut [u8],
        broadcast: bool,
        holding: &mut HoldingRegisters,
    ) -> usize {
        if request.len() != 8 {
            return self.exception(response, FC_WRITE_SINGLE, EX_ILLEGAL_VALUE);
        }

        let reg = u16::from_be_bytes([request[2], request[3]]);
        let value = u16::from_be_bytes([request[4], request[5]]);

        if let Err(code) = self.write_register(holding, reg, value) {
            return self.exception(response, FC_WRITE_SINGLE, code);
        }

        if broadcast {
            return 0;
        }

        response[..6].copy_from_slice(&request[..6]);
        let crc = crc16(&response[..6]);
        response[6] = crc as u8;
        response[7] = (crc >> 8) as u8;

        8
    }

    fn write_multiple(
        &mut self,
        request: &[u8],
        response: &mut [u8],
        broadcast: bool,
        holding: &mut HoldingRegisters,
    ) -> usize {
        if request.len() < 9 {
            return self.exception(response, FC_WRITE_MULTIPLE, EX_ILLEGAL_VALUE);
        }

        let start = u16::from_be_bytes([request[2], request[3]]);
        let count = u16::from_be_bytes([request[4], request[5]]);
        let byte_count = request[6] as usize;

        if byte_count != (count as usize * 2) || request.len() != 9 + byte_count {
            return self.exception(response, FC_WRITE_MULTIPLE, EX_ILLEGAL_VALUE);
        }

        if start + count > 4 {
            return self.exception(response, FC_WRITE_MULTIPLE, EX_ILLEGAL_ADDRESS);
        }

        let mut data = 7;
        for reg in start..start + count {
            let value = u16::from_be_bytes([request[data], request[data + 1]]);
            if let Err(code) = self.write_register(holding, reg, value) {
                return self.exception(response, FC_WRITE_MULTIPLE, code);
            }
            data += 2;
        }

        if broadcast {
            return 0;
        }

        response[..6].copy_from_slice(&request[..6]);
        let crc = crc16(&response[..6]);
        response[6] = crc as u8;
        response[7] = (crc >> 8) as u8;

        8
    }

    fn read_register(&self, holding: &HoldingRegisters, reg: u16) -> Result<u16, u8> {
        match reg {
            0 => Ok(holding.value_low),
            1 => Ok(holding.value_high),
            2 => Ok(holding.node_address),
            3 => Ok(holding.new_node_address),
            _ => Err(EX_ILLEGAL_ADDRESS),
        }
    }

    fn write_register(&mut self, holding: &mut HoldingRegisters, reg: u16, value: u16) -> Result<(), u8> {
        match reg {
            0 => holding.value_low = value,
            1 => holding.value_high = value,
            2 => {
                if value == 0 || value > 247 {
                    return Err(EX_ILLEGAL_VALUE);
                }
                holding.node_address = value;
            }
            3 => {
                if value == 0 || value > 247 {
                    return Err(EX_ILLEGAL_VALUE);
                }
                holding.new_node_address = value;
                self.address = value as u8;
            }
            _ => return Err(EX_ILLEGAL_ADDRESS),
        }
        Ok(())
    }
}

/// Standard Modbus CRC16.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;

    for &byte in data {
        crc ^= byte as u16;

        for _ in 0..8 {
            if crc & 1 != 0 {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    crc
}