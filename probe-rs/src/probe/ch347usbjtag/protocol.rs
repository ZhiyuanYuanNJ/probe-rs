use std::time::Duration;

use bitvec::vec::BitVec;
use nusb::{DeviceInfo, Interface, MaybeFuture};

use crate::probe::{
    self, DebugProbeError, DebugProbeInfo, DebugProbeSelector, ProbeCreationError,
    usb_util::InterfaceExt,
};

use super::Ch347UsbJtagFactory;

// ---------------------------------------------------------------------------
// SWD protocol constants
// ---------------------------------------------------------------------------

/// SWD interface initialization command (speed configuration).
const SWD_CMD_INIT: u8 = 0xE5;
/// SWD data exchange command (contains sub-commands).
const SWD_CMD_EXCHANGE: u8 = 0xE8;
/// SWD register write sub-command.
const SWD_SUB_REG_W: u8 = 0xA0;
/// SWD custom sequence write sub-command.
const SWD_SUB_SEQ_W: u8 = 0xA1;
/// SWD register read sub-command.
const SWD_SUB_REG_R: u8 = 0xA2;

/// Send bit-width for SWD register write: 41 bits (8 request + 32 data + 1 parity).
const SWD_REG_W_BIT_WIDTH: u8 = 0x29;
/// Send bit-width for SWD register read: 34 bits (8 request + 26 turnaround/trailing).
const SWD_REG_R_BIT_WIDTH: u8 = 0x22;

pub(super) const CH34X_VID_PID: [(u16, u16); 3] =
    [(0x1A86, 0x55DE), (0x1A86, 0x55DD), (0x1A86, 0x55E8)];

const CH347F_INTERFACE_NUM: u8 = 4;
const CH347T_INTERFACE_NUM: u8 = 2;

const EP_OUT: u8 = 0x06;
const EP_IN: u8 = 0x86;

pub(crate) fn is_ch34x_device(device: &DeviceInfo) -> bool {
    CH34X_VID_PID.contains(&(device.vendor_id(), device.product_id()))
}

#[derive(Debug, Clone, Copy)]
enum Pack {
    StandardPack,
    LargePack,
}

#[derive(Debug, Clone, Copy)]
enum Command {
    Clock { tms: bool, tdi: bool, capture: bool },
}

impl From<Command> for u8 {
    fn from(value: Command) -> Self {
        match value {
            Command::Clock { tms, tdi, .. } => (u8::from(tms) << 1) | (u8::from(tdi) << 4),
        }
    }
}

struct Clock {
    tms: bool,
    tdi: bool,
    trst: bool,
}

impl From<Clock> for u8 {
    fn from(value: Clock) -> Self {
        let Clock { tms, tdi, trst } = value;
        u8::from(tms) << 1 | u8::from(tdi) << 4 | u8::from(trst) << 5
    }
}

// ---------------------------------------------------------------------------
// Transport abstraction — cfg-gated for Windows (DLL + nusb) vs nusb only
// ---------------------------------------------------------------------------

#[cfg(target_os = "windows")]
use super::dll::Ch347DllTransport;

#[cfg(target_os = "windows")]
enum Transport {
    Dll(Ch347DllTransport),
    Usb(Interface),
}

#[cfg(not(target_os = "windows"))]
type Transport = Interface;

// ---------------------------------------------------------------------------
// Ch347UsbJtagDevice
// ---------------------------------------------------------------------------

/// Ch347 device, which is a usb to gpio/i2c/spi/jtag/swd
/// ch347 has different packages, ch347f and ch347t
/// ch347t work mode depend on pin state on bool
/// ch347f full work
pub struct Ch347UsbJtagDevice {
    transport: Transport,
    name: String,
    command_queue: Vec<Command>,
    response: BitVec,
    pack: Pack,
    speed_khz: u32,
}

impl std::fmt::Debug for Ch347UsbJtagDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Ch347UsbJtagDevice")
            .field("name", &self.name)
            .field("pack", &self.pack)
            .field("speed", &self.speed_khz)
            .finish()
    }
}

// ---------------------------------------------------------------------------
// USB I/O helpers — dispatch to active transport
// ---------------------------------------------------------------------------

impl Ch347UsbJtagDevice {
    /// Write data to the USB endpoint.
    fn usb_write(&self, data: &[u8]) -> Result<(), DebugProbeError> {
        match &self.transport {
            #[cfg(target_os = "windows")]
            Transport::Dll(dll) => {
                dll.write(data).map_err(DebugProbeError::Usb)?;
            }
            #[cfg(target_os = "windows")]
            Transport::Usb(iface) => {
                iface
                    .write_bulk(EP_OUT, data, Duration::from_millis(500))
                    .map_err(DebugProbeError::Usb)?;
            }
            #[cfg(not(target_os = "windows"))]
            iface => {
                iface
                    .write_bulk(EP_OUT, data, Duration::from_millis(500))
                    .map_err(DebugProbeError::Usb)?;
            }
        }
        Ok(())
    }

    /// Read data from the USB endpoint.
    fn usb_read(&self, buf: &mut [u8]) -> Result<(), DebugProbeError> {
        match &self.transport {
            #[cfg(target_os = "windows")]
            Transport::Dll(dll) => {
                dll.read(buf).map_err(DebugProbeError::Usb)?;
            }
            #[cfg(target_os = "windows")]
            Transport::Usb(iface) => {
                iface
                    .read_bulk(EP_IN, buf, Duration::from_millis(500))
                    .map_err(DebugProbeError::Usb)?;
            }
            #[cfg(not(target_os = "windows"))]
            iface => {
                iface
                    .read_bulk(EP_IN, buf, Duration::from_millis(500))
                    .map_err(DebugProbeError::Usb)?;
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Device opening — nusb path (shared by non-Windows and Windows fallback)
// ---------------------------------------------------------------------------

impl Ch347UsbJtagDevice {
    /// Open via nusb (WinUSB). Used on non-Windows and as a fallback on Windows.
    fn open_via_nusb(selector: &DebugProbeSelector) -> Result<(Transport, Pack), ProbeCreationError> {
        let devices = nusb::list_devices()
            .wait()
            .map_err(|e| ProbeCreationError::Usb(e.into()))?;
        let device = devices
            .filter(is_ch34x_device)
            .find(|device| selector.matches(device))
            .ok_or(ProbeCreationError::NotFound)?;

        let device_handle = device
            .open()
            .wait()
            .map_err(|e| probe::ProbeCreationError::Usb(e.into()))?;

        let config = device_handle
            .configurations()
            .next()
            .expect("Can get usb device configs");

        tracing::info!("Active config descriptor: {:?}", config);

        // ch347f default interface number is 4
        // ch347t default interface number is 2
        let interface = device_handle
            .claim_interface(CH347F_INTERFACE_NUM)
            .wait()
            .or(device_handle.claim_interface(CH347T_INTERFACE_NUM).wait())
            .map_err(|e| ProbeCreationError::Usb(e.into()))?;

        // Detect pack mode by sending the 0xD0 speed command
        let mut obuf = [0xD0, 0x06, 0x00, 0x00, 0x07, 0x30, 0x30, 0x30, 0x30];
        let mut ibuf = [0; 4];

        interface
            .write_bulk(EP_OUT, &obuf, Duration::from_millis(500))
            .map_err(ProbeCreationError::Usb)?;
        interface
            .read_bulk(EP_IN, &mut ibuf, Duration::from_millis(500))
            .map_err(ProbeCreationError::Usb)?;

        let pack;
        if ibuf[0] == 0xD0 && ibuf[3] == 0x00 {
            obuf[4] = 5;
            pack = Pack::LargePack;
        } else {
            obuf[4] = 3;
            pack = Pack::StandardPack;
        }

        interface
            .write_bulk(EP_OUT, &obuf, Duration::from_millis(500))
            .map_err(ProbeCreationError::Usb)?;
        interface
            .read_bulk(EP_IN, &mut ibuf, Duration::from_millis(500))
            .map_err(ProbeCreationError::Usb)?;

        #[cfg(target_os = "windows")]
        let transport = Transport::Usb(interface);
        #[cfg(not(target_os = "windows"))]
        let transport = interface;

        Ok((transport, pack))
    }

    /// Open via CH347DLL vendor driver (Windows-only).
    #[cfg(target_os = "windows")]
    fn open_via_dll(selector: &DebugProbeSelector) -> Result<(Transport, Pack), ProbeCreationError> {
        let transport = Ch347DllTransport::open(selector)?;

        // Detect pack mode by sending the same 0xD0 speed command
        let mut obuf = [0xD0, 0x06, 0x00, 0x00, 0x07, 0x30, 0x30, 0x30, 0x30];
        let mut ibuf = [0u8; 4];

        transport.write(&obuf).map_err(ProbeCreationError::Usb)?;
        transport.read(&mut ibuf).map_err(ProbeCreationError::Usb)?;

        let pack;
        if ibuf[0] == 0xD0 && ibuf[3] == 0x00 {
            obuf[4] = 5;
            pack = Pack::LargePack;
        } else {
            obuf[4] = 3;
            pack = Pack::StandardPack;
        }

        transport.write(&obuf).map_err(ProbeCreationError::Usb)?;
        transport.read(&mut ibuf).map_err(ProbeCreationError::Usb)?;

        Ok((Transport::Dll(transport), pack))
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

impl Ch347UsbJtagDevice {
    pub(crate) fn new_from_selector(
        selector: &DebugProbeSelector,
    ) -> Result<Self, ProbeCreationError> {
        #[cfg(target_os = "windows")]
        let (transport, pack) = {
            // Try DLL first (vendor driver), fall back to nusb (WinUSB).
            //
            // When the WCH vendor driver is installed, nusb (WinUSB) cannot claim
            // the USB interface — it will fail with "incompatible driver". So we
            // only fall back to nusb if the DLL couldn't be loaded at all or no
            // device was visible to the DLL (NotFound). If the DLL loaded but
            // returned a specific error, the vendor driver is likely blocking
            // nusb too, so we should propagate the error directly.
            match Self::open_via_dll(selector) {
                Ok(result) => result,
                Err(ProbeCreationError::NotFound) => {
                    tracing::info!("CH347DLL: no device found via DLL, trying nusb (WinUSB)");
                    Self::open_via_nusb(selector)?
                }
                Err(ProbeCreationError::Other(msg)) => {
                    // DLL loaded and saw devices, but something went wrong.
                    // The vendor driver is likely installed, which means nusb
                    // will also fail. Propagate the DLL error directly.
                    tracing::info!("CH347DLL: {msg}");
                    tracing::info!(
                        "CH347DLL: skipping nusb fallback — vendor driver likely blocks WinUSB"
                    );
                    return Err(ProbeCreationError::Other(msg));
                }
                Err(e) => {
                    tracing::info!("CH347DLL: {e}, falling back to nusb");
                    Self::open_via_nusb(selector)?
                }
            }
        };

        #[cfg(not(target_os = "windows"))]
        let (transport, pack) = Self::open_via_nusb(selector)?;

        Ok(Self {
            transport,
            name: "ch347".into(),
            command_queue: Vec::new(),
            response: BitVec::new(),
            pack,
            speed_khz: 15000,
        })
    }

    pub(crate) fn attach(&mut self) -> Result<(), DebugProbeError> {
        self.apply_clock_speed(self.speed_khz)?;
        Ok(())
    }

    pub(crate) fn speed_khz(&self) -> u32 {
        self.speed_khz
    }

    pub(crate) fn set_speed_khz(&mut self, speed_khz: u32) -> u32 {
        self.speed_khz = speed_khz;
        self.speed_khz
    }

    fn pack(&self) -> Pack {
        self.pack
    }

    // with speed index: 468.75Khz, 937.5KHz, 1.875MHz, 3.75MHz, 7.5MHz, 15MHz, 30MHz, 60Mhz
    // STANDARD_Pack start from 1.875MHz
    // LARGER_Pack start from 468.75KHz
    fn apply_clock_speed(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let mut buf = [0; 4];
        let index = match self.pack() {
            Pack::StandardPack => match speed_khz {
                1875 => 0,
                3750 => 1,
                7500 => 2,
                15000 => 3,
                30000 => 4,
                60000 => 5,
                _ => return Err(DebugProbeError::UnsupportedSpeed(speed_khz)),
            },
            Pack::LargePack => match speed_khz {
                468 => 0,
                937 => 1,
                1875 => 2,
                3750 => 3,
                7500 => 4,
                15000 => 5,
                30000 => 6,
                60000 => 7,
                _ => return Err(DebugProbeError::UnsupportedSpeed(speed_khz)),
            },
        };
        self.usb_write(&[0xD0, 0x06, 0x00, 0x00, index, 0x00, 0x00, 0x00, 0x00])?;
        self.usb_read(&mut buf)?;
        if buf[3] == 0x00 {
            Ok(speed_khz)
        } else {
            Err(DebugProbeError::UnsupportedSpeed(speed_khz))
        }
    }

    fn flush(&mut self) -> Result<(), DebugProbeError> {
        let mut buffer = [0; 130];
        let mut obuf = vec![];
        let mut command = vec![0xD2];

        for &i in self.command_queue.iter() {
            let byte = u8::from(i);
            // the byte is clock low, bit 0 = 1 that clock high
            obuf.push(byte);
            obuf.push(byte | 0x01);
        }
        command.extend_from_slice(&(obuf.len() as u16).to_le_bytes());
        command.extend_from_slice(&obuf);

        self.usb_write(&command)?;
        self.usb_read(&mut buffer)?;

        for (&c, &byte) in self.command_queue.iter().zip(&buffer[3..]) {
            let Command::Clock { capture, .. } = c;
            if capture {
                self.response.push(byte != 0x00);
            }
        }

        self.command_queue.clear();
        Ok(())
    }

    pub(crate) fn shift_bit(
        &mut self,
        tms: bool,
        tdi: bool,
        capture: bool,
    ) -> Result<(), DebugProbeError> {
        // max clock len is 127
        if self.command_queue.len() >= 127 {
            self.flush()?;
        }
        self.command_queue
            .push(Command::Clock { tms, tdi, capture });
        Ok(())
    }

    pub(crate) fn read_captured_bits(&mut self) -> Result<BitVec, DebugProbeError> {
        self.flush()?;
        Ok(std::mem::take(&mut self.response))
    }

    // -----------------------------------------------------------------------
    // SWD protocol methods
    // -----------------------------------------------------------------------

    /// Initialize the SWD interface with the given speed.
    ///
    /// Maps the requested speed (in kHz) to a CH347 delay byte:
    /// - delay 0x00 = 5 MHz
    /// - delay 0x01 = 1 MHz
    /// - delay N (N≥1) = 1/N MHz
    ///
    /// Returns the actual speed in kHz that was configured.
    pub(crate) fn swd_init(&mut self, speed_khz: u32) -> Result<u32, DebugProbeError> {
        let (delay_byte, actual_speed_khz) = if speed_khz >= 5000 || speed_khz == 0 {
            // 默认或请求 ≥5 MHz：使用 5 MHz 档（delay=0x00）
            (0x00, 5000)
        } else if speed_khz >= 1000 {
            // 1-5 MHz 统一到 1 MHz
            (0x01, 1000)
        } else {
            // <1 MHz：按 1/N MHz 映射，例：500 kHz → delay 2，250 kHz → delay 4
            let delay = (1000 / speed_khz).max(1) as u8;
            let actual = 1000u32 / delay as u32;
            (delay, actual)
        };

        // Packet: 0xE5 + len(0x0008) + speed_param(4 bytes) + delay_byte + reserved(3 bytes)
        let mut obuf = [0u8; 11];
        obuf[0] = SWD_CMD_INIT;
        obuf[1] = 0x08; // LEN low
        obuf[2] = 0x00; // LEN high
        obuf[3] = 0x40; // speed param byte 0
        obuf[4] = 0x42; // speed param byte 1
        obuf[5] = 0x0F; // speed param byte 2
        obuf[6] = 0x00; // speed param byte 3
        obuf[7] = delay_byte;
        obuf[8] = 0x00; // reserved
        obuf[9] = 0x00; // reserved
        obuf[10] = 0x00; // reserved

        self.usb_write(&obuf)?;

        let mut ibuf = [0u8; 4];
        self.usb_read(&mut ibuf)?;

        if ibuf[0] == SWD_CMD_INIT && ibuf[3] == 0x00 {
            tracing::info!(
                "CH347 SWD initialized: delay_byte={delay_byte:#04x}, actual_speed={actual_speed_khz} kHz"
            );
            self.speed_khz = actual_speed_khz;
            Ok(actual_speed_khz)
        } else {
            Err(DebugProbeError::Usb(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("CH347 SWD init failed: response = {:02x?}", ibuf),
            )))
        }
    }

    /// Perform an SWD register write (sub-command 0xA0).
    ///
    /// The CH347 firmware handles the complete SWD transaction:
    /// request phase, data phase, and parity.
    ///
    /// Returns `(ack, status)` where:
    /// - `ack`: 3-bit ACK result (lower 3 bits)
    /// - `status`: command status (0x00 = success)
    pub(crate) fn swd_register_write(
        &mut self,
        request: u8,
        data: u32,
    ) -> Result<(u8, u8), DebugProbeError> {
        // Compute odd parity of the 32-bit write data
        let parity = (data.count_ones() % 2 == 1) as u8;

        // Build the 0xE8 packet with 0xA0 sub-command:
        // [0xE8, LEN_lo, LEN_hi, 0xA0, 0x29, 0x00, request, data[0..4], parity]
        let data_bytes = data.to_le_bytes();
        let mut obuf = [0u8; 12];
        obuf[0] = SWD_CMD_EXCHANGE;
        obuf[1] = 0x09; // LEN low (9 bytes of sub-command data)
        obuf[2] = 0x00; // LEN high
        obuf[3] = SWD_SUB_REG_W;
        obuf[4] = SWD_REG_W_BIT_WIDTH;
        obuf[5] = 0x00;
        obuf[6] = request;
        obuf[7] = data_bytes[0];
        obuf[8] = data_bytes[1];
        obuf[9] = data_bytes[2];
        obuf[10] = data_bytes[3];
        obuf[11] = parity;

        self.usb_write(&obuf)?;

        // Response: [0xE8, LEN_lo, LEN_hi, 0xA0(echo), ack]
        // Note: per the CH347 SWD example, each sub-command response is
        // prefixed with the sub-command code echo, NOT a separate "status"
        // byte. The 0xA0 write sub-response is therefore 2 bytes total
        // (echo + 3-bit SWD ACK), giving 5 bytes on the wire including the
        // [E8 LEN_lo LEN_hi] header.
        let mut ibuf = [0u8; 5];
        self.usb_read(&mut ibuf)?;

        let ack = ibuf[4];
        // Keep the second tuple field for API compatibility — it now mirrors
        // the ACK and is unused by the caller for control flow.
        Ok((ack, ack))
    }

    /// Perform an SWD register write (0xA0) followed by a trailing idle
    /// sequence (0xA1), packed into a single 0xE8 packet.
    ///
    /// This saves one USB round-trip on the AP-write OK path: instead of
    /// `[E8 A0 …] → response → [E8 A1 …] → response → [E8 A2 RDBUFF …]`
    /// (3 round-trips), the caller now does
    /// `[E8 A0 … A1 …] → response → [E8 A2 RDBUFF …]` (2 round-trips).
    ///
    /// Behaviour on WAIT/FAULT is unchanged: the firmware executes both
    /// sub-commands back-to-back regardless of the A0 ACK, so a WAIT just
    /// means the target got `idle_bits` extra clock cycles for free
    /// before the host's normal WAIT recovery kicks in. The A1 sequence
    /// has no meaningful failure mode.
    ///
    /// `idle_bits` must be in `1..=255`. `idle_data` must hold at least
    /// `ceil(idle_bits / 8)` bytes of pad (typically all-zero).
    ///
    /// Returns the SWD ACK byte from the A0 sub-response (lower 3 bits
    /// are the meaningful ACK).
    pub(crate) fn swd_register_write_with_trailing_idle(
        &mut self,
        request: u8,
        data: u32,
        idle_bits: u8,
        idle_data: &[u8],
    ) -> Result<u8, DebugProbeError> {
        debug_assert!(idle_bits > 0, "idle_bits must be > 0");
        let idle_byte_count = ((idle_bits as usize) + 7) / 8;
        debug_assert!(
            idle_data.len() >= idle_byte_count,
            "idle_data too short for {} bits",
            idle_bits
        );

        let parity = (data.count_ones() % 2 == 1) as u8;
        let data_bytes = data.to_le_bytes();

        // Sub-payload layout:
        //   A0 sub-command — 9 bytes: A0 29 00 [req] [d0..d3] [parity]
        //   A1 sub-command — 3 + idle_byte_count: A1 [bit_count] 00 [data..]
        let sub_total = 9 + 3 + idle_byte_count;
        let mut obuf = Vec::with_capacity(3 + sub_total);
        obuf.push(SWD_CMD_EXCHANGE);
        obuf.push((sub_total & 0xFF) as u8);
        obuf.push(((sub_total >> 8) & 0xFF) as u8);
        // A0 — register write
        obuf.push(SWD_SUB_REG_W);
        obuf.push(SWD_REG_W_BIT_WIDTH);
        obuf.push(0x00);
        obuf.push(request);
        obuf.extend_from_slice(&data_bytes);
        obuf.push(parity);
        // A1 — trailing idle sequence
        obuf.push(SWD_SUB_SEQ_W);
        obuf.push(idle_bits);
        obuf.push(0x00);
        obuf.extend_from_slice(&idle_data[..idle_byte_count]);

        self.usb_write(&obuf)?;

        // Response: [E8 LL LL] [A0 ack] [A1] = 3 + 2 + 1 = 6 bytes.
        let mut ibuf = [0u8; 6];
        self.usb_read(&mut ibuf)?;

        if ibuf[3] != SWD_SUB_REG_W || ibuf[5] != SWD_SUB_SEQ_W {
            return Err(DebugProbeError::Usb(std::io::Error::other(format!(
                "CH347 SWD batched write+idle: unexpected echoes {:#04x} {:#04x}",
                ibuf[3], ibuf[5]
            ))));
        }

        Ok(ibuf[4])
    }

    /// Perform an SWD register read (sub-command 0xA2).
    ///
    /// The CH347 firmware handles the complete SWD transaction:
    /// request phase, turnaround, data read, and parity.
    ///
    /// Returns `(ack, data, parity_trace)` where:
    /// - `ack`: 3-bit ACK result (lower 3 bits)
    /// - `data`: 32-bit read value
    /// - `parity_trace`: bit 0 = odd parity of data, bit 1 = trace bit
    pub(crate) fn swd_register_read(&mut self, request: u8) -> Result<(u8, u32, u8), DebugProbeError> {
        // Build the 0xE8 packet with 0xA2 sub-command:
        // [0xE8, LEN_lo, LEN_hi, 0xA2, 0x22, 0x00, request]
        let obuf = [SWD_CMD_EXCHANGE, 0x04, 0x00, SWD_SUB_REG_R, SWD_REG_R_BIT_WIDTH, 0x00, request];

        self.usb_write(&obuf)?;

        // Response: [0xE8, LEN_lo, LEN_hi, 0xA2(echo), ack, data[0..4], parity_trace]
        // The 0xA2 read sub-response is 7 bytes (echo + ack + 4 data + parity),
        // giving 10 bytes on the wire including the [E8 LEN_lo LEN_hi] header.
        let mut ibuf = [0u8; 10];
        self.usb_read(&mut ibuf)?;

        let ack = ibuf[4];
        let data = u32::from_le_bytes([ibuf[5], ibuf[6], ibuf[7], ibuf[8]]);
        let parity_trace = ibuf[9];
        Ok((ack, data, parity_trace))
    }

    /// Send an SWD custom bit sequence (sub-command 0xA1).
    ///
    /// Used for line resets and SWJ protocol sequences.
    /// Bits not filling a complete byte should be padded with 0x00.
    ///
    /// Returns the command status (0x00 = success).
    pub(crate) fn swd_sequence(
        &mut self,
        bit_count: u8,
        data: &[u8],
    ) -> Result<u8, DebugProbeError> {
        // Build the 0xE8 packet with 0xA1 sub-command:
        // [0xE8, LEN_lo, LEN_hi, 0xA1, bit_count, 0x00, data...]
        let sub_len = 3 + data.len(); // sub-command header (3) + data bytes
        let mut obuf = vec![0u8; 3 + sub_len];
        obuf[0] = SWD_CMD_EXCHANGE;
        obuf[1] = (sub_len & 0xFF) as u8; // LEN low
        obuf[2] = ((sub_len >> 8) & 0xFF) as u8; // LEN high
        obuf[3] = SWD_SUB_SEQ_W;
        obuf[4] = bit_count;
        obuf[5] = 0x00;
        obuf[6..].copy_from_slice(data);

        self.usb_write(&obuf)?;

        // Response: [0xE8, LEN_lo, LEN_hi, 0xA1(echo)]
        // The 0xA1 sequence sub-response is just the echo byte — no separate
        // status. A successful sequence is implicit in receiving the echo.
        let mut ibuf = [0u8; 4];
        self.usb_read(&mut ibuf)?;

        // Return 0x00 to keep the existing "0x00 = success" contract with
        // callers; treat any non-echo value as a transport error.
        if ibuf[3] == SWD_SUB_SEQ_W {
            Ok(0)
        } else {
            Err(DebugProbeError::Usb(std::io::Error::other(format!(
                "CH347 SWD sequence: unexpected echo byte {:#04x}",
                ibuf[3]
            ))))
        }
    }
}

// ---------------------------------------------------------------------------
// Device enumeration
// ---------------------------------------------------------------------------

pub(super) fn list_ch347usbjtag_devices() -> Vec<DebugProbeInfo> {
    let mut devices = vec![];

    // On Windows, try DLL enumeration first (for devices with vendor driver)
    #[cfg(target_os = "windows")]
    {
        devices.extend(super::dll::list_via_dll());
    }

    // Also enumerate via nusb (works for listing regardless of driver type)
    match nusb::list_devices().wait() {
        Ok(nusb_devices) => {
            for device in nusb_devices.filter(is_ch34x_device) {
                let vid = device.vendor_id();
                let pid = device.product_id();
                // Deduplicate: skip if already found via DLL
                if !devices.iter().any(|d| d.vendor_id == vid && d.product_id == pid) {
                    devices.push(DebugProbeInfo::new(
                        "CH347 USB Jtag".to_string(),
                        vid,
                        pid,
                        device.serial_number().map(Into::into),
                        &Ch347UsbJtagFactory,
                        None,
                        false,
                    ));
                }
            }
        }
        Err(e) => {
            tracing::warn!("error listing CH347 devices: {e}");
        }
    }

    devices
}
