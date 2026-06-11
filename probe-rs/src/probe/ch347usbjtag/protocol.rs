use std::time::Duration;

use bitvec::vec::BitVec;
use nusb::{DeviceInfo, Interface, MaybeFuture};

use crate::probe::{
    self, DebugProbeError, DebugProbeInfo, DebugProbeSelector, ProbeCreationError,
    usb_util::InterfaceExt,
};

use super::Ch347UsbJtagFactory;

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
