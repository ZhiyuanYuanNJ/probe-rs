//! CH347 vendor DLL transport layer (Windows-only).
//!
//! Dynamically loads CH347DLLA64.DLL using Win32 `GetProcAddress` and provides
//! `CH347WriteData`/`CH347ReadData` as the USB I/O transport, replacing the
//! default WinUSB (nusb) backend.

use std::ffi::c_void;
use std::io;

use crate::probe::{DebugProbeInfo, DebugProbeSelector, ProbeCreationError};

use super::{Ch347UsbJtagFactory, protocol::CH34X_VID_PID};

// ---------------------------------------------------------------------------
// Win32 FFI declarations (kernel32.dll, always available on Windows)
// ---------------------------------------------------------------------------

unsafe extern "system" {
    fn LoadLibraryA(lpLibFileName: *const u8) -> *mut c_void;
    fn GetProcAddress(hModule: *mut c_void, lpProcName: *const u8) -> *mut c_void;
    fn FreeLibrary(hModule: *mut c_void) -> i32;
}

/// Sentinel value returned by `CH347OpenDevice` on failure (INVALID_HANDLE_VALUE).
const OPEN_FAILED: i32 = -1;

// ---------------------------------------------------------------------------
// CH347DLL function pointer types
// ---------------------------------------------------------------------------
// Signatures match WCH official CH347DLL.H / OpenOCD ch347.c typedefs.

type FnCh347OpenDevice = unsafe extern "system" fn(u32) -> i32;
type FnCh347CloseDevice = unsafe extern "system" fn(u32) -> i32;
type FnCh347WriteData = unsafe extern "system" fn(u32, *const c_void, *mut u32) -> u32;
type FnCh347ReadData = unsafe extern "system" fn(u32, *mut c_void, *mut u32) -> u32;
type FnCh347GetDeviceInfor = unsafe extern "system" fn(u32, *mut mDeviceInforS) -> u32;
type FnCh347SetTimeout = unsafe extern "system" fn(u32, u32, u32) -> u32;

// ---------------------------------------------------------------------------
// mDeviceInforS — device information struct from CH347DLL.H (packed)
// ---------------------------------------------------------------------------

/// Maximum number of simultaneous CH341/7 devices (from CH347DLL.H).
const CH347_MAX_NUMBER: u32 = 32;

/// FuncType value for JTAG + I2C interface (from CH347DLL.H / OpenOCD ch347.c).
const CH347_FUNC_JTAG_I2C: u8 = 2;

/// Device information structure from CH347DLL.
///
/// Layout matches `#pragma pack(1)` definition in CH347DLL.H.
/// All field access must copy to a local variable — never take a reference
/// into a packed struct (UB for unaligned fields).
#[repr(C, packed)]
struct mDeviceInforS {
    i_index: u8,
    device_path: [u8; 260], // MAX_PATH
    usb_class: u8,
    func_type: u8,
    device_id: [i8; 64],
    chip_mode: u8,
    dev_handle: usize, // HANDLE — pointer-sized
    bulk_out_endp_max_size: u16,
    bulk_in_endp_max_size: u16,
    usb_speed_type: u8,
    ch347_if_num: u8,
    data_up_endp: u8,
    data_dn_endp: u8,
    product_string: [i8; 64],
    manufacturer_string: [i8; 64],
    write_timeout: u32,
    read_timeout: u32,
    func_desc_str: [i8; 64],
    firmware_ver: u8,
}

// ---------------------------------------------------------------------------
// Ch347Dll — dynamically loaded DLL handle
// ---------------------------------------------------------------------------

/// Holds the loaded DLL module handle and resolved function pointers.
struct Ch347Dll {
    module: *mut c_void,
    open_device: FnCh347OpenDevice,
    close_device: FnCh347CloseDevice,
    write_data: FnCh347WriteData,
    read_data: FnCh347ReadData,
    get_device_infor: FnCh347GetDeviceInfor,
    set_timeout: FnCh347SetTimeout,
}

// SAFETY: Ch347Dll owns a Win32 DLL module handle (HMODULE) and function pointers
// resolved from it. The handle is safe to send across threads — Win32 DLL handles
// are process-global, and each function pointer is a plain code address.
unsafe impl Send for Ch347Dll {}

impl Ch347Dll {
    /// Load CH347DLLA64.DLL and resolve all required function pointers.
    fn load() -> io::Result<Self> {
        let module = unsafe { LoadLibraryA(concat!("CH347DLLA64", "\0").as_ptr()) };
        if module.is_null() {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "CH347DLLA64.DLL not found — install WCH CH347 vendor driver",
            ));
        }

        macro_rules! get_proc {
            ($name:literal, $ty:ty) => {{
                let name_cstr = concat!($name, "\0");
                let proc = unsafe { GetProcAddress(module, name_cstr.as_ptr()) };
                if proc.is_null() {
                    unsafe { FreeLibrary(module) };
                    return Err(io::Error::new(
                        io::ErrorKind::NotFound,
                        concat!("CH347DLL: function ", $name, " not found"),
                    ));
                }
                // SAFETY: transmute raw function pointer to our typed fn pointer.
                // The signature must match the DLL export exactly.
                unsafe { std::mem::transmute::<*mut c_void, $ty>(proc) }
            }};
        }

        let open_device = get_proc!("CH347OpenDevice", FnCh347OpenDevice);
        let close_device = get_proc!("CH347CloseDevice", FnCh347CloseDevice);
        let write_data = get_proc!("CH347WriteData", FnCh347WriteData);
        let read_data = get_proc!("CH347ReadData", FnCh347ReadData);
        let get_device_infor = get_proc!("CH347GetDeviceInfor", FnCh347GetDeviceInfor);
        let set_timeout = get_proc!("CH347SetTimeout", FnCh347SetTimeout);

        Ok(Self {
            module,
            open_device,
            close_device,
            write_data,
            read_data,
            get_device_infor,
            set_timeout,
        })
    }
}

impl Drop for Ch347Dll {
    fn drop(&mut self) {
        if !self.module.is_null() {
            unsafe { FreeLibrary(self.module) };
        }
    }
}

// ---------------------------------------------------------------------------
// Ch347DllTransport — transport using the vendor DLL
// ---------------------------------------------------------------------------

/// USB transport backed by CH347DLL vendor functions.
///
/// Opens a device by index via `CH347OpenDevice`, then uses `CH347WriteData`
/// and `CH347ReadData` for I/O. The device is closed in `Drop`.
pub(crate) struct Ch347DllTransport {
    dll: Ch347Dll,
    index: u32,
}

impl Ch347DllTransport {
    /// Open a CH347 device matching the given selector.
    ///
    /// Iterates DLL device indices 0..32, opens each, queries device info,
    /// and matches VID/PID from the `DeviceID` string against the selector.
    /// The first matching device is kept open.
    ///
    /// Matching strategy (in order of priority):
    /// 1. VID/PID parsed from `DeviceID` matches the selector
    /// 2. VID/PID parsed from `DeviceID` is a known CH347 VID/PID and
    ///    selector also expects a CH347 VID/PID
    /// 3. `func_type == 2` (JTAG) and selector VID is a known CH347 VID
    pub(crate) fn open(selector: &DebugProbeSelector) -> Result<Self, ProbeCreationError> {
        let dll = Ch347Dll::load().map_err(|_| {
            ProbeCreationError::Other(
                "CH347DLLA64.DLL not found — install WCH CH347 vendor driver",
            )
        })?;

        let mut opened_count = 0u32;
        let mut vid_pid_mismatch = false;
        let mut jtag_fallback_index: Option<u32> = None;

        let selector_is_ch347 = CH34X_VID_PID
            .iter()
            .any(|&(vid, pid)| vid == selector.vendor_id && pid == selector.product_id);

        for index in 0..CH347_MAX_NUMBER {
            let ret = unsafe { (dll.open_device)(index) };
            if ret == OPEN_FAILED {
                continue;
            }
            opened_count += 1;

            // Query device info for VID/PID matching
            let mut info: mDeviceInforS = unsafe { std::mem::zeroed() };
            let ok = unsafe { (dll.get_device_infor)(index, &mut info) };
            if ok == 0 {
                tracing::debug!(
                    "CH347DLL: CH347GetDeviceInfor returned 0 for index {index}, closing"
                );
                unsafe { (dll.close_device)(index) };
                continue;
            }

            // Copy fields from packed struct (required for packed access)
            let device_id = info.device_id;
            let func_type = info.func_type;
            let chip_mode = info.chip_mode;
            let product_string = info.product_string;
            let manufacturer_string = info.manufacturer_string;

            tracing::info!(
                "CH347DLL: index={index}, open_ret={ret}, func_type={func_type}, \
                 chip_mode={chip_mode}, device_id={}",
                device_id_to_string(&device_id)
            );
            tracing::debug!(
                "CH347DLL: index={index}, device_id hex={:02X?}",
                &device_id[..32]
            );
            tracing::debug!(
                "CH347DLL: index={index}, product={:?}, manufacturer={:?}",
                device_id_to_string(&product_string),
                device_id_to_string(&manufacturer_string)
            );

            if let Some((vid, pid)) = parse_device_id(&device_id) {
                tracing::info!(
                    "CH347DLL: parsed VID={vid:#06X}, PID={pid:#06X} \
                     (selector expects VID={:#06X}, PID={:#06X})",
                    selector.vendor_id,
                    selector.product_id
                );

                if vid == selector.vendor_id && pid == selector.product_id {
                    // Exact VID/PID match — use this device
                    unsafe { (dll.set_timeout)(index, 500, 500) };
                    tracing::info!("CH347DLL: opened device at index {index} (exact VID/PID match)");
                    return Ok(Self { dll, index });
                }

                // VID/PID is a known CH347 but doesn't match selector exactly
                if CH34X_VID_PID.contains(&(vid, pid)) {
                    vid_pid_mismatch = true;
                    // If selector also wants a CH347, this might be the right device
                    // (e.g. selector has VID from nusb enumeration)
                    if selector_is_ch347 {
                        unsafe { (dll.set_timeout)(index, 500, 500) };
                        tracing::info!(
                            "CH347DLL: opened device at index {index} \
                             (CH347 VID/PID match: {vid:#06X}:{pid:#06X})"
                        );
                        return Ok(Self { dll, index });
                    }
                    tracing::debug!(
                        "CH347DLL: VID/PID {vid:#06X}:{pid:#06X} is a CH347 device \
                         but doesn't match selector, continuing search"
                    );
                }
            } else {
                tracing::warn!(
                    "CH347DLL: could not parse VID/PID from device_id at index {index}, \
                     raw={:?}",
                    &device_id[..16]
                );

                // Fallback: if func_type indicates JTAG and selector wants a CH347,
                // this is likely our device
                if func_type == CH347_FUNC_JTAG_I2C && selector_is_ch347 && jtag_fallback_index.is_none() {
                    jtag_fallback_index = Some(index);
                    tracing::info!(
                        "CH347DLL: index {index} has JTAG func_type, saved as fallback"
                    );
                    // Don't close — keep as fallback candidate
                    continue;
                }
            }

            // Not a match — close and continue
            unsafe { (dll.close_device)(index) };
        }

        // Use JTAG fallback if we found one and no better match
        if let Some(fallback_index) = jtag_fallback_index {
            unsafe { (dll.set_timeout)(fallback_index, 500, 500) };
            tracing::info!(
                "CH347DLL: opened device at index {fallback_index} \
                 (JTAG func_type fallback)"
            );
            return Ok(Self { dll, index: fallback_index });
        }

        // Provide a specific error to help the caller decide whether to
        // fall back to nusb or give up entirely.
        if opened_count > 0 && vid_pid_mismatch {
            // DLL can see devices but VID/PID doesn't match —
            // don't fall back to nusb (vendor driver blocks it).
            Err(ProbeCreationError::Other(
                "CH347DLL: found CH347 device(s) but VID/PID does not match selector",
            ))
        } else if opened_count > 0 {
            // DLL opened devices but couldn't parse VID/PID —
            // likely a DLL/driver version issue.
            Err(ProbeCreationError::Other(
                "CH347DLL: opened device(s) but could not read VID/PID — \
                 check CH347 driver version",
            ))
        } else {
            Err(ProbeCreationError::NotFound)
        }
    }

    /// Write data to the CH347 device via `CH347WriteData`.
    pub(crate) fn write(&self, data: &[u8]) -> io::Result<usize> {
        let mut length = data.len() as u32;
        let ok =
            unsafe { (self.dll.write_data)(self.index, data.as_ptr() as *const c_void, &mut length) };
        if ok == 0 {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "CH347WriteData failed",
            ));
        }
        Ok(length as usize)
    }

    /// Read data from the CH347 device via `CH347ReadData`.
    pub(crate) fn read(&self, buf: &mut [u8]) -> io::Result<usize> {
        let mut length = buf.len() as u32;
        let ok = unsafe {
            (self.dll.read_data)(self.index, buf.as_mut_ptr() as *mut c_void, &mut length)
        };
        if ok == 0 {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "CH347ReadData failed",
            ));
        }
        Ok(length as usize)
    }
}

impl Drop for Ch347DllTransport {
    fn drop(&mut self) {
        unsafe { (self.dll.close_device)(self.index) };
    }
}

// ---------------------------------------------------------------------------
// Device enumeration via DLL
// ---------------------------------------------------------------------------

/// List CH347 devices available through the vendor DLL.
///
/// Iterates indices 0..32, opens each device, reads its info, then closes.
/// Returns a list of `DebugProbeInfo` for each found device.
pub(crate) fn list_via_dll() -> Vec<DebugProbeInfo> {
    let Ok(dll) = Ch347Dll::load() else {
        return vec![];
    };

    let mut devices = vec![];
    for index in 0..CH347_MAX_NUMBER {
        let ret = unsafe { (dll.open_device)(index) };
        if ret == OPEN_FAILED {
            continue;
        }

        let mut info: mDeviceInforS = unsafe { std::mem::zeroed() };
        let ok = unsafe { (dll.get_device_infor)(index, &mut info) };

        // Copy fields from packed struct before closing
        let device_id = info.device_id;
        let func_type = info.func_type;

        // Close immediately — we only needed info for listing
        unsafe { (dll.close_device)(index) };

        if ok == 0 {
            continue;
        }

        if let Some((vid, pid)) = parse_device_id(&device_id) {
            if CH34X_VID_PID.contains(&(vid, pid)) {
                devices.push(DebugProbeInfo::new(
                    "Jtag".to_string(),
                    vid,
                    pid,
                    None,
                    &Ch347UsbJtagFactory,
                    None,
                    false,
                ));
            }
        } else if func_type == CH347_FUNC_JTAG_I2C {
            // VID/PID not parseable but it's a JTAG device from the DLL —
            // report with known CH347 VID/PID so it's selectable
            tracing::debug!(
                "CH347DLL: listing index {index} as JTAG device (func_type=2) \
                 despite unparseable device_id"
            );
            devices.push(DebugProbeInfo::new(
                "CH347 USB Jtag".to_string(),
                0x1A86,
                0x55DE,
                None,
                &Ch347UsbJtagFactory,
                None,
                false,
            ));
        }
    }
    devices
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Convert a device_id C-string array to a Rust String for logging.
fn device_id_to_string(device_id: &[i8; 64]) -> String {
    device_id
        .iter()
        .take_while(|&&b| b != 0)
        .map(|&b| b as u8 as char)
        .collect()
}

/// Parse `"USB\VID_xxxx&PID_xxxx"` from the `DeviceID` field.
///
/// VID/PID values are hexadecimal (e.g. `1A86`), so we use
/// `u16::from_str_radix(_, 16)` rather than decimal `parse()`.
fn parse_device_id(device_id: &[i8; 64]) -> Option<(u16, u16)> {
    let s: String = device_id
        .iter()
        .take_while(|&&b| b != 0)
        .map(|&b| b as u8 as char)
        .collect();
    let s = s.to_uppercase();

    let vid_str = s.split("VID_").nth(1)?.split('&').next()?;
    let vid = u16::from_str_radix(vid_str, 16).ok()?;
    let pid_str = s.split("PID_").nth(1)?;
    let pid = u16::from_str_radix(
        pid_str.split(&['&', '\\', '\0']).next()?,
        16,
    )
    .ok()?;

    Some((vid, pid))
}
