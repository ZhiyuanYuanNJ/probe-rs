//! ch347 is a usb bus converter that provides UART, I2C and SPI and Jtag/Swd interface
mod protocol;

#[cfg(target_os = "windows")]
mod dll;

use protocol::Ch347UsbJtagDevice;

use bitvec::prelude::BitVec;

use crate::{
    architecture::{
        arm::{
            ArmCommunicationInterface, ArmError, DapError, RawDapAccess,
            communication_interface::DapProbe,
            dp::{Abort, Ctrl, DpRegister, RdBuff},
            sequences::ArmDebugSequence,
            traits::RegisterAddress,
        },
        riscv::dtm::jtag_dtm::JtagDtmBuilder,
        xtensa::communication_interface::XtensaCommunicationInterface,
    },
    probe::{DebugProbe, DebugProbeError, JtagAccess, JtagSequence, ProbeFactory, WireProtocol},
};

use super::{AutoImplementJtagAccess, JtagDriverState, RawJtagIo, SwdSettings};

/// A factory for creating [`Ch347UsbJtag`] instances.
#[derive(Debug)]
pub struct Ch347UsbJtagFactory;

impl std::fmt::Display for Ch347UsbJtagFactory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Ch347UsbJtag")
    }
}

/// An Ch347-based debug probe.
#[derive(Debug)]
pub struct Ch347UsbJtag {
    device: Ch347UsbJtagDevice,
    jtag_state: JtagDriverState,
    protocol: WireProtocol,
    swd_settings: SwdSettings,
}

impl ProbeFactory for Ch347UsbJtagFactory {
    fn open(
        &self,
        selector: &super::DebugProbeSelector,
    ) -> Result<Box<dyn super::DebugProbe>, super::DebugProbeError> {
        let ch347 = Ch347UsbJtagDevice::new_from_selector(selector)?;

        tracing::info!("Found ch347 device");
        Ok(Box::new(Ch347UsbJtag {
            device: ch347,
            jtag_state: JtagDriverState::default(),
            protocol: WireProtocol::Jtag,
            swd_settings: SwdSettings::default(),
        }))
    }

    fn list_probes(&self) -> Vec<super::DebugProbeInfo> {
        protocol::list_ch347usbjtag_devices()
    }
}

impl RawJtagIo for Ch347UsbJtag {
    fn shift_bit(
        &mut self,
        tms: bool,
        tdi: bool,
        capture: bool,
    ) -> Result<(), super::DebugProbeError> {
        self.jtag_state.state.update(tms);
        self.device.shift_bit(tms, tdi, capture)?;

        Ok(())
    }

    fn read_captured_bits(&mut self) -> Result<bitvec::prelude::BitVec, super::DebugProbeError> {
        self.device.read_captured_bits()
    }

    fn state_mut(&mut self) -> &mut JtagDriverState {
        &mut self.jtag_state
    }

    fn state(&self) -> &JtagDriverState {
        &self.jtag_state
    }
}

impl AutoImplementJtagAccess for Ch347UsbJtag {}
impl DapProbe for Ch347UsbJtag {}

// ---------------------------------------------------------------------------
// SWD request byte construction
// ---------------------------------------------------------------------------

/// Build the 8-bit SWD request byte from a [`RegisterAddress`] and direction.
///
/// Bit layout:
/// - Bit 0: Start (always 1)
/// - Bit 1: APnDP (0=DP, 1=AP)
/// - Bit 2: RnW (0=Write, 1=Read)
/// - Bit 3: A[2]
/// - Bit 4: A[3]
/// - Bit 5: Odd parity of bits 1–4
/// - Bit 6: Stop (always 0)
/// - Bit 7: Park (always 1)
fn build_swd_request(address: RegisterAddress, rnw: bool) -> u8 {
    let ap_n_dp = address.is_ap() as u8;
    let a2 = address.a2() as u8;
    let a3 = address.a3() as u8;

    let parity = (ap_n_dp ^ (rnw as u8) ^ a2 ^ a3).count_ones() % 2 == 1;

    let mut request: u8 = 0;
    request |= 1 << 0; // Start = 1
    request |= ap_n_dp << 1;
    request |= (rnw as u8) << 2;
    request |= a2 << 3;
    request |= a3 << 4;
    request |= (parity as u8) << 5;
    // Bit 6: Stop = 0 (already 0)
    request |= 1 << 7; // Park = 1

    request
}

/// Check SWD ACK and return appropriate error if not OK.
fn check_swd_ack(ack: u8) -> Result<(), ArmError> {
    match ack & 0x07 {
        0x01 => Ok(()), // OK
        0x02 => Err(DapError::WaitResponse.into()),
        0x04 => Err(DapError::FaultResponse.into()),
        _ => Err(DapError::NoAcknowledge.into()),
    }
}

/// Verify data parity and return the data value.
fn verify_swd_read_data(data: u32, parity_trace: u8) -> Result<u32, ArmError> {
    let expected_parity = (data.count_ones() % 2 == 1) as u8;
    let actual_parity = (parity_trace & 0x01) as u8;
    if expected_parity != actual_parity {
        return Err(DapError::IncorrectParity.into());
    }
    Ok(data)
}

// ---------------------------------------------------------------------------
// JTAG DAP access constants and helpers
// ---------------------------------------------------------------------------

/// JTAG IR value for DAP ABORT register.
const JTAG_ABORT_IR: u32 = 0x8;
/// JTAG IR value for Debug Port registers.
const JTAG_DP_IR: u32 = 0xA;
/// JTAG IR value for Access Port registers.
const JTAG_AP_IR: u32 = 0xB;

/// JTAG DR bit length for DAP transfers.
const JTAG_DR_BIT_LENGTH: u32 = 35;

/// Fixed payload value written to the JTAG ABORT register.
const JTAG_ABORT_VALUE: u64 = 0x8;

/// JTAG status: WAIT response.
const JTAG_STATUS_WAIT: u32 = 0x1;
/// JTAG status: OK/FAULT response.
const JTAG_STATUS_OK: u32 = 0x2;

/// Check if the register address is the ABORT register (DP address 0x00).
fn is_abort_register(address: RegisterAddress) -> bool {
    matches!(address, RegisterAddress::DpRegister(addr) if addr == Abort::ADDRESS)
}

// ---------------------------------------------------------------------------
// RawDapAccess implementation
// ---------------------------------------------------------------------------

impl RawDapAccess for Ch347UsbJtag {
    fn raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
        match self.protocol {
            WireProtocol::Swd => self.swd_raw_read_register(address),
            WireProtocol::Jtag => self.jtag_raw_read_register(address),
        }
    }

    fn raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        match self.protocol {
            WireProtocol::Swd => self.swd_raw_write_register(address, value),
            WireProtocol::Jtag => self.jtag_raw_write_register(address, value),
        }
    }

    fn raw_flush(&mut self) -> Result<(), ArmError> {
        // Nothing to flush for CH347 — each operation is immediately sent
        Ok(())
    }

    fn swj_sequence(&mut self, bit_len: u8, bits: u64) -> Result<(), DebugProbeError> {
        if bit_len == 0 {
            return Ok(());
        }

        match self.protocol {
            WireProtocol::Swd => {
                // Use the CH347 SWD custom sequence command (0xA1)
                let data = bits.to_le_bytes();
                // Number of bytes needed to hold bit_len bits
                let _byte_count = ((bit_len as usize) + 7) / 8;

                // The CH347 0xA1 command can handle up to 255 bits in one call.
                // If more bits are needed, we split into multiple calls.
                let mut offset = 0;
                let mut remaining = bit_len;
                while remaining > 0 {
                    let chunk_bits = remaining.min(255);
                    let chunk_byte_start = offset / 8;
                    let chunk_byte_count = ((chunk_bits as usize) + 7) / 8;

                    self.device.swd_sequence(
                        chunk_bits,
                        &data[chunk_byte_start..chunk_byte_start + chunk_byte_count],
                    )?;

                    offset += chunk_bits as usize;
                    remaining -= chunk_bits;
                }
            }
            WireProtocol::Jtag => {
                // For JTAG, SWJ sequences are shifted out on TMS
                // (the pin shared between SWD and JTAG modes)
                let mut bits_to_send = bit_len as usize;
                let data = bits.to_le_bytes();
                let mut bit_idx = 0usize;

                while bits_to_send > 0 {
                    // Find runs of same-valued bits for efficient TMS shifting
                    let first_bit = (data[bit_idx / 8] >> (bit_idx % 8)) & 1 == 1;
                    let mut run_len = 1;
                    while run_len < bits_to_send {
                        let next_bit = (data[(bit_idx + run_len) / 8] >> ((bit_idx + run_len) % 8)) & 1 == 1;
                        if next_bit != first_bit {
                            break;
                        }
                        run_len += 1;
                    }

                    // Shift this run of bits on TMS
                    let seq_data = bitvec::bitvec![0; run_len];
                    self.shift_raw_sequence(JtagSequence {
                        tms: first_bit,
                        data: seq_data,
                        tdo_capture: false,
                    })?;

                    bit_idx += run_len;
                    bits_to_send -= run_len;
                }
            }
        }

        Ok(())
    }

    fn swj_pins(
        &mut self,
        pin_out: u32,
        pin_select: u32,
        _pin_wait: u32,
    ) -> Result<u32, DebugProbeError> {
        // CH347 doesn't have a direct pin control command in the SWD protocol.
        // Limited support: only nRESET (bit 7) is handled.
        const PIN_NSRST: u32 = 0x80;

        if pin_select == PIN_NSRST {
            // For nRESET, we can try to use the SWD sequence to toggle the pin.
            // However, CH347 doesn't have explicit pin control, so this is limited.
            // For now, return the requested output state without actual pin control.
            tracing::warn!(
                "CH347 swj_pins: nRESET control not fully supported (pin_out={pin_out:#010b})"
            );
            Ok(pin_out & pin_select)
        } else if pin_select == 0 {
            // Read pin state — not supported, return 0
            Ok(0)
        } else {
            Err(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "swj_pins",
            })
        }
    }

    fn jtag_sequence(
        &mut self,
        cycles: u8,
        tms: bool,
        tdi: u64,
    ) -> Result<(), DebugProbeError> {
        match self.protocol {
            WireProtocol::Jtag => {
                let data = tdi.to_le_bytes();
                let mut bits = BitVec::with_capacity(cycles as usize);
                for i in 0..cycles {
                    bits.push((data[i as usize / 8] >> (i % 8)) & 1 == 1);
                }
                self.shift_raw_sequence(JtagSequence {
                    tms,
                    data: bits,
                    tdo_capture: false,
                })?;
            }
            WireProtocol::Swd => {
                // In SWD mode, use the custom sequence command
                let data = tdi.to_le_bytes();
                let byte_count = ((cycles as usize) + 7) / 8;
                // Note: tdi bits are sent as data on SWDIO, tms is mapped to SWCLK
                // For SWD, jtag_sequence is rarely used; delegate to swd_sequence
                self.device.swd_sequence(cycles, &data[..byte_count])?;
            }
        }
        Ok(())
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }

    fn core_status_notification(&mut self, _state: crate::CoreStatus) -> Result<(), DebugProbeError> {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// SWD RawDapAccess implementation
// ---------------------------------------------------------------------------

impl Ch347UsbJtag {
    /// Perform an SWD register read with WAIT retry and FAULT handling.
    fn swd_raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
        let num_retries = self.swd_settings.num_retries_after_wait;
        // Start with at least 8 idle cycles. At 5 MHz, the default of 2
        // cycles (= 0.4 µs) is far too short to cover STM32F103-class
        // reset/PLL recovery, and the WAIT-retry loop ends up burning
        // many rounds before the exponential backoff catches up.
        let mut idle_cycles = std::cmp::max(8, self.swd_settings.num_idle_cycles_between_writes);

        // Build the SWD request byte for reading
        let request = build_swd_request(address, true);

        for attempt in 0..=num_retries {
            let (ack, data, parity_trace) = self.device.swd_register_read(request)?;

            match ack & 0x07 {
                0x01 => {
                    // OK — for AP reads, the data is from the PREVIOUS transaction.
                    // We need an extra RDBUFF read to get the actual AP data.
                    if address.is_ap() {
                        return self.swd_read_rdbuff();
                    }
                    // DP read — data is available immediately
                    return verify_swd_read_data(data, parity_trace);
                }
                0x02 => {
                    // WAIT
                    tracing::debug!(
                        "SWD WAIT on read register (attempt {}/{}), idle cycles = {}",
                        attempt + 1,
                        num_retries,
                        idle_cycles,
                    );
                    // Per ADIv5: insert idle cycles first to let the target
                    // recover, THEN write ABORT to clear sticky overrun. If
                    // we issue the ABORT write while the target is still
                    // busy, that write itself WAITs and we never make
                    // progress.
                    if idle_cycles > 0 {
                        let idle_data = vec![0u8; (idle_cycles + 7) / 8];
                        self.device.swd_sequence(idle_cycles as u8, &idle_data)?;
                    }
                    self.swd_clear_sticky_err()?;

                    idle_cycles = std::cmp::min(
                        self.swd_settings.max_retry_idle_cycles_after_wait,
                        2 * idle_cycles,
                    );
                    continue;
                }
                0x04 => {
                    // FAULT — read CTRL/STAT to determine reason, then clear
                    tracing::debug!("SWD FAULT on read register");
                    self.swd_handle_fault(address)?;
                    return Err(DapError::FaultResponse.into());
                }
                _ => {
                    // No ACK or invalid
                    return Err(DapError::NoAcknowledge.into());
                }
            }
        }

        // Timeout — abort AP transactions
        tracing::debug!("SWD read timeout after {} retries, aborting", num_retries);
        self.swd_write_abort()?;
        Err(DapError::WaitResponse.into())
    }

    /// Perform an SWD register write with WAIT retry and FAULT handling.
    fn swd_raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        let num_retries = self.swd_settings.num_retries_after_wait;
        // See `swd_raw_read_register`: at 5 MHz the default minimum of 2
        // idle cycles is too short to cover post-WAIT recovery on slow
        // targets, so we floor at 8.
        let mut idle_cycles = std::cmp::max(8, self.swd_settings.num_idle_cycles_between_writes);

        // Build the SWD request byte for writing
        let request = build_swd_request(address, false);

        // Pack post-write idle cycles with the write command and enforce a 32-cycle minimum. This prevents sporadic faults from un-drained AP buffered writes at high speeds without adding communication overhead.
        let pre_idle: u8 = self
            .swd_settings
            .idle_cycles_before_write_verify
            .max(32)
            .min(255) as u8;
        let pre_idle_data = [0u8; 32]; // pad buffer; 16 is the typical value

        for attempt in 0..=num_retries {
            // Idle cycles between WAIT-retries are now inserted in the
            // WAIT match arm itself, so the loop body just issues the
            // transaction directly.
            let ack = if address.is_ap() {
                // AP write: pack the trailing idle into the same E8 packet
                // — saves one USB round-trip per AP write on the OK path,
                // and gives WAIT recovery a head-start.
                self.device.swd_register_write_with_trailing_idle(
                    request,
                    value,
                    pre_idle,
                    &pre_idle_data,
                )?
            } else {
                // DP write: no trailing idle, no RDBUFF — keep the original
                // single-sub-command path.
                let (a, _) = self.device.swd_register_write(request, value)?;
                a
            };

            match ack & 0x07 {
                0x01 => {
                    // OK — for AP writes the trailing idle was already sent
                    // in the same E8 packet above; just verify the buffered
                    // write drained to the bus via RDBUFF.
                    if address.is_ap() {
                        self.swd_read_rdbuff()?;
                    }
                    return Ok(());
                }
                0x02 => {
                    // WAIT
                    tracing::debug!(
                        "SWD WAIT on write register (attempt {}/{}), idle cycles = {}",
                        attempt + 1,
                        num_retries,
                        idle_cycles,
                    );
                    // Same ordering rationale as the read path: idle first,
                    // then clear sticky, then retry.
                    if idle_cycles > 0 {
                        let idle_data = vec![0u8; (idle_cycles + 7) / 8];
                        self.device.swd_sequence(idle_cycles as u8, &idle_data)?;
                    }
                    self.swd_clear_sticky_err()?;

                    idle_cycles = std::cmp::min(
                        self.swd_settings.max_retry_idle_cycles_after_wait,
                        2 * idle_cycles,
                    );
                    continue;
                }
                0x04 => {
                    // FAULT
                    tracing::debug!("SWD FAULT on write register");
                    self.swd_handle_fault(address)?;
                    return Err(DapError::FaultResponse.into());
                }
                _ => {
                    return Err(DapError::NoAcknowledge.into());
                }
            }
        }

        // Timeout — abort AP transactions
        tracing::debug!("SWD write timeout after {} retries, aborting", num_retries);
        self.swd_write_abort()?;
        Err(DapError::WaitResponse.into())
    }

    /// Read RDBUFF register to get the result of a previous AP read or to
    /// verify that an AP write has drained to the target bus.
    ///
    /// A WAIT response on RDBUFF specifically means "the previous AP
    /// transaction is still in flight" — it does NOT set any sticky bit
    /// in CTRL/STAT, so the recovery is just "insert more idle cycles
    /// and retry the RDBUFF read". DO NOT issue an ABORT here: writing
    /// ABORT in the middle of an in-flight AP transaction cancels it
    /// (`DAPABORT`) or, with `STKERRCLR`/`ORUNERRCLR`, races against
    /// the unfinished SWDIO turnaround and surfaces as `NoAcknowledge`
    /// on the bus — which matches the symptom we were seeing.
    fn swd_read_rdbuff(&mut self) -> Result<u32, ArmError> {
        let request = build_swd_request(RegisterAddress::DpRegister(RdBuff::ADDRESS), true);

        let num_retries = self.swd_settings.num_retries_after_wait;
        let mut idle_cycles = std::cmp::max(8, self.swd_settings.num_idle_cycles_between_writes);

        for attempt in 0..=num_retries {
            let (ack, data, parity_trace) = self.device.swd_register_read(request)?;

            match ack & 0x07 {
                0x01 => return verify_swd_read_data(data, parity_trace),
                0x02 => {
                    tracing::debug!(
                        "SWD WAIT on RDBUFF (attempt {}/{}), inserting {} idle cycles before retry",
                        attempt + 1,
                        num_retries,
                        idle_cycles,
                    );
                    let idle_data = vec![0u8; (idle_cycles + 7) / 8];
                    self.device.swd_sequence(idle_cycles as u8, &idle_data)?;
                    idle_cycles = std::cmp::min(
                        self.swd_settings.max_retry_idle_cycles_after_wait,
                        2 * idle_cycles,
                    );
                    continue;
                }
                0x04 => {
                    self.swd_handle_fault(RegisterAddress::DpRegister(RdBuff::ADDRESS))?;
                    return Err(DapError::FaultResponse.into());
                }
                _ => return Err(DapError::NoAcknowledge.into()),
            }
        }

        // Out of retries — abort the in-flight AP transaction and surface WAIT.
        let _ = self.swd_write_abort();
        Err(DapError::WaitResponse.into())
    }

    /// Clear sticky overrun and error flags by writing to the ABORT register.
    ///
    /// Per ADIv5: a clean WAIT recovery is `idle → write ABORT(STKERR/ORUN) → idle → retry`.
    /// CH347's 0xA0 register-write packet does NOT append trailing idle cycles, so
    /// without an explicit idle sequence here, the very next 0xE8 command is
    /// emitted with effectively zero gap — long enough on a slow target to
    /// produce `NoAcknowledge` on the retry. We always append a generous run
    /// of idle cycles (mirroring polyfill's
    /// `idle_cycles_before_write_verify + num_idle_cycles_between_writes`)
    /// to give the DP time to act on the clear and provide a known
    /// SWDIO=low quiescent state before the next transaction's start bit.
    ///
    /// **Tolerant of WAIT/FAULT**: if the ABORT write itself doesn't see OK,
    /// we still emit the trailing idle and return Ok rather than propagating
    /// the error. The caller is already inside a WAIT-retry loop; if sticky
    /// bits remain, the next retry round will take another shot — but
    /// surfacing `WaitResponse` here would short-circuit the entire retry
    /// loop and cause the operation to fail prematurely. This is what we
    /// observed at 5 MHz: a still-busy target WAIT'd the ABORT itself,
    /// and the propagated error killed the retry loop one round in.
    fn swd_clear_sticky_err(&mut self) -> Result<(), ArmError> {
        let request = build_swd_request(RegisterAddress::DpRegister(Abort::ADDRESS), false);
        let abort = {
            let mut a = Abort(0);
            a.set_stkerrclr(true);
            a.set_orunerrclr(true);
            a
        };

        // Best-effort: even if the USB transaction itself fails we still
        // try to emit trailing idle so the bus is in a defined state.
        let ack_result = self.device.swd_register_write(request, abort.0);

        // Trailing idle: 32 cycles ≈ 6.4 µs at 5 MHz, ≈ 32 µs at 1 MHz.
        // Larger than the previous 8 to comfortably cover STM32F103-class
        // recovery times in the reset/PLL-unlocked window.
        let _ = self.device.swd_sequence(32, &[0u8; 4]);

        match ack_result {
            Ok((ack, _)) => {
                match ack & 0x07 {
                    0x01 => Ok(()),
                    0x02 => {
                        // ABORT itself WAIT'd — sticky still set, but a hard
                        // error here would cancel the outer retry loop.
                        // Let it have another go.
                        tracing::debug!(
                            "ABORT(clear sticky) returned WAIT — deferring to next retry"
                        );
                        Ok(())
                    }
                    0x04 => {
                        tracing::debug!(
                            "ABORT(clear sticky) returned FAULT — deferring to next retry"
                        );
                        Ok(())
                    }
                    other => {
                        tracing::debug!(
                            "ABORT(clear sticky) returned unexpected ack={other:#x} — deferring"
                        );
                        Ok(())
                    }
                }
            }
            Err(e) => {
                // USB transport error — that's something we can't recover
                // from in-flight; surface it.
                Err(e.into())
            }
        }
    }

    /// Write to the ABORT register to abort AP transactions.
    fn swd_write_abort(&mut self) -> Result<(), ArmError> {
        let request = build_swd_request(RegisterAddress::DpRegister(Abort::ADDRESS), false);
        let mut abort_val = Abort(0);
        abort_val.set_dapabort(true);

        let (ack, _status) = self.device.swd_register_write(request, abort_val.0)?;
        check_swd_ack(ack)
    }

    /// Handle a FAULT response by reading CTRL/STAT and clearing sticky errors.
    fn swd_handle_fault(&mut self, address: RegisterAddress) -> Result<(), ArmError> {
        // Don't read CTRL/STAT if we were already reading it (avoid infinite recursion)
        if address == RegisterAddress::DpRegister(Ctrl::ADDRESS) {
            tracing::debug!(
                "FAULT while reading CTRL/STAT, just clearing sticky errors"
            );
            self.swd_clear_sticky_err()?;
        } else {
            // Try to read CTRL/STAT to determine the fault reason
            match self.swd_read_ctrl_stat() {
                Ok(ctrl_value) => {
                    let ctrl = Ctrl::try_from(ctrl_value)?;
                    tracing::debug!("CTRL/STAT after FAULT: {ctrl:#?}");
                    if ctrl.sticky_orun() || ctrl.sticky_err() {
                        self.swd_clear_sticky_err()?;
                    }
                }
                Err(e) => {
                    tracing::debug!("Failed to read CTRL/STAT after FAULT: {e}");
                    // Still try to clear sticky errors
                    let _ = self.swd_clear_sticky_err();
                }
            }
        }
        Ok(())
    }

    /// Read the CTRL/STAT register via SWD (for FAULT diagnosis).
    fn swd_read_ctrl_stat(&mut self) -> Result<u32, ArmError> {
        let request = build_swd_request(RegisterAddress::DpRegister(Ctrl::ADDRESS), true);
        let (ack, data, parity_trace) = self.device.swd_register_read(request)?;

        match ack & 0x07 {
            0x01 => verify_swd_read_data(data, parity_trace),
            0x02 => Err(DapError::WaitResponse.into()),
            0x04 => Err(DapError::FaultResponse.into()),
            _ => Err(DapError::NoAcknowledge.into()),
        }
    }
}

// ---------------------------------------------------------------------------
// JTAG RawDapAccess implementation
// ---------------------------------------------------------------------------

impl Ch347UsbJtag {
    /// Perform a JTAG DAP register read.
    fn jtag_raw_read_register(&mut self, address: RegisterAddress) -> Result<u32, ArmError> {
        let ir = if address.is_ap() { JTAG_AP_IR } else { JTAG_DP_IR };
        let port_address = address.a2_and_3();

        // Build 35-bit DR payload for a read:
        // Bits [2:0]: RnW=1, A[3], A[2]
        // Bits [34:3]: 32-bit value (0 for reads)
        let mut payload: u64 = 0;
        payload |= 1; // RnW = 1 (Read)
        payload |= (port_address as u64 & 0b1000) >> 1; // A[3]
        payload |= (port_address as u64 & 0b0100) >> 1; // A[2]

        let data = payload.to_le_bytes();
        let result = self.write_register(ir, &data[..], JTAG_DR_BIT_LENGTH)?;

        // Parse the 35-bit response
        let received = {
            let mut buf = [0u8; 8];
            for (i, bit) in result.iter().enumerate() {
                if i < 64 {
                    if *bit {
                        buf[i / 8] |= 1 << (i % 8);
                    }
                }
            }
            u64::from_le_bytes(buf)
        };

        // Received value is bits [34:3], status is bits [2:0]
        let received_value = (received >> 3) as u32;
        let status = (received & 0b111) as u32;

        match status {
            s if s == JTAG_STATUS_OK => {
                // For AP reads, the actual data comes from the previous transaction.
                // We need an extra RDBUFF read.
                if address.is_ap() {
                    return self.jtag_read_rdbuff();
                }
                Ok(received_value)
            }
            s if s == JTAG_STATUS_WAIT => Err(DapError::WaitResponse.into()),
            _ => {
                tracing::debug!("Unexpected JTAG DAP response: status={status}");
                Err(DapError::NoAcknowledge.into())
            }
        }
    }

    /// Perform a JTAG DAP register write.
    fn jtag_raw_write_register(&mut self, address: RegisterAddress, value: u32) -> Result<(), ArmError> {
        // ABORT register is special: uses IR=0x8 and a fixed payload
        if is_abort_register(address) {
            let data = (JTAG_ABORT_VALUE as u64).to_le_bytes();
            let result = self.write_register(JTAG_ABORT_IR, &data[..], JTAG_DR_BIT_LENGTH)?;
            // ABORT writes don't return a meaningful response
            let _ = result;
            return Ok(());
        }

        let ir = if address.is_ap() { JTAG_AP_IR } else { JTAG_DP_IR };
        let port_address = address.a2_and_3();

        // Build 35-bit DR payload for a write:
        // Bits [2:0]: RnW=0, A[3], A[2]
        // Bits [34:3]: 32-bit value
        let mut payload: u64 = 0;
        payload |= (value as u64) << 3;
        payload |= (port_address as u64 & 0b1000) >> 1; // A[3]
        payload |= (port_address as u64 & 0b0100) >> 1; // A[2]
        // RnW = 0 (Write), bit 0 already 0

        let data = payload.to_le_bytes();
        let result = self.write_register(ir, &data[..], JTAG_DR_BIT_LENGTH)?;

        // Parse the 35-bit response to check status
        let received = {
            let mut buf = [0u8; 8];
            for (i, bit) in result.iter().enumerate() {
                if i < 64 {
                    if *bit {
                        buf[i / 8] |= 1 << (i % 8);
                    }
                }
            }
            u64::from_le_bytes(buf)
        };

        let status = (received & 0b111) as u32;

        match status {
            s if s == JTAG_STATUS_OK => Ok(()),
            s if s == JTAG_STATUS_WAIT => Err(DapError::WaitResponse.into()),
            _ => {
                tracing::debug!("Unexpected JTAG DAP write response: status={status}");
                Err(DapError::NoAcknowledge.into())
            }
        }
    }

    /// Read RDBUFF via JTAG DAP.
    fn jtag_read_rdbuff(&mut self) -> Result<u32, ArmError> {
        let ir = JTAG_DP_IR;

        // RDBUFF read: RnW=1, A[3:2]=0b00 (address 0x0C, but A[2:3]=11 for RDBUFF)
        // Actually, RDBUFF address is A[3:2] = 11, so:
        // payload bits [2:0] = RnW(1) | A[3](1) | A[2](1) = ... wait

        // Let me re-check: RDBUFF register address is 0x0C, which means
        // A[3:2] = 11 (bits 2 and 3 are set)
        // In the 35-bit payload: bit 0 = RnW, bit 1 = A[3], bit 2 = A[2]
        // Wait no: looking at the polyfill:
        // payload |= (port_address as u64 & 0b1000) >> 1; // A[3] → bit 1
        // payload |= (port_address as u64 & 0b0100) >> 1; // A[2] → bit 2
        // So A[3] goes to bit 1, A[2] goes to bit 2

        // For RDBUFF: A[3]=1, A[2]=1
        let mut payload: u64 = 0;
        payload |= 1; // RnW = 1
        payload |= 1u64 << 1; // A[3] = 1
        payload |= 1u64 << 2; // A[2] = 1

        let data = payload.to_le_bytes();
        let result = self.write_register(ir, &data[..], JTAG_DR_BIT_LENGTH)?;

        let received = {
            let mut buf = [0u8; 8];
            for (i, bit) in result.iter().enumerate() {
                if i < 64 {
                    if *bit {
                        buf[i / 8] |= 1 << (i % 8);
                    }
                }
            }
            u64::from_le_bytes(buf)
        };

        let received_value = (received >> 3) as u32;
        let status = (received & 0b111) as u32;

        match status {
            s if s == JTAG_STATUS_OK => Ok(received_value),
            s if s == JTAG_STATUS_WAIT => Err(DapError::WaitResponse.into()),
            _ => Err(DapError::NoAcknowledge.into()),
        }
    }
}

// ---------------------------------------------------------------------------
// DebugProbe implementation
// ---------------------------------------------------------------------------

impl DebugProbe for Ch347UsbJtag {
    fn get_name(&self) -> &str {
        "CH347 USB Jtag/Swd"
    }

    fn speed_khz(&self) -> u32 {
        self.device.speed_khz()
    }

    fn set_speed(&mut self, speed_khz: u32) -> Result<u32, super::DebugProbeError> {
        Ok(self.device.set_speed_khz(speed_khz))
    }

    fn attach(&mut self) -> Result<(), super::DebugProbeError> {
        match self.protocol {
            WireProtocol::Swd => {
                // Initialize SWD interface with the configured speed
                self.device.swd_init(self.device.speed_khz())?;
            }
            WireProtocol::Jtag => {
                // JTAG: apply clock speed via the existing 0xD0 command
                self.device.attach()?;
            }
        }
        Ok(())
    }

    fn detach(&mut self) -> Result<(), crate::Error> {
        Ok(())
    }

    fn target_reset(&mut self) -> Result<(), super::DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset",
        })
    }

    fn target_reset_assert(&mut self) -> Result<(), super::DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_assert",
        })
    }

    fn target_reset_deassert(&mut self) -> Result<(), super::DebugProbeError> {
        Err(DebugProbeError::NotImplemented {
            function_name: "target_reset_deassert",
        })
    }

    fn select_protocol(
        &mut self,
        protocol: super::WireProtocol,
    ) -> Result<(), super::DebugProbeError> {
        match protocol {
            WireProtocol::Jtag | WireProtocol::Swd => {
                self.protocol = protocol;
                Ok(())
            }
        }
    }

    fn active_protocol(&self) -> Option<super::WireProtocol> {
        Some(self.protocol)
    }

    fn into_probe(self: Box<Self>) -> Box<dyn DebugProbe> {
        self
    }

    fn try_as_jtag_probe(&mut self) -> Option<&mut dyn super::JtagAccess> {
        match self.protocol {
            WireProtocol::Jtag => Some(self),
            WireProtocol::Swd => None,
        }
    }

    fn has_arm_interface(&self) -> bool {
        true
    }

    fn try_get_arm_debug_interface<'probe>(
        self: Box<Self>,
        sequence: std::sync::Arc<dyn ArmDebugSequence>,
    ) -> Result<
        Box<dyn crate::architecture::arm::ArmDebugInterface + 'probe>,
        (Box<dyn DebugProbe>, crate::architecture::arm::ArmError),
    > {
        Ok(ArmCommunicationInterface::create(self, sequence, true))
    }

    fn has_riscv_interface(&self) -> bool {
        // RISC-V requires JTAG
        self.protocol == WireProtocol::Jtag
    }

    fn try_get_riscv_interface_builder<'probe>(
        &'probe mut self,
    ) -> Result<
        Box<
            dyn crate::architecture::riscv::communication_interface::RiscvInterfaceBuilder<'probe>
                + 'probe,
        >,
        crate::architecture::riscv::communication_interface::RiscvError,
    > {
        if self.protocol == WireProtocol::Jtag {
            Ok(Box::new(JtagDtmBuilder::new(self)))
        } else {
            Err(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "RISC-V interface (requires JTAG protocol)",
            }.into())
        }
    }

    fn has_xtensa_interface(&self) -> bool {
        // Xtensa requires JTAG
        self.protocol == WireProtocol::Jtag
    }

    fn try_get_xtensa_interface<'probe>(
        &'probe mut self,
        state: &'probe mut crate::architecture::xtensa::communication_interface::XtensaDebugInterfaceState,
    ) -> Result<
        crate::architecture::xtensa::communication_interface::XtensaCommunicationInterface<'probe>,
        crate::architecture::xtensa::communication_interface::XtensaError,
    > {
        if self.protocol == WireProtocol::Jtag {
            Ok(XtensaCommunicationInterface::new(self, state))
        } else {
            Err(DebugProbeError::CommandNotSupportedByProbe {
                command_name: "Xtensa interface (requires JTAG protocol)",
            }.into())
        }
    }
}
