use core::ptr::{read_volatile, write_bytes, write_volatile};

extern crate alloc;

use crate::{
    KernelFunc, UseKernelFunc,
    constants::{Rtl8169Registers, register_bits::*, *},
    types::{Rtl8169Private, RxDesc},
};

#[repr(align(256))]
#[allow(dead_code)]
struct TxBufferArray([u8; NUM_TX_DESC * RX_BUF_SIZE]);
static mut TXB: TxBufferArray = TxBufferArray([0; NUM_TX_DESC * RX_BUF_SIZE]);

#[repr(align(256))]
#[allow(dead_code)]
struct RxBufferArray([u8; NUM_RX_DESC * RX_BUF_SIZE]);
static mut RXB: RxBufferArray = RxBufferArray([0; NUM_RX_DESC * RX_BUF_SIZE]);

// Static buffer for received packet data
static mut RXDATA: [u8; RX_BUF_LEN] = [0; RX_BUF_LEN];

pub struct Rtl8169 {
    mmio_base: usize,
    device_id: u16,
    /// Driver private data
    priv_data: Rtl8169Private,
    eth_addr: [u8; 6],
}

impl Rtl8169 {
    pub fn new(mmio_base: usize, device_id: u16) -> Self {
        let mut priv_data = Rtl8169Private::new();
        priv_data.iobase = mmio_base;

        Rtl8169 {
            mmio_base,
            device_id,
            priv_data,
            eth_addr: [0; 6],
        }
    }

    /// Read 8-bit register
    #[inline]
    fn read8(&self, reg: usize) -> u8 {
        unsafe { read_volatile((self.mmio_base + reg) as *const u8) }
    }

    /// Write 8-bit register
    #[inline]
    fn write8(&mut self, reg: usize, val: u8) {
        unsafe { write_volatile((self.mmio_base + reg) as *mut u8, val) }
    }

    /// Read 16-bit register
    #[inline]
    fn read16(&self, reg: usize) -> u16 {
        unsafe { read_volatile((self.mmio_base + reg) as *const u16) }
    }

    /// Write 16-bit register
    #[inline]
    fn write16(&mut self, reg: usize, val: u16) {
        unsafe { write_volatile((self.mmio_base + reg) as *mut u16, val) }
    }

    /// Read 32-bit register
    #[inline]
    fn read32(&self, reg: usize) -> u32 {
        unsafe { read_volatile((self.mmio_base + reg) as *const u32) }
    }

    /// Write 32-bit register
    #[inline]
    fn write32(&mut self, reg: usize, val: u32) {
        unsafe { write_volatile((self.mmio_base + reg) as *mut u32, val) }
    }

    #[inline]
    fn mdio_write(&mut self, paddr: usize, val: u32) {
        self.write32(
            Rtl8169Registers::PHYAR as usize,
            (0x80000000 | (paddr & 0xFF) << 16 | val as usize) as u32,
        );

        self.udelay(1000);

        for _ in 0..2000 {
            let phyar = self.read32(Rtl8169Registers::PHYAR as usize);
            if (phyar & 0x80000000) == 0 {
                break;
            } else {
                self.udelay(100);
            }
        }
    }

    #[inline]
    fn mdio_read(&mut self, paddr: usize) -> i32 {
        self.write32(
            Rtl8169Registers::PHYAR as usize,
            0x0 | ((paddr & 0xFF) << 16) as u32,
        );

        self.udelay(1000);

        for _ in 0..2000 {
            let phyar = self.read32(Rtl8169Registers::PHYAR as usize);
            if (phyar & 0x80000000) != 0 {
                return (phyar & 0xFFFF) as i32;
            } else {
                self.udelay(100);
            }
        }

        -1
    }

    pub fn eth_probe(&mut self) -> bool {
        let _region = match self.device_id {
            0x8125 | 0x8161 | 0x8168 => 2,
            _ => 1,
        };

        debug!(
            "Probing RTL8169 at MMIO base {:X}, device ID {:X}",
            self.mmio_base, self.device_id
        );

        if !self.rtl_init() {
            error!("RTL8169 initialization failed");
            return false;
        }

        let mut val = self.read32(Rtl8169Registers::FuncEvent as usize);
        debug!("FuncEvent register value: {:X}", val);
        val &= !RX_DV_GATED_EN;
        self.write32(Rtl8169Registers::FuncEvent as usize, val);
        true
    }

    pub fn eth_start(&mut self) -> bool {
        debug!(
            "Starting RTL8169 at MMIO base {:X}, device ID {:X}",
            self.mmio_base, self.device_id
        );
        self.init_ring();
        self.hw_start();

        unsafe {
            let txb_ptr = core::ptr::addr_of_mut!(TXB).cast::<u8>();
            for i in 0..192 {
                *txb_ptr.add(i) = 0xFF;
            }

            *txb_ptr.add(0) = self.eth_addr[0];
            *txb_ptr.add(1) = self.eth_addr[1];
            *txb_ptr.add(2) = self.eth_addr[2];
            *txb_ptr.add(3) = self.eth_addr[3];
            *txb_ptr.add(4) = self.eth_addr[4];
            *txb_ptr.add(5) = self.eth_addr[5];
        }

        true
    }

    fn init_board(&mut self) -> Result<(), ()> {
        // Board initialization logic
        debug!(
            "Board initialized for RTL8169 at MMIO base {:X}",
            self.mmio_base
        );

        self.write8(Rtl8169Registers::ChipCmd as usize, CMD_RESET);

        for _ in 0..1000 {
            let cmd = self.read8(Rtl8169Registers::ChipCmd as usize);
            if (cmd & CMD_RESET) == 0 {
                break;
            } else {
                self.udelay(10);
            }
        }

        let mut txconfig = self.read32(Rtl8169Registers::TxConfig as usize);
        txconfig = ((txconfig & 0x7c000000) + ((txconfig & 0x00800000) << 2)) >> 24;

        for (index, chip_info) in crate::constants::RTL_CHIP_INFO.iter().enumerate().rev() {
            if chip_info.version == (txconfig as u8) {
                debug!(
                    "Detected chip: {} (version {:X})",
                    chip_info.name, chip_info.version
                );
                self.priv_data.chipset = index as i32;
                return Ok(());
            }
        }

        debug!("Unknown chip version: {:X}", txconfig);
        return Err(());
    }

    fn udelay(&self, usecs: u32) {
        for _ in 0..usecs * 100 {
            unsafe { core::arch::asm!("nop") };
        }
    }

    fn rtl_init(&mut self) -> bool {
        debug!("Initializing RTL8169 at MMIO base {:X}", self.mmio_base);
        let mut board_idx: i32 = -1;
        board_idx += 1;

        if self.init_board().is_err() {
            error!("Board initialization failed");
            return false;
        }

        for i in 0..6 {
            self.eth_addr[i] = self.read8(Rtl8169Registers::MAC0 as usize + i);
        }

        debug!(
            "MAC Address: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            self.eth_addr[0],
            self.eth_addr[1],
            self.eth_addr[2],
            self.eth_addr[3],
            self.eth_addr[4],
            self.eth_addr[5]
        );

        let phy_status = self.read8(Rtl8169Registers::PHYstatus as usize);
        if (phy_status & TBI_ENABLE) == 0 {
            let value = self.mdio_read(PHY_AUTO_NEGO_REG as usize);
            let option: i32 = if board_idx >= (crate::constants::MAX_UNITS as i32) {
                0
            } else {
                -1
            }; // ?? TODO
            if option > 0 {
                let mut cap10_100 = 0;
                let mut cap1000 = 0;
                match option as u32 {
                    HALF_10 => {
                        cap10_100 = PHY_CAP_10_HALF;
                        cap1000 = PHY_CAP_NULL;
                    }

                    FULL_10 => {
                        cap10_100 = PHY_CAP_10_FULL;
                        cap1000 = PHY_CAP_NULL;
                    }

                    HALF_100 => {
                        cap10_100 = PHY_CAP_100_HALF;
                        cap1000 = PHY_CAP_NULL;
                    }

                    FULL_100 => {
                        cap10_100 = PHY_CAP_100_FULL;
                        cap1000 = PHY_CAP_NULL;
                    }

                    FULL_1000 => {
                        cap10_100 = PHY_CAP_NULL;
                        cap1000 = PHY_CAP_1000_FULL;
                    }

                    _ => {}
                }

                self.mdio_write(
                    PHY_AUTO_NEGO_REG as usize,
                    (cap10_100 | (value & 0x1F) as u16) as u32,
                );

                self.mdio_write(PHY_1000_CTRL_REG as usize, cap1000 as u32);
            } else {
                self.mdio_write(
                    PHY_AUTO_NEGO_REG as usize,
                    (PHY_CAP_10_HALF
                        | PHY_CAP_10_FULL
                        | PHY_CAP_100_HALF
                        | PHY_CAP_100_FULL
                        | (value & 0x1F) as u16) as u32,
                );

                self.mdio_write(PHY_1000_CTRL_REG as usize, PHY_CAP_1000_FULL as u32);
            }

            self.mdio_write(
                PHY_CTRL_REG as usize,
                (PHY_ENABLE_AUTO_NEGO | PHY_RESTART_AUTO_NEGO) as u32,
            );

            self.udelay(100);

            for _ in 0..1000 {
                let status = self.mdio_read(PHY_STAT_REG as usize);
                if (status & PHY_AUTO_NEGO_COMP as i32) != 0 {
                    self.udelay(100);
                    let option = self.mdio_read(Rtl8169Registers::PHYstatus as usize);

                    if (option & _1000BPSF as i32) != 0 {
                        debug!("Link up at 1000Mbps Full Duplex");
                    } else {
                        debug!("Link up at 10/100Mbps");
                    }
                    break;
                } else {
                    self.udelay(1000);
                }
            }
        } else {
            self.udelay(100);
            let tbiscr = self.read32(Rtl8169Registers::TBICSR as usize) & TBILINK_OK;

            debug!(
                "TBI link status: {}",
                if tbiscr != 0 { "up" } else { "down" }
            );
        }

        self.priv_data.rx_desc_array = Self::rtl_alloc_descs(NUM_RX_DESC) as *mut RxDesc;
        if self.priv_data.rx_desc_array.is_null() {
            error!("Failed to allocate RX descriptor array");
            return false;
        }

        self.priv_data.tx_desc_array =
            Self::rtl_alloc_descs(NUM_TX_DESC) as *mut crate::types::TxDesc;
        if self.priv_data.tx_desc_array.is_null() {
            error!("Failed to allocate TX descriptor array");
            return false;
        }

        true
    }

    fn rtl_alloc_descs(num: usize) -> *mut u8 {
        use crate::constants::RTL8169_DESC_SIZE;
        let size = num * RTL8169_DESC_SIZE;
        Self::memalign(RTL8169_ALIGN, size)
    }

    fn memalign(alignment: usize, size: usize) -> *mut u8 {
        unsafe {
            let layout = match core::alloc::Layout::from_size_align(
                size + alignment + core::mem::size_of::<usize>(),
                1,
            ) {
                Ok(l) => l,
                Err(_) => return core::ptr::null_mut(),
            };

            let ptr = alloc::alloc::alloc(layout);
            if ptr.is_null() {
                return core::ptr::null_mut();
            }

            let addr = ptr as usize + core::mem::size_of::<usize>();
            let aligned_addr = (addr + alignment - 1) & !(alignment - 1);

            let orig_ptr = (aligned_addr - core::mem::size_of::<usize>()) as *mut *mut u8;
            *orig_ptr = ptr;

            aligned_addr as *mut u8
        }
    }

    #[inline]
    fn rtl_inval_rx_desc(&self, desc: *mut RxDesc) {
        UseKernelFunc::invalidate_dcache_range(desc as usize, core::mem::size_of::<RxDesc>());
    }

    #[inline]
    fn rtl_flush_rx_desc(&self, desc: *mut RxDesc) {
        UseKernelFunc::clean_dcache_range(desc as usize, core::mem::size_of::<RxDesc>());
    }

    #[inline]
    fn rtl_inval_tx_desc(&self, desc: *mut crate::types::TxDesc) {
        UseKernelFunc::invalidate_dcache_range(desc as usize, core::mem::size_of::<crate::types::TxDesc>());
    }

    #[inline]
    fn rtl_flush_tx_desc(&self, desc: *mut crate::types::TxDesc) {
        UseKernelFunc::clean_dcache_range(desc as usize, core::mem::size_of::<crate::types::TxDesc>());
    }

    #[inline]
    fn rtl_inval_buffer(&self, buf: *mut u8, size: usize) {
        UseKernelFunc::invalidate_dcache_range(buf as usize, size);
    }

    #[inline]
    fn rtl_flush_buffer(&self, buf: *mut u8, size: usize) {
        UseKernelFunc::clean_dcache_range(buf as usize, size);
    }

    #[inline]
    fn dm_pci_mem_to_phys(&self, addr: usize) -> u32 {
        addr as u32
    }

    fn hw_start(&mut self) {
        debug!(
            "Starting hardware for RTL8169 at MMIO base {:X}",
            self.mmio_base
        );

        self.write8(Rtl8169Registers::Cfg9346 as usize, CFG9346_UNLOCK);

        if self.priv_data.chipset <= 5 {
            self.write8(Rtl8169Registers::ChipCmd as usize, CMD_TX_ENB | CMD_RX_ENB);
        }

        self.write8(Rtl8169Registers::EarlyTxThres as usize, EARLY_TX_THLD);

        self.write16(Rtl8169Registers::RxMaxSize as usize, RX_PACKET_MAXSIZE);

        let rxconfig = RTL8169_RX_CONFIG
            | (self.read32(Rtl8169Registers::RxConfig as usize)
                & RTL_CHIP_INFO[self.priv_data.chipset as usize].rx_config_mask);
        self.write32(Rtl8169Registers::RxConfig as usize, rxconfig);

        self.write32(
            Rtl8169Registers::TxConfig as usize,
            (TX_DMA_BURST << 8) | (INTER_FRAME_GAP << 24),
        );

        self.priv_data.cur_rx = 0;

        self.write32(
            Rtl8169Registers::TxDescStartAddrLow as usize,
            self.dm_pci_mem_to_phys(self.priv_data.tx_desc_array as usize),
        );
        self.write32(Rtl8169Registers::TxDescStartAddrHigh as usize, 0);
        self.write32(
            Rtl8169Registers::RxDescStartAddrLow as usize,
            self.dm_pci_mem_to_phys(self.priv_data.rx_desc_array as usize),
        );
        self.write32(Rtl8169Registers::RxDescStartAddrHigh as usize, 0);

        if self.priv_data.chipset > 5 {
            self.write8(Rtl8169Registers::ChipCmd as usize, CMD_TX_ENB | CMD_RX_ENB);
        }

        self.write8(Rtl8169Registers::Cfg9346 as usize, CFG9346_LOCK);
        self.udelay(10);

        self.write32(Rtl8169Registers::RxMissed as usize, 0);

        self.set_rx_mode();

        let multi_intr = self.read16(Rtl8169Registers::MultiIntr as usize);
        self.write16(Rtl8169Registers::MultiIntr as usize, multi_intr & 0xF000);
    }

    fn set_rx_mode(&mut self) {
        let mc_filter: [u32; 2] = [0xffffffff, 0xffffffff];
        let rx_mode = ACCEPT_BROADCAST | ACCEPT_MULTICAST | ACCEPT_MY_PHYS;

        let tmp = RTL8169_RX_CONFIG
            | rx_mode
            | (self.read32(Rtl8169Registers::RxConfig as usize)
                & RTL_CHIP_INFO[self.priv_data.chipset as usize].rx_config_mask);

        self.write32(Rtl8169Registers::RxConfig as usize, tmp);
        self.write32(Rtl8169Registers::MAR0 as usize, mc_filter[0]);
        self.write32(Rtl8169Registers::MAR0 as usize + 4, mc_filter[1]);
    }

    fn init_ring(&mut self) {
        unsafe {
            self.priv_data.cur_rx = 0;
            self.priv_data.cur_tx = 0;
            self.priv_data.dirty_tx = 0;

            if !self.priv_data.tx_desc_array.is_null() {
                write_bytes(self.priv_data.tx_desc_array, 0, NUM_TX_DESC);
            }
            if !self.priv_data.rx_desc_array.is_null() {
                write_bytes(self.priv_data.rx_desc_array, 0, NUM_RX_DESC);
            }

            for i in 0..NUM_TX_DESC {
                self.priv_data.tx_skbuff[i] = core::ptr::addr_of_mut!(TXB)
                    .cast::<u8>()
                    .add(i * RX_BUF_SIZE);
            }

            for i in 0..NUM_RX_DESC {
                let status = if i == (NUM_RX_DESC - 1) {
                    (OWN_BIT | EOR_BIT) + RX_BUF_SIZE as u32
                } else {
                    OWN_BIT + RX_BUF_SIZE as u32
                };

                if !self.priv_data.rx_desc_array.is_null() {
                    (*self.priv_data.rx_desc_array.add(i)).status = status;
                }

                self.priv_data.rx_buffer_ring[i] = core::ptr::addr_of_mut!(RXB)
                    .cast::<u8>()
                    .add(i * RX_BUF_SIZE);

                if !self.priv_data.rx_desc_array.is_null() {
                    let buf_addr =
                        self.dm_pci_mem_to_phys(self.priv_data.rx_buffer_ring[i] as usize);
                    (*self.priv_data.rx_desc_array.add(i)).buf_addr = buf_addr;
                    self.rtl_flush_rx_desc(self.priv_data.rx_desc_array.add(i));
                }
            }
        }
    }

    /// Send a packet
    /// Returns 0 on success, negative error code on failure
    pub fn eth_send(&mut self, packet: &[u8], length: usize) -> i32 {
        unsafe {
            let entry = self.priv_data.cur_tx % NUM_TX_DESC;
            let mut len = length;

            debug!("rtl_send_common");
            debug!("sending {} bytes", len);

            // Point to the current txb in case multiple tx_rings are used
            let ptxb = self.priv_data.tx_skbuff[entry];

            // Copy packet data
            core::ptr::copy_nonoverlapping(packet.as_ptr(), ptxb, length);

            // Pad to minimum ethernet frame size
            while len < ETH_ZLEN {
                *ptxb.add(len) = 0;
                len += 1;
            }

            // Flush buffer cache
            let aligned_len = (len + RTL8169_ALIGN - 1) & !(RTL8169_ALIGN - 1);
            self.rtl_flush_buffer(ptxb, aligned_len);

            // Setup TX descriptor
            if !self.priv_data.tx_desc_array.is_null() {
                let tx_desc = &mut *self.priv_data.tx_desc_array.add(entry);

                tx_desc.buf_haddr = 0;
                tx_desc.buf_addr = self.dm_pci_mem_to_phys(ptxb as usize);

                let status_len = if len > ETH_ZLEN { len } else { ETH_ZLEN } as u32;
                if entry != (NUM_TX_DESC - 1) {
                    tx_desc.status = (OWN_BIT | FS_BIT | LS_BIT) | status_len;
                } else {
                    tx_desc.status = (OWN_BIT | EOR_BIT | FS_BIT | LS_BIT) | status_len;
                }

                // Flush TX descriptor
                self.rtl_flush_tx_desc(tx_desc as *mut crate::types::TxDesc);
            }

            // Trigger TX by setting polling bit
            if self.device_id == 0x8125 {
                self.write8(TX_POLL_8125, 0x1);
            } else {
                self.write8(TX_POLL_8169, 0x40);
            }

            self.priv_data.cur_tx += 1;

            // Wait for TX completion with timeout
            // TX_TIMEOUT is 6000ms, check every 100us, max iterations = 60000
            let max_iterations = 60000;
            let mut tx_complete = false;

            for _ in 0..max_iterations {
                if !self.priv_data.tx_desc_array.is_null() {
                    let tx_desc = self.priv_data.tx_desc_array.add(entry);

                    // Invalidate descriptor cache
                    self.rtl_inval_tx_desc(tx_desc);

                    if ((*tx_desc).status & OWN_BIT) == 0 {
                        tx_complete = true;
                        break;
                    }
                }

                self.udelay(100);
            }

            if !tx_complete {
                error!("tx timeout/error");
                self.udelay(20);
                return -110; // -ETIMEDOUT
            }

            debug!("tx done");
            self.udelay(20);
            0
        }
    }

    /// Receive a packet
    /// Returns the length of received packet on success (positive value),
    /// 0 if no packet available, or negative error code on failure
    pub fn eth_recv(&mut self, packet_out: &mut [u8]) -> i32 {
        unsafe {
            let cur_rx = self.priv_data.cur_rx;

            // Invalidate RX descriptor to read fresh data from memory
            if !self.priv_data.rx_desc_array.is_null() {
                let rx_desc = self.priv_data.rx_desc_array.add(cur_rx);
                self.rtl_inval_rx_desc(rx_desc);

                // Check if packet is owned by CPU (OWN_BIT cleared by hardware)
                if ((*rx_desc).status & OWN_BIT) == 0 {
                    // Check for RX errors
                    if ((*rx_desc).status & RX_RES) == 0 {
                        // Extract packet length (bits 0-12), minus 4 bytes CRC
                        let length = (((*rx_desc).status & 0x00001FFF) - 4) as usize;

                        // Invalidate buffer to ensure CPU reads fresh data
                        self.rtl_inval_buffer(self.priv_data.rx_buffer_ring[cur_rx], length);

                        // Copy received data to RXDATA buffer
                        let rxdata_ptr = core::ptr::addr_of_mut!(RXDATA);
                        core::ptr::copy_nonoverlapping(
                            self.priv_data.rx_buffer_ring[cur_rx],
                            rxdata_ptr.cast::<u8>(),
                            length,
                        );

                        // Copy to output buffer
                        let copy_len = core::cmp::min(length, packet_out.len());
                        core::ptr::copy_nonoverlapping(
                            rxdata_ptr.cast::<u8>(),
                            packet_out.as_mut_ptr(),
                            copy_len,
                        );

                        // Re-own the descriptor for hardware
                        let new_status = if cur_rx == NUM_RX_DESC - 1 {
                            (OWN_BIT | EOR_BIT) + RX_BUF_SIZE as u32
                        } else {
                            OWN_BIT + RX_BUF_SIZE as u32
                        };

                        let rx_desc_mut =
                            &mut *(self.priv_data.rx_desc_array.add(cur_rx) as *mut RxDesc);
                        rx_desc_mut.status = new_status;
                        rx_desc_mut.buf_addr =
                            self.dm_pci_mem_to_phys(self.priv_data.rx_buffer_ring[cur_rx] as usize);

                        // Flush descriptor back to memory
                        self.rtl_flush_rx_desc(self.priv_data.rx_desc_array.add(cur_rx));

                        // Update current RX index
                        self.priv_data.cur_rx = (cur_rx + 1) % NUM_RX_DESC;

                        return length as i32;
                    } else {
                        error!("Error Rx");
                        self.priv_data.cur_rx = (cur_rx + 1) % NUM_RX_DESC;
                        return -1; // -EIO
                    }
                } else {
                    // No packet ready, clear any pending interrupt status
                    let intr_status_reg = if self.device_id == 0x8125 {
                        INTR_STATUS_8125
                    } else {
                        INTR_STATUS_8169
                    };

                    let sts = self.read8(intr_status_reg); // 改为8位
                    self.write8(
                        intr_status_reg,
                        sts & !((TX_ERR | RX_ERR | SYS_ERR) as u8), // 改为8位，并转换掩码类型
                    );
                    self.udelay(100);
                }
            }

            0 // No packet available
        }
    }

    /// Stop the chip's TX and RX DMA processes
    pub fn eth_stop(&mut self) {
        debug!("rtl_halt_common");

        // Stop the chip's Tx and Rx DMA processes
        self.write8(Rtl8169Registers::ChipCmd as usize, 0x00);

        // Disable interrupts by clearing the interrupt mask
        if self.device_id == 0x8125 {
            self.write16(INTR_MASK_8125, 0x0000);
        } else {
            self.write16(INTR_MASK_8169, 0x0000);
        }

        // Clear RxMissed counter
        self.write32(Rtl8169Registers::RxMissed as usize, 0);

        // Clear RX buffer ring pointers
        for i in 0..NUM_RX_DESC {
            self.priv_data.rx_buffer_ring[i] = core::ptr::null_mut();
        }
    }

    /// Write hardware MAC address to device registers
    /// This function writes the MAC address stored in eth_addr to the hardware MAC0 registers
    pub fn write_hwaddr(&mut self) -> i32 {
        debug!("rtl8169_write_hwaddr");

        // Unlock configuration registers
        self.write8(Rtl8169Registers::Cfg9346 as usize, CFG9346_UNLOCK);

        // Write each byte of MAC address to MAC0 register
        for i in 0..6 {
            self.write8(Rtl8169Registers::MAC0 as usize + i, self.eth_addr[i]);
        }

        // Lock configuration registers
        self.write8(Rtl8169Registers::Cfg9346 as usize, CFG9346_LOCK);

        0
    }
}
