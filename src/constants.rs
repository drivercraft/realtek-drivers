//! Network protocol constants
//!
//! This module contains constant definitions for Ethernet, IP, and ICMP protocols,
//! corresponding to the constants defined in realtek/rtl8169.h

/// Ethernet address length (MAC address)
pub const ETH_ALEN: usize = 6;

/// Ethernet header length
pub const ETH_HLEN: usize = 14;

/// Ethernet protocol type: IPv4
pub const ETH_P_IP: u16 = 0x0800;

/// Ethernet protocol type: ARP
pub const ETH_P_ARP: u16 = 0x0806;

/// IP protocol number: ICMP
pub const IPPROTO_ICMP: u8 = 1;

/// ICMP message type: Echo Request (ping)
pub const ICMP_ECHO: u8 = 8;

/// ICMP message type: Echo Reply (pong)
pub const ICMP_ECHOREPLY: u8 = 0;

/// Number of TX descriptor registers
pub const NUM_TX_DESC: usize = 1;

/// Number of RX descriptor registers
pub const NUM_RX_DESC: usize = 4;

/// RX buffer size
pub const RX_BUF_SIZE: usize = 1536;

/// RX buffer total length for received data
pub const RX_BUF_LEN: usize = 8192;

/// RTL8169 descriptor alignment requirement
pub const RTL8169_ALIGN: usize = 256;

/// RTL8169 descriptor size (in bytes)
pub const RTL8169_DESC_SIZE: usize = 16;

pub const EARLY_TX_THLD: u8 = 0x3F; // Example threshold value
pub const RX_PACKET_MAXSIZE: u16 = 0x0800; // Maximum RX packet size including Ethernet header and CRC

pub const RX_FIFO_THRESH: u32 = 7;
pub const RX_DMA_BURST: u32 = 6;
pub const TX_DMA_BURST: u32 = 6;
pub const INTER_FRAME_GAP: u32 = 0x03;

pub const RTL8169_RX_CONFIG: u32 = (RX_FIFO_THRESH << 13) | (RX_DMA_BURST << 8);

pub const MAX_UNITS: usize = 8;

/// Minimum ethernet frame size
pub const ETH_ZLEN: usize = 60;

/// Maximum ethernet frame size
pub const MAX_ETH_FRAME_SIZE: usize = 1536;

/// TX timeout in system ticks (6 * HZ, where HZ = 1000)
pub const TX_TIMEOUT: u64 = 6000;

/// TxPoll register offset for RTL8169
pub const TX_POLL_8169: usize = 0x38;

/// TxPoll register offset for RTL8125
pub const TX_POLL_8125: usize = 0x90;

/// IntrMask register offset for RTL8169
pub const INTR_MASK_8169: usize = 0x3C;

/// IntrStatus register offset for RTL8169
pub const INTR_STATUS_8169: usize = 0x3E;

/// IntrMask register offset for RTL8125
pub const INTR_MASK_8125: usize = 0x38;

/// IntrStatus register offset for RTL8125
pub const INTR_STATUS_8125: usize = 0x3C;

/// RTL8125 register definitions
#[repr(u32)]
#[allow(dead_code)]
pub enum Rtl8169Registers {
    MAC0 = 0x00, // Ethernet hardware address
    MAR0 = 0x08, // Multicast filter
    TxDescStartAddrLow = 0x20,
    TxDescStartAddrHigh = 0x24,
    TxHDescStartAddrLow = 0x28,
    TxHDescStartAddrHigh = 0x2c,
    FLASH = 0x30,
    ERSR = 0x36,
    ChipCmd = 0x37,
    IntrMask = 0x38,   // RTL8125 specific
    IntrStatus = 0x3C, // RTL8125 specific
    TxConfig = 0x40,
    RxConfig = 0x44,
    RxMissed = 0x4C,
    Cfg9346 = 0x50,
    Config0 = 0x51,
    Config1 = 0x52,
    Config2 = 0x53,
    Config3 = 0x54,
    Config4 = 0x55,
    Config5 = 0x56,
    MultiIntr = 0x5C,
    PHYAR = 0x60,
    TBICSR = 0x64,
    TbiAnar = 0x68,
    TbiLpar = 0x6A,
    PHYstatus = 0x6C,
    TxPoll = 0x90, // RTL8125 specific
    RxMaxSize = 0xDA,
    CPlusCmd = 0xE0,
    RxDescStartAddrLow = 0xE4,
    RxDescStartAddrHigh = 0xE8,
    EarlyTxThres = 0xEC,
    FuncEvent = 0xF0,
    FuncEventMask = 0xF4,
    FuncPresetState = 0xF8,
    FuncForceEvent = 0xFC,
}

/// Register content bit definitions
#[allow(dead_code)]
pub mod register_bits {
    // Interrupt Status Bits
    pub const SYS_ERR: u16 = 0x8000;
    pub const PCS_TIMEOUT: u16 = 0x4000;
    pub const SW_INT: u16 = 0x0100;
    pub const TX_DESC_UNAVAIL: u16 = 0x0080;
    pub const RX_FIFO_OVER: u16 = 0x0040;
    pub const RX_UNDERRUN: u16 = 0x0020;
    pub const RX_OVERFLOW: u16 = 0x0010;
    pub const TX_ERR: u16 = 0x0008;
    pub const TX_OK: u16 = 0x0004;
    pub const RX_ERR: u16 = 0x0002;
    pub const RX_OK: u16 = 0x0001;

    // RxStatusDesc
    pub const RX_RES: u32 = 0x00200000;
    pub const RX_CRC: u32 = 0x00080000;
    pub const RX_RUNT: u32 = 0x00100000;
    pub const RX_RWT: u32 = 0x00400000;

    // ChipCmd bits
    pub const CMD_RESET: u8 = 0x10;
    pub const CMD_RX_ENB: u8 = 0x08;
    pub const CMD_TX_ENB: u8 = 0x04;
    pub const RX_BUF_EMPTY: u8 = 0x01;

    // Cfg9346 bits
    pub const CFG9346_LOCK: u8 = 0x00;
    pub const CFG9346_UNLOCK: u8 = 0xC0;

    // rx_mode bits
    pub const ACCEPT_ERR: u32 = 0x20;
    pub const ACCEPT_RUNT: u32 = 0x10;
    pub const ACCEPT_BROADCAST: u32 = 0x08;
    pub const ACCEPT_MULTICAST: u32 = 0x04;
    pub const ACCEPT_MY_PHYS: u32 = 0x02;
    pub const ACCEPT_ALL_PHYS: u32 = 0x01;

    // PHY registers
    pub const PHY_CTRL_REG: u8 = 0;
    pub const PHY_STAT_REG: u8 = 1;
    pub const PHY_AUTO_NEGO_REG: u8 = 4;
    pub const PHY_1000_CTRL_REG: u8 = 9;

    // PHY bits
    pub const PHY_RESTART_AUTO_NEGO: u16 = 0x0200;
    pub const PHY_ENABLE_AUTO_NEGO: u16 = 0x1000;
    pub const PHY_AUTO_NEGO_COMP: u16 = 0x0020;

    // PHY capabilities
    pub const PHY_CAP_10_HALF: u16 = 0x0020;
    pub const PHY_CAP_10_FULL: u16 = 0x0040;
    pub const PHY_CAP_100_HALF: u16 = 0x0080;
    pub const PHY_CAP_100_FULL: u16 = 0x0100;
    pub const PHY_CAP_1000_FULL: u16 = 0x0200;
    pub const PHY_CAP_NULL : u16 = 0x0000;

    // PHYstatus
    pub const TBI_ENABLE: u8 = 0x80;
    pub const LINK_STATUS: u8 = 0x02;
    pub const FULL_DUP: u8 = 0x01;
    pub const _1000BPSF: u8 = 0x10;
    pub const _100BPS: u8 = 0x08;
    pub const _10BPS: u8 = 0x04;

    // FuncEvent/Misc
    pub const RX_DV_GATED_EN: u32 = 0x80000;

    // Descriptor Status Bits
    pub const OWN_BIT: u32 = 0x80000000;
    pub const EOR_BIT: u32 = 0x40000000;
    pub const FS_BIT: u32 = 0x20000000;
    pub const LS_BIT: u32 = 0x10000000;

    // Media Type
    pub const HALF_10: u32 = 0x01;
    pub const FULL_10: u32 = 0x02;
    pub const HALF_100: u32 = 0x04;
    pub const FULL_100: u32 = 0x08;
    pub const FULL_1000: u32 = 0x10;

    pub const TBILINK_OK: u32 = 0x02000000;
}

/// RTL chip information structure
///
/// Corresponds to the anonymous struct in rtl_chip_info array in C code.
/// This structure contains chip identification and configuration data.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RtlChipInfo {
    /// Chip name string
    pub name: &'static str,
    /// Chip version identifier (from RTL8169 documentation)
    pub version: u8,
    /// RX configuration mask - bits to clear for this chip
    pub rx_config_mask: u32,
}

/// RTL chip information database
///
/// Array of supported RTL8169/8168/8125 chip variants with their version
/// identifiers and configuration masks. The array index serves as the chipset ID
/// used throughout the driver code.
///
/// Corresponds to `rtl_chip_info[]` in realtek/rtl8169.c
pub const RTL_CHIP_INFO: [RtlChipInfo; 17] = [
    RtlChipInfo {
        name: "RTL-8169",
        version: 0x00,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8169",
        version: 0x04,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8169",
        version: 0x00,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8169s/8110s",
        version: 0x02,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8169s/8110s",
        version: 0x04,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8169sb/8110sb",
        version: 0x10,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8169sc/8110sc",
        version: 0x18,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8168b/8111sb",
        version: 0x30,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8168b/8111sb",
        version: 0x38,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8168c/8111c",
        version: 0x3c,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8168d/8111d",
        version: 0x28,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8168evl/8111evl",
        version: 0x2e,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8168/8111g",
        version: 0x4c,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8101e",
        version: 0x34,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8100e",
        version: 0x32,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8168h/8111h",
        version: 0x54,
        rx_config_mask: 0xff7e1880,
    },
    RtlChipInfo {
        name: "RTL-8125B",
        version: 0x64,
        rx_config_mask: 0xff7e1880,
    },
];

impl RtlChipInfo {
    /// Get the number of chip variants in the database
    pub const fn count() -> usize {
        RTL_CHIP_INFO.len()
    }

    /// Find chip info by version number, searching from the end of the array
    ///
    /// This matches the C code's search pattern in `rtl8169_init_board()` which
    /// searches backward through the array.
    ///
    /// Returns the chipset index and a reference to the chip info if found.
    pub fn find_by_version(version: u8) -> Option<(usize, &'static RtlChipInfo)> {
        for i in (0..RTL_CHIP_INFO.len()).rev() {
            if RTL_CHIP_INFO[i].version == version {
                return Some((i, &RTL_CHIP_INFO[i]));
            }
        }
        None
    }
}