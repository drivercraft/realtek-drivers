/// RTL8125 register definitions
#[repr(u32)]
#[allow(dead_code)]
pub enum Rtl8125Registers {
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

    // PHYstatus
    pub const LINK_STATUS: u8 = 0x02;
    pub const FULL_DUP: u8 = 0x01;
    pub const _1000BPSF: u8 = 0x10;
    pub const _100BPS: u8 = 0x08;
    pub const _10BPS: u8 = 0x04;

    // FuncEvent/Misc
    pub const RX_DV_GATED_EN: u32 = 0x80000;
}

/// Configuration constants
pub const NUM_TX_DESC: usize = 1;
pub const NUM_RX_DESC: usize = 4;
pub const RX_BUF_SIZE: usize = 1536;
pub const MAX_ETH_FRAME_SIZE: usize = 1536;
pub const RX_FIFO_THRESH: u32 = 7;
pub const RX_DMA_BURST: u32 = 6;
pub const TX_DMA_BURST: u32 = 6;
pub const EARLY_TX_THLD: u8 = 0x3F;
pub const RX_PACKET_MAX_SIZE: u16 = 0x0800;
pub const INTER_FRAME_GAP: u32 = 0x03;
pub const TX_TIMEOUT: usize = 6000; // milliseconds

/// Descriptor status bits
#[allow(dead_code)]
pub mod desc_status {
    pub const OWN: u32 = 0x80000000;
    pub const EOR: u32 = 0x40000000;
    pub const FS: u32 = 0x20000000;
    pub const LS: u32 = 0x10000000;
}
