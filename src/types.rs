//! Network protocol and device type definitions
//!
//! This module contains structure definitions for network protocols and RTL8169 device,
//! corresponding to the structures defined in realtek/rtl8169.h
//!
//! All packed structures use `#[repr(C, packed)]` to ensure identical memory layout
//! with their C counterparts, with no padding bytes between fields.

/// Ethernet frame header
///
/// Corresponds to C structure: `eth_hdr_t`
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct EthHdr {
    /// Destination MAC address
    pub dest: [u8; 6],
    /// Source MAC address
    pub src: [u8; 6],
    /// Protocol type (e.g., ETH_P_IP, ETH_P_ARP)
    pub proto: u16,
}

/// IPv4 header
///
/// Corresponds to C structure: `ip_hdr_t`
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct IpHdr {
    /// Version (4 bits) and header length (4 bits)
    pub version_ihl: u8,
    /// Type of service
    pub tos: u8,
    /// Total length of the IP packet
    pub total_len: u16,
    /// Identification
    pub id: u16,
    /// Fragment offset
    pub frag_off: u16,
    /// Time to live
    pub ttl: u8,
    /// Protocol (e.g., IPPROTO_ICMP)
    pub protocol: u8,
    /// Header checksum
    pub checksum: u16,
    /// Source IP address
    pub src_addr: u32,
    /// Destination IP address
    pub dest_addr: u32,
}

/// ICMP header
///
/// Corresponds to C structure: `icmp_hdr_t`
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct IcmpHdr {
    /// ICMP message type (e.g., ICMP_ECHO, ICMP_ECHOREPLY)
    pub type_: u8,
    /// ICMP message code
    pub code: u8,
    /// ICMP checksum
    pub checksum: u16,
    /// Identifier (for echo request/reply)
    pub id: u16,
    /// Sequence number (for echo request/reply)
    pub sequence: u16,
}

/// RTL8169 descriptor structure
///
/// Corresponds to C structure: `rtl_desc_t`
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct RtlDesc {
    /// Descriptor status and control flags
    pub status: u32,
    /// VLAN tag
    pub vlan_tag: u32,
    /// Buffer address (lower 32 bits)
    pub buf_addr_lo: u32,
    /// Buffer address (upper 32 bits)
    pub buf_addr_hi: u32,
}

/// PCI device identification
///
/// Corresponds to C structure: `pci_child_plat`
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PciChildPlat {
    /// PCI vendor ID
    pub vendor: u16,
    /// PCI device ID
    pub device: u16,
}

/// Device structure
///
/// Corresponds to C structure: `udevice`
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct UDevice {
    /// Device name (null-terminated string)
    pub name: [u8; 32],
    /// Memory-mapped I/O base address
    pub mmio_base: usize,
}

/// TX descriptor structure
///
/// Corresponds to C structure: `TxDesc`
/// 
/// Each descriptor is 16 bytes and describes a transmit buffer.
/// The hardware uses these descriptors to manage TX operations.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct TxDesc {
    /// Descriptor status and control flags
    pub status: u32,
    /// VLAN tag
    pub vlan_tag: u32,
    /// Buffer address (lower 32 bits)
    pub buf_addr: u32,
    /// Buffer address (upper 32 bits)
    pub buf_haddr: u32,
}

/// RX descriptor structure
///
/// Corresponds to C structure: `RxDesc`
/// 
/// Each descriptor is 16 bytes and describes a receive buffer.
/// The hardware uses these descriptors to manage RX operations.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RxDesc {
    /// Descriptor status and receive flags
    pub status: u32,
    /// VLAN tag
    pub vlan_tag: u32,
    /// Buffer address (lower 32 bits)
    pub buf_addr: u32,
    /// Buffer address (upper 32 bits)
    pub buf_haddr: u32,
}

/// RTL8169 driver private data structure
///
/// Corresponds to C structure: `rtl8169_private`
/// 
/// This structure contains all the runtime state for the RTL8169 driver,
/// including descriptor arrays, buffer pointers, and current indices.
/// 
/// # Safety
/// 
/// This structure contains raw pointers that must be properly initialized
/// and managed. All pointer fields should be either null or point to valid
/// memory regions.
#[repr(C)]
pub struct Rtl8169Private {
    /// I/O base address for register access
    pub iobase: usize,
    /// Memory-mapped I/O address (currently unused)
    pub mmio_addr: *mut core::ffi::c_void,
    /// Chipset index (into RTL_CHIP_INFO array)
    pub chipset: i32,
    /// Current RX descriptor index
    pub cur_rx: usize,
    /// Current TX descriptor index
    pub cur_tx: usize,
    /// Dirty TX descriptor index (for cleanup)
    pub dirty_tx: usize,
    /// TX descriptor array pointer
    pub tx_desc_array: *mut TxDesc,
    /// RX descriptor array pointer
    pub rx_desc_array: *mut RxDesc,
    /// RX buffer rings pointer (unused in current implementation)
    pub rx_buffer_rings: *mut u8,
    /// RX buffer ring array (one pointer per descriptor)
    pub rx_buffer_ring: [*mut u8; crate::constants::NUM_RX_DESC],
    /// TX socket buffer array (one pointer per descriptor)
    pub tx_skbuff: [*mut u8; crate::constants::NUM_TX_DESC],
}

impl Rtl8169Private {
    /// Create a new RTL8169Private with all fields initialized to zero/null
    /// 
    /// This provides a safe starting point for the driver structure.
    /// All pointers are initialized to null and must be set up properly
    /// before use.
    pub const fn new() -> Self {
        Self {
            iobase: 0,
            mmio_addr: core::ptr::null_mut(),
            chipset: 0,
            cur_rx: 0,
            cur_tx: 0,
            dirty_tx: 0,
            tx_desc_array: core::ptr::null_mut(),
            rx_desc_array: core::ptr::null_mut(),
            rx_buffer_rings: core::ptr::null_mut(),
            rx_buffer_ring: [core::ptr::null_mut(); crate::constants::NUM_RX_DESC],
            tx_skbuff: [core::ptr::null_mut(); crate::constants::NUM_TX_DESC],
        }
    }
}
