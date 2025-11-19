#![no_std]
#![cfg_attr(doc, feature(doc_auto_cfg))]

pub mod constants;
pub mod types;
pub mod rtl8169;

use memory_addr::{PhysAddr, VirtAddr};

#[macro_use]
extern crate log;

/// Kernel function interface that must be implemented by the platform
///
/// This trait provides the necessary kernel-level operations for the driver,
/// including address translation, DMA memory management, timing, and cache operations.
#[crate_interface::def_interface]
pub trait KernelFunc {
    /// Convert virtual address to physical address
    fn virt_to_phys(addr: VirtAddr) -> PhysAddr;

    /// Convert physical address to virtual address
    fn phys_to_virt(addr: PhysAddr) -> VirtAddr;

    /// Busy-wait for the specified duration
    fn busy_wait(duration: core::time::Duration);

    fn dma_alloc_coherent(pages: usize) -> (usize, usize);

    fn dma_free_coherent(vaddr: usize, pages: usize);

    /// Clean (write-back) data cache range
    ///
    /// Ensures CPU-written data is flushed to memory so hardware DMA can see it.
    /// Must be called before hardware reads from a buffer (e.g., before TX).
    ///
    /// # Safety
    /// Must be called with valid memory range.
    fn clean_dcache_range(addr: usize, size: usize);

    /// Invalidate (discard) data cache range
    ///
    /// Forces CPU to read from memory instead of cache on next access.
    /// Must be called before CPU reads hardware-written data (e.g., after RX).
    ///
    /// # Safety
    /// Must be called with valid memory range.
    fn invalidate_dcache_range(addr: usize, size: usize);
}

pub struct UseKernelFunc;

impl KernelFunc for UseKernelFunc {
    #[doc = " Convert virtual address to physical address"]
    fn virt_to_phys(addr: VirtAddr) -> PhysAddr {
        crate_interface::call_interface!(KernelFunc::virt_to_phys(addr))
    }

    #[doc = " Convert physical address to virtual address"]
    fn phys_to_virt(addr: PhysAddr) -> VirtAddr {
        crate_interface::call_interface!(KernelFunc::phys_to_virt(addr))
    }

    #[doc = " Busy-wait for the specified duration"]
    fn busy_wait(duration: core::time::Duration) {
        crate_interface::call_interface!(KernelFunc::busy_wait(duration))
    }

    #[doc = " Clean (write-back) data cache range"]
    #[doc = ""]
    #[doc = " Ensures CPU-written data is flushed to memory so hardware DMA can see it."]
    #[doc = " Must be called before hardware reads from a buffer (e.g., before TX)."]
    #[doc = ""]
    #[doc = " # Safety"]
    #[doc = " Must be called with valid memory range."]
    fn clean_dcache_range(addr: usize, size: usize) {
        crate_interface::call_interface!(KernelFunc::clean_dcache_range(addr, size))
    }

    #[doc = " Invalidate (discard) data cache range"]
    #[doc = ""]
    #[doc = " Forces CPU to read from memory instead of cache on next access."]
    #[doc = " Must be called before CPU reads hardware-written data (e.g., after RX)."]
    #[doc = ""]
    #[doc = " # Safety"]
    #[doc = " Must be called with valid memory range."]
    fn invalidate_dcache_range(addr: usize, size: usize) {
        crate_interface::call_interface!(KernelFunc::invalidate_dcache_range(addr, size))
    }
    
    fn dma_alloc_coherent(pages:usize) -> (usize,usize) {
        crate_interface::call_interface!(KernelFunc::dma_alloc_coherent(pages))
    }
    
    fn dma_free_coherent(vaddr:usize,pages:usize) {
        crate_interface::call_interface!(KernelFunc::dma_free_coherent(vaddr, pages))
    }
}
