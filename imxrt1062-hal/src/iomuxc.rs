//! IOMUX Controller

#![allow(non_camel_case_types)]

#[macro_use]
mod macros;

pub mod daisy;
pub mod gpio;
pub mod i2c;
pub mod pwm;
pub mod spi;
pub mod uart;

// IOMUXC section of docs originally state that there are up to 8
// alternative modes. However, some have up to 10 (like GPIO_AD_B1_00)

/// Pad alternative 0 (type tag)
pub struct Alt0;
/// Pad alternative 1 (type tag)
pub struct Alt1;
/// Pad alternative 2 (type tag)
pub struct Alt2;
/// Pad alternative 3 (type tag)
pub struct Alt3;
/// Pad alternative 4 (type tag)
pub struct Alt4;
/// Pad alternative 5 (type tag)
pub struct Alt5;
/// Pad alternative 6 (type tag)
pub struct Alt6;
/// Pad alternative 7 (type tag)
pub struct Alt7;
/// Pad alternative 8 (type tag)
pub struct Alt8;
/// Pad alternative 9 (type tag)
pub struct Alt9;

pub struct IOMUXC {
    //
    // GPIO_B0
    //
    pub gpio_b0_00: gpio::GPIO_B0_00<Alt5>,
    pub gpio_b0_01: gpio::GPIO_B0_01<Alt5>,
    pub gpio_b0_02: gpio::GPIO_B0_02<Alt5>,
    pub gpio_b0_03: gpio::GPIO_B0_03<Alt5>,
    pub gpio_b0_10: gpio::GPIO_B0_10<Alt5>,
    pub gpio_b0_11: gpio::GPIO_B0_11<Alt5>,
    //
    // GPIO_B1
    //
    pub gpio_b1_00: gpio::GPIO_B1_00<Alt5>,
    pub gpio_b1_01: gpio::GPIO_B1_01<Alt5>,
    //
    // GPIO_AD_B0
    //
    pub gpio_ad_b0_00: gpio::GPIO_AD_B0_00<Alt5>,
    pub gpio_ad_b0_01: gpio::GPIO_AD_B0_01<Alt5>,
    pub gpio_ad_b0_02: gpio::GPIO_AD_B0_02<Alt5>,
    pub gpio_ad_b0_03: gpio::GPIO_AD_B0_03<Alt5>,
    pub gpio_ad_b0_12: gpio::GPIO_AD_B0_12<Alt5>,
    pub gpio_ad_b0_13: gpio::GPIO_AD_B0_13<Alt5>,
    //
    // GPIO_AD_B1
    //
    pub gpio_ad_b1_00: gpio::GPIO_AD_B1_00<Alt5>,
    pub gpio_ad_b1_01: gpio::GPIO_AD_B1_01<Alt5>,
    pub gpio_ad_b1_02: gpio::GPIO_AD_B1_02<Alt5>,
    pub gpio_ad_b1_03: gpio::GPIO_AD_B1_03<Alt5>,
    pub gpio_ad_b1_06: gpio::GPIO_AD_B1_06<Alt5>,
    pub gpio_ad_b1_07: gpio::GPIO_AD_B1_07<Alt5>,
    pub gpio_ad_b1_08: gpio::GPIO_AD_B1_08<Alt5>,
    pub gpio_ad_b1_09: gpio::GPIO_AD_B1_09<Alt5>,
    pub gpio_ad_b1_10: gpio::GPIO_AD_B1_10<Alt5>,
    pub gpio_ad_b1_11: gpio::GPIO_AD_B1_11<Alt5>,
    //
    // GPIO_SD_B0
    //
    pub gpio_sd_b0_00: gpio::GPIO_SD_B0_00<Alt5>,
    pub gpio_sd_b0_01: gpio::GPIO_SD_B0_01<Alt5>,
    //
    // GPIO_EMC
    //
    pub gpio_emc_04: gpio::GPIO_EMC_04<Alt5>,
    pub gpio_emc_05: gpio::GPIO_EMC_05<Alt5>,
    pub gpio_emc_06: gpio::GPIO_EMC_06<Alt5>,
    pub gpio_emc_07: gpio::GPIO_EMC_07<Alt5>,
    pub gpio_emc_08: gpio::GPIO_EMC_08<Alt5>,
    pub gpio_emc_31: gpio::GPIO_EMC_31<Alt5>,
    pub gpio_emc_32: gpio::GPIO_EMC_32<Alt5>,
    //
    // GPRs
    //
    pub gpr: GPR,
}

impl IOMUXC {
    pub(crate) fn new(iomuxc: crate::pac::IOMUXC) -> Self {
        Self {
            //
            // GPIO_B0
            //
            gpio_b0_00: gpio::GPIO_B0_00::new(&iomuxc),
            gpio_b0_01: gpio::GPIO_B0_01::new(&iomuxc),
            gpio_b0_02: gpio::GPIO_B0_02::new(&iomuxc),
            gpio_b0_03: gpio::GPIO_B0_03::new(&iomuxc),
            gpio_b0_10: gpio::GPIO_B0_10::new(&iomuxc),
            gpio_b0_11: gpio::GPIO_B0_11::new(&iomuxc),
            //
            // GPIO_B1
            //
            gpio_b1_00: gpio::GPIO_B1_00::new(&iomuxc),
            gpio_b1_01: gpio::GPIO_B1_01::new(&iomuxc),
            //
            // GPIO_AD_B0
            //
            gpio_ad_b0_00: gpio::GPIO_AD_B0_00::new(&iomuxc),
            gpio_ad_b0_01: gpio::GPIO_AD_B0_01::new(&iomuxc),
            gpio_ad_b0_02: gpio::GPIO_AD_B0_02::new(&iomuxc),
            gpio_ad_b0_03: gpio::GPIO_AD_B0_03::new(&iomuxc),
            gpio_ad_b0_12: gpio::GPIO_AD_B0_12::new(&iomuxc),
            gpio_ad_b0_13: gpio::GPIO_AD_B0_13::new(&iomuxc),
            //
            // GPIO_AD_B1
            //
            gpio_ad_b1_00: gpio::GPIO_AD_B1_00::new(&iomuxc),
            gpio_ad_b1_01: gpio::GPIO_AD_B1_01::new(&iomuxc),
            gpio_ad_b1_02: gpio::GPIO_AD_B1_02::new(&iomuxc),
            gpio_ad_b1_03: gpio::GPIO_AD_B1_03::new(&iomuxc),
            gpio_ad_b1_06: gpio::GPIO_AD_B1_06::new(&iomuxc),
            gpio_ad_b1_07: gpio::GPIO_AD_B1_07::new(&iomuxc),
            gpio_ad_b1_08: gpio::GPIO_AD_B1_08::new(&iomuxc),
            gpio_ad_b1_09: gpio::GPIO_AD_B1_09::new(&iomuxc),
            gpio_ad_b1_10: gpio::GPIO_AD_B1_10::new(&iomuxc),
            gpio_ad_b1_11: gpio::GPIO_AD_B1_11::new(&iomuxc),
            //
            // GPIO_SD_B0
            //
            gpio_sd_b0_00: gpio::GPIO_SD_B0_00::new(&iomuxc),
            gpio_sd_b0_01: gpio::GPIO_SD_B0_01::new(&iomuxc),
            //
            // GPIO_EMC
            //
            gpio_emc_04: gpio::GPIO_EMC_04::new(&iomuxc),
            gpio_emc_05: gpio::GPIO_EMC_05::new(&iomuxc),
            gpio_emc_06: gpio::GPIO_EMC_06::new(&iomuxc),
            gpio_emc_07: gpio::GPIO_EMC_07::new(&iomuxc),
            gpio_emc_08: gpio::GPIO_EMC_08::new(&iomuxc),
            gpio_emc_31: gpio::GPIO_EMC_31::new(&iomuxc),
            gpio_emc_32: gpio::GPIO_EMC_32::new(&iomuxc),

            // GPRs
            gpr: GPR(()),
        }
    }
}

pub struct GPR(());

impl GPR {
    pub(crate) fn gpr26(&mut self) -> &crate::pac::iomuxc_gpr::GPR26 {
        unsafe { &(*crate::pac::IOMUXC_GPR::ptr()).gpr26 }
    }

    pub(crate) fn gpr27(&mut self) -> &crate::pac::iomuxc_gpr::GPR27 {
        unsafe { &(*crate::pac::IOMUXC_GPR::ptr()).gpr27 }
    }
}
