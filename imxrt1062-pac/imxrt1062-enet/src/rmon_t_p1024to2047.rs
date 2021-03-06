#[doc = "Reader of register RMON_T_P1024TO2047"]
pub type R = crate::R<u32, super::RMON_T_P1024TO2047>;
#[doc = "Reader of field `TXPKTS`"]
pub type TXPKTS_R = crate::R<u16, u16>;
impl R {
    #[doc = "Bits 0:15 - Number of 1024- to 2047-byte transmit packets"]
    #[inline(always)]
    pub fn txpkts(&self) -> TXPKTS_R {
        TXPKTS_R::new((self.bits & 0xffff) as u16)
    }
}
