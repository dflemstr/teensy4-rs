#[doc = "Reader of register SMCVAL3CYC"]
pub type R = crate::R<u16, super::SMCVAL3CYC>;
#[doc = "Reader of field `CVAL3CYC`"]
pub type CVAL3CYC_R = crate::R<u8, u8>;
impl R {
    #[doc = "Bits 0:3 - CVAL3CYC"]
    #[inline(always)]
    pub fn cval3cyc(&self) -> CVAL3CYC_R {
        CVAL3CYC_R::new((self.bits & 0x0f) as u8)
    }
}
