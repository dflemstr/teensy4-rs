#[doc = "Reader of register CAPT%s"]
pub type R = crate::R<u16, super::CAPT>;
#[doc = "Writer for register CAPT%s"]
pub type W = crate::W<u16, super::CAPT>;
#[doc = "Register CAPT%s `reset()`'s with value 0"]
impl crate::ResetValue for super::CAPT {
    type Type = u16;
    #[inline(always)]
    fn reset_value() -> Self::Type {
        0
    }
}
#[doc = "Reader of field `CAPTURE`"]
pub type CAPTURE_R = crate::R<u16, u16>;
#[doc = "Write proxy for field `CAPTURE`"]
pub struct CAPTURE_W<'a> {
    w: &'a mut W,
}
impl<'a> CAPTURE_W<'a> {
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub unsafe fn bits(self, value: u16) -> &'a mut W {
        self.w.bits = (self.w.bits & !0xffff) | ((value as u16) & 0xffff);
        self.w
    }
}
impl R {
    #[doc = "Bits 0:15 - Capture Value"]
    #[inline(always)]
    pub fn capture(&self) -> CAPTURE_R {
        CAPTURE_R::new((self.bits & 0xffff) as u16)
    }
}
impl W {
    #[doc = "Bits 0:15 - Capture Value"]
    #[inline(always)]
    pub fn capture(&mut self) -> CAPTURE_W {
        CAPTURE_W { w: self }
    }
}
