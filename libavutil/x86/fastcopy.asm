%include "libavutil/x86/x86util.asm"

SECTION .text

;-------------------------------------------------------------------------------
; void ff_lock_unlock_mem_from_uswc();
;-------------------------------------------------------------------------------
cglobal lock_unlock_mem_from_uswc , 0, 0, 0
    mfence
    RET

;-------------------------------------------------------------------------------
; void ff_copy_mem_from_uswc_sse4(void *dst, void *src, 
;                                 size_t len);
;-------------------------------------------------------------------------------
%if HAVE_SSE4_EXTERNAL
INIT_XMM sse4
cglobal copy_mem_from_uswc, 3, 3, 1, dst, src, len
%assign i 0
%rep 8
    movntdqa m0, [srcq+i*mmsize]
    movdqa[dstq+i*mmsize], m0
%assign i i+1
%endrep
    RET
%endif

;-------------------------------------------------------------------------------
; void ff_copy_mem_from_uswc_avx(void *dst, void *src,
;                                size_t len);
;-------------------------------------------------------------------------------
%if HAVE_AVX_EXTERNAL
INIT_YMM avx
cglobal copy_mem_from_uswc, 3, 3, 1, dst, src, len
%assign i 0
%rep 8
    vmovntdqa m0, [srcq+i*mmsize]
    vmovdqa[dstq+i*mmsize], m0
%assign i i+1
%endrep
    RET
%endif

