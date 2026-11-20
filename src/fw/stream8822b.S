# SPDX-License-Identifier: GPL-2.0-only
# Bank-scheduled LA streamer for the 8822B embedded CPU (MIPS16, little-endian), linked at 0x80000230.
#
# The LA ring (TXFF 0x20000..0x40000, 8 B/sample) is two 64 KiB banks. The CPU arms the ring, then
# for each bank, once the chip clock says the ADC has finished it (and is filling the other bank),
# ships it as four C2H data packets plus one trailer, all out bulk-IN EP 0x84 in order:
#
#   bank bytes [0, 128)        saved to the trailer before descriptor 0 overwrites [104, 128)
#   chunk k (k = 0..3)         payload [128 + k*16352, +16352), its 24-byte RX descriptor written
#                              in place just before it (over the already-shipped tail of chunk k-1)
#   trailer                    RX SRAM 0x1000: descriptor (host-staged) + 128 B prefix + stamps
#   sentinel                   after shipping, the bank's last sample is overwritten with
#                              0xA5A5A5A5_5E47F1E1; the ADC must have replaced it by the next
#                              revolution, otherwise the host knows the bank tail was shipped early
#
# Timing is open-loop on REG 0x568 (us) in fifths of a microsecond, so bank periods are exact
# integers at every rate: period5 = 512 << rate. measured: the ring starts at TXFF 0x20000 within
# ~1 us of the arm write and the ADC clock does not drift against REG 0x568 (latime, 5 s).
# The host halts the CPU to stop; the only host register traffic while streaming is a gain change
# (stream88xx.cpp, live gain: one EP0 transfer per bank).
#
# Gapped mode (+0x24 = 1; for rates the link cannot carry continuously, e.g. on USB 2): every round
# arms the LA (the writer restarts at bank 0), waits one bank time + guard, stops the LA, and ships
# bank 0 while the writer is stopped. Banks are then complete but not contiguous.
#
# DMEM block at 0x80201000 (host-seeded fields marked *):
#   +0x00 magic 0x57535431 ("WST1")  +0x04 seq (banks shipped)   +0x08 state (1 = running)
#   +0x0C target5 (current bank)     +0x10 t_arm (us)           *+0x14 arm LA (0 = synthetic)
#  *+0x18 period5                   *+0x1C guard5               *+0x20 data descriptor word 0
#  *+0x24 gapped (1) / continuous (0)
#  *+0x100, +0x140 bank 0 / 1 records: +0 bank CPU base, +4 + 8k {descriptor CPU addr, request}
# Trailer payload (RX SRAM, CPU 0xb8701018): 128 B prefix, then seq, target5, start5, end5.
        .set    noreorder
        .set    mips16

        .macro  LI32 reg, val
        lui     \reg, (((\val) + 0x8000) >> 16) & 0xffff
        addiu   \reg, ((\val) & 0xffff) - (((\val) & 0x8000) << 1)
        .endm

        # v0 = 5 * REG 0x568 (clobbers v1)
        .macro  NOW5
        lw      $3, 0x568($17)
        sll     $2, $3, 2
        addu    $2, $2, $3
        .endm

        # Write the C2H request in v0 and wait for the engine to take the packet (clobbers v0, v1).
        .macro  SEND
        sw      $2, 0x294($17)
99:     lw      $2, 0x294($17)
        lui     $3, 1
        and     $2, $3
        bnez    $2, 99b
        .endm

        .text
        .globl  _start
_start:
        LI32    $16, 0x80201000         # s0 = DMEM state block
        lui     $17, 0xb864             # s1 = MAC registers (uncached)
        LI32    $7, 0x80201100          # a3 = bank 0 record
        LI32    $2, 0x57535431
        sw      $2, 0($16)
        li      $2, 0
        sw      $2, 4($16)              # seq = 0
        li      $2, 1
        sw      $2, 8($16)              # running
        lw      $2, 0x14($16)
        beqz    $2, 1f                  # synthetic: banks are host-filled, leave the LA stopped
        li      $3, 0xcb
        sb      $3, 0x7c0($17)          # arm: the ADC starts bank 0 now
1:      lw      $3, 0x568($17)
        sw      $3, 0x10($16)           # t_arm
        sll     $2, $3, 2
        addu    $2, $2, $3
        lw      $4, 0x18($16)
        addu    $2, $2, $4
        lw      $4, 0x1c($16)
        addu    $2, $2, $4
        sw      $2, 0x0c($16)           # target5 = 5 t_arm + period5 + guard5

bank:   lw      $3, 0x24($16)
        beqz    $3, 5f
        li      $3, 0xcb
        sb      $3, 0x7c0($17)          # gapped: arm, the writer restarts at bank 0
        NOW5
        lw      $4, 0x18($16)
        addu    $2, $2, $4
        lw      $4, 0x1c($16)
        addu    $2, $2, $4
        sw      $2, 0x0c($16)           # target5 = now5 + period5 + guard5
5:      lw      $5, 0x0c($16)           # a1 = target5
2:      NOW5
        subu    $2, $2, $5
        slti    $2, 0                   # T = now5 < target5
        btnez   2b
        addu    $4, $2, $5              # a0 = start5
        lw      $3, 0x24($16)
        beqz    $3, 6f
        li      $3, 0
        sb      $3, 0x7c0($17)          # gapped: stop the LA (bank 0 stays as captured)
6:
        LI32    $6, 0xb8701098          # a2 = trailer stamps
        lw      $2, 4($16)
        sw      $2, 0($6)               # seq
        sw      $5, 4($6)               # target5
        sw      $4, 8($6)               # start5

        lw      $4, 0($7)               # a0 = bank CPU base
        LI32    $6, 0xb8701018          # a2 = trailer prefix
        li      $5, 32
3:      lw      $2, 0($4)
        sw      $2, 0($6)
        addiu   $4, 4
        addiu   $6, 4
        addiu   $5, -1
        bnez    $5, 3b

        move    $4, $7
        addiu   $4, 4                   # a0 = &record.chunk[0]
        li      $5, 4
4:      lw      $6, 0($4)               # a2 = descriptor CPU address
        lw      $2, 0x20($16)
        sw      $2, 0($6)               # length | OWN
        li      $2, 0
        sw      $2, 4($6)
        sw      $2, 12($6)
        sw      $2, 16($6)
        sw      $2, 20($6)
        lui     $2, 0x1000
        sw      $2, 8($6)               # byte 11 = 0x10: C2H
        lw      $2, 4($4)
        SEND
        addiu   $4, 8
        addiu   $5, -1
        bnez    $5, 4b

        NOW5
        LI32    $6, 0xb8701098
        sw      $2, 12($6)              # end5
        lw      $6, 0($7)
        LI32    $2, 0xfff8
        addu    $6, $6, $2              # a2 = last sample of the bank
        LI32    $2, 0x5e47f1e1
        sw      $2, 0($6)
        LI32    $2, 0xa5a5a5a5
        sw      $2, 4($6)
        LI32    $2, 0x00011000          # trailer: RX SRAM 0x1000
        SEND

        lw      $2, 4($16)
        addiu   $2, 1
        sw      $2, 4($16)              # seq++
        lw      $2, 0x0c($16)
        lw      $3, 0x18($16)
        addu    $2, $2, $3
        sw      $2, 0x0c($16)           # target5 += period5
        lw      $2, 0x24($16)
        bnez    $2, bank                # gapped: always bank 0
        li      $2, 0x40
        xor     $7, $2                  # other bank record
        b       bank
        nop
