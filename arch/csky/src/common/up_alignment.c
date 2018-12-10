/************************************************************************************
 * arch/src/common/up_alignment.c
 *
 * Copyright (C) 2015 The YunOS Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <debug.h>

#ifdef CONFIG_ARCH_BACKTRACE
#include <csky_backtrace.h>
#endif

#define THIS_MODULE MODULE_ARCH


#undef HANDLER_SUCCESS
#undef HANDLER_FAILURE
#define HANDLER_SUCCESS 0
#define HANDLER_FAILURE	1

#undef SP_NUM
#ifndef __CSKYABIV2__
#define SP_NUM 0
#else
#define SP_NUM 14
#endif

#undef R4_NUM
#define R4_NUM 4
#undef R15_NUM
#define R15_NUM 4
#undef R16_NUM
#define R16_NUM 16
#undef R28_NUM
#define R28_NUM 28

#if 1  //def CONFIG_SOFT_HANDMISSALIGN

/* C-SKY CPU V2 32 bit instruction like 11'B in the highest two bit  */
#define IS_T32(hi16)  (((hi16) & 0xc000) == 0xc000 )

#define CODING_BITS(i)  (i & 0xFC000000)
#define LDST_TYPE(i)    (i & 0xf000)

#define UM_WARN     (1 << 0)
#define UM_FIXUP    (1 << 1)
#define UM_SIGNAL   (1 << 2)
#define UM_KFIX     (1 << 3) // kernel alignment fix

static unsigned long ai_half;
static unsigned long ai_word;
static unsigned long ai_qword;

#ifdef  __cskyBE__
#define BE  1
#define FIRST_BYTE_16  "rotri %1, 8\n"
#define FIRST_BYTE_32  "rotri %1, 24\n"
#define NEXT_BYTE  "rotri %1, 24\n"
#else
#define BE  0
#define FIRST_BYTE_16
#define FIRST_BYTE_32
#define NEXT_BYTE  "lsri  %1, 8\n"
#endif

#define __get8_unaligned_check(val,addr,err)  \
    __asm__(  \
    "1: ldb %1, (%2)\n"  \
    "   addi %2, 1\n"  \
    "   br 3f\n"  \
    "2: movi %0, 1\n"  \
    "   br 3f\n"  \
    "3:\n"  \
    : "=r" (err), "=r" (val), "=r" (addr)  \
    : "0" (err), "2" (addr))

#define get16_unaligned_check(val,addr)  \
    do {  \
        unsigned int err = 0, v, a = addr;  \
        __get8_unaligned_check(v,a,err);  \
        val = v << ((BE) ? 8 : 0);  \
        __get8_unaligned_check(v,a,err);  \
        val |= v << ((BE) ? 0 : 8);  \
        if (err)  \
            goto fault;  \
    } while (0)

#define get32_unaligned_check(val,addr)  \
    do {  \
        unsigned int err = 0, v, a = addr;  \
        __get8_unaligned_check(v,a,err);    \
        val =  v << ((BE) ? 24 :  0);       \
        __get8_unaligned_check(v,a,err);    \
        val |= v << ((BE) ? 16 :  8);       \
        __get8_unaligned_check(v,a,err);    \
        val |= v << ((BE) ?  8 : 16);       \
        __get8_unaligned_check(v,a,err);    \
        val |= v << ((BE) ?  0 : 24);       \
        if (err)  \
            goto fault;  \
    } while (0)

#define put16_unaligned_check(val,addr)  \
    do {  \
        unsigned int err = 0, v = val, a = addr;  \
        __asm__( FIRST_BYTE_16  \
        "1: stb %1, (%2)\n"  \
        "   addi %2, 1\n"  \
        NEXT_BYTE  \
        "2: stb %1, (%2)\n"  \
        "   br 4f\n"  \
        "3: movi %0, 1\n"  \
        "   br 4f\n"  \
        "4:\n"  \
        :"=r" (err), "=r" (v), "=r" (a)  \
        :"0" (err), "1" (v), "2" (a));  \
        if (err)  \
            goto fault;  \
    } while (0)

#define put32_unaligned_check(val,addr)  \
    do {  \
        unsigned int err = 0, v = val, a = addr;  \
        __asm__( FIRST_BYTE_32  \
        "1: stb %1, (%2)\n"     \
        "   addi %2, 1\n"       \
        NEXT_BYTE               \
        "2: stb %1, (%2)\n"     \
        "   addi %2, 1\n"       \
        NEXT_BYTE               \
        "3: stb %1, (%2)\n"     \
        "   addi %2, 1\n"       \
        NEXT_BYTE               \
        "4: stb %1, (%2)\n"     \
        "   br 6f\n"            \
        "5: movi %0, 1\n"       \
        "   br 6f\n"            \
        "6:\n"                  \
        : "=r" (err), "=r" (v), "=r" (a)  \
        : "0" (err), "1" (v), "2" (a));  \
        if (err)  \
            goto fault;  \
    } while (0)

extern int check_addr_validity(uint32_t addr);
extern void up_trap_c(uint32_t *regs);

void alignment_check_addr(uint32_t addr, uint32_t *regs)
{
    sinfo("Handle addr: 0x%x\n", addr);
    if (check_addr_validity(addr)) {
        sinfo("Invalid addr, call up_trap_c...\n");
        up_trap_c(regs);
    }
}

inline static unsigned int
get_regs_value(unsigned int rx, uint32_t *regs)
{
    unsigned int value=0;//set initial value to 0

#ifndef __CSKYABIV2__
    value = regs[rx];
#else
    if(rx == 14) {
        /* TODO */
    } else {
        value = regs[rx];
    }
#endif
    return value;
}

inline static int
put_regs_value(unsigned int value, unsigned int rx, uint32_t *regs)
{
#ifndef __CSKYABIV2__
    if(rx == 0) {
        sinfo("alignment handler trying to write sp.\n");
        goto fault;
    } else {
        regs[rx] = value;
    }
#else
    if(rx == 14) {
        sinfo("alignment handler trying to write sp.\n");
        goto fault;
    } else {
        regs[rx] = value;
    }
#endif
    return 0;
fault:
    return 1;
}

#ifndef __CSKYABIV2__
static int
handle_ldh_ldw_v1(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = instr & 0xf;
    unsigned int regz = (instr >> 8) & 0xf;
    unsigned int imm4 = (instr >> 4) & 0xf;
    unsigned int destaddr, ldh_ldw;
    unsigned int dataregx, tmpval32;
    unsigned short tmpval16;

    dataregx = get_regs_value(regx, regs);

    ldh_ldw = instr & 0x6000;
    if(ldh_ldw == 0x4000) { // ldh
        destaddr = dataregx + (imm4 << 1);
        alignment_check_addr(destaddr, regs);
        get16_unaligned_check(tmpval16, destaddr);
        if(put_regs_value((unsigned int)tmpval16, regz, regs) != 0) {
            goto fault;
        }
        ai_half += 1;
    } else if(ldh_ldw == 0x0000) { // ldw
        destaddr = dataregx + (imm4 << 2);
        alignment_check_addr(destaddr, regs);
        get32_unaligned_check(tmpval32, destaddr);
        if(put_regs_value(tmpval32, regz, regs) != 0) {
            goto fault;
        }
        ai_word += 1;
    } else {
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_ldm_v1(unsigned long instr, uint32_t *regs)
{
    unsigned int regf = instr & 0xf;
    unsigned int datasp;
    unsigned int tmpval32, i;

    // regf can not be r0 or r15.
    if(regf == 0 || regf == 15) {
        goto fault;
    }

    datasp = get_regs_value(SP_NUM, regs);
    for(i = regf; i <= R15_NUM; i++) {
        get32_unaligned_check(tmpval32, datasp + (i - regf) * 4);
        if(put_regs_value(tmpval32, i, regs) != 0) {
            goto fault;
        }
    }
    ai_qword += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_ldq_v1(unsigned long instr, uint32_t *regs)
{
    unsigned int regf = instr & 0xf;
    unsigned int datarf;
    unsigned int tmpval32, i;

    // regf can not be r4 - r7.
    if(regf > 3 && regf < 8) {
        goto fault;
    }

    datarf = get_regs_value(regf, regs);
    for(i = 4; i <= 8; i++) {
        get32_unaligned_check(tmpval32, datarf + (i - 4) * 4);
        if(put_regs_value(tmpval32, i, regs) != 0) {
            goto fault;
        }
    }
    ai_qword += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_sth_stw_v1(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = instr & 0xf;
    unsigned int regz = (instr >> 8) & 0xf;
    unsigned int imm4 = (instr >> 4) & 0xf;
    unsigned int destaddr, sth_stw;
    unsigned int dataregx, dataregz;

    dataregx = get_regs_value(regx, regs);
    dataregz = get_regs_value(regz, regs);

    sth_stw = instr & 0x6000;
    if(sth_stw == 0x4000) { // sth
        destaddr = dataregx + (imm4 << 1);
        alignment_check_addr(destaddr, regs);
        put16_unaligned_check(dataregz, destaddr);
        ai_half += 1;
    } else if(sth_stw == 0x0000) { //stw
        destaddr = dataregx + (imm4 << 2);
        alignment_check_addr(destaddr, regs);
        put32_unaligned_check(dataregz, destaddr);
        ai_word += 1;
    } else {
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_stq_v1(unsigned long instr, uint32_t *regs)
{
    unsigned int regf = instr & 0xf;
    unsigned int datarf;
    unsigned int tmpval32, i;

    // regf can not be r4 - r7.
    if(regf > 3 && regf < 8) {
        goto fault;
    }

    datarf = get_regs_value(regf, regs);
    for(i = 4; i <= 7; i++) {
        tmpval32 = get_regs_value(i, regs);
        put32_unaligned_check(tmpval32, datarf + (i - 4) * 4);
    }
    ai_qword += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_stm_v1(unsigned long instr, uint32_t *regs)
{
    unsigned int regf = instr & 0xf;
    unsigned int datasp;
    unsigned int tmpval32, i;

    // regf can not be r0 or r15.
    if(regf == 0 || regf == 15) {
        goto fault;
    }

    datasp = get_regs_value(SP_NUM, regs);
    for(i = regf; i <= R15_NUM; i++) {
        tmpval32 = get_regs_value(i, regs);
        put32_unaligned_check(tmpval32, datasp + (i - regf) * 4);
    }
    ai_qword += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}
#else /* abiv2 */
static int
handle_ldh_16(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 8) & 0x7;
    unsigned int regz = (instr >> 5) & 0x7;
    unsigned int imm5 = instr & 0x1f;
    unsigned int destaddr;
    unsigned int dataregx;
    unsigned short tmpval16;

    dataregx = get_regs_value(regx, regs);
    destaddr = dataregx + (imm5 << 1);
    alignment_check_addr(destaddr, regs);
    get16_unaligned_check(tmpval16, destaddr);
    if(put_regs_value((unsigned int)tmpval16, regz, regs) != 0) {
        goto fault;
    }

    ai_half += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_ldw_16(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 8) & 0x7;
    unsigned int regz = (instr >> 5) & 0x7;
    unsigned int imm5 = instr & 0x1f;
    unsigned int destaddr,tmpval32;
    unsigned int dataregx;

    dataregx = get_regs_value(regx, regs);
    destaddr = dataregx + (imm5 << 2);
    alignment_check_addr(destaddr, regs);
    get32_unaligned_check(tmpval32, destaddr);
    if(put_regs_value(tmpval32, regz, regs) != 0) {
        goto fault;
    }

    ai_word += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_ldw_sp_16(unsigned long instr, uint32_t *regs)
{
    unsigned int regz = (instr >> 5) & 0x7;
    unsigned int imm5 = instr & 0x1f;
    unsigned int imm3 = (instr >> 8) & 0x7;
    unsigned int destaddr,tmpval32;
    unsigned int datasp;

    datasp = get_regs_value(SP_NUM, regs);
    destaddr = datasp + (((imm3 << 5) | imm5) << 2);
    alignment_check_addr(destaddr, regs);
    get32_unaligned_check(tmpval32, destaddr);
    if(put_regs_value(tmpval32, regz, regs) != 0) {
        goto fault;
    }

    ai_word += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_sth_16(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 8) & 0x7;
    unsigned int regz = (instr >> 5) & 0x7;
    unsigned int imm5 = instr & 0x1f;
    unsigned int destaddr;
    unsigned int dataregx,dataregz;

    dataregx = get_regs_value(regx, regs);
    destaddr = dataregx + (imm5 << 1);
    alignment_check_addr(destaddr, regs);
    dataregz = get_regs_value(regz, regs);
    put16_unaligned_check(dataregz, destaddr);

    ai_half += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_stw_16(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 8) & 0x7;
    unsigned int regz = (instr >> 5) & 0x7;
    unsigned int imm5 = instr & 0x1f;
    unsigned int destaddr;
    unsigned int dataregx,dataregz;

    dataregx = get_regs_value(regx, regs);
    destaddr = dataregx + (imm5 << 2);
    alignment_check_addr(destaddr, regs);
    dataregz = get_regs_value(regz, regs);
    put32_unaligned_check(dataregz, destaddr);

    ai_word += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_stw_sp_16(unsigned long instr, uint32_t *regs)
{
    unsigned int regz = (instr >> 5) & 0x7;
    unsigned int imm5 = instr & 0x1f;
    unsigned int imm3 = (instr >> 8) & 0x7;
    unsigned int destaddr;
    unsigned int datasp,dataregz;

    datasp = get_regs_value(SP_NUM, regs);
    destaddr = datasp + (((imm3 << 5) | imm5) << 2);
    alignment_check_addr(destaddr, regs);
    dataregz = get_regs_value(regz, regs);
    put32_unaligned_check(dataregz, destaddr);

    ai_word += 1;

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_push_pop_16(unsigned long instr, uint32_t *regs)
{
    unsigned int push_pop = instr & 0xffe0;
    unsigned int list1 = instr & 0xf;
    unsigned int has_r15 = (instr & 0x0010) >> 4;
    unsigned int datasp, tmpval32, i;

    datasp = get_regs_value(SP_NUM, regs);
    alignment_check_addr(datasp, regs);

    if(push_pop == 0x1480) { // pop
        for(i = 0; i < list1; i++) {
            get32_unaligned_check(tmpval32, datasp + i * 4);
            if(put_regs_value(tmpval32, R4_NUM + i, regs) != 0) {
                goto fault;
            }
        }
        if(has_r15) {
            get32_unaligned_check(tmpval32, datasp + list1 * 4);
            if(put_regs_value(tmpval32, R15_NUM, regs) != 0) {
                goto fault;
            }
        }
        datasp += (list1 + has_r15) * 4;
        if(put_regs_value(datasp, SP_NUM, regs) != 0) {
            goto fault;
        }
    } else if(push_pop == 0x14c0) { // push
        datasp -= (list1 + has_r15) * 4;
        if(put_regs_value(datasp, SP_NUM, regs) != 0) {
            goto fault;
        }
        for(i = 0; i < list1; i++) {
            tmpval32 = get_regs_value(R4_NUM + i, regs);
            put32_unaligned_check(tmpval32, datasp + i * 4);
        }
        if(has_r15) {
            tmpval32 = get_regs_value(R15_NUM, regs);
            put32_unaligned_check(tmpval32, datasp + list1 * 4);
        }
    } else {
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_push_pop(unsigned long instr, uint32_t *regs)
{
    unsigned int push_pop = instr & 0xfffffe00;
    unsigned int list1 = instr & 0xf;
    unsigned int list2 = (instr >> 5) & 0x7;
    unsigned int has_r15 = (instr >> 4) & 0x1;
    unsigned int has_r28 = (instr >> 8) & 0x1;
    unsigned int datasp, tmpval32, i;

    datasp = get_regs_value(SP_NUM, regs);
    alignment_check_addr(datasp, regs);

    if(push_pop == 0x1480) { // pop
        for(i = 0; i < list1; i++) {
            get32_unaligned_check(tmpval32, datasp + i * 4);
            if(put_regs_value(tmpval32, R4_NUM + i, regs) != 0) {
                goto fault;
            }
        }
        if(has_r15 == 0x1) {
            get32_unaligned_check(tmpval32, datasp + list1 * 4);
            if(put_regs_value(tmpval32, R15_NUM, regs) != 0) {
                goto fault;
            }
        }
        for(i = 0; i < list2; i++) {
            get32_unaligned_check(tmpval32, datasp + (i + list1 + has_r15) * 4);
            if(put_regs_value(tmpval32, R16_NUM + i, regs) != 0) {
                goto fault;
            }
        }
        if(has_r28 == 0x1) {
            get32_unaligned_check(tmpval32, datasp + (list1 + list2 + has_r15) * 4);
            if(put_regs_value(tmpval32, R28_NUM, regs) != 0) {
                goto fault;
            }
        }
        datasp += (list1 + list2 + has_r15 + has_r28) * 4;
        if(put_regs_value(datasp, SP_NUM, regs) != 0) {
            goto fault;
        }
    } else if(push_pop == 0x14c0) { // push
        datasp -= (list1 + list2 + has_r15 + has_r28) * 4;
        if(put_regs_value(datasp, SP_NUM, regs) != 0) {
            goto fault;
        }
        for(i = 0; i < list1; i++) {
            tmpval32 = get_regs_value(R4_NUM + i, regs);
            put32_unaligned_check(tmpval32, datasp + i * 4);
        }
        if(has_r15 == 0x1) {
            tmpval32 = get_regs_value(R15_NUM, regs);
            put32_unaligned_check(tmpval32, datasp + list1 * 4);
        }
        for(i = 0; i < list2; i++) {
            tmpval32 = get_regs_value(R16_NUM + i, regs);
            put32_unaligned_check(tmpval32, datasp + (i + list1 + has_r15) * 4);
        }
        if(has_r28 == 0x1) {
            tmpval32 = get_regs_value(R28_NUM, regs);
            put32_unaligned_check(tmpval32, datasp + (list1 + list2 + has_r15) * 4);
        }
    } else {
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_ldh_ldhs_ldw_ldd(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 16) & 0x1f;
    unsigned int regz = (instr >> 21) & 0x1f;
    unsigned int offset = instr & 0xfff;
    unsigned int destaddr, tmpval32;
    unsigned int dataregx;
    unsigned short tmpval16;

    dataregx = get_regs_value(regx, regs);

    switch((instr >> 12) & 0xf) {
    case 1: // ldh
        destaddr = dataregx + (offset << 1);
        alignment_check_addr(destaddr, regs);
        get16_unaligned_check(tmpval16, destaddr);
        if(put_regs_value((unsigned int)tmpval16, regz, regs) != 0) {
            goto fault;
        }
        ai_half += 1;
        break;
    case 3: // ldd
        destaddr = dataregx + (offset << 2);
        alignment_check_addr(destaddr, regs);
        get32_unaligned_check(tmpval32, destaddr);
        if(put_regs_value(tmpval32, regz, regs) != 0) {
            goto fault;
        }
        get32_unaligned_check(tmpval32, destaddr + 4);
        if(put_regs_value(tmpval32, regz + 1, regs) != 0) {
            goto fault;
        }
        ai_word += 2;
        break;
    case 5: // ldhs
        destaddr = dataregx + (offset << 1);
        alignment_check_addr(destaddr, regs);
        get16_unaligned_check(tmpval16, destaddr);
        if(put_regs_value((unsigned int)((short)tmpval16), regz, regs) != 0) {
            goto fault;
        }
        ai_half += 1;
        break;
    case 2: // ldw
        destaddr = dataregx + (offset << 2);
        alignment_check_addr(destaddr, regs);
        get32_unaligned_check(tmpval32, destaddr);
        if(put_regs_value(tmpval32, regz, regs) != 0) {
            goto fault;
        }
        ai_word += 1;
        break;
    default:
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_sth_stw_std(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 16) & 0x1f;
    unsigned int regz = (instr >> 21) & 0x1f;
    unsigned int offset = instr & 0xfff;
    unsigned int destaddr;
    unsigned int dataregx, dataregz;

    dataregx = get_regs_value(regx, regs);

    switch((instr >> 12) & 0xf) {
    case 1: // sth
        destaddr = dataregx + (offset << 1);
        alignment_check_addr(destaddr, regs);
        dataregz = get_regs_value(regz, regs);
        put16_unaligned_check(dataregz, destaddr);
        ai_half += 1;
        break;
    case 3: // std
        destaddr = dataregx + (offset << 2);
        alignment_check_addr(destaddr, regs);
        dataregz = get_regs_value(regz, regs);
        put32_unaligned_check(dataregz, destaddr);
        dataregz = get_regs_value(regz + 1, regs);
        put32_unaligned_check(dataregz, destaddr + 4);
        ai_word += 2;
        break;
    case 2: // stw
        destaddr = dataregx + (offset << 2);
        alignment_check_addr(destaddr, regs);
        dataregz = get_regs_value(regz, regs);
        put32_unaligned_check(dataregz, destaddr);
        ai_word += 1;
        break;
    default:
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_ldrh_ldrhs_ldrw_ldm(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 16) & 0x1f;
    unsigned int regy = (instr >> 21) & 0x1f;
    unsigned int regz = instr & 0x1f;
    unsigned int dataregx, dataregy;
    unsigned int destaddr, tmpval32, i;
    unsigned short tmpval16;

    dataregx = get_regs_value(regx, regs);
    dataregy = get_regs_value(regy, regs);

    destaddr = dataregx + dataregy * ((instr >> 5) & 0x1f);
    alignment_check_addr(destaddr, regs);

    switch((instr >> 10) & 0x3f) {
    case 1: // ldrh
        get16_unaligned_check(tmpval16, destaddr);
        if(put_regs_value((unsigned int)tmpval16, regz, regs) != 0) {
            goto fault;
        }
        ai_half += 1;
        break;
    case 5: // ldrhs
        get16_unaligned_check(tmpval16, destaddr);
        if(put_regs_value((unsigned int)((short)tmpval16), regz, regs) != 0) {
            goto fault;
        }
        ai_half += 1;
        break;
    case 2: // ldrw
        get32_unaligned_check(tmpval32, destaddr);
        if(put_regs_value(tmpval32, regz, regs) != 0) {
            goto fault;
        }
        ai_word += 1;
        break;
    case 7: // ldm
        for(i = 0; i <= (instr & 0x1f); i++) {
            get32_unaligned_check(tmpval32, dataregx + i * 4);
            if(put_regs_value(tmpval32, regy + i, regs) != 0) {
                goto fault;
            }
        }
        ai_qword += 1;
        return HANDLER_SUCCESS;
    default:
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}

static int
handle_strh_strw_stm(unsigned long instr, uint32_t *regs)
{
    unsigned int regx = (instr >> 16) & 0x1f;
    unsigned int regy = (instr >> 21) & 0x1f;
    unsigned int regz = instr & 0x1f;
    unsigned int dataregx, dataregy, dataregz;
    unsigned int destaddr, tmpval32, i;

    dataregx = get_regs_value(regx, regs);
    dataregy = get_regs_value(regy, regs);

    destaddr = dataregx + dataregy * ((instr >> 5) & 0x1f);
    alignment_check_addr(destaddr, regs);

    switch((instr >> 10) & 0x3f) {
    case 1: // strh
        dataregz = get_regs_value(regz, regs);
        put16_unaligned_check(dataregz, destaddr);
        ai_half += 1;
        break;
    case 2: // strw
        dataregz = get_regs_value(regz, regs);
        put32_unaligned_check(dataregz, destaddr);
        ai_word += 1;
        break;
    case 7: //  stm
        for(i = 0; i <= (instr & 0x1f); i++) {
            tmpval32 = get_regs_value(regy + i, regs);
            put32_unaligned_check(tmpval32, dataregx + 4 * i);
        }
        ai_qword += 1;
    default:
        goto fault;
    }

    return HANDLER_SUCCESS;
fault:
    return HANDLER_FAILURE;
}
#endif /* __CSKYABIV2__ */

uint32_t instruction_pointer(uint32_t *regs)
{
    return regs[REG_PC];
}

void alignment_c(uint32_t *regs)
{
    int err;
    unsigned long instr = 0, instrptr;
    uint16_t tinstr = 0;
    int (*handler)(unsigned long inst, uint32_t *regs) = NULL;
    int isize = 2;

    instrptr = instruction_pointer(regs);

    tinstr = *(uint16_t *)(instrptr & ~1);
    instr = (unsigned long)tinstr;

    syslog(LOG_EMERG, "Alignment --> epc: %x, insn: %x\n", instrptr, tinstr);
#ifdef CONFIG_ARCH_BACKTRACE
    dump_backtrace(regs);
#endif

#ifdef __CSKYABIV2__
    if (IS_T32(tinstr)) {
        uint16_t tinst2 = 0;
        tinst2 = *(uint16_t *)(instrptr+2);
        instr = (tinstr << 16) | tinst2;
        isize = 4;
    }
#endif


#ifndef __CSKYABIV2__  /* abiv1 */
    if((instr & 0x9000) == 0x9000) { // sth, stw
        handler = handle_sth_stw_v1;
    } else if((instr & 0x9000) == 0x8000) { // ldh, ldw
        handler = handle_ldh_ldw_v1;
    } else if((instr & 0xfff0) == 0x0070) { // stm
        handler = handle_stm_v1;
    } else if((instr & 0xfff0) == 0x0060) { // ldm
        handler = handle_ldm_v1;
    } else if((instr & 0xfff0) == 0x0050) { // stq
        handler = handle_stq_v1;
    } else if((instr & 0xfff0) == 0x0040) { // ldq
        handler = handle_ldq_v1;
    } else {
        goto bad_or_fault;
    }
#else  /* abiv2 */
    if(2 == isize ) {
        switch(instr & 0xf800) {
        case 0x8800: // ldh
            handler = handle_ldh_16;
            break;
        case 0x9800: // ldw sp
            handler = handle_ldw_sp_16;
            break;
        case 0x9000: // ldw
            handler = handle_ldw_16;
            break;
        case 0xa800: // ld
            handler = handle_sth_16;
            break;
        case 0xb000:
            handler = handle_stw_16;
            break;
        case 0xb800:
            handler = handle_stw_sp_16;
            break;
        case 0x0100:
            handler = handle_push_pop_16;
            sinfo("warnning: push/pop alignment.\n");
            break;
        default:
            goto bad_or_fault;
        }
    } else {
        switch (CODING_BITS(instr)) {
        case 0xD8000000: // ld.h/ld.hs/ld.w/ld.d
            handler = handle_ldh_ldhs_ldw_ldd;
            break;
        case 0xDC000000: // st.h/st.w/st.d
            handler = handle_sth_stw_std;
            break;
        case 0xD0000000: // ldr.w/ldr.h/ldr.hs/ldm
            handler = handle_ldrh_ldrhs_ldrw_ldm;
            break;
        case 0xD4000000: // stm/str.h/str.w
            handler = handle_strh_strw_stm;
            break;
        case 0xE8000000:  // push/pop instruction.
            sinfo("warnning: push/pop alignment.\n");
            handler = handle_push_pop;
            break;
        default:
            // FIXME: stq/stq is pseudo instruction of stm/stm and now ignore.
            goto bad_or_fault;
        }
    }
#endif

    if (!handler)
        goto bad_or_fault;

    err = handler(instr, regs);

    if (err != HANDLER_SUCCESS) {
        goto bad_or_fault;
    }

    regs[REG_PC] += isize;

    return;

bad_or_fault:
    sinfo("Alignment trap: not handle this instruction\n");
    return;
}

#else /* !CONFIG_SOFT_HANDMISSALIGN */

void alignment_c(uint32_t *regs)
{
    show_registers(regs);
}

#endif /* CONFIG_SOFT_HANDMISSALIGN */
