#include <arpa/inet.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "vpi_user.h"

typedef struct HotshiftFileCpuState {
    uint32_t magic_be;
    uint32_t version_be;
    uint64_t gpr[32];
    uint64_t pc;
    uint64_t priv;
    uint64_t mstatus;
    uint64_t medeleg;
    uint64_t mideleg;
    uint64_t mie;
    uint64_t mtvec;
    uint64_t mscratch;
    uint64_t mepc;
    uint64_t mcause;
    uint64_t mtval;
    uint64_t mip;
    uint64_t mxstatus;
    uint64_t sstatus;
    uint64_t sie;
    uint64_t stvec;
    uint64_t sscratch;
    uint64_t sepc;
    uint64_t scause;
    uint64_t stval;
    uint64_t sip;
    uint64_t satp;
    uint64_t sxstatus;
    uint32_t clint_valid;
    uint32_t clint_reserved;
    uint64_t clint_mtime;
    uint64_t clint_mtimecmp;
    uint64_t clint_stimecmp;
    uint32_t clint_msip;
    uint32_t clint_ssip;
    uint32_t plic_valid;
    uint32_t plic_num_sources;
    uint32_t plic_bitfield_words;
    uint32_t plic_context_mode;
    uint32_t plic_threshold;
    uint32_t plic_context_claim_id;
    uint32_t plic_pending[8];
    uint32_t plic_enable[8];
    uint32_t plic_claimed[8];
    uint32_t plic_priority[256];
} HotshiftFileCpuState;

#define HOTSHIFT_FILE_MAGIC 0x48535650u
#define HOTSHIFT_FILE_VERSION 4u

static void set_u64_by_name(const char *name, uint64_t value)
{
    vpiHandle handle;
    s_vpi_value val;
    char hexbuf[32];

    handle = vpi_handle_by_name((PLI_BYTE8 *)name, NULL);
    if (handle == NULL) {
        vpi_printf("[hotshift_vpi] missing handle %s\n", name);
        return;
    }

    snprintf(hexbuf, sizeof(hexbuf), "%016llx",
             (unsigned long long)value);
    val.format = vpiHexStrVal;
    val.value.str = hexbuf;
    vpi_put_value(handle, &val, NULL, vpiNoDelay);
}

static void set_u32_by_name(const char *name, uint32_t value)
{
    vpiHandle handle;
    s_vpi_value val;

    handle = vpi_handle_by_name((PLI_BYTE8 *)name, NULL);
    if (handle == NULL) {
        vpi_printf("[hotshift_vpi] missing handle %s\n", name);
        return;
    }

    val.format = vpiIntVal;
    val.value.integer = (PLI_INT32)value;
    vpi_put_value(handle, &val, NULL, vpiNoDelay);
}

static void set_bit_by_name(const char *name, int value)
{
    set_u32_by_name(name, value ? 1u : 0u);
}

static void seed_runtime_platform_state(const HotshiftFileCpuState *state)
{
    char namebuf[256];
    uint32_t source_limit;
    uint32_t src_idx;

    if (state->clint_valid) {
        set_u64_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.pad_cpu_sys_cnt",
            state->clint_mtime);
        set_u64_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_sysio_top.ccvr",
            state->clint_mtime);
        set_u64_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_clint_top.x_ct_clint_func.clint_mtime_reg",
            state->clint_mtime);
        set_bit_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_clint_top.x_ct_clint_func.msip0_reg",
            (int)state->clint_msip);
        set_bit_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_clint_top.x_ct_clint_func.ssip0_reg",
            (int)state->clint_ssip);
        set_u32_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_clint_top.x_ct_clint_func.mtimecmp0_reg",
            (uint32_t)(state->clint_mtimecmp & 0xffffffffu));
        set_u32_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_clint_top.x_ct_clint_func.mtimecmph0_reg",
            (uint32_t)(state->clint_mtimecmp >> 32));
        set_u32_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_clint_top.x_ct_clint_func.stimecmp0_reg",
            (uint32_t)(state->clint_stimecmp & 0xffffffffu));
        set_u32_by_name(
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_ct_clint_top.x_ct_clint_func.stimecmph0_reg",
            (uint32_t)(state->clint_stimecmp >> 32));
    }

    if (!state->plic_valid) {
        return;
    }

    source_limit = state->plic_num_sources;
    if (source_limit > 256u) {
        source_limit = 256u;
    }

    for (src_idx = 1; src_idx < source_limit; ++src_idx) {
        snprintf(
            namebuf,
            sizeof(namebuf),
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_plic_top.x_plic_kid_busif.INT_KID[%u].x_plic_int_kid.int_priority",
            src_idx);
        set_u32_by_name(namebuf, state->plic_priority[src_idx] & 0x1fu);

        snprintf(
            namebuf,
            sizeof(namebuf),
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_plic_top.x_plic_kid_busif.INT_KID[%u].x_plic_int_kid.int_pending",
            src_idx);
        set_bit_by_name(
            namebuf,
            (int)((state->plic_pending[src_idx >> 5] >> (src_idx & 31)) & 0x1u));

        snprintf(
            namebuf,
            sizeof(namebuf),
            "tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top.x_plic_top.x_plic_kid_busif.INT_KID[%u].x_plic_int_kid.int_active",
            src_idx);
        set_bit_by_name(
            namebuf,
            (int)((state->plic_claimed[src_idx >> 5] >> (src_idx & 31)) & 0x1u));
    }
}

static void populate_tb_state(const HotshiftFileCpuState *state)
{
    uint32_t valid_mask;
    char namebuf[128];
    int i;

    valid_mask = 0xfffffffeu;
    for (i = 0; i < 32; ++i) {
        snprintf(namebuf, sizeof(namebuf), "tb.core0_snapshot_gpr[%d]", i);
        set_u64_by_name(namebuf, state->gpr[i]);
    }
    set_u32_by_name("tb.core0_snapshot_gpr_valid", valid_mask);
    set_u64_by_name("tb.core0_snapshot_pc", state->pc);
    set_u64_by_name("tb.core0_snapshot_medeleg", state->medeleg);
    set_u64_by_name("tb.core0_snapshot_mideleg", state->mideleg);
    set_u64_by_name("tb.core0_snapshot_mscratch", state->mscratch);
    set_u64_by_name("tb.core0_snapshot_mepc", state->mepc);
    set_u64_by_name("tb.core0_snapshot_mtval", state->mtval);
    set_u64_by_name("tb.core0_snapshot_mtvec", state->mtvec);
    set_u64_by_name("tb.core0_snapshot_stvec", state->stvec);
    set_u64_by_name("tb.core0_snapshot_sscratch", state->sscratch);
    set_u64_by_name("tb.core0_snapshot_sepc", state->sepc);
    set_u64_by_name("tb.core0_snapshot_stval", state->stval);
    set_u64_by_name("tb.core0_snapshot_satp", state->satp);
    set_u64_by_name("tb.core0_snapshot_mxstatus", state->mxstatus);
    set_u64_by_name("tb.core0_snapshot_sxstatus", state->sxstatus);

    set_u32_by_name("tb.core0_snapshot_pm",
                    (uint32_t)((state->mxstatus >> 30) & 0x3u));
    set_u32_by_name("tb.core0_snapshot_mpp",
                    (uint32_t)((state->mstatus >> 11) & 0x3u));
    set_u32_by_name("tb.core0_snapshot_fs",
                    (uint32_t)((state->mstatus >> 13) & 0x3u));
    set_bit_by_name("tb.core0_snapshot_spp",
                    (int)((state->sstatus >> 8) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_mie_bit",
                    (int)((state->mstatus >> 3) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_mpie",
                    (int)((state->mstatus >> 7) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_sie_bit",
                    (int)((state->sstatus >> 1) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_spie",
                    (int)((state->sstatus >> 5) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_mprv",
                    (int)((state->mstatus >> 17) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_sum",
                    (int)((state->sstatus >> 18) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_mxr",
                    (int)((state->sstatus >> 19) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_tvm",
                    (int)((state->mstatus >> 20) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_tw",
                    (int)((state->mstatus >> 21) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_tsr",
                    (int)((state->mstatus >> 22) & 0x1u));

    set_bit_by_name("tb.core0_snapshot_meie", (int)((state->mie >> 11) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_mtie", (int)((state->mie >> 7) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_msie", (int)((state->mie >> 3) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_seie", (int)((state->sie >> 9) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_stie", (int)((state->sie >> 5) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_ssie", (int)((state->sie >> 1) & 0x1u));

    set_bit_by_name("tb.core0_snapshot_m_intr",
                    (int)((state->mcause >> 63) & 0x1u));
    set_u32_by_name("tb.core0_snapshot_m_vector",
                    (uint32_t)(state->mcause & 0x1fu));

    set_bit_by_name("tb.core0_snapshot_me_int", (int)((state->mip >> 11) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_mt_int", (int)((state->mip >> 7) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_ms_int", (int)((state->mip >> 3) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_se_int", (int)((state->mip >> 9) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_st_int", (int)((state->mip >> 5) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_ss_int", (int)((state->mip >> 1) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_seip_reg", (int)((state->sip >> 9) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_stip_reg", (int)((state->sip >> 5) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_ssip_reg", (int)((state->sip >> 1) & 0x1u));

    set_bit_by_name("tb.core0_snapshot_s_intr",
                    (int)((state->scause >> 63) & 0x1u));
    set_u32_by_name("tb.core0_snapshot_s_vector",
                    (uint32_t)(state->scause & 0x1fu));

    set_bit_by_name("tb.core0_snapshot_mm",
                    (int)((state->mxstatus >> 15) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_ucme",
                    (int)((state->mxstatus >> 16) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_clintee",
                    (int)((state->mxstatus >> 17) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_mhrd",
                    (int)((state->mxstatus >> 18) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_insde",
                    (int)((state->mxstatus >> 19) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_maee",
                    (int)((state->mxstatus >> 21) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_cskyisaee",
                    (int)((state->mxstatus >> 22) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_pmdm",
                    (int)((state->mxstatus >> 13) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_pmds",
                    (int)((state->mxstatus >> 11) & 0x1u));
    set_bit_by_name("tb.core0_snapshot_pmdu",
                    (int)((state->mxstatus >> 10) & 0x1u));

    set_bit_by_name("tb.core0_snapshot_clint_valid", (int)state->clint_valid);
    set_u64_by_name("tb.core0_snapshot_clint_mtime", state->clint_mtime);
    set_u64_by_name("tb.core0_snapshot_clint_mtimecmp", state->clint_mtimecmp);
    set_u64_by_name("tb.core0_snapshot_clint_stimecmp", state->clint_stimecmp);
    set_bit_by_name("tb.core0_snapshot_clint_msip", (int)state->clint_msip);
    set_bit_by_name("tb.core0_snapshot_clint_ssip", (int)state->clint_ssip);

    set_bit_by_name("tb.core0_snapshot_plic_valid", (int)state->plic_valid);
    set_u32_by_name("tb.core0_snapshot_plic_num_sources", state->plic_num_sources);
    set_u32_by_name("tb.core0_snapshot_plic_bitfield_words", state->plic_bitfield_words);
    set_u32_by_name("tb.core0_snapshot_plic_context_mode", state->plic_context_mode);
    set_u32_by_name("tb.core0_snapshot_plic_threshold", state->plic_threshold);
    set_u32_by_name("tb.core0_snapshot_plic_context_claim_id",
                    state->plic_context_claim_id);
    for (i = 0; i < 8; ++i) {
        snprintf(namebuf, sizeof(namebuf), "tb.core0_snapshot_plic_pending[%d]", i);
        set_u32_by_name(namebuf, state->plic_pending[i]);
        snprintf(namebuf, sizeof(namebuf), "tb.core0_snapshot_plic_enable[%d]", i);
        set_u32_by_name(namebuf, state->plic_enable[i]);
        snprintf(namebuf, sizeof(namebuf), "tb.core0_snapshot_plic_claimed[%d]", i);
        set_u32_by_name(namebuf, state->plic_claimed[i]);
    }
    for (i = 0; i < 256; ++i) {
        snprintf(namebuf, sizeof(namebuf), "tb.core0_snapshot_plic_priority[%d]", i);
        set_u32_by_name(namebuf, state->plic_priority[i]);
    }

    seed_runtime_platform_state(state);
    set_bit_by_name("tb.core0_snapshot_restore_done", 0);
    set_bit_by_name("tb.core0_snapshot_restore_busy", 0);
    set_bit_by_name("tb.vpi_hotshift_pending", 1);
}

static PLI_INT32 hotshift_vpi_poll_calltf(PLI_BYTE8 *user_data)
{
    const char *state_file;
    FILE *fp;
    HotshiftFileCpuState state;
    uint32_t magic;
    uint32_t version;
    size_t nread;

    (void)user_data;

    state_file = getenv("UV_HOTSHIFT_STATE_FILE");
    if (state_file == NULL || state_file[0] == '\0') {
        return 0;
    }

    fp = fopen(state_file, "rb");
    if (fp == NULL) {
        if (errno != ENOENT) {
            vpi_printf("[hotshift_vpi] failed to open %s errno=%d\n",
                       state_file, errno);
        }
        return 0;
    }

    memset(&state, 0, sizeof(state));
    nread = fread(&state, 1, sizeof(state), fp);
    fclose(fp);
    if (nread != sizeof(state)) {
        vpi_printf("[hotshift_vpi] short read from %s: got %lu expected %lu\n",
                   state_file,
                   (unsigned long)nread,
                   (unsigned long)sizeof(state));
        unlink(state_file);
        return 0;
    }

    magic = ntohl(state.magic_be);
    version = ntohl(state.version_be);
    if (magic != HOTSHIFT_FILE_MAGIC || version != HOTSHIFT_FILE_VERSION) {
        vpi_printf("[hotshift_vpi] invalid state file header magic=0x%08x version=%u\n",
                   magic, version);
        unlink(state_file);
        return 0;
    }

    populate_tb_state(&state);
    unlink(state_file);
    vpi_printf("[hotshift_vpi] imported QEMU CPU state from %s\n", state_file);
    return 0;
}

static void register_hotshift_vpi_tasks(void)
{
    s_vpi_systf_data tf_data;

    memset(&tf_data, 0, sizeof(tf_data));
    tf_data.type = vpiSysTask;
    tf_data.tfname = "$hotshift_vpi_poll";
    tf_data.calltf = hotshift_vpi_poll_calltf;
    vpi_register_systf(&tf_data);
}

void hotshift_vpi_startup(void)
{
    register_hotshift_vpi_tasks();
}

void (*vpi_startup_routines[])(void) = {
    register_hotshift_vpi_tasks,
    0
};
