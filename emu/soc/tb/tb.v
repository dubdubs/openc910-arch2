/*Copyright 2019-2021 T-Head Semiconductor Co., Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
/*Copyright 2019-2021 T-Head Semiconductor Co., Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

`timescale 1ns/100ps

`define CLK_PERIOD          10
`define TCLK_PERIOD         40
`define MAX_RUN_TIME        32'h3000000

`define SOC_TOP             tb.x_soc
`define RTL_MEM             tb.x_soc.x_axi_slave128.x_f_spsram_large

`define CPU_TOP             tb.x_soc.x_cpu_sub_system_axi.x_rv_integration_platform.x_cpu_top
`define tb_retire0          `CPU_TOP.core0_pad_retire0
`define retire0_pc          `CPU_TOP.core0_pad_retire0_pc[39:0]
`define tb_retire1          `CPU_TOP.core0_pad_retire1
`define retire1_pc          `CPU_TOP.core0_pad_retire1_pc[39:0]
`define tb_retire2          `CPU_TOP.core0_pad_retire2
`define retire2_pc          `CPU_TOP.core0_pad_retire2_pc[39:0]
`define CPU_CLK             `CPU_TOP.pll_cpu_clk
`define CPU_RST             `CPU_TOP.pad_cpu_rst_b
`define clk_en              `CPU_TOP.axim_clk_en
`define CP0_RSLT_VLD        `CPU_TOP.x_ct_top_0.x_ct_core.x_ct_cp0_top.x_ct_cp0_iui.cp0_iu_ex3_rslt_vld
`define CP0_RSLT            `CPU_TOP.x_ct_top_0.x_ct_core.x_ct_cp0_top.x_ct_cp0_iui.cp0_iu_ex3_rslt_data[63:0]
`define CORE0_TOP           `CPU_TOP.x_ct_top_0
`define CORE0_HAD_REGS      `CORE0_TOP.x_ct_had_private_top.x_ct_had_regs
`define CORE0_HAD_CTRL      `CORE0_TOP.x_ct_had_private_top.x_ct_had_ctrl
`define CORE0_CP0_TOP       `CORE0_TOP.x_ct_core.x_ct_cp0_top
`define CORE0_CP0_REGS      `CORE0_CP0_TOP.x_ct_cp0_regs
`define CORE0_RTU_RETIRE    `CORE0_TOP.x_ct_core.x_ct_rtu_top.x_ct_rtu_retire

// `define APB_BASE_ADDR       40'h4000000000
`define APB_BASE_ADDR       40'hb0000000

module tb();
  reg clk;
  reg jclk;
  reg rst_b;
  reg jrst_b;
  reg jtap_en;
  wire jtg_tms;
  wire jtg_tdi;
  wire jtg_tdo;
  wire  pad_yy_gate_clk_en_b;
  
  static integer FILE;
  
  wire uart0_sin;
  wire [7:0]b_pad_gpio_porta;

  //-------------------------------------------------------------------------
  // QEMU snapshot restore knobs
  // Fill these values from a known-good QEMU checkpoint before running.
  //-------------------------------------------------------------------------
  localparam CORE0_SNAPSHOT_ENABLE          = 1'b1;
  localparam [31:0] CORE0_SNAPSHOT_TRIGGER_CYCLES = 32'd20000;
  localparam [63:0] CORE0_SNAPSHOT_PC       = 64'h0000_0000_0000_1000;
  localparam [63:0] CORE0_SNAPSHOT_MEPC     = 64'h0000_0000_0000_1000;
  localparam [63:0] CORE0_SNAPSHOT_MTVAL    = 64'h0000_0000_0000_0000;
  localparam [63:0] CORE0_SNAPSHOT_MTVEC    = 64'h0000_0000_0000_0000;
  localparam [1:0]  CORE0_SNAPSHOT_PM       = 2'b11;
  localparam [1:0]  CORE0_SNAPSHOT_MPP      = 2'b11;
  localparam [1:0]  CORE0_SNAPSHOT_FS       = 2'b00;
  localparam        CORE0_SNAPSHOT_SPP      = 1'b0;
  localparam        CORE0_SNAPSHOT_MIE_BIT  = 1'b0;
  localparam        CORE0_SNAPSHOT_MPIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_SIE_BIT  = 1'b0;
  localparam        CORE0_SNAPSHOT_SPIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_MPRV     = 1'b0;
  localparam        CORE0_SNAPSHOT_MXR      = 1'b0;
  localparam        CORE0_SNAPSHOT_SUM      = 1'b0;
  localparam        CORE0_SNAPSHOT_MEIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_MTIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_MSIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_SEIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_STIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_SSIE     = 1'b0;
  localparam        CORE0_SNAPSHOT_M_INTR   = 1'b0;
  localparam [4:0]  CORE0_SNAPSHOT_M_VECTOR = 5'd0;
  localparam        CORE0_SNAPSHOT_ME_INT   = 1'b0;
  localparam        CORE0_SNAPSHOT_MS_INT   = 1'b0;
  localparam        CORE0_SNAPSHOT_MT_INT   = 1'b0;
  localparam        CORE0_SNAPSHOT_SE_INT   = 1'b0;
  localparam        CORE0_SNAPSHOT_SS_INT   = 1'b0;
  localparam        CORE0_SNAPSHOT_ST_INT   = 1'b0;

  reg [63:0] core0_snapshot_gpr [0:31];
  reg [31:0] core0_snapshot_gpr_valid;
  reg [31:0] core0_no_retire_cycles;
  reg        core0_snapshot_restore_done;
  reg        core0_snapshot_restore_busy;
  integer    core0_snapshot_idx;

  function [31:0] core0_had_mv_self_ir;
    input [4:0] gpr_idx;
    begin
      case (gpr_idx[4:0])
        5'd0  : core0_had_mv_self_ir = 32'h0000_0013;
        5'd1  : core0_had_mv_self_ir = 32'h0000_8086;
        5'd2  : core0_had_mv_self_ir = 32'h0000_810a;
        5'd3  : core0_had_mv_self_ir = 32'h0000_818e;
        5'd4  : core0_had_mv_self_ir = 32'h0000_8212;
        5'd5  : core0_had_mv_self_ir = 32'h0000_8296;
        5'd6  : core0_had_mv_self_ir = 32'h0000_831a;
        5'd7  : core0_had_mv_self_ir = 32'h0000_839e;
        5'd8  : core0_had_mv_self_ir = 32'h0000_8422;
        5'd9  : core0_had_mv_self_ir = 32'h0000_84a6;
        5'd10 : core0_had_mv_self_ir = 32'h0000_852a;
        5'd11 : core0_had_mv_self_ir = 32'h0000_85ae;
        5'd12 : core0_had_mv_self_ir = 32'h0000_8632;
        5'd13 : core0_had_mv_self_ir = 32'h0000_86b6;
        5'd14 : core0_had_mv_self_ir = 32'h0000_873a;
        5'd15 : core0_had_mv_self_ir = 32'h0000_87be;
        5'd16 : core0_had_mv_self_ir = 32'h0000_8842;
        5'd17 : core0_had_mv_self_ir = 32'h0000_88c6;
        5'd18 : core0_had_mv_self_ir = 32'h0000_894a;
        5'd19 : core0_had_mv_self_ir = 32'h0000_89ce;
        5'd20 : core0_had_mv_self_ir = 32'h0000_8a52;
        5'd21 : core0_had_mv_self_ir = 32'h0000_8ad6;
        5'd22 : core0_had_mv_self_ir = 32'h0000_8b5a;
        5'd23 : core0_had_mv_self_ir = 32'h0000_8bde;
        5'd24 : core0_had_mv_self_ir = 32'h0000_8c62;
        5'd25 : core0_had_mv_self_ir = 32'h0000_8ce6;
        5'd26 : core0_had_mv_self_ir = 32'h0000_8d6a;
        5'd27 : core0_had_mv_self_ir = 32'h0000_8dee;
        5'd28 : core0_had_mv_self_ir = 32'h0000_8e72;
        5'd29 : core0_had_mv_self_ir = 32'h0000_8ef6;
        5'd30 : core0_had_mv_self_ir = 32'h0000_8f7a;
        5'd31 : core0_had_mv_self_ir = 32'h0000_8ffe;
        default: core0_had_mv_self_ir = 32'h0000_0013;
      endcase
    end
  endfunction

  task core0_force_snapshot_csrs;
    begin
      force `CORE0_CP0_REGS.pm               = CORE0_SNAPSHOT_PM;
      force `CORE0_CP0_REGS.mpp              = CORE0_SNAPSHOT_MPP;
      force `CORE0_CP0_REGS.fs               = CORE0_SNAPSHOT_FS;
      force `CORE0_CP0_REGS.spp              = CORE0_SNAPSHOT_SPP;
      force `CORE0_CP0_REGS.mie_bit          = CORE0_SNAPSHOT_MIE_BIT;
      force `CORE0_CP0_REGS.mpie             = CORE0_SNAPSHOT_MPIE;
      force `CORE0_CP0_REGS.sie_bit          = CORE0_SNAPSHOT_SIE_BIT;
      force `CORE0_CP0_REGS.spie             = CORE0_SNAPSHOT_SPIE;
      force `CORE0_CP0_REGS.mprv             = CORE0_SNAPSHOT_MPRV;
      force `CORE0_CP0_REGS.mxr              = CORE0_SNAPSHOT_MXR;
      force `CORE0_CP0_REGS.sum              = CORE0_SNAPSHOT_SUM;
      force `CORE0_CP0_REGS.meie             = CORE0_SNAPSHOT_MEIE;
      force `CORE0_CP0_REGS.mtie             = CORE0_SNAPSHOT_MTIE;
      force `CORE0_CP0_REGS.msie             = CORE0_SNAPSHOT_MSIE;
      force `CORE0_CP0_REGS.seie             = CORE0_SNAPSHOT_SEIE;
      force `CORE0_CP0_REGS.stie             = CORE0_SNAPSHOT_STIE;
      force `CORE0_CP0_REGS.ssie             = CORE0_SNAPSHOT_SSIE;
      force `CORE0_CP0_REGS.m_intr           = CORE0_SNAPSHOT_M_INTR;
      force `CORE0_CP0_REGS.m_vector         = CORE0_SNAPSHOT_M_VECTOR;
      force `CORE0_CP0_REGS.mtvec_base       = CORE0_SNAPSHOT_MTVEC[63:2];
      force `CORE0_CP0_REGS.mtvec_mode       = {1'b0, CORE0_SNAPSHOT_MTVEC[0]};
      force `CORE0_CP0_REGS.mepc_reg         = CORE0_SNAPSHOT_MEPC[63:1];
      force `CORE0_CP0_REGS.mtval_data       = CORE0_SNAPSHOT_MTVAL;
      force `CORE0_TOP.biu_cp0_me_int        = CORE0_SNAPSHOT_ME_INT;
      force `CORE0_TOP.biu_cp0_ms_int        = CORE0_SNAPSHOT_MS_INT;
      force `CORE0_TOP.biu_cp0_mt_int        = CORE0_SNAPSHOT_MT_INT;
      force `CORE0_TOP.biu_cp0_se_int        = CORE0_SNAPSHOT_SE_INT;
      force `CORE0_TOP.biu_cp0_ss_int        = CORE0_SNAPSHOT_SS_INT;
      force `CORE0_TOP.biu_cp0_st_int        = CORE0_SNAPSHOT_ST_INT;
    end
  endtask

  task core0_release_snapshot_csrs;
    begin
      release `CORE0_TOP.biu_cp0_st_int;
      release `CORE0_TOP.biu_cp0_ss_int;
      release `CORE0_TOP.biu_cp0_se_int;
      release `CORE0_TOP.biu_cp0_mt_int;
      release `CORE0_TOP.biu_cp0_ms_int;
      release `CORE0_TOP.biu_cp0_me_int;
      release `CORE0_CP0_REGS.mtval_data;
      release `CORE0_CP0_REGS.mepc_reg;
      release `CORE0_CP0_REGS.mtvec_mode;
      release `CORE0_CP0_REGS.mtvec_base;
      release `CORE0_CP0_REGS.m_vector;
      release `CORE0_CP0_REGS.m_intr;
      release `CORE0_CP0_REGS.ssie;
      release `CORE0_CP0_REGS.stie;
      release `CORE0_CP0_REGS.seie;
      release `CORE0_CP0_REGS.msie;
      release `CORE0_CP0_REGS.mtie;
      release `CORE0_CP0_REGS.meie;
      release `CORE0_CP0_REGS.sum;
      release `CORE0_CP0_REGS.mxr;
      release `CORE0_CP0_REGS.mprv;
      release `CORE0_CP0_REGS.spie;
      release `CORE0_CP0_REGS.sie_bit;
      release `CORE0_CP0_REGS.mpie;
      release `CORE0_CP0_REGS.mie_bit;
      release `CORE0_CP0_REGS.spp;
      release `CORE0_CP0_REGS.fs;
      release `CORE0_CP0_REGS.mpp;
      release `CORE0_CP0_REGS.pm;
    end
  endtask

  task core0_restore_one_gpr;
    input [4:0] gpr_idx;
    input [63:0] gpr_val;
    begin
      force `CORE0_HAD_REGS.wbbr_reg      = gpr_val;
      force `CORE0_HAD_REGS.ffy           = 1'b1;
      force `CORE0_HAD_REGS.ir_reg        = core0_had_mv_self_ir(gpr_idx);
      force `CORE0_HAD_CTRL.ctrl_go_noex  = 1'b1;
      force `CORE0_TOP.had_ifu_ir_vld     = 1'b1;
      repeat (2) @(posedge `CPU_CLK);
      release `CORE0_TOP.had_ifu_ir_vld;
      release `CORE0_HAD_CTRL.ctrl_go_noex;
      release `CORE0_HAD_REGS.ir_reg;
      release `CORE0_HAD_REGS.ffy;
      release `CORE0_HAD_REGS.wbbr_reg;
      @(posedge `CPU_CLK);
    end
  endtask

  task core0_restore_snapshot;
    begin
      $display("[SNAPSHOT] core0 restore start at cycle %0d", cycle_count);
      force `CORE0_TOP.rtu_yy_xx_dbgon      = 1'b1;
      force `CORE0_RTU_RETIRE.dbg_mode_on   = 1'b1;
      core0_force_snapshot_csrs();
      repeat (2) @(posedge `CPU_CLK);

      for (core0_snapshot_idx = 1; core0_snapshot_idx < 32; core0_snapshot_idx = core0_snapshot_idx + 1) begin
        if (core0_snapshot_gpr_valid[core0_snapshot_idx]) begin
          core0_restore_one_gpr(core0_snapshot_idx[4:0], core0_snapshot_gpr[core0_snapshot_idx]);
        end
      end

      force `CORE0_HAD_REGS.pc             = CORE0_SNAPSHOT_PC;
      force `CORE0_HAD_CTRL.ctrl_exit_dbg  = 1'b1;
      force `CORE0_TOP.had_ifu_pcload      = 1'b1;
      force `CORE0_TOP.had_yy_xx_exit_dbg  = 1'b1;
      repeat (2) @(posedge `CPU_CLK);

      release `CORE0_TOP.had_yy_xx_exit_dbg;
      release `CORE0_TOP.had_ifu_pcload;
      release `CORE0_HAD_CTRL.ctrl_exit_dbg;
      release `CORE0_HAD_REGS.pc;
      repeat (2) @(posedge `CPU_CLK);

      core0_release_snapshot_csrs();
      release `CORE0_RTU_RETIRE.dbg_mode_on;
      release `CORE0_TOP.rtu_yy_xx_dbgon;

      $display("[SNAPSHOT] core0 restore done, resume pc = 0x%h", CORE0_SNAPSHOT_PC);
    end
  endtask

  initial
  begin
    core0_snapshot_gpr_valid = 32'b0;
    for (core0_snapshot_idx = 0; core0_snapshot_idx < 32; core0_snapshot_idx = core0_snapshot_idx + 1)
      core0_snapshot_gpr[core0_snapshot_idx] = 64'b0;

    // Fill these from the QEMU architectural snapshot when needed.
    // Typical minimum set is x1(ra), x2(sp), x3(gp), x4(tp), plus any live a*/s* regs.
    core0_snapshot_restore_done = 1'b0;
    core0_snapshot_restore_busy = 1'b0;
    core0_no_retire_cycles      = 32'b0;
  end
  
  assign pad_yy_gate_clk_en_b = 1'b1;
  
  initial
  begin
    clk =0;
    forever begin
      #(`CLK_PERIOD/2) clk = ~clk;
    end
  end
  
  initial 
  begin 
    jclk = 0;
    forever begin
      #(`TCLK_PERIOD/2) jclk = ~jclk;
    end
  end
  
  initial
  begin
    rst_b = 1;
    #100;
    rst_b = 0;
    #100;
    rst_b = 1;
  end
  
  initial
  begin
    jrst_b = 1;
    #400;
    jrst_b = 0;
    #400;
    jrst_b = 1;
  end
 
  integer i;
  bit [31:0] mem_inst_temp [65536];
  bit [31:0] mem_data_temp [65536];
  integer j;
  initial
  begin
    $display("\t********* Init Program *********");
    $display("\t********* Wipe memory to 0 *********");
    for(i=0; i < 32'h16384; i=i+1)
    begin
      `RTL_MEM.ram0.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram1.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram2.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram3.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram4.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram5.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram6.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram7.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram8.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram9.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram10.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram11.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram12.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram13.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram14.mem[i][7:0] = 8'h0;
      `RTL_MEM.ram15.mem[i][7:0] = 8'h0;
    end
  
    $display("\t********* Read program *********");
    $readmemh("inst.pat", mem_inst_temp);
    $readmemh("data.pat", mem_data_temp);
  
    $display("\t********* Load program to memory *********");
    i=0;
    for(j=0;i<32'h4000;i=j/4)
    begin
      `RTL_MEM.ram0.mem[i][7:0] = mem_inst_temp[j][31:24];
      `RTL_MEM.ram1.mem[i][7:0] = mem_inst_temp[j][23:16];
      `RTL_MEM.ram2.mem[i][7:0] = mem_inst_temp[j][15: 8];
      `RTL_MEM.ram3.mem[i][7:0] = mem_inst_temp[j][ 7: 0];
      j = j+1;
      `RTL_MEM.ram4.mem[i][7:0] = mem_inst_temp[j][31:24];
      `RTL_MEM.ram5.mem[i][7:0] = mem_inst_temp[j][23:16];
      `RTL_MEM.ram6.mem[i][7:0] = mem_inst_temp[j][15: 8];
      `RTL_MEM.ram7.mem[i][7:0] = mem_inst_temp[j][ 7: 0];
      j = j+1;
      `RTL_MEM.ram8.mem[i][7:0] = mem_inst_temp[j][31:24];
      `RTL_MEM.ram9.mem[i][7:0] = mem_inst_temp[j][23:16];
      `RTL_MEM.ram10.mem[i][7:0] = mem_inst_temp[j][15: 8];
      `RTL_MEM.ram11.mem[i][7:0] = mem_inst_temp[j][ 7: 0];
      j = j+1;
      `RTL_MEM.ram12.mem[i][7:0] = mem_inst_temp[j][31:24];
      `RTL_MEM.ram13.mem[i][7:0] = mem_inst_temp[j][23:16];
      `RTL_MEM.ram14.mem[i][7:0] = mem_inst_temp[j][15: 8];
      `RTL_MEM.ram15.mem[i][7:0] = mem_inst_temp[j][ 7: 0];
      j = j+1;
    end
    i=0;
    for(j=0;i<32'h4000;i=j/4)
    begin
      `RTL_MEM.ram0.mem[i+32'h4000][7:0]  = mem_data_temp[j][31:24];
      `RTL_MEM.ram1.mem[i+32'h4000][7:0]  = mem_data_temp[j][23:16];
      `RTL_MEM.ram2.mem[i+32'h4000][7:0]  = mem_data_temp[j][15: 8];
      `RTL_MEM.ram3.mem[i+32'h4000][7:0]  = mem_data_temp[j][ 7: 0];
      j = j+1;
      `RTL_MEM.ram4.mem[i+32'h4000][7:0]  = mem_data_temp[j][31:24];
      `RTL_MEM.ram5.mem[i+32'h4000][7:0]  = mem_data_temp[j][23:16];
      `RTL_MEM.ram6.mem[i+32'h4000][7:0]  = mem_data_temp[j][15: 8];
      `RTL_MEM.ram7.mem[i+32'h4000][7:0]  = mem_data_temp[j][ 7: 0];
      j = j+1;
      `RTL_MEM.ram8.mem[i+32'h4000][7:0]   = mem_data_temp[j][31:24];
      `RTL_MEM.ram9.mem[i+32'h4000][7:0]   = mem_data_temp[j][23:16];
      `RTL_MEM.ram10.mem[i+32'h4000][7:0]  = mem_data_temp[j][15: 8];
      `RTL_MEM.ram11.mem[i+32'h4000][7:0]  = mem_data_temp[j][ 7: 0];
      j = j+1;
      `RTL_MEM.ram12.mem[i+32'h4000][7:0]  = mem_data_temp[j][31:24];
      `RTL_MEM.ram13.mem[i+32'h4000][7:0]  = mem_data_temp[j][23:16];
      `RTL_MEM.ram14.mem[i+32'h4000][7:0]  = mem_data_temp[j][15: 8];
      `RTL_MEM.ram15.mem[i+32'h4000][7:0]  = mem_data_temp[j][ 7: 0];
      j = j+1;
    end
  end

  initial
  begin
  #(`MAX_RUN_TIME * `CLK_PERIOD);
    $display("**********************************************");
    $display("*   meeting max simulation time, stop!       *");
    $display("**********************************************");
    FILE = $fopen("run_case.report","w");
    $fwrite(FILE,"TEST FAIL");   
  $finish;
  end
  
  reg [31:0] retire_inst_in_period;
  reg [31:0] cycle_count;
  
  `define LAST_CYCLE 50000
  always @(posedge clk or negedge rst_b)
  begin
    if(!rst_b)
      cycle_count[31:0] <= 32'b1;
    else 
      cycle_count[31:0] <= cycle_count[31:0] + 1'b1;
  end

  always @(posedge `CPU_CLK or negedge `CPU_RST)
  begin
    if(!`CPU_RST)
      core0_no_retire_cycles[31:0] <= 32'b0;
    else if(core0_snapshot_restore_busy || core0_snapshot_restore_done)
      core0_no_retire_cycles[31:0] <= 32'b0;
    else if(`tb_retire0 || `tb_retire1 || `tb_retire2)
      core0_no_retire_cycles[31:0] <= 32'b0;
    else
      core0_no_retire_cycles[31:0] <= core0_no_retire_cycles[31:0] + 1'b1;
  end

  initial
  begin : auto_restore_core0_snapshot
    wait(`CPU_RST == 1'b1);
    forever begin
      @(posedge `CPU_CLK);
      if(CORE0_SNAPSHOT_ENABLE
         && !core0_snapshot_restore_done
         && !core0_snapshot_restore_busy
         && (core0_no_retire_cycles[31:0] >= CORE0_SNAPSHOT_TRIGGER_CYCLES)) begin
        core0_snapshot_restore_busy = 1'b1;
        core0_restore_snapshot();
        core0_snapshot_restore_busy = 1'b0;
        core0_snapshot_restore_done = 1'b1;
      end
    end
  end
  
  
  always @(posedge clk or negedge rst_b)
  begin
    if(!rst_b) //reset to zero
      retire_inst_in_period[31:0] <= 32'b0;
    else if( (cycle_count[31:0] % `LAST_CYCLE) == 0)//check and reset retire_inst_in_period every 50000 cycles
    begin
      if(retire_inst_in_period[31:0] == 0)begin
        $display("*************************************************************");
        $display("* Error: There is no instructions retired in the last %d cycles! *", `LAST_CYCLE);
        $display("*              Simulation Fail and Finished!                *");
        $display("*************************************************************");
        #10;
        FILE = $fopen("run_case.report","w");
        $fwrite(FILE,"TEST FAIL");   
  
        $finish;
      end
      retire_inst_in_period[31:0] <= 32'b0;
    end
    else if(`tb_retire0 || `tb_retire1 || `tb_retire2)
      retire_inst_in_period[31:0] <= retire_inst_in_period[31:0] + 1'b1;
  end
  
  
  
  reg [31:0] cpu_awaddr;
  reg [3:0]  cpu_awlen;
  reg [15:0] cpu_wstrb;
  reg        cpu_wvalid;
  reg [63:0] value0;
  reg [63:0] value1;
  reg [63:0] value2;
  
  
  always @(posedge clk)
  begin
    cpu_awlen[3:0]   <= `SOC_TOP.x_axi_slave128.awlen[3:0];
    cpu_awaddr[31:0] <= `SOC_TOP.x_axi_slave128.mem_addr[31:0];
    cpu_wvalid       <= `SOC_TOP.biu_pad_wvalid;
    cpu_wstrb        <= `SOC_TOP.biu_pad_wstrb;
    // value0           <= `CPU_TOP.core0_pad_wb0_data[63:0];
    // value1           <= `CPU_TOP.core0_pad_wb1_data[63:0];
    // value2           <= `CPU_TOP.core0_pad_wb2_data[63:0];
    value0              <= `CPU_TOP.x_ct_top_0.x_ct_core.x_ct_iu_top.x_ct_iu_rbus.rbus_pipe0_wb_data[63:0];
    value1              <= `CPU_TOP.x_ct_top_0.x_ct_core.x_ct_iu_top.x_ct_iu_rbus.rbus_pipe1_wb_data[63:0];
    value2              <= `CPU_TOP.x_ct_top_0.x_ct_core.x_ct_lsu_top.x_ct_lsu_ld_wb.ld_wb_preg_data_sign_extend[63:0];
  end
  
  always @(posedge clk)
  begin
      if(value0 == 64'h444333222 || value1 == 64'h444333222 || value2 == 64'h444333222)
    begin
      $display("**********************************************");
      $display("*    simulation finished successfully        *");
      $display("**********************************************");
     #10;
     FILE = $fopen("run_case.report","w");
     $fwrite(FILE,"TEST PASS");   
  
     $finish;
    end
      else if (value0 == 64'h2382348720 || value1 == 64'h2382348720 || value2 == 64'h444333222)
    begin
     $display("**********************************************");
     $display("*    simulation finished with error          *");
     $display("**********************************************");
     #10;
     FILE = $fopen("run_case.report","w");
     $fwrite(FILE,"TEST FAIL");   
  
     $finish;
    end
  
    else if((cpu_awlen[3:0] == 4'b0) &&
  //     (cpu_awaddr[31:0] == 32'h6000fff8) &&
  //     (cpu_awaddr[31:0] == 32'h0003fff8) &&
       (cpu_awaddr[31:0] == 32'h01ff_fff0) &&
        cpu_wvalid &&
       `clk_en)
    begin
     if(cpu_wstrb[15:0] == 16'hf)
     begin
        $write("%c", `SOC_TOP.biu_pad_wdata[7:0]);
     end
     else if(cpu_wstrb[15:0] == 16'hf0)
     begin
        $write("%c", `SOC_TOP.biu_pad_wdata[39:32]);
     end
     else if(cpu_wstrb[15:0] == 16'hf00)
     begin
        $write("%c", `SOC_TOP.biu_pad_wdata[71:64]);
     end
     else if(cpu_wstrb[15:0] == 16'hf000)
     begin
        $write("%c", `SOC_TOP.biu_pad_wdata[103:96]);
     end
    end
  
  end
  
  
  
  parameter cpu_cycle = 110;
  `ifndef NO_DUMP
  initial
  begin
  `ifdef NC_SIM
    $dumpfile("test.vcd");
    $dumpvars;  
  `else
    `ifdef IVERILOG_SIM
      $dumpfile("test.vcd");
      $dumpvars;  
    `else
      $fsdbDumpvars();
    `endif
  `endif
  end
  `endif
  
  assign jtg_tdi = 1'b0;
  assign uart0_sin = 1'b1;
  
  
  soc x_soc(
    .i_pad_clk           ( clk                  ),
    .b_pad_gpio_porta    ( b_pad_gpio_porta     ),
    .i_pad_jtg_trst_b    ( jrst_b               ),
    .i_pad_jtg_tclk      ( jclk                 ),
    .i_pad_jtg_tdi       ( jtg_tdi              ),
    .i_pad_jtg_tms       ( jtg_tms              ),
    .i_pad_uart0_sin     ( uart0_sin            ),
    .o_pad_jtg_tdo       ( jtg_tdo              ),
    .o_pad_uart0_sout    ( uart0_sout           ),
    .i_pad_rst_b         ( rst_b                )
  );
  
  int_mnt x_int_mnt(
  );
  
  // debug_stim x_debug_stim(
  // );

// Latest Power control
`ifdef UPF_INCLUDED
  import UPF::*;

  initial
  begin
        supply_on ("VDD", 1.00);
     	supply_on ("VDDG", 1.00);
  end

  initial 
  begin
    $deposit(tb.x_soc.pmu_cpu_pwr_on,  1'b1);
    $deposit(tb.x_soc.pmu_cpu_iso_in,  1'b0);
    $deposit(tb.x_soc.pmu_cpu_iso_out, 1'b0);
    $deposit(tb.x_soc.pmu_cpu_save,    1'b0);
    $deposit(tb.x_soc.pmu_cpu_restore, 1'b0);
  end
`endif
  
  reg [31:0] virtual_counter;
  
  always @(posedge `CPU_CLK or negedge `CPU_RST)
  begin
    if(!`CPU_RST)
      virtual_counter[31:0] <= 32'b0;
    else if(virtual_counter[31:0]==32'hffffffff)
      virtual_counter[31:0] <= virtual_counter[31:0];
    else
      virtual_counter[31:0] <= virtual_counter[31:0] +1'b1;
  end 
  
  //always @(*)
  //begin
  //if(virtual_counter[31:0]> 32'h3000000) $finish;
  //end
  
endmodule
