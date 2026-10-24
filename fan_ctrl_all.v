// fan_ctrl_all.v : all RTL concatenated, `include files expanded.  Top = fan_ctrl_top

// ===== sync_ff.v =====
// -----------------------------------------------------------------------------
// sync_ff : N-stage flip-flop synchronizer for a SINGLE-BIT asynchronous signal.
//   * Use only for 1-bit level signals or toggle signals (never for buses).
//   * Async reset to RESET_VAL; the reset must itself be deasserted
//     synchronously to 'clk' (see reset_sync).
// -----------------------------------------------------------------------------
module sync_ff #(
    parameter STAGES    = 2,
    parameter RESET_VAL = 1'b0
) (
    input  wire clk,
    input  wire rstn,
    input  wire d,
    output wire q
);
    (* ASYNC_REG = "TRUE" *) reg [STAGES-1:0] sr;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) sr <= {STAGES{RESET_VAL}};
        else       sr <= {sr[STAGES-2:0], d};
    end

    assign q = sr[STAGES-1];
endmodule

// ===== reset_sync.v =====
// -----------------------------------------------------------------------------
// reset_sync : asynchronous-assert, synchronous-deassert reset synchronizer.
//   arstn  : external asynchronous active-low reset
//   srstn  : reset released synchronously to 'clk' (2 clock edges after arstn)
// -----------------------------------------------------------------------------
module reset_sync (
    input  wire clk,
    input  wire arstn,
    output wire srstn
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] sr;

    always @(posedge clk or negedge arstn) begin
        if (!arstn) sr <= 2'b00;
        else        sr <= {sr[0], 1'b1};
    end

    assign srstn = sr[1];
endmodule

// ===== cdc_bus_hs.v =====
// -----------------------------------------------------------------------------
// cdc_bus_hs : multi-bit clock-domain crossing using a 4-phase-equivalent
//              toggle handshake. The bus is COHERENT: all W bits arrive in the
//              destination domain together from one snapshot.
//
//   src domain : whenever src_data differs from the last snapshot and the
//                channel is idle, snapshot it into 'hold' and toggle req_t.
//   dst domain : req_t is synchronized (2-FF); on a change the (stable) 'hold'
//                is sampled into dst_data and ack_t is toggled back.
//   src domain : ack_t synchronized back; when ack == req the channel is idle.
//
//   'hold' never changes while a transfer is in flight, so the destination
//   samples a stable bus. Latency ~ 2 dst clocks + 2 src clocks + 2.
//   Assumption: both domains share ONE asynchronous reset source (each reset
//   deasserted synchronously by its own reset_sync), so req/ack start equal.
// -----------------------------------------------------------------------------
module cdc_bus_hs #(
    parameter W = 8
) (
    input  wire         src_clk,
    input  wire         src_rstn,
    input  wire [W-1:0] src_data,

    input  wire         dst_clk,
    input  wire         dst_rstn,
    output reg  [W-1:0] dst_data,
    output reg          dst_valid     // 1-cycle pulse when dst_data refreshed
);
    // ---------------- source side ----------------
    reg  [W-1:0] hold;
    reg          req_t;
    reg          busy;
    wire         ack_s;                // ack toggle, synchronized to src_clk

    always @(posedge src_clk or negedge src_rstn) begin
        if (!src_rstn) begin
            hold  <= {W{1'b0}};
            req_t <= 1'b0;
            busy  <= 1'b0;
        end else begin
            if (!busy) begin
                if (src_data != hold) begin
                    hold  <= src_data;
                    req_t <= ~req_t;
                    busy  <= 1'b1;
                end
            end else if (ack_s == req_t) begin
                busy <= 1'b0;
            end
        end
    end

    // ---------------- destination side ----------------
    wire req_s;                        // req toggle, synchronized to dst_clk
    reg  ack_t;

    sync_ff #(.STAGES(2)) u_req_sync (.clk(dst_clk), .rstn(dst_rstn), .d(req_t), .q(req_s));
    sync_ff #(.STAGES(2)) u_ack_sync (.clk(src_clk), .rstn(src_rstn), .d(ack_t), .q(ack_s));

    always @(posedge dst_clk or negedge dst_rstn) begin
        if (!dst_rstn) begin
            dst_data  <= {W{1'b0}};
            dst_valid <= 1'b0;
            ack_t     <= 1'b0;
        end else begin
            dst_valid <= 1'b0;
            if (req_s != ack_t) begin
                dst_data  <= hold;     // 'hold' is stable: src waits for ack
                dst_valid <= 1'b1;
                ack_t     <= req_s;
            end
        end
    end
endmodule

// ===== axi4l_slave_if.v =====
// -----------------------------------------------------------------------------
// axi4l_slave_if : AXI4-Lite slave -> simple register bus bridge.
//
//   * AW and W are accepted INDEPENDENTLY (any order, or simultaneously).
//   * One outstanding write and one outstanding read at a time.
//   * Write executes in the cycle after both AW and W have been captured:
//        wr_en (1-cycle pulse) with wr_addr / wr_data / wr_strb; the register
//        file answers COMBINATIONALLY in that same cycle with wr_err.
//   * Read executes in the cycle after AR is captured: rd_en pulse, rd_data /
//        rd_err valid combinationally.
//   * wr_err / rd_err -> SLVERR (2'b10), otherwise OKAY (2'b00).
//   * All outputs are registered or derived from registers (no comb. path from
//     master VALID to slave READY).
// -----------------------------------------------------------------------------
module axi4l_slave_if #(
    parameter AW = 8
) (
    input  wire          ACLK,
    input  wire          ARESETN,

    input  wire [AW-1:0] AWADDR,
    input  wire          AWVALID,
    output wire          AWREADY,
    input  wire [31:0]   WDATA,
    input  wire [3:0]    WSTRB,
    input  wire          WVALID,
    output wire          WREADY,
    output reg  [1:0]    BRESP,
    output reg           BVALID,
    input  wire          BREADY,

    input  wire [AW-1:0] ARADDR,
    input  wire          ARVALID,
    output wire          ARREADY,
    output reg  [31:0]   RDATA,
    output reg  [1:0]    RRESP,
    output reg           RVALID,
    input  wire          RREADY,

    // register bus
    output wire          wr_en,
    output wire [AW-1:0] wr_addr,
    output wire [31:0]   wr_data,
    output wire [3:0]    wr_strb,
    input  wire          wr_err,
    output wire          rd_en,
    output wire [AW-1:0] rd_addr,
    input  wire [31:0]   rd_data,
    input  wire          rd_err
);
    // ------------------------------ write ------------------------------
    reg          aw_got, w_got;
    reg [AW-1:0] awaddr_q;
    reg [31:0]   wdata_q;
    reg [3:0]    wstrb_q;

    assign AWREADY = ~aw_got & ~BVALID;
    assign WREADY  = ~w_got  & ~BVALID;

    wire aw_fire = AWVALID & AWREADY;
    wire w_fire  = WVALID  & WREADY;

    assign wr_en   = aw_got & w_got;      // exactly one cycle (flags cleared below)
    assign wr_addr = awaddr_q;
    assign wr_data = wdata_q;
    assign wr_strb = wstrb_q;

    always @(posedge ACLK or negedge ARESETN) begin
        if (!ARESETN) begin
            aw_got   <= 1'b0;
            w_got    <= 1'b0;
            awaddr_q <= {AW{1'b0}};
            wdata_q  <= 32'd0;
            wstrb_q  <= 4'd0;
            BVALID   <= 1'b0;
            BRESP    <= 2'b00;
        end else begin
            if (aw_fire) begin aw_got <= 1'b1; awaddr_q <= AWADDR; end
            if (w_fire)  begin w_got  <= 1'b1; wdata_q  <= WDATA; wstrb_q <= WSTRB; end

            if (wr_en) begin
                aw_got <= 1'b0;
                w_got  <= 1'b0;
                BVALID <= 1'b1;
                BRESP  <= wr_err ? 2'b10 : 2'b00;
            end else if (BVALID && BREADY) begin
                BVALID <= 1'b0;
            end
        end
    end

    // ------------------------------ read -------------------------------
    reg          ar_got;
    reg [AW-1:0] araddr_q;

    assign ARREADY = ~ar_got & ~RVALID;
    wire   ar_fire = ARVALID & ARREADY;

    assign rd_en   = ar_got;
    assign rd_addr = araddr_q;

    always @(posedge ACLK or negedge ARESETN) begin
        if (!ARESETN) begin
            ar_got   <= 1'b0;
            araddr_q <= {AW{1'b0}};
            RVALID   <= 1'b0;
            RRESP    <= 2'b00;
            RDATA    <= 32'd0;
        end else begin
            if (ar_fire) begin ar_got <= 1'b1; araddr_q <= ARADDR; end

            if (rd_en) begin
                ar_got <= 1'b0;
                RVALID <= 1'b1;
                RDATA  <= rd_err ? 32'd0 : rd_data;
                RRESP  <= rd_err ? 2'b10 : 2'b00;
            end else if (RVALID && RREADY) begin
                RVALID <= 1'b0;
            end
        end
    end
endmodule

// ===== fan_regs.v =====
// -----------------------------------------------------------------------------
// fan_regs : ACLK-domain register file, IRQ controller, write-lock.
//   Unmapped / read-only-written / unaligned / locked-write -> SLVERR.
//   Byte strobes honoured on every writable register.
// -----------------------------------------------------------------------------
// Bundle widths needed in module port lists (checked against fan_cfg.vh / fan_stat.vh
// inside the modules; a mismatch breaks elaboration).
`define FAN_CFG_W 641
`define FAN_ST_W  105

module fan_regs #(
    parameter NCH = 4                       // 1..8
) (
    input  wire              ACLK,
    input  wire              ARESETN,
    input  wire              wr_en,
    input  wire [7:0]        wr_addr,
    input  wire [31:0]       wr_data,
    input  wire [3:0]        wr_strb,
    output reg               wr_err,
    input  wire              rd_en,
    input  wire [7:0]        rd_addr,
    output reg  [31:0]       rd_data,
    output reg               rd_err,
    output wire [`FAN_CFG_W-1:0] cfg_bus,
    input  wire [`FAN_ST_W-1:0]  st_bus,
    output wire              irq
);
    // -----------------------------------------------------------------------------
// fan_cfg.vh : SINGLE SOURCE OF TRUTH for the ACLK->PCLK configuration bundle.
// Included inside both fan_regs (packs) and fan_core (unpacks), so the two
// sides cannot disagree on field positions.  CW_x = width, CO_x = LSB offset.
// Per-channel fields are always sized for the maximum of 8 channels.
// -----------------------------------------------------------------------------
/* verilator lint_off UNUSEDPARAM */
localparam CW_CTRL   = 8;    // [0]en [2:1]mode [3]center [4]comp [5]temp_src_sw [6]sync_en
localparam CW_PRESC  = 16;
localparam CW_PERIOD = 16;
localparam CW_DEAD   = 8;
localparam CW_POL    = 8;
localparam CW_FLVL   = 8;
localparam CW_GAMMA  = 8;
localparam CW_DUTY   = 128;  // 8 x 16
localparam CW_PHASE  = 128;  // 8 x 16
localparam CW_CT     = 32;   // 4 x 8   curve temperatures
localparam CW_CL     = 64;   // 4 x 16  curve levels
localparam CW_HYST   = 8;
localparam CW_TCRIT  = 8;
localparam CW_FANMIN = 16;
localparam CW_TEMPSW = 8;
localparam CW_KP     = 16;
localparam CW_KI     = 16;
localparam CW_RPMMAX = 16;
localparam CW_GATE   = 32;
localparam CW_MULT   = 16;
localparam CW_STALL  = 12;   // [7:0] duty threshold (top byte), [11:8] windows
localparam CW_FAULTEN= 8;
localparam CW_STALETO= 24;
localparam CW_WDOGTO = 24;
localparam CW_WDOGEN = 1;
localparam CW_CMD    = 12;   // [3:0] fault_clr_cnt [7:4] wdog_kick_cnt [11:8] tele_clr_cnt

localparam CO_CTRL   = 0;
localparam CO_PRESC  = CO_CTRL   + CW_CTRL;
localparam CO_PERIOD = CO_PRESC  + CW_PRESC;
localparam CO_DEAD   = CO_PERIOD + CW_PERIOD;
localparam CO_POL    = CO_DEAD   + CW_DEAD;
localparam CO_FLVL   = CO_POL    + CW_POL;
localparam CO_GAMMA  = CO_FLVL   + CW_FLVL;
localparam CO_DUTY   = CO_GAMMA  + CW_GAMMA;
localparam CO_PHASE  = CO_DUTY   + CW_DUTY;
localparam CO_CT     = CO_PHASE  + CW_PHASE;
localparam CO_CL     = CO_CT     + CW_CT;
localparam CO_HYST   = CO_CL     + CW_CL;
localparam CO_TCRIT  = CO_HYST   + CW_HYST;
localparam CO_FANMIN = CO_TCRIT  + CW_TCRIT;
localparam CO_TEMPSW = CO_FANMIN + CW_FANMIN;
localparam CO_KP     = CO_TEMPSW + CW_TEMPSW;
localparam CO_KI     = CO_KP     + CW_KP;
localparam CO_RPMMAX = CO_KI     + CW_KI;
localparam CO_GATE   = CO_RPMMAX + CW_RPMMAX;
localparam CO_MULT   = CO_GATE   + CW_GATE;
localparam CO_STALL  = CO_MULT   + CW_MULT;
localparam CO_FAULTEN= CO_STALL  + CW_STALL;
localparam CO_STALETO= CO_FAULTEN+ CW_FAULTEN;
localparam CO_WDOGTO = CO_STALETO+ CW_STALETO;
localparam CO_WDOGEN = CO_WDOGTO + CW_WDOGTO;
localparam CO_CMD    = CO_WDOGEN + CW_WDOGEN;
localparam CFG_W     = CO_CMD    + CW_CMD;
/* verilator lint_on UNUSEDPARAM */

    // -----------------------------------------------------------------------------
// fan_stat.vh : PCLK->ACLK status/telemetry bundle layout (single source of
// truth, included by fan_core (packs) and fan_regs (unpacks)).
// -----------------------------------------------------------------------------
/* verilator lint_off UNUSEDPARAM */
localparam SW_RPM  = 16;
localparam SW_DUTY = 16;
localparam SW_TEMP = 8;
localparam SW_FLT  = 8;
localparam SW_FCNT = 8;
localparam SW_DMIN = 16;
localparam SW_DMAX = 16;
localparam SW_TCNT = 8;    // increments on every tach window completion
localparam SW_PCNT = 8;    // increments on every PWM frame
localparam SW_SAFE = 1;

localparam SO_RPM  = 0;
localparam SO_DUTY = SO_RPM  + SW_RPM;
localparam SO_TEMP = SO_DUTY + SW_DUTY;
localparam SO_FLT  = SO_TEMP + SW_TEMP;
localparam SO_FCNT = SO_FLT  + SW_FLT;
localparam SO_DMIN = SO_FCNT + SW_FCNT;
localparam SO_DMAX = SO_DMIN + SW_DMIN;
localparam SO_TCNT = SO_DMAX + SW_DMAX;
localparam SO_PCNT = SO_TCNT + SW_TCNT;
localparam SO_SAFE = SO_PCNT + SW_PCNT;
localparam ST_W    = SO_SAFE + SW_SAFE;
/* verilator lint_on UNUSEDPARAM */


    generate if (CFG_W != `FAN_CFG_W || ST_W != `FAN_ST_W) begin : g_width_check
        BUNDLE_WIDTH_MISMATCH_update_fan_defs_vh bad();
    end endgenerate

    localparam [31:0] ID_VALUE = 32'h4641_4E01;   // "FAN" v1

    // word indices (byte address / 4)
    localparam A_ID=0,  A_CTRL=1,  A_STATUS=2, A_IRQEN=3, A_IRQST=4, A_LOCK=5,
               A_PRESC=6, A_PERIOD=7, A_DEAD=8, A_POL=9, A_FLVL=10, A_GAMMA=11,
               A_DUTY0=12, A_PHASE0=20, A_CT=28, A_CL0=29, A_HYST=33, A_TCRIT=34,
               A_FANMIN=35, A_TEMPSW=36, A_TEMPNOW=37, A_KP=38, A_KI=39,
               A_RPMMAX=40, A_GATE=41, A_MULT=42, A_STALL=43, A_FAULTEN=44,
               A_STALETO=45, A_WDOGTO=46, A_WDOGCTL=47, A_WDOGKICK=48,
               A_FSTAT=49, A_FCLR=50, A_TCLR=51, A_RPM=52, A_DUTYNOW=53,
               A_DMIN=54, A_DMAX=55, A_FCNT=56;

    // ---------------- state ----------------
    reg [6:0]  r_ctrl;
    reg [2:0]  r_irqen, r_irqst;
    reg        r_locked;
    reg [15:0] r_presc, r_period;
    reg [7:0]  r_dead, r_pol, r_flvl, r_gamma;
    reg [15:0] r_duty  [0:7];
    reg [15:0] r_phase [0:7];
    reg [31:0] r_ct;
    reg [15:0] r_cl0, r_cl1, r_cl2, r_cl3;
    reg [7:0]  r_hyst, r_tcrit, r_tempsw;
    reg [15:0] r_fanmin, r_kp, r_ki, r_rpmmax, r_mult;
    reg [31:0] r_gate;
    reg [11:0] r_stall;
    reg [4:0]  r_faulten;
    reg [23:0] r_staleto, r_wdogto;
    reg        r_wdogen;
    reg [3:0]  r_fclr, r_kick, r_tclr;

    // ---------------- status unpack ----------------
    wire [15:0] st_rpm  = st_bus[SO_RPM  +: SW_RPM];
    wire [15:0] st_duty = st_bus[SO_DUTY +: SW_DUTY];
    wire [7:0]  st_temp = st_bus[SO_TEMP +: SW_TEMP];
    wire [7:0]  st_flt  = st_bus[SO_FLT  +: SW_FLT];
    wire [7:0]  st_fcnt = st_bus[SO_FCNT +: SW_FCNT];
    wire [15:0] st_dmin = st_bus[SO_DMIN +: SW_DMIN];
    wire [15:0] st_dmax = st_bus[SO_DMAX +: SW_DMAX];
    wire [7:0]  st_tcnt = st_bus[SO_TCNT +: SW_TCNT];
    wire [7:0]  st_pcnt = st_bus[SO_PCNT +: SW_PCNT];
    wire        st_safe = st_bus[SO_SAFE];

    // ---------------- config pack ----------------
    assign cfg_bus[CO_CTRL   +: CW_CTRL  ] = {1'b0, r_ctrl};
    assign cfg_bus[CO_PRESC  +: CW_PRESC ] = r_presc;
    assign cfg_bus[CO_PERIOD +: CW_PERIOD] = r_period;
    assign cfg_bus[CO_DEAD   +: CW_DEAD  ] = r_dead;
    assign cfg_bus[CO_POL    +: CW_POL   ] = r_pol;
    assign cfg_bus[CO_FLVL   +: CW_FLVL  ] = r_flvl;
    assign cfg_bus[CO_GAMMA  +: CW_GAMMA ] = r_gamma;
    genvar g;
    generate for (g = 0; g < 8; g = g + 1) begin : g_ch
        assign cfg_bus[CO_DUTY  + 16*g +: 16] = (g < NCH) ? r_duty[g]  : 16'd0;
        assign cfg_bus[CO_PHASE + 16*g +: 16] = (g < NCH) ? r_phase[g] : 16'd0;
    end endgenerate
    assign cfg_bus[CO_CT     +: CW_CT    ] = r_ct;
    assign cfg_bus[CO_CL     +: CW_CL    ] = {r_cl3, r_cl2, r_cl1, r_cl0};
    assign cfg_bus[CO_HYST   +: CW_HYST  ] = r_hyst;
    assign cfg_bus[CO_TCRIT  +: CW_TCRIT ] = r_tcrit;
    assign cfg_bus[CO_FANMIN +: CW_FANMIN] = r_fanmin;
    assign cfg_bus[CO_TEMPSW +: CW_TEMPSW] = r_tempsw;
    assign cfg_bus[CO_KP     +: CW_KP    ] = r_kp;
    assign cfg_bus[CO_KI     +: CW_KI    ] = r_ki;
    assign cfg_bus[CO_RPMMAX +: CW_RPMMAX] = r_rpmmax;
    assign cfg_bus[CO_GATE   +: CW_GATE  ] = r_gate;
    assign cfg_bus[CO_MULT   +: CW_MULT  ] = r_mult;
    assign cfg_bus[CO_STALL  +: CW_STALL ] = r_stall;
    assign cfg_bus[CO_FAULTEN+: CW_FAULTEN] = {3'b0, r_faulten};
    assign cfg_bus[CO_STALETO+: CW_STALETO] = r_staleto;
    assign cfg_bus[CO_WDOGTO +: CW_WDOGTO] = r_wdogto;
    assign cfg_bus[CO_WDOGEN +: CW_WDOGEN] = r_wdogen;
    assign cfg_bus[CO_CMD    +: CW_CMD   ] = {r_tclr, r_kick, r_fclr};

    // ---------------- helpers ----------------
    function [31:0] mrg;
        input [31:0] o; input [31:0] n; input [3:0] s;
        mrg = {s[3] ? n[31:24] : o[31:24], s[2] ? n[23:16] : o[23:16],
               s[1] ? n[15:8]  : o[15:8],  s[0] ? n[7:0]   : o[7:0]};
    endfunction

    // write decode: {valid, lockable}
    wire [5:0] wi = wr_addr[7:2];
    reg        w_valid, w_lockable;
    always @* begin
        w_valid = 1'b0; w_lockable = 1'b0;
        if (wr_addr[1:0] == 2'b00) begin
            case (wi)
                A_CTRL, A_IRQEN, A_IRQST, A_LOCK, A_GAMMA, A_CT, A_CL0, A_CL0+1,
                A_CL0+2, A_CL0+3, A_HYST, A_FANMIN, A_TEMPSW, A_KP, A_KI, A_RPMMAX,
                A_GATE, A_MULT, A_STALL, A_WDOGKICK, A_FCLR, A_TCLR: w_valid = 1'b1;
                A_PRESC, A_PERIOD, A_DEAD, A_POL, A_FLVL, A_TCRIT, A_FAULTEN,
                A_STALETO, A_WDOGTO, A_WDOGCTL: begin w_valid = 1'b1; w_lockable = 1'b1; end
                default: begin
                    if (wi >= A_DUTY0  && wi < A_DUTY0  + NCH) w_valid = 1'b1;
                    if (wi >= A_PHASE0 && wi < A_PHASE0 + NCH) w_valid = 1'b1;
                end
            endcase
        end
    end

    wire [7:0] lock_key = wr_data[7:0];
    wire lock_ok  = wr_strb[0] && (lock_key == 8'hA5 || lock_key == 8'h5A);
    always @* begin
        wr_err = ~w_valid | (w_lockable & r_locked);
        if (wi == A_LOCK && w_valid) wr_err = ~lock_ok;
    end

    // ---------------- IRQ events ----------------
    reg [7:0] flt_q, tcnt_q, pcnt_q;
    wire [2:0] irq_set = { (st_pcnt != pcnt_q), (st_tcnt != tcnt_q), |(st_flt & ~flt_q) };
    wire [2:0] irq_clr = (wr_en && !wr_err && wi == A_IRQST && wr_strb[0]) ? wr_data[2:0] : 3'b0;
    assign irq = |(r_irqst & r_irqen);

    integer i;
    integer j;

    always @(posedge ACLK or negedge ARESETN) begin
        if (!ARESETN) begin
            r_ctrl <= 7'd0; r_irqen <= 3'd0; r_irqst <= 3'd0; r_locked <= 1'b0;
            r_presc <= 16'd0; r_period <= 16'd1000; r_dead <= 8'd0; r_pol <= 8'd0;
            r_flvl <= 8'h01; r_gamma <= 8'd0;
            for (i = 0; i < 8; i = i + 1) begin r_duty[i] <= 16'h8000; r_phase[i] <= 16'd0; end
            r_ct <= {8'd75, 8'd60, 8'd45, 8'd30};
            r_cl0 <= 16'h4000; r_cl1 <= 16'h8000; r_cl2 <= 16'hC000; r_cl3 <= 16'hFFFF;
            r_hyst <= 8'd2; r_tcrit <= 8'd90; r_tempsw <= 8'd25;
            r_fanmin <= 16'd0; r_kp <= 16'h0200; r_ki <= 16'h0040; r_rpmmax <= 16'd5000;
            r_mult <= 16'd30; r_gate <= 32'd50_000_000; r_stall <= {4'd3, 8'h20};
            r_faulten <= 5'h1F; r_staleto <= 24'd0; r_wdogto <= 24'd0; r_wdogen <= 1'b0;
            r_fclr <= 4'd0; r_kick <= 4'd0; r_tclr <= 4'd0;
            flt_q <= 8'd0; tcnt_q <= 8'd0; pcnt_q <= 8'd0;
        end else begin
            flt_q <= st_flt; tcnt_q <= st_tcnt; pcnt_q <= st_pcnt;
            r_irqst <= (r_irqst & ~irq_clr) | irq_set;

            if (wr_en && !wr_err) begin
                case (wi)
                    A_CTRL:    r_ctrl   <= mrg({25'd0, r_ctrl}, wr_data, wr_strb);
                    A_IRQEN:   r_irqen  <= mrg({29'd0, r_irqen}, wr_data, wr_strb);
                    A_LOCK:    r_locked <= (lock_key == 8'hA5);
                    A_PRESC:   r_presc  <= mrg({16'd0, r_presc}, wr_data, wr_strb);
                    A_PERIOD:  r_period <= mrg({16'd0, r_period}, wr_data, wr_strb);
                    A_DEAD:    r_dead   <= mrg({24'd0, r_dead}, wr_data, wr_strb);
                    A_POL:     r_pol    <= mrg({24'd0, r_pol}, wr_data, wr_strb);
                    A_FLVL:    r_flvl   <= mrg({24'd0, r_flvl}, wr_data, wr_strb);
                    A_GAMMA:   r_gamma  <= mrg({24'd0, r_gamma}, wr_data, wr_strb);
                    A_CT:      r_ct     <= mrg(r_ct, wr_data, wr_strb);
                    A_CL0:     r_cl0    <= mrg({16'd0, r_cl0}, wr_data, wr_strb);
                    A_CL0+1:   r_cl1    <= mrg({16'd0, r_cl1}, wr_data, wr_strb);
                    A_CL0+2:   r_cl2    <= mrg({16'd0, r_cl2}, wr_data, wr_strb);
                    A_CL0+3:   r_cl3    <= mrg({16'd0, r_cl3}, wr_data, wr_strb);
                    A_HYST:    r_hyst   <= mrg({24'd0, r_hyst}, wr_data, wr_strb);
                    A_TCRIT:   r_tcrit  <= mrg({24'd0, r_tcrit}, wr_data, wr_strb);
                    A_FANMIN:  r_fanmin <= mrg({16'd0, r_fanmin}, wr_data, wr_strb);
                    A_TEMPSW:  r_tempsw <= mrg({24'd0, r_tempsw}, wr_data, wr_strb);
                    A_KP:      r_kp     <= mrg({16'd0, r_kp}, wr_data, wr_strb);
                    A_KI:      r_ki     <= mrg({16'd0, r_ki}, wr_data, wr_strb);
                    A_RPMMAX:  r_rpmmax <= mrg({16'd0, r_rpmmax}, wr_data, wr_strb);
                    A_GATE:    r_gate   <= mrg(r_gate, wr_data, wr_strb);
                    A_MULT:    r_mult   <= mrg({16'd0, r_mult}, wr_data, wr_strb);
                    A_STALL:   r_stall  <= mrg({20'd0, r_stall}, wr_data, wr_strb);
                    A_FAULTEN: r_faulten<= mrg({27'd0, r_faulten}, wr_data, wr_strb);
                    A_STALETO: r_staleto<= mrg({8'd0, r_staleto}, wr_data, wr_strb);
                    A_WDOGTO:  r_wdogto <= mrg({8'd0, r_wdogto}, wr_data, wr_strb);
                    A_WDOGCTL: if (wr_strb[0]) r_wdogen <= wr_data[0];
                    A_WDOGKICK: if (wr_strb[0]) r_kick <= r_kick + 4'd1;
                    A_FCLR:    if (wr_strb[0] && wr_data[0]) r_fclr <= r_fclr + 4'd1;
                    A_TCLR:    if (wr_strb[0] && wr_data[0]) r_tclr <= r_tclr + 4'd1;
                    default: begin
                        for (i = 0; i < 8; i = i + 1) begin
                            if (wi == A_DUTY0  + i) r_duty[i]  <= mrg({16'd0, r_duty[i]},  wr_data, wr_strb);
                            if (wi == A_PHASE0 + i) r_phase[i] <= mrg({16'd0, r_phase[i]}, wr_data, wr_strb);
                        end
                    end
                endcase
            end
        end
    end

    // ---------------- read mux ----------------
    wire [5:0] ri = rd_addr[7:2];
    reg [15:0] ch_rd;
    reg        ch_hit;
    always @* begin                         // per-channel DUTY/PHASE read select
        ch_rd = 16'd0; ch_hit = 1'b0;
        for (j = 0; j < 8; j = j + 1) begin
            if (ri == A_DUTY0  + j && j < NCH) begin ch_rd = r_duty[j];  ch_hit = 1'b1; end
            if (ri == A_PHASE0 + j && j < NCH) begin ch_rd = r_phase[j]; ch_hit = 1'b1; end
        end
    end

    always @* begin
        rd_data = 32'd0; rd_err = 1'b0;
        if (rd_addr[1:0] != 2'b00) rd_err = 1'b1;
        else case (ri)
            A_ID:      rd_data = ID_VALUE;
            A_CTRL:    rd_data = {25'd0, r_ctrl};
            A_STATUS:  rd_data = {29'd0, irq, r_locked, st_safe};
            A_IRQEN:   rd_data = {29'd0, r_irqen};
            A_IRQST:   rd_data = {29'd0, r_irqst};
            A_LOCK:    rd_data = {31'd0, r_locked};
            A_PRESC:   rd_data = {16'd0, r_presc};
            A_PERIOD:  rd_data = {16'd0, r_period};
            A_DEAD:    rd_data = {24'd0, r_dead};
            A_POL:     rd_data = {24'd0, r_pol};
            A_FLVL:    rd_data = {24'd0, r_flvl};
            A_GAMMA:   rd_data = {24'd0, r_gamma};
            A_CT:      rd_data = r_ct;
            A_CL0:     rd_data = {16'd0, r_cl0};
            A_CL0+1:   rd_data = {16'd0, r_cl1};
            A_CL0+2:   rd_data = {16'd0, r_cl2};
            A_CL0+3:   rd_data = {16'd0, r_cl3};
            A_HYST:    rd_data = {24'd0, r_hyst};
            A_TCRIT:   rd_data = {24'd0, r_tcrit};
            A_FANMIN:  rd_data = {16'd0, r_fanmin};
            A_TEMPSW:  rd_data = {24'd0, r_tempsw};
            A_TEMPNOW: rd_data = {24'd0, st_temp};
            A_KP:      rd_data = {16'd0, r_kp};
            A_KI:      rd_data = {16'd0, r_ki};
            A_RPMMAX:  rd_data = {16'd0, r_rpmmax};
            A_GATE:    rd_data = r_gate;
            A_MULT:    rd_data = {16'd0, r_mult};
            A_STALL:   rd_data = {20'd0, r_stall};
            A_FAULTEN: rd_data = {27'd0, r_faulten};
            A_STALETO: rd_data = {8'd0, r_staleto};
            A_WDOGTO:  rd_data = {8'd0, r_wdogto};
            A_WDOGCTL: rd_data = {31'd0, r_wdogen};
            A_FSTAT:   rd_data = {24'd0, st_flt};
            A_RPM:     rd_data = {16'd0, st_rpm};
            A_DUTYNOW: rd_data = {16'd0, st_duty};
            A_DMIN:    rd_data = {16'd0, st_dmin};
            A_DMAX:    rd_data = {16'd0, st_dmax};
            A_FCNT:    rd_data = {24'd0, st_fcnt};
            A_WDOGKICK, A_FCLR, A_TCLR: rd_data = 32'd0;
            default: begin
                rd_data = {16'd0, ch_rd};
                rd_err  = ~ch_hit;
            end
        endcase
    end
endmodule

// ===== pwm_core.v =====
// -----------------------------------------------------------------------------
// pwm_core : multi-channel PWM, shared time base (PCLK domain).
//   * 16-bit prescaler, 16-bit period (clamped to >=2), 16-bit FRACTIONAL duty
//     (0x0000 = 0 %, 0xFFFF = exactly 100 %).  Duty is period-independent.
//   * Shadow registers: every parameter is latched only at a frame boundary
//     (or while disabled) -> no runt pulses on live updates.
//   * Edge-aligned (per-channel phase offset) or center-aligned (up/down).
//   * Optional gamma (x^2) per channel; polarity per channel.
//   * Complementary pairs (0,1) (2,3)... with exact dead-time insertion
//     (both outputs low for DEAD PCLK cycles at every transition).
//   * 'safe' forces every pin to its FLVL level (registered output, 1 flop).
//   * External sync input restarts the frame; frame_start pulse = ADC trigger.
// Pin = polarity XOR raw; disabled -> raw=0 (pin sits at its polarity level).
// -----------------------------------------------------------------------------
module pwm_core #(
    parameter NCH = 4
) (
    input  wire             clk,
    input  wire             rstn,
    input  wire             en,
    input  wire             center,
    input  wire             comp_en,
    input  wire [15:0]      prescale,
    input  wire [15:0]      period,
    input  wire [7:0]       dead,
    input  wire [7:0]       pol,
    input  wire [7:0]       flvl,
    input  wire [7:0]       gamma_en,
    input  wire [127:0]     duty_f,
    input  wire [127:0]     phase_f,
    input  wire             safe,
    input  wire             sync_in,
    input  wire             sync_en,
    output reg  [NCH-1:0]   pwm,
    output reg              frame_start
);
    reg  [15:0] psc_cnt, cnt;
    reg         dir, sync_q;
    reg  [15:0] s_presc, s_period;
    reg         s_center, s_comp;
    reg  [7:0]  s_dead, s_pol;

    wire [15:0] per_live = (period < 16'd2) ? 16'd2 : period;
    wire [15:0] last     = s_period - 16'd1;
    wire        tick     = (psc_cnt >= s_presc);
    wire        at_end   = tick & (s_center ? (dir & (cnt == 16'd0)) : (cnt == last));
    wire        load     = ~en | (en & at_end);
    wire        sync_pls = sync_en & sync_in & ~sync_q;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            psc_cnt <= 16'd0; cnt <= 16'd0; dir <= 1'b0; sync_q <= 1'b0;
            frame_start <= 1'b0;
            s_presc <= 16'd0; s_period <= 16'd2; s_center <= 1'b0; s_comp <= 1'b0;
            s_dead <= 8'd0; s_pol <= 8'd0;
        end else begin
            sync_q      <= sync_in;
            frame_start <= en & at_end;
            if (!en || sync_pls) begin
                psc_cnt <= 16'd0; cnt <= 16'd0; dir <= 1'b0;
            end else if (tick) begin
                psc_cnt <= 16'd0;
                if (at_end) begin
                    cnt <= 16'd0; dir <= 1'b0;
                end else if (s_center) begin
                    if (!dir) begin
                        if (cnt == last) dir <= 1'b1; else cnt <= cnt + 16'd1;
                    end else cnt <= cnt - 16'd1;
                end else cnt <= cnt + 16'd1;
            end else begin
                psc_cnt <= psc_cnt + 16'd1;
            end
            if (load) begin
                s_presc <= prescale; s_period <= per_live; s_center <= center;
                s_comp <= comp_en;   s_dead <= dead;       s_pol <= pol;
            end
        end
    end

    // ---------------- per channel compare ----------------
    wire [NCH-1:0] raw;
    genvar n, k;
    generate for (n = 0; n < NCH; n = n + 1) begin : g_ch
        wire [15:0] d    = duty_f[16*n +: 16];
        wire [31:0] dsq  = d * d;
        wire [15:0] gd   = gamma_en[n] ? ((d == 16'hFFFF) ? 16'hFFFF : dsq[31:16]) : d;
        wire [31:0] prod = gd * per_live;
        wire [15:0] dc   = (gd == 16'hFFFF) ? per_live : prod[31:16];
        wire [15:0] ph_i = phase_f[16*n +: 16];
        wire [15:0] ph   = (ph_i >= per_live) ? (per_live - 16'd1) : ph_i;
        reg  [15:0] s_dcnt, s_ph;
        always @(posedge clk or negedge rstn) begin
            if (!rstn) begin s_dcnt <= 16'd0; s_ph <= 16'd0; end
            else if (load) begin s_dcnt <= dc; s_ph <= ph; end
        end
        wire [16:0] sum = {1'b0, cnt} + {1'b0, s_ph};
        wire [16:0] wrp = (sum >= {1'b0, s_period}) ? (sum - {1'b0, s_period}) : sum;
        wire [15:0] cc  = s_center ? cnt : wrp[15:0];
        assign raw[n] = (cc < s_dcnt);
    end endgenerate

    // ---------------- complementary pairs + dead time ----------------
    wire [NCH-1:0] hs, ls;
    generate for (k = 0; k < NCH; k = k + 1) begin : g_pair
        if (k < NCH/2) begin : g_real
            localparam S_HS = 2'd0, S_LS = 2'd1, S_DEAD = 2'd2;
            reg [1:0] st;
            reg [7:0] dtc;
            wire r = raw[2*k];
            always @(posedge clk or negedge rstn) begin
                if (!rstn) begin st <= S_LS; dtc <= 8'd0; end
                else if (!en || !s_comp) begin st <= S_LS; dtc <= 8'd0; end
                else case (st)
                    S_HS:   if (!r) begin
                                if (s_dead == 8'd0) st <= S_LS;
                                else begin st <= S_DEAD; dtc <= s_dead - 8'd1; end
                            end
                    S_LS:   if (r) begin
                                if (s_dead == 8'd0) st <= S_HS;
                                else begin st <= S_DEAD; dtc <= s_dead - 8'd1; end
                            end
                    S_DEAD: if (dtc == 8'd0) st <= r ? S_HS : S_LS;
                            else dtc <= dtc - 8'd1;
                    default: st <= S_LS;
                endcase
            end
            assign hs[k] = (st == S_HS);
            assign ls[k] = (st == S_LS);
        end else begin : g_none
            assign hs[k] = 1'b0;
            assign ls[k] = 1'b0;
        end
    end endgenerate

    // ---------------- output stage ----------------
    wire [NCH-1:0] pre;
    generate for (n = 0; n < NCH; n = n + 1) begin : g_pre
        if ((n % 2 == 1) || (n + 1 < NCH)) begin : g_paired
            assign pre[n] = !en ? 1'b0 :
                            s_comp ? ((n % 2 == 0) ? hs[n/2] : ls[n/2]) : raw[n];
        end else begin : g_single
            assign pre[n] = !en ? 1'b0 : raw[n];
        end
    end endgenerate

    wire [NCH-1:0] pin_d = safe ? flvl[NCH-1:0] : (pre ^ s_pol[NCH-1:0]);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) pwm <= {NCH{1'b0}};
        else       pwm <= pin_d;
    end
endmodule

// ===== tach_capture.v =====
// -----------------------------------------------------------------------------
// tach_capture : counts tach pulses over a programmable gate window.
//   rpm = pulses * mult (saturating 16 bit).   e.g. 1 s gate, 2 pulses/rev
//   -> mult = 30.   'valid' pulses when a window completes.
// -----------------------------------------------------------------------------
module tach_capture (
    input  wire        clk,
    input  wire        rstn,
    input  wire        tach_in,        // asynchronous
    input  wire [31:0] gate,           // window length in PCLK cycles (<2 = off)
    input  wire [15:0] mult,
    output reg  [15:0] rpm,
    output reg         valid,
    output reg         zero_win,       // last window had no pulses
    output reg  [7:0]  win_cnt
);
    wire ts;
    sync_ff #(.STAGES(2)) u_s (.clk(clk), .rstn(rstn), .d(tach_in), .q(ts));
    reg tq;
    wire edge_p = ts & ~tq;

    reg  [31:0] gcnt;
    reg  [15:0] pcnt;
    wire [15:0] pn   = (pcnt == 16'hFFFF) ? pcnt : (pcnt + {15'd0, edge_p});
    wire [31:0] prod = pn * mult;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            tq <= 1'b0; gcnt <= 32'd0; pcnt <= 16'd0; rpm <= 16'd0;
            valid <= 1'b0; zero_win <= 1'b0; win_cnt <= 8'd0;
        end else begin
            tq    <= ts;
            valid <= 1'b0;
            if (gate < 32'd2) begin
                gcnt <= 32'd0; pcnt <= 16'd0;
            end else if (gcnt >= gate - 32'd1) begin
                gcnt     <= 32'd0;
                pcnt     <= 16'd0;
                valid    <= 1'b1;
                zero_win <= (pn == 16'd0);
                rpm      <= (prod[31:16] != 16'd0) ? 16'hFFFF : prod[15:0];
                win_cnt  <= win_cnt + 8'd1;
            end else begin
                gcnt <= gcnt + 32'd1;
                pcnt <= pn;
            end
        end
    end
endmodule

// ===== div_seq.v =====
// -----------------------------------------------------------------------------
// div_seq : restoring sequential divider, NW cycles.  Caller guarantees den!=0.
// -----------------------------------------------------------------------------
module div_seq #(
    parameter NW = 24,
    parameter DW = 8
) (
    input  wire          clk,
    input  wire          rstn,
    input  wire          start,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,
    output reg           busy,
    output reg           done,
    output reg  [NW-1:0] q
);
    reg [NW-1:0] nsh;
    reg [DW-1:0] d, rem;
    reg [5:0]    cntr;
    wire [DW:0]  rs = {rem, nsh[NW-1]};
    wire         ge = (rs >= {1'b0, d});

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            nsh <= {NW{1'b0}}; d <= {DW{1'b0}}; rem <= {DW{1'b0}};
            cntr <= 6'd0; busy <= 1'b0; done <= 1'b0; q <= {NW{1'b0}};
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                nsh <= num; d <= den; rem <= {DW{1'b0}}; q <= {NW{1'b0}};
                cntr <= NW[5:0]; busy <= 1'b1;
            end else if (busy) begin
                nsh  <= {nsh[NW-2:0], 1'b0};
                q    <= {q[NW-2:0], ge};
                rem  <= ge ? (rs[DW-1:0] - d) : rs[DW-1:0];
                cntr <= cntr - 6'd1;
                if (cntr == 6'd1) begin busy <= 1'b0; done <= 1'b1; end
            end
        end
    end
endmodule

// ===== fan_curve.v =====
// -----------------------------------------------------------------------------
// fan_curve : 4-point temperature -> level piecewise-linear table.
//   level = l_i + (t - t_i) * (l_{i+1} - l_i) / (t_{i+1} - t_i)   (sequential
//   divider).  Clamped to l0 below t0 and to l3 above t3.  Re-evaluates
//   continuously (~30 cycles per evaluation).
// -----------------------------------------------------------------------------
module fan_curve (
    input  wire        clk,
    input  wire        rstn,
    input  wire [7:0]  temp,
    input  wire [31:0] ct,
    input  wire [63:0] cl,
    output reg  [15:0] level
);
    wire [7:0]  t0 = ct[7:0],  t1 = ct[15:8], t2 = ct[23:16], t3 = ct[31:24];
    wire [15:0] l0 = cl[15:0], l1 = cl[31:16], l2 = cl[47:32], l3 = cl[63:48];

    reg [2:0]  st;
    reg [7:0]  tl, bt, nt;
    reg [15:0] bl, nl, delta;
    reg        up;
    reg [23:0] dnum;
    reg [7:0]  dden;
    wire       dstart = (st == 3'd2);
    wire       ddone, dbusy;
    wire [23:0] dq;

    div_seq #(.NW(24), .DW(8)) u_div (.clk(clk), .rstn(rstn), .start(dstart),
        .num(dnum), .den(dden), .busy(dbusy), .done(ddone), .q(dq));

    wire [15:0] qc = (dq > {8'd0, delta}) ? delta : dq[15:0];

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st <= 3'd0; tl <= 8'd0; bt <= 8'd0; nt <= 8'd0; bl <= 16'd0; nl <= 16'd0;
            delta <= 16'd0; up <= 1'b1; dnum <= 24'd0; dden <= 8'd1; level <= 16'd0;
        end else begin
            case (st)
                3'd0: begin
                    if (temp <= t0)      level <= l0;
                    else if (temp >= t3) level <= l3;
                    else begin
                        if (temp < t1)      begin bt <= t0; nt <= t1; bl <= l0; nl <= l1; end
                        else if (temp < t2) begin bt <= t1; nt <= t2; bl <= l1; nl <= l2; end
                        else                begin bt <= t2; nt <= t3; bl <= l2; nl <= l3; end
                        tl <= temp;
                        st <= 3'd1;
                    end
                end
                3'd1: begin
                    if (nt <= bt) begin level <= nl; st <= 3'd0; end
                    else begin
                        up    <= (nl >= bl);
                        delta <= (nl >= bl) ? (nl - bl) : (bl - nl);
                        dden  <= nt - bt;
                        st    <= 3'd4;
                    end
                end
                3'd4: begin
                    dnum <= {16'd0, (tl - bt)} * {8'd0, delta};
                    st   <= 3'd2;
                end
                3'd2: st <= 3'd3;               // divider start strobe
                3'd3: if (ddone) begin
                    level <= up ? (bl + qc) : (bl - qc);
                    st    <= 3'd0;
                end
                default: st <= 3'd0;
            endcase
        end
    end
endmodule

// ===== pi_ctrl.v =====
// -----------------------------------------------------------------------------
// pi_ctrl : fixed-point PI controller, update on 'update' pulse.
//   e  = target - meas                      (signed, RPM)
//   u  = Kp*e + I,  I += Ki*e               (Kp,Ki unsigned Q8.8)
//   output clamped to 0..0xFFFF (fractional duty).
//   Anti-windup: conditional integration (I frozen while saturated in the
//   direction of the error) + I clamped to the output range.
//   Bumpless transfer: while !en, I and u track 'preload'.
// -----------------------------------------------------------------------------
module pi_ctrl (
    input  wire        clk,
    input  wire        rstn,
    input  wire        en,
    input  wire        update,
    input  wire [15:0] target,
    input  wire [15:0] meas,
    input  wire [15:0] kp,
    input  wire [15:0] ki,
    input  wire [15:0] preload,
    output reg  [15:0] u
);
    reg  signed [34:0] iacc;
    wire signed [17:0] e    = $signed({2'b00, target}) - $signed({2'b00, meas});
    wire signed [34:0] pm   = e * $signed({1'b0, kp});
    wire signed [34:0] im   = e * $signed({1'b0, ki});
    wire signed [34:0] pt   = pm >>> 8;
    wire signed [34:0] it   = im >>> 8;
    wire signed [34:0] inew = iacc + it;
    wire signed [34:0] uraw = pt + inew;
    wire hi   = (uraw > 35'sd65535);
    wire lo   = (uraw < 35'sd0);
    wire stop = (hi && (e > 18'sd0)) || (lo && (e < 18'sd0));
    wire signed [34:0] iclamp = (inew < 35'sd0) ? 35'sd0 :
                                (inew > 35'sd65535) ? 35'sd65535 : inew;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            iacc <= 35'sd0; u <= 16'd0;
        end else if (!en) begin
            iacc <= $signed({19'd0, preload}); u <= preload;
        end else if (update) begin
            if (!stop) iacc <= iclamp;
            u <= hi ? 16'hFFFF : (lo ? 16'h0000 : uraw[15:0]);
        end
    end
endmodule

// ===== safety_ctrl.v =====
// -----------------------------------------------------------------------------
// safety_ctrl : sticky fault latch, watchdog, stale-sensor timeout.
//   fault bits: [0] external pin  [1] fan stall  [2] stale sensor
//               [3] watchdog      [4] over-temperature
//   A fault is latched only if enabled in fault_en; 'clr' re-evaluates (a cause
//   that is still present re-latches immediately).  'safe' is registered.
// -----------------------------------------------------------------------------
module safety_ctrl (
    input  wire        clk,
    input  wire        rstn,
    input  wire        pin_c,
    input  wire        stall_c,
    input  wire        overtemp_c,
    input  wire        hw_temp,        // stale check only for the hardware sensor
    input  wire        temp_valid,
    input  wire [23:0] stale_to,
    input  wire        wdog_en,
    input  wire [23:0] wdog_to,
    input  wire        kick,
    input  wire [4:0]  fault_en,
    input  wire        clr,
    input  wire        tele_clr,
    output reg  [4:0]  lat,
    output reg         safe,
    output reg  [7:0]  fcnt
);
    reg [23:0] stale_cnt, wd_cnt;
    wire stale_c = hw_temp && (stale_to != 24'd0) && (stale_cnt >= stale_to);
    wire wd_c    = wdog_en && (wdog_to  != 24'd0) && (wd_cnt    >= wdog_to);
    wire [4:0] cause = {overtemp_c, wd_c, stale_c, stall_c, pin_c};
    wire [4:0] nxt   = (clr ? 5'd0 : lat) | (cause & fault_en);
    wire [4:0] newf  = nxt & ~lat;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            lat <= 5'd0; safe <= 1'b0; fcnt <= 8'd0; stale_cnt <= 24'd0; wd_cnt <= 24'd0;
        end else begin
            lat  <= nxt;
            safe <= |nxt;
            if (tele_clr) fcnt <= 8'd0;
            else if ((|newf) && fcnt != 8'hFF) fcnt <= fcnt + 8'd1;

            if (temp_valid || !hw_temp) stale_cnt <= 24'd0;
            else if (stale_cnt != 24'hFFFFFF) stale_cnt <= stale_cnt + 24'd1;

            if (!wdog_en || kick) wd_cnt <= 24'd0;
            else if (wd_cnt != 24'hFFFFFF) wd_cnt <= wd_cnt + 24'd1;
        end
    end
endmodule

// ===== fan_core.v =====
// -----------------------------------------------------------------------------
// fan_core : everything in the PCLK domain - sensor conditioning, fan curve,
//            PI loop, tach, safety, PWM.  Config arrives as one coherent bundle.
//   mode 0 MANUAL      ch0 duty = DUTY0
//   mode 1 CURVE       ch0 duty = curve(temp)               (open loop)
//   mode 2 CLOSED_LOOP target RPM = curve(temp) * RPM_MAX   (PI on tach)
//   mode 3 CL_MANUAL   target RPM = DUTY0 * RPM_MAX         (PI on tach)
//   Channels 1..NCH-1 are always manual (DUTY[n]).
// -----------------------------------------------------------------------------
// Bundle widths needed in module port lists (checked against fan_cfg.vh / fan_stat.vh
// inside the modules; a mismatch breaks elaboration).
`define FAN_CFG_W 641
`define FAN_ST_W  105

module fan_core #(
    parameter NCH = 4
) (
    input  wire                    clk,
    input  wire                    rstn,
    input  wire [`FAN_CFG_W-1:0]   cfg,
    input  wire                    tach_in,
    input  wire [7:0]              temp_in,
    input  wire                    temp_valid,
    input  wire                    fault_in,
    input  wire                    sync_in,
    output wire [NCH-1:0]          pwm,
    output wire                    sync_out,
    output wire [`FAN_ST_W-1:0]    st_bus
);
    // -----------------------------------------------------------------------------
// fan_cfg.vh : SINGLE SOURCE OF TRUTH for the ACLK->PCLK configuration bundle.
// Included inside both fan_regs (packs) and fan_core (unpacks), so the two
// sides cannot disagree on field positions.  CW_x = width, CO_x = LSB offset.
// Per-channel fields are always sized for the maximum of 8 channels.
// -----------------------------------------------------------------------------
/* verilator lint_off UNUSEDPARAM */
localparam CW_CTRL   = 8;    // [0]en [2:1]mode [3]center [4]comp [5]temp_src_sw [6]sync_en
localparam CW_PRESC  = 16;
localparam CW_PERIOD = 16;
localparam CW_DEAD   = 8;
localparam CW_POL    = 8;
localparam CW_FLVL   = 8;
localparam CW_GAMMA  = 8;
localparam CW_DUTY   = 128;  // 8 x 16
localparam CW_PHASE  = 128;  // 8 x 16
localparam CW_CT     = 32;   // 4 x 8   curve temperatures
localparam CW_CL     = 64;   // 4 x 16  curve levels
localparam CW_HYST   = 8;
localparam CW_TCRIT  = 8;
localparam CW_FANMIN = 16;
localparam CW_TEMPSW = 8;
localparam CW_KP     = 16;
localparam CW_KI     = 16;
localparam CW_RPMMAX = 16;
localparam CW_GATE   = 32;
localparam CW_MULT   = 16;
localparam CW_STALL  = 12;   // [7:0] duty threshold (top byte), [11:8] windows
localparam CW_FAULTEN= 8;
localparam CW_STALETO= 24;
localparam CW_WDOGTO = 24;
localparam CW_WDOGEN = 1;
localparam CW_CMD    = 12;   // [3:0] fault_clr_cnt [7:4] wdog_kick_cnt [11:8] tele_clr_cnt

localparam CO_CTRL   = 0;
localparam CO_PRESC  = CO_CTRL   + CW_CTRL;
localparam CO_PERIOD = CO_PRESC  + CW_PRESC;
localparam CO_DEAD   = CO_PERIOD + CW_PERIOD;
localparam CO_POL    = CO_DEAD   + CW_DEAD;
localparam CO_FLVL   = CO_POL    + CW_POL;
localparam CO_GAMMA  = CO_FLVL   + CW_FLVL;
localparam CO_DUTY   = CO_GAMMA  + CW_GAMMA;
localparam CO_PHASE  = CO_DUTY   + CW_DUTY;
localparam CO_CT     = CO_PHASE  + CW_PHASE;
localparam CO_CL     = CO_CT     + CW_CT;
localparam CO_HYST   = CO_CL     + CW_CL;
localparam CO_TCRIT  = CO_HYST   + CW_HYST;
localparam CO_FANMIN = CO_TCRIT  + CW_TCRIT;
localparam CO_TEMPSW = CO_FANMIN + CW_FANMIN;
localparam CO_KP     = CO_TEMPSW + CW_TEMPSW;
localparam CO_KI     = CO_KP     + CW_KP;
localparam CO_RPMMAX = CO_KI     + CW_KI;
localparam CO_GATE   = CO_RPMMAX + CW_RPMMAX;
localparam CO_MULT   = CO_GATE   + CW_GATE;
localparam CO_STALL  = CO_MULT   + CW_MULT;
localparam CO_FAULTEN= CO_STALL  + CW_STALL;
localparam CO_STALETO= CO_FAULTEN+ CW_FAULTEN;
localparam CO_WDOGTO = CO_STALETO+ CW_STALETO;
localparam CO_WDOGEN = CO_WDOGTO + CW_WDOGTO;
localparam CO_CMD    = CO_WDOGEN + CW_WDOGEN;
localparam CFG_W     = CO_CMD    + CW_CMD;
/* verilator lint_on UNUSEDPARAM */

    // -----------------------------------------------------------------------------
// fan_stat.vh : PCLK->ACLK status/telemetry bundle layout (single source of
// truth, included by fan_core (packs) and fan_regs (unpacks)).
// -----------------------------------------------------------------------------
/* verilator lint_off UNUSEDPARAM */
localparam SW_RPM  = 16;
localparam SW_DUTY = 16;
localparam SW_TEMP = 8;
localparam SW_FLT  = 8;
localparam SW_FCNT = 8;
localparam SW_DMIN = 16;
localparam SW_DMAX = 16;
localparam SW_TCNT = 8;    // increments on every tach window completion
localparam SW_PCNT = 8;    // increments on every PWM frame
localparam SW_SAFE = 1;

localparam SO_RPM  = 0;
localparam SO_DUTY = SO_RPM  + SW_RPM;
localparam SO_TEMP = SO_DUTY + SW_DUTY;
localparam SO_FLT  = SO_TEMP + SW_TEMP;
localparam SO_FCNT = SO_FLT  + SW_FLT;
localparam SO_DMIN = SO_FCNT + SW_FCNT;
localparam SO_DMAX = SO_DMIN + SW_DMIN;
localparam SO_TCNT = SO_DMAX + SW_DMAX;
localparam SO_PCNT = SO_TCNT + SW_TCNT;
localparam SO_SAFE = SO_PCNT + SW_PCNT;
localparam ST_W    = SO_SAFE + SW_SAFE;
/* verilator lint_on UNUSEDPARAM */


    generate if (CFG_W != `FAN_CFG_W || ST_W != `FAN_ST_W) begin : g_width_check
        BUNDLE_WIDTH_MISMATCH_update_fan_defs_vh bad();
    end endgenerate

    // ---------------- unpack ----------------
    wire [7:0]  c_ctrl    = cfg[CO_CTRL   +: CW_CTRL];
    wire [15:0] c_presc   = cfg[CO_PRESC  +: CW_PRESC];
    wire [15:0] c_period  = cfg[CO_PERIOD +: CW_PERIOD];
    wire [7:0]  c_dead    = cfg[CO_DEAD   +: CW_DEAD];
    wire [7:0]  c_pol     = cfg[CO_POL    +: CW_POL];
    wire [7:0]  c_flvl    = cfg[CO_FLVL   +: CW_FLVL];
    wire [7:0]  c_gamma   = cfg[CO_GAMMA  +: CW_GAMMA];
    wire [127:0] c_duty   = cfg[CO_DUTY   +: CW_DUTY];
    wire [127:0] c_phase  = cfg[CO_PHASE  +: CW_PHASE];
    wire [31:0] c_ct      = cfg[CO_CT     +: CW_CT];
    wire [63:0] c_cl      = cfg[CO_CL     +: CW_CL];
    wire [7:0]  c_hyst    = cfg[CO_HYST   +: CW_HYST];
    wire [7:0]  c_tcrit   = cfg[CO_TCRIT  +: CW_TCRIT];
    wire [15:0] c_fanmin  = cfg[CO_FANMIN +: CW_FANMIN];
    wire [7:0]  c_tempsw  = cfg[CO_TEMPSW +: CW_TEMPSW];
    wire [15:0] c_kp      = cfg[CO_KP     +: CW_KP];
    wire [15:0] c_ki      = cfg[CO_KI     +: CW_KI];
    wire [15:0] c_rpmmax  = cfg[CO_RPMMAX +: CW_RPMMAX];
    wire [31:0] c_gate    = cfg[CO_GATE   +: CW_GATE];
    wire [15:0] c_mult    = cfg[CO_MULT   +: CW_MULT];
    wire [11:0] c_stall   = cfg[CO_STALL  +: CW_STALL];
    wire [4:0]  c_faulten = cfg[CO_FAULTEN+: 5];
    wire [23:0] c_staleto = cfg[CO_STALETO+: CW_STALETO];
    wire [23:0] c_wdogto  = cfg[CO_WDOGTO +: CW_WDOGTO];
    wire        c_wdogen  = cfg[CO_WDOGEN];
    wire [11:0] c_cmd     = cfg[CO_CMD    +: CW_CMD];

    wire       c_en     = c_ctrl[0];
    wire [1:0] c_mode   = c_ctrl[2:1];
    wire       c_center = c_ctrl[3];
    wire       c_comp   = c_ctrl[4];
    wire       c_tsrc   = c_ctrl[5];     // 1 = software temperature register
    wire       c_syncen = c_ctrl[6];

    // ---------------- command events (counter-change detection) ----------------
    reg [11:0] cmd_q;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) cmd_q <= 12'd0; else cmd_q <= c_cmd;
    end
    wire ev_fclr = (c_cmd[3:0]  != cmd_q[3:0]);
    wire ev_kick = (c_cmd[7:4]  != cmd_q[7:4]);
    wire ev_tclr = (c_cmd[11:8] != cmd_q[11:8]);

    // ---------------- async inputs ----------------
    wire fault_s, sync_s;
    sync_ff #(.STAGES(2)) u_fs (.clk(clk), .rstn(rstn), .d(fault_in), .q(fault_s));
    sync_ff #(.STAGES(2)) u_ss (.clk(clk), .rstn(rstn), .d(sync_in),  .q(sync_s));

    // ---------------- temperature conditioning (deadband hysteresis) ----------------
    reg [7:0] temp_raw, temp_eff;
    reg       t_init;
    wire [7:0] tdiff = (temp_raw > temp_eff) ? (temp_raw - temp_eff) : (temp_eff - temp_raw);
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            temp_raw <= 8'd0; temp_eff <= 8'd0; t_init <= 1'b0;
        end else begin
            if (c_tsrc)          temp_raw <= c_tempsw;
            else if (temp_valid) temp_raw <= temp_in;
            if (c_tsrc || temp_valid) t_init <= 1'b1;
            if (!t_init || tdiff > c_hyst) temp_eff <= temp_raw;
        end
    end

    // ---------------- curve ----------------
    wire [15:0] curve_lvl;
    fan_curve u_curve (.clk(clk), .rstn(rstn), .temp(temp_eff), .ct(c_ct), .cl(c_cl),
                       .level(curve_lvl));

    // ---------------- tach ----------------
    wire [15:0] rpm;
    wire        tach_valid, tach_zero;
    wire [7:0]  tach_wcnt;
    tach_capture u_tach (.clk(clk), .rstn(rstn), .tach_in(tach_in), .gate(c_gate),
        .mult(c_mult), .rpm(rpm), .valid(tach_valid), .zero_win(tach_zero), .win_cnt(tach_wcnt));

    // ---------------- target + PI ----------------
    wire [15:0] duty0_reg  = c_duty[15:0];
    wire [15:0] tgt_lvl    = (c_mode == 2'd3) ? duty0_reg : curve_lvl;
    wire [31:0] tgt_prod   = tgt_lvl * c_rpmmax;
    wire [15:0] tgt_rpm    = (tgt_lvl == 16'hFFFF) ? c_rpmmax : tgt_prod[31:16];
    wire        safe;
    wire        pi_en      = c_en & c_mode[1] & ~safe;
    wire [15:0] pi_u;
    pi_ctrl u_pi (.clk(clk), .rstn(rstn), .en(pi_en), .update(tach_valid), .target(tgt_rpm),
                  .meas(rpm), .kp(c_kp), .ki(c_ki), .preload(tgt_lvl), .u(pi_u));

    wire [15:0] auto_d = (c_mode == 2'd1) ? curve_lvl : pi_u;
    wire [15:0] d0     = (c_mode == 2'd0) ? duty0_reg : ((auto_d < c_fanmin) ? c_fanmin : auto_d);

    // ---------------- stall + over-temp detection ----------------
    reg [3:0] bad_win;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) bad_win <= 4'd0;
        else if (tach_valid) begin
            if (tach_zero && c_en && (d0[15:8] >= c_stall[7:0])) begin
                if (bad_win != 4'hF) bad_win <= bad_win + 4'd1;
            end else bad_win <= 4'd0;
        end
    end
    wire stall_c    = (c_stall[11:8] != 4'd0) && (bad_win >= c_stall[11:8]);
    wire overtemp_c = (c_tcrit != 8'd0) && (temp_eff >= c_tcrit);

    // ---------------- safety ----------------
    wire [4:0] flt;
    wire [7:0] fcnt;
    safety_ctrl u_safe (.clk(clk), .rstn(rstn), .pin_c(fault_s), .stall_c(stall_c),
        .overtemp_c(overtemp_c), .hw_temp(~c_tsrc), .temp_valid(temp_valid),
        .stale_to(c_staleto), .wdog_en(c_wdogen), .wdog_to(c_wdogto), .kick(ev_kick),
        .fault_en(c_faulten), .clr(ev_fclr), .tele_clr(ev_tclr),
        .lat(flt), .safe(safe), .fcnt(fcnt));

    // ---------------- PWM ----------------
    wire fs;
    pwm_core #(.NCH(NCH)) u_pwm (.clk(clk), .rstn(rstn), .en(c_en), .center(c_center),
        .comp_en(c_comp), .prescale(c_presc), .period(c_period), .dead(c_dead),
        .pol(c_pol), .flvl(c_flvl), .gamma_en(c_gamma),
        .duty_f({c_duty[127:16], d0}), .phase_f(c_phase), .safe(safe),
        .sync_in(sync_s), .sync_en(c_syncen), .pwm(pwm), .frame_start(fs));
    assign sync_out = fs;

    // ---------------- telemetry ----------------
    reg [15:0] dmin, dmax;
    reg [7:0]  pcnt;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            dmin <= 16'hFFFF; dmax <= 16'd0; pcnt <= 8'd0;
        end else begin
            if (fs) pcnt <= pcnt + 8'd1;
            if (ev_tclr) begin dmin <= 16'hFFFF; dmax <= 16'd0; end
            else if (c_en) begin
                if (d0 < dmin) dmin <= d0;
                if (d0 > dmax) dmax <= d0;
            end
        end
    end

    assign st_bus[SO_RPM  +: SW_RPM ] = rpm;
    assign st_bus[SO_DUTY +: SW_DUTY] = d0;
    assign st_bus[SO_TEMP +: SW_TEMP] = temp_eff;
    assign st_bus[SO_FLT  +: SW_FLT ] = {3'b000, flt};
    assign st_bus[SO_FCNT +: SW_FCNT] = fcnt;
    assign st_bus[SO_DMIN +: SW_DMIN] = dmin;
    assign st_bus[SO_DMAX +: SW_DMAX] = dmax;
    assign st_bus[SO_TCNT +: SW_TCNT] = tach_wcnt;
    assign st_bus[SO_PCNT +: SW_PCNT] = pcnt;
    assign st_bus[SO_SAFE]            = safe;
endmodule

// ===== fan_ctrl_top.v =====
// -----------------------------------------------------------------------------
// fan_ctrl_top : AXI4-Lite GPU fan / thermal controller IP.
//   ACLK domain : AXI slave + register file + IRQ.
//   PCLK domain : PWM, tach, curve, PI, safety.   (ACLK, PCLK asynchronous)
//   One async reset (RESETN) -> per-domain async-assert / sync-deassert.
// -----------------------------------------------------------------------------
// Bundle widths needed in module port lists (checked against fan_cfg.vh / fan_stat.vh
// inside the modules; a mismatch breaks elaboration).
`define FAN_CFG_W 641
`define FAN_ST_W  105

module fan_ctrl_top #(
    parameter NCH = 4
) (
    input  wire              ACLK,
    input  wire              PCLK,
    input  wire              RESETN,

    input  wire [7:0]        AWADDR,
    input  wire              AWVALID,
    output wire              AWREADY,
    input  wire [31:0]       WDATA,
    input  wire [3:0]        WSTRB,
    input  wire              WVALID,
    output wire              WREADY,
    output wire [1:0]        BRESP,
    output wire              BVALID,
    input  wire              BREADY,
    input  wire [7:0]        ARADDR,
    input  wire              ARVALID,
    output wire              ARREADY,
    output wire [31:0]       RDATA,
    output wire [1:0]        RRESP,
    output wire              RVALID,
    input  wire              RREADY,

    output wire [NCH-1:0]    PWM_OUT,
    output wire              PWM_SYNC_OUT,   // frame-start pulse (ADC trigger)
    output wire              IRQ,
    input  wire              TACH_IN,
    input  wire [7:0]        TEMP_IN,        // synchronous to PCLK
    input  wire              TEMP_VALID,     // 1-cycle strobe, synchronous to PCLK
    input  wire              FAULT_IN,       // async external brake / fault
    input  wire              SYNC_IN         // async external sync
);
    wire arstn, prstn;
    reset_sync u_ars (.clk(ACLK), .arstn(RESETN), .srstn(arstn));
    reset_sync u_prs (.clk(PCLK), .arstn(RESETN), .srstn(prstn));

    wire        wr_en, rd_en, wr_err, rd_err;
    wire [7:0]  wr_addr, rd_addr;
    wire [31:0] wr_data, rd_data;
    wire [3:0]  wr_strb;

    axi4l_slave_if #(.AW(8)) u_axi (
        .ACLK(ACLK), .ARESETN(arstn),
        .AWADDR(AWADDR), .AWVALID(AWVALID), .AWREADY(AWREADY),
        .WDATA(WDATA), .WSTRB(WSTRB), .WVALID(WVALID), .WREADY(WREADY),
        .BRESP(BRESP), .BVALID(BVALID), .BREADY(BREADY),
        .ARADDR(ARADDR), .ARVALID(ARVALID), .ARREADY(ARREADY),
        .RDATA(RDATA), .RRESP(RRESP), .RVALID(RVALID), .RREADY(RREADY),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data), .wr_strb(wr_strb),
        .wr_err(wr_err), .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data), .rd_err(rd_err));

    wire [`FAN_CFG_W-1:0] cfg_a, cfg_p;
    wire [`FAN_ST_W-1:0]  st_p,  st_a;

    fan_regs #(.NCH(NCH)) u_regs (
        .ACLK(ACLK), .ARESETN(arstn),
        .wr_en(wr_en), .wr_addr(wr_addr), .wr_data(wr_data), .wr_strb(wr_strb), .wr_err(wr_err),
        .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data), .rd_err(rd_err),
        .cfg_bus(cfg_a), .st_bus(st_a), .irq(IRQ));

    cdc_bus_hs #(.W(`FAN_CFG_W)) u_cfg_cdc (
        .src_clk(ACLK), .src_rstn(arstn), .src_data(cfg_a),
        .dst_clk(PCLK), .dst_rstn(prstn), .dst_data(cfg_p), .dst_valid());

    cdc_bus_hs #(.W(`FAN_ST_W)) u_st_cdc (
        .src_clk(PCLK), .src_rstn(prstn), .src_data(st_p),
        .dst_clk(ACLK), .dst_rstn(arstn), .dst_data(st_a), .dst_valid());

    fan_core #(.NCH(NCH)) u_core (
        .clk(PCLK), .rstn(prstn), .cfg(cfg_p),
        .tach_in(TACH_IN), .temp_in(TEMP_IN), .temp_valid(TEMP_VALID),
        .fault_in(FAULT_IN), .sync_in(SYNC_IN),
        .pwm(PWM_OUT), .sync_out(PWM_SYNC_OUT), .st_bus(st_p));
endmodule
