`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.06.2025 11:54:03
// Design Name: 
// Module Name: CLIC
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

module CLIC #(
    parameter RV32M = 1,
    parameter RV32E = 0,
    parameter RV32B = 0,
    parameter RV32C = 0,
    parameter NUM_INTERRUPTS = 32
)(
    input clk, resetb,
    input wready,
    output wvalid,
    input [31:0] waddr,
    input [31:0] wdata,
    input [3:0] wstrb,
    input timer_en,
    input rready,
    output rvalid,
    input [31:0] raddr,
    output reg rresp,
    output reg [31:0] rdata,
    output irq_valid,
    output [4:0] irq_id,
    output [7:0] irq_level,
    input clic_irq,// from riscv module 
    input [NUM_INTERRUPTS-1:0] ex_irq// right now from testbench as an external interrupt

);

localparam TIMER_IRQ_ID = 7;
`include "opcode.vh"

integer i;

reg [63:0] mtime;
reg [63:0] mtimecmp;
wire [63:0] mtime_nxt;

reg [31:0] cliccfg;                        // CLIC configuration register
reg [31:0] mintthresh;                     // Threshold for masking interrupts
reg [31:0] clicintie;                      // Interrupt enable bits
reg [31:0] clicintip;                      // Interrupt pending bits
reg [7:0] clicintctl [0:NUM_INTERRUPTS-1]; // Priority/level config for each IRQ
reg [31:0] mnxti;                          // Encoded next interrupt info
reg [31:0] clicintattr;                    // level or edge external interrupt 


assign rvalid = 1'b1;
assign wvalid = 1'b1;
assign mtime_nxt = mtime + 1;

// Timer update logic
/* verilator lint_off MULTIDRIVEN */
// always @(posedge clk or negedge resetb) begin
//     if (timer_en)
//         mtime <= mtime_nxt;
//     else if (mtime >= mtimecmp) 

//         clicintip[TIMER_IRQ_ID] <= 1'b1; // Raise timer interrupt
// end

// Write operations for memory-mapped registers
always @(posedge clk or negedge resetb) begin
    if (!resetb) begin
        for (i = 0; i < NUM_INTERRUPTS; i = i + 1)
            clicintctl[i] <= 8'b0;

        // mtime <= 64'b0;
        mtimecmp <= 64'b0;// on reset it is set to zero 
        clicintie <= 32'b0;
       // clicintip <= 32'b0;
        cliccfg <= 32'b0;
        mintthresh <= 32'b0;
    end else begin
        if (wready) begin
            case (waddr[31:24])
                8'h90: case (waddr)
                     MTIME_BASE:         mtime[31:0]     <= wdata;
                     (MTIME_BASE+4):    mtime[63:32]    <= wdata;
                    MTIMECMP_BASE:      mtimecmp[31:0]  <= wdata;
                    (MTIMECMP_BASE+4):  mtimecmp[63:32] <= wdata;
                    CLICCFG_BASE:       cliccfg         <= wdata;
                endcase

                CLICINTIE_BASE: begin
                    if (wdata[0])
                        clicintie <= clicintie | (32'b1 << waddr[7:3]); // Set enable bit
                    else
                        clicintie <= clicintie & ~(32'b1 << waddr[7:3]); // Clear enable bit
                end

                CLICINTIP_BASE: begin
                    if (wdata[0])
                        clicintip <= clicintip | (32'b1 << waddr[7:3]); // Set pending bit
                    else
                        clicintip <= clicintip & ~(32'b1 << waddr[7:3]); // Clear pending bit
                end

                CLICINTCTL_BASE:
                    clicintctl[waddr[7:3]] <= wdata[7:0]; // Set interrupt level/priority
                 
                CLICINTATTR_BASE: begin
                    if (wdata[0])
                        clicintattr <= clicintattr | (32'b1 << waddr[7:3]); // Set hardware/software bit 
                    else
                        clicintattr <= clicintattr & ~(32'b1 << waddr[7:3]); // Clear hardware/software bit
                end
                   
                 default: mintthresh<=32'b0;
            endcase
        end
    end
end

reg [NUM_INTERRUPTS-1:0] ex_irq_prev;

always @(posedge clk or negedge resetb) begin
    if (!resetb) begin
        ex_irq_prev <= 0;
        clicintip    <= 0;
    end else begin
        for (i = 0; i < NUM_INTERRUPTS; i = i + 1) begin
            if (clicintattr[i] == 1'b0) begin
                // Level-triggered: set IP if signal high
                clicintip[i] <= ex_irq[i];
            end else begin
                // Edge-triggered: set IP on rising edge
                if (~ex_irq_prev[i] && ex_irq[i])
                    clicintip[i] <= 1'b1;
            end
        end
        ex_irq_prev <= ex_irq;
    end
end


// Extract level bits from clicintctl[i]
function automatic [7:0] extract_level;
    input [7:0] ctl;
    input [2:0] nlbits;
    begin
        extract_level = ctl >> (8 - nlbits); // Top `nlbits` are used
    end
endfunction

reg [7:0] best_level;
reg [4:0] best_id;
reg i_found;
reg [7:0] lvl;

// Priority arbitration logic for selecting next IRQ
always @(*) begin
    best_level = 8'd0;
    best_id    = 5'd0;
    i_found    = 1'b0;

    for (i = 0; i < NUM_INTERRUPTS; i = i + 1) begin
        if ((clicintie[i])  && (clicintip[i]) ) begin
            lvl = extract_level(clicintctl[i], cliccfg[2:0]);

            if (lvl > mintthresh[7:0] && (!i_found || lvl > best_level)) begin
                best_level = lvl;
                best_id    = i[4:0];
                i_found    = 1'b1;
            end
        end
    end
end

assign irq_valid = (i_found && !clic_irq); // Assert interrupt if allowed
assign irq_id    = best_id;
assign irq_level = best_level;

// Save vector info to mnxti CSR (read by vector table)
always @(posedge clk) begin
    if (i_found)
        mnxti <= {19'b0, best_level, best_id};
end

// Read logic for memory-mapped registers
always @(posedge clk) begin
    if (rready) begin
        rresp <= 1'b1;
        case (raddr[31:24])
            CLICINTIE_BASE:
                rdata <= (clicintie >> raddr[7:3]) & 32'h00000001;

            CLICINTIP_BASE:
                rdata <= (clicintip >> raddr[7:3]) & 32'h00000001;

            CLICINTCTL_BASE:
                rdata <= {24'b0, clicintctl[raddr[7:3]]};
             
             CLICINTATTR_BASE: 
              rdata <= {31'b0, clicintattr[raddr[7:3]]};

            8'h90: begin
                case (raddr)
                    MNXTI_BASE:
                        rdata <= mnxti;
                endcase
            end

            default:
                rdata <= 32'h0000_0000;
        endcase
    end else begin
        rresp <= 1'b0;
    end
end

endmodule