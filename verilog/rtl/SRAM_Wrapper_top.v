// Design name : SRAM_top

module SRAM_Wrapper_top(
//===================================inputs==============================
VDD,
VSS,
clk,					// Common clock
reset_n,				//wb_rst_i
cntrl_reset, 				//SRAM controller:  LA
wishbone_buffer_rd_en ,		//wbs_we_i=0 		// for instruction memory
wishbone_buffer_wr_en ,		//wbs_we_i=1 		// for instruction memory      
wishbone_buffer_data_in,      		//wbs_dat_i,
//wishbone_rw_addr,			//wbs_adr_i
//select_buff,   			// selects between buffers IM,Input buffer, weight buffer		
Iref0, Iref1, Iref2,Iref3,		// Reference inputs for ADC	
VCLP,
EN,

				
//======================OUTPUTS=========================				
wishbone_databus_out,      		//wbs_dat_o
out_bus_large,
read_sram_data,
flag
);


// Global inputs
input clk;
input reset_n;
//input [1:0] select_buff;
input cntrl_reset;
//Common Inputs for buffers

input wishbone_buffer_rd_en ;			
input wishbone_buffer_wr_en ;
//input [31:0] wishbone_rw_addr;  
input [31:0] wishbone_buffer_data_in; 
input VCLP;
input EN; 			
input Iref0;
input Iref1;
input Iref2;
input Iref3;
inout VDD,VSS;


output wire [31:0] wishbone_databus_out;	// Output signals for Output buffer
output wire [63:0] out_bus_large;		// Output signals for Output buffer
output wire [15:0] read_sram_data;

output reg [31:0] flag ;			// can goto LA
//=============================== Signal Declaration =========================================
reg SRAM_OUTPUT;
reg PRE_SRAM;
reg [15:0] WWL;
reg WE;
reg PRE_VLSA; 
reg PRE_CLSA; 
reg PRE_A;
reg [15:0] RWL; 
reg [15:0] RWLB;
reg SAEN; 
reg data_ready_signal_output; 
reg writing_finished_signal_output; 
reg busy_signal_output;
reg [15:0] Din;
reg [63:0] IMC_out;
reg [15:0]  SA_OUT;
reg halt;
reg address_input;
reg en_dec;
reg imc_en;
reg mem_en;
reg rw;
reg Input_buffer_rd_en;
reg Weight_buffer_rd_en;
reg sram_data_rd_enable;
reg IB_out;
reg Output_buffer_wr_en;

//========================= FLAGS =============================================================
reg wait_IM;
reg IN_buffer_empty ;			// Output for input buffer
reg IN_buffer_full ;
reg [3:0] IN_fifo_cnt;			// Output for input buffer
reg Weight_buffer_empty ;		// Output for weight buffer
reg Weight_buffer_full ;		// Output for weight buffer
reg [3:0] Weight_buffer_fifo_cnt;
reg Output_buffer_empty ;		// Output signals for Output buffer
reg Output_buffer_full;		// Output signals for Output buffer
reg [3:0] Output_buffer_fifo_cnt;
reg SA_buffer_empty;
reg SA_buffer_full;
reg [3:0] SA_buffer_fifo_cnt;
reg state_reg_cntrl;
reg lsb_data_read_en;
reg msb_data_read_en;
reg [15:0]sram_read_data;
//==================================================================================================
always @(posedge clk) begin
	flag<= {IN_buffer_empty, IN_buffer_full, IN_fifo_cnt,  Weight_buffer_empty, Weight_buffer_full, Weight_buffer_fifo_cnt, Output_buffer_empty, Output_buffer_full, Output_buffer_fifo_cnt, SA_buffer_empty, SA_buffer_full,SA_buffer_fifo_cnt, wait_IM,  data_ready_signal_output, writing_finished_signal_output,busy_signal_output,state_reg_cntrl};
	
	if(lsb_data_read_en && ~sram_data_rd_en) begin
		wishbone_databus_out <= out_bus_large[31:0];
	end
	else if (msb_data_read_en && ~sram_data_rd_en)begin
		wishbone_databus_out <= out_bus_large[63:32];
	end
	else if(sram_data_rd_en && ~lsb_data_read_en && ~msb_data_read_en) begin
		wishbone_databus_out[15:0] <= sram_read_data;
	end
	else begin
		wishbone_databus_out <= flag;
	end


end

//======================  MODULES  INSTANTIATION ====================================================


//======================== Instruction Decoder instantiation =========================================

instruction_decoder DUT_decoder(
.clk(clk), 
.reset(reset_n),
.instr(wishbone_buffer_data_in),
.halt(halt),
.address_input(address_input), 
.en_dec(en_dec),
.imc_en(imc_en),
.mem_en(mem_en),
.rw(rw),
.wait_IM(wait_IM),
.Input_buffer_rd_en(Input_buffer_rd_en),
.Weight_buffer_rd_en(Weight_buffer_rd_en),
.sram_data_rd_enable(sram_data_rd_enable),

);

//=========== Instantiation for Input buffer ==========================
sync_fifo_16x16  DUT_sync_fifo_input_buffer (
.clk(clk)  ,
.rst_n(reset_n)  ,
.data_in(wishbone_buffer_data_in[15:0]) ,
.rd_en(Input_buffer_rd_en)  , 			// will come from instruction decoder
.wr_en(wishbone_buffer_wr_en)  , 			// will come from wishbone 
.data_out(IB_out),			 	// will go to IB_out of sram controller
.empty(IN_buffer_empty)    ,
.full(IN_buffer_full),
.fifo_cnt(IN_fifo_cnt)	
);    


//=========== Buffer Instantiation for Weight buffer ========================
sync_fifo_16x16  DUT_sync_fifo_Weight_buffer (
.clk(clk)  ,
.rst_n(reset_n)  ,
.data_in(wishbone_buffer_data_in[15:0]) ,	// wishbone data in
.rd_en(Weight_buffer_rd_en)   , 	       
.wr_en(wishbone_buffer_wr_en)   ,           	// will come from wishbone 
.data_out(Din), 		// will go to SRAM data in 
.empty( Weight_buffer_empty)   ,
.full( Weight_buffer_full),
.fifo_cnt(Weight_buffer_fifo_cnt)
); 


//=========== Buffer Instantiation for SA_OUT buffer ========================
sync_fifo_16x16  DUT_sync_fifo_SA_OUT (
.clk(clk)  ,
.rst_n(reset_n)  ,
.data_in(SA_OUT) ,				// will come form SRAM memory read output
.rd_en(wishbone_buffer_rd_en)   , 	       
.wr_en(sram_data_rd_enable)   ,           	// will come from instruction decoder
.data_out(sram_read_data), 	// will go to wishbone data bus output
.empty( SA_buffer_empty)   ,
.full( SA_buffer_full),
.fifo_cnt(SA_buffer_fifo_cnt)
); 

//=========== Buffer Instantiation for Output buffer ===============
sync_fifo_64x16  DUT_sync_fifo_Output_buffer (
.clk(clk)  ,
.rst_n(reset_n)  ,
.data_in(IMC_out) ,		// will come from IMC_out(array output)
.rd_en(wishbone_buffer_rd_en)    , 		// will come from wishbone read en or instruction decoder
.wr_en(Output_buffer_wr_en)    ,   		// from FSM / Instruction decoder
.data_out(out_bus_large), 	      		//output_buffer_data_out      /// ==================  sochna hai  =========
.empty( Output_buffer_empty)    ,
.full( Output_buffer_full),
.fifo_cnt(Output_buffer_fifo_cnt)
); 

//===========================================================================

//===========SRAM Controller Instantiation  ========================

sram_controller DUT_sram_control(
.clk(clk), 
.reset(cntrl_reset), 
.rw(rw), 
.address_input(address_input), 
.PRE_SRAM(PRE_SRAM),
.WWL(WWL), 
.WE(WE), 
.PRE_VLSA(PRE_VLSA), 
.data_ready_signal_output(data_ready_signal_output), 
.writing_finished_signal_output(writing_finished_signal_output),
.busy_signal_output(busy_signal_output),
.IB_out(IB_out),
.en_dec(en_dec),
.mem_en(mem_en),
.imc_en(imc_en),
.PRE_CLSA(PRE_CLSA),
.PRE_A(PRE_A),
.RWL(RWL),
.RWLB(RWLB),
.SAEN(SAEN), 
.state_reg(state_reg_cntrl),
.halt(halt),
.Output_buffer_wr_en(Output_buffer_wr_en)
);

//============================ SRAM Array Analog part instantiation =======================

Integrated_bitcell_with_dummy_cells  DUT (
.WWL(WWL),
.RWL(RWL),
.RWLB(RWLB),
.Din(Din),
.WE(WE), 
.PRE_SRAM(PRE_SRAM),
.PRE_VLSA(PRE_VLSA), 
.PRE_CLSA(PRE_CLSA),
.PRE_A(PRE_A), 
.SAEN(SAEN),
.VCLP(VCLP),
.EN(EN),
.Iref0(Iref0), 
.Iref1(Iref1), 
.Iref2(Iref2),
.Iref3(Iref3),
.VSS(VSS),
.VDD(VDD),
.ADC0_OUT(IMC_out[3:0]),
.ADC1_OUT(IMC_out[7:4]),
.ADC2_OUT(IMC_out[11:8]),
.ADC3_OUT(IMC_out[15:12]),
.ADC4_OUT(IMC_out[19:16]),
.ADC5_OUT(IMC_out[23:20]),
.ADC6_OUT(IMC_out[27:24]),
.ADC7_OUT(IMC_out[31:28]),
.ADC8_OUT(IMC_out[35:32]),
.ADC9_OUT(IMC_out[39:36]),
.ADC10_OUT(IMC_out[43:40]),
.ADC11_OUT(IMC_out[47:44]),
.ADC12_OUT(IMC_out[51:48]),
.ADC13_OUT(IMC_out[55:52]),
.ADC14_OUT(IMC_out[59:56]),
.ADC15_OUT(IMC_out[63:60]),
.SA_OUT(SA_OUT)
);


//=========================================================================================
endmodule
