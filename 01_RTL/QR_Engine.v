`timescale 1ns/1ps

// square 14b (by shared 4 stages multiplier)
// square_add 21b
// sqrt 21b
// div 20b/12b
// mul_add 32b

//SIFFFFFFFFFFFFFF0000
//        SIIIFFFFFFFF

module QR_Engine (
    i_clk,
    i_rst,
    i_trig,
    i_data,
    o_rd_vld,
    o_last_data,
    o_y_hat,
    o_r 
);

// IO description
input          i_clk;
input          i_rst;
input          i_trig;
input  [ 47:0] i_data;
output         o_rd_vld;
output         o_last_data;
output [159:0] o_y_hat;
output [319:0] o_r;

parameter IDLE          = 3'd0;
// parameter INIT_READ = 3'd1;
parameter PROC1          = 3'd1;
parameter PROC2          = 3'd2;
parameter PROC3          = 3'd3;
parameter PROC4          = 3'd4;

parameter SQRT           = 2'd0;
parameter DIVIDE         = 2'd1;
parameter INNER_PRODUCT  = 2'd2;
parameter CAL_ORTHOGONAL = 2'd3;
// parameter FINAL_STATE    = 3'd4;

parameter SQUARE_WIDTH = 14;
parameter SQRT_WIDTH = 21; // 33 = 4+29 35 = 4+31
parameter DIV_WIDTH1 = 20;
parameter DIV_WIDTH2 = 12;
parameter MUL_WIDTH = 16;

parameter SQRT_CYCLE           = 5'd12;
parameter DIVIDE_CYCLE         = 5'd11;
parameter INNER_PRODUCT_CYCLE  = 5'd19;
parameter CAL_ORTHOGONAL_CYCLE = 5'd19;
reg [2:0] state, state_nxt;
reg [1:0] sub_state, sub_state_nxt;
reg [4:0] count20_r, count20_w; // count 20 Cycle
reg [2:0] count4_r, count4_w; // read sram 4 cycle
reg [3:0] count_re_r, count_re_w; // count 10 completed resource element
reg [3:0] count_re2_r, count_re2_w; // count whether there are two resource elements read 
reg [4:0] count_ctrl_r, count_ctrl_w; 
reg trig_r, trig_w;
reg [31:0] data_r, data_w;
reg rd_vld_r, rd_vld_w;
reg last_data_r, last_data_w;
reg [159:0] y_hat_r, y_hat_w;
reg [319:0] r_r, r_w;
// wire finish_fisrt_read;
//reg finish_proc3, finish_proc2, finish_proc1;
reg finish_sram_read_w, finish_sram_read_r;
reg finish_sqrt, finish_divide, finish_inner, finish_orthogonal;
wire div_en;
wire mul_en;

reg [2*SQUARE_WIDTH-1:0] prev_square;
reg [32*4 - 1:0] prev_square_buffer0_r[0:3];
reg [32*4 - 1:0] prev_square_buffer0_w[0:3];

reg [32*4 - 1:0] y_buffer_r, y_buffer_w;

reg en_sqrt;

// reg  [2*SQUARE_WIDTH-1:0] square_in_r, square_in_w;
wire [2*SQUARE_WIDTH-1:0] square_out_im, square_out_real;
reg  [2*SQUARE_WIDTH-1:0] square_out_im_r, square_out_real_r;
reg  [SQRT_WIDTH-1:0] square_add_w,  square_add_r; //33 bit
wire [2*SQUARE_WIDTH-1:0] square_out_w;
reg  [2*SQUARE_WIDTH-1:0] square_out_r;



reg  [SQRT_WIDTH-1:0]   sqrt_in_w, sqrt_in_r; //33
// reg  [(SQRT_WIDTH+1)/2 -1:0]   ;// (33+1)/2 = 17
wire [(SQRT_WIDTH+1)/2 -1:0] sqrt_out;


reg  [DIV_WIDTH1-1:0] div_in_a;
reg  [DIV_WIDTH2-1:0] div_in_b;
wire [DIV_WIDTH1-1:0] div_out;

reg  [MUL_WIDTH-1:0] mul_in_a1;
reg  [MUL_WIDTH-1:0] mul_in_a2;
reg  [MUL_WIDTH-1:0] mul_in_a3;
reg  [MUL_WIDTH-1:0] mul_in_b1;
reg  [MUL_WIDTH-1:0] mul_in_b2;
reg  [MUL_WIDTH-1:0] mul_in_b3;

reg  [2*MUL_WIDTH-1:0] mul_out1_r;
reg  [2*MUL_WIDTH-1:0] mul_out2_r;
reg  [2*MUL_WIDTH-1:0] mul_out3_r;
wire [2*MUL_WIDTH-1:0] mul_out1_w;
wire [2*MUL_WIDTH-1:0] mul_out2_w;
wire [2*MUL_WIDTH-1:0] mul_out3_w;

reg  [2*MUL_WIDTH-1:0] mul_add1_r;
reg  [2*MUL_WIDTH-1:0] mul_add1_w;
reg  [2*MUL_WIDTH-1:0] mul_add2_r;
reg  [2*MUL_WIDTH-1:0] mul_add2_w;
reg  [2*MUL_WIDTH-1:0] mul_add3_r;
reg  [2*MUL_WIDTH-1:0] mul_add3_w;
reg  [2*MUL_WIDTH-1:0] mul_add4_r;
reg  [2*MUL_WIDTH-1:0] mul_add4_w;
reg  [2*MUL_WIDTH-1:0] mul_add5_r;
reg  [2*MUL_WIDTH-1:0] mul_add5_w;
reg  [2*MUL_WIDTH-1:0] mul_add6_r;
reg  [2*MUL_WIDTH-1:0] mul_add6_w;

reg  [2*MUL_WIDTH-1:0] orthogonal_sub1;
reg  [2*MUL_WIDTH-1:0] orthogonal_sub2;
reg  [2*MUL_WIDTH-1:0] orthogonal_sub3;


//Control signal 
reg sram_read;
//SRAM signal 
reg CEN ;
wire [7:0] Q1, Q2, Q4, Q5 ;
wire [7:0] D1, D2, D4, D5 ;
wire [47:0] sram_r_data;
reg  [2:0] count5_r, count5_w ;
reg [7:0] addr_read_r, addr_read_w, addr_write_r, addr_write_w, addr_ref_r, addr_ref_w ;
wire [7:0] addr ;
reg  WEN ;
//reg  en_square;
//integer 
integer i;
//Module Instantiation
DW_sqrt_pipe #(.width(SQRT_WIDTH), .tc_mode(1'b0), .num_stages(5), .stall_mode(1'b1), 
    .rst_mode(1'b0), .op_iso_mode(0)) 
    SQRT1 (.clk(i_clk), .rst_n(~i_rst),
        .en(en_sqrt), .a(sqrt_in_r), .root(sqrt_out) ); // sqrt_out fix point at 14==> 1 2 14

DW_div_pipe #(.a_width(DIV_WIDTH1), .b_width(DIV_WIDTH2), .tc_mode(1'b1), 
    .rem_mode(1'b0), .num_stages(5), .stall_mode(1'b1), .rst_mode(1'b0),  .op_iso_mode(0)) 
    DIV (.clk(i_clk), .rst_n(~i_rst), .en(div_en),
        .a(div_in_a), .b(div_in_b), .quotient(div_out),
        .remainder(), .divide_by_0() 
        );



Multiplier #(.WIDTH(MUL_WIDTH))
        MUL (.clk(i_clk), .rst(i_rst), .a1(mul_in_a1), .a2(mul_in_a2), .a3(mul_in_a3), .b1(mul_in_b1), .b2(mul_in_b2), .b3(mul_in_b3), 
            .en(mul_en), .product1(mul_out1_w), .product2(mul_out2_w), .product3(mul_out3_w));




sram_256x8 SRAM1(
   .Q(Q1),
   .CLK(i_clk),
   .CEN(CEN),
   .WEN(WEN),
   .A(addr),
   .D(D1)
);

sram_256x8 SRAM2(
   .Q(Q2),
   .CLK(i_clk),
   .CEN(CEN),
   .WEN(WEN),
   .A(addr),
   .D(D2)
);

sram_256x8 SRAM4(
   .Q(Q4),
   .CLK(i_clk),
   .CEN(CEN),
   .WEN(WEN),
   .A(addr),
   .D(D4)
);

sram_256x8 SRAM5(
   .Q(Q5),
   .CLK(i_clk),
   .CEN(CEN),
   .WEN(WEN),
   .A(addr),
   .D(D5)
);


//Continuous Assignment
assign o_rd_vld = rd_vld_r;
assign o_last_data = last_data_r;
assign o_y_hat = y_hat_r;
assign o_r = r_r;
assign sram_r_data = {Q5, Q4, Q2, Q1}; //48bit
assign {D5, D4, D2, D1} = data_r;
assign addr = (WEN)? addr_read_r : addr_write_r;  // read: WEN = 1'b1 
assign div_en = (sub_state == DIVIDE)? 1'b1 : 1'b0;
assign mul_en = (sub_state == INNER_PRODUCT || sub_state == CAL_ORTHOGONAL||sub_state == SQRT)? 1'b1 : 1'b0;

// assign finish_fisrt_read = (count_re2_r != 4'd0) ;
// FSM

always @(*) begin
    sub_state_nxt = sub_state;
    state_nxt = state;
    case (state)
       IDLE :begin
            if (trig_r && ~last_data_r && ((count_re2_r == 4'd0 && count20_r == 5'd15 ) || count_re2_r >= 5'd1)) begin
                state_nxt = PROC1;
            end
            else begin
                state_nxt = state;
            end
       end
       PROC1 :begin
            case (sub_state)
                SQRT:begin
                    if (finish_sqrt) begin
                        sub_state_nxt = DIVIDE;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end 
                DIVIDE:begin
                    if (finish_divide) begin
                        sub_state_nxt = INNER_PRODUCT;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end
                INNER_PRODUCT:begin
                    if (finish_inner) begin
                        sub_state_nxt = CAL_ORTHOGONAL;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end 
                CAL_ORTHOGONAL:begin
                    if (finish_orthogonal) begin
                        state_nxt = PROC2;
                        sub_state_nxt = SQRT;
                    end
                    else begin
                        state_nxt = state;
                        sub_state_nxt = sub_state;
                    end
                end
            endcase
       end
       PROC2 :begin
           case (sub_state)
                SQRT:begin
                    if (finish_sqrt) begin
                        sub_state_nxt = DIVIDE;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end 
                DIVIDE:begin
                    if (finish_divide) begin
                        sub_state_nxt = INNER_PRODUCT;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end
                INNER_PRODUCT:begin
                    if (finish_inner) begin
                        sub_state_nxt = CAL_ORTHOGONAL;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end 
                CAL_ORTHOGONAL:begin
                    if (finish_orthogonal) begin
                        state_nxt = PROC3;
                        sub_state_nxt = SQRT;
                    end
                    else begin
                        state_nxt = state;
                        sub_state_nxt = sub_state;
                    end
                end
            endcase
       end
       PROC3:begin
           case (sub_state)
                SQRT:begin
                    if (finish_sqrt) begin
                        sub_state_nxt = DIVIDE;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end 
                DIVIDE:begin
                    if (finish_divide) begin
                        sub_state_nxt = INNER_PRODUCT;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end
                INNER_PRODUCT:begin
                    if (finish_inner) begin
                        sub_state_nxt = CAL_ORTHOGONAL;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end 
                CAL_ORTHOGONAL:begin
                    if (finish_orthogonal) begin
                        state_nxt = PROC4;
                        sub_state_nxt = SQRT;
                    end
                    else begin
                        state_nxt = state;
                        sub_state_nxt = sub_state;
                    end
                end
            endcase
       end
       PROC4:begin
           case (sub_state)
                SQRT:begin
                    if (finish_sqrt) begin
                        sub_state_nxt = DIVIDE;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end 
                DIVIDE:begin
                    if (finish_divide) begin
                        sub_state_nxt = CAL_ORTHOGONAL;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                    end
                end
                CAL_ORTHOGONAL: begin
                    if (finish_orthogonal) begin
                        sub_state_nxt = SQRT;
                        if(count_re_r == 4'd9) state_nxt = IDLE;
                        else state_nxt = PROC1;
                    end
                    else begin
                        sub_state_nxt = sub_state;
                        state_nxt = state;
                    end
                 end
            endcase
        end
       default:begin
            state_nxt = state;
            sub_state_nxt = sub_state;
       end 
    endcase
end

//Combinational
always @(*) begin
    trig_w = i_trig;
    data_w = {i_data[47-:16], i_data[23-:16]};
end

// Enable
always @(*) begin
    if (sub_state == SQRT) begin
        en_sqrt = 1'b1;
    end
    else begin
        en_sqrt = 1'd0;
    end
end

//Enable
always @(*) begin
    //CEN = 1'd0;
    if (~|count_re2_r && trig_r) begin //re0
        CEN = 1'd1;
        WEN = 1'd1;
    end
    else if(trig_r)begin // re1-9
        CEN = 1'd0;
        WEN = 1'd0;
    end
    else if (state == PROC4 && (sub_state == DIVIDE||sub_state == CAL_ORTHOGONAL)) begin
        CEN = 1'd0;
        WEN = 1'd1;
    end
    else begin
        CEN = 1'd1;
        WEN = 1'd1;
    end
    
end
always @(*) begin
    if ((state == PROC4 && sub_state == CAL_ORTHOGONAL && count5_r == 3'd3) ) begin // power ? area ? tradeoff
        count5_w = 3'd0;
    end
    else if ( count5_r == 3'd4 || last_data_r) begin
        count5_w = 3'd0;
    end
    else if(trig_r || (state == PROC4 && sub_state == CAL_ORTHOGONAL))begin
        count5_w = count5_r + 3'd1;
    end
    else begin
        count5_w = count5_r;
    end
end
always @(*) begin
    if (last_data_r) begin
        addr_ref_w = 8'd0;
    end
    else if (~WEN) begin
        if (count20_r == 5'd19) begin
            addr_ref_w = addr_write_r + 8'd1;
        end
        else if (count5_r == 3'd4) begin
            addr_ref_w = addr_ref_r + 8'd1;
        end
        else begin
            addr_ref_w = addr_ref_r;
        end
    end
    else begin
        addr_ref_w = addr_ref_r;
    end
end
always @(*) begin
    if (last_data_r) begin
        addr_write_w = 8'd0;
    end
    else if (~WEN) begin
        if (count20_r == 5'd19) begin
            addr_write_w = addr_write_r + 8'd1;
        end
        else if (count5_r == 3'd4) begin
            addr_write_w = addr_ref_r + 8'd1;
        end
        else begin
            addr_write_w = addr_write_r + 8'd4;
        end
    end
    else begin
        addr_write_w = addr_write_r;
    end
end
always @(*) begin
    if (state == PROC4 && sub_state == DIVIDE) begin
        finish_sram_read_w = 1'd0;
    end
    else if (state == PROC4 && sub_state == CAL_ORTHOGONAL && count_ctrl_r == 5'd18) begin //so count_ctrl_r == 5'd19, finish_sram_read_r == 1, count20_r == 5'd19
        finish_sram_read_w = 1'd1;
    end
    else begin
        finish_sram_read_w = finish_sram_read_r;
    end
end
always @(*) begin
    if (last_data_r) begin
        addr_read_w = 8'd0;
    end
    else if (count_re_r!=9 && state == PROC4 && (finish_divide|| (sub_state == CAL_ORTHOGONAL&& ~finish_sram_read_r) )) begin     
        addr_read_w = addr_read_r + 8'd1;// the time when count20_r == 5'd0 5'd1 5'd2 5'd3 5'd4 ... 5'd19   sram_r_data is correct
    end
    else begin
        addr_read_w = addr_read_r;
    end
   
end
always @(*) begin  
    if (last_data_r) begin
        count20_w = 5'd0;
    end
    else if (count20_r == 5'd19) begin
        count20_w = 5'd0;
    end
    else if (trig_r == 1'd1) begin
        count20_w = count20_r + 5'd1;
    end
    else begin
        count20_w = count20_r;
    end
end

always @(*) begin  // Count the # of computed resource elements
    if (last_data_r) begin
        count_re_w = 4'd0;
    end
    else if (rd_vld_w) begin
        count_re_w = count_re_r + 4'd1;
    end
    else begin
        count_re_w = count_re_r;
    end
    
end
always @(*) begin
    if (rd_vld_w && count_re_r == 4'd9) begin
        last_data_w = 1'd1;
    end
    else begin
        last_data_w = 1'd0;
    end
end
always @(*) begin
    if (state == PROC4 && finish_orthogonal) begin
        rd_vld_w = 1'd1;
    end
    else begin
        rd_vld_w = 1'd0;
    end
end

always @(*) begin

    if (count_re2_r == 4'd10) begin  // 10 resource elements have been read
        count_re2_w = 4'd0;
    end
    else if(count20_r == 5'd19) begin
        count_re2_w = count_re2_r + 4'd1;
    end
    else begin
        count_re2_w = count_re2_r;
    end
end
always @(*) begin
    if (sub_state == SQRT && count_ctrl_r == SQRT_CYCLE) begin
        finish_sqrt = 1'd1;
    end
    else begin
        finish_sqrt = 1'd0;
    end
end
always @(*) begin
    if (sub_state == DIVIDE && count_ctrl_r == DIVIDE_CYCLE) begin
        finish_divide = 1'd1;
    end
    else begin
        finish_divide = 1'd0;
    end
end

always @(*) begin
    if (sub_state == INNER_PRODUCT && count_ctrl_r == INNER_PRODUCT_CYCLE) begin
        finish_inner = 1'd1;
    end
    else begin
        finish_inner = 1'd0;
    end
end
always @(*) begin
    if (sub_state == CAL_ORTHOGONAL && count_ctrl_r == CAL_ORTHOGONAL_CYCLE) begin
        finish_orthogonal = 1'd1;
    end
    else begin
        finish_orthogonal = 1'd0;
    end
end
always @(*) begin
    if(state != IDLE) begin
        case(sub_state)
            SQRT: begin
                if (finish_sqrt) begin
                    count_ctrl_w = 5'd0;
                end
                else begin
                    count_ctrl_w = count_ctrl_r + 5'd1;
                end
            end
            DIVIDE: begin 
                if (finish_divide) begin
                    count_ctrl_w = 5'd0;
                end
                else begin
                    count_ctrl_w = count_ctrl_r + 5'd1;
                end 
                // if(count_ctrl_r == 5'd11) count_ctrl_w = 5'd0;   // In this time, the last element sent to DIV has computed
                // else count_ctrl_w = count_ctrl_r + 5'd1;
            end
            INNER_PRODUCT: begin
                if (finish_inner) begin
                    count_ctrl_w = 5'd0;
                end
                else begin
                    count_ctrl_w = count_ctrl_r + 5'd1;
                end
            end
            CAL_ORTHOGONAL: begin
                if (finish_orthogonal) begin
                    count_ctrl_w = 5'd0;
                end
                else begin
                    count_ctrl_w = count_ctrl_r + 5'd1;
                end
            end
        endcase
    end
    else count_ctrl_w = 5'd0;
end

always @(*) begin
    if(sub_state == SQRT) begin
        case (state)
            PROC1:begin
                prev_square = {prev_square_buffer0_r[0][31-:SQUARE_WIDTH] , prev_square_buffer0_r[0][15-:SQUARE_WIDTH]};
            end
            PROC2:begin
                prev_square = {prev_square_buffer0_r[1][31-:SQUARE_WIDTH] , prev_square_buffer0_r[1][15-:SQUARE_WIDTH]};
            end
            PROC3:begin
                prev_square = {prev_square_buffer0_r[2][31-:SQUARE_WIDTH] , prev_square_buffer0_r[2][15-:SQUARE_WIDTH]};
            end
            PROC4:begin
                prev_square = {prev_square_buffer0_r[3][31-:SQUARE_WIDTH] , prev_square_buffer0_r[3][15-:SQUARE_WIDTH]};
            end
            default:begin
                prev_square = {2*SQUARE_WIDTH{1'd0}};
            end 
        endcase
    end
    else begin
        prev_square = {2*SQUARE_WIDTH{1'd0}};
    end
        
end

always @(*) begin
    for ( i=0 ;i<4 ;i=i+1 ) begin
        prev_square_buffer0_w[i] = prev_square_buffer0_r[i];
    end
    case (state)
        IDLE:begin
            if (trig_r && count_re2_r == 4'd0) begin // resource element 0
                case (count5_r)
                    3'd0: begin
                        prev_square_buffer0_w[0][127 : 96] = data_r;
                        prev_square_buffer0_w[0][95 : 0]   = prev_square_buffer0_r[0][127:32];
                    end
                    3'd1: begin
                        prev_square_buffer0_w[1][127 : 96] = data_r;
                        prev_square_buffer0_w[1][95 : 0]   = prev_square_buffer0_r[1][127:32];
                    end
                    3'd2: begin
                        prev_square_buffer0_w[2][127 : 96] = data_r;
                        prev_square_buffer0_w[2][95 : 0]   = prev_square_buffer0_r[2][127:32];
                    end
                    3'd3: begin
                        prev_square_buffer0_w[3][127 : 96] = data_r;
                        prev_square_buffer0_w[3][95 : 0]   = prev_square_buffer0_r[3][127:32];
                    end

                    default: begin
                        for ( i=0 ;i<4 ;i=i+1 ) begin
                            prev_square_buffer0_w[i] = prev_square_buffer0_r[i];
                        end
                    end 
                endcase
            end
        end 
        PROC1:begin
            case(sub_state)
                SQRT: begin
                    if(count_ctrl_r < 5'd4) begin
                        prev_square_buffer0_w[0] = {prev_square_buffer0_r[0][31:0], prev_square_buffer0_r[0][127:32]};  // keep h1(0)
                    end
                    
                    if (count_re_r == 4'd0) begin
                        if(count_ctrl_r < 5'd4) begin
                            case (count_ctrl_r[1:0])
                                2'd0: begin
                                    prev_square_buffer0_w[1][127 : 96] = data_r;
                                    prev_square_buffer0_w[1][95 : 0]   = prev_square_buffer0_r[1][127:32];
                                end
                                2'd1: begin
                                    prev_square_buffer0_w[2][127 : 96] = data_r;
                                    prev_square_buffer0_w[2][95 : 0]   = prev_square_buffer0_r[2][127:32];
                                end
                                2'd2: begin
                                    prev_square_buffer0_w[3][127 : 96] = data_r;
                                    prev_square_buffer0_w[3][95 : 0]   = prev_square_buffer0_r[3][127:32];
                                end
    
                                default: begin
                                    for ( i=1 ;i<4 ;i=i+1 ) begin
                                        prev_square_buffer0_w[i] = prev_square_buffer0_r[i];
                                    end
                                end 
                            endcase
                        end
                    end
                end
                DIVIDE: begin                                      // 6 = (DIV_WIDTH1 - DIV_WIDTH2) + 2
                    prev_square_buffer0_w[0] = {div_out[11:0], 4'd0, prev_square_buffer0_r[0][127:16]};  // Sequentially input {H11, H21, H31, H41}, then output e1
                end
                INNER_PRODUCT: begin
                    if(count_ctrl_r[1:0] == 2'b11 && count_ctrl_r < 5'd16) begin  // Has already input all the h so after shifting back to the original h2(0), h3(0), h4(0) and e1, don't shift anymore
                        prev_square_buffer0_w[0] = {prev_square_buffer0_r[0][31:0], prev_square_buffer0_r[0][127:32]};  // e1
                        prev_square_buffer0_w[1] = {prev_square_buffer0_r[1][31:0], prev_square_buffer0_r[1][127:32]};  // h2(0)
                        prev_square_buffer0_w[2] = {prev_square_buffer0_r[2][31:0], prev_square_buffer0_r[2][127:32]};  // h3(0)
                        prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][31:0], prev_square_buffer0_r[3][127:32]};  // h4(0)
                    end
                end
                CAL_ORTHOGONAL: begin
                    if(count_ctrl_r[1:0] == 2'b11 && count_ctrl_r < 5'd16) begin
                        prev_square_buffer0_w[0] = {prev_square_buffer0_r[0][31:0], prev_square_buffer0_r[0][127:32]};  // e1
                    end
                    if(count_ctrl_r > 5'd3) begin  // start to use the mul_out
                        if(count_ctrl_r[0] == 1'd1) begin  //SSSSSSSSSI.FFFFFF + 16'd0 
                                                           //SSSSSSIIII.FFFFFFFFFFFFFFFF
                                                           //
                                                           //siii. * si.
                                                           //ssiiii.
                                                           //    si.
                            prev_square_buffer0_w[1] = {orthogonal_sub1[(2*MUL_WIDTH-5) -:16], prev_square_buffer0_r[1][127:16]};  // save h2(1)
                            prev_square_buffer0_w[2] = {orthogonal_sub2[(2*MUL_WIDTH-5) -:16], prev_square_buffer0_r[2][127:16]};  // save h3(1)
                            prev_square_buffer0_w[3] = {orthogonal_sub3[(2*MUL_WIDTH-5) -:16], prev_square_buffer0_r[3][127:16]};  // save h4(1)
                        end 
                        else begin
                            prev_square_buffer0_w[1] = {prev_square_buffer0_r[1][127:16], orthogonal_sub1[(2*MUL_WIDTH-5) -:16] };  // save h2(1)
                            prev_square_buffer0_w[2] = {prev_square_buffer0_r[2][127:16], orthogonal_sub2[(2*MUL_WIDTH-5) -:16] };  // save h3(1)
                            prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][127:16], orthogonal_sub3[(2*MUL_WIDTH-5) -:16] };  // save h4(1)
                        end
                    end
                end
            endcase
        end
        PROC2: begin
            case(sub_state)
                SQRT: begin
                    if(count_ctrl_r < 5'd4) begin
                        prev_square_buffer0_w[1] = {prev_square_buffer0_r[1][31:0], prev_square_buffer0_r[1][127:32]};  // keep h2(1)
                    end
                end
                DIVIDE: begin
                    prev_square_buffer0_w[1] = {div_out[11:0], 4'd0, prev_square_buffer0_r[1][127:16]};
                end
                INNER_PRODUCT: begin
                    if(count_ctrl_r[1:0] == 2'b11 && count_ctrl_r < 5'd16) begin  
                        prev_square_buffer0_w[0] = {prev_square_buffer0_r[0][31:0], prev_square_buffer0_r[0][127:32]};  // e1
                        prev_square_buffer0_w[1] = {prev_square_buffer0_r[1][31:0], prev_square_buffer0_r[1][127:32]};  // e2
                        prev_square_buffer0_w[2] = {prev_square_buffer0_r[2][31:0], prev_square_buffer0_r[2][127:32]};  // keep h3(1)
                        prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][31:0], prev_square_buffer0_r[3][127:32]};  // keep h4(1)
                    end
                end
                CAL_ORTHOGONAL: begin
                    if(count_ctrl_r[1:0] == 2'b11 && count_ctrl_r < 5'd16) begin
                        prev_square_buffer0_w[1] = {prev_square_buffer0_r[1][31:0], prev_square_buffer0_r[1][127:32]};  // e2
                    end
                    if(count_ctrl_r > 5'd3) begin  // start to use the mul_out
                        if (count_ctrl_r[0] == 1'd1) begin
                            prev_square_buffer0_w[2] = {orthogonal_sub1[(2*MUL_WIDTH-5) -:16], prev_square_buffer0_r[2][127:16]};  // save h3(2)
                            prev_square_buffer0_w[3] = {orthogonal_sub2[(2*MUL_WIDTH-5) -:16], prev_square_buffer0_r[3][127:16]};  // save h4(2)
                        end
                        else begin
                            prev_square_buffer0_w[2] = {prev_square_buffer0_r[2][127:16], orthogonal_sub1[(2*MUL_WIDTH-5) -:16] };  // save h3(2)
                            prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][127:16], orthogonal_sub2[(2*MUL_WIDTH-5) -:16] };  // save h4(2)
                        end
                    end
                end
            endcase
        end
        PROC3: begin
            case(sub_state)
                SQRT: begin
                    if(count_ctrl_r < 5'd4) begin
                        prev_square_buffer0_w[2] = {prev_square_buffer0_r[2][31:0], prev_square_buffer0_r[2][127:32]};  // keep h3(2)
                    end
                end
                DIVIDE: begin
                    prev_square_buffer0_w[2] = {div_out[11:0], 4'd0, prev_square_buffer0_r[2][127:16]};
                end
                INNER_PRODUCT: begin
                    if(count_ctrl_r[1:0] == 2'b11 && count_ctrl_r < 5'd16) begin
                        prev_square_buffer0_w[2] = {prev_square_buffer0_r[2][31:0], prev_square_buffer0_r[2][127:32]};  // keep e3
                        prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][31:0], prev_square_buffer0_r[3][127:32]};  // keep h4(2)
                    end
                end
                CAL_ORTHOGONAL: begin
                    if(count_ctrl_r[1:0] == 2'b11 && count_ctrl_r < 5'd16) begin
                        prev_square_buffer0_w[2] = {prev_square_buffer0_r[2][31:0], prev_square_buffer0_r[2][127:32]};  // e3
                    end
                    if(count_ctrl_r > 5'd3) begin  // start to use the mul_out
                        if(count_ctrl_r[0] == 1'd1) begin
                            prev_square_buffer0_w[3] = {orthogonal_sub1[(2*MUL_WIDTH-5) -:16], prev_square_buffer0_r[3][127:16]};  // save h4(3)
                        end
                        else begin
                            prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][127:16], orthogonal_sub1[(2*MUL_WIDTH-5) -:16] };  // save h4(3)
                        end
                    end
                    
                end
            endcase
        end
        PROC4:begin
            case(sub_state)
                SQRT: begin
                    if(count_ctrl_r < 5'd4) begin
                        prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][31:0], prev_square_buffer0_r[3][127:32]};  // keep h4(3)
                    end
                end
                DIVIDE: begin
                    prev_square_buffer0_w[3] = {div_out[11:0], 4'd0, prev_square_buffer0_r[3][127:16]};
                end
                CAL_ORTHOGONAL: begin
                    if(finish_sram_read_r == 0 && count_ctrl_r[4] == 1'd0) begin  // when finish_sram_read == 1, count20_r == 20 count5_r=0
                        case (count_ctrl_r[3:2])
                              2'd0: begin
                                  prev_square_buffer0_w[0] = {sram_r_data, prev_square_buffer0_r[0][127:32]}; 
                              end
                              2'd1: begin
                                  prev_square_buffer0_w[1] = {sram_r_data, prev_square_buffer0_r[1][127:32]};
                              end
                              2'd2: begin
                                  prev_square_buffer0_w[2] = {sram_r_data, prev_square_buffer0_r[2][127:32]};
                              end
                              2'd3: begin
                                  prev_square_buffer0_w[3] = {sram_r_data, prev_square_buffer0_r[3][127:32]};
                              end
                          endcase
                    end 
                    if(count_ctrl_r[1:0] == 2'b11 && count_ctrl_r < 5'd12) begin
                        prev_square_buffer0_w[3] = {prev_square_buffer0_r[3][31:0], prev_square_buffer0_r[3][127:32]};  // e3
                    end
                end
                default: begin
                     for ( i=0 ;i<4 ;i=i+1 ) begin
                          prev_square_buffer0_w[i] = prev_square_buffer0_r[i];
                     end 
                end
            endcase
        end
        default:begin
            for ( i=0 ;i<4 ;i=i+1 ) begin
                prev_square_buffer0_w[i] = prev_square_buffer0_r[i];
            end
        end 
    endcase
end
always @(*) begin 
    if (trig_r && count_re2_r == 4'd0 && count5_r == 3'd4) begin // resource element 0\
        y_buffer_w = {data_r, y_buffer_r[127:32]};     
    end
    else if (state == PROC4 && sub_state == CAL_ORTHOGONAL && ~count_ctrl_r[4] && &count_ctrl_r[1:0]) begin
        if(count_ctrl_r[3:2] == 2'b10) begin
            y_buffer_w = {y_buffer_r[127-:64], prev_square_buffer0_r[3][63:32], y_buffer_r[63:32]};
        end
        else begin
            y_buffer_w = {y_buffer_r[31:0], y_buffer_r[127:32]};
        end
    end
    else if((sub_state == INNER_PRODUCT || sub_state == CAL_ORTHOGONAL)&& ~count_ctrl_r[4] && &count_ctrl_r[1:0]) begin
        y_buffer_w = {y_buffer_r[31:0], y_buffer_r[127:32]};
    end
    else if(state == PROC4 && sub_state == CAL_ORTHOGONAL && count_ctrl_r[4] == 1'd1 && count_ctrl_r!= 5'd20)begin //count20_r 16 17 18 19 !=20
        y_buffer_w = {sram_r_data, y_buffer_r[127:32]}; 
    end
    else begin
        y_buffer_w = y_buffer_r;
    end
end
always @(*) begin
    if (sub_state == SQRT && count_ctrl_r > 5'd3) begin
         // square_out_real{sx2.44} square_out_im{sx2.44} 1 + 2 + 2(integer) + 28(30)
        //  square_add_w ={3'd0,square_out_im_r[2*SQUARE_WIDTH-1-2 -: (SQRT_WIDTH-1-8)], 6'd0} + {3'd0,square_out_real_r[2*SQUARE_WIDTH-1-2 -: (SQRT_WIDTH-1-8)], 6'd0}; 
        square_add_w ={3'd0,mul_out2_r[2*MUL_WIDTH-1-6 : 8]} + {3'd0,mul_out1_r[2*MUL_WIDTH-1-6 : 8]};  
        // SSSSSSSI.FFFFFFF(total 20) SSSSSSS_SSSSSSS_II.FFFFFF
        // SQRT_WIDTH =  SQRT_WIDTH +  SQRT_WIDTH
        //fixpoint at 28(30)th bit
    end
    else begin
        square_add_w = {{SQRT_WIDTH{1'd0}}};
    end
   
end
always @(*) begin
    if (sub_state == SQRT) begin
        sqrt_in_w = sqrt_in_r + square_add_w; // SQRTWIDTH //fixpoint at 28(30)th bit
    end
    else begin
        sqrt_in_w = {{SQRT_WIDTH{1'd0}}};
    end
    
end

always @(*) begin
    mul_add1_w = mul_add1_r;
    mul_add2_w = mul_add2_r;
    mul_add3_w = mul_add3_r;
    mul_add4_w = mul_add4_r;
    mul_add5_w = mul_add5_r;
    mul_add6_w = mul_add6_r;
    
    if(count_ctrl_r == 5'd0) begin
        mul_add1_w = { {2*MUL_WIDTH{1'd0}} };
        mul_add2_w = { {2*MUL_WIDTH{1'd0}} };
        mul_add3_w = { {2*MUL_WIDTH{1'd0}} };
        mul_add4_w = { {2*MUL_WIDTH{1'd0}} };
        mul_add5_w = { {2*MUL_WIDTH{1'd0}} };
        mul_add6_w = { {2*MUL_WIDTH{1'd0}} };
    end
    else begin
        case(state)
            PROC1: begin
                case(sub_state)
                    INNER_PRODUCT: begin
                        if(count_ctrl_r > 5'd3) begin  // When counter = 5'd4, mul_out has been computed, mul_out has 2b integer, 44b fraction, 
                            if(~count_ctrl_r[1]) begin  //101  110  0111  1000  1001 1010 1011 1100 1101 1110 1111 10000 10001 10010 10011 10100  
                                mul_add1_w = mul_add1_r + mul_out1_r;  // R12 real  mul_out= SSSSSII.FFFFFFFF
                                mul_add2_w = mul_add2_r + mul_out2_r;  // R13 real           SSIIIII.FFFFFFFF
                                mul_add3_w = mul_add3_r + mul_out3_r;  // R14 real
                            end
                            else begin
                                mul_add4_w = mul_add4_r + mul_out1_r;  // R12 real  mul_out= SSSSSII.FFFFFFFF
                                mul_add5_w = mul_add5_r + mul_out2_r;  // R13 real           SSIIIII.FFFFFFFF
                                mul_add6_w = mul_add6_r + mul_out3_r;  // R14 real
                            end
                        end
                        else begin
                            mul_add1_w = mul_add1_r;
                            mul_add2_w = mul_add2_r;
                            mul_add3_w = mul_add3_r;
                            mul_add4_w = mul_add4_r;
                            mul_add5_w = mul_add5_r;
                            mul_add6_w = mul_add6_r;
                        end
                    end
                    default: begin
                        mul_add1_w = mul_add1_r;
                        mul_add2_w = mul_add2_r;
                        mul_add3_w = mul_add3_r;
                        mul_add4_w = mul_add4_r;
                        mul_add5_w = mul_add5_r;
                        mul_add6_w = mul_add6_r;
                    end
                endcase
            end
            PROC2: begin
                case(sub_state)
                    INNER_PRODUCT: begin
                        if(count_ctrl_r > 5'd3) begin  // When counter = 5'd4, mul_out has been computed
                            if(~count_ctrl_r[1]) begin
                                mul_add1_w = mul_add1_r + mul_out1_r;  // R23 real
                                mul_add2_w = mul_add2_r + mul_out2_r;  // R24 real 
                                mul_add3_w = mul_add3_r + mul_out3_r;
                            end
                            else begin
                                mul_add4_w = mul_add4_r + mul_out1_r;  // R23 image
                                mul_add5_w = mul_add5_r + mul_out2_r;  // R24 image
                                mul_add6_w = mul_add6_r + mul_out3_r;
                            end
                        end
                        else begin
                            mul_add1_w = mul_add1_r;
                            mul_add2_w = mul_add2_r;
                            mul_add3_w = mul_add3_r;
                            mul_add4_w = mul_add4_r;
                            mul_add5_w = mul_add5_r;
                            mul_add6_w = mul_add6_r;
                        end
                    end
                    CAL_ORTHOGONAL: begin
                        if(count_ctrl_r > 5'd3) begin  // When counter = 5'd4, mul_out has been computed
                            if(~count_ctrl_r[1]) begin
                                mul_add3_w = mul_add3_r + mul_out3_r;
                            end
                            else begin
                                mul_add6_w = mul_add6_r + mul_out3_r;
                            end
                        end
                    end
                    default: begin
                        mul_add1_w = mul_add1_r;
                        mul_add2_w = mul_add2_r;
                        mul_add3_w = mul_add3_r;
                        mul_add4_w = mul_add4_r;
                        mul_add5_w = mul_add5_r;
                        mul_add6_w = mul_add6_r;
                    end
                endcase  
            end
            PROC3: begin
                case(sub_state)
                    INNER_PRODUCT: begin
                        if(count_ctrl_r > 5'd3) begin  // When counter = 5'd4, mul_out has been computed
                            if(~count_ctrl_r[1]) begin
                                mul_add1_w = mul_add1_r + mul_out1_r;  // R34 real
                                mul_add2_w = mul_add2_r + mul_out2_r;
                            end
                            else begin
                                mul_add4_w = mul_add4_r + mul_out1_r;  // R34 image
                                mul_add5_w = mul_add5_r + mul_out2_r;
                            end
                        end
                        else begin
                            mul_add1_w = mul_add1_r;
                            mul_add2_w = mul_add2_r;
                            mul_add3_w = mul_add3_r;
                            mul_add4_w = mul_add4_r;
                            mul_add5_w = mul_add5_r;
                            mul_add6_w = mul_add6_r;
                        end
                    end
                    default: begin
                        mul_add1_w = mul_add1_r;
                        mul_add2_w = mul_add2_r;
                        mul_add3_w = mul_add3_r;
                        mul_add4_w = mul_add4_r;
                        mul_add5_w = mul_add5_r;
                        mul_add6_w = mul_add6_r;
                    end
                    
                endcase
            end
            PROC4: begin
                if(sub_state == CAL_ORTHOGONAL) begin
                    if(count_ctrl_r > 5'd3) begin  // When counter = 5'd4, mul_out has been computed
                        if(~count_ctrl_r[1]) begin
                            mul_add1_w = mul_add1_r + mul_out1_r;  // R34 real
                        end
                        else begin
                            mul_add4_w = mul_add4_r + mul_out1_r;  // R34 image
                        end
                    end
                    else begin
                        mul_add1_w = mul_add1_r;
                        mul_add2_w = mul_add2_r;
                        mul_add3_w = mul_add3_r;
                        mul_add4_w = mul_add4_r;
                        mul_add5_w = mul_add5_r;
                        mul_add6_w = mul_add6_r;
                    end
                end
            end
        endcase
    end
    
end

reg en_r;
always @(*) begin
    r_w = r_r;
    en_r = 1'd0;
    if(rd_vld_r) begin
        r_w = 320'b0;
        en_r = 1'd1;
    end
    else begin
        case(state)
            PROC1: begin
                case(sub_state)
                    SQRT: begin 
                        if(finish_sqrt) begin
                            r_w[19:0] = {1'd0,sqrt_out,8'd0};  // R11, sqrt is 17=1+2+14 bit fix point at 14, sqrt_width=35 18=1+2+15
                            en_r = 1'd1;
                        end
                    end
                    INNER_PRODUCT: begin
                        if(finish_inner) begin
                            en_r = 1'd1;
                            r_w[39:20]   = mul_add1_w[(2*MUL_WIDTH-1)-:20];  // R12 real  
                            r_w[99:80]   = mul_add2_w[(2*MUL_WIDTH-1)-:20];  // R13 real 
                            r_w[199:180] = mul_add3_w[(2*MUL_WIDTH-1)-:20];  // R14 real
                            r_w[59:40]   = mul_add4_w[(2*MUL_WIDTH-1)-:20];  // R12 image
                            r_w[119:100] = mul_add5_w[(2*MUL_WIDTH-1)-:20];  // R13 image
                            r_w[219:200] = mul_add6_w[(2*MUL_WIDTH-1)-:20];  // R14 image
                        end
                    end
                    default: begin
                        r_w = r_r;
                    end
                endcase
            end
            PROC2: begin
                case(sub_state)
                    SQRT: begin
                        if(finish_sqrt) begin
                            r_w[79:60] = {1'd0,sqrt_out,8'd0};  // R22
                            en_r = 1'd1;
                        end
                    end
                    INNER_PRODUCT: begin
                        if(finish_inner) begin
                            en_r = 1'd1;
                            r_w[139:120] = mul_add1_w[(2*MUL_WIDTH-1)-:20];  // R23 real
                            r_w[239:220] = mul_add2_w[(2*MUL_WIDTH-1)-:20];  // R24 real 
                            r_w[159:140] = mul_add4_w[(2*MUL_WIDTH-1)-:20];  // R23 image
                            r_w[259:240] = mul_add5_w[(2*MUL_WIDTH-1)-:20];  // R24 image
                        end
                    end
                    default: begin
                        r_w = r_r;
                    end
                endcase
            end
            PROC3: begin
                case(sub_state)
                    SQRT: begin
                        if(finish_sqrt) begin
                            r_w[179:160] = {1'd0,sqrt_out,8'd0};  // R33
                            en_r = 1'd1;
                        end
                    end
                    INNER_PRODUCT: begin
                        if(finish_inner) begin
                            en_r = 1'd1;
                            r_w[279:260] = mul_add1_w[(2*MUL_WIDTH-1)-:20];  // R34 real
                            r_w[299:280] = mul_add4_w[(2*MUL_WIDTH-1)-:20];  // R34 image
                        end
                        
                    end
                    default: begin
                        r_w = r_r;
                    end
                endcase
            end
            PROC4: begin
                case(sub_state)
                    SQRT: begin
                        if(finish_sqrt) begin
                            r_w[319:300] = {1'd0,sqrt_out,8'd0};  // R44
                            en_r = 1'd1;
                        end
                    end
                    default: begin
                        r_w = r_r;
                    end
                endcase
            end
            default: begin
                r_w = 320'b0;
            end
        endcase
    end
    
end
reg en_y_hat;
always @(*) begin
  y_hat_w = y_hat_r;
  en_y_hat = 1'b0;
  case(state)
      PROC2: begin
          if(sub_state == INNER_PRODUCT) begin
            if (finish_inner) begin
                y_hat_w[39:0] = {mul_add6_w[(2*MUL_WIDTH-1)-:20], mul_add3_w[(2*MUL_WIDTH-1)-:20]};
                en_y_hat = 1'b1;
            end
          end
          else if(sub_state == CAL_ORTHOGONAL) begin
              if(finish_orthogonal) begin
                  y_hat_w[79:40] = {mul_add6_w[(2*MUL_WIDTH-1)-:20], mul_add3_w[(2*MUL_WIDTH-1)-:20]};
                  en_y_hat = 1'b1;
              end
          end
          else begin
              y_hat_w = y_hat_r;
          end
      end
      PROC3: begin
          if(sub_state == INNER_PRODUCT) begin
              if(finish_inner) begin
                  y_hat_w[119:80] = {mul_add5_w[(2*MUL_WIDTH-1)-:20], mul_add2_w[(2*MUL_WIDTH-1)-:20]};
                  en_y_hat = 1'b1;
              end
          end
      end
      PROC4: begin
          if(sub_state == CAL_ORTHOGONAL) begin
              if(finish_orthogonal) begin
                  y_hat_w[159:120] = {mul_add4_w[(2*MUL_WIDTH-1)-:20], mul_add1_w[(2*MUL_WIDTH-1)-:20]};
                  en_y_hat = 1'b1;
              end
          end
      end
  endcase
end

always @(*) begin
    div_in_a = { {DIV_WIDTH1{1'b0}} };
    div_in_b = { {(DIV_WIDTH2-1){1'b0}}, 1'b1 };
    if(sub_state == DIVIDE) begin
        case(state)
            PROC1: begin                                    // -64 = 11000000
                // div_in_a = {prev_square_buffer0_r[0][23:0], 4'd0};  // H11 =       SI.FFFFFFFFFFFFFFFFFFFFFF
                                                            // in  = SIIIIIII.FFFFFFFFFFFFFFFF  24b
                                                            // r11 =     SIII.FFFFFFFFFFFFFFFF  20b
                                                            // out = SIIIIIII.FFFFFFFFFFFFFFFF => 
                                                            //     =       SI.FFFFFFFFFFFFFFFFFFFFFF
                div_in_a = {prev_square_buffer0_r[0][15:0], 4'd0};
                div_in_b = r_r[19:8];
            end
            PROC2: begin
                div_in_a = {prev_square_buffer0_r[1][15:0], 4'd0};
                div_in_b = r_r[79:68];   // r22
            end
            PROC3: begin
                div_in_a = {prev_square_buffer0_r[2][15:0], 4'd0};
                div_in_b = r_r[179:168]; // r33
            end
            PROC4: begin

                div_in_a = {prev_square_buffer0_r[3][15:0], 4'd0};
                div_in_b = r_r[319:308]; // r44
            end
        endcase
    end

end

always @(*) begin
    orthogonal_sub1 = { {2*MUL_WIDTH{1'd0}} };
    orthogonal_sub2 = { {2*MUL_WIDTH{1'd0}} };
    orthogonal_sub3 = { {2*MUL_WIDTH{1'd0}} };

    case(state)
        PROC1: begin                                      // siii * si
                                                          // ssiiii.
                                                          // ssss
            if(sub_state == CAL_ORTHOGONAL) begin
                orthogonal_sub1 = { {4{prev_square_buffer0_r[1][15]}}, prev_square_buffer0_r[1][15:0], 12'd0} - mul_out1_r; // prev_square_buffer0_r[1][23:0] fix at 22 mul_out1_r fix at 16+22=38
                orthogonal_sub2 = { {4{prev_square_buffer0_r[2][15]}}, prev_square_buffer0_r[2][15:0], 12'd0} - mul_out2_r;
                orthogonal_sub3 = { {4{prev_square_buffer0_r[3][15]}}, prev_square_buffer0_r[3][15:0], 12'd0} - mul_out3_r;
            end
        end
        PROC2: begin
            if(sub_state == CAL_ORTHOGONAL) begin
                orthogonal_sub1 = { {4{prev_square_buffer0_r[2][15]}}, prev_square_buffer0_r[2][15:0], 12'd0} - mul_out1_r;
                orthogonal_sub2 = { {4{prev_square_buffer0_r[3][15]}}, prev_square_buffer0_r[3][15:0], 12'd0} - mul_out2_r;
            end
        end
        PROC3: begin
            if(sub_state == CAL_ORTHOGONAL) begin
                orthogonal_sub1 = { {4{prev_square_buffer0_r[3][15]}}, prev_square_buffer0_r[3][15:0], 12'd0} - mul_out1_r;
            end
        end
        PROC4: begin
        
        end
        default: begin
            orthogonal_sub1 = { {2*MUL_WIDTH{1'd0}} };
            orthogonal_sub2 = { {2*MUL_WIDTH{1'd0}} };
            orthogonal_sub3 = { {2*MUL_WIDTH{1'd0}} };
        end
    endcase
    
end

always @(*) begin
    mul_in_a1 = { {MUL_WIDTH{1'b0}} };
    mul_in_a2 = { {MUL_WIDTH{1'b0}} };
    mul_in_a3 = { {MUL_WIDTH{1'b0}} };
    mul_in_b1 = { {MUL_WIDTH{1'b0}} };
    mul_in_b2 = { {MUL_WIDTH{1'b0}} };
    mul_in_b3 = { {MUL_WIDTH{1'b0}} };
    
    case(state)
        PROC1: begin
            case(sub_state)
                SQRT:begin
                    if (count_ctrl_r < 5'd4) begin
                        //mul_in_a1 = {{2{prev_square[13]}},prev_square[13:0]};
                        
                        mul_in_a1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_a2 = {{2{prev_square[27]}},prev_square[27:14]};
                        mul_in_b1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_b2 = {{2{prev_square[27]}},prev_square[27:14]};
                    end
                end
                INNER_PRODUCT: begin
                    if(count_ctrl_r < 5'd16) begin
                        case(count_ctrl_r[1:0])
                            2'd0: begin
                                mul_in_a1 = prev_square_buffer0_r[1][15-:MUL_WIDTH];  // real part of h2(0)
                                mul_in_a2 = prev_square_buffer0_r[2][15-:MUL_WIDTH];  // h3(0)
                                mul_in_a3 = prev_square_buffer0_r[3][15-:MUL_WIDTH];  // h4(0)
                                mul_in_b1 = prev_square_buffer0_r[0][15-:MUL_WIDTH];  //real part of e1
                                mul_in_b2 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                                mul_in_b3 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                            end
                            2'd1: begin
                                mul_in_a1 = prev_square_buffer0_r[1][31-:MUL_WIDTH];  // image part
                                mul_in_a2 = prev_square_buffer0_r[2][31-:MUL_WIDTH];  
                                mul_in_a3 = prev_square_buffer0_r[3][31-:MUL_WIDTH];
                                mul_in_b1 = prev_square_buffer0_r[0][31-:MUL_WIDTH];  //image part
                                mul_in_b2 = prev_square_buffer0_r[0][31-:MUL_WIDTH];
                                mul_in_b3 = prev_square_buffer0_r[0][31-:MUL_WIDTH];
                            end
                            2'd2: begin
                                mul_in_a1 = prev_square_buffer0_r[1][31-:MUL_WIDTH];  // image part
                                mul_in_a2 = prev_square_buffer0_r[2][31-:MUL_WIDTH];  
                                mul_in_a3 = prev_square_buffer0_r[3][31-:MUL_WIDTH];
                                mul_in_b1 = prev_square_buffer0_r[0][15-:MUL_WIDTH];  // real part
                                mul_in_b2 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                                mul_in_b3 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                            end
                            2'd3: begin
                                mul_in_a1 = prev_square_buffer0_r[1][15-:MUL_WIDTH];  // real part
                                mul_in_a2 = prev_square_buffer0_r[2][15-:MUL_WIDTH];  
                                mul_in_a3 = prev_square_buffer0_r[3][15-:MUL_WIDTH];
                                mul_in_b1 = ~(prev_square_buffer0_r[0][31-:MUL_WIDTH]) + 1'd1;  // image part
                                mul_in_b2 = ~(prev_square_buffer0_r[0][31-:MUL_WIDTH]) + 1'd1;
                                mul_in_b3 = ~(prev_square_buffer0_r[0][31-:MUL_WIDTH]) + 1'd1;
                            end
                        endcase
                    end
                end
                CAL_ORTHOGONAL: begin
                    if(count_ctrl_r < 5'd16) begin //SSSSSSSSSI.FFFFFFFF - SSIIII.FFFFFFFFFF0000
                        case(count_ctrl_r[1:0])   // SSSSSSIIII.FFFFFFFFFFFFFFFF
                                                  // SSSSSSSSSI.FFFFFF + 16'd0
                            2'd0: begin
                                mul_in_a1 = r_r[39-:MUL_WIDTH];  // real part of R12 SIII.FFFFFFF
                                mul_in_a2 = r_r[99-:MUL_WIDTH];  // R13
                                mul_in_a3 = r_r[199-:MUL_WIDTH];  // R14
                                mul_in_b1 = prev_square_buffer0_r[0][15-:MUL_WIDTH];    //real part of e1 = SI.FFFFF
                                mul_in_b2 = prev_square_buffer0_r[0][15-:MUL_WIDTH]; 
                                mul_in_b3 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                            end
                            2'd1: begin
                                mul_in_a1 = r_r[59-:MUL_WIDTH];  // image part
                                mul_in_a2 = r_r[119-:MUL_WIDTH];  
                                mul_in_a3 = r_r[219-:MUL_WIDTH]; 
                                mul_in_b1 = ~(prev_square_buffer0_r[0][31-:MUL_WIDTH]) + 24'd1;  //image part
                                mul_in_b2 = ~(prev_square_buffer0_r[0][31-:MUL_WIDTH]) + 24'd1;
                                mul_in_b3 = ~(prev_square_buffer0_r[0][31-:MUL_WIDTH]) + 24'd1;
                            end
                            2'd2: begin
                                mul_in_a1 = r_r[59-:(MUL_WIDTH)];  // image part
                                mul_in_a2 = r_r[119-:(MUL_WIDTH)];   
                                mul_in_a3 = r_r[219-:(MUL_WIDTH)]; 
                                mul_in_b1 = prev_square_buffer0_r[0][15-:MUL_WIDTH];  // real part
                                mul_in_b2 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                                mul_in_b3 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                            end
                            2'd3: begin
                                mul_in_a1 = r_r[39-:(MUL_WIDTH)];  // real part
                                mul_in_a2 = r_r[99-:(MUL_WIDTH)];   
                                mul_in_a3 = r_r[199-:(MUL_WIDTH)]; 
                                mul_in_b1 = prev_square_buffer0_r[0][31-:MUL_WIDTH];  // image part
                                mul_in_b2 = prev_square_buffer0_r[0][31-:MUL_WIDTH];
                                mul_in_b3 = prev_square_buffer0_r[0][31-:MUL_WIDTH];
                            end
                        endcase
                    end
                end
                default: begin
                    mul_in_a1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a3 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b3 = { {MUL_WIDTH{1'b0}} };
                end
            endcase
        end
        PROC2: begin
            case(sub_state)
                SQRT:begin
                    if (count_ctrl_r < 5'd4) begin
                        mul_in_a1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_a2 = {{2{prev_square[27]}},prev_square[27:14]};
                        mul_in_b1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_b2 = {{2{prev_square[27]}},prev_square[27:14]};
                    end
                end
                INNER_PRODUCT: begin
                    if(count_ctrl_r < 5'd16) begin
                        case(count_ctrl_r[1:0])
                            2'd0: begin
                                mul_in_a1 = prev_square_buffer0_r[2][15-:MUL_WIDTH];  // real part of h3(1)
                                mul_in_a2 = prev_square_buffer0_r[3][15-:MUL_WIDTH];  // h4(1)
                                mul_in_a3 = prev_square_buffer0_r[0][15-:MUL_WIDTH];  // e1*
                                
                                mul_in_b1 = prev_square_buffer0_r[1][15-:MUL_WIDTH];  // real part of e2*
                                mul_in_b2 = prev_square_buffer0_r[1][15-:MUL_WIDTH];  // e2*
                                mul_in_b3 = y_buffer_r[15-:MUL_WIDTH];  // compute y1_hat
                            end
                            2'd1: begin
                                mul_in_a1 = prev_square_buffer0_r[2][31-:MUL_WIDTH];  // image part
                                mul_in_a2 = prev_square_buffer0_r[3][31-:MUL_WIDTH];  
                                mul_in_a3 = prev_square_buffer0_r[0][31-:MUL_WIDTH];
                                
                                mul_in_b1 = (prev_square_buffer0_r[1][31-:MUL_WIDTH]);  //image part
                                mul_in_b2 = (prev_square_buffer0_r[1][31-:MUL_WIDTH]);
                                mul_in_b3 = y_buffer_r[31-:MUL_WIDTH];
                            end
                            2'd2: begin
                                mul_in_a1 = prev_square_buffer0_r[2][31-:MUL_WIDTH];  // image part
                                mul_in_a2 = prev_square_buffer0_r[3][31-:MUL_WIDTH];  
                                mul_in_a3 = ~(prev_square_buffer0_r[0][31-:MUL_WIDTH]) + 24'd1;  // 
                                
                                mul_in_b1 = prev_square_buffer0_r[1][15-:MUL_WIDTH];  // real part
                                mul_in_b2 = prev_square_buffer0_r[1][15-:MUL_WIDTH];
                                mul_in_b3 = y_buffer_r[15-:MUL_WIDTH];
                            end
                            2'd3: begin
                                mul_in_a1 = prev_square_buffer0_r[2][15-:MUL_WIDTH];  // real part
                                mul_in_a2 = prev_square_buffer0_r[3][15-:MUL_WIDTH];  
                                mul_in_a3 = prev_square_buffer0_r[0][15-:MUL_WIDTH];
                                
                                mul_in_b1 = ~(prev_square_buffer0_r[1][31-:MUL_WIDTH])+24'd1;  // image part e2*
                                mul_in_b2 = ~(prev_square_buffer0_r[1][31-:MUL_WIDTH])+24'd1;  // e2*
                                mul_in_b3 = y_buffer_r[31-:MUL_WIDTH];  //y
                            end
                        endcase
                    end
                end
                CAL_ORTHOGONAL: begin
                    if(count_ctrl_r < 5'd16) begin
                        case(count_ctrl_r[1:0])
                            2'd0: begin
                                mul_in_a1 = r_r[139-:(MUL_WIDTH)];  // real part of R23 
                                mul_in_a2 = r_r[239-:(MUL_WIDTH)];  // R24
                                mul_in_a3 = y_buffer_r[15-:MUL_WIDTH];  // y
                                mul_in_b1 = prev_square_buffer0_r[1][15-:MUL_WIDTH];    //real part of e2
                                mul_in_b2 = prev_square_buffer0_r[1][15-:MUL_WIDTH];
                                mul_in_b3 = prev_square_buffer0_r[1][15-:MUL_WIDTH];    // e2*
                            end
                            2'd1: begin
                                mul_in_a1 = r_r[159-:(MUL_WIDTH)];  // image part
                                mul_in_a2 = r_r[259-:(MUL_WIDTH)];  
                                mul_in_a3 = y_buffer_r[31-:MUL_WIDTH];
                                mul_in_b1 = ~(prev_square_buffer0_r[1][31-:MUL_WIDTH]) + 1'd1;  //image part
                                mul_in_b2 = ~(prev_square_buffer0_r[1][31-:MUL_WIDTH]) + 1'd1;
                                mul_in_b3 = prev_square_buffer0_r[1][31-:MUL_WIDTH];
                            end
                            2'd2: begin
                                mul_in_a1 = r_r[159-:(MUL_WIDTH)];  // image part
                                mul_in_a2 = r_r[259-:(MUL_WIDTH)];   
                                mul_in_a3 = y_buffer_r[31-:MUL_WIDTH];
                                mul_in_b1 = prev_square_buffer0_r[1][15-:MUL_WIDTH];  // real part
                                mul_in_b2 = prev_square_buffer0_r[1][15-:MUL_WIDTH];
                                mul_in_b3 = prev_square_buffer0_r[1][15-:MUL_WIDTH];
                            end
                            2'd3: begin
                                mul_in_a1 = r_r[139-:(MUL_WIDTH)];  // real part
                                mul_in_a2 = r_r[239-:(MUL_WIDTH)];   
                                mul_in_a3 = y_buffer_r[15-:MUL_WIDTH];
                                mul_in_b1 = prev_square_buffer0_r[1][31-:MUL_WIDTH];  // image part
                                mul_in_b2 = prev_square_buffer0_r[1][31-:MUL_WIDTH];
                                mul_in_b3 = ~(prev_square_buffer0_r[1][31-:MUL_WIDTH]) + 1'd1;
                            end
                        endcase
                    end
                end
                default: begin
                    mul_in_a1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a3 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b3 = { {MUL_WIDTH{1'b0}} };
                end
            endcase
        end
        PROC3: begin

            case(sub_state)
                SQRT:begin
                    if (count_ctrl_r < 5'd4) begin
                        mul_in_a1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_a2 = {{2{prev_square[27]}},prev_square[27:14]};
                        mul_in_b1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_b2 = {{2{prev_square[27]}},prev_square[27:14]};
                    end
                end
                INNER_PRODUCT: begin
                if(count_ctrl_r < 5'd16) begin
                        case(count_ctrl_r[1:0])
                            2'd0: begin
                                mul_in_a1 = prev_square_buffer0_r[3][15-:MUL_WIDTH];  // real part of h4(2)
                                mul_in_a2 = y_buffer_r[15-:MUL_WIDTH];  // y
                                
                                mul_in_b1 = prev_square_buffer0_r[2][15-:MUL_WIDTH];  // real part of e3*
                                mul_in_b2 = prev_square_buffer0_r[2][15-:MUL_WIDTH];
                            end
                            2'd1: begin
                                mul_in_a1 = prev_square_buffer0_r[3][31-:MUL_WIDTH];  // image part
                                mul_in_a2 = y_buffer_r[31-:MUL_WIDTH]; 
                                
                                mul_in_b1 = prev_square_buffer0_r[2][31-:MUL_WIDTH];  //image part
                                mul_in_b2 = prev_square_buffer0_r[2][31-:MUL_WIDTH];
                            end
                            2'd2: begin
                                mul_in_a1 = prev_square_buffer0_r[3][31-:MUL_WIDTH];  // image part
                                mul_in_a2 = y_buffer_r[31-:MUL_WIDTH];   
                                
                                mul_in_b1 = prev_square_buffer0_r[2][15-:MUL_WIDTH];  // real part
                                mul_in_b2 = prev_square_buffer0_r[2][15-:MUL_WIDTH];
                            end
                            2'd3: begin
                                mul_in_a1 = prev_square_buffer0_r[3][15-:MUL_WIDTH];  // real part
                                mul_in_a2 = y_buffer_r[15-:MUL_WIDTH];  
                                
                                mul_in_b1 = ~(prev_square_buffer0_r[2][31-:MUL_WIDTH]) + 1'd1;  // image part
                                mul_in_b2 = ~(prev_square_buffer0_r[2][31-:MUL_WIDTH]) + 1'd1;
                            end
                        endcase
                    end
                end
                CAL_ORTHOGONAL: begin
                    if(count_ctrl_r < 5'd16) begin
                        case(count_ctrl_r[1:0])
                            2'd0: begin
                                mul_in_a1 = r_r[279-:(MUL_WIDTH)];  // real part of R34
                                mul_in_b1 = prev_square_buffer0_r[2][15-:MUL_WIDTH];    //real part of e3
                            end
                            2'd1: begin
                                mul_in_a1 = r_r[299-:(MUL_WIDTH)];  // image part
                                mul_in_b1 = ~(prev_square_buffer0_r[2][31-:MUL_WIDTH]) + 1'd1;  //image part
                            end
                            2'd2: begin
                                mul_in_a1 = r_r[299-:(MUL_WIDTH)];  // image part
                                mul_in_b1 = prev_square_buffer0_r[2][15-:MUL_WIDTH];  // real part
                            end
                            2'd3: begin
                                mul_in_a1 = r_r[279-:(MUL_WIDTH)];  // real part
                                mul_in_b1 = prev_square_buffer0_r[2][31-:MUL_WIDTH];  // image part
                            end
                        endcase
                    end
                end
                default: begin
                    mul_in_a1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a3 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b3 = { {MUL_WIDTH{1'b0}} };
                end
            endcase
        end
        PROC4: begin

            case(sub_state)
                SQRT:begin
                    if (count_ctrl_r < 5'd4) begin
                        mul_in_a1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_a2 = {{2{prev_square[27]}},prev_square[27:14]};
                        mul_in_b1 = {{2{prev_square[13]}},prev_square[13:0]};
                        mul_in_b2 = {{2{prev_square[27]}},prev_square[27:14]};
                    end
                end
                CAL_ORTHOGONAL: begin
                    if(count_ctrl_r[4] == 1'b0) begin
                        case(count_ctrl_r[1:0])
                            2'd0: begin
                                if(&count_ctrl_r[3:2]) begin
                                    mul_in_a1 = y_buffer_r[47 -:MUL_WIDTH];
                                end
                                else begin
                                    mul_in_a1 = prev_square_buffer0_r[3][15-:MUL_WIDTH];  // real part of e4*
                                end
                                mul_in_b1 = y_buffer_r[15-:MUL_WIDTH];    // real part of y
                            end
                            2'd1: begin
                                if(&count_ctrl_r[3:2]) begin
                                    mul_in_a1 = y_buffer_r[63 -:MUL_WIDTH];
                                end
                                else begin
                                    mul_in_a1 = prev_square_buffer0_r[3][31-:MUL_WIDTH];  // image part
                                end
                                mul_in_b1 = y_buffer_r[31-:MUL_WIDTH];  //image part
                            end 
                            2'd2: begin
                                if(&count_ctrl_r[3:2]) begin
                                    mul_in_a1 = ~(y_buffer_r[63 -:MUL_WIDTH]) + 1'd1;
                                end
                                else begin
                                    mul_in_a1 = ~(prev_square_buffer0_r[3][31-:MUL_WIDTH]) + 1'd1;  // image part
                                end
                                mul_in_b1 = y_buffer_r[15-:MUL_WIDTH];  // real part
                            end
                            2'd3: begin
                                if(&count_ctrl_r[3:2]) begin
                                    mul_in_a1 = y_buffer_r[47 -:MUL_WIDTH];
                                end
                                else begin
                                    mul_in_a1 = prev_square_buffer0_r[3][15-:MUL_WIDTH];  // real part
                                end
                                mul_in_b1 = y_buffer_r[31-:MUL_WIDTH];  // image part
                            end
                        endcase
                    end


                end
                default: begin
                    mul_in_a1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_a3 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b1 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b2 = { {MUL_WIDTH{1'b0}} };
                    mul_in_b3 = { {MUL_WIDTH{1'b0}} };
                end
            endcase
        end
    endcase
end



//Seqeuntial 
always @(posedge i_clk) begin
    if(sub_state != DIVIDE) begin
        mul_add1_r <= mul_add1_w;
        mul_add2_r <= mul_add2_w;
        mul_add3_r <= mul_add3_w;
        mul_add4_r <= mul_add4_w;
        mul_add5_r <= mul_add5_w;
        mul_add6_r <= mul_add6_w;
        mul_out1_r <= mul_out1_w;
        mul_out2_r <= mul_out2_w;
        mul_out3_r <= mul_out3_w;
    end
    
    for ( i=0 ;i<4 ;i=i+1 ) begin
        prev_square_buffer0_r[i]<= prev_square_buffer0_w[i];
    end

end
always @(posedge i_clk or posedge i_rst) begin
    
    if (i_rst) begin
        count20_r <= 5'd0;
        count_re_r<= 4'd0;
        count_re2_r<=4'd0;
        count5_r <= 3'd0;
        count_ctrl_r <= 5'd0;
        state <= 2'd0;
        sub_state<=3'd0;

        addr_read_r<=8'd0;
        addr_write_r<=8'd0;
        addr_ref_r<=8'd0;
        
        trig_r <= 1'b0;
        data_r <= 32'b0;
        rd_vld_r <= 1'b0;
        last_data_r <= 1'b0;
        y_hat_r <= 160'b0;
        r_r <= 320'b0;
        y_buffer_r <= 128'd0; 

        finish_sram_read_r <= 1'd0;
        
        
        // square_in_r <= 0;
        square_out_im_r   <=  {{2*SQUARE_WIDTH{1'd0}}};
        square_out_real_r <=  {{2*SQUARE_WIDTH{1'd0}}};
        square_add_r <= {{SQRT_WIDTH{1'd0}}};
        sqrt_in_r <= {{SQRT_WIDTH{1'd0}}}; //33
        
    end
    else begin
        count20_r <= count20_w;
        count_re_r<= count_re_w;
        count_re2_r<= count_re2_w;
        count5_r <= count5_w;
        count_ctrl_r <= count_ctrl_w;
        state <= state_nxt;
        sub_state<= sub_state_nxt;

        addr_read_r<= addr_read_w;
        addr_write_r<=  addr_write_w;
        addr_ref_r<=addr_ref_w;
        
        trig_r <= trig_w;
        data_r <= data_w;
        rd_vld_r <= rd_vld_w;
        last_data_r <= last_data_w;
        if(en_y_hat) begin
            y_hat_r <= y_hat_w;
        end
        
        if(en_r) begin
            r_r <= r_w;
        end


        y_buffer_r <= y_buffer_w;
        
        finish_sram_read_r <= finish_sram_read_w;
        

        
        // square_in_r <= square_in_w;
        // square_add_r <= square_add_w;
        if(sub_state == SQRT) begin
            square_out_im_r   <= square_out_im;
            square_out_real_r <= square_out_real;
        end

            
        sqrt_in_r <= sqrt_in_w;
       
    end
end

endmodule



module Multiplier(clk, rst, a1, a2, a3, b1, b2, b3, en, product1, product2, product3);
    parameter WIDTH = 24;
    
    input clk, rst;
    input [WIDTH-1:0] a1, b1;
    input [WIDTH-1:0] a2, b2;
    input [WIDTH-1:0] a3, b3;
    input en;
    output [2*WIDTH-1:0] product1;
    output [2*WIDTH-1:0] product2;
    output [2*WIDTH-1:0] product3;
    
    DW_mult_pipe #(.a_width(WIDTH), .b_width(WIDTH), .num_stages(4), .stall_mode(1'b1), 
        .rst_mode(1'b0), .op_iso_mode(1'b0)) 
        MULT1 (.clk(clk), .rst_n(~rst), .en(en),
            .tc(1'b1), .a(a1), .b(b1), 
            .product(product1) );

    DW_mult_pipe #(.a_width(WIDTH), .b_width(WIDTH), .num_stages(4), .stall_mode(1'b1), 
        .rst_mode(1'b0), .op_iso_mode(1'b0)) 
        MULT2 (.clk(clk), .rst_n(~rst), .en(en),
            .tc(1'b1), .a(a2), .b(b2), 
            .product(product2) );
    
    DW_mult_pipe #(.a_width(WIDTH), .b_width(WIDTH), .num_stages(4), .stall_mode(1'b1),
        .rst_mode(1'b0), .op_iso_mode(1'b0)) 
        MULT3 (.clk(clk), .rst_n(~rst), .en(en),
            .tc(1'b1), .a(a3), .b(b3), 
            .product(product3) );
    
    // always @(*) begin
    //   if(finish) en_w = 1'b0;
    //   else if(valid) en_w = 1'b1;
    //   else en_w = en_r;
    // end
    
    // always @(posedge clk or posedge rst) begin 
    //   if(rst) begin
    //     en_r <= 1'b0;
    //   end
    //   else begin
    //     en_r <= en_w;
    //   end
    // end
    
    
endmodule