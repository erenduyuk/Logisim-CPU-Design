
module add_ins(
    input wire [17:0] input1,
    input wire [17:0] input2,
    output wire [17:0] result
    );
    assign result = input1 + input2;
endmodule

module nor_ins(
    input wire [17:0] a,
    input wire [17:0] b,
    output wire [17:0] out
);
    assign out = ~(a | b);
endmodule

module cmp_ins(
    input wire [17:0] a,
    input wire [17:0] b,
    output wire zero_flag,
    output wire carry_flag
);
    assign zero_flag = (a > b) ? 1'b0 : ((a == b) ? 1'b1 : 1'b0);
    assign carry_flag = (b > a) ? 1'b1 : 1'b0;
endmodule

module and_ins(
    input wire [17:0] a,
    input wire [17:0] b,
    output wire [17:0] out 
    );
    assign out = a & b;
endmodule

module nand_ins(
    input wire [17:0] a,
    input wire [17:0] b,
    output wire [17:0] out
);
    assign out = ~(a & b);
endmodule


module alu(
    input wire [17:0] input1,
    input wire [17:0] input2,
    input wire [3:0] opcode,
    output reg [17:0] result
);
    wire [17:0] add_result, nand_result, and_result, nor_result;

    add_ins add_inst(
        .input1(input1),
        .input2(input2),
        .result(add_result)
    );

    nand_ins nand_inst(
        .a(input1),
        .b(input2),
        .out(nand_result)
    );

    and_ins and_inst(
        .a(input1),
        .b(input2),
        .out(and_result)
    );

    nor_ins nor_inst(
        .a(input1),
        .b(input2),
        .out(nor_result)
    );
    
    always @(*) begin
        case (opcode)
            4'b0000, 4'b0001: 
            begin
                result = add_result;
            end

            4'b0110:
            begin
                result = nand_result;
            end

            4'b0100, 4'b0101:
            begin
                result = and_result;
            end

            4'b0010: 
            begin
                result = nor_result;
            end
            // Define other opcodes here
        endcase
    end
endmodule





module program_counter(
    input wire [9:0] input_address,
    input wire clk,             // Clock signal
    input wire reset,           // Reset signal
    input wire enable,          // Enable signal
    input wire [9:0] increment,  // Increment value
    output wire [9:0] address    // Program counter address output
);
    reg [9:0] pc_val;  // Program counter register

    always @(posedge clk) begin
        pc_val = input_address; 
        if (reset) begin
            pc_val <= 10'b0;  // Reset the counter to 0
        end
        else if (enable & !reset) begin
            pc_val <= pc_val + increment;  // Increment the counter
        end
    end

    assign address = pc_val;  // Assign the counter value to the address output

endmodule



module RAM(
    input wire clk,             // Clock signal
    input wire [9:0] address,    // 10-bit memory address
    input wire [17:0] data_input, // 18-bit data input
    input wire write_enable,     // Write enable signal
    output reg [17:0] data_output // 18-bit data output
);
    reg [17:0] mem [0:1023];  // Memory array (1024 cells)

    always @(posedge clk) begin
        if (write_enable) 
        begin
            mem[address] <= data_input;
        end
        else 
        begin
            data_output <= mem[address];
        end
        // TODO: write'sa outputa bir şey assignlamıyorum ! sınkıntı mı bilmiyorum
    end
endmodule



module ROM(
    input wire [9:0] address,    // 10-bit memory address
    output reg [17:0] data_output // 18-bit data output
);
    reg [17:0] rom [0:1023];  // Memory array (1024 cells)

    always @* begin
        data_output = rom[address];
    end
endmodule


module FSM (
    input wire clk,
    input wire reset,
    input wire [3:0] opcode,
    output reg [3:0] state
    
);
    // State definitions
    parameter INIT_STATE = 4'b0000;
    parameter FETCH_STATE = 4'b0001;
    parameter DECODE_STATE = 4'b0010;
    parameter ALU_STATE = 4'b0011;
    parameter CMP_STATE = 4'b0100;
    parameter LD_STATE = 4'b0101;
    parameter STR_STATE = 4'b0110;
    parameter JUMP_STATE = 4'b0111;
    parameter WRITE_STATE = 4'b1000;

    // Internal signals
    reg [3:0] next_state;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= INIT_STATE;
        end
        else begin
            state <= next_state;
        end
    end

    always @* begin
    // Default next state is current state
    next_state = state;

    // State transitions and actions
    case (state)
        INIT_STATE: 
        begin
            next_state = FETCH_STATE;
        end
        FETCH_STATE: 
        begin
            next_state = DECODE_STATE;
        end
        DECODE_STATE: 
        begin
            case (opcode)
                4'b0000, 4'b0001, 4'b0100, 4'b0101, 4'b0010, 4'b0110 : 
                begin
                    next_state = ALU_STATE;
                end
                4'b0011: 
                begin
                    next_state = CMP_STATE;
                end    
                4'b1000: 
                begin
                    next_state = LD_STATE;
                end
                4'b1001: 
                begin
                    next_state = STR_STATE;
                end
                4'b1010, 4'b1011, 4'b1100, 4'b1101, 4'b1110:
                begin
                    next_state = JUMP_STATE;
                end
                default: next_state = INIT_STATE;
            endcase
        end
        ALU_STATE, CMP_STATE, LD_STATE, STR_STATE:
        begin
            next_state = WRITE_STATE;
        end
        JUMP_STATE:
        begin
            next_state = WRITE_STATE;
            //TODO: jumpsa ve jump enable'sa böyle olmalı ama onu daha eklemedim
        end
        WRITE_STATE: begin
            next_state = INIT_STATE;
        end
        default: next_state = INIT_STATE;
    endcase
end

endmodule



module ControlUnit (
    input wire [0:0] carry_flag ,
    input wire [0:0] zero_flag ,
    input wire [17:0] instruction,
    input wire start,
    input wire reset,
    input wire clk,
    input wire [17:0] R1_data,
    input wire [17:0] R2_data,
    input wire [17:0] R3_data,
    input wire [17:0] Alu_result,
    input wire [17:0] memdata_load,
    output reg [3:0] read_address_1 = 4'b0000,
    output reg [3:0] read_address_2 = 4'b0000,
    output reg [3:0] read_address_3 = 4'b0000,
    output reg [0:0] Reg1_Read_Enable = 1'b0,
    output reg [0:0] Reg2_Read_Enable = 1'b0,
    output reg [0:0] Reg3_Read_Enable = 1'b0,
    output reg [0:0] Memwrite_Enable = 1'b0,
    output reg [0:0] Memread_Enable = 1'b0,
    output reg [17:0] memdata_store = 18'b000000000000000000,
    output reg [0:0] is_imm = 1'b0,
    output reg [5:0] immediate = 6'b000000,
    output reg [17:0] R2_out = 18'b000000000000000000,
    output reg [17:0] R3_out = 18'b000000000000000000,
    output reg [1:0] selectbits = 2'b00,
    output reg [17:0] result = 18'b000000000000000000,
    output reg [9:0] mem_address,
    output reg [0:0] jump_enable = 1'b0,
    output reg [9:0] jump_value = 10'b0000000000,
    output reg [0:0] R1_write_enable = 1'b0,
    output reg [0:0] Flag_write_enable = 1'b0,
    output reg [0:0] Fetch_Signal = 1'b0,
    output reg [0:0] Decode_Signal = 1'b0,
    output reg [0:0] Alu_Signal = 1'b0,
    output reg [0:0] Compare_Signal = 1'b0,
    output reg [0:0] Read_Signal = 1'b0,
    output wire [4:0] state

);
    reg [0:0] Write_Signal = 1'b0;
    reg [0:0] Jump_Signal = 1'b0;
    reg [0:0] JE_Enable = 1'b0;
    reg [0:0] JA_Enable = 1'b0;
    reg [0:0] JB_Enable = 1'b0;
    reg [0:0] JAE_Enable = 1'b0;
    reg [0:0] JBE_Enable = 1'b0;
        


    FSM fsm(
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .state(state)
    );

    wire [9:0] address_w;
    wire [9:0] jump_val_w;

    OpdcodeDecoder opdcodeDecoder(
        .instruction(instruction),
        .opcode(opcode),
        .R1(R1),
        .R2(R2),
        .R3(R3),
        .immediate(immediate),
        .jump_addres_val(jump_addres_val_w),
        .address(address_w),
        .imm_bit(imm_bit),
        .is_LD(is_LD),
        .ADD(ADD),
        .NOR(NOR),
        .AND(AND),
        .NAND(NAND),
        .COMP(COMP),
        .LD(LD),
        .ST(ST),
        .JE(JE),
        .JA(JA),
        .JB(JB),
        .JAE(JAE),
        .JBE(JBE),
        .JUMP(JUMP)
    );
    
    always @* begin
        
        if(state == 4'b0001) begin
            assign Fetch_Signal = 1'b1;
        end
        else if(state == 4'b0010) begin
            assign Decode_Signal = 1'b1;
        end
        else if(state == 4'b0011) begin
            assign Alu_Signal = 1'b1;
        end
        else if(state == 4'b0100) begin
            assign Compare_Signal = 1'b1;
        end
        else if(state == 4'b0101) begin
            assign Read_Signal = 1'b1;
        end
        else if(state ==  4'b1000) begin
            assign Write_Signal = 1'b1;
        end
        else if(state == 4'b0111) begin
            assign Jump_Signal = 1'b1;
        end
    end

    always @* begin
        if (LD == 1'b1) begin
            Memread_Enable = 1'b1;
            Memwrite_Enable = 1'b0; // Eğer LD 1 ise Memwrite_Enable 0 olacak
        end
        else if (ST == 1'b1) begin
            Memread_Enable = 1'b0; // Eğer ST 1 ise Memread_Enable 0 olacak
            Memwrite_Enable = 1'b1;
        end
    end
    
    always @* begin
        if(COMP == 1'b1 | ST == 1'b1)
        begin
            Reg1_Read_Enable = 1'b1;
        end
    end

    always @* begin
        if (ADD == 1'b1  | NOR == 1'b1  | AND == 1'b1  | NAND == 1'b1  | COMP == 1'b1 ) 
        begin
            Reg2_Read_Enable = 1'b1;
        end
    end
    
    always @* begin
        if((ADD == 1'b1  | NOR == 1'b1 | AND == 1'b1 | NAND == 1'b1 ) & is_imm  != 1'b1) begin
            Reg3_Read_Enable = 1'b1;
        end
    end
    
    always @* begin  
        if ((ADD == 1'b1  | NOR == 1'b1  | AND == 1'b1  | NAND == 1'b1  | LD == 1'b1 ) & Write_Signal == 1'b1 ) 
        begin
            R1_write_enable = 1'b1;
        end
    end

    always @* begin
        if(COMP == 1'b1 & Write_Signal == 1'b1 ) 
        begin
            Flag_write_enable = 1'b1;
        end
    end
    
    always @* begin
        if((JE_Enable == 1'b1 | JA_Enable == 1'b1 | JB_Enable == 1'b1 | JAE_Enable == 1'b1 | JBE_Enable == 1'b1 | JUMP == 1'b1) & Jump_Signal == 1'b1) 
        begin
           jump_enable = 1'b1;
           jump_value = jump_val_w;
        end
    end
    always @* begin
        if(JE == 1'b1 & zero_flag == 1'b1 & carry_flag != 1'b1) begin
            JE_Enable = 1'b1;
        end
    end
    always @* begin
        if(JA == 1'b1 & zero_flag != 1'b1 & carry_flag != 1'b1) begin
            JA_Enable = 1'b1;
        end
    end
    always @* begin
        if(JB == 1'b1 & carry_flag == 1'b1 & zero_flag != 1'b1) begin
            JB_Enable = 1'b1;
        end
    end
    always @* begin
        if(JAE == 1'b1 & carry_flag != 1'b1) begin
            JAE_Enable = 1'b1;
        end
    end
    always @* begin
        if(JBE & (carry_flag == 1'b1 | zero_flag == 1'b1)) begin
            JBE_Enable = 1'b1;
        end
    end
    
    always @* begin
        if(ADD == 1'b1) begin
            selectbits = 2'b00;
        end
        else if(NOR == 1'b1) begin
            selectbits = 2'b01;
        end
        else if(AND == 1'b1) begin
            selectbits = 2'b10;
        end
        else if(NAND == 1'b1) begin
            selectbits = 2'b11;
        end
    end
      
    always @* begin
        if(is_LD == 1'b1)
        begin
            result = memdata_load;
        end
        else 
        begin
            result = Alu_result;
        end
    end

    always @* begin
        assign mem_address = address_w; 
    end

    always @* begin
        assign R2_out = R2_data;
        assign R3_out = R3_data;
        assign memdata_store = R1_data;
    end

endmodule



module OpdcodeDecoder(
    input wire [17:0] instruction,
    output wire [3:0] opcode,
    output wire [3:0] R1,
    output wire [3:0] R2,
    output wire [3:0] R3,
    output wire [5:0] immediate,
    output wire [9:0] jump_addres_val,
    output wire [9:0] address,
    output wire [0:0] imm_bit,
    output wire [0:0] is_LD,
    output reg [0:0] ADD = 1'b0,
    output reg [0:0] NOR = 1'b0,
    output reg [0:0] AND = 1'b0,
    output reg [0:0] NAND = 1'b0,
    output reg [0:0] COMP = 1'b0,
    output reg [0:0] LD = 1'b0,
    output reg [0:0] ST = 1'b0,
    output reg [0:0] JE = 1'b0,
    output reg [0:0] JA = 1'b0,
    output reg [0:0] JB = 1'b0,
    output reg [0:0] JAE = 1'b0,
    output reg [0:0] JBE = 1'b0,
    output reg [0:0] JUMP = 1'b0
); 
    assign opcode = instruction[17:14];
    assign R1 = instruction[13:10];
    assign R2 = instruction[9:6];
    assign R3 = instruction[5:2];
    assign immediate = instruction[5:0];
    assign jump_addres_val = instruction[13:4];
    assign address = instruction[9:0];
    assign imm_bit = instruction[14];
    assign is_LD = instruction[17];

always @* begin
    if (opcode == 4'b0000 | opcode == 4'b0001) begin
        ADD = 1'b1;
    end
    else if (opcode == 4'b0010) begin
        NOR = 1'b1;
    end
    else if (opcode == 4'b0011) begin
        COMP = 1'b1;
    end
    else if (opcode == 4'b0100 | opcode == 4'b0101) begin
        AND = 1'b1;
    end
    else if (opcode == 4'b0110) begin
        NAND = 1'b1;
    end
    else if (opcode == 4'b1000) begin
        LD = 1'b1;
    end
    else if (opcode == 4'b1001) begin
        ST = 1'b1;
    end
    else if (opcode == 4'b1010) begin
        JE = 1'b1;
    end
    else if (opcode == 4'b1011) begin
        JA = 1'b1;
    end
    else if (opcode == 4'b1100) begin
        JB = 1'b1;
    end
    else if (opcode == 4'b1101) begin
        JAE = 1'b1;
    end
    else if (opcode == 4'b1110) begin
        JBE = 1'b1;
    end
    else if (opcode == 4'b1111) begin
        JUMP = 1'b1;
    end
end
endmodule



module RegisterFile(
    input wire [3:0] Write_Address,
    input wire [17:0] data_input,
    input wire [0:0] clk,
    input wire [0:0] reset,
    input wire [0:0] Write_Enable,
    input wire [0:0] Read_Enable1,
    input wire [0:0] Read_Enable2,
    input wire [0:0] Read_Enable3,
    input wire [3:0] Read_Address1,
    input wire [3:0] Read_Address2,
    input wire [3:0] Read_Address3,
    output reg [17:0] data_output1,
    output reg [17:0] data_output2,
    output reg [17:0] data_output3
); 
    reg [17:0] regfile [0:15];

    always @(*) begin
        if (Write_Enable) begin
            regfile[Write_Address] = data_input;
        end
        else if (Read_Enable1) begin
            data_output1 = regfile[Read_Address1];
        end
        else if (Read_Enable2) begin
            data_output2 = regfile[Read_Address2];
        end
        else if (Read_Enable3) begin
            data_output3 = regfile[Read_Address3];
        end
    end
endmodule

module CPU(
    wire clock,
    wire reset,
    wire [17:0] instruction
);

    reg [17:0] input1, input2;

    RegisterFile registerFile(
        .Write_Address(Write_Address),
        .data_input(data_input),
        .clk(clk),
        .reset(reset),
        .Write_Enable(Write_Enable),
        .Read_Enable1(Read_Enable1),
        .Read_Enable2(Read_Enable2),
        .Read_Enable3(Read_Enable3),
        .Read_Address1(Read_Address1),
        .Read_Address2(Read_Address2),
        .Read_Address3(Read_Address3),
        .data_output1(data_output1),
        .data_output2(data_output2),
        .data_output3(data_output3)
    );

    ControlUnit controlUnit(
        .carry_flag(carry_flag),
        .zero_flag(zero_flag),
        .instruction(instruction),
        .start(start),
        .reset(reset),
        .clk(clk),
        .R1_data(R1_data),
        .R2_data(R2_data),
        .R3_data(R3_data),
        .Alu_result(Alu_result),
        .memdata_load(memdata_load),
        .read_address_1(read_address_1),
        .read_address_2(read_address_2),
        .read_address_3(read_address_3),
        //.write_address(write_address),
        .Reg1_Read_Enable(Reg1_Read_Enable),
        .Reg2_Read_Enable(Reg2_Read_Enable),
        .Reg3_Read_Enable(Reg3_Read_Enable),
        .Memwrite_Enable(Memwrite_Enable),
        .Memread_Enable(Memread_Enable),
        .memdata_store(memdata_store),
        .is_imm(is_imm),
        .immediate(immediate),
        .R2_out(R2_out),
        .R3_out(R3_out),
        .selectbits(selectbits),
        .result(result),
        .mem_address(mem_address),
        .jump_enable(jump_enable),
        .jump_value(jump_value),
        .R1_write_enable(R1_write_enable),
        .Flag_write_enable(Flag_write_enable),
        .Fetch_Signal(Fetch_Signal),
        .Decode_Signal(Decode_Signal),
        .Alu_Signal(Alu_Signal),
        .Compare_Signal(Compare_Signal),
        .Read_Signal(Read_Signal),
        .state(state)
    );

    always @(*) begin
        if(is_imm == 1'b1) begin
            assign input1 = R2_data;
            assign input2 = immediate;
        end
        else begin
            assign input1 = R2_data;
            assign input2 = R3_data;
        end
    end

    alu alu(
        .input1(input1),
        .input2(input2),
        .opcode(opcode),
        .result(result)
    );

    program_counter program_counter(
        .clk(clk),
        .reset(reset),
        .enable(enable),
        .increment(increment),
        .address(address)
    );

    RAM RAM(
        .clk(clk),
        .address(address),
        .data_input(data_input),
        .write_enable(write_enable),
        .data_output(data_output)
    );

    ROM ROM(
        .address(address),
        .data_output(data_output)
    );

    comparator comparator(
        .a(a),
        .b(b),
        .zero_flag(zero_flag),
        .carry_flag(carry_flag)
    );


endmodule