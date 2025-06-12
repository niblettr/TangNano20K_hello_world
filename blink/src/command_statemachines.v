module state_machines #(
    parameter CLOCK_FREQUENCY = 27000000 // System clock frequency in Hz
)(
    input       clock,    // System clock
    input       reset,

    // Activation Inputs
    input       reg       substate_pb_i_write4_active,
    input       reg       substate_pb_read4_active,
    input       reg       substate_pb_adc4_active,
    input       reg       substate_pb_adc1_active,
    input       reg       substate_pb_test_active,

    // Completion Outputs
    output      reg       substate_pb_i_write4_complete, 
    output      reg       substate_pb_read4_complete,
    output      reg       substate_pb_adc4_complete,
    output      reg       substate_pb_adc1_complete, 
    output      reg       substate_pb_test_complete, 

    // Board Control Outputs
    output      reg [3:0] BOARD_X,
    output      reg [2:0] AddessPortPin,
    output      reg       TestAddressP,
    output      reg       RdP,
    output      reg       WrP,
    output      reg       LampResetPin,

    // Command Parameters
    input       reg [7:0] command_param_data [0:3],
    input       reg [1:0] CommandType,

    // Data Ports
    output      reg [7:0] Data_Out_Port,
    input       [7:0]     Data_In_Port,
    output      reg       data_dir,

    // Response Handling
    output       reg [7:0] ResponseBytes[0:3],
    output       reg [3:0] ResponseByteCount
);

`include "utils.v"

/*********************************************************************************************************/
parameter _PORT_MUX  = 3'h18; // 0x18 // --011--- /  Same Port address
parameter BOARD_1    = 4'b0001;
/*********************************************************************************************************/

typedef enum logic [1:0] {
    DIR_INPUT  = 0,
    DIR_OUTPUT = 1          
} dir_mode_t;

typedef enum logic [1:0] {
    DISABLE  = 1,
    ENABLE   = 0          
} en_mode_t;


    // Define states for the new state machine
typedef enum logic [3:0] {
    SUBSTATE_PB_I_WRITE4_IDLE              = 4'b0000,
    SUBSTATE_PB_I_WRITE4_PRE_DELAY,
    SUBSTATE_PB_I_WRITE4_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_I_WRITE4_WAIT_750N,
    SUBSTATE_PB_I_WRITE4_ASSERT_DATA,
    SUBSTATE_PB_I_WRITE4_ASSERT_WR_ENABLE,
    SUBSTATE_PB_I_WRITE4_RELEASE_WR,
    SUBSTATE_PB_I_WRITE4_RELEASE_DATA,
    SUBSTATE_PB_I_WRITE4_INC_CARD_ID_LOOP,
    SUBSTATE_PB_I_WRITE4_DONE
} substate_pb_i_write4_t;
substate_pb_i_write4_t substate_pb_i_write4      = SUBSTATE_PB_I_WRITE4_IDLE;
substate_pb_i_write4_t substate_pb_i_write4_next = SUBSTATE_PB_I_WRITE4_IDLE;

typedef enum logic [3:0] {
    SUBSTATE_PB_READ4_IDLE              = 4'b0000,
    SUBSTATE_PB_READ4_PRE_DELAY,
    SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_READ4_WAIT_750N,
    SUBSTATE_PB_READ4_READ_DATA,
    SUBSTATE_PB_READ4_ASSERT_WR_ENABLE,
    SUBSTATE_PB_READ4_RELEASE_WR_ENABLE,
    SUBSTATE_PB_READ4_RELEASE_DATA,
    SUBSTATE_PB_READ4_INC_CARD_ID_LOOP,
    SUBSTATE_PB_READ4_DONE              
} substate_pb_read4_t;
substate_pb_read4_t substate_pb_read4            = SUBSTATE_PB_READ4_IDLE;
substate_pb_read4_t substate_pb_read4_next       = SUBSTATE_PB_READ4_IDLE;

typedef enum logic [3:0] {
    SUBSTATE_PB_ADC4_IDLE              = 4'b0000,
    SUBSTATE_PB_ADC4_PRE_DELAY,
    SUBSTATE_PB_ADC4_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_ADC4_WAIT_750N,
    SUBSTATE_PB_ADC4_ASSERT_DATA,
    SUBSTATE_PB_ADC4_ASSERT_WR_ENABLE,
    SUBSTATE_PB_ADC4_RELEASE_WR_ENABLE,
    SUBSTATE_PB_ADC4_RELEASE_DATA,
    SUBSTATE_PB_ADC4_INC_CARD_ID_LOOP,
    SUBSTATE_PB_ADC4_TEST_READ,
    SUBSTATE_PB_ADC4_DONE              
} substate_pb_adc4_t;

substate_pb_adc4_t substate_pb_adc4              = SUBSTATE_PB_ADC4_IDLE;
substate_pb_adc4_t substate_pb_adc4_next         = SUBSTATE_PB_ADC4_IDLE;

typedef enum logic [3:0] {
    SUBSTATE_PB_ADC1_IDLE              = 4'b0000,
    SUBSTATE_PB_ADC1_PRE_DELAY,
    SUBSTATE_PB_ADC1_ASSERT_ADDRESS_ID,
    SUBSTATE_PB_ADC1_WAIT_750N,
    SUBSTATE_PB_ADC1_ASSERT_DATA,
    SUBSTATE_PB_ADC1_ASSERT_WR_ENABLE,
    SUBSTATE_PB_ADC1_RELEASE_WR_ENABLE,
    SUBSTATE_PB_ADC1_RELEASE_DATA,
    SUBSTATE_PB_ADC1_INC_CARD_ID_LOOP,
    SUBSTATE_PB_ADC1_TEST_READ,
    SUBSTATE_PB_ADC1_DONE              
} substate_pb_adc1_t;

substate_pb_adc1_t substate_pb_adc1              = SUBSTATE_PB_ADC1_IDLE;
substate_pb_adc1_t substate_pb_adc1_next         = SUBSTATE_PB_ADC1_IDLE;

typedef enum logic [3:0] {
    SUBSTATE_PB_TEST_IDLE              = 4'b0000,
    SUBSTATE_PB_TEST_PRE_DELAY,
    SUBSTATE_PB_TEST_ADDR_ON,
    SUBSTATE_PB_TEST_WAIT_750N,
    SUBSTATE_TEST_READ_DATA,
    SUBSTATE_TEST_CTR_OFF,
    SUBSTATE_PB_TEST_INC_CARD_ID_LOOP,
    SUBSTATE_PB_TEST_TEST_READ,
    SUBSTATE_PB_TEST_DONE              
} substate_pb_test_t;

substate_pb_test_t substate_pb_test              = SUBSTATE_PB_TEST_IDLE;
substate_pb_test_t substate_pb_test_next         = SUBSTATE_PB_TEST_IDLE;

reg [4:0] substate_wait_counter  = 1; // NOTE: set to 1 instead of 0 because statemachine transitions waste 1 clock cyle
reg [4:0] wait_multiples         = 0; // 4bit : 0>31
reg [2:0] Board_ID_ptr           = 0;

reg [7:0] Read_Data_buffer [4];

reg [7:0] uart_tx_response_string [0:20];
reg [7:0] uart_tx_response_string_len;
reg       uart_tx_response_process = 1'b0;

reg [7:0] reset_counter    = 8'b0; // 1-bit counter for reset delay

task automatic TEST_ADDR_ON;
begin
    WrP <= ENABLE;
    RdP <= ENABLE;
end
endtask

task automatic CTR_OFF;
begin
    WrP <= DISABLE;
    RdP <= DISABLE;
end
endtask

task automatic BOARD_ALL;
begin
    BOARD_X <= 4'hF;
end
endtask

task automatic NO_BOARD_IDLE;
begin
   BOARD_X <= 4'h0;
end
endtask

always @(posedge clock) begin


    `include "InitCard.v"       // code for the Card initialisation

    `include "Command_Read4.v" // code/statmachine for pb_i_read

    `include "Command_Write4.v" // code/statmachine for pb_i_write

    `include "Command_Address4Test.v" // code/statmachine for pb_i_address4_test

    `include "Command_Address4Test.v" // code/statmachine for pb_i_address4_test

    `include "Command_ADC4.v" // code/statmachine for pb_adc4_16 & pb_adc4_8

     `include "Command_ADC1.v" // code/statmachine for asm_pb_adc1_16


end
endmodule