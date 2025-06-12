/*
///////////////////////////////////////////////////////////////////////////////////////////////
// Pseudocode for asm_pb_i_address4_test(test_adr_y)
// Purpose: Test address 'test_adr_y' on lamp switch cards 1-4
// Result is stored in TEST_CARD_Y_OUT[] array

function asm_pb_i_address4_test(test_adr_y):
    R1 = address of TEST_Y_READ      // Output/result pointer    
    R4 = test_adr_y << 3 // Prepare address port1 value: R4 = 8 * test_adr_y

    for board in [BOARD_1, BOARD_2, BOARD_3, BOARD_4]:
        
        P1 = board + R4 + TEST_ADDR_ON // Set P1 = board + (8 * test_adr_y) + TEST_ADDR_ON        
        wait_some_cycles() // Wait for data ready (simulate with two NOPs)        
        data = read_data_bus() // Read address via data bus
        P1 = NO_BOARD_IDLE | 0 | CTR_OFF // Reset Test_RD line: P1 = NO_BOARD_IDLE | 0 | CTR_OFF        
        store_result(R1, result) // Store result (0 means OK) in TEST_Y_READ[]
        R1 = R1 + 1
    // End for

    return
*/


// remeber, test_adr_y = command_param_data[0]
// P1 = Port1 :- 
    if (substate_pb_test_active) begin
        case (substate_pb_test)
            SUBSTATE_PB_TEST_IDLE: begin
                Board_ID_ptr    <= 0;
                data_dir <= DIR_INPUT;
                wait_multiples  <= 1;
                substate_pb_test_next <= SUBSTATE_PB_TEST_ADDR_ON;   
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
                end

            //P1 = board + R4 + TEST_ADDR_ON // Set P1 = board + (8 * test_adr_y) + TEST_ADDR_ON
            SUBSTATE_PB_TEST_ADDR_ON : begin // loop start
                BOARD_X <= BOARD_1 << Board_ID_ptr; //BOARD_X = 1, 2, 4 or 8
                TEST_ADDR_ON;
                wait_multiples <= 3; // total guess for time being...
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
                substate_pb_test_next <= SUBSTATE_TEST_READ_DATA;
            end

            SUBSTATE_PB_TEST_WAIT_750N: begin
                if(wait_multiples) begin
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns) if clock = 20MHZ, 750ns can be achieved
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples - 3'd1;
                    end
                end else begin
                   substate_pb_test <= substate_pb_test_next;
                end
            end

            //data = read_data_bus() // Read address via data bus
            SUBSTATE_TEST_READ_DATA: begin
               Read_Data_buffer[Board_ID_ptr] <= Data_In_Port;
               wait_multiples <= 3; // guess
               substate_pb_test_next <= SUBSTATE_TEST_CTR_OFF;
               substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
            end

            //P1 = NO_BOARD_IDLE | 0 | CTR_OFF // Reset Test_RD line: P1 = NO_BOARD_IDLE | 0 | CTR_OFF
            SUBSTATE_TEST_CTR_OFF: begin
                CTR_OFF;
                NO_BOARD_IDLE;
                wait_multiples <= 3; // guess
                substate_pb_test_next <= SUBSTATE_PB_TEST_INC_CARD_ID_LOOP;
                substate_pb_test <= SUBSTATE_PB_TEST_WAIT_750N;
            end

            SUBSTATE_PB_TEST_INC_CARD_ID_LOOP: begin               
               if(Board_ID_ptr < (4 - 1)) begin   // 0->3 is 4 hence the -1  
                  Board_ID_ptr <= Board_ID_ptr + 3'd1; 
                  substate_pb_test <= SUBSTATE_PB_TEST_ADDR_ON; // loop back round to do remaining cards
               end else begin
                   substate_pb_test <= SUBSTATE_PB_TEST_DONE;
               end
            end

            SUBSTATE_PB_TEST_DONE: begin
               ResponseBytes[0:3] <= Read_Data_buffer[0:3];
               ResponseByteCount  <= 4;
               substate_pb_test_complete <= 1'b1;
               substate_pb_test <= SUBSTATE_PB_TEST_IDLE;
            end

        endcase
    end else begin
        substate_pb_test_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin