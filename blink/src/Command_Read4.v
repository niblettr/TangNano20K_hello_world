/*
; (port) defines port address to be read (same for all boards)
; (buf_addr) defines address of data input buffer, data will be stored at contiguous
; addresses
; data buffer addresses must be in idata segment
[pb_read4](port,buf_adrr) x4 as there are 4 boards
MOV     P1,#BOARD_4 OR %port OR RD_ON
MOV     DPTR,#%buf_adrr
MOVX    A,@R0                 ; read data from bus
SETB    PB_RD                 ; read inactive
MOVX    @DPTR,A               ; store data to DPR
[loop 3 more times to do remaining cards (4 in total)]
*/


    //M_C pb_read4
    if (substate_pb_read4_active) begin
        case (substate_pb_read4)
            SUBSTATE_PB_READ4_IDLE: begin
                Board_ID_ptr  <= 0; // start from first card
                data_dir <= DIR_INPUT;
                substate_pb_read4 <= SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID;
            end
            
            SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID: begin //MOV     P1,#BOARD_4 OR %port OR RD_ON // note its now 1,2,3 and 4 not 4,3,2 and 1
                BOARD_X <= BOARD_1 << Board_ID_ptr; //BOARD_X = 1, 2, 4 or 8  
                AddessPortPin <= command_param_data[0][2:0];  // only use lowest 3 bits
                RdP <= ENABLE;
                wait_multiples <= 3;
                substate_pb_read4 <= SUBSTATE_PB_READ4_WAIT_750N;
                substate_pb_read4_next <= SUBSTATE_PB_READ4_READ_DATA;
            end

            SUBSTATE_PB_READ4_WAIT_750N: begin
               if (substate_wait_counter < 30 -1) begin // @ 40MHZ, 1 cycle takes 25ns, 750ns
                  substate_wait_counter <= substate_wait_counter + 1'b1;
               end else begin                   
                   if(wait_multiples <= 1) begin  // preempt wait_multiples decrementing to zero
                       substate_wait_counter <= 1;
                       wait_multiples <= 0;
                       substate_pb_read4 <= substate_pb_read4_next;
                    end else begin
                       wait_multiples <= wait_multiples - 5'd1;
                       substate_wait_counter <= 0;
                    end
                end
            end
            
            SUBSTATE_PB_READ4_READ_DATA: begin
               ResponseBytes[Board_ID_ptr] <= Data_In_Port;
               wait_multiples <= 3;
               substate_pb_read4_next <= SUBSTATE_PB_READ4_INC_CARD_ID_LOOP;
               substate_pb_read4 <= SUBSTATE_PB_READ4_WAIT_750N;
            end

            SUBSTATE_PB_READ4_INC_CARD_ID_LOOP: begin
               RdP <= DISABLE;        
               if(Board_ID_ptr < (4 - 1)) begin   // 0->3 is 4 hence the -1
                  Board_ID_ptr <= Board_ID_ptr + 3'd1; 
                  substate_pb_read4 <= SUBSTATE_PB_READ4_ASSERT_ADDRESS_ID; // loop back round to do remaining cards
               end else begin
                   substate_pb_read4 <= SUBSTATE_PB_READ4_DONE;
               end
            end

            SUBSTATE_PB_READ4_DONE: begin
                substate_pb_read4_complete <= 1'b1;   // Indicate substate_pb_read4 completion
                substate_pb_read4 <= SUBSTATE_PB_READ4_IDLE;
            end

        endcase
    end else begin
        substate_pb_read4_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin