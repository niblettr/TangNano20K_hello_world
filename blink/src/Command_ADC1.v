/*
_asm_pb_adc1_16:
MOV     P1,#BOARD_ALL OR PORT_MUX OR CTR_OFF
;Obtain the required multiplexer address and multiplexer enables...
MOV     A,R5                  ; load mux_inp_code to A

;Send the required data to the multiplexer latches...
MOVX    @R0,A                 ; output mux_inp_code to boards
CLR     DIR_OUT               ; switch data on Phase Bus
CLR     PB_WR                 ; SETB WR activ
NOP                           ; make pulse longer
SETB    PB_WR                 ; SETB WR passiv
NOP                           ; wait until mux-out stable
NOP
NOP
NOP
CLR     PB_WR                 ; SETB WR activ
NOP                           ; make pulse longer
SETB    PB_WR                 ; SETB WR passiv
SETB    DIR_OUT               ; Switch Phase Bus for input
MOV     A,R7                  ; load 2. parameter board code
MOV     R2,A                  ; store board code in R2
ADD     A,#PORT_ADC_LOW + RD_ON
MOV     P1,A
NOP
NOP
MOVX    A,@R0                 ; read data from bus
SETB    PB_RD                 ; read inactive
MOV     R7,A                  ; return low byte to R7
MOV     A,R2                  ; load board code
ADD     A,#PORT_ADC_HIGH + RD_ON
MOV     P1,A
NOP
NOP
MOVX    A,@R0                 ; read data from bus
SETB    PB_RD                 ; read inactive
ENDIF
MOV     R6,A                  ; return high byte to R6
RET
*/
    if (substate_pb_adc1_active) begin
        case (substate_pb_adc1)
            SUBSTATE_PB_ADC1_IDLE: begin
                Board_ID_ptr <= 0;
                data_dir <= DIR_INPUT;
                wait_multiples <= 3;
                substate_pb_adc1_next = SUBSTATE_PB_ADC1_ASSERT_ADDRESS_ID;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                end
            
            SUBSTATE_PB_ADC1_ASSERT_ADDRESS_ID: begin //MOV     P1,#BOARD_ALL OR PORT_MUX OR CTR_OFF
                BOARD_ALL;
                PORT_MUX;
                CTR_OFF;
                wait_multiples <= 1;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                substate_pb_adc1_next <= SUBSTATE_PB_ADC1_LOAD_MUX_INP_CODE;
            end

            SUBSTATE_PB_ADC1_LOAD_MUX_INP_CODE: begin
                AddessPort <= command_param_data[0][2:0];
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                substate_pb_adc1_next <= SUBSTATE_PB_ADC1_ASSERT_WR_ENABLE;
            end

            SUBSTATE_PB_ADC1_ASSERT_WR_ENABLE: begin
                PB_WR <= ENABLE;
                wait_multiples <= 2;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                substate_pb_adc1_next <= SUBSTATE_PB_ADC1_RELEASE_WR_ENABLE;
            end

            SUBSTATE_PB_ADC1_RELEASE_WR_ENABLE: begin
                PB_WR <= DISABLE;
                wait_multiples <= 2;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                substate_pb_adc1_next <= SUBSTATE_PB_ADC1_DONE;
            end

            SUBSTATE_PB_ADC1_WAIT_750N: begin
                if(wait_multiples) begin
                    if (substate_wait_counter < 21) begin // Proceed after 21 cycles (~777ns) if clock = 20MHZ, 750ns can be achieved
                        substate_wait_counter <= substate_wait_counter + 1'b1;
                    end else begin                        
                        substate_wait_counter <= 0;
                        wait_multiples <= wait_multiples - 3'd1;
                    end
                end else begin
                   substate_pb_adc1 <= substate_pb_adc1_next;
                end
            end

            SUBSTATE_PB_ADC1_DONE: begin
                //if(CommandType == 0) begin
                   ResponseBytes[0:3] <= Read_Data_buffer[0:3];
                   ResponseByteCount  <= 4;
                //end

                substate_pb_adc1_complete <= 1'b1;   // Indicate substate_pb_adc4 completion
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_IDLE;
            end

        endcase
    end else begin
        substate_pb_adc1_complete <= 1'b0; // Clear the flag when substate_pb_i_read4 is inactive
    end // end of if (substate_pb_i_read4_active) begin