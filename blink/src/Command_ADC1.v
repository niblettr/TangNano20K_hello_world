    if (substate_pb_adc1_active) begin
        case (substate_pb_adc1)
            SUBSTATE_PB_ADC1_IDLE: begin
                Board_ID_ptr <= 0;
                data_dir <= DIR_INPUT;
                wait_multiples <= 3;
                substate_pb_adc1_next = SUBSTATE_PB_ADC1_DONE;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                end

            //MOV     P1,#BOARD_ALL OR PORT_MUX OR CTR_OFF
            //#define BOARD_ALL            0x05 // -----101
            SUBSTATE_PB_ADC1_ASSERT_ADDRESS_ID: begin
                BOARD_X <= 5;//BOARD_ALL;
                AddessPortPin <= _PORT_MUX;
                WrP <= DISABLE; // CTR_OFF in the assembler
                RdP <= DISABLE; // CTR_OFF in the assembler
                wait_multiples <= 1;
                substate_pb_adc1 <= SUBSTATE_PB_ADC1_WAIT_750N;
                substate_pb_adc1_next <= SUBSTATE_PB_ADC1_DONE; // <<<<<<<<<<<<<<<<<<<<<<
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

            SUBSTATE_PB_ADC4_DONE: begin
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