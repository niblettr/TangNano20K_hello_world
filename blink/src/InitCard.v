

 
 // reset portion, only carries out once at startup..
    if(lamp_card_reset_activate) begin
           if(reset_counter < 100) begin
              reset_counter  <= reset_counter + 1'b1;
              LampResetPin   <= 1'b1;        // hold in reset
              data_dir       <= DIR_INPUT;
              Data_Out_Port  <= 8'b0;
              AddessPortPin  <= 3'b0;
              TestAddressP   <= 1'b01;
              RdP            <= 1'b1;
              WrP            <= 1'b1;
           end else begin
              LampResetPin   <= 1'b0;   // release from reset state
              lamp_card_reset_complete <= 1'b1;
           end
    end
    
