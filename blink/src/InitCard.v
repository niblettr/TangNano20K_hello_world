// reset portion, only carries out once at startup..
    if(reset) begin
       LampResetPin   <= 1'b1;        // hold in reset
       data_dir       <= DIR_INPUT;
       Data_Out_Port  <= 8'b0;
       AddessPortPin  <= 3'b0;
       TestAddressP   <= 1'b01;
       RdP            <= 1'b1;
       WrP            <= 1'b1;
    end else begin
       LampResetPin   <= 1'b0;   // release from reset state
    end
    
