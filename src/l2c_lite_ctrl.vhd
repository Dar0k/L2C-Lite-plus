------------------------------------------------------------------------------
--  This file is a part of the GRLIB VHDL IP LIBRARY
--  Copyright (C) 2003 - 2008, Gaisler Research
--  Copyright (C) 2008 - 2014, Aeroflex Gaisler
--  Copyright (C) 2015 - 2022, Cobham Gaisler
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library gaisler;
use gaisler.l2c_lite.all;

library grlib;
use grlib.stdlib.all;
use grlib.amba.all;
use grlib.devices.all;

library techmap;
use techmap.gencomp.all;

entity l2c_lite_ctrl is
    generic (
        hsindex  : integer          := 0;
        haddr    : integer          := 16#000#;
        hmask    : integer          := 16#000#;
        ioaddr   : integer          := 16#000#;
        waysize  : integer          := 32;
        linesize : integer          := 32;
        cached   : std_logic_vector := x"FFFF";
        repl     : integer          := 0;
        ways     : integer          := 2);
    port (
        rstn   : in std_ulogic;
        clk    : in std_ulogic;
        ctrlo  : out cache_ctrli_type;
        ctrli  : in cache_ctrlo_type;
        ahbsi  : in ahb_slv_in_type;
        ahbso  : out ahb_slv_out_type;
        bm_out : in bm_out_type;
        bm_in  : out bm_in_type);
        

end entity l2c_lite_ctrl;

architecture rtl of l2c_lite_ctrl is


    constant addr_depth : integer := log2ext(waysize * (2 ** 10) / linesize);
    constant tag_len    : integer := 32 - addr_depth - log2ext(linesize);
    constant tag_dbits  : integer := tag_len + 2;
    constant owners	: integer := 2 ** ahbsi.hmaster'length;
    constant totalRegisters : integer := register_count + owners;


    -- RANGES -- 
    subtype TAG_R is natural range 31 downto addr_depth + log2ext(linesize);
    subtype TAG_DATA_R is natural range tag_dbits - 1 downto 2;
    subtype INDEX_R is natural range (addr_depth + log2ext(linesize) - 1) downto log2ext(linesize);
    subtype TAG_INDEX_R is natural range 31 downto log2ext(linesize);
    subtype OFFSET_R is natural range log2ext(linesize) - 1 downto 0;
    

    type reg_type is record
        control_state : state_type;
        write_delay   : std_logic;
        bw_state      : bw_state_type;
        bmrd_addr : std_logic_vector(32 - 1 downto 0);
        bmrd_size : std_logic_vector(log2ext(max_size) - 1 downto 0);
        bmrd_req  : std_logic;
        bmwr_addr : std_logic_vector(32 - 1 downto 0);
        bmwr_size : std_logic_vector(log2ext(max_size) - 1 downto 0);
        bmwr_req  : std_logic;
    	internal_repl : integer range 0 to 3; -- DETERMINES REPLACEMENT POLICY OF CACHE --
        bmwr_data : std_logic_vector(bm_dw - 1 downto 0);
        cache_data_temp : std_logic_vector(linesize * 8 - 1 downto 0);
        bm_counter      : integer range 0 to linesize * 8/bm_dw + 1;
        fetch_delay : std_ulogic;
        cache_registers : std_logic_vector(register_count * 32 - 1 downto 0);
        cfo            : cf_out_type;
        miss           : std_logic;
        miss_addr      : std_logic_vector(31 downto 0);
    end record;


    ----------------------- FUNCTIONS -------------------------------
    ---- FUNCTION DESCRIPTION:
    --          Outputs next state of the cache after the last request has been served

    function stateAfterResponse(signal frontend : in cf_out_type)
        return state_type is 
        begin
            if frontend.valid = '1' then
                if frontend.IO_request = '0' then -- Request to cache data

                    if frontend.write = '0' then -- Read
                        if is_cachable(frontend.addr(31 downto 28), cached) then
                            return READ_S;
                        else
                            return DIRECT_READ_S;
                        end if;
                    else    -- Write
                        if is_cachable(frontend.addr(31 downto 28), cached) then
                            return WRITE_S;
                        else
                            return DIRECT_WRITE_S;
                        end if;
                    end if;
                
                else    -- IO request
                    
                    if frontend.write = '0' then
                        return IO_READ_S;
                    else
                        return IO_WRITE_S;
                    end if;

                end if;
            else
                return IDLE_S;
            end if;



        end function stateAfterResponse;

    
    -- COMPONENT INITIALIZATION
    component l2c_lite_frontend is
        generic(
            hsindex     : integer;
            haddr       : integer;
            hmask       : integer;
            ioaddr      : integer;
            linesize    : integer
        );
        port(
            clk         : in std_ulogic;
            rstn        : in std_ulogic;
            ahbsi       : in ahb_slv_in_type;       -- AHB INPUT 
            cfi         : in cf_in_type;            -- L2 OUTPUT

            cfo         : out cf_out_type;          -- L2 INPUT
            ahbso       : out ahb_slv_out_type      -- AHB OUTPUT
        );
    end component;



    -- INTERNAL SIGNALS 

    signal fetch_ready : std_ulogic := '0';
    signal r, rin      : reg_type;
    signal cfi         : cf_in_type;
    signal cfo         : cf_out_type;
    signal out_ahb     : ahb_slv_out_type;

begin

    frontend : l2c_lite_frontend 
    generic map(
        hsindex  => hsindex,
        haddr    => haddr,
        hmask    => hmask,
        ioaddr   => ioaddr,
        linesize => linesize 
    )
    port map(
        clk      => clk,
        rstn     => rstn,
        ahbsi    => ahbsi,
        cfi  => cfi,
        cfo => cfo,
        ahbso    => out_ahb
    );

    
    ctrlo.cachectrl_s <= r.control_state; 
    ctrlo.backendw_s  <= r.bw_state;
    ctrlo.fetch_ready <= fetch_ready;
    ctrlo.frontout    <= cfo;
    ahbso             <= out_ahb;

    comb : process (r, ahbsi, rstn, ctrli, bm_out, fetch_ready, cfo)
        variable v                         : reg_type;
        variable hit_trigger, miss_trigger : std_ulogic;
    	variable dirty_evict               : std_ulogic;
        variable counter                   : integer range 0 to AHBDW/8;
        variable c_offset                  : integer range 0 to linesize-1;
        variable offset                    : integer range 0 to 63;
        variable writeDataTemp             : std_logic_vector(127 downto 0); --TODO GENERIC IMPL
    begin

        offset := IO_offset(r.cfo.addr);
        v := r;
        ---- SAMPLE FRONTEND BUS ----
        v.cfo := cfo;

        ---- RESTART VALUE OF SIGNALS TO AVOID UNDESIRED LATCHES --

        v.fetch_delay               := '0';
        hit_trigger                 := '0';
        miss_trigger                := '0';
    	dirty_evict                 := '0';
        fetch_ready                 <= '0';
        ctrlo.cache_data_in_backend <= (others => 'U');
        cfi.read_data               <= (others => 'U');
        cfi.resp                    <= CF_WAIT;
        cfi.miss_bus                <= (others => 'U');

        
        v.miss                      := '0';

        ---- BACKEND RESTART ----
        v.bmrd_req  := '0';
        v.bmrd_addr := (others => '0');
        v.bmrd_size := (others => '0');
        v.bmwr_req  := '0';
        v.bmwr_addr := (others => '0');
        v.bmwr_size := (others => '0');

        
        

        
        case r.control_state is
            when IDLE_S =>

                cfi.resp    <= CF_OKAY;             
                v.control_state := stateAfterResponse(cfo);


            when READ_S =>
                if fetch_ready = '0' then
                if ctrli.c_miss = '1'  then -- CACHE MISS

		            if ctrli.not_owner = '1' then
			            v.control_state := DIRECT_READ_S;              -- MASTER HAS NO WAYS ASSIGNED
		            else 

                        
                        
                        v.control_state := IDLE_S;  
                        
                        if r.bw_state = IDLE_S then
                    	    miss_trigger    := '1';
                            v.miss          := '1';
                            v.miss_addr     := r.cfo.addr;
                            cfi.resp        <= CF_MISS_ACCEPT;
                        else    -- count_miss?
                            cfi.resp        <= CF_MISS_DENY;
                        end if;

		            end if;
                    

                elsif ctrli.c_hit = '1' then    -- CACHE HIT

                    cfi.resp    <= CF_OKAY; -- TRANSACTION ENDS IN THIS CYCLE
                    hit_trigger := '1';

                    cfi.read_data <= ctrli.frontend_buf(linesize*8 - 1 downto 0);   -- SEND CACHE LINE TO FRONTEND
                    
                    v.control_state := stateAfterResponse(cfo);        -- COMPUTE NEXT CACHE STATE
                    
                end if;
                end if;
                

              
            when WRITE_S =>
                v.write_delay := '0';
                if (fetch_ready = '0') then
                    if (ctrli.c_miss = '1') then                -- CACHE MISS
                        
        		        if ctrli.not_owner = '1' then           -- OWNER HAS NO WAYS ASSIGNED
		                	v.control_state := DIRECT_WRITE_S;  
		                else
                            
                            if r.bw_state = IDLE_S then
                                cfi.resp        <= CF_OKAY;
                        	    miss_trigger    := '1';
                                v.miss          := '1';
                                v.miss_addr     := r.cfo.addr;

                                --if cfo.seq_access = '1' and cfo.valid = '1' then  -- CHECK INCREMENT READS (MAYBE NOT NECESSARY NOW ???)
                                --    v.control_state := W_INCR_S; --TODO: CHECK THAT TAG_INDEX COINCIDES 
                                --else
                                v.control_state := stateAfterResponse(cfo);        -- COMPUTE NEXT CACHE STATE
                                --end if;
                                
                            else
                                cfi.resp        <= CF_MISS_DENY;
                                v.control_state := IDLE_S;
                            end if;
                    
		                end if;

                        
                    
                    elsif ctrli.c_hit = '1' then                -- CACHE HIT
                                   
                        hit_trigger         := '1';             
                        cfi.resp            <= CF_OKAY;            -- TRANSACTION ENDS IN THIS CYCLE
                        
                        --v.control_state := IDLE_S;
                        --if cfo.seq_access = '1' and cfo.valid = '1' then  -- CHECK IF NEXT ACCESS IS A WRITE 
                        --    v.control_state := W_INCR_S;--TODO: CHECK THAT TAG_INDEX COINCIDES 
                        --else
                        v.control_state := stateAfterResponse(cfo);        -- CALCULATE NEXT CACHE STATE
                        --end if;
                        
                    end if;
                end if;

            -- CHECK IF THIS STATE IS REALLY NEEDED, MAYBE TREAT IN FRONTEND, MAYBE JUST DO NORMAL READ INSTEAD?
            -- REMOVING THIS AND WRITE INCR WOULD MAKE COMMUNICATION FROM FRONTEND TO CTRL EASIER

            when R_INCR_S =>               
                cfi.resp         <= CF_OKAY;
                hit_trigger := '1';
 
                cfi.read_data <= ctrli.frontend_buf(linesize*8 - 1 downto 0);
               
                if cfo.seq_access = '1' and cfo.valid = '1' then
                    v.control_state := R_INCR_S;--TODO: CHECK THAT TAG_INDEX COINCIDES 
                else
                    v.control_state := stateAfterResponse(cfo);
                end if;


            when W_INCR_S =>
                cfi.resp    <= CF_OKAY;
                hit_trigger := '1';

                if cfo.seq_access = '1' and cfo.valid = '1' then
                    v.control_state := W_INCR_S;--TODO: CHECK THAT TAG_INDEX COINCIDES 
                else
                    v.control_state := stateAfterResponse(cfo);
                end if;

        
            -- MAYBE MOVE IO TO EXTERNAL MODULE -> EASIER TO UNDERSTAND AND SIMPLER COMMUNICATION??

            when IO_READ_S =>
                
                cfi.resp      <= CF_OKAY;
                
                if offset < register_count  then                    -- ACCESS TO CONTROL MODULE REGISTERS --

                    for i in 0 to 7 loop -- NOT GENERIC
                        cfi.read_data(i*32 + 31 downto i*32) <= r.cache_registers((register_count - offset) * 32 - 1
                                                                              downto (register_count - (offset + 1))*32);
                    end loop;

                elsif offset < register_count + ways + owners then  -- ACCESS TO MEMORY MODULE REGISTERS  --

                    for i in 0 to 7 loop -- NOT GENERIC
                        cfi.read_data(i*32 + 31 downto i*32) <= ctrli.frontend_buf(31 downto 0);
                    end loop;
                end if;

                v.control_state := stateAfterResponse(cfo);
                    


            when IO_WRITE_S =>
                
                cfi.resp      <= CF_OKAY;
                
                if offset < register_count then -- ACCESS TO CONTROL REGISTERS --

                    if offset = register_count - 2 then -- CACHE INFO REGISTER --

                        --v.internal_repl := to_integer(unsigned(v.cfo.write_data(31 downto 30))); -- CHANGES REPLACEMENT POLICY

                    else                                  

                        --v.cache_registers((register_count - offset) * 32 - 1 downto (register_count - (offset + 1))*32) := v.cfo.write_data(31 downto 0);

                    end if;
                end if;

                v.control_state := stateAfterResponse(cfo);

            
            when DIRECT_READ_S =>

                if r.bw_state = IDLE_S and ctrli.evict = '0' then

                if bm_out.bmrd_req_granted = '1' then
                    v.bmrd_req  := '1';
                    v.bmrd_addr := r.cfo.addr;
                    v.bmrd_size := conv_std_logic_vector(size_vector_to_int(r.cfo.transfer_size) - 1, r.bmrd_size'length);

                elsif (bm_out.bmrd_valid and bm_out.bmrd_done) = '1' then

                    counter  := 0;
                    c_offset := to_integer(unsigned(r.cfo.addr(OFFSET_R)));

                    for i in 0 to (linesize*8)/bm_dw - 1 loop
                        cfi.read_data(linesize*8 - bm_dw*i - 1 downto linesize*8 - bm_dw*(i+1)) <= bm_out.bmrd_data;
                    end loop;
                    

                    cfi.resp             <= CF_OKAY;
                    
                    v.control_state := stateAfterResponse(cfo);

                end if;
                end if;

            when DIRECT_WRITE_S =>
                
                if r.bw_state = IDLE_S and ctrli.evict = '0' then
                

                if bm_out.bmwr_req_granted = '1' then
                    v.bmwr_req  := '1';
                    v.bmwr_addr := r.cfo.addr;
                    v.bmwr_size := conv_std_logic_vector(size_vector_to_int(r.cfo.transfer_size) - 1, r.bmwr_size'length);

                    if endianess = 1 then
                        writeDataTemp := reversedata(v.cfo.write_data, 8);
                    else
                        writeDataTemp := v.cfo.write_data;
                    end if;

                    counter := 0;
                    for i in 0 to AHBDW/8 - 1 loop
                        if counter < size_vector_to_int(r.cfo.transfer_size) then
                            v.bmwr_data(bm_dw - 8 * counter - 1 downto bm_dw - 8 * (counter + 1)) :=
                            writeDataTemp(AHBDW - 8 * counter - 1 downto AHBDW - 8 * (counter + 1));
                            counter := counter + 1;
                        end if;
                    end loop;

                elsif bm_out.bmwr_done = '1' then
                    cfi.resp        <= CF_OKAY;
                    v.control_state := stateAfterResponse(cfo);
                end if;
                end if;

            when FLUSH_S =>

                if ctrli.f_done =   '1' then
                    cfi.resp        <= CF_OKAY;
                    v.control_state := stateAfterResponse(cfo);
                    v.cache_registers(register_count * 32 - 1
                    downto (register_count - 1) * 32) := (others => '0');
                end if;

            when others =>
        end case;
        ---- BACKEND EVICTION INTERFACE ----
        case r.bw_state is
            when IDLE_S =>

                if v.miss = '1' then
                    v.bw_state := BACKEND_READ_S;

                elsif ctrli.evict = '1' then
                    v.bw_state := BACKEND_WRITE_S;
		            dirty_evict := '1';
                end if;

            when BACKEND_WRITE_S =>

                if bm_out.bmwr_req_granted = '1' then

                    v.bmwr_req   := '1';
                    v.bmwr_addr  := ctrli.backend_buf_addr;
                    v.bmwr_size  := conv_std_logic_vector(linesize - 1, r.bmwr_size'length);
                    v.bmwr_data  := ctrli.backend_buf(linesize * 8 - 1 downto linesize * 8 - bm_dw);
                    v.bm_counter := 0;

                end if;
                if bm_out.bmwr_full = '0' and (r.bm_counter < linesize * 8/bm_dw - 1) then

                    v.bm_counter := r.bm_counter + 1;

                    v.bmwr_data := ctrli.backend_buf(linesize * 8 - bm_dw * v.bm_counter - 1
                    downto linesize * 8 - bm_dw * (1 + v.bm_counter));

                end if;
                
                if v.bm_counter >= linesize *8 /bm_dw - 1 then

                    v.bm_counter    := 0;
                    
                end if;

                if bm_out.bmwr_done = '1' then

                    v.bw_state   := IDLE_S;
                    v.bm_counter := 0;
                    fetch_ready     <= '1';
                    
                end if;

            when BACKEND_READ_S =>

                if r.fetch_delay = '1' then

                    ctrlo.cache_data_in_backend(linesize * 8 - 1 downto 0) <= r.cache_data_temp;    -- Pass data to mem module to write it into cache
                    cfi.miss_bus <= r.cache_data_temp; -- Pass data into frontend to send response

                    fetch_ready <= '1';
                    
                    if ctrli.evict = '1' then   -- IF MISS HAS CAUSED A DIRTY LINE TO BE EVICTED, IT SHOULD BE WRITTEN NOW
                        v.bw_state := BACKEND_WRITE_S;
                    else
                        v.bw_state := IDLE_S;
                    end if;

                elsif bm_out.bmrd_req_granted = '1' then
                    v.bmrd_req            := '1';
                    v.bmrd_addr           := r.miss_addr;
                    v.bmrd_addr(OFFSET_R) := (others => '0');
                    v.bmrd_size           := conv_std_logic_vector(linesize - 1, r.bmwr_size'length);
                    v.bm_counter          := 0;

                elsif bm_out.bmrd_valid = '1' then

                    v.cache_data_temp(linesize * 8 - bm_dw * r.bm_counter - 1
                    downto linesize * 8 - bm_dw * (1 + r.bm_counter)) := bm_out.bmrd_data;

                    if bm_out.bmrd_done = '1' then

                        ctrlo.cache_data_in_backend(linesize * 8 - 1 downto 0) <= r.cache_data_temp;

                        v.fetch_delay := '1';
                    else
                        v.bm_counter := r.bm_counter + 1;
                    end if;

                end if;
            

            when others =>
        end case;

        ---- FLUSH TRIGGER ----
        if v.cache_registers(register_count * 32 - 32) = '1' then
            v.control_state := FLUSH_S;
            cfi.resp        <= CF_WAIT;
        end if;

        ---- UPDATES INTERNAL CACHE REGISTERS ----
        if hit_trigger = '1' then	
            v.cache_registers((register_count - 1) * 32 - 1 downto (register_count - (1 + 1)) * 32) := conv_std_logic_vector((
            to_integer(unsigned(r.cache_registers((register_count - 1) * 32 - 1 downto (register_count - (1 + 1)) * 32))) + 1), 32);
        end if;
        if miss_trigger = '1' then
            v.cache_registers((register_count - 2) * 32 - 1 downto (register_count - (2 + 1)) * 32) := conv_std_logic_vector((
            to_integer(unsigned(r.cache_registers((register_count - 2) * 32 - 1 downto (register_count - (2 + 1)) * 32))) + 1), 32);
        end if;

	    if dirty_evict = '1' then
	        v.cache_registers((register_count - 4) * 32 - 1 downto (register_count - (4 + 1)) * 32) := conv_std_logic_vector((
	        to_integer(unsigned(r.cache_registers((register_count - 4) * 32 - 1 downto (register_count - (4 + 1)) * 32))) + 1), 32);
    	end if;

        v.cache_registers((register_count - 3) * 32 - 1 downto (register_count - 3) * 32 - 2)   := conv_std_logic_vector(v.internal_repl, 2); 
        v.cache_registers((register_count - 3) * 32 - 5 downto (register_count - 3) * 32 - 12)  := conv_std_logic_vector(ways-1, 8); 
        v.cache_registers((register_count - 3) * 32 - 13 downto (register_count - 3) * 32 - 16) := conv_std_logic_vector(log2ext(linesize)-4, 4);
        v.cache_registers((register_count - 3) * 32 - 19 downto (register_count - 3) * 32 - 32) := conv_std_logic_vector(waysize, 14);

        --------  OUTPUTS  --------
        bm_in.bmwr_req  <= v.bmwr_req;
        bm_in.bmwr_addr <= v.bmwr_addr;
        bm_in.bmwr_size <= v.bmwr_size;
        bm_in.bmwr_data <= v.bmwr_data;

        bm_in.bmrd_req  <= v.bmrd_req;
        bm_in.bmrd_addr <= v.bmrd_addr;
        bm_in.bmrd_size <= v.bmrd_size;

	    ctrlo.internal_repl <= v.internal_repl;

        cfi.fetch_ready <= fetch_ready;
        

        rin <= v;

    end process;
    ---- ASYNC RESET ----
    regs : process (clk, rstn)
    begin
        if rstn = '0' then
            r.cache_registers <= (others => '0');
            r.control_state   <= IDLE_S;
            r.bw_state        <= IDLE_S;
	        r.internal_repl   <= repl;

        elsif rising_edge(clk) then
            r <= rin;
        end if;

    end process;

end architecture rtl;
