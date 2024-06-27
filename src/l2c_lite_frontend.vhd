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

entity l2c_lite_frontend is
    generic(
        hsindex     : integer          := 0;
        haddr       : integer          := 16#000#;
        hmask       : integer          := 16#000#;
        ioaddr      : integer          := 16#000#;
        linesize    : integer          := 32
    );
    port(
        clk         : in std_ulogic;
        rstn        : in std_ulogic;
        ahbsi       : in ahb_slv_in_type;       -- AHB INPUT 
        cfi         : in cf_in_type;            -- CACHE TO FRONTEND BUS

        ahbso       : out ahb_slv_out_type;     -- AHB OUTPUT
        cfo         : out cf_out_type           -- FRONTEND TO CACHE BUS

    );
end entity l2c_lite_frontend;

architecture rtl of l2c_lite_frontend is

    -- CONSTANT DECLARATION
    constant hconfig : ahb_config_type := (
        0      => ahb_device_reg (VENDOR_GAISLER, GAISLER_L2CL, 0, 0, 0),
        4      => gen_pnp_bar(haddr, hmask, 2, 1, 1),
        5      => gen_pnp_bar(ioaddr, IO_ADDR_MASK, 1, 0, 0),
        6      => gen_pnp_bar(16#000#, 16#000#, 0, 0, 0),
        7      => gen_pnp_bar(16#000#, 16#000#, 0, 0, 0),
        others => zero32
    );
    constant linesize_bits : integer := log2ext(linesize) - 1;
    constant bus_bits      : integer := log2ext(AHBDW) - 4;


    subtype OFFSET_R     is natural range log2ext(linesize) - 1 downto 0;
    subtype OFFSET_R_BUS is natural range log2ext(AHBDW)  - 4 downto 0;

      -- FUNCTION DESCRIPTION:
            -- WRITES CACHE RESPONSE IN BUS
            -- 
            -- After transfer_size bits have passed, the data is replicated.
            -- i.e. AHB width is 16b and transfer size is 4b (0110) then output is -> 0110 0110 0110 0110
            --
            -- According to the AHB 2.0. standard the slave must only ensure that the data is correct on its corresponding offset,
            -- but Gaisler does this
            
    function dataToBusFormat (datain : std_logic_vector(linesize*8-1 downto 0); addr : std_logic_vector(31 downto 0);
                              transfer_size : std_logic_vector(2 downto 0); endianess : integer) return std_logic_vector is
        variable counter                   : integer range 0 to AHBDW/8;
        variable c_offset                  : integer range 0 to linesize-1;
        variable output_data               : std_logic_vector(AHBDW - 1 downto 0);
    begin
        counter  := 0;
        c_offset := to_integer(unsigned(addr(OFFSET_R)));

        for i in 0 to AHBDW/8 - 1 loop
            output_data(AHBDW - 8 * i - 1 downto AHBDW - (i + 1) * 8) :=
            datain(linesize * 8 - (c_offset + counter) * 8 - 1 downto linesize * 8 - (c_offset + counter + 1) * 8);
        
            if counter = size_vector_to_int(transfer_size) - 1 then
                counter := 0;
            else
                counter := counter + 1; 
            end if;
            
        end loop;
            
        if endianess = 1 then
            return(reversedata(output_data, 8));
        else
            return(output_data);
        end if;
    end;


    -- Array with as many positions as owners (AHB allows a maximum of 16), where the miss read data is stored
    -- in the time between when the data is fetched by the backend and the master has access again to the bus.
    -- This is done because we cannot answer a request if the master is not in the bus in that moment.
    -- Each master has a buffer of 1 cacheline, with a valid bit (LSB) that indicates that the data has not been accessed yet.
    --256     1   0
    -- | miss_line | v |
    type split_data_buffer_t is array (0 to 15) of std_logic_vector(linesize * 8 downto 0);

    type split_reg_type is record
        master : std_logic_vector(15 downto 0);
        delay  : std_logic;
        read   : std_logic;     -- Maybe this signal is not necessary
        miss_master : std_logic_vector(3 downto 0);
        data_buffer : split_data_buffer_t;
    end record;

    constant split_init : split_reg_type := (
        master      => (others => '0'),
        delay       => '0',
        read        => '0',
        miss_master => (others => '0'),
        data_buffer => (others => (others => '0'))
    );

    type reg_type is record
        cfo        : cf_out_type; 
        split      : split_reg_type; 
        resp_delay : std_logic;
        fetch_delay: std_logic;
        hsplit     : std_logic_vector(15 downto 0);

    end record;
   

    -- INTERNAL SIGNAL DECLARATION
    signal r, rin : reg_type;
    signal hready : std_logic;
    --signal frontout_temp : front_out_type;
    
 

begin

    comb : process(ahbsi, r, cfi, hready) is
            
        variable owner_curr                : integer range 0 to 15;
        variable owner_prev                : integer range 0 to 15;

    begin
            owner_curr := to_integer(unsigned(ahbsi.hmaster));
            owner_prev := to_integer(unsigned(r.cfo.master));

            
            -- RESET SIGNALS
            rin                         <= r;
            rin.split.delay             <= '0';
            rin.cfo.valid               <= '0';
            ahbso.hsplit                <= (others => '0');
            rin.cfo.write_data          <= ahbsi.hwdata;
            hready                      <= '1';
            rin.hsplit                  <= (others => '0');
            ahbso.hrdata                <= (others => 'U');
            

            -- MISS FETCH HANDLING

            if cfi.fetch_ready = '1' then    -- Notify masters that miss has been completed
                ahbso.hsplit <= r.split.master;
                rin.split.master <= (others => '0');

                if r.split.read = '1' then   -- Save miss info in buffer until master gets access to bus
                    rin.split.read <= '0';
                    rin.split.data_buffer(to_integer(unsigned(r.split.miss_master))) <= cfi.miss_bus & '1'; -- Read data + valid bit set to '1'
                end if;
            else
                ahbso.hsplit <= r.hsplit;
            end if;

            -- RESPONSE HANDLING

            if r.split.delay = '1' then -- split signal must be up for 2 cycles
                ahbso.hresp <= HRESP_SPLIT;

                if cfi.fetch_ready = '1' then
                    rin.fetch_delay <= '1';
                    rin.hsplit(to_integer(unsigned(r.cfo.master))) <= '1';
                end if; 


            elsif r.resp_delay = '1' then -- Response to request given directly by frontend
                rin.resp_delay <= '0';
                ahbso.hrdata <= dataToBusFormat(r.split.data_buffer(owner_prev)(linesize*8 downto 1), r.cfo.addr, r.cfo.transfer_size, endianess);
                ahbso.hresp <= HRESP_OKAY;

            else    -- Cache responses

                case cfi.resp is

                    when CF_WAIT =>
                        hready <= '0';
                        ahbso.hresp  <= HRESP_OKAY;


                    when CF_OKAY =>
                        ahbso.hresp  <= HRESP_OKAY;
                        ahbso.hrdata <= dataToBusFormat(cfi.read_data, r.cfo.addr, r.cfo.transfer_size, endianess);


                    when CF_MISS_ACCEPT =>

                        hready <= '0';
                        ahbso.hresp <= HRESP_SPLIT;
                        rin.split.master(to_integer(unsigned(r.cfo.master))) <= '1';
                        rin.split.delay <= '1';

                        rin.split.read        <= '1';           -- Miss Accept happens only for reading transfers. Note that data must be written in miss data buffer when it gets back
                        rin.split.miss_master <= r.cfo.master;


                    when CF_MISS_DENY =>    -- TODO: SOLVE FETCH PROBLEM
                    
                        hready <= '0';
                        ahbso.hresp <= HRESP_SPLIT;
                        rin.split.master(to_integer(unsigned(r.cfo.master))) <= '1';
                        rin.split.delay <= '1';

                    when others =>

                end case;
            end if;

                       

            -- REQUEST HANDLING

            if(ahbsi.hsel(hsindex) and ahbsi.htrans(1) and ahbsi.hready) = '1' then  -- Cache transfer request detected

                
                rin.cfo.valid          <= '1';
                rin.cfo.addr           <= ahbsi.haddr;
                rin.cfo.seq_access     <= ahbsi.htrans(0) and ahbsi.hburst(0);
                rin.cfo.transfer_size  <= ahbsi.hsize;
                rin.cfo.write          <= ahbsi.hwrite;
                rin.cfo.master         <= ahbsi.hmaster;
                
                if bank_select(ahbsi.hmbsel and active_IO_banks) then
                    rin.cfo.IO_request  <= '1';
                elsif bank_select(ahbsi.hmbsel and active_mem_banks) then
                    rin.cfo.IO_request  <= '0';
                else
                    rin.cfo.valid <= '0'; -- There is no request to either mem or io 
                end if;
                
                if (r.split.data_buffer(to_integer(unsigned(ahbsi.hmaster)))(VALID_BIT)) = '1' and hready = '1' then
                    rin.cfo.valid <= '0';
                    rin.resp_delay <= '1';
                    rin.split.data_buffer(owner_curr)(VALID_BIT) <= '0';
                end if;

            end if;
            
    end process;


    regs : process(clk, cfi, ahbsi, rin, rstn, hready)
    begin

        if rising_edge(clk) then
            if rstn = '0' then
                r.split <= split_init;
            else

                r <= rin;
                if (hready or r.split.delay) = '1' then
                    r.cfo <= rin.cfo;
                else
                    r.cfo <= r.cfo;
                end if;
                
                
                -- Write data is received after the address phase is over
                r.cfo.write_data <= ahbsi.hwdata;
            end if;

        end if;

    end process;


    -- BUS TO CACHE MUX

    with hready select cfo <=
        rin.cfo when '1',
        r.cfo when others;


    -- CONNECT AHB OUTPUT SIGNALS

    ahbso.hconfig <= hconfig;
    ahbso.hindex  <= hsindex;
    ahbso.hirq    <= (others => '0');
    ahbso.hready  <= hready;
    --frontout      <= frontout_temp;
    
    

   

end architecture rtl;
