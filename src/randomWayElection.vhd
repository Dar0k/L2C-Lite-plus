library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- MODULE DESCRIPTION --

-- Prerrequisites:
-- Number of ways must be greater than 2
--
-- Functionality;
-- 
-- 
--
--
-- END DESCRIPTION --


entity randomWayElection is
    Generic(
        ways        : positive;      -- Number of ways of the cache
        numBitmasks : positive       -- Number of bitmasks
         
    );

    Port (
        clk            : in std_ulogic;                                                               -- Clock signal
        rstn           : in std_ulogic;                                                               -- Reset signal
        enable         : in std_ulogic;                                                               -- Enable signal 
        write          : in std_ulogic;                                                               -- 1 to write, 0 to read
    	renable	       : in std_ulogic;								                                  -- Returns bitmask selected 
        bitmask_select : in natural range 0 to numBitmasks - 1;                                       -- bitmask selected
        bitmask_write  : in std_logic_vector(ways-1 downto 0);                                        -- write data 
        wayElected     : out natural range 0 to ways;                                                 -- way chosen to be replaced
	    bitmask_read   : out std_logic_vector(ways-1 downto 0)					       
        
     );
end randomWayElection;


architecture rtl  of randomWayElection is

-- Type declaration

type bitmask_array_t is array (0 to numBitmasks-1) of std_logic_vector(ways - 1 downto 0);
type lastWay_t       is array (0 to numBitmasks-1) of integer range 0 to ways;


-- Signal declaration

signal bitmask_array : bitmask_array_t          := (others => (others => '1'));
signal randomNumber  : integer                  := 0;
signal lfsr          : std_logic_vector(0 to 7) := (others => '1');
signal lastWay       : lastWay_t                := (others => ways);

begin

waySelection: process(clk, rstn, enable)
variable wayPointer : integer := 0;
begin
    
    if rstn = '0' then
        bitmask_array <= (others => (others => '1'));
        lastWay       <= (others => ways);
   
    elsif rising_edge(clk) and enable = '1' then
        
        if write = '1' then 

            -- WRITES BITMASK AND SETS LAST WAY TO UNDEFINED
            bitmask_array(bitmask_select) <= bitmask_write;
            lastWay(bitmask_select)       <= ways;

        else 

            -- UPDATES POINTER POSITION

            wayPointer := wayPointer + randomNumber;
            wayPointer := wayPointer rem ways;
             
            -- UPDATES LAST WAY IF POINTER IS SELECTING VALID WAY
            for i in 0 to numBitmasks - 1 loop 

                if bitmask_array(i)(wayPointer) = '1' then
                    lastWay(i) <= wayPointer;
                end if;

            end loop;
 
        end if;
           
    end if;

end process;


-- GENERATES RANDOM NUMBER WITH 8-BIT LFSR
randomNumberGen: process(clk, lfsr)
begin

    if(rising_edge(clk)) then

	    for i in 1 to 7 loop
        	lfsr(i) <= lfsr(i-1);
	    end loop;

    end if;
    
    lfsr(0)      <= lfsr(3) xor (lfsr(4) xor (lfsr(5) xor lfsr(7)));    
    randomNumber <= to_integer(unsigned(lfsr(0 to 7)));

end process;

-- READING OF LAST WAY AND BITMASK IS DONE ASYNCHRONOUSLY 

--wayElected       <= lastWay(bitmask_select) when write = '0' else ways; 
--bitmask_read     <= bitmask_array(bitmask_select) when renable = '1' else (others => '0');


end architecture rtl;
