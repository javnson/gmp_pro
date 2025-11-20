component system_clk_gen is
    port(
        ref_clk_i: in std_logic;
        rst_n_i: in std_logic;
        outcore_o: out std_logic;
        outglobal_o: out std_logic
    );
end component;

__: system_clk_gen port map(
    ref_clk_i=>,
    rst_n_i=>,
    outcore_o=>,
    outglobal_o=>
);
