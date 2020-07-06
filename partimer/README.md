# partimer Test Program

Based on the TIMERs_parallelsynchro example program from the GD32VF103
firmware library.

The program retrieves the mcycleh/mcycle counter via SDK function
get_cycle_value(). Note that the machine cycle counter must be
explicitly enabled in main(). After startup it is disabled.
