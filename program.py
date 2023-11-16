from jtag import JTAG
bin_file = './build/project/vexriscv.runs/impl_1/top.bin'
print("Initializing FTDI connection")
jtag_ = JTAG(0x0403, 0x6010)
print("Reconfiguring the FPGA")
jtag_.program(bin_file)
print("Done")
jtag_.free_dev()