all:
	git clone https://github.com/SpinalHDL/openocd_riscv.git
	cd openocd_riscv && git checkout 50f74cd
	mkdir -p build
	cp patch/configure openocd_riscv/configure && cp patch/configure.ac openocd_riscv/configure.ac && cp patch/interfaces.c openocd_riscv/src/jtag/interfaces.c && cp patch/commands.h openocd_riscv/src/jtag/commands.h  && cp patch/tcl.c openocd_riscv/src/jtag/tcl.c  && cp patch/Makefile.am openocd_riscv/src/jtag/drivers/Makefile.am && cp patch/emulate.c openocd_riscv/src/jtag/drivers/emulate.c  && cd openocd_riscv && ./bootstrap && ./configure --prefix=${PWD}/build --enable-ftdi --enable-dummy  --enable-emulate && make -j8 && make install
	/opt/Xilinx/Vivado/2020.1/bin/vivado -nojournal -nolog -mode batch -source ./run.tcl 
hdl:
	/opt/Xilinx/Vivado/2020.1/bin/vivado -nojournal -nolog -mode batch -source ./run.tcl

toolchain:
	-git clone https://github.com/riscv-collab/riscv-gnu-toolchain.git
	cd riscv-gnu-toolchain && git checkout 170a9a3 && ./configure --prefix=$(PWD)/build --with-arch=rv32i  --with-abi=ilp32  && make

clean:
	-rm -rf build
	
.PHONY: all clean hdl toolchain
