ifeq ($(VERBOSE), 1)
  QUIET:=
else 
  QUIET:=@
endif

NUMPROCS:= $(shell nproc)

ifeq ($(INSTALL_PREFIX),)
  $(warning provide an out-of-tree build and install directory. INSTALL_PREFIX=<path>)
endif
INSTALL_DIR = $(abspath $(INSTALL_PREFIX))

ifeq ($(CROSS_GCC_INSTALL),)
  $(warning provide cross-compiler gcc binary or hardcode within makefile here. CROSS_GCC_INSTALL=<path>)
endif
ifeq ($(CROSS_CLANG_INSTALL),)
  $(warning provide cross-compiler clang binary or hardcode within makefile here. CROSS_CLANG_INSTALL=<path>)
endif

ifeq ($(CHERIOT_RTOS_SDK),)
  $(warning provide CHERI rtos sdk include path or hardcode within makefile here. CHERIOT_RTOS_SDK=<path>)
endif
CHERIOT_RTOS_INCLUDE_PATH:= $(CHERIOT_RTOS_SDK)/sdk/include

ifeq ($(SONATA_SDK),)
  $(warning provide SONATA-system software sdk include path or hardcode within makefile here. SONATA_SDK=<path>. Get from git@github.com:lowRISC/sonata-system.git)
endif
SONATA_SDK_INCLUDES:= $(SONATA_SDK)/sw

ifeq ($(UF2_UTILS),)
  $(warning provide path to UF2 utilities or hardcode within makefile here. UF2_UTILS=<path>)
endif

SONATA_APP_INCLUDES = -Iinclude
SONATA_SDK_INCLUDES = $(foreach dir,include,-I$(dir $(realpath $(lastword $(MAKEFILE_LIST))))$(dir))
CHERIOT_SDK_INCLUDES = $(foreach dir, c++-config libc++ platform/generic-riscv, -I$(CHERIOT_RTOS_INCLUDE_PATH)/$(dir))


BOOTLOADER_LINKER_SCRIPT= boot/rv32e-sonata.ld
APPLICATION_LINKER_SCRIPT= app/rv32e-sonata-application.ld


CROSS_CC:= $(CROSS_CLANG_INSTALL)/bin/clang
CROSS_CPP:= $(CROSS_CLANG_INSTALL)/bin/clang++
CROSS_OBJCOPY:=$(CROSS_CLANG_INSTALL)/bin/llvm-objcopy

#Ugly Hack to get GCC version number. This is used to get the path for libgcc
CROSS_GCC_VERSION:=$(shell $(CROSS_GCC_INSTALL)/bin/riscv32-unknown-elf-gcc --version | awk '{if (NR == 1) { print $$3}}')

# custom-linker-script.ld should be replaced for each application
LD_OPTS:= -Wl,-T,custom-linker-script.ld -fuse-ld=lld \
          -L$(CROSS_GCC_INSTALL)/lib/gcc/riscv32-unknown-elf/$(CROSS_GCC_VERSION)  -lgcc


CXX_OPTIONS = -std=c++20                                                            \
              --sysroot=$(CROSS_GCC_INSTALL)/riscv32-unknown-elf                    \
              -march=rv32em_zicsr_zifencei -mabi=ilp32e                             \
              -ffreestanding                                                        \
              -fomit-frame-pointer -fno-builtin -fno-exceptions                     \
              -fno-asynchronous-unwind-tables -fno-c++-static-destructors -fno-rtti \
              -nostdlib

BINARIES:= rv32e-bootloader.bin.vmem \
           rv32e-application.elf.uf2


#####################################
# Build application 
#####################################
APPLICATION_OBJS = app/crt0.o \
                   app/led_walk_baremetal.o

# compile application c++ files 
$(INSTALL_DIR)/build/$(dir $(lastword $(APPLICATION_OBJS)))%.o: $(lastword $(dir $(APPLICATION_OBJS)))%.cpp
	@echo compiling $^ -> $@
	$(QUIET)$(CROSS_CPP) \
                    -O2 \
                    $(CXX_OPTIONS) \
                    $(SONATA_APP_INCLUDES) $(SONATA_SDK_INCLUDES) $(CHERIOT_SDK_INCLUDES) \
		    -c $^ -o $@
        
# compile application assembly files 
$(INSTALL_DIR)/build/$(dir $(lastword $(APPLICATION_OBJS)))%.o: $(lastword $(dir $(APPLICATION_OBJS)))%.S
	$(QUIET)echo compiling $^ -> $@
	$(QUIET)$(CROSS_CPP) \
                    -O2 \
                    $(CXX_OPTIONS) \
                    $(SONATA_APP_INCLUDES) $(SONATA_SDK_INCLUDES) $(CHERIOT_SDK_INCLUDES) \
		    -c $^ -o $@

# link and uf2-encode application elf file
# bootloader loads and executes elf files
$(INSTALL_DIR)/build/bin/rv32e-application.elf.uf2: $(foreach obj,$(APPLICATION_OBJS),$(INSTALL_DIR)/build/$(obj))
	@echo ===========================
	@echo Linking $(basename $@)
	$(QUIET)$(CROSS_CPP)    \
	    $(CXX_OPTIONS)          \
	    $(subst custom-linker-script.ld,$(APPLICATION_LINKER_SCRIPT),$(LD_OPTS)) \
            $(CHERIOT_SDK_INCLUDES) \
	    $^ -o $(basename $@)
	@echo Generating UF2 file : $@
	$(QUIET)$(UF2_UTILS)/utils/uf2conv.py $(basename $@) -f 0x6CE29E60 -b 0x00000000 -co $@




#####################################
# Build bootloader 
# 1. Link ELF file
# 2. Strip ELF file to create a binary
# 3. Generate EEPROM `.vmem` file 
# 4. Synthesize SONATA bitstream SoC for FPGA which includes bootloader in EEPROM
# 5. UF2 encode bitstream file for flashing to bitstream flash
#####################################
BOOTLOADER_OBJS = boot/crt_boot.o \
                  boot/boot_loader.o

# Compile bootloader assembly files 
$(INSTALL_DIR)/build/$(dir $(lastword $(BOOTLOADER_OBJS)))%.o: $(lastword $(dir $(BOOTLOADER_OBJS)))%.cpp
	@echo compiling $^ '->' $@
	$(QUIET)$(CROSS_CPP) \
                    -Os \
                    $(CXX_OPTIONS) \
                    $(SONATA_APP_INCLUDES) $(SONATA_SDK_INCLUDES) $(CHERIOT_SDK_INCLUDES) \
		    -c $^ -o $@

# Compile bootloader c++ files 
$(INSTALL_DIR)/build/$(dir $(lastword $(BOOTLOADER_OBJS)))%.o: $(lastword $(dir $(BOOTLOADER_OBJS)))%.S
	@echo compiling $^ '->' $@
	$(QUIET)$(CROSS_CPP) \
                    -Os \
                    $(CXX_OPTIONS) \
                    $(SONATA_APP_INCLUDES) $(SONATA_SDK_INCLUDES) $(CHERIOT_SDK_INCLUDES) \
		    -c $^ -o $@
        
# 1. Link ELF file
$(INSTALL_DIR)/build/bin/rv32e-bootloader.elf: $(foreach obj,$(BOOTLOADER_OBJS),$(INSTALL_DIR)/build/$(obj))
	@echo ===========================
	@echo Linking $@
	$(QUIET)$(CROSS_CPP)    \
	    $(CXX_OPTIONS)          \
	    $(subst custom-linker-script.ld,$(BOOTLOADER_LINKER_SCRIPT),$(LD_OPTS)) \
            $(CHERIOT_SDK_INCLUDES) \
	    $^ -o $@

# 2. Strip ELF file to create a binary
# 3. Generate EEPROM `.vmem` file 
$(INSTALL_DIR)/build/bin/rv32e-bootloader.bin.vmem: $(INSTALL_DIR)/build/bin/rv32e-bootloader.elf 
	@echo Strip and generate binary  $(basename $@)
	$(QUIET)$(CROSS_OBJCOPY) -O binary $< $(basename $@)
	@echo Convert $(basename $@) to SREC format to include within Sonata EEPROM bitstream
	$(QUIET)srec_cat $(basename $@) -binary -offset 0x0000 -byte-swap 4 -o $(basename $@).vmem -vmem



#####################################
# Create build and install directories
# for out-of-tree build
#####################################
.PHONY: $(INSTALL_DIR)
$(INSTALL_DIR):
	@echo "Creating build and installation directory"
	$(QUIET)mkdir -p $(INSTALL_DIR)/{build,install}
	$(QUIET)mkdir -p $(foreach dir,$(dir $(APPLICATION_OBJS)) $(dir $(BOOTLOADER_OBJS)) bin fpga_bitstream, $(INSTALL_DIR)/build/$(dir))

#####################################
# Generate the install directory 
# Build all applications + bootloader
#####################################
sw_binaries: $(INSTALL_DIR) $(foreach bin,$(BINARIES),$(INSTALL_DIR)/build/bin/$(bin))
	@echo ===========================
	@echo "Software binaries available in $(INSTALL_DIR)/build"


#####################################
# Build FPGA bit-stream + bootloader
# Build Other applications
# 4. Synthesize SONATA bitstream SoC for FPGA which includes bootloader in EEPROM
# 5. UF2 encode bitstream file for flashing to bitstream flash
#####################################
$(INSTALL_DIR)/build/fpga_bitstream/lowrisc_sonata_system_0.bit.uf2: sw_binaries 
	@echo ===========================
	@echo Creating FPGA bitstream 
	$(QUIET)pushd $(INSTALL_DIR) && \
                fusesoc --cores-root=$(SONATA_SDK) run \
                        --target=synth \
                        --setup \
                        --build lowrisc:sonata:system \
                        --SRAMInitFile=$(INSTALL_DIR)/build/bin/rv32e-bootloader.bin.vmem && \
                popd
	$(QUIET)cp $(INSTALL_DIR)/build/lowrisc_sonata_system_0/synth-vivado/lowrisc_sonata_system_0.bit $(basename $@)
	@echo Generating UF2 file : $@
	$(QUIET)$(UF2_UTILS)/utils/uf2conv.py $(basename $@) -f 0x6CE29E6B -b 0x00000000 -co $@


all: $(INSTALL_DIR) $(INSTALL_DIR)/build/fpga_bitstream/lowrisc_sonata_system_0.bit.uf2
	@echo "Build completed in $(INSTALL_DIR)/build"

sw_install:
	@echo ===========================
	@echo Installing applications in $(INSTALL_DIR)/install
	$(QUIET)cp $(INSTALL_DIR)/build/bin/*.uf2 $(INSTALL_DIR)/install

install: sw_install
	@echo ===========================
	@echo Installing FPGA bitstream and applications in $(INSTALL_DIR)/install
	$(QUIET)cp \
                $(INSTALL_DIR)/build/fpga_bitstream/lowrisc_sonata_system_0.bit.uf2 \
                $(INSTALL_DIR)/install

clean:
	@echo cleaning directory $(foreach dir,build install,$(INSTALL_DIR)/$(dir))
	$(QUIET)rm -rf $(INSTALL_DIR)


###########################################
# Generate all required compiler toolchains
###########################################

TOOLCHAIN_INSTALL_DIRS= $(INSTALL_DIR)/toolchains/install/gnu-toolchain \
                        $(INSTALL_DIR)/toolchains/install/llvm-toolchain


$(TOOLCHAIN_INSTALL_DIRS):
	$(QUIET)mkdir -p $@

toolchains: $(TOOLCHAIN_INSTALL_DIRS)
	$(QUIET)mkdir -p $(INSTALL_DIR)/toolchains/{build,install}
	@echo ===========================
	@echo "Fetching Risc-V - GNU (GCC) toolchain"
	$(QUIET)git clone https://github.com/riscv/riscv-gnu-toolchain $(INSTALL_DIR)/toolchains/build/riscv-gnu-toolchain 
	$(QUIET)pushd $(INSTALL_DIR)/toolchains/build/riscv-gnu-toolchain &&  \
                git submodule update --init --recursive && \
                ./configure --prefix=$(INSTALL_DIR)/toolchains/install/gnu-toolchain --with-arch=rv32e --with-abi=ilp32e && \
                popd
	$(QUIET)pushd $(INSTALL_DIR)/toolchains/build/riscv-gnu-toolchain &&  \
                make -j $(NUMPROCS) && \
                make install && \
                popd
	@echo ===========================
	@echo "Installed Risc-V - (GNU) toolchain in $(INSTALL_DIR)/toolchains/install/gnu-toolchain"
	@echo ===========================
	@echo "Fetching RiscV - (LLVM) toolchain"
	$(QUIET)git clone https://github.com/llvm/llvm-project $(INSTALL_DIR)/toolchains/build/riscv-llvm-toolchain 
	$(QUIET)pushd $(INSTALL_DIR)/toolchains/build/riscv-llvm-toolchain  && \
                git submodule update --init --recursive && \
                popd 
	$(QUIET)pushd $(INSTALL_DIR)/toolchains/build && \
	        cmake -S riscv-llvm-toolchain/llvm -B ./riscv-llvm-toolchain-build \
                  --install-prefix $(INSTALL_DIR)/toolchains/install/llvm-toolchain  \
                  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=True -DLLVM_USE_SPLIT_DWARF=True \
                  -DLLVM_OPTIMIZED_TABLEGEN=True -DLLVM_BUILD_TESTS=False \
                  -DLLVM_DEFAULT_TARGET_TRIPLE="riscv32-unknown-elf" \
                  -DDEFAULT_SYSROOT=$(INSTALL_DIR)/toolchains/install/gnu-toolchain/riscv32-unknown-elf \
                  -DLLVM_ENABLE_PROJECTS="clang;clang-tools-extra;lld" \
                  -DLLVM_DISTRIBUTION_COMPONENTS="clang;clangd;lld;llvm-objdump;llvm-objcopy" \
                  -DLLVM_TARGETS_TO_BUILD=RISCV \
                  -DLLVM_ENABLE_UNWIND_TABLES=NO \
                  -G Ninja && \
                popd 
	$(QUIET)pushd $(INSTALL_DIR)/toolchains/build && \
                cmake --build ./riscv-llvm-toolchain-build && cmake --install ./riscv-llvm-toolchain-build && \
                popd
	@echo ===========================
	@echo "Installed Risc-V - (LLVM) toolchain in $(INSTALL_DIR)/toolchains/install/llvm-toolchain"
	@echo ===========================
	@echo "Fetching UF2 utilities"
	$(QUIET)git clone https://github.com/microsoft/uf2.git $(INSTALL_DIR)/toolchains/install/uf2
	@echo ===========================
	@echo "Installed UF2 utils in $(INSTALL_DIR)/toolchains/install/uf2"
	@echo ===========================
	@echo Toolchains installed in $(INSTALL_DIR)/toolchains/install


help:
	@echo "==================================="
	@echo "Helper makefile to build RV32E software for Sonata"
	@echo "==================================="
	@echo Makefile targets:
	@echo 1. sw_binaries: Generates bootloader and application
	@echo 2. all: Generates FPGA-bitstream with included bootloader and applications
	@echo '        Xilinx fpga-synthesis tool (vivado) should be found in PATH'
	@echo '        Should be executed within Python venv found in SONATA_SDK (see below).'
	@echo '        In the shell for this makefile execute source SONATA_SDK/.venv/bin/activate'
	@echo 3. sw_install: installs bootloader and application into INSTALL_PREFIX/install
	@echo '     should have been preceded by make all or make sw_binaries with relevant PATHS'
	@echo 4. install: installs software components from sw_install as well as
	@echo '     FPGA-bitstream from make all. Should be preceded by make all with relevant PATHS'
	@echo 5. tool_chains: downloads and installs required toolchains into INSTALL_PREFIX/toolchains
	@echo 6. clean: cleans out INSTALL_PREFIX directory
	@echo '     Specify NUMPROCS=<number-of-cores>. defaults to cores reported by nproc'
	@echo 7. help: Shows this help message
	@echo "=================================================================="
	@echo "Variables to be provided on Command-line or hard-coded in Makefile"
	@echo "If toolchains are not already present use make toolchains with"
	@echo "     " 'INSTALL_PREFIX=<out-of-tree-path>' and use this path for clang+gcc+uf2 
	@echo "     " Xilinx-Vivado synthesis tools have to be installed separately
	@echo "=================================================================="
	@echo 'VERBOSE=1 to see execution of each command'
	@echo 'INSTALL_PREFIX=<out-of-tree-path>'
	@echo 'CROSS_GCC_INSTALL=<path-to installation of riscv32-unknown-elf-gcc>'
	@echo '      The compiler filename should be included'
	@echo 'CROSS_CLANG_INSTALL=<path-to installation of riscv32e clang compiler>'
	@echo '      The compiler filename should be included'
	@echo 'CHERIOT_RTOS_SDK=<path to cheriot-rtos-sdk>. The git repo is available at:'
	@echo '      <git@github.com:microsoft/cheriot-rtos>'
	@echo 'SONATA_SDK=<path-to-sonata-FPGA-RTL>. The git repo is available at:'
	@echo '      <git@github.com:lowRISC/sonata-system.git>'
	@echo '      For first time set-up of this sonata-system git-repo execute:'
	@echo '        cd <path-to-sonata-FPGA-RTL>'
	@echo '        python3 -m venv .venv'
	@echo '        source .venv/bin/activate'
	@echo '        pip3 install -r python-requirements.txt'
	@echo '      For subsequent executions in a different shell'
	@echo '        pushd <path-to-sonata-FPGA-RTL> && source .venv/bin/activate && popd'
	@echo 'UF2_UTILS=<path-to-UF2-utils>. git repo available at: ' 
	@echo '      <git@github.com:microsoft/uf2.git>'

dtool:
	@echo cross-gcc = $(CROSS_GCC_INSTALL)
	@echo base dir cross-gcc = $(dir $(CROSS_GCC_INSTALL))
	@echo second derivative  dir cross-gcc = $(dir $(dir $(CROSS_GCC_INSTALL)))
