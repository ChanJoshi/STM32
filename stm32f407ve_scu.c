/*
 * STM32f407 Machine Model
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * Copyright (c) 2018 Martin SchrĂ¶der <mkschreder.uk@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "qemu/error-report.h"
#include "hw/arm/arm.h"
#include "hw/arm/armv7m.h"
#include "exec/address-spaces.h"

struct stm32f407ve_scu {
	DeviceState *soc;
	struct arm_boot_info boot_info;
	
};

static void stm32f407ve_scu_init(MachineState *machine) { //create a space for the machine kernel
	struct stm32f407ve_scu *s =g_new0(struct stm32f407ve_scu, 1);
	
	if (!machine->kernel_filename){
		fprintf(stderr," Guest image is missing (use -kernel)\n");
		exit(1);
	}
	
	s->soc = qdev_create(NULL, "stm32f407-soc");
	qdev_prop_set_string (s->soc, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4")); // assign cortex-m4 as the processor
	object_property_set_bool(OBJECT(s->soc), true, "realized", &error_fatal);
	
	MemoryRegion *sram =g_new(MemoryRegion,1); //creates new memory region
	memory_region_init_ram(sram, NULL, "scu.sram", 1024 * 128, &error_fatal); // ram area defined with size
	vmstate_register_ram_global(sram);
	memory_region_add_subregion(get_system_memory(), 0x90000000, sram);

	
	//loads kernel of maximum size 2MB
	armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename, 2 * 1024 * 1024);  
    rom_check_and_register_reset();
    qemu_devices_reset();	
	
}
static void stm32f407ve_scu_machine_init(MachineClass *mc) //defines the machine init struct and description
{
	mc->desc = "STM32F407 board with RAM";
	mc->init = stm32f407ve_scu_init;
}
DEFINE_MACHINE("stm32f407ve_scu", stm32f407ve_scu_machine_init) //machine is defined with initialization clas