/*
 * STM32F4xx SoC
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
#include "qemu-common.h"
#include "hw/arm/arm.h"
#include "exec/address-spaces.h"
#include "hw/arm/arm.h"
#include "hw/arm/armv7m.h"
#include "cpu.h"

#define FLASH_BASE_ADDRESS 0x08000000
#define FLASH_SIZE (1 * 1024 * 1024)
#define SRAM_BASE_ADDRESS 0x20000000
#define SRAM_SIZE (128 * 1024)

#include "hw/misc/stm32f2xx_syscfg.h"
#include "hw/timer/stm32f2xx_timer.h"
#include "hw/adc/stm32f2xx_adc.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/or-irq.h"
#include "hw/arm/arm.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/stm32fxxx.h"


#define TYPE_STM32FXXX_SOC "stm32f407-soc"
#define STM32FXXX_SOC(obj) \
    OBJECT_CHECK(struct stm32f407_soc, (obj), TYPE_STM32FXXX_SOC)

#define NAME_SIZE 20	

struct stm32f407_soc { // define the soc components

	SysBusDevice parent_obj; //private
	
	ARMv7MState armv7m;
	SysBusDevice *syscfg;
	
	SysBusDevice *rcc;
	SysBusDevice *gpio[STM32FXXX_NUM_GPIOS];
	
	char *cpu_type;
	MemoryRegion mmio;
	
	struct stm32fxxx_state state;
	
};	

static void stm32f407_rogue_mem_write(void *opaque, hwaddr addr, uint64_t val64, unsigned int size) {
	// to check the out of memory write accesses
	printf("Out of memory write access to %08x\n",(uint32_t)addr);
}

static uint64_t stm32f407_rogue_mem_read(void *opaque, hwaddr addr, unsigned int size) {
	// to check the out of memory read accesses
	printf("Out of memory read access to %08x\n",(uint32_t)addr);
	return 0;

}
static const MemoryRegionOps _stm32_rogue_mem_ops  = {
.read = stm32f407_rogue_mem_read, // initializes as readable area
.write = stm32f407_rogue_mem_write, // initializes as writeable area
.endianness = DEVICE_NATIVE_ENDIAN,	
};

static int stm32_realize_peripheral (ARMv7MState *cpu, SysBusDevice *dev, hwaddr base, unsigned int irqnr, Error **errp){
	// defines the peripheral section
	Error *err = NULL;
	 
	object_property_set_bool (OBJECT(dev), true, "realized", &err);
	
	if (err!= NULL){
		error_propagate(errp,err);
		return -1;
	}
	
	sysbus_mmio_map(dev, 0, base);
	sysbus_connect_irq(dev, 0, qdev_get_gpio_in(DEVICE(cpu), irqnr));
	
	return 0;
}


static void stm32f407_soc_initfunc(Object *obj){
	struct stm32f407_soc *s = STM32FXXX_SOC(obj);
	int i;
	char name [NAME_SIZE];
	
	//catching all the corrupted accesses to memory
	
	memory_region_init_io(&s->mmio, obj, &_stm32_rogue_mem_ops, s, "stm32f407-soc", 0xffffffff); //out of 4GB memory
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio); // addresses accessible by sysbus
	object_initialize(&s->armv7m, sizeof(s->armv7m), TYPE_ARMV7M); // initialize the SOC
	qdev_set_parent_bus(DEVICE(&s->armv7m), sysbus_get_default()); // puts device on the bus
	s->syscfg = sysbus_create_child_obj(obj, name, "stm32f2xx-syscfg");
	
	DeviceState *rcc = qdev_create(NULL, "stm32f407xx_rcc");
    qdev_prop_set_uint32(rcc, "osc_freq", 8000000); // not sure of the value
    qdev_prop_set_uint32(rcc, "osc32_freq", 32000);
    qdev_init_nofail(rcc);
    object_property_add_child(obj, "rcc", OBJECT(rcc), NULL);
    s->rcc = SYS_BUS_DEVICE(rcc);
	
	for (i = 0; i < STM32FXXX_NUM_GPIOS; i++) {
        snprintf(name, NAME_SIZE, "GPIO%c", 'A' + i);
        s->gpio[i] = sysbus_create_child_obj(obj, name, "stm32f407xx-gpio");
        qdev_prop_set_uint8(DEVICE(s->gpio[i]), "port_id", i);
        qdev_prop_set_ptr(DEVICE(s->gpio[i]), "state", &s->state);
    }
	
}

static void stm32f407_soc_realize(DeviceState *dev_soc, Error **errp) {
	struct stm32f407_soc *s = STM32FXXX_SOC(dev_soc);
    Error *err = NULL;
    //int i;
	
	DeviceState *armv7m = DEVICE(&s->armv7m);
	
	MemoryRegion *system_memory = get_system_memory();
	MemoryRegion *sram = g_new(MemoryRegion, 1);
	MemoryRegion *flash =g_new(MemoryRegion, 1);
	MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
	
	memory_region_init_ram(flash, NULL, "STM32F407.flash", FLASH_SIZE, &error_fatal);
    memory_region_init_alias(flash_alias, NULL, "STM32F407.flash.alias", flash, 0, FLASH_SIZE);

    vmstate_register_ram_global(flash);

    memory_region_set_readonly(flash, true);
    memory_region_set_readonly(flash_alias, true);

    memory_region_add_subregion(system_memory, FLASH_BASE_ADDRESS, flash);
    memory_region_add_subregion(system_memory, 0, flash_alias);

    memory_region_init_ram(sram, NULL, "STM32F407.sram", SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, SRAM_BASE_ADDRESS, sram);

    memory_region_add_subregion_overlap(system_memory, 0, &s->mmio, -1);

	// init the cpu on the soc
	qdev_prop_set_uint32(armv7m, "num-irq", 96);
    qdev_prop_set_string(armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    object_property_set_link(OBJECT(&s->armv7m), OBJECT(get_system_memory()),
                                     "memory", &error_abort);

    object_property_set_bool(OBJECT(&s->armv7m), true, "realized", &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
	
	if(stm32_realize_peripheral(&s->armv7m, s->rcc, 0x40023800, 5, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->syscfg, 0x40013800, 91, errp) < 0) return;
		
	if(stm32_realize_peripheral(&s->armv7m, s->gpio[0], 0x40020000, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[1], 0x40020400, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[2], 0x40020800, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[3], 0x40020C00, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[4], 0x40021000, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[5], 0x40021400, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[6], 0x40021800, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[7], 0x40021C00, 0, errp) < 0) return;
    if(stm32_realize_peripheral(&s->armv7m, s->gpio[8], 0x40022000, 0, errp) < 0) return;
}



static Property stm32f407_soc_properties[] = {
	DEFINE_PROP_STRING("cpu-type", struct stm32f407_soc, cpu_type),
	DEFINE_PROP_END_OF_LIST(),
	
};

static void stm32f407_soc_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	dc->realize = stm32f407_soc_realize;
	dc->props = stm32f407_soc_properties;
	
}


static const TypeInfo stm32f407_soc_info = { // define the device with name, bus, init function
	.name	 		= TYPE_STM32FXXX_SOC,
	.parent 		= TYPE_SYS_BUS_DEVICE,
	.instance_size	= sizeof(struct stm32f407_soc),
	.instance_init 	= stm32f407_soc_initfunc,
	.class_init 	= stm32f407_soc_class_init,
	
};

static void stm32f407_soc_types(void)
{
	type_register_static(&stm32f407_soc_info); // instantiate a info class of type type_register
}

type_init(stm32f407_soc_types)

