#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x78c14ddb, "module_layout" },
	{ 0xcf3a662e, "pcmcia_dev_present" },
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x1fedf0f4, "__request_region" },
	{ 0x794a2c29, "kmalloc_caches" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0x28ed8f6c, "pcmcia_enable_device" },
	{ 0x69a358a6, "iomem_resource" },
	{ 0x9f9f8038, "dev_set_drvdata" },
	{ 0x3d793b82, "pcmcia_register_driver" },
	{ 0x91e4100b, "usb_init_urb" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x910c0a54, "dma_set_mask" },
	{ 0x4bdbbdf3, "usb_reset_endpoint" },
	{ 0x38dd7f9f, "pci_disable_device" },
	{ 0x10f80d0d, "i2c_transfer" },
	{ 0x20000329, "simple_strtoul" },
	{ 0xc87c1f84, "ktime_get" },
	{ 0xc73a8cff, "usb_kill_urb" },
	{ 0x64063398, "remove_proc_entry" },
	{ 0x627b5a0e, "usb_reset_configuration" },
	{ 0x7f0e661f, "parport_find_base" },
	{ 0xf34131ee, "__register_chrdev" },
	{ 0x2b3d0d5, "x86_dma_fallback_dev" },
	{ 0x289a8d2c, "driver_for_each_device" },
	{ 0xeae3dfd6, "__const_udelay" },
	{ 0x82de5d3b, "pci_release_regions" },
	{ 0xfb0e29f, "init_timer_key" },
	{ 0x6baae653, "cancel_delayed_work_sync" },
	{ 0x42119286, "mutex_unlock" },
	{ 0x8e58b224, "pci_bus_write_config_word" },
	{ 0x91715312, "sprintf" },
	{ 0x57332891, "pcmcia_request_io" },
	{ 0x7d11c268, "jiffies" },
	{ 0x855c5a7, "mutex_trylock" },
	{ 0xe2d5255a, "strcmp" },
	{ 0x48eb0c0d, "__init_waitqueue_head" },
	{ 0x53a05d4c, "dma_get_required_mask" },
	{ 0x72aa82c6, "param_ops_charp" },
	{ 0xc18b0d86, "pci_set_master" },
	{ 0xd5f2172f, "del_timer_sync" },
	{ 0x2bc95bd4, "memset" },
	{ 0xff7559e4, "ioport_resource" },
	{ 0xe6c8ce04, "device_del" },
	{ 0x72689ab6, "pci_iounmap" },
	{ 0xe988ec11, "dev_err" },
	{ 0xf97456ea, "_raw_spin_unlock_irqrestore" },
	{ 0xc5734835, "current_task" },
	{ 0x37befc70, "jiffies_to_msecs" },
	{ 0xc18304c2, "usb_deregister" },
	{ 0x466cddcf, "__mutex_init" },
	{ 0x50eedeb8, "printk" },
	{ 0xd76505e8, "sysfs_remove_file_from_group" },
	{ 0xf2b6f596, "parport_unregister_device" },
	{ 0x5f3cd55d, "usb_set_interface" },
	{ 0xb6ed1e53, "strncpy" },
	{ 0x2f287f0d, "copy_to_user" },
	{ 0xb4390f9a, "mcount" },
	{ 0x6238261a, "usb_control_msg" },
	{ 0x6c2e3320, "strncmp" },
	{ 0x4158396c, "mutex_lock" },
	{ 0x667f28b7, "class_remove_file" },
	{ 0xc60796c9, "device_create" },
	{ 0xbe2c0274, "add_timer" },
	{ 0xc0d39b77, "dma_release_from_coherent" },
	{ 0x5bf09500, "parport_claim" },
	{ 0x2072ee9b, "request_threaded_irq" },
	{ 0x683bed45, "pcmcia_loop_config" },
	{ 0x184320e9, "class_create_file" },
	{ 0xd7c0a6f, "parport_release" },
	{ 0xcb502c92, "dma_alloc_from_coherent" },
	{ 0xa33c346c, "i2c_del_adapter" },
	{ 0xa8a6f639, "__check_region" },
	{ 0x191d47ed, "usb_submit_urb" },
	{ 0x42c8de35, "ioremap_nocache" },
	{ 0xacacb76a, "pci_bus_read_config_word" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x7025dcf9, "usb_reset_device" },
	{ 0x77edf722, "schedule_delayed_work" },
	{ 0x3bd1b1f6, "msecs_to_jiffies" },
	{ 0x37d2baba, "parport_register_device" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0x4292364c, "schedule" },
	{ 0x86a4889a, "kmalloc_order_trace" },
	{ 0x5fc5869, "usb_clear_halt" },
	{ 0x829b533d, "create_proc_entry" },
	{ 0x7c61340c, "__release_region" },
	{ 0x22baf9f7, "pci_unregister_driver" },
	{ 0x3f9b9190, "kmem_cache_alloc_trace" },
	{ 0x21fb443e, "_raw_spin_lock_irqsave" },
	{ 0x34f9e73, "param_ops_byte" },
	{ 0xe45f60d8, "__wake_up" },
	{ 0xf6ebc03b, "net_ratelimit" },
	{ 0x1d2e87c6, "do_gettimeofday" },
	{ 0x37a0cba, "kfree" },
	{ 0x2e60bace, "memcpy" },
	{ 0x30a06192, "pci_request_regions" },
	{ 0x622fa02a, "prepare_to_wait" },
	{ 0xf59f197, "param_array_ops" },
	{ 0xb27d00dd, "pci_disable_msi" },
	{ 0xdd73a191, "dma_supported" },
	{ 0xedc03953, "iounmap" },
	{ 0x4f0e48d6, "pcmcia_unregister_driver" },
	{ 0x2a75035a, "__pci_register_driver" },
	{ 0x1143bbdc, "usb_register_driver" },
	{ 0x6dcd7881, "class_destroy" },
	{ 0x75bb675a, "finish_wait" },
	{ 0x78a49796, "dev_warn" },
	{ 0x75a1cd0d, "sysfs_add_file_to_group" },
	{ 0x1f2ad4f9, "i2c_bit_add_bus" },
	{ 0xb81960ca, "snprintf" },
	{ 0x8873eaa7, "pci_enable_msi_block" },
	{ 0x8235805b, "memmove" },
	{ 0x9c388c50, "pci_iomap" },
	{ 0x8436f8e3, "param_ops_ushort" },
	{ 0x2e91b7ab, "pcmcia_disable_device" },
	{ 0x6468ccc, "pci_enable_device" },
	{ 0x362ef408, "_copy_from_user" },
	{ 0xc3fe87c8, "param_ops_uint" },
	{ 0x34d76c42, "__class_create" },
	{ 0xec51d6a4, "dev_get_drvdata" },
	{ 0xbb064e58, "dma_ops" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=pcmcia,parport,i2c-algo-bit";

MODULE_ALIAS("usb:v0C72p000Cd*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0C72p000Dd*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0C72p0012d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("usb:v0C72p0011d*dc*dsc*dp*ic*isc*ip*");
MODULE_ALIAS("pci:v0000001Cd00000001sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000003sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000004sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000005sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000006sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000007sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000008sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000002sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd0000000Asv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000009sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000010sv*sd*bc*sc*i*");
MODULE_ALIAS("pci:v0000001Cd00000013sv*sd*bc*sc*i*");
MODULE_ALIAS("pcmcia:m0377c0001f*fn*pfn*pa*pb*pc*pd*");

MODULE_INFO(srcversion, "7ABE64F98B48F2890E35E35");
