#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
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
	{ 0x683cfe8d, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0xcf77804a, __VMLINUX_SYMBOL_STR(cdev_del) },
	{ 0x7875fbf2, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x2ef2a021, __VMLINUX_SYMBOL_STR(pci_bus_read_config_byte) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x1e90132d, __VMLINUX_SYMBOL_STR(cdev_init) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x43a53735, __VMLINUX_SYMBOL_STR(__alloc_workqueue_key) },
	{ 0x70d7563d, __VMLINUX_SYMBOL_STR(pci_disable_device) },
	{ 0xc1fe759a, __VMLINUX_SYMBOL_STR(set_page_dirty_lock) },
	{ 0x6dc0c9dc, __VMLINUX_SYMBOL_STR(down_interruptible) },
	{ 0x9dc5772e, __VMLINUX_SYMBOL_STR(device_destroy) },
	{ 0xe4d612d5, __VMLINUX_SYMBOL_STR(put_zone_device_page) },
	{ 0xfdec9ed1, __VMLINUX_SYMBOL_STR(pci_release_regions) },
	{ 0x7485e15e, __VMLINUX_SYMBOL_STR(unregister_chrdev_region) },
	{ 0xfb0456ee, __VMLINUX_SYMBOL_STR(pci_bus_write_config_word) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x7d11c268, __VMLINUX_SYMBOL_STR(jiffies) },
	{ 0x4f8b5ddb, __VMLINUX_SYMBOL_STR(_copy_to_user) },
	{ 0x65d462c4, __VMLINUX_SYMBOL_STR(pci_set_master) },
	{ 0x241e37b1, __VMLINUX_SYMBOL_STR(pci_restore_state) },
	{ 0xa31df149, __VMLINUX_SYMBOL_STR(pci_iounmap) },
	{ 0x2270cd45, __VMLINUX_SYMBOL_STR(current_task) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xa818c158, __VMLINUX_SYMBOL_STR(pci_bus_write_config_dword) },
	{ 0x8c03d20c, __VMLINUX_SYMBOL_STR(destroy_workqueue) },
	{ 0x7b3a1d4c, __VMLINUX_SYMBOL_STR(device_create) },
	{ 0x2072ee9b, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xd6b8cb9f, __VMLINUX_SYMBOL_STR(up_write) },
	{ 0x7cd20baf, __VMLINUX_SYMBOL_STR(down_write) },
	{ 0x42160169, __VMLINUX_SYMBOL_STR(flush_workqueue) },
	{ 0x8d3bb837, __VMLINUX_SYMBOL_STR(pci_find_capability) },
	{ 0x161650ac, __VMLINUX_SYMBOL_STR(cdev_add) },
	{ 0x42c8de35, __VMLINUX_SYMBOL_STR(ioremap_nocache) },
	{ 0x12f103c2, __VMLINUX_SYMBOL_STR(pci_bus_read_config_word) },
	{ 0x47360bd7, __VMLINUX_SYMBOL_STR(pci_bus_read_config_dword) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x969da5c3, __VMLINUX_SYMBOL_STR(get_user_pages) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0xf3fdd38, __VMLINUX_SYMBOL_STR(pci_unregister_driver) },
	{ 0xccca000a, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x6ffd2780, __VMLINUX_SYMBOL_STR(pci_bus_write_config_byte) },
	{ 0x1e047854, __VMLINUX_SYMBOL_STR(warn_slowpath_fmt) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x69acdf38, __VMLINUX_SYMBOL_STR(memcpy) },
	{ 0xeca0f12f, __VMLINUX_SYMBOL_STR(pci_request_regions) },
	{ 0xaeab1f68, __VMLINUX_SYMBOL_STR(send_sig_info) },
	{ 0x78e739aa, __VMLINUX_SYMBOL_STR(up) },
	{ 0xbef9803c, __VMLINUX_SYMBOL_STR(__pci_register_driver) },
	{ 0xcbdc8014, __VMLINUX_SYMBOL_STR(class_destroy) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0xda9d58fa, __VMLINUX_SYMBOL_STR(pci_enable_device) },
	{ 0x4f6b400b, __VMLINUX_SYMBOL_STR(_copy_from_user) },
	{ 0x47b440b5, __VMLINUX_SYMBOL_STR(__class_create) },
	{ 0x88db9f48, __VMLINUX_SYMBOL_STR(__check_object_size) },
	{ 0x29537c9e, __VMLINUX_SYMBOL_STR(alloc_chrdev_region) },
	{ 0xccfaf4db, __VMLINUX_SYMBOL_STR(__put_page) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0xfbe6c3db, __VMLINUX_SYMBOL_STR(pci_save_state) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("pci:v00001172d0000AB00sv00001172sd00000004bc*sc*i*");

MODULE_INFO(srcversion, "8BD7E2FDC29E1C58DE4A2FE");
