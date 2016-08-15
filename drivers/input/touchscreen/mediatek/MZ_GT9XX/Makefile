include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/MZ_GT9XX/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/MZ_GT9XX/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  goodix_tool.o gt9xx_driver.o gt9xx_openshort.o gt9xx_update.o

