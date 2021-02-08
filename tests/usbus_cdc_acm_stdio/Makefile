BOARD ?= samr21-xpro
include ../Makefile.tests_common

USEMODULE += stdio_cdc_acm
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

USB_VID ?= $(USB_VID_TESTING)
USB_PID ?= $(USB_PID_TESTING)

include $(RIOTBASE)/Makefile.include
