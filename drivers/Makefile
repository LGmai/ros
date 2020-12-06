obj-y	+= drivers/

export CONFIG_SRF04 := m

export INCLUDEDIR :=  $(dir $(abspath $(lastword $(MAKEFILE_LIST))))include
$(info $$INCLUDEDIR is [${INCLUDEDIR}])
LINUXINCLUDE += -I$(INCLUDEDIR)
#CC += -I$(CURDIR)/../rpi-receiver-linux-rpi-4.19.y/include

all: clean compile

compile:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install