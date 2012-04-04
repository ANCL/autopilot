#=======
# This is an automatically generated record.
# The area between QNX Internal Start and QNX Internal End is controlled by
# the QNX IDE properties.

ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

USEFILE=

# Next lines are for C++ projects only

EXTRA_SUFFIXES+=cxx cpp

LDFLAGS+=-lang-c++ -lgsl -lgslcblas

VFLAG_g=-gstabs+

EXTRA_INCVPATH+=$(PROJECT_ROOT)/../extern  \
	$(PROJECT_ROOT)/../../mavlink/include/ualberta

LIBS+=boost_thread boost_date_time -Bstatic  \
	boost_filesystem -Bdynamic boost_system socket

NAME=autopilot

EXTRA_LIBVPATH+=$(PROJECT_ROOT)/../lib

EXTRA_SRCVPATH+=$(PROJECT_ROOT)/gx3 $(PROJECT_ROOT)/qgclink  \
	$(PROJECT_ROOT)/control $(PROJECT_ROOT)/novatel

include $(MKFILES_ROOT)/qmacros.mk
ifndef QNX_INTERNAL
QNX_INTERNAL=$(PROJECT_ROOT)/.qnx_internal.mk
endif
include $(QNX_INTERNAL)

include $(MKFILES_ROOT)/qtargets.mk

OPTIMIZE_TYPE_g=none
OPTIMIZE_TYPE=$(OPTIMIZE_TYPE_$(filter g, $(VARIANTS)))

