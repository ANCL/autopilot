/**************************************************************************
 * Copyright 2013 Bryan Godbolt
 *
 * This file is part of ANCL Autopilot.
 *
 *     ANCL Autopilot is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     ANCL Autopilot is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with ANCL Autopilot.  If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/

/* Project Headers */
#include "adc.h"
#include "Debug.h"

/* Boost Headers */
#include <boost/function.hpp>

/* QNX Headers */
#include <hw/pci.h>
#include <sys/mman.h>

ADC* ADC::_instance = NULL;
boost::mutex ADC::_instance_lock;

ADC* ADC::getInstance()
{
	boost::mutex::scoped_lock lock(_instance_lock);
	if (!_instance)
		_instance = new ADC;
	return _instance;
}

ADC::ADC()
 : pci_handle(0)
{

	// attach this process to the pci system in qnx
	pci_handle = pci_attach(0);
	// initialize a data structure for the pci device info
	pci_dev_info pci_info;
	memset(&pci_info, 0, sizeof(pci_info));
	int pidx = 0;
	pci_info.VendorId = 0x12c4;
	pci_info.DeviceId =	0x1201;
	// attach this process to the connecttech adc and get address info
	void * hdl = pci_attach_device(NULL, PCI_INIT_ALL, pidx, &pci_info);

	if (hdl == NULL)
	{
		critical() << "Could not attach to ConnectTech ADC";
		return;
	}

	debug() << "CpuMemTranslation: " << std::hex << (long unsigned int)pci_info.CpuMemTranslation;
	debug() << "is mem" << PCI_IS_MEM(pci_info.CpuBaseAddress[0]);
	debug() << "Device Bar0 Address: " << std::hex << (long unsigned int)(pci_info.CpuBaseAddress[0]) << "and the size is" << std::dec << pci_info.BaseAddressSize[0];

	// map bar0 into process memory - now the registered can be read and written as described in adc docs
	adc_bar0 = (char *) mmap_device_memory(NULL, pci_info.BaseAddressSize[0], PROT_READ|PROT_WRITE, 0, PCI_MEM_ADDR(pci_info.CpuBaseAddress[0]));

	debug() << "Command Reg: " << std::hex << (uint32_t)*(adc_bar0 + 8);
	// compute the address of the command register
	uint32_t *cmd_reg = (uint32_t*)(adc_bar0 + 8);
	// set the card to sample ch0 single-ended on each ADC
	uint32_t batt_voltage_cmd = 0x8c8c8c8c;
	// write the settings to the command register
	*cmd_reg = batt_voltage_cmd;
	update_thread = boost::thread(update());
}

ADC::~ADC()
{
	// detach this process from the pci system (also detaches any devices)
	pci_detach(pci_handle);
}

void ADC::update::operator()()
{
	uint16_t* adc_bar0 = (uint16_t*)getInstance()->adc_bar0;
	double adc0 = (*(uint16_t*)adc_bar0)*10.0/65536;
	double adc1 = (*(uint16_t*)(adc_bar0+2))*10.0/65536;
	// this one requires the voltage divider to be inverted
	double adc2 = (*(uint16_t*)(adc_bar0+4))*10.0/65536*(6.8e3+3.9e3)/6.8e3;
	debug() << "ADC0:" << adc0 << "ADC1:" << adc1 << "ADC2:" << adc2;

}
