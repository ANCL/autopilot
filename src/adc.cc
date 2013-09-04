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
#include "heli.h"

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

//	debug() << "CpuMemTranslation: " << std::hex << (long unsigned int)pci_info.CpuMemTranslation;
//	debug() << "is mem" << PCI_IS_MEM(pci_info.CpuBaseAddress[0]);
//	debug() << "Device Bar0 Address: " << std::hex << (long unsigned int)(pci_info.CpuBaseAddress[0]) << "and the size is" << std::dec << pci_info.BaseAddressSize[0];

	// map bar0 into process memory - now the registered can be read and written as described in adc docs
	adc_bar0 = (char *) mmap_device_memory(NULL, pci_info.BaseAddressSize[0], PROT_READ|PROT_WRITE, 0, PCI_MEM_ADDR(pci_info.CpuBaseAddress[0]));

//	debug() << "Command Reg: " << std::hex << (uint32_t)*(adc_bar0 + 8);
	// compute the address of the command register
	uint32_t *cmd_reg = (uint32_t*)(adc_bar0 + 8);
	// set the card to sample ch0 single-ended on each ADC
	uint32_t batt_voltage_cmd = 0x8c8c8c8c;
	// write the settings to the command register
	LogFile::getInstance()->logHeader(heli::Log_Battery_Voltage, "Ampro, Rx, Kontron");
	LogFile::getInstance()->logData(heli::Log_Battery_Voltage,std::vector<double>());
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
	struct sigevent         event;
	struct itimerspec       itime;
	timer_t                 timer_id;
	/* ChannelCreate() func. creates a channel that is owned by the process (and isn't bound to the creating thread). */
	int chid = ChannelCreate(0);

	event.sigev_notify = SIGEV_PULSE;

	/* Threads wishing to connect to the channel identified by 'chid'(channel id) by ConnectAttach() func.
		 Once attached thread can MsgSendv() & MsgSendPulse() to enqueue messages & pulses on the channel in priority order. */
	event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0,
			chid,
			_NTO_SIDE_CHANNEL, 0);
	event.sigev_priority = heli::adc_send_priority;
	event.sigev_code = heli::adc_pulse_code;
	timer_create(CLOCK_REALTIME, &event, &timer_id);

	itime.it_value.tv_sec = 1;
	itime.it_value.tv_nsec = 0;
	itime.it_interval.tv_sec = 1;
	itime.it_interval.tv_nsec = 0;
	timer_settime(timer_id, 0, &itime, NULL);

	_pulse pulse;

	for(;;)
	{
		MsgReceivePulse(chid, &pulse, sizeof(pulse), NULL);
		ADC* adc = getInstance();
		uint16_t* adc_bar0 = (uint16_t*)adc->adc_bar0;
		std::vector<double> data;
		double adc0 = (*adc_bar0)*10.0/65536;
		adc->set_ampro(adc0);
		data.push_back(adc0);
		double adc1 = (*(adc_bar0+1))*10.0/65536;
		adc->set_rx(adc1);
		data.push_back(adc1);
		// this one requires the voltage divider to be inverted
		double adc2 = (*(adc_bar0+2))*10.0/65536*(6.8e3+3.9e3)/6.8e3;
		adc->set_kontron(adc2);
		data.push_back(adc2);
		LogFile::getInstance()->logData(heli::Log_Battery_Voltage, data);
//		debug() << "ADC0:" << adc0 << "ADC1:" << adc1 << "ADC2:" << adc2;
	}

}
