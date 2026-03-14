// license:BSD-3-Clause
// copyright-holders:Martin Sternevald
//
// RS-232 device that bridges an emulated serial port to host stdin/stdout.
//
// Example usage:
//   mame h19 -dte console

#ifndef MAME_BUS_RS232_CONSOLE_H
#define MAME_BUS_RS232_CONSOLE_H

#pragma once

#include "rs232.h"
#include "diserial.h"

#include <termios.h>

class console_device : public device_t,
					   public device_serial_interface,
					   public device_rs232_port_interface
{
public:
	console_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_txd(int state) override { device_serial_interface::rx_w(state); }

	void update_serial(int state);

protected:
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	TIMER_CALLBACK_MEMBER(update_queue);

	required_ioport m_rs232_txbaud;
	required_ioport m_rs232_rxbaud;
	required_ioport m_rs232_databits;
	required_ioport m_rs232_parity;
	required_ioport m_rs232_stopbits;
	required_ioport m_rs232_txdelay;

	emu_timer *m_timer_poll;
	struct termios m_saved_termios;
	bool m_termios_saved;
};

DECLARE_DEVICE_TYPE(CONSOLE, console_device)

#endif // MAME_BUS_RS232_CONSOLE_H
