// license:BSD-3-Clause
// copyright-holders:Martin Sternevald
//
// RS-232 device that bridges an emulated serial port to host stdin/stdout.
//
// Example usage:
//   mame h19 -dte console
//
// stdin is placed in raw/no-echo mode so every keystroke is delivered
// immediately.  The original terminal state is restored when MAME exits.
//
// Many vintage systems process incoming serial characters while simultaneously
// transmitting their echo.  At full baud rate, back-to-back characters can
// arrive faster than the CPU services the UART receive register, causing
// overflow. While typing into the terminal this is never a problem but
// copy-paste is a sure way of triggering it.
//
// The "Char Delay" option inserts a pause after each transmitted
// character, giving the target system time to process it before the next one
// arrives.  A delay of 10–20 ms is usually sufficient, simulates human input
// and is applied by default.

#include "emu.h"
#include "console.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


DEFINE_DEVICE_TYPE(CONSOLE, console_device, "console", "RS-232 Console (stdin/stdout)")

// Lookup table: port value → inter-character delay in milliseconds.
static constexpr int TXDELAY_MS[] = { 0, 1, 2, 5, 10, 20, 50, 100 };

console_device::console_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	device_t(mconfig, CONSOLE, tag, owner, clock),
	device_serial_interface(mconfig, *this),
	device_rs232_port_interface(mconfig, *this),
	m_rs232_txbaud(*this, "RS232_TXBAUD"),
	m_rs232_rxbaud(*this, "RS232_RXBAUD"),
	m_rs232_databits(*this, "RS232_DATABITS"),
	m_rs232_parity(*this, "RS232_PARITY"),
	m_rs232_stopbits(*this, "RS232_STOPBITS"),
	m_rs232_txdelay(*this, "RS232_TXDELAY"),
	m_timer_poll(nullptr),
	m_termios_saved(false)
{
}

static INPUT_PORTS_START(console)
	PORT_RS232_BAUD("RS232_TXBAUD", RS232_BAUD_9600, "TX Baud", console_device, update_serial)
	PORT_RS232_BAUD("RS232_RXBAUD", RS232_BAUD_9600, "RX Baud", console_device, update_serial)
	PORT_RS232_DATABITS("RS232_DATABITS", RS232_DATABITS_8, "Data Bits", console_device, update_serial)
	PORT_RS232_PARITY("RS232_PARITY", RS232_PARITY_NONE, "Parity", console_device, update_serial)
	PORT_RS232_STOPBITS("RS232_STOPBITS", RS232_STOPBITS_1, "Stop Bits", console_device, update_serial)

	PORT_START("RS232_TXDELAY")
	PORT_CONFNAME(0x07, 0x04, "Char Delay")
	PORT_CONFSETTING(   0x00, DEF_STR(None))
	PORT_CONFSETTING(   0x01, "1 ms")
	PORT_CONFSETTING(   0x02, "2 ms")
	PORT_CONFSETTING(   0x03, "5 ms")
	PORT_CONFSETTING(   0x04, "10 ms")
	PORT_CONFSETTING(   0x05, "20 ms")
	PORT_CONFSETTING(   0x06, "50 ms")
	PORT_CONFSETTING(   0x07, "100 ms")
INPUT_PORTS_END

ioport_constructor console_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(console);
}

void console_device::device_start()
{
	m_timer_poll = timer_alloc(FUNC(console_device::update_queue), this);

	// Put stdin in raw mode so keystrokes arrive one at a time with no echo.
	if (isatty(STDIN_FILENO))
	{
		if (tcgetattr(STDIN_FILENO, &m_saved_termios) == 0)
		{
			m_termios_saved = true;
			struct termios raw = m_saved_termios;
			cfmakeraw(&raw);
			// Keep output processing so '\n' still works on stdout
			raw.c_oflag = m_saved_termios.c_oflag;
			tcsetattr(STDIN_FILENO, TCSANOW, &raw);
		}
	}

	// Make stdin non-blocking so the poll timer never stall mame.
	int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
	if (flags >= 0)
		fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void console_device::device_stop()
{
	// Restore stdin flags to blocking
	int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
	if (flags >= 0)
		fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);

	if (m_termios_saved)
	{
		tcsetattr(STDIN_FILENO, TCSANOW, &m_saved_termios);
		m_termios_saved = false;
	}
}

void console_device::update_serial(int)
{
	int startbits   = 1;
	int databits    = convert_databits(m_rs232_databits->read());
	parity_t parity = convert_parity(m_rs232_parity->read());
	stop_bits_t stopbits = convert_stopbits(m_rs232_stopbits->read());

	set_data_frame(startbits, databits, parity, stopbits);
	set_tra_rate(convert_baud(m_rs232_txbaud->read()));
	set_rcv_rate(convert_baud(m_rs232_rxbaud->read()));

	output_rxd(1);
	output_dcd(0);
	output_dsr(0);
	output_cts(0);
}

void console_device::device_reset()
{
	update_serial(0);
	update_queue(0);
}

// Called when the emulated machine sends a bit to us.
// device_serial_interface reassembles bits into bytes; rcv_complete fires
// when a full byte is ready.
void console_device::tra_callback()
{
	output_rxd(transmit_register_get_data_bit());
}

void console_device::tra_complete()
{
	int const delay_ms = TXDELAY_MS[m_rs232_txdelay->read() & 0x07];
	if (delay_ms > 0)
		m_timer_poll->adjust(attotime::from_msec(delay_ms));
	else
		update_queue(0);
}

// A full byte has been received from the emulated machine — write to stdout.
void console_device::rcv_complete()
{
	receive_register_extract();
	uint8_t const ch = get_received_char();
	(void)write(STDOUT_FILENO, &ch, 1);
}

// Poll stdin for any bytes the user has typed and queue them to be sent
// to the emulated machine as incoming serial data.
TIMER_CALLBACK_MEMBER(console_device::update_queue)
{
	if (is_transmit_register_empty())
	{
		uint8_t ch;
		ssize_t const n = read(STDIN_FILENO, &ch, 1);
		if (n == 1)
		{
			transmit_register_setup(ch);
			m_timer_poll->adjust(attotime::never);
			return;
		}
	}

	// Reschedule at the TX baud rate
	int const txbaud = convert_baud(m_rs232_txbaud->read());
	m_timer_poll->adjust(attotime::from_hz(txbaud));
}
