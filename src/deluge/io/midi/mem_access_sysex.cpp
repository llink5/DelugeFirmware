/*
 * Copyright (c) 2024 Synthstrom Audible Limited
 *
 * This file is part of The Synthstrom Audible Deluge Firmware.
 *
 * The Synthstrom Audible Deluge Firmware is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
 */

#include "io/midi/mem_access_sysex.h"
#include "io/midi/midi_device.h"
#include "io/midi/midi_engine.h"
#include "io/midi/sysex.h"
#include "version.h"

#include "util/pack.h"

#include <cstring>

namespace MemAccess {

constexpr uint8_t CMD_READ = 0x00;
constexpr uint8_t CMD_WRITE = 0x01;
constexpr uint8_t CMD_PING = 0x02;

constexpr uint8_t STATUS_OK = 0x00;
constexpr uint8_t STATUS_BAD_RANGE = 0x01;
constexpr uint8_t STATUS_BAD_FORMAT = 0x02;

constexpr int32_t MAX_TRANSFER_BYTES = 512;

// SysEx header: F0 [DelugeID x4] [MemAccess] [subcmd] = 7 bytes
constexpr int32_t HEADER_LEN = 7;

static bool isReadableAddress(uint32_t addr, uint32_t len) {
	uint32_t end = addr + len;
	if (end < addr) {
		return false; // overflow
	}

	// Internal SRAM
	if (addr >= 0x20000000 && end <= 0x20300000) {
		return true;
	}
	// SDRAM
	if (addr >= 0x0C000000 && end <= 0x10000000) {
		return true;
	}
	// SDRAM uncached mirror
	if (addr >= 0x4C000000 && end <= 0x50000000) {
		return true;
	}
	// SPI Flash
	if (addr >= 0x18000000 && end <= 0x1A000000) {
		return true;
	}

	return false;
}

static bool isWritableAddress(uint32_t addr, uint32_t len) {
	uint32_t end = addr + len;
	if (end < addr) {
		return false;
	}

	// Internal SRAM (excluding stack and TTB)
	if (addr >= 0x20020000 && end <= 0x202F8000) {
		return true;
	}
	// SDRAM
	if (addr >= 0x0C000000 && end <= 0x10000000) {
		return true;
	}

	return false;
}

static int32_t writeHeader(uint8_t* buf, uint8_t subCmd) {
	buf[0] = SysEx::SYSEX_START;
	buf[1] = SysEx::DELUGE_SYSEX_ID_BYTE0;
	buf[2] = SysEx::DELUGE_SYSEX_ID_BYTE1;
	buf[3] = SysEx::DELUGE_SYSEX_ID_BYTE2;
	buf[4] = SysEx::DELUGE_SYSEX_ID_BYTE3;
	buf[5] = SysEx::SysexCommands::MemAccess;
	buf[6] = subCmd;
	return HEADER_LEN;
}

static void sendErrorResponse(MIDICable& cable, uint8_t subCmd, uint8_t status) {
	uint8_t* buf = midiEngine.sysex_fmt_buffer;
	int32_t pos = writeHeader(buf, subCmd);
	buf[pos++] = status;
	buf[pos++] = SysEx::SYSEX_END;
	cable.sendSysex(buf, pos);
}

static uint32_t decodeAddr(const uint8_t* src) {
	return (uint32_t)src[0] | ((uint32_t)src[1] << 7) | ((uint32_t)src[2] << 14) | ((uint32_t)src[3] << 21)
	       | ((uint32_t)(src[4] & 0x0F) << 28);
}

static void encodeAddr(uint8_t* dst, uint32_t addr) {
	dst[0] = addr & 0x7F;
	dst[1] = (addr >> 7) & 0x7F;
	dst[2] = (addr >> 14) & 0x7F;
	dst[3] = (addr >> 21) & 0x7F;
	dst[4] = (addr >> 28) & 0x0F;
}

static uint16_t decodeLen(const uint8_t* src) {
	return (uint16_t)src[0] | ((uint16_t)(src[1] & 0x07) << 7);
}

static void encodeLen(uint8_t* dst, uint16_t len) {
	dst[0] = len & 0x7F;
	dst[1] = (len >> 7) & 0x07;
}

// READ: payload = [MemAccess] [CMD_READ] [addr x5] [len x2] [F7]
// Minimum payload length: 1 + 5 + 2 + 1 = 9
static void handleRead(MIDICable& cable, uint8_t* data, int32_t len) {
	if (len < 9) {
		sendErrorResponse(cable, CMD_READ, STATUS_BAD_FORMAT);
		return;
	}

	uint32_t addr = decodeAddr(data + 1);
	uint16_t readLen = decodeLen(data + 6);

	if (readLen == 0 || readLen > MAX_TRANSFER_BYTES) {
		sendErrorResponse(cable, CMD_READ, STATUS_BAD_FORMAT);
		return;
	}

	if (!isReadableAddress(addr, readLen)) {
		sendErrorResponse(cable, CMD_READ, STATUS_BAD_RANGE);
		return;
	}

	uint8_t* buf = midiEngine.sysex_fmt_buffer;
	int32_t pos = writeHeader(buf, CMD_READ);
	buf[pos++] = STATUS_OK;

	encodeAddr(buf + pos, addr);
	pos += 5;
	encodeLen(buf + pos, readLen);
	pos += 2;

	int32_t packedLen = pack_8bit_to_7bit(buf + pos,
	                                      sizeof(midiEngine.sysex_fmt_buffer) - pos - 1, // reserve 1 for F7
	                                      reinterpret_cast<uint8_t*>(addr), readLen);

	if (packedLen == 0) {
		sendErrorResponse(cable, CMD_READ, STATUS_BAD_FORMAT);
		return;
	}

	pos += packedLen;
	buf[pos++] = SysEx::SYSEX_END;
	cable.sendSysex(buf, pos);
}

// WRITE: payload = [MemAccess] [CMD_WRITE] [addr x5] [len x2] [packed_data...] [F7]
// Minimum payload length: 1 + 5 + 2 + 1(at least some data) + 1 = 10
static void handleWrite(MIDICable& cable, uint8_t* data, int32_t len) {
	if (len < 10) {
		sendErrorResponse(cable, CMD_WRITE, STATUS_BAD_FORMAT);
		return;
	}

	uint32_t addr = decodeAddr(data + 1);
	uint16_t writeLen = decodeLen(data + 6);

	if (writeLen == 0 || writeLen > MAX_TRANSFER_BYTES) {
		sendErrorResponse(cable, CMD_WRITE, STATUS_BAD_FORMAT);
		return;
	}

	if (!isWritableAddress(addr, writeLen)) {
		sendErrorResponse(cable, CMD_WRITE, STATUS_BAD_RANGE);
		return;
	}

	// Packed data starts at offset 8 within payload, ends before the trailing F7
	int32_t packedDataOffset = 8;
	int32_t packedDataLen = len - packedDataOffset - 1; // subtract F7

	if (packedDataLen <= 0) {
		sendErrorResponse(cable, CMD_WRITE, STATUS_BAD_FORMAT);
		return;
	}

	uint8_t tmpBuf[MAX_TRANSFER_BYTES];
	int32_t decoded = unpack_7bit_to_8bit(tmpBuf, writeLen, data + packedDataOffset, packedDataLen);

	if (decoded < writeLen) {
		sendErrorResponse(cable, CMD_WRITE, STATUS_BAD_FORMAT);
		return;
	}

	memcpy(reinterpret_cast<void*>(addr), tmpBuf, writeLen);

	uint8_t* buf = midiEngine.sysex_fmt_buffer;
	int32_t pos = writeHeader(buf, CMD_WRITE);
	buf[pos++] = STATUS_OK;
	buf[pos++] = SysEx::SYSEX_END;
	cable.sendSysex(buf, pos);
}

// PING: payload = [MemAccess] [CMD_PING] [F7]
static void handlePing(MIDICable& cable) {
	uint8_t* buf = midiEngine.sysex_fmt_buffer;
	int32_t pos = writeHeader(buf, CMD_PING);
	buf[pos++] = STATUS_OK;

	// Append firmware version string as 7-bit ASCII
	const char* ver = kFirmwareVersionStringShort;
	int32_t maxVerLen = sizeof(midiEngine.sysex_fmt_buffer) - pos - 1; // reserve F7
	for (int32_t i = 0; ver[i] != '\0' && i < maxVerLen; i++) {
		buf[pos++] = ver[i] & 0x7F;
	}

	buf[pos++] = SysEx::SYSEX_END;
	cable.sendSysex(buf, pos);
}

void sysexReceived(MIDICable& cable, uint8_t* data, int32_t len) {
	if (len < 2) {
		return;
	}

	// data[0] = MemAccess command byte (already matched by caller)
	// data[1] = subcommand
	switch (data[1]) {
	case CMD_READ:
		handleRead(cable, data + 1, len - 1);
		break;
	case CMD_WRITE:
		handleWrite(cable, data + 1, len - 1);
		break;
	case CMD_PING:
		handlePing(cable);
		break;
	default:
		break;
	}
}

} // namespace MemAccess
