package transport

import (
	"encoding/binary"
	"testing"
)

func TestCRC8AndTurboTelemetryRoundTrip(t *testing.T) {
	pkt := BuildTurboPacket(CmdControl, []byte{OpHoldMask, 0x03, 0x00, 0x00, 0x00})
	if pkt[0] != TurboSync0 || pkt[1] != TurboSync1 || pkt[2] != CmdControl || pkt[3] != 5 {
		t.Fatalf("unexpected header: %x", pkt[:4])
	}
	wantCRC := CRC8(pkt[2:9])
	if pkt[9] != wantCRC {
		t.Fatalf("CRC mismatch: got %02x want %02x", pkt[9], wantCRC)
	}

	// Build a 50-byte telemetry payload matching ESP32-S3 firmware layout
	var pl [50]byte
	binary.LittleEndian.PutUint16(pl[0:2], 2049)   // M1 actual
	binary.LittleEndian.PutUint16(pl[2:4], 2050)   // M1 target
	binary.LittleEndian.PutUint16(pl[4:6], 0xFFE2) // M1 duty = -30
	binary.LittleEndian.PutUint16(pl[6:8], 3)      // M1 noise p2p = 3 counts (12-bit!)
	binary.LittleEndian.PutUint16(pl[8:10], 85)    // M1 noise RMS = 0.85 counts
	binary.LittleEndian.PutUint16(pl[10:12], 12)   // M1 12x GDMA samples
	pl[36] = 0x04                                  // M3 disabled, M1+M2 enabled
	pl[37] = 0x00                                  // no faults
	pl[38] = 2                                     // 2DOF
	pl[39] = 1                                     // divider 1
	binary.LittleEndian.PutUint16(pl[40:42], 18)   // loop exec 18 us
	binary.LittleEndian.PutUint16(pl[42:44], 24)   // loop max 24 us
	binary.LittleEndian.PutUint16(pl[44:46], 2)    // jitter 2 us
	binary.LittleEndian.PutUint32(pl[46:50], 123456)

	var telem Telemetry
	if !ParseTurboTelemetry(pl[:], &telem) {
		t.Fatalf("ParseTurboTelemetry failed")
	}
	if telem.Motors[0].Actual12 != 2049 || telem.Motors[0].Duty != -30 || telem.Motors[0].NoiseP2P12 != 3 {
		t.Fatalf("unexpected M1 state: %+v", telem.Motors[0])
	}
	if telem.Motors[0].Disabled || telem.Motors[1].Disabled || !telem.Motors[2].Disabled {
		t.Fatalf("unexpected disabled mask decoding: %02x", telem.DisabledMask)
	}
	if telem.LoopExecUs != 18 || telem.LoopJitterUs != 2 || telem.LoopCount != 123456 {
		t.Fatalf("unexpected timing stats: %+v", telem)
	}
}
