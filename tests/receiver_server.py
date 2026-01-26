import argparse
import socket
import struct
from pathlib import Path
from typing import Optional, Tuple


# According to cib::daq::iols_tcp_packet_t layout:
#   - 16-byte trigger word (iols_trigger_t)
#   - 4-byte tcp_header_t (bitfield grouping; interpreted here as <HBB)
WORD_SIZE = 16
HEADER_SIZE = 4
PACKET_SIZE = WORD_SIZE + HEADER_SIZE
HEADER_STRUCT = struct.Struct("<HBB")  # packet_size (uint16), seq_id (uint8), format_version (uint8)


def recv_exact(conn: socket.socket, n_bytes: int) -> Optional[bytes]:
   data = bytearray()
   while len(data) < n_bytes:
      chunk = conn.recv(n_bytes - len(data))
      if not chunk:
         return None
      data.extend(chunk)
   return bytes(data)


def parse_header(buf: bytes) -> Tuple[int, int, int]:
   """Return (packet_size, seq_id, fmt_version)."""
   return HEADER_STRUCT.unpack(buf)


def fmt_version_valid(fmt_version: int) -> bool:
   """Upper nibble should be complement of lower nibble, set via set_version()."""
   hi = (fmt_version >> 4) & 0xF
   lo = fmt_version & 0xF
   return (hi ^ lo) == 0xF


def run_server(host: str, port: int, outfile: Optional[Path], header_last: bool, quiet: bool) -> None:
   with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
      server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      server_socket.bind((host, port))
      server_socket.listen(1)
      print(f"Listening on {host}:{port}")

      conn, address = server_socket.accept()
      print(f"Connection from: {address}")

      total_packets = 0
      total_bytes = 0
      out_handle = None
      if outfile:
         outfile.parent.mkdir(parents=True, exist_ok=True)
         out_handle = outfile.open("wb")

      try:
         detected_header_last: Optional[bool] = None if not header_last else True
         while True:
            pkt = recv_exact(conn, PACKET_SIZE)
            if pkt is None:
               break

            # Header placement detection (once): prefer header-last if both look valid.
            if detected_header_last is None:
               first_hdr = pkt[:HEADER_SIZE]
               last_hdr = pkt[WORD_SIZE:WORD_SIZE + HEADER_SIZE]
               ps1, s1, f1 = parse_header(first_hdr)
               ps2, s2, f2 = parse_header(last_hdr)
               v1 = (ps1 == WORD_SIZE and fmt_version_valid(f1))
               v2 = (ps2 == WORD_SIZE and fmt_version_valid(f2))
               if v2 and (not v1 or v2):
                  detected_header_last = True
                  packet_size, seq_id, fmt_version = ps2, s2, f2
               elif v1:
                  detected_header_last = False
                  packet_size, seq_id, fmt_version = ps1, s1, f1
               else:
                  print(f"Header auto-detect failed: first(size={ps1},fmt=0x{f1:02X}) last(size={ps2},fmt=0x{f2:02X})")
                  break
               if not quiet:
                  placement = "end" if detected_header_last else "start"
                  print(f"Detected header at {placement} of packet")
            else:
               use_last = detected_header_last
               hdr_buf = pkt[WORD_SIZE:WORD_SIZE + HEADER_SIZE] if use_last else pkt[:HEADER_SIZE]
               packet_size, seq_id, fmt_version = parse_header(hdr_buf)
               if packet_size != WORD_SIZE or not fmt_version_valid(fmt_version):
                  print(f"Header check failed: size={packet_size} fmt=0x{fmt_version:02X}")
                  break

            word = pkt[:WORD_SIZE] if (detected_header_last or header_last) else pkt[HEADER_SIZE:HEADER_SIZE + WORD_SIZE]

            total_packets += 1
            total_bytes += len(pkt)

            if out_handle:
               out_handle.write(pkt)

            if not quiet:
               # Show brief info; dump first 8 bytes of word for quick sanity
               preview = word[:8].hex()
               print(f"pkt {total_packets} seq={seq_id} fmt=0x{fmt_version:02X} size={packet_size} word[0:8]={preview}")
      finally:
         if out_handle:
            out_handle.close()
         conn.close()
         print(f"Connection closed. Received {total_packets} packets, {total_bytes} bytes")


def parse_args() -> argparse.Namespace:
   parser = argparse.ArgumentParser(description="Fixed-size TCP receiver for iols_tcp_packet_t packets")
   parser.add_argument("--host", default="0.0.0.0", help="Interface to bind")
   parser.add_argument("--port", type=int, default=8972, help="TCP port")
   parser.add_argument("--outfile", type=Path, help="Optional path to store raw packets")
   parser.add_argument(
      "--header-last",
      action="store_true",
      help="Expect header after the 16-byte word (default tries header-first, then auto-switch)",
   )
   parser.add_argument("--quiet", action="store_true", help="Reduce per-packet logging")
   return parser.parse_args()


if __name__ == "__main__":
   args = parse_args()
   run_server(args.host, args.port, args.outfile, args.header_last, args.quiet)
