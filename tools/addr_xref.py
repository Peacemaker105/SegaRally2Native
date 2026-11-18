"""Finds absolute RAM addresses the game accesses, by pairing lis/addi
register materialization in out/listing.asm with the loads and stores that
use the register.

Usage: python tools/addr_xref.py <lo hex> <hi hex> [w]    (w = writes only)
Prints one line per access:  address  function  listing-address  mnemonic+ops
"""
import re
import sys

def sx16(v):
    return v - 0x10000 if v & 0x8000 else v

def main():
    lo, hi = int(sys.argv[1], 16), int(sys.argv[2], 16)
    writes_only = len(sys.argv) > 3 and sys.argv[3] == "w"
    regs = {}
    fn = "?"
    mem_ops = {"lwz", "lfs", "lfd", "lhz", "lbz", "stw", "stfs", "stfd", "sth", "stb"}
    for line in open("out/listing.asm"):
        m = re.match(r"^(fn_[0-9A-Fa-f]+):", line)
        if m:
            fn = m.group(1)
            continue
        m = re.match(r"^  ([0-9A-Fa-f]{8}): [0-9A-Fa-f]{8}\s+(\S+)\s*(.*)", line)
        if not m:
            continue
        addr = int(m.group(1), 16)
        mn = m.group(2)
        toks = m.group(3).replace(",", " ").split()
        if mn == "lis" and len(toks) >= 2:
            try:
                regs[toks[0]] = (int(toks[1], 0) << 16) & 0xFFFFFFFF
            except ValueError:
                regs.pop(toks[0], None)
            continue
        if mn in ("addi", "addis") and len(toks) >= 3 and toks[0] == toks[1] and toks[0] in regs:
            try:
                imm = int(toks[2], 0)
            except ValueError:
                continue
            regs[toks[0]] = (regs[toks[0]] + (sx16(imm) if mn == "addi" else (imm << 16) & 0xFFFFFFFF if imm < 0x8000 else sx16(imm << 16))) & 0xFFFFFFFF
            continue
        if mn in mem_ops and len(toks) >= 2:
            last = toks[-1]
            if "(" in last and last.endswith(")"):
                inner = last[last.index("(") + 1:-1]
                disp = last[:last.index("(")]
                try:
                    off = int(disp, 0) if disp and disp != "0" else 0
                except ValueError:
                    off = 0
                if inner in regs:
                    if writes_only and not mn.startswith("st"):
                        continue
                    ea = (regs[inner] + off) & 0xFFFFFFFF
                    if lo <= ea <= hi:
                        print(f"{ea:08X} {fn} @{addr:08X}  {mn} {m.group(3)}")
        # a non-memory, non-materializing write to a register kills its constant
        if len(toks) >= 1 and toks[0] in regs and mn not in mem_ops | {"addi", "addis", "lis", "mr", "li", "ori", "oris", "nop"}:
            if len(toks) >= 2 and toks[0] == toks[1] and mn == "rotl":
                continue
            regs.pop(toks[0], None)

if __name__ == "__main__":
    main()
