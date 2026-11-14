"""Decode GT4 SpecDB DBT/IDI tables. Layout attribution: Nenkai/PDTools (MIT).
Raw bytes stay in the local audit area; schemas/physical meanings are separate.
"""
import argparse, hashlib, json, struct
from pathlib import Path

def decode(table):
    b = table.with_suffix('.dbt').read_bytes()
    labels = table.with_suffix('.idi').read_bytes()
    if not (b[:4] == b'GTDB' and labels[:4] == b'GTID'):
        raise ValueError('Invalid binary structure')
    endian = '>' if int.from_bytes(b[4:6], 'little') >= 256 else '<'

    def u32(at):
        return struct.unpack_from(endian + 'I', b, at)[0]
    attr, aligned, n, size = struct.unpack_from(endian + 'HHII', b, 4)
    label_endian = '<' if labels[4] else '>'
    ln = struct.unpack_from(label_endian + 'I', labels, 4)[0]
    names = {}
    for i in range(ln):
        off, key = struct.unpack_from(label_endian + 'II', labels, 16 + i * 8)
        start = 16 + ln * 8 + off
        length = struct.unpack_from(label_endian + 'H', labels, start)[0]
        if not labels[start + 2 + length - 1] == 0:
            raise ValueError('Invalid binary structure')
        names[key] = labels[start + 2:start + 2 + length - 1].decode('utf-8')
    if n == 0:
        return dict(name=table.name, table_id=struct.unpack_from(label_endian + 'I', labels, 12)[0], row_size=size, row_count=0, dbt_sha256=hashlib.sha256(b).hexdigest(), idi_sha256=hashlib.sha256(labels).hexdigest(), rows=[])
    book = 16 + n * 8
    template = book + u32(book) if attr & 1 else None
    data = template + u32(template) if attr & 1 else book
    long_codes = {}
    if attr & 1:
        for i in range(u32(book + 4)):
            at = book + 520 + i * 8
            bits, sym = b[at:at + 2]
            code = u32(at + 4)
            if not (bits, code) not in long_codes:
                raise ValueError('Invalid binary structure')
            long_codes[bits, code] = sym
    rows = []
    for i in range(n):
        key, relative = struct.unpack_from(endian + 'II', b, 16 + i * 8)
        at = data + relative
        if not attr & 1:
            row = b[at:at + size]
        else:
            nsymbols = b[at]
            bit = 0
            decoded = bytearray()
            for _ in range(nsymbols):
                pos = at + 1 + bit // 8
                code = int.from_bytes(b[pos:pos + 4].ljust(4, b'\x00'), 'little') >> bit % 8
                sym, bits = b[book + 8 + (code & 255) * 2:book + 10 + (code & 255) * 2]
                if not bits:
                    for bits in range(9, 32):
                        sym = long_codes.get((bits, code & (1 << bits) - 1))
                        if sym is not None:
                            break
                    else:
                        raise ValueError('invalid Huffman code')
                decoded.append(sym)
                bit += bits
            kind, ix = (decoded[0] >> 6, decoded[0] & 63)
            if kind == 1:
                row = decoded[1:1 + size]
            else:
                if not kind in (0, 2):
                    raise ValueError('Invalid binary structure')
                row = bytearray(b[template + 8 + ix * size:template + 8 + (ix + 1) * size])
                if kind == 2:
                    ptr = 1 + (size + 7) // 8
                    for j in range(size):
                        if decoded[1 + j // 8] >> j % 8 & 1:
                            if not ptr < len(decoded):
                                raise ValueError('Invalid binary structure')
                            row[j] = decoded[ptr]
                            ptr += 1
                    if not ptr == len(decoded):
                        raise ValueError('Invalid binary structure')
        if not len(row) == size:
            raise ValueError('Invalid binary structure')
        rows.append(dict(id=key, label=names.get(key), bytes=list(row), hex=bytes(row).hex()))
    return dict(name=table.name, table_id=struct.unpack_from(label_endian + 'I', labels, 12)[0], row_size=size, row_count=n, dbt_sha256=hashlib.sha256(b).hexdigest(), idi_sha256=hashlib.sha256(labels).hexdigest(), rows=rows)
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('table', type=Path)
    ap.add_argument('out', type=Path)
    a = ap.parse_args()
    r = decode(a.table)
    a.out.write_text(json.dumps(r, indent=2))
    print(r['name'], r['row_count'], 'rows of', r['row_size'], 'bytes')
