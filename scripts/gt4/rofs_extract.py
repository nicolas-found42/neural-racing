"""Read-only RoFS 3.1 index and selected extraction.
Layout reference: Razer2015/GT4FS b938f71fd939e0ea44c38839a50ffb66cfd2067a.
Does not run game code. Records original packed byte ranges and output hashes.
"""
import argparse, hashlib, json, struct, zlib
from pathlib import Path

def index(volume):
    volume_size = volume.stat().st_size
    with volume.open('rb') as f:
        for page in range(10000):
            base = page * 2048
            f.seek(base)
            h = f.read(64)
            if len(h) < 64:
                raise ValueError('RoFS3.1 header absent')
            magic = int.from_bytes(h[:4], 'big')
            if magic in (0x526f4653, (-0x526f4654) & 0xffffffff) and struct.unpack_from('<I', h, 4)[0] == 0x30001:
                break
        else:
            raise ValueError('RoFS3.1 header absent within documented search bound')
        encrypted = magic != 0x526f4653
        version, toc_length, page_count, page_length, count = struct.unpack_from('<IIIHH', h, 4)
        if page_length != 2048 or not 0 < count <= 4096:
            raise ValueError('Unsupported RoFS page size/count')
        f.seek(base + 64)
        offsets = []
        for i in range(count + 1):
            v = struct.unpack('<I', f.read(4))[0]
            offsets.append(v ^ ((0x14ac327a * (i + 1)) & 0xffffffff) if encrypted else v)
        if offsets != sorted(offsets) or offsets[0] < 64 + 4 * (count + 1) or base + offsets[-1] > volume_size:
            raise ValueError('Invalid RoFS page ranges')
        entries = []
        for page, (start, end) in enumerate(zip(offsets, offsets[1:])):
            f.seek(base + start)
            packed = f.read(end - start)
            if len(packed) != end - start:
                raise ValueError('Truncated RoFS index page')
            d = zlib.decompressobj(-15)
            b = d.decompress(packed, page_length + 1)
            # Fixed-size independently framed pages can omit DEFLATE BFINAL.
            # Require their exact decoded extent instead of a false EOF claim.
            if len(b) != page_length or d.unconsumed_tail:
                raise ValueError('Invalid decoded RoFS index page')
            if encrypted:
                b = bytes((v ^ 0x55 for v in b))
            flag, n, next_page, prev_page = struct.unpack_from('<hhii', b)
            if flag:
                continue
            if n < 0 or n % 2 != 0 or 12 + (n // 2) * 8 > len(b):
                raise ValueError('Invalid RoFS record count')
            for i in range(n // 2):
                no, nl, mo, ml = struct.unpack_from('<hhhh', b, len(b) - (i + 1) * 8)
                if not (0 <= no < len(b) and 4 <= nl <= len(b) - no and 0 <= mo < len(b) and 5 <= ml <= len(b) - mo):
                    raise ValueError('Invalid RoFS record range')
                parent = struct.unpack_from('>I', b, no)[0]
                name = b[no + 4:no + nl].decode('utf-8')
                if '\x00' in name or '/' in name or '\\' in name or (parent != 0 and name in ('', '.', '..')):
                    raise ValueError('Invalid RoFS path component')
                typ = b[mo]
                node = dict(parent=parent, name=name, type=typ, page_index=page)
                if typ == 0:
                    node['id'] = struct.unpack_from('<I', b, mo + 1)[0]
                elif typ in (1, 2):
                    if ml < (13 if typ == 1 else 17):
                        raise ValueError('Truncated RoFS file metadata')
                    po, mtime = struct.unpack_from('<II', b, mo + 1)
                    if typ == 1:
                        size = packed_size = struct.unpack_from('<I', b, mo + 9)[0]
                    else:
                        packed_size, size = struct.unpack_from('<II', b, mo + 9)
                    node.update(offset=base + page_count * page_length + po * page_length, packed_size=packed_size, size=size)
                    if node['offset'] + packed_size > volume_size:
                        raise ValueError('RoFS file exceeds volume')
                else:
                    raise ValueError(('bad record kind', typ, page, i))
                entries.append(node)
        dirs = {e['id']: e for e in entries if e['type'] == 0}
        if len(dirs) != sum(e['type'] == 0 for e in entries):
            raise ValueError('Duplicate RoFS directory identity')

        def path(e, seen=None):
            if e['parent'] == 0:
                return ''
            seen = set() if seen is None else seen
            if e['parent'] in seen or len(seen) > 128 or e['parent'] not in dirs:
                raise ValueError('Invalid RoFS directory ancestry')
            seen.add(e['parent'])
            return '/'.join(filter(None, [path(dirs[e['parent']], seen), e['name']]))
        for e in entries:
            e['path'] = path(e)
        return dict(base=base, encrypted=encrypted, page_count=page_count, page_length=page_length, record_pages=count, entries=entries)

def extract(volume, entry, out):
    relative = Path(entry['path'])
    if relative.is_absolute() or not relative.parts or '..' in relative.parts:
        raise ValueError('Invalid extraction path')
    root = out.resolve()
    target = root / relative
    for ancestor in (target, *target.parents):
        if ancestor == root:
            break
        if ancestor.is_symlink():
            raise ValueError('Extraction refuses symlink paths')
    if not target.resolve().is_relative_to(root):
        raise ValueError('Extraction path escapes output directory')
    if target.exists() or target.is_symlink():
        raise FileExistsError(target)
    if not 0 <= entry['size'] <= 64 * 1024 * 1024 or not 0 <= entry['packed_size'] <= 64 * 1024 * 1024:
        raise ValueError('Selected extraction exceeds 64 MiB per-file bound')
    with volume.open('rb') as f:
        f.seek(entry['offset'])
        packed = f.read(entry['packed_size'])
    if len(packed) != entry['packed_size']:
        raise ValueError(('truncated file', entry))
    b = packed
    if b[:4] == bytes.fromhex('c5eef7ff'):
        if len(b) < 8:
            raise ValueError('Truncated compressed-file header')
        declared = -struct.unpack_from('<i', b, 4)[0]
        if declared != entry['size']:
            raise ValueError('Compressed-file size disagrees with index')
        d = zlib.decompressobj(-15)
        b = d.decompress(b[8:], declared + 1)
        if not d.eof or len(b) != declared or d.unconsumed_tail:
            raise ValueError('Invalid compressed file')
    if len(b) != entry['size']:
        raise ValueError(('size mismatch', entry, len(b)))
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('xb') as output:
        output.write(b)
    return dict(path=entry['path'], source_offset=entry['offset'], packed_size=len(packed), size=len(b), packed_sha256=hashlib.sha256(packed).hexdigest(), sha256=hashlib.sha256(b).hexdigest())
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('volume', type=Path)
    ap.add_argument('--index', type=Path, required=True)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--prefix', action='append', default=[])
    a = ap.parse_args()
    r = index(a.volume)
    a.index.write_text(json.dumps(r, indent=2))
    print('Index:', len(r['entries']), 'entries; base', hex(r['base']))
    if a.out:
        selected = [e for e in r['entries'] if e['type'] != 0 and any((e['path'].lower().startswith(p.lower()) for p in a.prefix))]
        results = [extract(a.volume, e, a.out) for e in selected]
        (a.out / 'extraction-manifest.json').write_text(json.dumps(results, indent=2))
        print('Extracted', len(results), 'selected files')
