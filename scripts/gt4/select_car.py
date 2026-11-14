"""Join actual GT4 EqDbCode keys without inferring part slot names or units.

Input is decode_specdb.py output. All-ones keys are NULL, not table ID -1.
This produces raw source data, not a fully initialized physics configuration.
"""
import argparse
import json
import struct
from pathlib import Path


def select(directory, car_label):
    tables = {}
    for path in sorted(directory.glob('*.json')):
        table = json.loads(path.read_text())
        if not isinstance(table, dict) or 'table_id' not in table:
            continue
        identity = table['table_id']
        # Localized display-name tables are outside equipment relationships.
        if identity >= 39:
            continue
        if identity in tables:
            raise ValueError(f'Duplicate equipment table ID {identity}')
        tables[identity] = table

    def row_for(table, row_id):
        matches = [row for row in table['rows'] if row['id'] == row_id]
        if len(matches) != 1:
            raise ValueError(f'Unresolved/ambiguous {table["name"]} row {row_id}')
        return matches[0]

    def resolve(key):
        row_id, table_id = struct.unpack('<II', key)
        if row_id == table_id == 0xffffffff:
            return None
        if 0xffffffff in (row_id, table_id) or table_id not in tables:
            raise ValueError(f'Invalid equipment key {key.hex()}')
        table = tables[table_id]
        return dict(table_id=table_id, row_id=row_id, table=table['name'],
                    dbt_sha256=table['dbt_sha256'], idi_sha256=table['idi_sha256'],
                    row=row_for(table, row_id))

    generic = tables[0]
    matches = [row for row in generic['rows'] if row['label'] == car_label]
    if len(matches) != 1:
        raise ValueError(f'Unresolved/ambiguous car label {car_label}')
    car = matches[0]
    default_parts = resolve(bytes(car['bytes'][:8]))
    if default_parts is None or default_parts['table'] != 'DEFAULT_PARTS':
        raise ValueError('Generic car does not link to DEFAULT_PARTS')
    raw_parts = bytes(default_parts['row']['bytes'])
    if len(raw_parts) % 8:
        raise ValueError('DEFAULT_PARTS is not an array of EqDbCode keys')
    parts = [dict(slot=offset // 8, value=resolve(raw_parts[offset:offset + 8]))
             for offset in range(0, len(raw_parts), 8)]
    # Tire equipment embeds seven further keys: size, three compound selections,
    # and force-volume selections. Retain their actual order without relabeling.
    tire_references = []
    for part in parts:
        value = part['value']
        if value is not None and value['table'] in ('FRONTTIRE', 'REARTIRE'):
            raw = bytes(value['row']['bytes'])
            if len(raw) < 56:
                raise ValueError('Truncated tire equipment row')
            tire_references.append(dict(table=value['table'],
                keys=[resolve(raw[offset:offset + 8]) for offset in range(0, 56, 8)]))
    return dict(car=car, generic_dbt_sha256=generic['dbt_sha256'],
                generic_idi_sha256=generic['idi_sha256'], default_parts=default_parts,
                resolved_parts=parts, tire_references=tire_references,
                scope='Raw key joins only; native defaults/auto-gearing/initializers still required')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('decoded_directory', type=Path)
    parser.add_argument('car_label')
    parser.add_argument('output', type=Path)
    arguments = parser.parse_args()
    result = select(arguments.decoded_directory, arguments.car_label)
    arguments.output.write_text(json.dumps(result, indent=2) + '\n')
    print(f'{arguments.car_label}: {len(result["resolved_parts"])} default part slots joined')
