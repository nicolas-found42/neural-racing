//! Real GT4 database rows passed through unmodified converters/loaders in Play!.
use neural_racing_core::gt4::tire::{expand_compound_into, TireTables};
use serde_json::Value;

fn array<const N: usize>(value: &Value) -> [u8; N] {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap() as u8)
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn bits(value: &Value) -> u32 {
    u32::from_str_radix(value.as_str().unwrap(), 16).unwrap()
}
fn ordered(value: u32) -> i64 {
    if value & 0x80000000 != 0 {
        i64::from(!value)
    } else {
        i64::from(value | 0x80000000)
    }
}
fn compare(value: f32, reference: &Value, context: &str) -> i64 {
    let distance = (ordered(value.to_bits()) - ordered(bits(reference))).abs();
    assert!(
        distance <= 12,
        "Play EE difference {distance} ULP for {context}: port={:08x}, original={}",
        value.to_bits(),
        reference
    );
    distance
}

#[test]
fn real_compound_expansion_and_tables_match_original_play_execution() {
    let fixture: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_tire_tables.json")).unwrap();
    assert_eq!(
        fixture["authority"],
        "controlled-original-opcode-execution-on-upstream-Play-EE"
    );
    assert_eq!(
        fixture["emulator"]["commit"],
        "1f27232772e484bfb79364b751b84f41f4c87481"
    );
    let mut expansions = 0;
    let mut tables_checked = 0;
    let mut worst = 0;
    for case in fixture["cases"].as_array().unwrap() {
        let mut expanded = [0; 0xa8];
        expand_compound_into(
            &mut expanded,
            &array(&case["compound_bytes"]),
            &array(&case["volume_bytes"]),
        );
        let context = format!(
            "{}/{}/{}",
            case["compound_label"], case["volume_label"], case["input_scale"]
        );
        if case["kind"] == "tire_tables" {
            let tables = TireTables::from_runtime(
                &mut expanded,
                case["input_scale"].as_f64().unwrap() as f32,
            )
            .unwrap();
            for (name, table) in [
                ("slip_mu_a", &tables.slip_mu_a),
                ("slip_mu_b", &tables.slip_mu_b),
                ("side", &tables.side),
                ("corner_drag", &tables.corner_drag),
                ("load", &tables.load),
            ] {
                for (axis, values) in [("x_bits", &table.x), ("y_bits", &table.y)] {
                    let reference = case["tables"][name][axis].as_array().unwrap();
                    assert_eq!(values.len(), reference.len());
                    for (value, expected) in values.iter().zip(reference) {
                        worst = worst.max(compare(
                            *value,
                            expected,
                            &format!("{context}/{name}/{axis}"),
                        ));
                    }
                }
            }
            for (offset, value) in [
                (0, tables.at_00),
                (4, tables.at_04),
                (0x68, tables.at_68),
                (0x164, tables.negative_peak_at_164),
                (0x168, tables.positive_peak_at_168),
                (0x16c, tables.positive_peak_mu_at_16c),
            ] {
                worst = worst.max(compare(
                    value,
                    &case["coefficient_bits"][format!("{offset:03x}")],
                    &context,
                ));
            }
            for (i, value) in tables.terrain_at_08.iter().enumerate() {
                worst = worst.max(compare(
                    *value,
                    &case["coefficient_bits"][format!("{:03x}", 8 + i * 4)],
                    &context,
                ));
            }
            tables_checked += 1;
        } else {
            assert_eq!(case["kind"], "tire_expand");
            expansions += 1;
        }
        assert_eq!(
            expanded,
            array::<0xa8>(&case["expanded_bytes"]),
            "expanded/mutated bytes: {context}"
        );
        assert_eq!(case["stopping_pc"], "01fff000");
    }
    assert_eq!(expansions, 70);
    assert_eq!(tables_checked, 140);
    println!("{expansions} byte-exact expansions, {tables_checked} original tire-table chains; maximum Play EE difference {worst} ULP");
}
