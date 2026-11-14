//! Source-derived GT4 course identities. Resource presence alone does not make a race.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CourseTopology {
    Closed,
    Open,
    NonDriving,
    /// Native topology 2: free/photo/gymkhana areas without a lap route.
    Area,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CourseKind {
    Road,
    Dirt,
    Snow,
    License,
    Photo,
    Unknown,
}

/// COURSE row +0x10, named by native MCourseData::GetType (0x1b3030).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CourseCondition {
    Dry,
    Wet,
    Dirt,
    Snow,
    Unknown,
}

#[derive(Clone, Copy, Debug)]
pub struct Gt4Course {
    pub id: &'static str,
    pub name: &'static str,
    pub slug: &'static str,
    pub topology: CourseTopology,
    pub condition: CourseCondition,
    pub source_topology: u8,
    pub section: Option<u16>,
    pub start_v: f32,
    pub goal_v: f32,
    pub kind: CourseKind,
    /// Requires validated geometry; Area resources support manual practice only.
    pub available: bool,
    /// Source-verified identities only; never fuzzy display-name matching.
    pub aliases: &'static [&'static str],
}

pub fn courses() -> &'static [Gt4Course] {
    include!("circuit_data/gt4_catalog.rs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Circuit;
    use std::collections::HashSet;

    #[test]
    fn course_identities_and_statistics_remain_distinct() {
        let mut ids = HashSet::new();
        let mut slugs = HashSet::new();
        let mut slots = HashSet::new();
        for (index, course) in courses().iter().enumerate() {
            assert!(ids.insert(course.id), "duplicate course ID {}", course.id);
            assert!(
                slugs.insert(course.slug),
                "duplicate course slug {}",
                course.slug
            );
            assert_eq!(course.slug, format!("gt4-{}", course.id));
            let circuit = Circuit::Gt4(u16::try_from(index).unwrap());
            assert!(slots.insert(circuit.stats_index()));
            assert!(circuit.stats_index() < Circuit::stats_count() - 1);
            if course.available {
                assert!(matches!(
                    course.topology,
                    CourseTopology::Closed | CourseTopology::Open | CourseTopology::Area
                ));
                assert_eq!(Circuit::lookup(course.slug), Some(circuit));
                assert!(Circuit::all().contains(&circuit));
            } else {
                assert!(!Circuit::all().contains(&circuit));
            }
        }
    }
    #[test]
    fn dedicated_sprints_bind_original_event_sections() {
        for (id, metres) in [
            ("testcourse_100m", 100.0),
            ("testcourse_200m", 200.0),
            ("testcourse_500m", 500.0),
            ("testcourse_1000m", 1000.0),
        ] {
            let course = courses().iter().find(|c| c.id == id).unwrap();
            assert_eq!(course.topology, CourseTopology::Open);
            assert_eq!(course.section, Some(0));
            assert!((course.goal_v - course.start_v - metres).abs() < 0.02);
        }
        let base = courses().iter().find(|c| c.id == "testcourse").unwrap();
        assert_eq!(base.topology, CourseTopology::Closed);
        assert_eq!(base.section, None);
        let wet = courses().iter().find(|c| c.id == "tsukuba_wet").unwrap();
        assert_eq!(wet.condition, CourseCondition::Wet);
    }
    #[test]
    fn original_geographic_layouts_have_no_unverified_gt4_substitutes() {
        // Bound to the complete original110-row COURSE join, rather than a
        // fuzzy name match across other GT games or similarly named rings.
        for circuit in Circuit::ALL {
            assert_eq!(Circuit::lookup(circuit.slug()), Some(circuit));
        }
        assert_eq!(Circuit::lookup("ring"), None);
        assert_ne!(Circuit::lookup("highspeedring"), Some(Circuit::RedBullRing));
    }
}
