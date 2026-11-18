use super::*;
use crate::traffic_lights::groups::TrafficLightGroup;
use crate::traffic_lights::signals::SignalType;

#[test]
fn test_traffic_light() {
    let groups = vec![
        TrafficLightGroup::new(1)
            .with_label("Group 1".to_string())
            .with_cells_ids(vec![20])
            .with_signal(vec![SignalType::Red, SignalType::Yellow, SignalType::Green])
            .build(),
        TrafficLightGroup::new(2)
            .with_label("Group 2".to_string())
            .with_cells_ids(vec![16])
            .with_signal(vec![
                SignalType::Green,
                SignalType::Green,
                SignalType::Green,
            ])
            .build(),
        TrafficLightGroup::new(3)
            .with_label("Group 3".to_string())
            .with_cells_ids(vec![11])
            .with_signal(vec![SignalType::Red, SignalType::Red, SignalType::Red])
            .build(),
        TrafficLightGroup::new(4)
            .with_label("Group 4".to_string())
            .with_cells_ids(vec![18, 20])
            .with_signal(vec![SignalType::Red, SignalType::Red, SignalType::Red])
            .build(),
    ];

    let mut traffic_light = TrafficLight::new(1)
        .with_groups(groups)
        .with_phases_times(vec![2, 3, 10])
        .with_active_phase(0)
        .build();

    assert_eq!(traffic_light.get_current_time(), 0);
    assert_eq!(traffic_light.get_active_phase(), 0);

    // Simulate time steps
    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 1);
    assert_eq!(traffic_light.get_active_phase(), 0);

    // Timer should reset because first time is 2
    // Phase has to be changed to 1
    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 0);
    assert_eq!(traffic_light.get_active_phase(), 1);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 1);
    assert_eq!(traffic_light.get_active_phase(), 1);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 2);
    assert_eq!(traffic_light.get_active_phase(), 1);

    // Timer should reset because second time is 3
    // Phase has to be changed to 2
    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 0);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 1);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 2);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 3);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 4);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 5);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 6);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 7);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 8);
    assert_eq!(traffic_light.get_active_phase(), 2);

    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 9);
    assert_eq!(traffic_light.get_active_phase(), 2);

    // Timer should reset because third time is 10
    // Phase has to be changed to 0 (reset) since there are only 3 phases and we've reached the end of the second one
    traffic_light.step();
    assert_eq!(traffic_light.get_current_time(), 0);
    assert_eq!(traffic_light.get_active_phase(), 0);
}
