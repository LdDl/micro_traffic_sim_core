use super::*;

#[test]
fn test_parse_signal_valid() {
    // Test for valid signal strings
    assert_eq!(SignalType::from_str("r").unwrap(), SignalType::Red);
    assert_eq!(SignalType::from_str("y").unwrap(), SignalType::Yellow);
    assert_eq!(SignalType::from_str("g").unwrap(), SignalType::Green);
    assert_eq!(
        SignalType::from_str("G").unwrap(),
        SignalType::GreenPriority
    );
    assert_eq!(SignalType::from_str("s").unwrap(), SignalType::GreenRight);
    assert_eq!(SignalType::from_str("u").unwrap(), SignalType::RedYellow);
    assert_eq!(SignalType::from_str("o").unwrap(), SignalType::Blinking);
    assert_eq!(SignalType::from_str("O").unwrap(), SignalType::NoSignal);
}
#[test]
fn test_parse_signal_invalid() {
    // Test for invalid signal strings
    let result = SignalType::from_str("z");
    assert!(result.is_err());
    assert_eq!(result.unwrap_err().to_string(), "Invalid signal type: 'z'");
}
