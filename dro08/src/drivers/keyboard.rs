//! keyboard.rs
//!
//! Minimal keyboard driver.
//! Call update() every 10 ms.

pub const KEY6: u32 = 0x0010_0000; // Left most key ACK/Mode
pub const KEY5: u32 = 0x0001_0000;
pub const KEY4: u32 = 0x0000_1000;
pub const KEY3: u32 = 0x0000_0100;
pub const KEY2: u32 = 0x0000_0010;
pub const KEY1: u32 = 0x0000_0001;

/*

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Key {
    Key1,
    Key2,
    Key3,
    Key4,
    Key5,
    Key6,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum KeyEvent {
    Short(Key),
    Long(Key),
}

pub struct Keyboard;

impl Keyboard {
    pub const fn new() -> Self {
        Self
    }

    pub fn update(&mut self, raw_keys: u32) -> Option<KeyEvent> {
        match raw_keys {
            KEY1 => Some(KeyEvent::Short(Key::Key1)),
            KEY2 => Some(KeyEvent::Short(Key::Key2)),
            KEY3 => Some(KeyEvent::Short(Key::Key3)),
            KEY4 => Some(KeyEvent::Short(Key::Key4)),
            KEY5 => Some(KeyEvent::Short(Key::Key5)),
            KEY6 => Some(KeyEvent::Short(Key::Key6)),
            _ => None,
        }
    }
}
*/

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Key {
    Key1,
    Key2,
    Key3,
    Key4,
    Key5,
    Key6,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum KeyEvent {
    Short(Key),
    Long(Key),
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    Idle,
    KeyPressed,
    KeyReleased,
}

pub struct Keyboard {
    active_key: u32,
    state: State,
    ticks: u16,
}

impl Keyboard {
    pub const fn new() -> Self {
        Self {
            active_key: 0,
            state: State::Idle,
            ticks: 0,
        }
    }

    pub fn update(&mut self, raw_keys: u32) -> Option<KeyEvent> {
        let mut event = None;

        match self.state {
            State::Idle => {
                if raw_keys != 0 {
                    // Transition: 0 -> KEY (First press detected)
                    self.active_key = raw_keys;
                    self.state = State::KeyPressed;
                    self.ticks = 0;
                }
            }
            State::KeyPressed => {
                if raw_keys == self.active_key {
                    self.ticks += 1;

                    // Long key trigger at 300 ticks (300 * 10ms = 3s)
                    if self.ticks >= 300 {
                        if let Some(key) = parse_key(self.active_key) {
                            event = Some(KeyEvent::Long(key));
                        }
                        // Move to KeyReleased so we don't continuously fire Long events
                        self.state = State::KeyReleased;
                    }
                } else if raw_keys == 0 {
                    // Key released before hitting long-press threshold
                    if let Some(key) = parse_key(self.active_key) {
                        event = Some(KeyEvent::Short(key));
                    }
                    self.reset();
                } else {
                    // Spurious read or different key pressed mid-debounce; reset
                    self.reset();
                }
            }
            State::KeyReleased => {
                if raw_keys == 0 {
                    self.reset();
                }
            }
        }

        event
    }

    fn reset(&mut self) {
        self.active_key = 0;
        self.state = State::Idle;
        self.ticks = 0;
    }
}

/// Helper function to map raw bitmask keys to the Key enum
fn parse_key(raw_keys: u32) -> Option<Key> {
    match raw_keys {
        KEY1 => Some(Key::Key1),
        KEY2 => Some(Key::Key2),
        KEY3 => Some(Key::Key3),
        KEY4 => Some(Key::Key4),
        KEY5 => Some(Key::Key5),
        KEY6 => Some(Key::Key6),
        _ => None,
    }
}
