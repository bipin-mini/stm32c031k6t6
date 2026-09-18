
use dro08::display_i32;
fn main() {
    let mut ram_data = [0u8;16];
    display_i32(127, &mut ram_data, 0);
    println!("{:2X?}", ram_data);
}
