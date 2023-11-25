use trace::trace;

trace::init_depth_var!();

#[trace]
fn a(b: u8) {
}

#[trace]
fn x(y: u8) -> u8 {
    2 * y
}

fn main() {
    a(x(3));
}
