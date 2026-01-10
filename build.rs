// SPDX-FileCopyrightText: 2026 Sam Hanes <sam@maltera.com>
// SPDX-License-Identifier: GPL-3.0-or-later

fn main() {
    println!("cargo:rustc-link-arg=-Tdefmt.x");
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}
