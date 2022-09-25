pub mod ikcode {
    use bincode::{Decode, Encode};

    #[derive(Decode, Encode, Debug, Clone, Copy)]
    pub struct Angle {
        pub angle: f64,
        pub time: f64,
    }
    impl Angle {
        pub fn new(angle: f64) -> Self {
            Self { angle, time: 2.0 }
        }
    }
    #[derive(Decode, Encode, Debug, Clone, Copy)]
    pub struct TAngle {
        pub a: [Angle; 10],
        pub c: usize,
    }
    impl TAngle {
        pub fn new() -> Self {
            Self {
                a: [Angle::new(0.0); 10],
                c: 0,
            }
        }

        pub fn push(&mut self, angle: f64) {
            self.a[self.c].angle = angle;
            self.c += 1;
        }
    }
}
