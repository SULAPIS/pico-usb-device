pub mod sensor {
    use cortex_m::prelude::_embedded_hal_blocking_i2c_WriteRead;
    use rp2040_hal::gpio::{bank0::*, Function, Pin, I2C};
    use rp_pico::pac::I2C0;

    ///编码器结构体
    pub struct As5600 {
        addr: u8,
        raw_angle_high: u8,
        raw_angle_low: u8,
        i2c: rp2040_hal::I2C<I2C0, (Pin<Gpio16, Function<I2C>>, Pin<Gpio17, Function<I2C>>)>,
        pub angle: f64,
        pub circle: i32,
    }
    impl As5600 {
        pub fn new(
            i2c: rp2040_hal::I2C<I2C0, (Pin<Gpio16, Function<I2C>>, Pin<Gpio17, Function<I2C>>)>,
        ) -> Self {
            let mut as5600 = Self {
                addr: 0x36,
                raw_angle_high: 0x0c,
                raw_angle_low: 0x0d,
                i2c,
                angle: 0.0,
                circle: 0,
            };
            as5600.angle = as5600.get_angle();
            as5600
        }

        ///return: 编码器12位bit数值
        pub fn get_bit(&mut self) -> u16 {
            let mut buff_l = [0; 1];
            let mut buff_h = [0; 1];
            self.i2c
                .write_read(self.addr, &[self.raw_angle_low], &mut buff_l)
                .unwrap();
            self.i2c
                .write_read(self.addr, &[self.raw_angle_high], &mut buff_h)
                .unwrap();
            u16::from_be_bytes([buff_h[0], buff_l[0]])
        }

        ///return: 当前绝对角度
        pub fn get_angle(&mut self) -> f64 {
            let u = self.get_bit();
            360.0 / 4096.0 * u as f64
        }

        pub fn update_angle(&mut self) {
            let angle = self.get_angle();
            if angle == self.angle {
                return;
            }
            if angle < 90.0 && self.angle > 270.0 {
                self.circle += 1;
            } else if angle > 270.0 && self.angle < 90.0 {
                self.circle -= 1;
            }
            self.angle = angle;
        }
    }
}
