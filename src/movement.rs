pub mod movement {
    use cortex_m::delay::Delay;
    use embedded_hal::digital::v2::OutputPin;
    use rp2040_hal::gpio::{bank0::*, Output, Pin, PushPull};

    use fmt::fmt::FmtBuf;
    use sensor::sensor::As5600;

    use crate::{fmt, sensor};

    ///驱动器结构体
    pub struct Pulse {
        ena: Pin<Gpio22, Output<PushPull>>,
        dir: Pin<Gpio21, Output<PushPull>>,
        pul: Pin<Gpio20, Output<PushPull>>,
    }

    impl Pulse {
        pub fn new(
            ena: Pin<Gpio22, Output<PushPull>>,
            dir: Pin<Gpio21, Output<PushPull>>,
            pul: Pin<Gpio20, Output<PushPull>>,
        ) -> Self {
            Self { ena, dir, pul }
        }

        pub fn sent_pulse(&mut self, delay: &mut Delay) {
            self.pul.set_high().unwrap();
            delay.delay_us(1);
            self.pul.set_low().unwrap();
        }

        //1: 0->360 2:360->0
        pub fn change_dir(&mut self, dir: usize) {
            if dir == 0 {
                self.dir.set_low().unwrap();
            } else {
                self.dir.set_high().unwrap();
            }
        }

        pub fn enable(&mut self) {
            self.ena.set_low().unwrap();
        }

        pub fn disable(&mut self) {
            self.ena.set_high().unwrap();
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub struct RPoint {
        angle: f64,
        speed: f64,
        a: f64,
        time: f64,
        //0: 给定运行时间 1: 匀速 2: 变速
        mode: usize,
    }
    impl RPoint {
        /// mode:
        ///
        /// 0: 给定运行时间| angle:目标角度 time:时间
        ///
        /// 1: 匀速| speed:速度 time: 持续时间
        ///
        /// 2: 变速| speed:初始速度 a:加速度 time:持续时间
        pub fn new(angle: f64, speed: f64, a: f64, time: f64, mode: usize) -> Self {
            Self {
                angle,
                speed,
                a,
                time,
                mode,
            }
        }

        pub fn start(
            &self,
            now_angle: f64,
            bit_angle: f64,
            pulse: &mut Pulse,
            delay: &mut Delay,
            as5600: &mut As5600,
        ) -> i32 {
            match self.mode {
                0 => self.time_speed(now_angle, pulse, delay, bit_angle, as5600),
                1 => self.uniform_speed(now_angle, pulse, delay, bit_angle, as5600),
                2 => self.variable_speed(now_angle, pulse, delay, bit_angle, as5600),
                _ => 0,
            }
        }

        pub fn time_speed(
            &self,
            now_angle: f64,
            pulse: &mut Pulse,
            delay: &mut Delay,
            bit_angle: f64,
            as5600: &mut As5600,
        ) -> i32 {
            let n = ((self.angle - now_angle) / bit_angle) as i32;

            let num = n.abs();
            let speed = (self.time * 1000_000.0 / num as f64) as u32;
            if now_angle < self.angle {
                pulse.change_dir(0);
            } else {
                pulse.change_dir(1);
            }
            for i in 0..num {
                if i % 300 == 0 {
                    as5600.update_angle();
                }
                pulse.sent_pulse(delay);
                // let a = as5600.get_angle() + 360.0 * as5600.circle as f64;
                // *angle = a / reduction_ratio;
                // *circle = (*angle / 360.0) as i32;

                delay.delay_us(speed);
            }

            n
        }
        pub fn uniform_speed(
            &self,
            now_angle: f64,
            pulse: &mut Pulse,
            delay: &mut Delay,
            bit_angle: f64,
            as5600: &mut As5600,
        ) -> i32 {
            let n = (self.speed * self.time / bit_angle) as i32;
            let num = n.abs();
            let speed = (self.time * 1000_000.0 / num as f64) as u32;
            if self.speed > 0.0 {
                pulse.change_dir(0);
            } else {
                pulse.change_dir(1);
            }
            for i in 0..num {
                if i % 300 == 0 {
                    as5600.update_angle();
                }
                pulse.sent_pulse(delay);
                delay.delay_us(speed);
            }
            as5600.update_angle();

            n
        }
        pub fn variable_speed(
            &self,
            now_angle: f64,
            pulse: &mut Pulse,
            delay: &mut Delay,
            bit_angle: f64,
            as5600: &mut As5600,
        ) -> i32 {
            let subdivide = 100;
            let mut n = 0;
            let mut speed = self.speed;
            let dur_time = self.time / subdivide as f64;
            for i in 1..(subdivide + 1) {
                speed = self.speed + self.a * dur_time * i as f64;
                if speed > 0.0 {
                    pulse.change_dir(0);
                } else {
                    pulse.change_dir(1);
                }

                n += self.v_uniform_speed(pulse, delay, bit_angle, dur_time, speed);
                as5600.update_angle();
            }

            n
        }
        pub fn v_uniform_speed(
            &self,
            pulse: &mut Pulse,
            delay: &mut Delay,
            bit_angle: f64,
            dur_time: f64,
            now_speed: f64,
        ) -> i32 {
            let n = (now_speed * dur_time / bit_angle) as i32;
            let num = n.abs();
            let speed = (dur_time * 1000_000.0 / num as f64) as u32;
            for _ in 0..num {
                pulse.sent_pulse(delay);
                delay.delay_us(speed);
            }

            n
        }
    }

    //运动控制
    pub struct MoveMent {
        pub angle: f64,
        pub as_angle: f64,
        pub bit_angle: f64,
        pub as5600: As5600,
        r_path: [RPoint; 1000],
        capacity: usize,
        now_capacity: usize,
        reduction_ratio: f64,
        pub circle: i32,
    }
    impl MoveMent {
        pub fn new(as5600: As5600, pulse: &mut Pulse, delay: &mut Delay) -> Self {
            let capacity = 1000;

            let mut movement = Self {
                angle: 0.0,
                as_angle: 0.0,
                bit_angle: 0.0434363,
                as5600,
                r_path: [RPoint::new(0.0, 0.0, 0.0, 0.0, 0); 1000],
                capacity,
                now_capacity: 0,
                reduction_ratio: 5.18,
                circle: 0,
            };
            movement.as5600.update_angle();
            movement.angle = movement.as5600.angle / movement.reduction_ratio;
            movement.add_point(RPoint::new(0.0, 0.0, 0.0, 1.0, 0));

            movement.start(pulse, delay);
            movement.clean_path();
            movement
        }

        // 添加路径
        pub fn add_point(&mut self, point: RPoint) -> Result<(), ()> {
            if self.now_capacity < self.capacity {
                self.r_path[self.now_capacity] = point;
                self.now_capacity += 1;
                Ok(())
            } else {
                Err(())
            }
        }

        pub fn clean_path(&mut self) {
            self.now_capacity = 0;
        }

        pub fn start(&mut self, pulse: &mut Pulse, delay: &mut Delay) -> Result<(), ()> {
            for i in 0..self.now_capacity {
                let n = self.r_path[i].start(
                    self.angle,
                    self.bit_angle,
                    pulse,
                    delay,
                    &mut self.as5600,
                );

                self.angle += self.bit_angle * n as f64;
            }
            self.update();
            Ok(())
        }

        pub fn update(&mut self) {
            self.as5600.update_angle();
            let angle = self.as5600.get_angle() + 360.0 * self.as5600.circle as f64;
            self.as_angle = angle / self.reduction_ratio;
            self.circle = (self.as_angle / 360.0) as i32;
        }
    }
}
