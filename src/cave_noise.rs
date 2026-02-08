use bevy::prelude::Vec2;
use noiz::prelude::{Noise, SampleableFor, ScalableNoise, SeedableNoise, common_noise};

const BASE_FLOOR_Y: i32 = -8;
const FLOOR_AMPLITUDE: f32 = 8.0;
const FLOOR_PERIOD: f32 = 80.0;

const BASE_CEILING_Y: i32 = 24;
const CEILING_AMPLITUDE: f32 = 10.0;
const CEILING_PERIOD: f32 = 110.0;

const MIN_PASSAGE_HEIGHT: i32 = 8;

pub const FLOOR_MIN_Y: i32 = (BASE_FLOOR_Y as f32 - FLOOR_AMPLITUDE) as i32;
pub const CEILING_MAX_Y: i32 = (BASE_CEILING_Y as f32 + CEILING_AMPLITUDE) as i32;

#[derive(Clone)]
pub struct CaveNoise {
    floor: Noise<common_noise::Perlin>,
    ceiling: Noise<common_noise::Perlin>,
}

impl CaveNoise {
    pub fn new() -> Self {
        let mut floor = Noise::<common_noise::Perlin>::default();
        floor.set_seed(7);
        floor.set_period(FLOOR_PERIOD);

        let mut ceiling = Noise::<common_noise::Perlin>::default();
        ceiling.set_seed(101);
        ceiling.set_period(CEILING_PERIOD);

        Self { floor, ceiling }
    }

    pub fn sample_column(&self, x: i32, z: i32) -> (i32, i32) {
        let pos = Vec2::new(x as f32, z as f32);

        let floor_noise: f32 = self.floor.sample(pos);
        let ceiling_noise: f32 = self.ceiling.sample(pos);

        let floor_y = BASE_FLOOR_Y + (floor_noise * FLOOR_AMPLITUDE).round() as i32;
        let ceiling_y_raw = BASE_CEILING_Y + (ceiling_noise * CEILING_AMPLITUDE).round() as i32;
        let ceiling_y = ceiling_y_raw.max(floor_y + MIN_PASSAGE_HEIGHT);

        (floor_y, ceiling_y)
    }
}
