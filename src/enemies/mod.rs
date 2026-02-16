mod spider;
mod spider_ik;

use bevy::prelude::*;

pub struct EnemyPlugin;

impl Plugin for EnemyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(spider::SpiderEnemyPlugin);
    }
}
