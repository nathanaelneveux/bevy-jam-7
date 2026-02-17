mod ai;
mod spider;
mod spider_ik;
mod virus;
mod virus_ik;

use bevy::prelude::*;

pub struct EnemyPlugin;

impl Plugin for EnemyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            ai::EnemyAiPlugin,
            spider::SpiderEnemyPlugin,
            virus::VirusEnemyPlugin,
        ));
    }
}
