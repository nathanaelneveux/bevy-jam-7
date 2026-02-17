mod ai;
mod death_vfx;
mod spider;
mod spider_ik;
mod virus;
mod virus_ik;

use bevy::prelude::*;

pub(crate) use ai::EnemyHealth;

#[derive(Clone, Debug)]
pub(crate) struct EnemyDebrisPiece {
    pub(crate) mesh: Handle<Mesh>,
    pub(crate) material: Handle<StandardMaterial>,
    pub(crate) transform: Transform,
}

#[derive(Message, Clone, Debug)]
pub(crate) struct EnemyKilledEvent {
    pub(crate) position: Vec3,
    pub(crate) debris_pieces: Vec<EnemyDebrisPiece>,
}

pub struct EnemyPlugin;

impl Plugin for EnemyPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<EnemyKilledEvent>().add_plugins((
            ai::EnemyAiPlugin,
            death_vfx::EnemyDeathVfxPlugin,
            spider::SpiderEnemyPlugin,
            virus::VirusEnemyPlugin,
        ));
    }
}
