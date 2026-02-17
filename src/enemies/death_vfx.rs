use bevy::prelude::*;
use bevy_hanabi::prelude::*;

use super::{EnemyDebrisPiece, EnemyKilledEvent};

const ENEMY_DEATH_BURST_PARTICLES: f32 = 192.0;
const ENEMY_DEATH_BURST_CLEANUP_SECS: f32 = 1.2;
const ENEMY_DEBRIS_GRAVITY: f32 = 18.0;
const ENEMY_DEBRIS_DRAG_PER_SEC: f32 = 2.6;
const ENEMY_DEBRIS_MIN_LIFETIME_SECS: f32 = 1.4;
const ENEMY_DEBRIS_LIFETIME_VARIANCE_SECS: f32 = 1.1;

pub(crate) struct EnemyDeathVfxPlugin;

impl Plugin for EnemyDeathVfxPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<EnemyDeathBurstEffect>().add_systems(
            Update,
            (
                spawn_enemy_death_bursts,
                tick_enemy_death_debris,
                cleanup_enemy_death_bursts,
            ),
        );
    }
}

#[derive(Resource)]
struct EnemyDeathBurstEffect {
    handle: Handle<EffectAsset>,
}

impl FromWorld for EnemyDeathBurstEffect {
    fn from_world(world: &mut World) -> Self {
        let mut effects = world.resource_mut::<Assets<EffectAsset>>();

        let mut color_gradient = bevy_hanabi::Gradient::new();
        color_gradient.add_key(0.0, Vec4::new(1.0, 0.95, 0.7, 1.0));
        color_gradient.add_key(0.35, Vec4::new(1.0, 0.36, 0.12, 0.75));
        color_gradient.add_key(1.0, Vec4::new(0.0, 0.0, 0.0, 0.0));

        let mut size_gradient = bevy_hanabi::Gradient::new();
        size_gradient.add_key(0.0, Vec3::splat(0.28));
        size_gradient.add_key(1.0, Vec3::splat(0.02));

        let writer = ExprWriter::new();
        let init_age = SetAttributeModifier::new(Attribute::AGE, writer.lit(0.0).expr());
        let init_lifetime = SetAttributeModifier::new(
            Attribute::LIFETIME,
            (writer.lit(0.18) + writer.rand(ScalarType::Float) * writer.lit(0.35)).expr(),
        );
        let init_pos = SetPositionSphereModifier {
            center: writer.lit(Vec3::ZERO).expr(),
            radius: writer.lit(0.48).expr(),
            dimension: ShapeDimension::Volume,
        };
        let init_vel = SetVelocitySphereModifier {
            center: writer.lit(Vec3::ZERO).expr(),
            speed: (writer.lit(9.0) + writer.rand(ScalarType::Float) * writer.lit(12.0)).expr(),
        };
        let update_drag = LinearDragModifier::new(writer.lit(6.0).expr());

        let effect = effects.add(
            EffectAsset::new(
                512,
                SpawnerSettings::once(ENEMY_DEATH_BURST_PARTICLES.into()),
                writer.finish(),
            )
            .with_name("enemy_death_burst")
            .with_alpha_mode(bevy_hanabi::AlphaMode::Add)
            .init(init_age)
            .init(init_lifetime)
            .init(init_pos)
            .init(init_vel)
            .update(update_drag)
            .render(ColorOverLifetimeModifier::new(color_gradient))
            .render(SizeOverLifetimeModifier {
                gradient: size_gradient,
                screen_space_size: false,
            })
            .render(OrientModifier::new(OrientMode::FaceCameraPosition)),
        );

        Self { handle: effect }
    }
}

#[derive(Component)]
struct EnemyDeathBurstLifetime(Timer);

#[derive(Component)]
struct EnemyDeathDebris {
    velocity: Vec3,
    angular_velocity: Vec3,
    lifetime: Timer,
}

fn spawn_enemy_death_bursts(
    mut commands: Commands,
    mut killed_events: MessageReader<EnemyKilledEvent>,
    burst_effect: Res<EnemyDeathBurstEffect>,
) {
    for killed in killed_events.read() {
        commands.spawn((
            Name::new("EnemyDeathBurst"),
            ParticleEffect::new(burst_effect.handle.clone()),
            Transform::from_translation(killed.position),
            EnemyDeathBurstLifetime(Timer::from_seconds(
                ENEMY_DEATH_BURST_CLEANUP_SECS,
                TimerMode::Once,
            )),
        ));

        for (piece_index, piece) in killed.debris_pieces.iter().enumerate() {
            spawn_enemy_debris_piece(&mut commands, killed.position, piece_index, piece);
        }
    }
}

fn spawn_enemy_debris_piece(
    commands: &mut Commands,
    death_position: Vec3,
    piece_index: usize,
    piece: &EnemyDebrisPiece,
) {
    let seed = death_position.x * 0.91 + death_position.y * 1.37 + death_position.z * 2.11;
    let random_a = hash01(seed + piece_index as f32 * 3.17);
    let random_b = hash01(seed + piece_index as f32 * 4.91 + 9.0);
    let random_c = hash01(seed + piece_index as f32 * 6.43 + 17.0);
    let random_d = hash01(seed + piece_index as f32 * 2.79 + 31.0);

    let mut transform = piece.transform;
    let scale_multiplier = 0.42 + random_a * 0.48;
    transform.scale *= scale_multiplier;

    let away = (transform.translation - death_position).normalize_or_zero();
    let orbit_angle = piece_index as f32 * 2.399_963_1 + random_b * core::f32::consts::TAU;
    let up = (random_c * 2.0 - 1.0) * 0.7;
    let planar = (1.0 - up * up).sqrt();
    let swirl = Vec3::new(orbit_angle.cos() * planar, up, orbit_angle.sin() * planar);
    let direction = (away * 0.72 + swirl * 0.28).normalize_or_zero();

    let speed = 6.0 + random_d * 12.0;
    let angular_velocity = Vec3::new(
        random_b * 14.0 - 7.0,
        random_c * 16.0 - 8.0,
        random_d * 12.0 - 6.0,
    );
    let lifetime = Timer::from_seconds(
        ENEMY_DEBRIS_MIN_LIFETIME_SECS + random_a * ENEMY_DEBRIS_LIFETIME_VARIANCE_SECS,
        TimerMode::Once,
    );

    commands.spawn((
        Name::new("EnemyDeathDebris"),
        Mesh3d(piece.mesh.clone()),
        MeshMaterial3d(piece.material.clone()),
        transform,
        EnemyDeathDebris {
            velocity: direction * speed,
            angular_velocity,
            lifetime,
        },
    ));
}

fn tick_enemy_death_debris(
    mut commands: Commands,
    time: Res<Time>,
    mut debris: Query<(Entity, &mut EnemyDeathDebris, &mut Transform)>,
) {
    let dt = time.delta_secs();
    let drag = (1.0 - ENEMY_DEBRIS_DRAG_PER_SEC * dt).clamp(0.0, 1.0);

    for (entity, mut debris, mut transform) in &mut debris {
        debris.velocity.y -= ENEMY_DEBRIS_GRAVITY * dt;
        debris.velocity *= drag;
        transform.translation += debris.velocity * dt;

        transform.rotation *= Quat::from_euler(
            EulerRot::XYZ,
            debris.angular_velocity.x * dt,
            debris.angular_velocity.y * dt,
            debris.angular_velocity.z * dt,
        );

        if debris.lifetime.tick(time.delta()).is_finished() {
            commands.entity(entity).despawn();
        }
    }
}

fn cleanup_enemy_death_bursts(
    mut commands: Commands,
    time: Res<Time>,
    mut bursts: Query<(Entity, &mut EnemyDeathBurstLifetime)>,
) {
    for (entity, mut lifetime) in &mut bursts {
        if lifetime.0.tick(time.delta()).is_finished() {
            commands.entity(entity).despawn();
        }
    }
}

fn hash01(value: f32) -> f32 {
    (value.sin() * 43_758.547).fract().abs()
}
