use crate::debug_plugin::DebugPlugin;
use crate::physics::*;
use bevy::color::palettes::basic::*;
use bevy::color::palettes::css::*;
use bevy::core_pipeline::motion_blur::{MotionBlur, MotionBlurBundle};
use bevy::math::vec3;
use bevy::pbr::ExtendedMaterial;
use bevy::window::PresentMode;
use bevy::{
    pbr::{CascadeShadowConfigBuilder, DirectionalLightShadowMap},
    prelude::*,
};
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_trackball::prelude::*;
use std::f32::consts::*;
use std::sync::{Arc, Mutex};
use std::time::Instant;

pub mod camera_config;
pub mod debug_plugin;
pub mod physics;

fn main() {
    let mut app = App::new();

    app.add_plugins(DefaultPlugins)
        .add_plugins(DebugPlugin)
        .add_plugins(TrackballPlugin)
        .add_plugins(InfiniteGridPlugin)
        .add_plugins(physics::PhysicsPlugin)
        .add_systems(Startup, setup_camera)
        .add_systems(Startup, setup_car)
        .add_systems(Update, movements)
        .add_systems(FixedUpdate, update_car)
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 1000.0,
        })
        .add_event::<Movement>()
        .run();
}

#[derive(Component)]
struct Car {
    width: f32,
    length: f32,
    height: f32,
}

#[derive(Component)]
struct Wheel {
    max_angle: f32,
    angle: f32,
    min_angle: f32,
}

#[derive(Event)]
enum Movement {
    Forward,
    Backward,
}

fn setup_camera(mut commands: Commands, asset_server: Res<AssetServer>) {
    let eye = vec3(0.0, 40.0, 40.0);
    let target = vec3(0.0, 0.0, 0.0);
    let up = Vec3::Y;
    commands.spawn((
        Camera3dBundle { ..default() },
        camera_config::trackball_config(),
        TrackballCamera::look_at(target, eye, up),
    ));
}

fn setup_car(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(InfiniteGridBundle::default());
    let car_scale = 1.0;
    let car_length = 3.0 * car_scale;
    let car_width = 2.0 * car_scale;
    let car_height = 1.5 * car_scale;
    let collider = Collider::cuboid(car_width / 2.0, car_height / 2.0, car_length / 2.0);
    let car = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(Cuboid::new(car_width, car_height, car_length)),
                material: materials.add(Color::Srgba(GREEN).with_alpha(0.5)),
                transform: Transform {
                    translation: vec3(0.0, 4.0, 10.0),
                    scale: Vec3::splat(1.0),
                    ..default()
                },
                ..default()
            },
            collider,
            physics::mass(2000.0),
            physics::external_impulse(),
            physics::external_force(),
            physics::rigid_body_dynamic(),
            physics::LinearVelocity::default(),
            #[cfg(feature = "avian3d")]
            physics::AngularVelocity::default(),
            Friction::new(1.0),
            Car {
                width: car_width,
                length: car_length,
                height: car_height,
            },
        ))
        .id();
    let mut tire_mesh: Mesh = Cylinder::new(0.4, 0.4).into();
    tire_mesh.rotate_by(Quat::from_rotation_z(FRAC_PI_2));

    let wheel = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(tire_mesh),
                material: materials.add(Color::Srgba(RED)),
                transform: Transform {
                    translation: vec3(1.2, 0.0, -1.0),
                    ..default()
                },
                ..default()
            },
            Wheel {
                angle: 0.0,
                max_angle: 30f32.to_radians(),
                min_angle: -30f32.to_radians(),
            },
        ))
        .id();

    commands.entity(car).add_child(wheel);

    let ground_size = 10_000.0;
    let ground_thickness = 1.0;
    let ground_collider = Collider::cuboid(ground_size, ground_thickness, ground_size);

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(ground_size, ground_thickness, ground_size)),
            material: materials.add(Color::Srgba(GRAY)),
            transform: Transform {
                translation: vec3(0.0, -1.0, 0.0),
                ..default()
            },
            ..default()
        },
        ground_collider,
        Friction::new(1.0),
    ));
}

fn movements(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    car: Query<&Car>,
    mut wheel: Query<(&mut Transform, &mut Wheel)>,
    mut movement_writer: EventWriter<Movement>,
) {
    if keys.pressed(KeyCode::KeyW) {
        println!("forward");
        movement_writer.send(Movement::Forward);
    }
    if keys.pressed(KeyCode::KeyS) {
        println!("backward");
        movement_writer.send(Movement::Backward);
    }
    for (mut transform, mut wheel) in wheel.iter_mut() {
        let inc = time.delta_seconds();
        if keys.pressed(KeyCode::KeyA) {
            println!("turn left");
            if wheel.angle <= wheel.max_angle {
                wheel.angle += inc;
            }
        }
        if keys.pressed(KeyCode::KeyD) {
            println!("turn right");
            if wheel.angle >= wheel.min_angle {
                wheel.angle -= inc;
            }
        }
        transform.rotation = Quat::from_rotation_y(wheel.angle);
    }
}

fn update_car(
    time: Res<Time>,
    mut gizmos: Gizmos,
    mut car: Query<
        (
            &mut ExternalImpulse,
            &mut ExternalForce,
            &physics::LinearVelocity,
            &physics::AngularVelocity,
            &physics::Mass,
            &Transform,
            &Car,
        ),
        With<Car>,
    >,
    wheel: Query<(&mut Transform, &Wheel), (With<Wheel>, Without<Car>)>,
    mut events: EventReader<Movement>,
) {
    let car_speed = 10.0;
    let delta_seconds = time.delta_seconds();
    let (
        mut car_impulse,
        mut car_force,
        linear_velocity,
        angular_velocity,
        mass,
        car_transform,
        car,
    ) = car.single_mut();
    let mass = physics::get_mass(mass);

    let linear_velocity = physics::linear_velocity(linear_velocity);
    // this are the car velocity
    // GREEN indicates where the car will be.
    gizmos.arrow(
        car_transform.translation,
        car_transform.translation + linear_velocity,
        Color::Srgba(GREEN),
    );

    let angular_velocity = physics::angular_velocity(angular_velocity);
    // BLUE indicates how much the car is spinning, It will be pointing up if spinning right, pointing down if
    // spinning left
    gizmos.arrow(
        car_transform.translation,
        car_transform.translation + angular_velocity,
        Color::Srgba(BLUE),
    );

    let (wheel_transform, wheel) = wheel.single();
    let global_wheel_transform = *car_transform * *wheel_transform;
    let car_position = car_transform.translation;
    let wheel_direction = *global_wheel_transform.forward();
    let car_forward = *car_transform.forward();

    // GRAY arrow points where the car body is heading to
    gizmos.arrow(
        car_position,
        car_position + car_forward * 10.0,
        Color::Srgba(GRAY),
    );
    // RED arrow points where the wheel is heading to, oriented at the wheel location
    gizmos.arrow(
        global_wheel_transform.translation,
        global_wheel_transform.translation + global_wheel_transform.forward() * 5.0,
        Color::Srgba(RED),
    );

    let car_front = car_transform.forward() * car.length / 2.0;
    let car_back = car_transform.back() * car.length / 2.0;
    let global_car_front = car_transform.translation + car_front;

    let wheel_lateral_direction = wheel.angle.sin() * global_wheel_transform.left();
    let wheel_forward_direction = wheel.angle.cos() * global_wheel_transform.forward();

    let acceleration = 10.0;
    let wheel_lateral_force = wheel_lateral_direction * acceleration;
    let wheel_forward_force = wheel_forward_direction * acceleration;
    let total_force = (wheel_forward_direction + wheel_lateral_direction) * acceleration;

    // longitudinal forward force
    gizmos.arrow(
        car_position,
        car_position + wheel_forward_force,
        Color::Srgba(PINK),
    );

    /// lateral counter force
    gizmos.arrow(
        car_position,
        car_position + wheel_lateral_force,
        Color::Srgba(YELLOW),
    );

    // main force
    gizmos.arrow(
        car_position,
        car_position + total_force,
        Color::Srgba(WHITE),
    );

    let car_direction = *car_transform.forward();

    for movement in events.read() {
        match movement {
            Movement::Forward => {
                let longitudinal_force =
                    wheel_forward_direction * mass * car_speed * delta_seconds;

                let lateral_counter_force =
                    wheel_lateral_direction * mass * car_speed * delta_seconds;

                let total_force = longitudinal_force * 1.5 + lateral_counter_force * 1.1;
                physics::add_external_impulse(
                    &mut car_impulse,
                    total_force,
                    Vec3::ZERO,
                    Vec3::ZERO,
                );

                // push the car to rotate to align the wheel direction
                physics::add_external_impulse(
                    &mut car_impulse,
                    wheel_lateral_direction * mass * car_speed * delta_seconds * 1.0,
                    car_front,
                    Vec3::ZERO,
                );
            }
            Movement::Backward => {
                let longitudinal_force =
                    wheel_forward_direction * mass * car_speed * delta_seconds;

                let lateral_counter_force =
                    wheel_lateral_direction * mass * car_speed * delta_seconds;

                let total_force = longitudinal_force * 1.5 + lateral_counter_force * 1.1;
                physics::add_external_impulse(
                    &mut car_impulse,
                    -total_force,
                    Vec3::ZERO,
                    Vec3::ZERO,
                );

                // push the car to rotate to align the wheel direction
                physics::add_external_impulse(
                    &mut car_impulse,
                    -wheel_lateral_direction * mass * car_speed * delta_seconds * 1.0,
                    car_front,
                    Vec3::ZERO,
                );

            }
            _ => (),
        }
    }
}
