//#![deny(warnings)]
use crate::debug_plugin::DebugPlugin;
use crate::physics::*;
use bevy::color::palettes::basic::*;
use bevy::color::palettes::css::*;
use bevy::math::vec3;
use bevy::prelude::*;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_trackball::prelude::*;
use std::f32::consts::*;

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
        //.add_systems(Update, movements)
        //.add_systems(FixedUpdate, update_car)
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

fn setup_camera(mut commands: Commands) {
    let eye = vec3(0.0, 20.0, 20.0);
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

    let obstacle_size = 60.0;
    let obstacle = Cuboid::new(obstacle_size, obstacle_size, obstacle_size);
    let obstacle_collider = Collider::cuboid(
        obstacle_size / 2.0,
        obstacle_size / 2.0,
        obstacle_size / 2.0,
    );
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(obstacle),
            material: materials.add(Color::Srgba(ORANGE)),
            transform: Transform {
                translation: vec3(10.0, -20.0, -40.0),
                rotation: Quat::from_rotation_x(30f32.to_radians()),
                ..default()
            },
            ..default()
        },
        obstacle_collider,
    ));

    const CAR_GROUP: Group = Group::GROUP_1;

    let car_scale = 2.0;

    let car_length = 0.9 * car_scale;
    let car_width = 0.85 * car_scale;
    let car_height = 0.3 * car_scale;

    let wheel_height = car_height * -Vec3::Y;
    let car_front = car_length * -Vec3::Z;
    let car_back = car_length * Vec3::Z;
    let car_left = car_width * -Vec3::X;
    let car_right = car_width * Vec3::X;

    let tire_front_left = car_front * 0.95 + car_left * 0.85 + wheel_height * 0.75;
    let tire_front_right = car_front * 0.95 + car_right * 0.85 + wheel_height * 0.75;
    let tire_rear_left = car_back * 0.95 + car_left * 0.85 + wheel_height * 0.75;
    let tire_rear_right = car_back * 0.95 + car_right * 0.85 + wheel_height * 0.75;

    let wheel_params = [
        tire_front_left,
        tire_front_right,
        tire_rear_left,
        tire_rear_right,
    ];
    // TODO: lower center of mass?
    // let mut center_of_mass = wheel_params.iter().sum().unwrap() / 4.0;
    // center_of_mass.y = 0.0;

    let suspension_height = 1.0;
    let max_steering_angle = 35.0f32.to_radians();
    let drive_strength = 1.0;
    let wheel_radius = 0.28 * car_scale;
    let car_position = vec3(0.0, wheel_radius + suspension_height, 0.0);

    let body_position_in_car_space = car_position;

    let car_body = Cuboid {
        half_size: vec3(car_width, car_height, car_length),
    };

    let car_collider = Collider::cuboid(car_width, car_height, car_length);

    let car_entity = commands
        .spawn((
            PbrBundle {
                mesh: meshes.add(car_body),
                material: materials.add(Color::Srgba(GREEN)),
                transform: Transform {
                    translation: vec3(0.0, 10.0, 0.0),
                    ..default()
                },
                ..default()
            },
            physics::rigid_body_dynamic(),
            car_collider,
        ))
        .id();

    for (wheel_id, wheel_pos_in_car_space) in wheel_params.into_iter().enumerate() {
        let is_front = wheel_id >= 2;
        let is_left = wheel_id == 0 || wheel_id == 2;
        let is_right = wheel_id == 1 || wheel_id == 3;
        let wheel_center = car_position + wheel_pos_in_car_space;

        let wheel_thickness = wheel_radius / 2.0;
        let axle_mass_props = MassProperties {
            mass: 100.0,
            ..default()
        };

        let axle_mesh = Cuboid::new(2.0, 0.1, 0.1);
        let axle_co = Collider::cuboid(2.0, 0.1, 0.1);

        let mut locked_axes = JointAxesMask::LIN_X
            | JointAxesMask::LIN_Z
            | JointAxesMask::ANG_X
            | JointAxesMask::ANG_Z;

        if !is_front {
            locked_axes |= JointAxesMask::ANG_Y;
        }

        let suspension_attachment_in_body_space =
            wheel_pos_in_car_space - body_position_in_car_space;

        let mut suspension_joint = GenericJointBuilder::new(locked_axes)
            .limits(JointAxis::LinY, [0.0, suspension_height])
            .motor_position(JointAxis::LinY, 0.0, 1.0e4, 1.0e3)
            .local_anchor1(suspension_attachment_in_body_space);

        if is_front {
            suspension_joint =
                suspension_joint.limits(JointAxis::AngY, [-max_steering_angle, max_steering_angle]);
        }

        let axle_handle = commands
            .spawn((
                PbrBundle {
                    mesh: meshes.add(axle_mesh),
                    material: materials.add(Color::Srgba(YELLOW)),
                    transform: Transform {
                        translation: wheel_center,
                        ..default()
                    },
                    ..default()
                },
                // dont put the axle collider here
                physics::rigid_body_dynamic(),
                CollisionGroups::new(CAR_GROUP, !CAR_GROUP),
                AdditionalMassProperties::MassProperties(axle_mass_props),
                ImpulseJoint::new(
                    car_entity,
                    TypedJoint::GenericJoint(suspension_joint.build()),
                ),
            ))
            .id();

        let wheel_co = Collider::ball(wheel_radius);
        let mut wheel_mesh: Mesh = Cylinder {
            half_height: wheel_thickness,
            radius: wheel_radius,
        }
        .into();
        wheel_mesh.rotate_by(Quat::from_rotation_z(FRAC_PI_2));

        let wheel_entity = commands
            .spawn((
                PbrBundle {
                    mesh: meshes.add(wheel_mesh),
                    material: materials.add(Color::Srgba(RED)),
                    transform: Transform {
                        translation: wheel_center,
                        ..default()
                    },
                    ..default()
                },
                wheel_co,
                physics::rigid_body_dynamic(),
                ImpulseJoint::new(axle_handle, RevoluteJointBuilder::new(Vec3::X)),
            ))
            .id();
    }
}

fn movements(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
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
            &physics::LinearVelocity,
            &physics::AngularVelocity,
            &physics::Mass,
            &Transform,
            &Car,
        ),
        With<Car>,
    >,
    mut wheel_query: Query<(&mut Transform, &mut Wheel), (With<Wheel>, Without<Car>)>,
    mut events: EventReader<Movement>,
) {
    let car_speed = 20.0;
    let delta_seconds = time.delta_seconds();
    let (mut car_impulse, linear_velocity, angular_velocity, mass, car_transform, car) =
        car.single_mut();
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

    let (wheel_transform, mut wheel) = wheel_query.single_mut();
    let global_wheel_transform = *car_transform * *wheel_transform;
    let car_position = car_transform.translation;
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
    let car_left = car_transform.left() * car.width / 2.0;
    let car_right = car_transform.right() * car.width / 2.0;

    let front_tire_left = car_front * 0.75 + car_left * 0.75;
    let front_tire_right = car_front * 0.75 + car_right * 0.75;
    let rear_tire_left = car_back * 0.75 + car_left * 0.75;
    let rear_tire_right = car_back * 0.75 + car_right * 0.75;

    let upward = car_transform.up();
    let upward_force = upward * mass * delta_seconds * 8.0;

    // car tires
    gizmos.arrow(
        car_position + front_tire_left,
        car_position + front_tire_left + upward * 10.0,
        Color::Srgba(FUCHSIA),
    );
    gizmos.arrow(
        car_position + front_tire_right,
        car_position + front_tire_right + upward * 10.0,
        Color::Srgba(FUCHSIA),
    );
    gizmos.arrow(
        car_position + rear_tire_left,
        car_position + rear_tire_left + upward * 10.0,
        Color::Srgba(FUCHSIA),
    );
    gizmos.arrow(
        car_position + rear_tire_right,
        car_position + rear_tire_right + upward * 10.0,
        Color::Srgba(FUCHSIA),
    );

    let wheel_lateral_direction = wheel.angle.sin() * global_wheel_transform.left();
    let wheel_forward_direction = wheel.angle.cos() * global_wheel_transform.forward();

    // longitudinal forward force
    gizmos.arrow(
        car_position,
        car_position + wheel_forward_direction * 10.0,
        Color::Srgba(PINK),
    );

    // lateral counter force
    gizmos.arrow(
        car_position,
        car_position + wheel_lateral_direction * 10.0,
        Color::Srgba(YELLOW),
    );

    // main force
    gizmos.arrow(
        car_position,
        car_position + (wheel_forward_direction + wheel_lateral_direction) * 10.0,
        Color::Srgba(WHITE),
    );

    let car_torque = mass * car_speed * delta_seconds;

    for movement in events.read() {
        match movement {
            Movement::Forward => {
                let longitudinal_force = wheel_forward_direction * car_torque;
                let lateral_counter_force = wheel_lateral_direction * car_torque;
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
                    wheel_lateral_direction * car_torque * 1.0,
                    car_front,
                    Vec3::ZERO,
                );
            }
            Movement::Backward => {
                let longitudinal_force = wheel_forward_direction * car_torque;
                let lateral_counter_force = wheel_lateral_direction * car_torque;
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
                    wheel_lateral_direction * car_torque * 1.0,
                    car_back,
                    Vec3::ZERO,
                );
            }
        }
    }
}
