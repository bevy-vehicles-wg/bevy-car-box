#[cfg(feature = "avian3d")]
pub use avian3d::prelude::*;
use bevy::prelude::*;
#[cfg(feature = "bevy_rapier3d")]
pub use bevy_rapier3d::prelude::*;

pub const ENABLE_PHYSICS: bool = true;
pub const DEBUG_RENDER: bool = true;
pub const DEBUG_RENDER_ALL: bool = true;

#[cfg(feature = "bevy_rapier3d")]
pub type Mass = AdditionalMassProperties;

#[cfg(feature = "avian3d")]
pub type Mass = avian3d::prelude::Mass;

#[cfg(feature = "bevy_rapier3d")]
pub type LinearVelocity = bevy_rapier3d::prelude::Velocity;

#[cfg(feature = "bevy_rapier3d")]
pub type AngularVelocity = bevy_rapier3d::prelude::Velocity;

#[cfg(feature = "avian3d")]
pub type LinearVelocity = avian3d::prelude::LinearVelocity;
#[cfg(feature = "avian3d")]
pub type AngularVelocity = avian3d::prelude::AngularVelocity;

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        #[cfg(feature = "bevy_rapier3d")]
        if ENABLE_PHYSICS {
            app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());

            if DEBUG_RENDER {
                if DEBUG_RENDER_ALL {
                    app.add_plugins(RapierDebugRenderPlugin {
                        mode: DebugRenderMode::all(),
                        ..default()
                    });
                } else {
                    app.add_plugins(RapierDebugRenderPlugin::default());
                }
            }
        }

        #[cfg(feature = "avian3d")]
        if ENABLE_PHYSICS {
            app.add_plugins(PhysicsPlugins::default());

            if DEBUG_RENDER {
                app.add_plugins(PhysicsDebugPlugin::default());
            }
        }
    }
}

#[cfg(feature = "bevy_rapier3d")]
pub fn get_mass(mass: &AdditionalMassProperties) -> f32 {
    let AdditionalMassProperties::Mass(mass) = mass else {
        panic!()
    };
    *mass
}

#[cfg(feature = "avian3d")]
pub fn get_mass(mass: &Mass) -> f32 {
    mass.0
}

#[cfg(feature = "bevy_rapier3d")]
pub fn linear_velocity(velocity: &LinearVelocity) -> Vec3 {
    velocity.linvel
}

#[cfg(feature = "avian3d")]
pub fn linear_velocity(velocity: &LinearVelocity) -> Vec3 {
    velocity.0
}

#[cfg(feature = "bevy_rapier3d")]
pub fn angular_velocity(velocity: &AngularVelocity) -> Vec3 {
    velocity.angvel
}

#[cfg(feature = "avian3d")]
pub fn angular_velocity(velocity: &AngularVelocity) -> Vec3 {
    velocity.0
}

pub fn backend_used() -> &'static str {
    #[cfg(feature = "bevy_rapier3d")]
    let title = "bevy_rapier3d";
    #[cfg(feature = "avian3d")]
    let title = "avian3d";
    title
}

pub fn collider_convex(mesh: &Mesh) -> Option<Collider> {
    #[cfg(feature = "bevy_rapier3d")]
    let collider = Collider::from_bevy_mesh(mesh, &ComputedColliderShape::ConvexHull);
    #[cfg(feature = "avian3d")]
    let collider = Collider::convex_hull_from_mesh(mesh);

    collider
}

pub fn collider_trimesh(mesh: &Mesh) -> Collider {
    #[cfg(feature = "bevy_rapier3d")]
    let collider = Collider::from_bevy_mesh(mesh, &ComputedColliderShape::TriMesh).unwrap();
    #[cfg(feature = "avian3d")]
    let collider = Collider::trimesh_from_mesh(mesh).unwrap();

    collider
}

pub fn rigid_body_fixed() -> RigidBody {
    #[cfg(feature = "bevy_rapier3d")]
    let rigid_body = RigidBody::Fixed;

    #[cfg(feature = "avian3d")]
    let rigid_body = RigidBody::Static;

    rigid_body
}

pub fn rigid_body_dynamic() -> RigidBody {
    #[cfg(feature = "bevy_rapier3d")]
    let rigid_body = RigidBody::Dynamic;

    #[cfg(feature = "avian3d")]
    let rigid_body = RigidBody::Dynamic;

    rigid_body
}

pub fn gravity_scale(scale: f32) -> GravityScale {
    #[cfg(feature = "bevy_rapier3d")]
    let gravity_scale = GravityScale(scale);

    #[cfg(feature = "avian3d")]
    let gravity_scale = GravityScale(scale);

    gravity_scale
}

/// TODO: There is a bug with bevy_rpaier kinemetic
/// the location of the object will diverged from the parent it is attached to.
/// This applies to all RigidBody variant in bevy_rapier
pub fn rigid_body_kinematic() -> RigidBody {
    #[cfg(feature = "bevy_rapier3d")]
    let rigid_body = RigidBody::KinematicVelocityBased;

    #[cfg(feature = "avian3d")]
    let rigid_body = RigidBody::Kinematic;

    rigid_body
}

#[cfg(feature = "bevy_rapier3d")]
pub fn mass(m: f32) -> AdditionalMassProperties {
    AdditionalMassProperties::Mass(m)
}

#[cfg(feature = "avian3d")]
pub fn mass(m: f32) -> Mass {
    Mass(m)
}

pub fn external_impulse() -> ExternalImpulse {
    #[cfg(feature = "bevy_rapier3d")]
    let ext_impulse = ExternalImpulse::default();

    #[cfg(feature = "avian3d")]
    let ext_impulse = ExternalImpulse::new(Vec3::ZERO);

    ext_impulse
}

pub fn external_force() -> ExternalForce {
    #[cfg(feature = "bevy_rapier3d")]
    let ext_force = ExternalForce::default();

    #[cfg(feature = "avian3d")]
    let ext_force = ExternalForce::default();
    ext_force
}

pub fn external_impulse_at(impulse: Vec3, point: Vec3, center_of_mass: Vec3) -> ExternalImpulse {
    #[cfg(feature = "bevy_rapier3d")]
    let ext_impulse = ExternalImpulse::at_point(impulse, point, center_of_mass);

    #[cfg(feature = "avian3d")]
    let mut ext_impulse = ExternalImpulse::new(Vec3::ZERO);
    #[cfg(feature = "avian3d")]
    ext_impulse.apply_impulse_at_point(impulse, point, center_of_mass);

    ext_impulse
}

pub fn add_external_impulse(
    old: &mut ExternalImpulse,
    impulse: Vec3,
    point: Vec3,
    center_of_mass: Vec3,
) {
    #[cfg(feature = "bevy_rapier3d")]
    {
        *old += ExternalImpulse::at_point(impulse, point, center_of_mass);
    }

    #[cfg(feature = "avian3d")]
    old.apply_impulse_at_point(impulse, point, center_of_mass);
}

pub fn add_external_force(old: &mut ExternalForce, force: Vec3, point: Vec3, center_of_mass: Vec3) {
    #[cfg(feature = "bevy_rapier3d")]
    {
        *old += ExternalForce::at_point(force, point, center_of_mass);
    }

    #[cfg(feature = "avian3d")]
    old.apply_force_at_point(force, point, center_of_mass);
}

pub fn set_external_impulse(
    old: &mut ExternalImpulse,
    impulse: Vec3,
    point: Vec3,
    center_of_mass: Vec3,
) {
    #[cfg(feature = "bevy_rapier3d")]
    {
        *old = ExternalImpulse::at_point(impulse, point, center_of_mass);
    }

    #[cfg(feature = "avian3d")]
    old.set_impulse(impulse);
}

pub fn set_external_force(old: &mut ExternalForce, force: Vec3, point: Vec3, center_of_mass: Vec3) {
    #[cfg(feature = "bevy_rapier3d")]
    {
        *old = ExternalForce::at_point(force, point, center_of_mass);
    }
    #[cfg(feature = "avian3d")]
    old.set_force(force);
}

pub fn fixed_impulse_joint(commands: &mut Commands, parent: Entity, object: Entity, loc: Vec3) {
    #[cfg(feature = "bevy_rapier3d")]
    commands.entity(object).with_children(|children| {
        children.spawn(ImpulseJoint::new(
            parent,
            FixedJointBuilder::new().local_anchor1(loc),
        ));
    });

    #[cfg(feature = "avian3d")]
    commands.spawn(FixedJoint::new(parent, object).with_local_anchor_1(loc));
}

pub fn revolute_impulse_joint(commands: &mut Commands, parent: Entity, object: Entity, loc: Vec3) {
    #[cfg(feature = "bevy_rapier3d")]
    commands.entity(object).with_children(|children| {
        children.spawn(ImpulseJoint::new(
            parent,
            RevoluteJointBuilder::new(Vec3::X).local_anchor1(loc),
        ));
    });
    #[cfg(feature = "avian3d")]
    commands.spawn(RevoluteJoint::new(parent, object).with_local_anchor_1(loc));
}

pub fn generic_impulse_joint(
    commands: &mut Commands,
    parent: Entity,
    object: Entity,
    loc: Vec3,
    locked_axes: JointAxesMask,
    limits: (JointAxis, [f32; 2])
) {
    #[cfg(feature = "bevy_rapier3d")]
    commands.entity(object).with_children(|children| {
        let generic_joint = GenericJointBuilder::new(locked_axes)
            .limits(limits.0, limits.1)
            .local_anchor1(loc).build();
        children.spawn(ImpulseJoint::new(
            parent,
            TypedJoint::GenericJoint(generic_joint),
        ));
    });
}
