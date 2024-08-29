use bevy::diagnostic::DiagnosticsStore;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::pbr::wireframe::WireframeConfig;
use bevy::pbr::wireframe::WireframePlugin;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy::winit::WinitWindows;
pub use bevy_mod_outline::OutlineBundle;
use bevy_mod_outline::OutlinePlugin;
pub use bevy_mod_outline::OutlineVolume;
use std::io::Cursor;
use winit::window::Icon;

pub struct DebugPlugin;

#[derive(Component)]
struct FpsText;

#[derive(Resource, Default)]
pub struct DebugConfig {
    pub show_aabb: bool,
    pub show_sphere_bb: bool,
    pub show_debug_material: bool,
}

impl Plugin for DebugPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(OutlinePlugin)
            .add_plugins(FrameTimeDiagnosticsPlugin)
            .add_plugins(WireframePlugin)
            .add_systems(PreStartup, set_window_icon)
            .add_systems(Startup, setup_fps_text)
            .add_systems(Update, update_fps_text)
            .add_systems(Update, update_wireframe_config)
            .add_systems(Update, update_outline_config)
            .insert_resource(WireframeConfig {
                global: false,
                default_color: Color::WHITE,
            })
            .insert_resource(DebugConfig::default());
    }
}

fn set_window_icon(
    windows: NonSend<WinitWindows>,
    primary_window: Query<Entity, With<PrimaryWindow>>,
) {
    let primary_entity = primary_window.single();
    let primary = windows.get_window(primary_entity).unwrap();
    let icon_buf = Cursor::new(include_bytes!("../assets/icons/bevy.png"));
    if let Ok(image) = image::load(icon_buf, image::ImageFormat::Png) {
        let image = image.into_rgba8();
        let (width, height) = image.dimensions();
        let rgba = image.into_raw();
        let icon = Icon::from_rgba(rgba, width, height).unwrap();
        primary.set_window_icon(Some(icon));
    };
}

fn setup_fps_text(mut commands: Commands) {
    println!("fps text is setup...");
    // FPS text
    commands.spawn((
        // Create a TextBundle that has a Text with a list of sections.
        TextBundle::from_sections([
            TextSection::new(
                "FPS: ",
                TextStyle {
                    font_size: 20.0,
                    ..default()
                },
            ),
            TextSection::from_style(TextStyle {
                font_size: 20.0,
                color: Color::WHITE,
                ..default()
            }),
        ])
        .with_style(Style {
            position_type: PositionType::Absolute,
            top: Val::Px(5.0),
            right: Val::Px(5.0),
            ..default()
        }),
        FpsText,
    ));
}

/// set outlines on meshes whether to be visible or not
fn update_outline_config(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut outline_query: Query<&mut OutlineVolume>,
) {
    if keyboard_input.just_pressed(KeyCode::KeyX) {
        for mut outline in outline_query.iter_mut() {
            outline.visible = !outline.visible;
        }
    }
}

/// update the fps value
fn update_fps_text(diagnostics: Res<DiagnosticsStore>, mut query: Query<&mut Text, With<FpsText>>) {
    for mut text in &mut query {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                text.sections[1].value = format!("{value:.1}");
            }
        }
    }
}

fn update_wireframe_config(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut wireframe_config: ResMut<WireframeConfig>,
    mut conf: ResMut<DebugConfig>,
) {
    if keyboard_input.just_pressed(KeyCode::KeyZ) {
        wireframe_config.global = !wireframe_config.global;
        conf.show_aabb = !conf.show_aabb;
        conf.show_sphere_bb = !conf.show_sphere_bb;
        conf.show_debug_material = !conf.show_debug_material;
    }
}
