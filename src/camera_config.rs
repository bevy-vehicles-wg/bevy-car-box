use bevy::prelude::*;
use bevy_trackball::TrackballController;

pub fn trackball_config() -> TrackballController {
    let mut trackball = TrackballController::default();
    trackball.input.map_wasd();
    trackball.input.slide_left_key = Some(KeyCode::ArrowLeft);
    trackball.input.slide_right_key = Some(KeyCode::ArrowRight);
    trackball.input.slide_far_key = Some(KeyCode::ArrowUp);
    trackball.input.slide_near_key = Some(KeyCode::ArrowDown);
    trackball.input.reset_key = Some(KeyCode::KeyR);
    trackball.input.orbit_button = Some(MouseButton::Right);
    trackball.input.focus = false;
    trackball.input.gamer_key = None;
    trackball.input.ortho_key = None;
    trackball.input.first_key = None;
    trackball.input.first_button = None;
    trackball.input.first_left_key = None;
    trackball.input.first_right_key = None;
    trackball.input.first_up_key = None;
    trackball.input.first_down_key = None;
    trackball.input.screw_left_key = None;
    trackball.input.screw_right_key = None;
    trackball.input.orbit_left_key = None;
    trackball.input.orbit_right_key = None;
    trackball.input.orbit_up_key = None;
    trackball.input.orbit_down_key = None;
    trackball.input.slide_button = None;
    trackball.input.slide_up_key = None;
    trackball.input.slide_down_key = None;
    trackball.input.scale_in_key = None;
    trackball.input.scale_out_key = None;

    trackball
}
