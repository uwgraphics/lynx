use crate::app_v2::app_structs::game_engine_structs::game_engine_status_structs::FrameCount;
use bevy::prelude::ResMut;


pub fn frame_counter(mut frame_count: ResMut<FrameCount>) {
    frame_count.0+=1;
}