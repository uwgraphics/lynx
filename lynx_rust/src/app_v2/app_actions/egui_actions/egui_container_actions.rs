use crate::app_v2::app_structs::gui_structs::egui_structs::*;
use bevy_egui::egui::*;
use bevy::prelude::{Query, Entity, Commands};
use crate::app_v2::app_type_enums::enums::EguiWindowType;

/*
pub fn get_egui_window_handle<'a>(egui_window_type: EguiWindowType,
                                  additional_identifier: Option<String>,
                                  delete_on_close_by_user: bool,
                                  commands: &mut Commands,
                                  query: &mut Query<(Entity, &mut EguiWindow)>) -> Window<'a> {

    let mut found_entity: Option<Entity> = None;
    let mut found_egui_window: Option<&mut EguiWindow> = None;

    for (entity_, mut egui_window_) in query.iter_mut() {
        let egui_window: &mut EguiWindow = &mut egui_window_;

        if egui_window.egui_window_type == egui_window_type {
            if additional_identifier.is_some() && egui_window.additional_identifier.is_some() {
                if additional_identifier.as_ref().unwrap() == egui_window.additional_identifier.as_ref().unwrap() {
                    found_entity = Some(entity_.clone());
                    found_egui_window = Some(egui_window);
                }
            } else if additional_identifier.is_none() {
                found_entity = Some(entity_.clone());
                found_egui_window = Some(egui_window);
            }
        }
    }

    if found_entity.is_none() {
        let mut new_window = EguiWindow::new(egui_window_type.clone(), additional_identifier.clone());
        commands.spawn().insert(new_window.clone());

        return Window::new(new_window.name_str).open(&mut false);
    } else {
        let found_entity_unwrap = found_entity.as_ref().unwrap().clone();
        let found_egui_window_unwrap = found_egui_window.as_mut().unwrap();

    }

    unimplemented!()
}
*/