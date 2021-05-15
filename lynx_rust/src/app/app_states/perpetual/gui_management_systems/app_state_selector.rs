use crate::robot_modules::prelude::*;
use crate::app::app_states::res_comps::*;
use crate::app::app_states::app_states_enum::*;
use crate::app::app_utils::gui_utils::robot_selection_choices::*;
use bevy_egui::egui::Ui;
use bevy::prelude::{ResMut, State};
use bevy_egui::egui;


pub fn gui_app_state_selector(ui: &mut Ui,
                              current_main_gui_values: &mut ResMut<CurrentMainGUIValues>,
                              state: &mut ResMut<State<AppState>>) {
    egui::CollapsingHeader::new("Application Selection").show(ui, |ui| {

        let mut all_states = vec![
            (AppState::JointValueSliders, "Joint Value Sliders"),
            (AppState::PathPlanning, "Path Planning")
        ];

        for a in all_states {
            let mut selected = false;
            let current_selected_state = &current_main_gui_values.curr_app_state;
            if current_selected_state.is_some() && current_selected_state.as_ref().unwrap() == &a.0 { selected = true; }
            let mut selected_display = selected.clone();
            if ui.checkbox(&mut selected_display, a.1).clicked() {
                if !selected {
                    current_main_gui_values.curr_app_state = Some(a.0.clone());
                    state.set(a.0.clone()).unwrap();
                } else {
                    current_main_gui_values.curr_app_state = None;
                    state.set(AppState::NullState).unwrap();
                }
            }
        }


    });
}