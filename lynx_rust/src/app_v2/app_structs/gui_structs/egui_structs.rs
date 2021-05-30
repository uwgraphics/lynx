use crate::app_v2::app_type_enums::enums::EguiWindowType;
use crate::utils::utils_files_and_strings::prelude::u64_to_string;
use rand::Rng;
use bevy_egui::egui::*;
use termion::{style, color};

/*
impl EguiWindowContainer {
    pub fn add_new_egui_window(&mut self, open: bool, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> usize {
        let add_idx = self.egui_windows.len();
        self.egui_windows.push(
            EguiWindow {
                open,
                name_str: self._get_random_name_string(),
                egui_window_type,
                additional_identifier
            }
        );
        return add_idx;
    }

    pub fn delete_egui_window(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) {
        let idx = self._find_egui_window_idx(egui_window_type, additional_identifier);
        if idx.is_some() {
            let idx_unwrap = idx.unwrap();
            self.egui_windows.remove(idx_unwrap);
        }
    }

    pub fn delete_egui_windows_of_same_type(&mut self, egui_window_type: EguiWindowType) {
        let mut idxs = self._find_egui_window_idxs_of_same_type(egui_window_type);
        idxs.reverse();
        for i in idxs {
            self.egui_windows.remove(i);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn get_egui_window_mut_ref(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> Option<&mut EguiWindow> {
        let idx = self._find_egui_window_idx(egui_window_type, additional_identifier);
        if idx.is_some() {
            return Some(&mut self.egui_windows[idx.unwrap()]);
        } else {
            return None;
        }
    }

    pub fn get_egui_window_mut_ref_add_if_necessary(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> &mut EguiWindow {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            return &mut self.egui_windows[idx.unwrap()];
        } else {
            let add_idx = self.add_new_egui_window(false, egui_window_type, additional_identifier);
            return &mut self.egui_windows[add_idx];
        }
    }

    fn _find_egui_window_idx(&self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> Option<usize> {
        let l = self.egui_windows.len();
        for i in 0..l {
            if self.egui_windows[i].egui_window_type == egui_window_type {
                if additional_identifier.is_some() && self.egui_windows[i].additional_identifier.is_some() {
                    if additional_identifier.as_ref().unwrap() == self.egui_windows[i].additional_identifier.as_ref().unwrap() {
                        return Some(i);
                    }
                } else if additional_identifier.is_none() {
                    return Some(i);
                }
            }
        }
        return None;
    }

    fn _find_egui_window_idxs_of_same_type(&self, egui_window_type: EguiWindowType) -> Vec<usize> {
        let mut out_vec = Vec::new();

        let l = self.egui_windows.len();
        for i in 0..l {
            if self.egui_windows[i].egui_window_type == egui_window_type {
                out_vec.push(i);
            }
        }
        return out_vec;
    }
}
*/

#[derive(Clone, Debug)]
pub struct EguiWindowContainer {
    pub egui_windows: Vec<EguiWindow>
}

impl EguiWindowContainer {
    pub fn new_empty() -> Self {
        Self { egui_windows: Vec::new() }
    }

    pub fn open_window(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            self.egui_windows[idx.unwrap()].open = true;
        } else {
            self._add_new_egui_window(true, egui_window_type, additional_identifier);
        }
    }

    pub fn close_window(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>, delete: bool) {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            self.egui_windows[idx.unwrap()].open = false;
            if delete {
                self._delete_egui_window(egui_window_type.clone(), additional_identifier);
            }
        } else {
            if !delete {
                self._add_new_egui_window(false, egui_window_type, additional_identifier);
            }
        }
    }

    pub fn close_window_by_idx(&mut self, idx: usize, delete: bool) -> Result<(), String> {
        if idx >= self.egui_windows.len() {
            return Err(format!("idx {:?} is too high for number of egui_windows ({:?})", idx, self.egui_windows.len()));
        }
        if delete {
            self.egui_windows.remove(idx);
        } else {
            self.egui_windows[idx].open = false;
        }
        Ok(())
    }

    /*
    pub fn get_window_handle(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> Window {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            let name_str = self.egui_windows[idx.unwrap()].name_str.clone();
            return Window::new(name_str);
        } else {
            let add_idx = self.egui_windows.len();
            self._add_new_egui_window(false, egui_window_type.clone(), additional_identifier.clone());
            let name = self.egui_windows[add_idx].name_str.clone();
            return Window::new(name);
        }
    }

    pub fn get_window_handle2(&self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> Window {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            let name_str = self.egui_windows[idx.unwrap()].name_str.clone();
            return Window::new(name_str);
        } else {
            return Window::new("");
        }
    }
    */

    /*
    pub fn get_window_handle_and_window_is_open_bundle(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> (Window, bool) {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            let name = self.egui_windows[idx.unwrap()].name_str.clone();
            let open = self.egui_windows[idx.unwrap()].open.clone();
            return (Window::new(""), open);
        } else {
            let add_idx = self.egui_windows.len();
            self._add_new_egui_window(false, egui_window_type.clone(), additional_identifier.clone());
            let name = self.egui_windows[add_idx].name_str.clone();
            return (Window::new(""), false);
        }
    }

    pub fn get_window_handle_and_window_is_open_bundle2(&self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> (Window, bool) {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            let window = &self.egui_windows[idx.unwrap()];
            return (Window::new(window.name_str.clone()), window.open);
        } else {
            return (Window::new(""), false);
        }
    }
    */

    pub fn get_window_name_str(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> String {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            return self.egui_windows[idx.unwrap()].name_str.clone();
        } else {
            let add_idx = self.egui_windows.len();
            self._add_new_egui_window(false, egui_window_type.clone(), additional_identifier.clone());
            return self.egui_windows[add_idx].name_str.clone();
        }
    }

    pub fn get_window_name_str_and_window_is_open_bundle(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> (String, bool, usize) {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_some() {
            return (self.egui_windows[idx.unwrap()].name_str.clone(), self.egui_windows[idx.unwrap()].open, idx.unwrap());
        } else {
            let add_idx = self.egui_windows.len();
            self._add_new_egui_window(false, egui_window_type.clone(), additional_identifier.clone());
            return (self.egui_windows[add_idx].name_str.clone(), false, add_idx);
        }
    }

    pub fn get_window_is_open(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> Option<bool> {
        let idx = self._find_egui_window_idx(egui_window_type.clone(), additional_identifier.clone());
        if idx.is_none() { return None; }
        return Some(self.egui_windows[idx.unwrap()].open);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _add_new_egui_window(&mut self, open: bool, egui_window_type: EguiWindowType, additional_identifier: Option<String>) {
        self.egui_windows.push(
            EguiWindow {
                open,
                name_str: Self::_get_random_name_string(),
                egui_window_type,
                additional_identifier
            }
        )
    }

    fn _delete_egui_window(&mut self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) {
        let idx = self._find_egui_window_idx(egui_window_type, additional_identifier);
        if idx.is_some() {
            let idx_unwrap = idx.unwrap();
            self.egui_windows.remove(idx_unwrap);
        }
    }

    fn _delete_egui_windows_of_same_type(&mut self, egui_window_type: EguiWindowType) {
        let mut idxs = self._find_egui_window_idxs_of_same_type(egui_window_type);
        idxs.reverse();
        for i in idxs {
            self.egui_windows.remove(i);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _find_egui_window_idx(&self, egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> Option<usize> {
        let l = self.egui_windows.len();
        for i in 0..l {
            if self.egui_windows[i].egui_window_type == egui_window_type {
                if additional_identifier.is_some() && self.egui_windows[i].additional_identifier.is_some() {
                    if additional_identifier.as_ref().unwrap() == self.egui_windows[i].additional_identifier.as_ref().unwrap() {
                        return Some(i);
                    }
                } else if additional_identifier.is_none() && self.egui_windows[i].additional_identifier.is_none() {
                    return Some(i);
                }
            }
        }
        return None;
    }

    fn _find_egui_window_idxs_of_same_type(&self, egui_window_type: EguiWindowType) -> Vec<usize> {
        let mut out_vec = Vec::new();

        let l = self.egui_windows.len();
        for i in 0..l {
            if self.egui_windows[i].egui_window_type == egui_window_type {
                out_vec.push(i);
            }
        }
        return out_vec;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _get_random_name_string() -> String {
        let mut rng = rand::thread_rng();
        let n: u64 = rng.gen();
        return u64_to_string(n);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn print_summary(&self) {
        let l = self.egui_windows.len();
        for i in 0..l {
            println!("{}{}egui_window {} ---> {}", style::Bold, color::Fg(color::Blue), i, style::Reset);
            println!("      name_str: {:?}", self.egui_windows[i].name_str);
            println!("      open: {:?}", self.egui_windows[i].open);
            println!("      egui_window_type: {:?}", self.egui_windows[i].egui_window_type);
            println!("      additional_identifier: {:?}", self.egui_windows[i].additional_identifier);
        }
        println!();
    }
}

#[derive(Clone, Debug)]
pub struct EguiWindow {
    pub open: bool,
    pub name_str: String,
    pub egui_window_type: EguiWindowType,
    pub additional_identifier: Option<String>
}
impl EguiWindow {
    pub fn new(egui_window_type: EguiWindowType, additional_identifier: Option<String>) -> Self {
        Self {
            open: false,
            name_str: Self::_get_random_name_string(),
            egui_window_type,
            additional_identifier
        }
    }

    fn _get_random_name_string() -> String {
        let mut rng = rand::thread_rng();
        let n: u64 = rng.gen();
        return u64_to_string(n);
    }
}