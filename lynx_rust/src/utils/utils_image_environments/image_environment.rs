use crate::utils::utils_files_and_strings::file_utils::{*};
use crate::utils::utils_math::geometry_utils::{*};
use crate::utils::utils_sampling::prelude::*;
use crate::utils::utils_vars::prelude::*;
use nalgebra::Vector2;
use std::process::Command;
use termion::{color, style};
use nalgebra::DVector;

#[derive(Clone, Debug)]
pub struct ImageEnvironment {
    pub image: Vec<Vec<f64>>,
    pub image_name: String,
    pub pixel_width: usize,
    pub pixel_height: usize,
    pub world_width: f64,
    pub world_height: f64
}
impl ImageEnvironment {
    // origin is upper left, coordinates are (down [i.e., y], right [i.e., x])
    pub fn new_objective_image(image_name: &str) -> Self {
        let mut path_to_src = get_path_to_src();
        let fp = path_to_src + "assets/image_environments/objective_images/" + image_name + "/" + "numeric_image.txt";
        let res = Self::_create_numeric_images_if_need_be(&fp);
        if res.is_err() {
            println!("{}{}WARNING: There was an error when trying to make a numeric image for image {}.  Returning empty image.{}", color::Fg(color::Yellow), style::Bold, image_name, style::Reset);
            return ImageEnvironment::_new_empty();
        } else {
            return ImageEnvironment::_new(&fp, image_name.to_string())
        }
    }

    pub fn new_collision_image(image_name: &str) -> Self {
        let mut path_to_src = get_path_to_src();
        let fp = path_to_src + "assets/image_environments/collision_images/" + image_name + "/" + "numeric_image.txt";
        let res = Self::_create_numeric_images_if_need_be(&fp);
        if res.is_err() {
            println!("{}{}WARNING: There was an error when trying to make a numeric image for image {}.  Returning empty image.{}", color::Fg(color::Yellow), style::Bold, image_name, style::Reset);
            return ImageEnvironment::_new_empty();
        } else {
            return ImageEnvironment::_new(&fp, image_name.to_string())
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _new(fp: &String, image_name: String) -> Self {
        let contents = read_file_contents(fp.clone());
        if contents.is_none() {
            println!("{}{}WARNING: Contents of image {} is empty.  Returning empty image.{}", color::Fg(color::Yellow), style::Bold, image_name, style::Reset);
            return Self::_new_empty();
        }

        let lines: Vec<&str> = contents.as_ref().unwrap().lines().collect();
        let num_lines = lines.len();

        let mut image: Vec<Vec<f64>> = Vec::new();

        for i in 0..num_lines {
            let mut tmp: Vec<f64> = Vec::new();
            image.push(tmp);
            let values_split = lines[i].split(',');
            let values: Vec<&str> = values_split.collect();
            let l = values.len();
            for j in 0..l {
                if !(values[j] == "") {
                    let val: f64 = values[j].parse().unwrap();
                    image[i].push(val);
                }
            }
        }

        let pixel_height = image.len();
        let pixel_width = image[0].len();

        let world_height = 1.0;
        let world_width = pixel_width as f64 / pixel_height as f64;
        Self {image, image_name, pixel_width, pixel_height, world_width, world_height}
    }

    fn _new_empty() -> Self {
        let image = Vec::new();
        let image_name = "".to_string();

        return Self { image, image_name, pixel_width: 0, pixel_height: 0, world_width: 0.0, world_height: 0.0 }
    }

    fn _create_numeric_images_if_need_be(fp: &String) -> Result<(), ()> {
        let file_exists = check_if_path_exists(fp.clone());
        if !file_exists {
            println!("{}{}Converting image to numeric image using python script.  {}", color::Fg(color::Blue), style::Bold, style::Reset);
            let mut c = Command::new("python2");
            let fp = get_path_to_src() + "assets/image_environments/python_image_utils/convert_images_to_numeric_images.py";
            c.arg( fp.as_str() );
            if c.status().is_err() {
                return Err(());
            }
            if !c.status().ok().unwrap().success() {
                return Err(());
            }
            return Ok(());
        }
        return Ok(());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    pub fn query_image(&self, y: f64, x: f64, interpolate: bool) -> f64 {
        if interpolate {
            return self._interpolate_value(y, x);
        } else {
            let i = self.world_space_to_image_space(y, x);
            return self.image[i.0][i.1]
        }
    }

    pub fn image_space_to_world_space(&self, y: usize, x: usize, clamp: bool) -> (f64, f64) {
        // note: both image space and world space have origin in upper left corner of image
        let mut world_space_y = y as f64 / self.pixel_height as f64;
        if clamp {
            world_space_y = world_space_y.min(1.0).max(0.0);
        }

        let mut world_space_x = x as f64 / self.pixel_width as f64;
        world_space_x = world_space_x * self.world_width;
        if clamp {
            world_space_x = world_space_x.min(self.world_width).max(0.0);
        }

        (world_space_y, world_space_x)
    }

    pub fn world_space_to_image_space(&self, y: f64, x: f64) -> (usize, usize) {
        // note: both image space and world space have origin in upper left corner of image
        let mut image_space_y: usize = (self.pixel_height as f64 * y).round() as usize;
        image_space_y = image_space_y.max(0).min(self.pixel_height-1);
        let mut image_space_x= ((self.pixel_width as f64 * x)/ self.world_width).round() as usize;
        image_space_x = image_space_x.max(0).min(self.pixel_width-1);

        (image_space_y,image_space_x)
    }

    pub fn world_space_to_image_space_nonrounded(&self, y: f64, x: f64) -> (f64, f64) {
        // note: both image space and world space have origin in upper left corner of image
        let mut image_space_y = (self.pixel_height as f64 * y);
        let mut image_space_x= ((self.pixel_width as f64 * x)/ self.world_width);

        (image_space_y,image_space_x)
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    fn _interpolate_value(&self, y: f64, x: f64) -> f64 {
        let mut lower_bound_y: usize = (self.pixel_height as f64 * y).floor().max(0.0).min((self.pixel_height - 1)  as f64) as usize;
        let mut upper_bound_y: usize = (self.pixel_height as f64 * y).ceil().max(0.0).min((self.pixel_height-1) as f64) as usize;
        let mut lower_bound_x = ((self.pixel_width as f64 * x)/ self.world_width).floor().max(0.0).min((self.pixel_width-1) as f64) as usize;
        let mut upper_bound_x = ((self.pixel_width as f64 * x)/self.world_width).ceil().max(0.0).min((self.pixel_width-1) as f64) as usize;

        if lower_bound_y == upper_bound_y {
            if lower_bound_y == 0 {
                upper_bound_y += 1
            } else if  upper_bound_y == self.pixel_height - 1 {
                lower_bound_y -= 1
            } else {
                upper_bound_y += 1
            }
        }
        if lower_bound_x == upper_bound_x {
            if lower_bound_x == 0 {
                upper_bound_x += 1
            } else if  upper_bound_x == self.pixel_width - 1 {
                lower_bound_x -= 1
            } else {
                upper_bound_x += 1
            }
        }

        let mut image_space_pt = self.world_space_to_image_space_nonrounded(y, x);

        let mut lower_left = Vector2::new(lower_bound_y as f64, lower_bound_x as f64);
        let mut upper_left = Vector2::new(upper_bound_y as f64, lower_bound_x as f64);
        let mut lower_right = Vector2::new(lower_bound_y as f64, upper_bound_x as f64);
        let mut upper_right = Vector2::new(upper_bound_y as f64, upper_bound_x as f64);
        let mut pt = Vector2::new(image_space_pt.0, image_space_pt.1);

        let q = quadratic_barycentric_coordinates(&pt, &lower_left, &upper_left, &upper_right, &lower_right);

        let ret = q.0*self.image[lower_bound_y][lower_bound_x] + q.1*self.image[upper_bound_y][lower_bound_x] + q.2*self.image[upper_bound_y][upper_bound_x] + q.3*self.image[lower_bound_y][upper_bound_x];
        ret

    }
}

impl FloatVecSampler for ImageEnvironment {
    fn float_vec_sampler_sample(&self) -> Result<DVector<f64>, String> {
        let range_sampler = RectangleFloatVecSampler::new(vec![ (0.0, self.world_height), (0.0, self.world_width) ]);
        return range_sampler.float_vec_sampler_sample();
    }
}
impl LynxFloatVecSampler for ImageEnvironment {
    fn lynx_float_vec_sampler_sample(&self, lynx_vars: &mut LynxVarsGeneric) -> Result<DVector<f64>, String> {
        let range_sampler = RectangleFloatVecSampler::new(vec![ (0.0, self.world_height), (0.0, self.world_width) ]);
        return range_sampler.float_vec_sampler_sample();
    }
}
impl LynxVarsUser for ImageEnvironment{ }
unsafe impl Send for ImageEnvironment { }
unsafe impl Sync for ImageEnvironment { }