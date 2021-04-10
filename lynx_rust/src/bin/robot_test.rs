extern crate lynx_lib;
use lynx_lib::robot_modules::robot_module_toolbox::RobotModuleToolbox;

fn main() -> Result<(), String>{
    let mut r = RobotModuleToolbox::new("hubo", Some("c2"), None)?;

    // r.get_dof_module_ref().print_joint_dof_order();
    // r.get_configuration_module_ref().robot_model_module.print_link_order();

    r.get_salient_links_module_ref().print_summary();

    println!("{:?}", r.get_configuration_module_ref().robot_model_module.robot_name);

    r.get_dof_module_ref().print_joint_dof_order();

    let mut fk_res = r.get_fk_module_ref().compute_fk_vec(&vec![0.,0.,0.,0.,-1.73,0.,0.,0.,0.,0.,0.,-1.73,0.,0.,0.])?;

    r.get_core_collision_module_mut_ref().add_not_in_collision_example(&fk_res);

    Ok(())
}