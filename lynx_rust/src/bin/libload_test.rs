use libloading;

fn main() {
    unsafe {
        let lib = libloading::Library::new("/home/rakita/lynx/lynx_rust/src/bin/libnn_lib.so");
        let lib_unwrap = lib.as_ref().ok().unwrap();
    }
}