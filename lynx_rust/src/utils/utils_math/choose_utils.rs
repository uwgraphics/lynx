

pub fn n_choose_k(n: u64, k: u64) -> u64 {
    let mut res = 1;
    for i in 0..k {
        res = (res * (n - i)) /
              (i + 1);
    }
    res
}