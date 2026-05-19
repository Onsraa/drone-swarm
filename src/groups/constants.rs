/// Where drone-visibility presets are read from / written to. Lives at
/// the workspace root, not under `assets/`, so it's user state rather
/// than tracked content. Add to `.git/info/exclude` to keep it out of
/// VCS.
pub const PRESETS_PATH: &str = "presets.txt";
