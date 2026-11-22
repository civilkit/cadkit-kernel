//! STEP export (rs_paths: ck-io/src/step/write.rs) and import (parse.rs, read.rs).
pub mod parse;
pub mod read;
pub mod recog;
pub mod write;
pub use parse::ImportError;
pub use read::{read_step, read_step_plain, Imported, SolidRead};
pub use write::{
    try_write_step, try_write_step_with_meta, try_write_step_with_surfaces, write_step, write_step_signed_zeros,
    write_step_with_meta, StepExportMeta, StepOpts, StepWriter,
};
