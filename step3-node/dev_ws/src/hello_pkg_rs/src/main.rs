use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};

/// Creates a ROS 2 context and node, prints a hello message,
/// then spins until shutdown.
fn main() -> Result<()> {
    // Create a context and node, and print a hello message.
    let context: Context = Context::default_from_env()?;
    // Create a basic executor and a node, then print a hello message.
    let mut executor = context.create_basic_executor();
    // The node must be kept alive until shutdown, so we assign it to a variable.
    let _node = executor.create_node("hello_pkg_rs_node")?;
    println!("Hello from hello_pkg_rs package!");

    // Spin until shutdown, returning any error that occurs.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
