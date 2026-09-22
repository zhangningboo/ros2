use moveit_msgs::action::{MoveGroup, MoveGroup_Goal};
use moveit_msgs::msg::{MotionPlanRequest, PlanningOptions};
use rclrs::{
    log_info,
    Context,
    CreateBasicExecutor,
    SpinOptions,
};

fn main() -> Result<(), rclrs::RclrsError> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("rust_moveit_demo")?;

    log_info!(node.logger(), "creating MoveIt action client");

    let client = node.create_action_client::<MoveGroup>("/move_action")?;

    log_info!(node.logger(), "MoveIt action client created");

    let request = MotionPlanRequest {
        group_name: "left_arm".to_string(),
        ..Default::default()
    };

    let planning_options = PlanningOptions {
        plan_only: true,
        ..Default::default()
    };

    let goal = MoveGroup_Goal {
        request,
        planning_options,
    };

    log_info!(node.logger(), "calling request_goal()");

    let goal_request = client.request_goal(goal);

    let promise = executor.commands().run(async move {
        println!("async task started");
        println!("waiting for MoveIt goal response...");

        match goal_request.await {
            Some(goal_client) => {
                println!("MoveIt accepted the goal");

                println!("waiting for MoveIt result...");

                let (status, result) = goal_client.result.await;

                println!("MoveIt action status: {:?}", status);
                println!("MoveIt error code: {}", result.error_code.val);
            }

            None => {
                println!("MoveIt rejected the goal");
            }
        }
    });

    executor.spin(
        SpinOptions::default()
            .until_promise_resolved(promise),
    );

    Ok(())
}