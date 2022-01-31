use rapier3d::na::Isometry3;
use rapier3d::prelude::*;
use rapier_testbed3d::{PhysicsState, Testbed, TestbedApp, TestbedGraphics};
use rapier_testbed3d::harness::RunState;
use rapier_testbed3d::physics::PhysicsEvents;


fn init(testbed: &mut Testbed) {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();
    
    let kinematic = {
        let rigid_body = RigidBodyBuilder::kinematic_position_based()
            .translation(vector![0.0, 0.0, 0.0])
            .build();
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::ball(1.0).build();
        colliders.insert_with_parent(collider, handle, &mut bodies);
        
        handle
    };
    
    let dynamic = {
        let rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, -5.0, 0.0])
            .linvel(vector![0.1, 0.0, 0.1])
            .build();
        let handle = bodies.insert(rigid_body);
        let collider = ColliderBuilder::ball(1.0).build();
        colliders.insert_with_parent(collider, handle, &mut bodies);
    
        handle
    };
    
    let mut joint = GenericJoint::new(JointAxesMask::LIN_X | JointAxesMask::LIN_Y | JointAxesMask::LIN_Z);
    
    joint.set_local_anchor2(point!(0.0, -5.0, 0.0))
         .set_motor(JointAxis::AngX, 0.0, 0.0, 0.0, 0.5)
         .set_motor(JointAxis::AngZ, 0.0, 0.0, 0.0, 0.5)
         .set_limits(JointAxis::AngX, [0.0, 0.5])
         .set_limits(JointAxis::AngZ, [0.0, 0.5]);
    
    joint.coupled_axes |= JointAxesMask::ANG_X | JointAxesMask::ANG_Z;
    
    impulse_joints.insert(kinematic, dynamic, joint, true);
    
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![20.0, 12.0, 4.0], point![0.0, 0.0, 0.0]);
    
    let mut pushed = false;
    testbed.add_callback(move |_: Option<&mut TestbedGraphics>, physics: &mut PhysicsState, _: &PhysicsEvents, run_state: &RunState| {
        if run_state.time > 3.0 && !pushed {
            pushed = true;
            
            if let Some(dynamic) = physics.bodies.get_mut(dynamic) {
                dynamic.apply_impulse(vector![5.0, 0.0, 0.0], true);
            }
        }
    });
}

fn main() {
    let testbed = TestbedApp::from_builders(0, vec![("Issue", init)]);
    testbed.run()
}
