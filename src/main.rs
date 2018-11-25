extern crate nalgebra as na;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate nphysics_testbed2d;

use na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Cuboid, ShapeHandle};
use nphysics2d::joint::RevoluteJoint;
use nphysics2d::object::{BodyHandle, Material};
use nphysics2d::volumetric::Volumetric;
use nphysics2d::world::World;
use nphysics_testbed2d::Testbed;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector2::new(0.0, -9.81));

    /*
     * Ground.
     */
    let ground_radx = 25.0;
    let ground_rady = 1.0;
    let ground_shape = ShapeHandle::new(Cuboid::new(Vector2::new(
        ground_radx - COLLIDER_MARGIN,
        ground_rady - COLLIDER_MARGIN,
    )));

    let ground_pos = Isometry2::new(-Vector2::y() * 5.0, na::zero());
    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );
    let base = BodyHandle::ground();

    /*
     * Upper joint
     */
    let upper_revo = RevoluteJoint::new(0.0);
    let upper_geom = ShapeHandle::new(Cuboid::new(Vector2::new(
        0.5 - COLLIDER_MARGIN,
        0.05 - COLLIDER_MARGIN,
    )));
    let upper_inertia = upper_geom.inertia(1.0);
    let upper_center_of_mass = upper_geom.center_of_mass();

    let upper = world.add_multibody_link(
        base,
        upper_revo,
        na::zero(),
        Vector2::new(-0.6, 0.0),
        upper_inertia,
        upper_center_of_mass,
    );

    world.add_collider(
        COLLIDER_MARGIN,
        upper_geom.clone(),
        upper,
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Lower joint
     */
    let lower_revo = RevoluteJoint::new(0.0);
    let lower_geom = ShapeHandle::new(Cuboid::new(Vector2::new(
        0.3 - COLLIDER_MARGIN,
        0.05 - COLLIDER_MARGIN,
    )));
    let lower_inertia = lower_geom.inertia(1.0);
    let lower_center_of_mass = lower_geom.center_of_mass();

    let lower = world.add_multibody_link(
        upper,
        lower_revo,
        Vector2::new(0.6, 0.0),
        Vector2::new(-0.4, 0.0),
        lower_inertia,
        lower_center_of_mass,
    );

    world.add_collider(
        COLLIDER_MARGIN,
        lower_geom.clone(),
        lower,
        Isometry2::identity(),
        Material::default(),
    );

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    testbed.hide_performance_counters();
    testbed.look_at(Point2::new(1.0, 2.0), 130.0);
    use std::sync::Mutex;

    let v = Mutex::new(0.0);
    enum Direction {
        Forward,
        Backward,
    };
    let dir = Mutex::new(Direction::Forward);

    testbed.add_callback(move |world_owner, _, _| {
        let mut world = world_owner.get_mut();
        let mut link = world.multibody_link_mut(upper).unwrap();

        let rev_joint: &mut RevoluteJoint<f32> = link.joint_mut().downcast_mut().unwrap();
        rev_joint.enable_angular_motor();
        let mut v = v.lock().unwrap();

        rev_joint.set_desired_angular_motor_velocity(*v);
        let mut dir = dir.lock().unwrap();
        if *v < -5.0 || *v > 5.0 {
            *dir = match *dir {
                Direction::Forward => Direction::Backward,
                Direction::Backward => Direction::Forward,
            };
        }
        match *dir {
            Direction::Forward => *v += 0.01,
            Direction::Backward => *v -= 0.01,
        }
    });
    testbed.run();
}
