use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationConfig {
    pub gravity: [f32; 3],
    pub dimensions: u8,
    pub dt: f32,
    pub integrator: String,
}

#[derive(Debug, Clone)]
pub struct RigidBodyState {
    pub position: [f32; 3],
    pub orientation: [f32; 4],
    pub velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub contacts: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct ContactEventData {
    pub time: f32,
    pub body_a: String,
    pub body_b: String,
    pub contact_point: [f32; 3],
    pub normal: [f32; 3],
    pub impulse_magnitude: f32,
    pub relative_velocity: [f32; 3],
    pub event_type: String, // "started" | "ongoing" | "ended"
}

#[derive(Debug, Clone, Deserialize)]
pub struct JointDefinition {
    pub id: String,
    pub joint_type: String,  // "fixed", "revolute", "spherical", "prismatic"
    pub body_a: String,
    pub body_b: String,
    #[serde(default = "default_anchor")]
    pub anchor_a: [f32; 3],
    #[serde(default = "default_anchor")]
    pub anchor_b: [f32; 3],
    pub axis: Option<[f32; 3]>,
    pub limits: Option<[f32; 2]>,
}

fn default_anchor() -> [f32; 3] {
    [0.0, 0.0, 0.0]
}

#[allow(dead_code)]
pub struct Simulation {
    pub config: SimulationConfig,
    pub time: f32,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    gravity: Vector3<f32>,
    body_ids: HashMap<String, RigidBodyHandle>,
    body_names: HashMap<RigidBodyHandle, String>,

    // Contact tracking (Phase 1.2)
    previous_contacts: HashMap<(String, String), bool>,
    contact_events: Vec<ContactEventData>,

    // Joint tracking (Phase 1.3)
    joint_ids: HashMap<String, ImpulseJointHandle>,
    joint_names: HashMap<ImpulseJointHandle, String>,
}

impl Simulation {
    pub fn new(config: SimulationConfig) -> Self {
        let gravity = Vector3::new(config.gravity[0], config.gravity[1], config.gravity[2]);

        let mut integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = config.dt;

        Self {
            config,
            time: 0.0,
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            gravity,
            body_ids: HashMap::new(),
            body_names: HashMap::new(),
            previous_contacts: HashMap::new(),
            contact_events: Vec::new(),
            joint_ids: HashMap::new(),
            joint_names: HashMap::new(),
        }
    }

    pub fn add_body(
        &mut self,
        id: String,
        kind: String,
        shape: String,
        size: Vec<f32>,
        mass: Option<f32>,
        position: Option<[f32; 3]>,
        orientation: Option<[f32; 4]>,
        velocity: Option<[f32; 3]>,
        angular_velocity: Option<[f32; 3]>,
        friction: f32,
        restitution: f32,
        normal: Option<[f32; 3]>,
        offset: Option<f32>,
        linear_damping: Option<f32>,
        angular_damping: Option<f32>,
    ) {
        // Create rigid body
        let pos = position.unwrap_or([0.0, 0.0, 0.0]);
        let ori = orientation.unwrap_or([0.0, 0.0, 0.0, 1.0]);

        let rotation = UnitQuaternion::from_quaternion(Quaternion::new(ori[3], ori[0], ori[1], ori[2]));
        let isometry = Isometry::from_parts(
            Translation::new(pos[0], pos[1], pos[2]),
            rotation,
        );

        let rigid_body = match kind.as_str() {
            "static" => RigidBodyBuilder::fixed().position(isometry),
            "kinematic" => RigidBodyBuilder::kinematic_position_based().position(isometry),
            _ => {
                let mut builder = RigidBodyBuilder::dynamic().position(isometry);

                if let Some(vel) = velocity {
                    builder = builder.linvel(Vector3::new(vel[0], vel[1], vel[2]));
                }

                if let Some(ang_vel) = angular_velocity {
                    builder = builder.angvel(Vector3::new(ang_vel[0], ang_vel[1], ang_vel[2]));
                }

                // Apply damping (Phase 1.4)
                if let Some(ld) = linear_damping {
                    builder = builder.linear_damping(ld);
                }

                if let Some(ad) = angular_damping {
                    builder = builder.angular_damping(ad);
                }

                builder
            }
        };

        let body_handle = self.rigid_body_set.insert(rigid_body);

        // Create collider
        let collider = match shape.as_str() {
            "box" => {
                let half_extents = if size.len() >= 3 {
                    Vector3::new(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
                } else {
                    Vector3::new(0.5, 0.5, 0.5)
                };
                ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z)
            }
            "sphere" => {
                let radius = size.first().copied().unwrap_or(0.5);
                ColliderBuilder::ball(radius)
            }
            "capsule" => {
                let half_height = size.first().copied().unwrap_or(0.5);
                let radius = size.get(1).copied().unwrap_or(0.25);
                ColliderBuilder::capsule_y(half_height, radius)
            }
            "plane" => {
                // Plane shape: infinite plane defined by normal vector and offset
                let normal_arr = normal.unwrap_or([0.0, 1.0, 0.0]); // Default: upward facing
                let plane_offset = offset.unwrap_or(0.0);
                let normal_vec = Vector3::new(normal_arr[0], normal_arr[1], normal_arr[2]);
                ColliderBuilder::halfspace(UnitVector::new_normalize(normal_vec))
                    .translation(normal_vec * plane_offset)
            }
            _ => ColliderBuilder::ball(0.5), // Default to sphere
        }
        .friction(friction)
        .restitution(restitution);

        // Set mass for dynamic bodies
        let collider = if kind == "dynamic" {
            if let Some(m) = mass {
                collider.mass(m)
            } else {
                collider.density(1.0)
            }
        } else {
            collider
        };

        self.collider_set
            .insert_with_parent(collider, body_handle, &mut self.rigid_body_set);

        self.body_ids.insert(id.clone(), body_handle);
        self.body_names.insert(body_handle, id);
    }

    pub fn step(&mut self, steps: usize, dt: Option<f32>) {
        // Update dt if provided
        if let Some(new_dt) = dt {
            self.integration_parameters.dt = new_dt;
        }

        for _ in 0..steps {
            self.physics_pipeline.step(
                &self.gravity,
                &self.integration_parameters,
                &mut self.island_manager,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.rigid_body_set,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                &mut self.ccd_solver,
                Some(&mut self.query_pipeline),
                &(),
                &(),
            );

            // Detect contact events after physics step (Phase 1.2)
            self.detect_contact_events();

            self.time += self.integration_parameters.dt;
        }
    }

    pub fn get_body_state(&self, body_id: &str) -> Option<RigidBodyState> {
        let handle = self.body_ids.get(body_id)?;
        let body = self.rigid_body_set.get(*handle)?;

        let pos = body.translation();
        let rot = body.rotation();
        let vel = body.linvel();
        let ang_vel = body.angvel();

        // Find contacts for this body
        let mut contacts = Vec::new();
        for (collider_handle, _) in self.collider_set.iter() {
            for contact_pair in self.narrow_phase.contact_pairs_with(collider_handle) {
                // Check if contact is active (has manifolds)
                if !contact_pair.has_any_active_contact {
                    continue;
                }

                // Find which other body is in contact
                let other_collider = if contact_pair.collider1 == collider_handle {
                    contact_pair.collider2
                } else {
                    contact_pair.collider1
                };

                if let Some(other_collider_obj) = self.collider_set.get(other_collider) {
                    if let Some(other_body_handle) = other_collider_obj.parent() {
                        if let Some(other_name) = self.body_names.get(&other_body_handle) {
                            if !contacts.contains(other_name) {
                                contacts.push(other_name.clone());
                            }
                        }
                    }
                }
            }
        }

        Some(RigidBodyState {
            position: [pos.x, pos.y, pos.z],
            orientation: [rot.i, rot.j, rot.k, rot.w],
            velocity: [vel.x, vel.y, vel.z],
            angular_velocity: [ang_vel.x, ang_vel.y, ang_vel.z],
            contacts,
        })
    }

    pub fn get_all_bodies(&self) -> HashMap<String, RigidBodyState> {
        let mut result = HashMap::new();

        for (id, _handle) in &self.body_ids {
            if let Some(state) = self.get_body_state(id) {
                result.insert(id.clone(), state);
            }
        }

        result
    }

    /// Detect contact events (Phase 1.2)
    fn detect_contact_events(&mut self) {
        let mut current_contacts = HashMap::new();

        // Iterate through all active contact pairs
        for pair in self.narrow_phase.contact_pairs() {
            // Only track pairs with active contacts
            if !pair.has_any_active_contact {
                continue;
            }

            let collider_handle1 = pair.collider1;
            let collider_handle2 = pair.collider2;

            // Get body names
            let collider1 = self.collider_set.get(collider_handle1);
            let collider2 = self.collider_set.get(collider_handle2);

            if collider1.is_none() || collider2.is_none() {
                continue;
            }

            let body_handle1 = collider1.unwrap().parent();
            let body_handle2 = collider2.unwrap().parent();

            if body_handle1.is_none() || body_handle2.is_none() {
                continue;
            }

            let body_name1 = self.body_names.get(&body_handle1.unwrap());
            let body_name2 = self.body_names.get(&body_handle2.unwrap());

            if body_name1.is_none() || body_name2.is_none() {
                continue;
            }

            let name_a = body_name1.unwrap().clone();
            let name_b = body_name2.unwrap().clone();

            // Create sorted key to avoid duplicates
            let key = if name_a < name_b {
                (name_a.clone(), name_b.clone())
            } else {
                (name_b.clone(), name_a.clone())
            };

            current_contacts.insert(key.clone(), true);

            // Check if this is a NEW contact
            if !self.previous_contacts.contains_key(&key) {
                // Get contact details from first manifold
                if let Some(manifold) = pair.manifolds.first() {
                    let contact_point = manifold.local_n1;
                    let normal = manifold.local_n2;

                    // Get bodies for relative velocity
                    let body1 = self.rigid_body_set.get(body_handle1.unwrap());
                    let body2 = self.rigid_body_set.get(body_handle2.unwrap());

                    let rel_vel = if let (Some(b1), Some(b2)) = (body1, body2) {
                        let v1 = b1.linvel();
                        let v2 = b2.linvel();
                        [v1.x - v2.x, v1.y - v2.y, v1.z - v2.z]
                    } else {
                        [0.0, 0.0, 0.0]
                    };

                    // Estimate impulse magnitude from relative velocity
                    let impulse = (rel_vel[0].powi(2) + rel_vel[1].powi(2) + rel_vel[2].powi(2)).sqrt();

                    self.contact_events.push(ContactEventData {
                        time: self.time,
                        body_a: key.0.clone(),
                        body_b: key.1.clone(),
                        contact_point: [contact_point.x, contact_point.y, contact_point.z],
                        normal: [normal.x, normal.y, normal.z],
                        impulse_magnitude: impulse,
                        relative_velocity: rel_vel,
                        event_type: "started".to_string(),
                    });
                }
            }
        }

        // Check for ENDED contacts
        for (key, _) in &self.previous_contacts {
            if !current_contacts.contains_key(key) {
                self.contact_events.push(ContactEventData {
                    time: self.time,
                    body_a: key.0.clone(),
                    body_b: key.1.clone(),
                    contact_point: [0.0, 0.0, 0.0],
                    normal: [0.0, 0.0, 0.0],
                    impulse_magnitude: 0.0,
                    relative_velocity: [0.0, 0.0, 0.0],
                    event_type: "ended".to_string(),
                });
            }
        }

        self.previous_contacts = current_contacts;
    }

    /// Get and clear accumulated contact events
    pub fn get_and_clear_contact_events(&mut self) -> Vec<ContactEventData> {
        std::mem::take(&mut self.contact_events)
    }

    /// Add a joint between two bodies (Phase 1.3)
    pub fn add_joint(&mut self, def: JointDefinition) -> Result<String, String> {
        // Get body handles
        let handle_a = self.body_ids.get(&def.body_a)
            .ok_or(format!("Body '{}' not found", def.body_a))?;
        let handle_b = self.body_ids.get(&def.body_b)
            .ok_or(format!("Body '{}' not found", def.body_b))?;

        // Convert anchors to Point3
        let anchor_a = Point3::new(def.anchor_a[0], def.anchor_a[1], def.anchor_a[2]);
        let anchor_b = Point3::new(def.anchor_b[0], def.anchor_b[1], def.anchor_b[2]);

        // Create joint based on type and convert to GenericJoint
        let joint: GenericJoint = match def.joint_type.as_str() {
            "fixed" => {
                FixedJointBuilder::new()
                    .local_anchor1(anchor_a)
                    .local_anchor2(anchor_b)
                    .build()
                    .into()
            }

            "revolute" => {
                let axis = def.axis.unwrap_or([0.0, 1.0, 0.0]);
                let axis_unit = UnitVector::new_normalize(Vector3::new(axis[0], axis[1], axis[2]));

                let mut joint = RevoluteJointBuilder::new(axis_unit)
                    .local_anchor1(anchor_a)
                    .local_anchor2(anchor_b)
                    .build();

                // Apply limits if provided
                if let Some([min, max]) = def.limits {
                    joint.set_limits([min, max]);
                }

                joint.into()
            }

            "spherical" => {
                SphericalJointBuilder::new()
                    .local_anchor1(anchor_a)
                    .local_anchor2(anchor_b)
                    .build()
                    .into()
            }

            "prismatic" => {
                let axis = def.axis.unwrap_or([0.0, 1.0, 0.0]);
                let axis_unit = UnitVector::new_normalize(Vector3::new(axis[0], axis[1], axis[2]));

                let mut joint = PrismaticJointBuilder::new(axis_unit)
                    .local_anchor1(anchor_a)
                    .local_anchor2(anchor_b)
                    .build();

                // Apply limits if provided
                if let Some([min, max]) = def.limits {
                    joint.set_limits([min, max]);
                }

                joint.into()
            }

            _ => return Err(format!("Unknown joint type: '{}'", def.joint_type)),
        };

        // Insert joint into the physics world
        let joint_handle = self.impulse_joint_set.insert(
            *handle_a,
            *handle_b,
            joint,
            true,  // wake up bodies
        );

        // Track joint by name
        self.joint_ids.insert(def.id.clone(), joint_handle);
        self.joint_names.insert(joint_handle, def.id.clone());

        Ok(def.id)
    }
}
