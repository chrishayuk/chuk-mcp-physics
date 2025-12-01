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

#[derive(Debug, Clone)]
pub struct DragProperties {
    pub drag_coefficient: f32,  // Base drag coefficient (Cd)
    pub drag_area: f32,  // Reference cross-sectional area (m²)
    pub drag_axis_ratios: [f32; 3],  // Drag variation along body axes [x, y, z]
    pub fluid_density: f32,  // Fluid density (kg/m³)
    pub use_damping_only: bool,  // If true, use Rapier's damping instead of force-based drag
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

    // Drag tracking (Phase 2)
    body_drag_properties: HashMap<RigidBodyHandle, DragProperties>,
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
            body_drag_properties: HashMap::new(),
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
        drag_coefficient: Option<f32>,
        drag_area: Option<f32>,
        drag_axis_ratios: Option<[f32; 3]>,
        fluid_density: Option<f32>,
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

        // Hybrid drag approach: use damping for extreme cases, force-based drag for normal cases (Phase 2)
        if let (Some(cd), Some(area), Some(ratios), Some(density)) =
            (drag_coefficient, drag_area, drag_axis_ratios, fluid_density) {

            let m = mass.unwrap_or(1.0);
            let g = 9.81;
            let v_ref = 15.0; // Reference velocity for classification (m/s)

            // Calculate drag-to-weight ratio at reference velocity
            let drag_at_vref = 0.5 * density * cd * area * v_ref * v_ref;
            let weight = m * g;
            let drag_weight_ratio = if weight > 1e-6 {
                drag_at_vref / weight
            } else {
                0.0
            };

            // Threshold for extreme drag: if drag > 2x weight at v_ref, use damping
            let use_damping = drag_weight_ratio > 2.0;

            if use_damping {
                // For extreme drag cases (ping pong ball, leaf, feather):
                // Use Rapier's linear damping for numerical stability
                // Approximate: linear_damping ≈ (0.5 * rho * cd * area * v_ref) / mass
                let damping_coefficient = (0.5 * density * cd * area * v_ref) / m;
                let clamped_damping = damping_coefficient.min(1.0); // Cap at 1.0 for safety

                // Update the body's damping
                if let Some(body) = self.rigid_body_set.get_mut(body_handle) {
                    body.set_linear_damping(clamped_damping);
                    eprintln!("INFO: Body '{}' has extreme drag (ratio={:.2}), using damping={:.4} instead of force-based drag",
                        self.body_names.get(&body_handle).unwrap_or(&"unknown".to_string()),
                        drag_weight_ratio, clamped_damping);
                }

                // Still store drag properties but mark as damping-only
                self.body_drag_properties.insert(body_handle, DragProperties {
                    drag_coefficient: cd,
                    drag_area: area,
                    drag_axis_ratios: ratios,
                    fluid_density: density,
                    use_damping_only: true,
                });
            } else {
                // Normal drag case: use force-based orientation-dependent drag
                self.body_drag_properties.insert(body_handle, DragProperties {
                    drag_coefficient: cd,
                    drag_area: area,
                    drag_axis_ratios: ratios,
                    fluid_density: density,
                    use_damping_only: false,
                });
            }
        }
    }

    pub fn step(&mut self, steps: usize, dt: Option<f32>) {
        // Update dt if provided
        if let Some(new_dt) = dt {
            self.integration_parameters.dt = new_dt;
        }

        for _ in 0..steps {
            // Apply orientation-dependent drag forces before physics step (Phase 2)
            self.apply_orientation_dependent_drag();

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

    /// Apply orientation-dependent drag forces (Phase 2)
    fn apply_orientation_dependent_drag(&mut self) {
        // Iterate through all bodies with drag properties
        for (body_handle, drag_props) in &self.body_drag_properties {
            // Skip bodies using damping-only mode (extreme drag cases)
            if drag_props.use_damping_only {
                continue;
            }

            if let Some(body) = self.rigid_body_set.get_mut(*body_handle) {
                // Only apply drag to dynamic bodies
                if !body.is_dynamic() {
                    continue;
                }

                // Get velocity in world space
                let velocity_world = *body.linvel();
                let speed = velocity_world.magnitude();

                // Skip if velocity is negligible
                if speed < 1e-6 {
                    continue;
                }

                // Get body orientation (rotation matrix)
                let rotation = body.rotation();

                // Transform velocity to body-local coordinates
                // This gives us the velocity components along the body's x, y, z axes
                let velocity_local = rotation.inverse_transform_vector(&velocity_world);

                // Calculate drag coefficient modulated by orientation
                // drag_axis_ratios: [x_ratio, y_ratio, z_ratio]
                // Higher ratio = more drag along that axis
                // For streamlined object: [1.0, 0.2, 1.0] means low drag along Y

                let vx = velocity_local.x;
                let vy = velocity_local.y;
                let vz = velocity_local.z;

                // Calculate the fraction of velocity² along each axis
                let speed_squared = speed * speed;
                if speed_squared < 1e-12 {
                    continue; // Skip if essentially stationary
                }

                let vx_frac = (vx * vx) / speed_squared;
                let vy_frac = (vy * vy) / speed_squared;
                let vz_frac = (vz * vz) / speed_squared;

                // Weighted drag based on velocity direction and axis ratios
                // drag_axis_ratios modulate how drag varies with orientation
                // For isotropic drag [1,1,1], drag_factor = 1.0
                // For streamlined [1, 0.2, 1], drag is reduced when moving along Y
                let mut drag_factor = vx_frac * drag_props.drag_axis_ratios[0] +
                                      vy_frac * drag_props.drag_axis_ratios[1] +
                                      vz_frac * drag_props.drag_axis_ratios[2];

                // Safety clamp: ensure drag_factor is always positive and reasonable
                if drag_factor < 0.0 {
                    drag_factor = 0.0;
                }
                let drag_factor_max = 10.0;
                if drag_factor > drag_factor_max {
                    drag_factor = drag_factor_max;
                }

                // Calculate drag force magnitude: F = 0.5 * ρ * Cd * A * v²
                // The drag_factor modulates the effective drag based on orientation
                let drag_magnitude = 0.5 * drag_props.fluid_density *
                    drag_props.drag_coefficient * drag_props.drag_area *
                    speed_squared * drag_factor;

                // Drag direction: strictly opposite to velocity
                // Normalize velocity to get unit vector opposite to motion
                let drag_direction = -velocity_world / speed;

                // Compute drag force vector
                let mut drag_force_vec = drag_direction * drag_magnitude;

                // Clamp drag force to prevent reversing velocity direction
                // Maximum impulse that can be applied without reversing in one timestep
                let mass = body.mass();
                let dt = self.integration_parameters.dt;

                // Momentum: p = m * v
                let momentum = mass * speed;

                // Maximum impulse per timestep (use safety factor of 0.8 to avoid reversal)
                let max_impulse = 0.8 * momentum;

                // Impulse that will be applied: J = F * dt
                let impulse_magnitude = drag_magnitude * dt;

                if impulse_magnitude > max_impulse {
                    // Scale down the force to limit impulse
                    drag_force_vec *= max_impulse / impulse_magnitude;
                }

                // Safety check: drag must always do negative work (oppose motion)
                // If dot(F_drag, v) > 0, drag would accelerate instead of decelerate
                let dot = drag_force_vec.dot(&velocity_world);
                if dot > 0.0 {
                    // This should never happen with correct implementation
                    // If it does, flip the force to ensure it opposes motion
                    eprintln!("WARNING: drag force dot product positive! dot={}, flipping force", dot);
                    drag_force_vec = -drag_force_vec;
                }

                // Debug logging for ping pong ball issue
                if speed > 15.0 {
                    eprintln!("DEBUG: v=[{:.2}, {:.2}, {:.2}] speed={:.2} F_drag=[{:.2}, {:.2}, {:.2}] mag={:.4} dot={:.4}",
                        velocity_world.x, velocity_world.y, velocity_world.z, speed,
                        drag_force_vec.x, drag_force_vec.y, drag_force_vec.z,
                        drag_magnitude, dot);
                }

                // Apply force to body
                body.add_force(drag_force_vec, true);
            }
        }
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
