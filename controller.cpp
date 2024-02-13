extern "C"
{
#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
}
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "spring.h"
#include "array.h"
#include "character.h"
#include "database.h"
#include "nnet.h"
#include "lmm.h"
#include "logiccontroller.h"
#include "charactercontroller.h"
#include "contactcontroller.h"
#include "render.h"

#include <initializer_list>
#include <functional>

//--------------------------------------

void update_callback(void* args)
{
    ((std::function<void()>*)args)->operator()();
}

int main(void)
{
    // Init Window
    
    const int screen_width = 2048;
    const int screen_height = 1024;
    
    SetConfigFlags(FLAG_VSYNC_HINT);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(screen_width, screen_height, "raylib [data vs code driven displacement]");
    SetTargetFPS(60);
    
    // Camera

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 0.0f, 10.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float camera_azimuth = 0.0f;
    float camera_altitude = 0.4f;
    float camera_distance = 4.0f;
    
    // Scene Obstacles
    
    array1d<vec3> obstacles_positions(3);
    array1d<vec3> obstacles_scales(3);
    
    obstacles_positions(0) = vec3(5.0f, 0.0f, 6.0f);
    obstacles_positions(1) = vec3(-3.0f, 0.0f, -5.0f);
    obstacles_positions(2) = vec3(-8.0f, 0.0f, 3.0f);
    
    obstacles_scales(0) = vec3(2.0f, 1.0f, 5.0f);
    obstacles_scales(1) = vec3(4.0f, 1.0f, 4.0f);
    obstacles_scales(2) = vec3(2.0f, 1.0f, 2.0f);
    
    // Ground Plane
    
    Shader ground_plane_shader = LoadShader("./resources/checkerboard.vs", "./resources/checkerboard.fs");
    Mesh ground_plane_mesh = GenMeshPlane(20.0f, 20.0f, 10, 10);
    Model ground_plane_model = LoadModelFromMesh(ground_plane_mesh);
    ground_plane_model.materials[0].shader = ground_plane_shader;
    
    // Character
    
    character character_data;
    character_load(character_data, "./resources/character.bin");
    
    Shader character_shader = LoadShader("./resources/character.vs", "./resources/character.fs");
    Mesh character_mesh = make_character_mesh(character_data);
    Model character_model = LoadModelFromMesh(character_mesh);
    character_model.materials[0].shader = character_shader;
    
    // Load Animation Data and build Matching Database
    
    database db;
    database_load(db, "./resources/database.bin");
    
    float feature_weight_foot_position = 0.75f;
    float feature_weight_foot_velocity = 1.0f;
    float feature_weight_hip_velocity = 1.0f;
    float feature_weight_trajectory_positions = 1.0f;
    float feature_weight_trajectory_directions = 1.5f;
    
    database_build_matching_features(
        db,
        feature_weight_foot_position,
        feature_weight_foot_velocity,
        feature_weight_hip_velocity,
        feature_weight_trajectory_positions,
        feature_weight_trajectory_directions);
        
    database_save_matching_features(db, "./resources/features.bin");
   
    // Pose & Inertializer Data
    
    int frame_index = db.range_starts(0);
    float inertialize_blending_halflife = 0.1f;

    array1d<vec3> curr_bone_positions = db.bone_positions(frame_index);
    array1d<vec3> curr_bone_velocities = db.bone_velocities(frame_index);
    array1d<quat> curr_bone_rotations = db.bone_rotations(frame_index);
    array1d<vec3> curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    array1d<bool> curr_bone_contacts = db.contact_states(frame_index);

    array1d<vec3> trns_bone_positions = db.bone_positions(frame_index);
    array1d<vec3> trns_bone_velocities = db.bone_velocities(frame_index);
    array1d<quat> trns_bone_rotations = db.bone_rotations(frame_index);
    array1d<vec3> trns_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    array1d<bool> trns_bone_contacts = db.contact_states(frame_index);

    array1d<vec3> bone_positions = db.bone_positions(frame_index);
    array1d<vec3> bone_velocities = db.bone_velocities(frame_index);
    array1d<quat> bone_rotations = db.bone_rotations(frame_index);
    array1d<vec3> bone_angular_velocities = db.bone_angular_velocities(frame_index);
    
    array1d<vec3> bone_offset_positions(db.nbones());
    array1d<vec3> bone_offset_velocities(db.nbones());
    array1d<quat> bone_offset_rotations(db.nbones());
    array1d<vec3> bone_offset_angular_velocities(db.nbones());
    
    array1d<vec3> global_bone_positions(db.nbones());
    array1d<vec3> global_bone_velocities(db.nbones());
    array1d<quat> global_bone_rotations(db.nbones());
    array1d<vec3> global_bone_angular_velocities(db.nbones());
    array1d<bool> global_bone_computed(db.nbones());
    
    vec3 transition_src_position;
    quat transition_src_rotation;
    vec3 transition_dst_position;
    quat transition_dst_rotation;
    
    inertialize_pose_reset(
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        bone_positions(0),
        bone_rotations(0));
    
    inertialize_pose_update(
        bone_positions,
        bone_velocities,
        bone_rotations,
        bone_angular_velocities,
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        db.bone_positions(frame_index),
        db.bone_velocities(frame_index),
        db.bone_rotations(frame_index),
        db.bone_angular_velocities(frame_index),
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        inertialize_blending_halflife,
        0.0f);
        
    // Trajectory & Gameplay Data
    
    float search_time = 0.1f;
    float search_timer = search_time;
    float force_search_timer = search_time;
    
    vec3 desired_velocity;
    vec3 desired_velocity_change_curr;
    vec3 desired_velocity_change_prev;
    float desired_velocity_change_threshold = 50.0;
    
    quat desired_rotation;
    vec3 desired_rotation_change_curr;
    vec3 desired_rotation_change_prev;
    float desired_rotation_change_threshold = 50.0;
    
    float desired_gait = 0.0f;
    float desired_gait_velocity = 0.0f;
    
    vec3 simulation_position;
    vec3 simulation_velocity;
    vec3 simulation_acceleration;
    quat simulation_rotation;
    vec3 simulation_angular_velocity;
    
    float simulation_velocity_halflife = 0.27f;
    float simulation_rotation_halflife = 0.27f;
    
    // All speeds in m/s
    float simulation_run_fwrd_speed = 4.0f;
    float simulation_run_side_speed = 3.0f;
    float simulation_run_back_speed = 2.5f;
    
    float simulation_walk_fwrd_speed = 1.75f;
    float simulation_walk_side_speed = 1.5f;
    float simulation_walk_back_speed = 1.25f;
    
    array1d<vec3> trajectory_desired_velocities(4);
    array1d<quat> trajectory_desired_rotations(4);
    array1d<vec3> trajectory_positions(4);
    array1d<vec3> trajectory_velocities(4);
    array1d<vec3> trajectory_accelerations(4);
    array1d<quat> trajectory_rotations(4);
    array1d<vec3> trajectory_angular_velocities(4);
    
    // Synchronization
    
    bool synchronization_enabled = false;
    float synchronization_data_factor = 1.0f;
    
    // Adjustment
    
    bool adjustment_enabled = true;
    bool adjustment_by_velocity_enabled = true;
    float adjustment_position_halflife = 0.1f;
    float adjustment_rotation_halflife = 0.2f;
    float adjustment_position_max_ratio = 0.5f;
    float adjustment_rotation_max_ratio = 0.5f;
    
    // Clamping
    
    bool clamping_enabled = true;
    float clamping_max_distance = 0.15f;
    float clamping_max_angle = 0.5f * PIf;
    
    // IK
    
    bool ik_enabled = true;
    float ik_max_length_buffer = 0.015f;
    float ik_foot_height = 0.02f;
    float ik_toe_length = 0.15f;
    float ik_unlock_radius = 0.2f;
    float ik_blending_halflife = 0.1f;
    
    // Contact and Foot Locking data
    
    array1d<int> contact_bones(2);
    contact_bones(0) = Bone_LeftToe;
    contact_bones(1) = Bone_RightToe;
    
    array1d<bool> contact_states(contact_bones.size);
    array1d<bool> contact_locks(contact_bones.size);
    array1d<vec3> contact_positions(contact_bones.size);
    array1d<vec3> contact_velocities(contact_bones.size);
    array1d<vec3> contact_points(contact_bones.size);
    array1d<vec3> contact_targets(contact_bones.size);
    array1d<vec3> contact_offset_positions(contact_bones.size);
    array1d<vec3> contact_offset_velocities(contact_bones.size);
    
    for (int i = 0; i < contact_bones.size; i++)
    {
        vec3 bone_position;
        vec3 bone_velocity;
        quat bone_rotation;
        vec3 bone_angular_velocity;
        
        forward_kinematics_velocity(
            bone_position,
            bone_velocity,
            bone_rotation,
            bone_angular_velocity,
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            db.bone_parents,
            contact_bones(i));
        
        contact_reset(
            contact_states(i),
            contact_locks(i),
            contact_positions(i),  
            contact_velocities(i),
            contact_points(i),
            contact_targets(i),
            contact_offset_positions(i),
            contact_offset_velocities(i),
            bone_position,
            bone_velocity,
            false);
    }
    
    array1d<vec3> adjusted_bone_positions = bone_positions;
    array1d<quat> adjusted_bone_rotations = bone_rotations;
    
    // Learned Motion Matching
    
    bool lmm_enabled = false;
    
    nnet decompressor, stepper, projector;    
    nnet_load(decompressor, "./resources/decompressor.bin");
    nnet_load(stepper, "./resources/stepper.bin");
    nnet_load(projector, "./resources/projector.bin");

    nnet_evaluation decompressor_evaluation, stepper_evaluation, projector_evaluation;
    decompressor_evaluation.resize(decompressor);
    stepper_evaluation.resize(stepper);
    projector_evaluation.resize(projector);

    array1d<float> features_proj = db.features(frame_index);
    array1d<float> features_curr = db.features(frame_index);
    array1d<float> latent_proj(32); latent_proj.zero();
    array1d<float> latent_curr(32); latent_curr.zero();
    
    // Go

    float dt = 1.0f / 60.0f;

    auto update_func = [&]()
    {
      
        // Get gamepad stick states
        vec3 gamepadstick_left = gamepad_get_stick(GAMEPAD_STICK_LEFT);
        vec3 gamepadstick_right = gamepad_get_stick(GAMEPAD_STICK_RIGHT);
        
        // Get if strafe is desired
        bool desired_strafe = desired_strafe_update();
        
        // Get the desired gait (walk / run)
        desired_gait_update(
            desired_gait,
            desired_gait_velocity,
            dt);
        
        // Get the desired simulation speeds based on the gait
        float simulation_fwrd_speed = lerpf(simulation_run_fwrd_speed, simulation_walk_fwrd_speed, desired_gait);
        float simulation_side_speed = lerpf(simulation_run_side_speed, simulation_walk_side_speed, desired_gait);
        float simulation_back_speed = lerpf(simulation_run_back_speed, simulation_walk_back_speed, desired_gait);
        
        // Get the desired velocity
        vec3 desired_velocity_curr = desired_velocity_update(
            gamepadstick_left,
            camera_azimuth,
            simulation_rotation,
            simulation_fwrd_speed,
            simulation_side_speed,
            simulation_back_speed);
            
        // Get the desired rotation/direction
        quat desired_rotation_curr = desired_rotation_update(
            desired_rotation,
            gamepadstick_left,
            gamepadstick_right,
            camera_azimuth,
            desired_strafe,
            desired_velocity_curr);
        
        // Check if we should force a search because input changed quickly
        desired_velocity_change_prev = desired_velocity_change_curr;
        desired_velocity_change_curr =  (desired_velocity_curr - desired_velocity) / dt;
        desired_velocity = desired_velocity_curr;
        
        desired_rotation_change_prev = desired_rotation_change_curr;
        desired_rotation_change_curr = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(desired_rotation_curr, desired_rotation))) / dt;
        desired_rotation =  desired_rotation_curr;
        
        bool force_search = false;

        if (force_search_timer <= 0.0f && (
            (length(desired_velocity_change_prev) >= desired_velocity_change_threshold && 
             length(desired_velocity_change_curr)  < desired_velocity_change_threshold)
        ||  (length(desired_rotation_change_prev) >= desired_rotation_change_threshold && 
             length(desired_rotation_change_curr)  < desired_rotation_change_threshold)))
        {
            force_search = true;
            force_search_timer = search_time;
        }
        else if (force_search_timer > 0)
        {
            force_search_timer -= dt;
        }
        
        // Predict Future Trajectory
        
        trajectory_desired_rotations_predict(
          trajectory_desired_rotations,
          trajectory_desired_velocities,
          desired_rotation,
          camera_azimuth,
          gamepadstick_left,
          gamepadstick_right,
          desired_strafe,
          20.0f * dt);
        
        trajectory_rotations_predict(
            trajectory_rotations,
            trajectory_angular_velocities,
            simulation_rotation,
            simulation_angular_velocity,
            trajectory_desired_rotations,
            simulation_rotation_halflife,
            20.0f * dt);
        
        trajectory_desired_velocities_predict(
          trajectory_desired_velocities,
          trajectory_rotations,
          desired_velocity,
          camera_azimuth,
          gamepadstick_left,
          gamepadstick_right,
          desired_strafe,
          simulation_fwrd_speed,
          simulation_side_speed,
          simulation_back_speed,
          20.0f * dt);
        
        trajectory_positions_predict(
            trajectory_positions,
            trajectory_velocities,
            trajectory_accelerations,
            simulation_position,
            simulation_velocity,
            simulation_acceleration,
            trajectory_desired_velocities,
            simulation_velocity_halflife,
            20.0f * dt,
            obstacles_positions,
            obstacles_scales);
           
        // Make query vector for search.
        // In theory this only needs to be done when a search is 
        // actually required however for visualization purposes it
        // can be nice to do it every frame
        array1d<float> query(db.nfeatures());
                
        // Compute the features of the query vector

        slice1d<float> query_features = lmm_enabled ? slice1d<float>(features_curr) : db.features(frame_index);

        int offset = 0;
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Position
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Position
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Velocity
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Velocity
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Hip Velocity
        query_compute_trajectory_position_feature(query, offset, bone_positions(0), bone_rotations(0), trajectory_positions);
        query_compute_trajectory_direction_feature(query, offset, bone_rotations(0), trajectory_rotations);
        
        assert(offset == db.nfeatures());

        // Check if we reached the end of the current anim
        bool end_of_anim = database_trajectory_index_clamp(db, frame_index, 1) == frame_index;
        
        // Do we need to search?
        if (force_search || search_timer <= 0.0f || end_of_anim)
        {
            if (lmm_enabled)
            {
                // Project query onto nearest feature vector
                
                float best_cost = FLT_MAX;
                bool transition = false;
                
                projector_evaluate(
                    transition,
                    best_cost,
                    features_proj,
                    latent_proj,
                    projector_evaluation,
                    query,
                    db.features_offset,
                    db.features_scale,
                    features_curr,
                    projector);
                
                // If projection is sufficiently different from current
                if (transition)
                {   
                    // Evaluate pose for projected features
                    decompressor_evaluate(
                        trns_bone_positions,
                        trns_bone_velocities,
                        trns_bone_rotations,
                        trns_bone_angular_velocities,
                        trns_bone_contacts,
                        decompressor_evaluation,
                        features_proj,
                        latent_proj,
                        curr_bone_positions(0),
                        curr_bone_rotations(0),
                        decompressor,
                        dt);
                    
                    // Transition inertializer to this pose
                    inertialize_pose_transition(
                        bone_offset_positions,
                        bone_offset_velocities,
                        bone_offset_rotations,
                        bone_offset_angular_velocities,
                        transition_src_position,
                        transition_src_rotation,
                        transition_dst_position,
                        transition_dst_rotation,
                        bone_positions(0),
                        bone_velocities(0),
                        bone_rotations(0),
                        bone_angular_velocities(0),
                        curr_bone_positions,
                        curr_bone_velocities,
                        curr_bone_rotations,
                        curr_bone_angular_velocities,
                        trns_bone_positions,
                        trns_bone_velocities,
                        trns_bone_rotations,
                        trns_bone_angular_velocities);
                    
                    // Update current features and latents
                    features_curr = features_proj;
                    latent_curr = latent_proj;
                }
            }
            else
            {
                // Search
                
                int best_index = end_of_anim ? -1 : frame_index;
                float best_cost = FLT_MAX;
                
                database_search(
                    best_index,
                    best_cost,
                    db,
                    query);
                
                // Transition if better frame found
                
                if (best_index != frame_index)
                {
                    trns_bone_positions = db.bone_positions(best_index);
                    trns_bone_velocities = db.bone_velocities(best_index);
                    trns_bone_rotations = db.bone_rotations(best_index);
                    trns_bone_angular_velocities = db.bone_angular_velocities(best_index);
                    
                    inertialize_pose_transition(
                        bone_offset_positions,
                        bone_offset_velocities,
                        bone_offset_rotations,
                        bone_offset_angular_velocities,
                        transition_src_position,
                        transition_src_rotation,
                        transition_dst_position,
                        transition_dst_rotation,
                        bone_positions(0),
                        bone_velocities(0),
                        bone_rotations(0),
                        bone_angular_velocities(0),
                        curr_bone_positions,
                        curr_bone_velocities,
                        curr_bone_rotations,
                        curr_bone_angular_velocities,
                        trns_bone_positions,
                        trns_bone_velocities,
                        trns_bone_rotations,
                        trns_bone_angular_velocities);
                    
                    frame_index = best_index;
                }
            }

            // Reset search timer
            search_timer = search_time;
        }
        
        // Tick down search timer
        search_timer -= dt;

        if (lmm_enabled)
        {
            // Update features and latents
            stepper_evaluate(
                features_curr,
                latent_curr,
                stepper_evaluation,
                stepper,
                dt);
            
            // Decompress next pose
            decompressor_evaluate(
                curr_bone_positions,
                curr_bone_velocities,
                curr_bone_rotations,
                curr_bone_angular_velocities,
                curr_bone_contacts,
                decompressor_evaluation,
                features_curr,
                latent_curr,
                curr_bone_positions(0),
                curr_bone_rotations(0),
                decompressor,
                dt);
        }
        else
        {
            // Tick frame
            frame_index++; // Assumes dt is fixed to 60fps
            
            // Look-up Next Pose
            curr_bone_positions = db.bone_positions(frame_index);
            curr_bone_velocities = db.bone_velocities(frame_index);
            curr_bone_rotations = db.bone_rotations(frame_index);
            curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
            curr_bone_contacts = db.contact_states(frame_index);
        }
        
        // Update inertializer
        
        inertialize_pose_update(
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            bone_offset_positions,
            bone_offset_velocities,
            bone_offset_rotations,
            bone_offset_angular_velocities,
            curr_bone_positions,
            curr_bone_velocities,
            curr_bone_rotations,
            curr_bone_angular_velocities,
            transition_src_position,
            transition_src_rotation,
            transition_dst_position,
            transition_dst_rotation,
            inertialize_blending_halflife,
            dt);
        
        // Update Simulation
        
        vec3 simulation_position_prev = simulation_position;
        
        simulation_positions_update(
            simulation_position, 
            simulation_velocity, 
            simulation_acceleration,
            desired_velocity,
            simulation_velocity_halflife,
            dt,
            obstacles_positions,
            obstacles_scales);
            
        simulation_rotations_update(
            simulation_rotation, 
            simulation_angular_velocity, 
            desired_rotation,
            simulation_rotation_halflife,
            dt);
        
        // Synchronization 
        
        if (synchronization_enabled)
        {
            vec3 synchronized_position = lerp(
                simulation_position, 
                bone_positions(0),
                synchronization_data_factor);
                
            quat synchronized_rotation = quat_nlerp_shortest(
                simulation_rotation,
                bone_rotations(0), 
                synchronization_data_factor);
          
            synchronized_position = simulation_collide_obstacles(
                simulation_position_prev,
                synchronized_position,
                obstacles_positions,
                obstacles_scales);
            
            simulation_position = synchronized_position;
            simulation_rotation = synchronized_rotation;
            
            inertialize_root_adjust(
                bone_offset_positions(0),
                transition_src_position,
                transition_src_rotation,
                transition_dst_position,
                transition_dst_rotation,
                bone_positions(0),
                bone_rotations(0),
                synchronized_position,
                synchronized_rotation);
        }
        
        // Adjustment 
        
        if (!synchronization_enabled && adjustment_enabled)
        {   
            vec3 adjusted_position = bone_positions(0);
            quat adjusted_rotation = bone_rotations(0);
            
            if (adjustment_by_velocity_enabled)
            {
                adjusted_position = adjust_character_position_by_velocity(
                    bone_positions(0),
                    bone_velocities(0),
                    simulation_position,
                    adjustment_position_max_ratio,
                    adjustment_position_halflife,
                    dt);
                
                adjusted_rotation = adjust_character_rotation_by_velocity(
                    bone_rotations(0),
                    bone_angular_velocities(0),
                    simulation_rotation,
                    adjustment_rotation_max_ratio,
                    adjustment_rotation_halflife,
                    dt);
            }
            else
            {
                adjusted_position = adjust_character_position(
                    bone_positions(0),
                    simulation_position,
                    adjustment_position_halflife,
                    dt);
                
                adjusted_rotation = adjust_character_rotation(
                    bone_rotations(0),
                    simulation_rotation,
                    adjustment_rotation_halflife,
                    dt);
            }
      
            inertialize_root_adjust(
                bone_offset_positions(0),
                transition_src_position,
                transition_src_rotation,
                transition_dst_position,
                transition_dst_rotation,
                bone_positions(0),
                bone_rotations(0),
                adjusted_position,
                adjusted_rotation);
        }
        
        // Clamping
        
        if (!synchronization_enabled && clamping_enabled)
        {
            vec3 adjusted_position = bone_positions(0);
            quat adjusted_rotation = bone_rotations(0);
            
            adjusted_position = clamp_character_position(
                adjusted_position,
                simulation_position,
                clamping_max_distance);
            
            adjusted_rotation = clamp_character_rotation(
                adjusted_rotation,
                simulation_rotation,
                clamping_max_angle);
            
            inertialize_root_adjust(
                bone_offset_positions(0),
                transition_src_position,
                transition_src_rotation,
                transition_dst_position,
                transition_dst_rotation,
                bone_positions(0),
                bone_rotations(0),
                adjusted_position,
                adjusted_rotation);
        }
        
        // Contact fixup with foot locking and IK

        adjusted_bone_positions = bone_positions;
        adjusted_bone_rotations = bone_rotations;

        if (ik_enabled)
        {
            for (int i = 0; i < contact_bones.size; i++)
            {
                // Find all the relevant bone indices
                int toe_bone = contact_bones(i);
                int heel_bone = db.bone_parents(toe_bone);
                int knee_bone = db.bone_parents(heel_bone);
                int hip_bone = db.bone_parents(knee_bone);
                int root_bone = db.bone_parents(hip_bone);
                
                // Compute the world space position for the toe
                global_bone_computed.zero();
                
                forward_kinematics_partial(
                    global_bone_positions,
                    global_bone_rotations,
                    global_bone_computed,
                    bone_positions,
                    bone_rotations,
                    db.bone_parents,
                    toe_bone);
                
                // Update the contact state
                contact_update(
                    contact_states(i),
                    contact_locks(i),
                    contact_positions(i),  
                    contact_velocities(i),
                    contact_points(i),
                    contact_targets(i),
                    contact_offset_positions(i),
                    contact_offset_velocities(i),
                    global_bone_positions(toe_bone),
                    curr_bone_contacts(i),
                    ik_unlock_radius,
                    ik_foot_height,
                    ik_blending_halflife,
                    dt);
                
                // Ensure contact position never goes through floor
                vec3 contact_position_clamp = contact_positions(i);
                contact_position_clamp.y = maxf(contact_position_clamp.y, ik_foot_height);
                
                // Re-compute toe, heel, knee, hip, and root bone positions
                for (int bone : {heel_bone, knee_bone, hip_bone, root_bone})
                {
                    forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        bone_positions,
                        bone_rotations,
                        db.bone_parents,
                        bone);
                }
                
                // Perform simple two-joint IK to place heel
                ik_two_bone(
                    adjusted_bone_rotations(hip_bone),
                    adjusted_bone_rotations(knee_bone),
                    global_bone_positions(hip_bone),
                    global_bone_positions(knee_bone),
                    global_bone_positions(heel_bone),
                    contact_position_clamp + (global_bone_positions(heel_bone) - global_bone_positions(toe_bone)),
                    quat_mul_vec3(global_bone_rotations(knee_bone), vec3(0.0f, 1.0f, 0.0f)),
                    global_bone_rotations(hip_bone),
                    global_bone_rotations(knee_bone),
                    global_bone_rotations(root_bone),
                    ik_max_length_buffer);
                
                // Re-compute toe, heel, and knee positions 
                global_bone_computed.zero();
                
                for (int bone : {toe_bone, heel_bone, knee_bone})
                {
                    forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        adjusted_bone_positions,
                        adjusted_bone_rotations,
                        db.bone_parents,
                        bone);
                }
                
                // Rotate heel so toe is facing toward contact point
                ik_look_at(
                    adjusted_bone_rotations(heel_bone),
                    global_bone_rotations(knee_bone),
                    global_bone_rotations(heel_bone),
                    global_bone_positions(heel_bone),
                    global_bone_positions(toe_bone),
                    contact_position_clamp);
                
                // Re-compute toe and heel positions
                global_bone_computed.zero();
                
                for (int bone : {toe_bone, heel_bone})
                {
                    forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        adjusted_bone_positions,
                        adjusted_bone_rotations,
                        db.bone_parents,
                        bone);
                }
                
                // Rotate toe bone so that the end of the toe 
                // does not intersect with the ground
                vec3 toe_end_curr = quat_mul_vec3(
                    global_bone_rotations(toe_bone), vec3(ik_toe_length, 0.0f, 0.0f)) + 
                    global_bone_positions(toe_bone);
                    
                vec3 toe_end_targ = toe_end_curr;
                toe_end_targ.y = maxf(toe_end_targ.y, ik_foot_height);
                
                ik_look_at(
                    adjusted_bone_rotations(toe_bone),
                    global_bone_rotations(heel_bone),
                    global_bone_rotations(toe_bone),
                    global_bone_positions(toe_bone),
                    toe_end_curr,
                    toe_end_targ);
            }
        }
        
        // Full pass of forward kinematics to compute 
        // all bone positions and rotations in the world
        // space ready for rendering
        
        forward_kinematics_full(
            global_bone_positions,
            global_bone_rotations,
            adjusted_bone_positions,
            adjusted_bone_rotations,
            db.bone_parents);
        
        // Update camera
        
        orbit_camera_update(
            camera, 
            camera_azimuth,
            camera_altitude,
            camera_distance,
            bone_positions(0) + vec3(0, 1, 0),
            // simulation_position + vec3(0, 1, 0),
            gamepadstick_right,
            desired_strafe,
            dt);

        // Render
        
        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        BeginMode3D(camera);
        
        // Draw Simulation Object
        
        DrawCylinderWires(to_Vector3(simulation_position), 0.6f, 0.6f, 0.001f, 17, ORANGE);
        DrawSphereWires(to_Vector3(simulation_position), 0.05f, 4, 10, ORANGE);
        DrawLine3D(to_Vector3(simulation_position), to_Vector3(
            simulation_position + 0.6f * quat_mul_vec3(simulation_rotation, vec3(0.0f, 0.0f, 1.0f))), ORANGE);
        
        // Draw Clamping Radius/Angles
        
        if (clamping_enabled)
        {
            DrawCylinderWires(
                to_Vector3(simulation_position), 
                clamping_max_distance, 
                clamping_max_distance, 
                0.001f, 17, SKYBLUE);
            
            quat rotation_clamp_0 = quat_mul(quat_from_angle_axis(+clamping_max_angle, vec3(0.0f, 1.0f, 0.0f)), simulation_rotation);
            quat rotation_clamp_1 = quat_mul(quat_from_angle_axis(-clamping_max_angle, vec3(0.0f, 1.0f, 0.0f)), simulation_rotation);
            
            vec3 rotation_clamp_0_dir = simulation_position + 0.6f * quat_mul_vec3(rotation_clamp_0, vec3(0.0f, 0.0f, 1.0f));
            vec3 rotation_clamp_1_dir = simulation_position + 0.6f * quat_mul_vec3(rotation_clamp_1, vec3(0.0f, 0.0f, 1.0f));

            DrawLine3D(to_Vector3(simulation_position), to_Vector3(rotation_clamp_0_dir), SKYBLUE);
            DrawLine3D(to_Vector3(simulation_position), to_Vector3(rotation_clamp_1_dir), SKYBLUE);
        }
        
        // Draw IK foot lock positions
        
        if (ik_enabled)
        {
            for (int i = 0; i <  contact_positions.size; i++)
            {
                if (contact_locks(i))
                {
                    DrawSphereWires(to_Vector3(contact_positions(i)), 0.05f, 4, 10, PINK);
                }
            }
        }
        
        draw_trajectory(
            trajectory_positions,
            trajectory_rotations,
            ORANGE);
        
        draw_obstacles(
            obstacles_positions,
            obstacles_scales);
        
        deform_character_mesh(
            character_mesh, 
            character_data, 
            global_bone_positions, 
            global_bone_rotations,
            db.bone_parents);
        
        DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, RAYWHITE);
        
        // Draw matched features
        
        array1d<float> current_features = lmm_enabled ? slice1d<float>(features_curr) : db.features(frame_index);
        denormalize_features(current_features, db.features_offset, db.features_scale);        
        draw_features(current_features, bone_positions(0), bone_rotations(0), MAROON);
        
        // Draw Simuation Bone
        
        DrawSphereWires(to_Vector3(bone_positions(0)), 0.05f, 4, 10, MAROON);
        DrawLine3D(to_Vector3(bone_positions(0)), to_Vector3(
            bone_positions(0) + 0.6f * quat_mul_vec3(bone_rotations(0), vec3(0.0f, 0.0f, 1.0f))), MAROON);
        
        // Draw Ground Plane
        
        DrawModel(ground_plane_model, (Vector3){0.0f, -0.01f, 0.0f}, 1.0f, WHITE);
        DrawGrid(20, 1.0f);
        draw_axis(vec3(), quat());
        
        EndMode3D();

        // UI
        
        //---------
        
        float ui_sim_hei = 20;
        
        GuiGroupBox((Rectangle){ 970, ui_sim_hei, 290, 250 }, "simulation object");

        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 10, 120, 20 }, 
            "velocity halflife", 
            TextFormat("%5.3f", simulation_velocity_halflife), 
            &simulation_velocity_halflife, 0.0f, 0.5f);
            
        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 40, 120, 20 }, 
            "rotation halflife", 
            TextFormat("%5.3f", simulation_rotation_halflife), 
            &simulation_rotation_halflife, 0.0f, 0.5f);
            
        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 70, 120, 20 }, 
            "run forward speed", 
            TextFormat("%5.3f", simulation_run_fwrd_speed), 
            &simulation_run_fwrd_speed, 0.0f, 10.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 100, 120, 20 }, 
            "run sideways speed", 
            TextFormat("%5.3f", simulation_run_side_speed), 
            &simulation_run_side_speed, 0.0f, 10.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 130, 120, 20 }, 
            "run backwards speed", 
            TextFormat("%5.3f", simulation_run_back_speed), 
            &simulation_run_back_speed, 0.0f, 10.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 160, 120, 20 }, 
            "walk forward speed", 
            TextFormat("%5.3f", simulation_walk_fwrd_speed), 
            &simulation_walk_fwrd_speed, 0.0f, 5.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 190, 120, 20 }, 
            "walk sideways speed", 
            TextFormat("%5.3f", simulation_walk_side_speed), 
            &simulation_walk_side_speed, 0.0f, 5.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100, ui_sim_hei + 220, 120, 20 }, 
            "walk backwards speed", 
            TextFormat("%5.3f", simulation_walk_back_speed), 
            &simulation_walk_back_speed, 0.0f, 5.0f);
        
        //---------
        
        float ui_inert_hei = 280;
        
        GuiGroupBox((Rectangle){ 970, ui_inert_hei, 290, 40 }, "inertiaization blending");
        
        GuiSliderBar(
            (Rectangle){ 1100, ui_inert_hei + 10, 120, 20 }, 
            "halflife", 
            TextFormat("%5.3f", inertialize_blending_halflife), 
            &inertialize_blending_halflife, 0.0f, 0.3f);
        
        //---------
        
        float ui_lmm_hei = 330;
        
        GuiGroupBox((Rectangle){ 970, ui_lmm_hei, 290, 40 }, "learned motion matching");
        
        GuiCheckBox(
            (Rectangle){ 1000, ui_lmm_hei + 10, 20, 20 }, 
            "enabled",
            &lmm_enabled);
        
        //---------
        
        float ui_ctrl_hei = 380;
        
        GuiGroupBox((Rectangle){ 1010, ui_ctrl_hei, 250, 140 }, "controls");
        
        GuiLabel((Rectangle){ 1030, ui_ctrl_hei +  10, 200, 20 }, "Left Trigger - Strafe");
        GuiLabel((Rectangle){ 1030, ui_ctrl_hei +  30, 200, 20 }, "Left Stick - Move");
        GuiLabel((Rectangle){ 1030, ui_ctrl_hei +  50, 200, 20 }, "Right Stick - Camera / Facing (Stafe)");
        GuiLabel((Rectangle){ 1030, ui_ctrl_hei +  70, 200, 20 }, "Left Shoulder - Zoom In");
        GuiLabel((Rectangle){ 1030, ui_ctrl_hei +  90, 200, 20 }, "Right Shoulder - Zoom Out");
        GuiLabel((Rectangle){ 1030, ui_ctrl_hei + 110, 200, 20 }, "A Button - Walk");
        

        
        //---------
        
        GuiGroupBox((Rectangle){ 20, 20, 290, 190 }, "feature weights");
        
        GuiSliderBar(
            (Rectangle){ 150, 30, 120, 20 }, 
            "foot position", 
            TextFormat("%5.3f", feature_weight_foot_position), 
            &feature_weight_foot_position, 0.001f, 3.0f);
            
        GuiSliderBar(
            (Rectangle){ 150, 60, 120, 20 }, 
            "foot velocity", 
            TextFormat("%5.3f", feature_weight_foot_velocity), 
            &feature_weight_foot_velocity, 0.001f, 3.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, 90, 120, 20 }, 
            "hip velocity", 
            TextFormat("%5.3f", feature_weight_hip_velocity), 
            &feature_weight_hip_velocity, 0.001f, 3.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, 120, 120, 20 }, 
            "trajectory positions", 
            TextFormat("%5.3f", feature_weight_trajectory_positions), 
            &feature_weight_trajectory_positions, 0.001f, 3.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, 150, 120, 20 }, 
            "trajectory directions", 
            TextFormat("%5.3f", feature_weight_trajectory_directions), 
            &feature_weight_trajectory_directions, 0.001f, 3.0f);
            
        if (GuiButton((Rectangle){ 150, 180, 120, 20 }, "rebuild database"))
        {
            database_build_matching_features(
                db,
                feature_weight_foot_position,
                feature_weight_foot_velocity,
                feature_weight_hip_velocity,
                feature_weight_trajectory_positions,
                feature_weight_trajectory_directions);
        }
        
        //---------
        
        float ui_sync_hei = 220;
        
        GuiGroupBox((Rectangle){ 20, ui_sync_hei, 290, 70 }, "synchronization");

        GuiCheckBox(
            (Rectangle){ 50, ui_sync_hei + 10, 20, 20 }, 
            "enabled",
            &synchronization_enabled);

        GuiSliderBar(
            (Rectangle){ 150, ui_sync_hei + 40, 120, 20 }, 
            "data-driven amount", 
            TextFormat("%5.3f", synchronization_data_factor), 
            &synchronization_data_factor, 0.0f, 1.0f);

        //---------
        
        float ui_adj_hei = 300;
        
        GuiGroupBox((Rectangle){ 20, ui_adj_hei, 290, 130 }, "adjustment");
        
        GuiCheckBox(
            (Rectangle){ 50, ui_adj_hei + 10, 20, 20 }, 
            "enabled",
            &adjustment_enabled);    
        
        GuiCheckBox(
            (Rectangle){ 50, ui_adj_hei + 40, 20, 20 }, 
            "clamp to max velocity",
            &adjustment_by_velocity_enabled);    
        
        GuiSliderBar(
            (Rectangle){ 150, ui_adj_hei + 70, 120, 20 }, 
            "position halflife", 
            TextFormat("%5.3f", adjustment_position_halflife), 
            &adjustment_position_halflife, 0.0f, 0.5f);
        
        GuiSliderBar(
            (Rectangle){ 150, ui_adj_hei + 100, 120, 20 }, 
            "rotation halflife", 
            TextFormat("%5.3f", adjustment_rotation_halflife), 
            &adjustment_rotation_halflife, 0.0f, 0.5f);
        
        //---------
        
        float ui_clamp_hei = 440;
        
        GuiGroupBox((Rectangle){ 20, ui_clamp_hei, 290, 100 }, "clamping");
        
        GuiCheckBox(
            (Rectangle){ 50, ui_clamp_hei + 10, 20, 20 }, 
            "enabled",
            &clamping_enabled);      
        
        GuiSliderBar(
            (Rectangle){ 150, ui_clamp_hei + 40, 120, 20 }, 
            "distance", 
            TextFormat("%5.3f", clamping_max_distance), 
            &clamping_max_distance, 0.0f, 0.5f);
        
        GuiSliderBar(
            (Rectangle){ 150, ui_clamp_hei + 70, 120, 20 }, 
            "angle", 
            TextFormat("%5.3f", clamping_max_angle), 
            &clamping_max_angle, 0.0f, PIf);
        
        //---------
        
        float ui_ik_hei = 550;
        
        GuiGroupBox((Rectangle){ 20, ui_ik_hei, 290, 100 }, "inverse kinematics");
        
        bool ik_enabled_prev = ik_enabled;
        
        GuiCheckBox(
            (Rectangle){ 50, ui_ik_hei + 10, 20, 20 }, 
            "enabled",
            &ik_enabled);      
        
        // Foot locking needs resetting when IK is toggled
        if (ik_enabled && !ik_enabled_prev)
        {
            for (int i = 0; i < contact_bones.size; i++)
            {
                vec3 bone_position;
                vec3 bone_velocity;
                quat bone_rotation;
                vec3 bone_angular_velocity;
                
                forward_kinematics_velocity(
                    bone_position,
                    bone_velocity,
                    bone_rotation,
                    bone_angular_velocity,
                    bone_positions,
                    bone_velocities,
                    bone_rotations,
                    bone_angular_velocities,
                    db.bone_parents,
                    contact_bones(i));
                
                contact_reset(
                    contact_states(i),
                    contact_locks(i),
                    contact_positions(i),  
                    contact_velocities(i),
                    contact_points(i),
                    contact_targets(i),
                    contact_offset_positions(i),
                    contact_offset_velocities(i),
                    bone_position,
                    bone_velocity,
                    false);
            }
        }
        
        GuiSliderBar(
            (Rectangle){ 150, ui_ik_hei + 40, 120, 20 }, 
            "blending halflife", 
            TextFormat("%5.3f", ik_blending_halflife), 
            &ik_blending_halflife, 0.0f, 1.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, ui_ik_hei + 70, 120, 20 }, 
            "unlock radius", 
            TextFormat("%5.3f", ik_unlock_radius), 
            &ik_unlock_radius, 0.0f, 0.5f);
        
        //---------

        EndDrawing();

    };

#if defined(PLATFORM_WEB)
    std::function<void()> u{update_func};
    emscripten_set_main_loop_arg(update_callback, &u, 0, 1);
#else
    while (!WindowShouldClose())
    {
        update_func();
    }
#endif

    // Unload stuff and finish
    UnloadModel(character_model);
    UnloadModel(ground_plane_model);
    UnloadShader(character_shader);
    UnloadShader(ground_plane_shader);

    CloseWindow();

    return 0;
}
