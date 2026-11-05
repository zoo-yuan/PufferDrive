#include "include/drive_fixture.h"
#include "include/test.h"

static int test_classic_action_clipping(void) {
    srand(19);
    Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
    env.action_type = 1;
    env.dynamics_model = DYNAMICS_MODEL_CLASSIC;
    Agent *agent = &env.agents[env.active_agent_indices[0]];
    agent->sim_speed_signed = env.base_max_speed_mps * 0.9f;
    agent->steering_angle = STEERING_LIMIT * 0.9f;
    ((float (*)[2]) env.actions)[0][0] = ACCELERATION_VALUES[6] * 10.0f; // large acceleration
    ((float (*)[2]) env.actions)[0][1] = STEERING_VALUES[8] * 10.0f;     // large steering
    move_dynamics(&env, 0, env.active_agent_indices[0]);
    EXPECT_TRUE(agent->sim_speed_signed <= env.base_max_speed_mps);
    EXPECT_TRUE(agent->steering_angle <= STEERING_LIMIT);
    free_allocated(&env);
    return 0;
}

static int test_jerk_action_clipping(void) {
    srand(23);
    Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
    env.action_type = 1;
    env.dynamics_model = DYNAMICS_MODEL_JERK;
    Agent *agent = &env.agents[env.active_agent_indices[0]];
    agent->sim_speed_signed = env.base_max_speed_mps * 0.9f;
    agent->steering_angle = STEERING_LIMIT * 0.9f;
    ((float (*)[2]) env.actions)[0][0] = ACCELERATION_VALUES[0] * 10.0f; // large braking
    ((float (*)[2]) env.actions)[0][1] = STEERING_VALUES[0] * 10.0f;     // large steering
    move_dynamics(&env, 0, env.active_agent_indices[0]);
    EXPECT_TRUE(agent->accel_long >= ACCEL_LONG_LIMIT[0]);
    EXPECT_TRUE(agent->accel_lat <= ACCEL_LAT_LIMIT[1]);
    EXPECT_TRUE(agent->steering_angle <= STEERING_LIMIT);
    free_allocated(&env);
    return 0;
}

static int test_dynamics_stopped_agent_clears_motion(void) {
    Drive env = {0};
    Agent agent = drive_test_agent(5.0f, 3.0f, 0.5f);
    env.agents = &agent;
    env.dt = 0.1f;
    agent.stopped = 1;
    agent.sim_vx = 4.0f;
    update_agent_speed(&agent);
    agent.steering_angle = 0.3f;

    move_dynamics(&env, 0, 0);

    EXPECT_NEAR(agent.sim_vx, 0.0f, 1e-6f);
    EXPECT_NEAR(agent.sim_speed, 0.0f, 1e-6f);
    EXPECT_NEAR(agent.steering_angle, 0.0f, 1e-6f);
    return 0;
}

static int test_dynamics_removed_agent_invalidated(void) {
    Drive env = {0};
    Agent agent = drive_test_agent(5.0f, 3.0f, 0.5f);
    env.agents = &agent;
    env.dt = 0.1f;
    agent.removed = 1;

    move_dynamics(&env, 0, 0);

    EXPECT_EQ_INT(agent.sim_valid, 0);
    EXPECT_NEAR(agent.sim_x, INVALID_POSITION, 1e-3f);
    return 0;
}

static int test_neutral_actions_zero_out(void) {
    {
        Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
        env.action_type = 0;
        env.dynamics_model = DYNAMICS_MODEL_CLASSIC;
        drive_set_neutral_actions(&env);
        int action_val = ((int *) env.actions)[0];
        int num_steer = sizeof(STEERING_VALUES) / sizeof(STEERING_VALUES[0]);
        int acceleration_index = action_val / num_steer;
        int steering_index = action_val % num_steer;
        EXPECT_NEAR(ACCELERATION_VALUES[acceleration_index], 0.0f, 1e-6f);
        EXPECT_NEAR(STEERING_VALUES[steering_index], 0.0f, 1e-6f);
        free_allocated(&env);
    }
    {
        Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
        env.action_type = 0;
        env.dynamics_model = DYNAMICS_MODEL_JERK;
        drive_set_neutral_actions(&env);
        int action_val = ((int *) env.actions)[0];
        int num_lat = sizeof(JERK_LAT) / sizeof(JERK_LAT[0]);
        int j_long_idx = action_val / num_lat;
        int j_lat_idx = action_val % num_lat;
        EXPECT_NEAR(JERK_LONG[j_long_idx], 0.0f, 1e-6f);
        EXPECT_NEAR(JERK_LAT[j_lat_idx], 0.0f, 1e-6f);
        free_allocated(&env);
    }
    return 0;
}

static void drive_set_agent_motion(Agent *agent, float x, float y, float heading, float speed) {
    agent->sim_x = x;
    agent->sim_y = y;
    agent->sim_heading = heading;
    agent->cos_heading = cosf(heading);
    agent->sin_heading = sinf(heading);
    agent->sim_vx = speed * cosf(heading);
    agent->sim_vy = speed * sinf(heading);
    agent->sim_speed = fabsf(speed);
    agent->sim_speed_signed = speed;
    agent->accel_long = 0.0f;
    agent->accel_lat = 0.0f;
    agent->steering_angle = 0.0f;
    agent->wheelbase = 2.7f;
    copy_pose_to_prev(agent);
}

static int test_control_mode_executes_accel_and_curvature(void) {
    srand(31);
    Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
    env.action_type = ACTION_TYPE_CONTROL;
    env.integration_scheme = INTEGRATION_SCHEME_MODEL_EULER;
    env.skip_actuator_limits = 1;
    env.dynamics_model = DYNAMICS_MODEL_JERK;
    int agent_idx = env.active_agent_indices[0];
    Agent *agent = &env.agents[agent_idx];
    drive_set_agent_motion(agent, 10.0f, 20.0f, 0.3f, 10.0f);

    ((float (*)[2]) env.actions)[0][0] = 1.0f;  // a_long [m/s^2]
    ((float (*)[2]) env.actions)[0][1] = 0.05f; // kappa [1/m]
    move_dynamics(&env, 0, agent_idx);

    float v_expected = 10.0f + 1.0f * env.dt;
    float heading_expected = 0.3f + 0.05f * v_expected * env.dt;
    EXPECT_NEAR(agent->sim_speed_signed, v_expected, 1e-5f);
    EXPECT_NEAR(agent->sim_x, 10.0f + v_expected * cosf(heading_expected) * env.dt, 1e-4f);
    EXPECT_NEAR(agent->sim_y, 20.0f + v_expected * sinf(heading_expected) * env.dt, 1e-4f);
    EXPECT_NEAR(agent->sim_heading, heading_expected, 1e-5f);
    EXPECT_NEAR(agent->accel_long, 1.0f, 1e-6f);
    EXPECT_NEAR(agent->accel_lat, 0.05f * v_expected * v_expected, 1e-4f);
    EXPECT_NEAR(agent->yaw_rate, 0.05f * v_expected, 1e-5f);
    free_allocated(&env);
    return 0;
}

static int test_control_mode_ignores_reward_coefficients(void) {
    srand(37);
    Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
    env.action_type = ACTION_TYPE_CONTROL;
    env.integration_scheme = INTEGRATION_SCHEME_MODEL_EULER;
    env.skip_actuator_limits = 1;
    env.dynamics_model = DYNAMICS_MODEL_JERK;
    int agent_idx = env.active_agent_indices[0];
    Agent *agent = &env.agents[agent_idx];
    drive_set_agent_motion(agent, 0.0f, 0.0f, 0.0f, 10.0f);
    // Reward conditioning must not scale a direct control action.
    agent->reward_coefs[REWARD_COEF_THROTTLE] = 0.1f;
    agent->reward_coefs[REWARD_COEF_STEER] = 0.1f;
    agent->reward_coefs[REWARD_COEF_ACC] = 0.1f;

    ((float (*)[2]) env.actions)[0][0] = 2.0f;
    ((float (*)[2]) env.actions)[0][1] = 0.0f;
    move_dynamics(&env, 0, agent_idx);

    EXPECT_NEAR(agent->accel_long, 2.0f, 1e-6f);
    EXPECT_NEAR(agent->sim_speed_signed, 10.0f + 2.0f * env.dt, 1e-5f);
    free_allocated(&env);
    return 0;
}

static int test_control_mode_applies_actuator_limits(void) {
    srand(41);
    Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
    env.action_type = ACTION_TYPE_CONTROL;
    env.integration_scheme = INTEGRATION_SCHEME_MODEL_EULER;
    env.skip_actuator_limits = 0;
    env.dynamics_model = DYNAMICS_MODEL_JERK;
    int agent_idx = env.active_agent_indices[0];
    Agent *agent = &env.agents[agent_idx];
    drive_set_agent_motion(agent, 0.0f, 0.0f, 0.0f, 10.0f);

    ((float (*)[2]) env.actions)[0][0] = -50.0f; // clipped to ACCEL_LONG_LIMIT[0]
    ((float (*)[2]) env.actions)[0][1] = 5.0f;   // clipped by steering rate
    move_dynamics(&env, 0, agent_idx);

    EXPECT_NEAR(agent->accel_long, ACCEL_LONG_LIMIT[0], 1e-6f);
    EXPECT_NEAR(agent->sim_speed_signed, 10.0f + ACCEL_LONG_LIMIT[0] * env.dt, 1e-5f);
    EXPECT_TRUE(agent->steering_angle > 0.0f);
    EXPECT_TRUE(agent->steering_angle <= STEER_RATE_LIMIT_RADPS * env.dt + 1e-6f);
    free_allocated(&env);
    return 0;
}

static int test_control_mode_euler_integrates_three_steps(void) {
    srand(43);
    Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
    env.action_type = ACTION_TYPE_CONTROL;
    env.integration_scheme = INTEGRATION_SCHEME_MODEL_EULER;
    env.skip_actuator_limits = 1;
    env.dynamics_model = DYNAMICS_MODEL_JERK;
    int agent_idx = env.active_agent_indices[0];
    Agent *agent = &env.agents[agent_idx];
    drive_set_agent_motion(agent, 0.0f, 0.0f, 0.0f, 8.0f);

    ((float (*)[2]) env.actions)[0][0] = 0.5f;
    ((float (*)[2]) env.actions)[0][1] = 0.02f;
    for (int step = 0; step < 3; step++) {
        move_dynamics(&env, 0, agent_idx);
    }

    // Independently computed semi-implicit Euler reference for the same inputs.
    EXPECT_NEAR(agent->sim_speed_signed, 8.15f, 1e-4f);
    EXPECT_NEAR(agent->sim_heading, 0.0486f, 1e-4f);
    EXPECT_NEAR(agent->sim_x, 2.428511f, 1e-4f);
    EXPECT_NEAR(agent->sim_y, 0.078712f, 1e-4f);
    free_allocated(&env);
    return 0;
}

// Braking to a standstill with reset_accel_on_stop must report the driver's jerk, not the zeroed acceleration
static int test_jerk_stop_reset_keeps_driver_jerk(void) {
    srand(29);
    Drive env = drive_test_make_env(drive_carla_map(), SIMULATION_MODE_GIGAFLOW, 1, 0);
    env.action_type = 1;
    env.dynamics_model = DYNAMICS_MODEL_JERK;
    env.reset_accel_on_stop = 1;
    env.dt = 0.3f;
    Agent *agent = &env.agents[env.active_agent_indices[0]];
    agent->reward_coefs[REWARD_COEF_THROTTLE] = 1.0f;
    agent->reward_coefs[REWARD_COEF_STEER] = 1.0f;
    agent->reward_coefs[REWARD_COEF_ACC] = 1.0f;
    agent->reward_coefs[REWARD_COEF_SPEED] = 1.0f;
    float (*actions)[2] = (float (*)[2]) env.actions;
    const float braking_jerk_action = JERK_LONG[1] / -JERK_LONG[0]; // decodes to -4 m/s^3
    const float stop_speed_mps = 0.3f;
    const float stop_accel_mps2 = -3.0f;
    {
        agent->sim_vx = stop_speed_mps * agent->cos_heading;
        agent->sim_vy = stop_speed_mps * agent->sin_heading;
        update_agent_speed(agent);
        agent->accel_long = stop_accel_mps2;
        actions[0][0] = 0.0f;
        actions[0][1] = 0.0f;
        move_dynamics(&env, 0, env.active_agent_indices[0]);
        EXPECT_NEAR(agent->sim_speed, 0.0f, 1e-6f);
        EXPECT_NEAR(agent->accel_long, 0.0f, 1e-6f);
        EXPECT_NEAR(agent->jerk_long, 0.0f, 1e-6f);
    }
    {
        agent->sim_vx = stop_speed_mps * agent->cos_heading;
        agent->sim_vy = stop_speed_mps * agent->sin_heading;
        update_agent_speed(agent);
        agent->accel_long = stop_accel_mps2;
        actions[0][0] = braking_jerk_action;
        actions[0][1] = 0.0f;
        move_dynamics(&env, 0, env.active_agent_indices[0]);
        EXPECT_NEAR(agent->sim_speed, 0.0f, 1e-6f);
        EXPECT_NEAR(agent->accel_long, 0.0f, 1e-6f);
        EXPECT_NEAR(agent->jerk_long, JERK_LONG[1], 1e-3f);
    }
    free_allocated(&env);
    return 0;
}

int main(void) {
    int failures = 0;
    RUN_TEST(test_classic_action_clipping);
    RUN_TEST(test_jerk_action_clipping);
    RUN_TEST(test_dynamics_stopped_agent_clears_motion);
    RUN_TEST(test_dynamics_removed_agent_invalidated);
    RUN_TEST(test_neutral_actions_zero_out);
    RUN_TEST(test_control_mode_executes_accel_and_curvature);
    RUN_TEST(test_control_mode_ignores_reward_coefficients);
    RUN_TEST(test_control_mode_applies_actuator_limits);
    RUN_TEST(test_control_mode_euler_integrates_three_steps);
    RUN_TEST(test_jerk_stop_reset_keeps_driver_jerk);
    return test_summary(failures);
}
