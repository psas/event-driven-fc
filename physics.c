/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#include "physics.h"
#include "coord.h"

vec3 ECEF_to_rocket(struct rocket_state *rocket_state, vec3 v)
{
	return mat3_vec3_mul(rocket_state->rotpos, v);
}

vec3 rocket_to_ECEF(const struct rocket_state *rocket_state, vec3 v)
{
	return mat3_vec3_mul(mat3_transpose(rocket_state->rotpos), v);
}

vec3 gravity_acceleration(const struct rocket_state *rocket_state)
{
	/* TODO: apply gravity at the approximate center of mass */
	return vec_scale(rocket_state->pos, -EARTH_GRAVITY / vec_abs(rocket_state->pos));
}


static void numerical_integration(double t, double delta_t, vec3 (*f)(double, const struct rocket_state *), struct rocket_state *rocket_state){

    vec3 org_pos = rocket_state->pos;
    vec3 org_vel = rocket_state->vel;
    
    vec3 dm_k= rocket_state->acc;
    vec3 m_k = org_vel;
    
    rocket_state->pos = vec_add(org_pos, vec_scale(m_k, delta_t/2));
    rocket_state->vel = vec_add(org_vel, vec_scale(dm_k, delta_t/2));
    vec3 dn_k= f(t + delta_t/2, rocket_state);
    vec3 n_k = vec_add(org_vel, vec_scale(dm_k, delta_t/2));

    rocket_state->pos = vec_add(org_pos, vec_scale(n_k, delta_t/2));
    rocket_state->vel = vec_add(org_vel, vec_scale(dn_k, delta_t/2));
    vec3 dq_k= f(t + delta_t/2, rocket_state);
    vec3 q_k = vec_add(org_vel, vec_scale(dn_k, delta_t/2));

    rocket_state->pos = vec_add(org_pos, vec_scale(q_k, delta_t));
    rocket_state->vel = vec_add(org_vel, vec_scale(dq_k, delta_t));
    vec3 dp_k= f(t + delta_t, rocket_state);
    vec3 p_k = vec_add(org_vel, vec_scale(dq_k, delta_t));
   
    rocket_state->vel = vec_add(org_vel, vec_scale(vec_add(dm_k, vec_add(vec_scale(dn_k,2), vec_add(vec_scale(dq_k,2), dp_k))),delta_t/6));
    rocket_state->pos = vec_add(org_pos,  vec_scale(vec_add(m_k,  vec_add(vec_scale(n_k,2),  vec_add(vec_scale(q_k,2),  p_k))), delta_t/6));
}

void update_rocket_state_sim(struct rocket_state *rocket_state, double delta_t, vec3 (*f)(double, const struct rocket_state *), double t)
{
	numerical_integration(t, delta_t, f, rocket_state);
	rocket_state->acc = f(t + delta_t, rocket_state);
	rocket_state->rotpos = mat3_mul(rocket_state->rotpos, axis_angle_to_mat3(vec_scale(rocket_state->rotvel, delta_t)));
}

void update_rocket_state(struct rocket_state *rocket_state, double delta_t)
{
	/* FIXME: this should use a better numerical integration technique,
	 * such as Runge-Kutta or leapfrog integration. */
	rocket_state->pos = vec_add(rocket_state->pos, vec_scale(rocket_state->vel, delta_t));
	rocket_state->vel = vec_add(rocket_state->vel, vec_scale(rocket_state->acc, delta_t));
	rocket_state->rotpos = mat3_mul(rocket_state->rotpos, axis_angle_to_mat3(vec_scale(rocket_state->rotvel, delta_t)));
}
