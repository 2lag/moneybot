#include "movement.hpp"
#include "interface.hpp"
#include "settings.hpp"
#include "context.hpp"

#include <algorithm>
#include "base_cheat.hpp"
#include "input_system.hpp"
#include "renderer.hpp"

NAMESPACE_REGION( features )

float get_ideal_strafe_step( float speed ) {
	static auto* sv_airaccelerate = g_csgo.m_cvar( )->FindVar( xors( "sv_airaccelerate" ) );
	float airaccel = std::min< float >( sv_airaccelerate->get_float( ), 30.f );

	return RAD2DEG( std::asinf( 30.f / speed ) ) * 0.5f;
}

void rotate_movement( user_cmd_t* cmd, float rotation ) {
	rotation = rotation * M_PIRAD;

	float cos_rot = cos( rotation );
	float sin_rot = sin( rotation );

	float new_forwardmove = ( cos_rot * cmd->m_forwardmove ) - ( sin_rot * cmd->m_sidemove );
	float new_sidemove = ( sin_rot * cmd->m_forwardmove ) + ( cos_rot * cmd->m_sidemove );

	cmd->m_forwardmove = new_forwardmove;
	cmd->m_sidemove = new_sidemove;
}

void c_movement::bhop( ) {
	if ( !g_settings.misc.bunny_hop )
		return;

	if ( g_ctx.m_local->m_nMoveType( ) == MOVETYPE_LADDER || 
		g_ctx.m_local->m_nMoveType( ) == MOVETYPE_NOCLIP )
		return;

	//jump like you nohat
	if ( m_ucmd->m_buttons & IN_JUMP && !( g_ctx.m_local->m_fFlags( ) & FL_ONGROUND ) ) {
		m_ucmd->m_buttons &= ~IN_JUMP;
	}
}

void c_movement::auto_strafer( ) {
	if( !g_settings.misc.auto_strafe )
		return;

	vec3_t velocity = g_ctx.m_local->m_vecVelocity( );
	float speed = velocity.length2d( );

	bool use_original = !g_settings.rage.enabled && !g_settings.rage.anti_aim;

	auto cmd = use_original ? m_ucmd : g_ctx.get_last_cmd( );

	auto on_ground = g_ctx.m_local->m_fFlags( ) & FL_ONGROUND;
	if( cmd && ( m_ucmd->m_buttons & IN_JUMP ) && ( speed > 1.0f || g_settings.misc.air_duck( ) ) && !on_ground ) {
		if( !cmd->m_forwardmove && !cmd->m_sidemove ) {
			if( !cmd->m_mousedx ) {
				float ideal_rotation = std::min( RAD2DEG( std::asinf( 30.f / std::max( speed, FLT_EPSILON ) ) ) * 0.5f, 45.f );

				float sign = cmd->m_cmd_nr % 2 ? 1.f : -1.f;


				cmd->m_sidemove = 0.f;
				cmd->m_forwardmove = 450.f;

				rotate_movement( cmd, ( ideal_rotation - 90.f ) * sign );
			}
			else {
				cmd->m_sidemove = m_ucmd->m_mousedx < 0.f ? -450.f : 450.f;
			}
		}
		else if( g_settings.misc.auto_strafe == 2 ) {
			if( !cmd->m_mousedx ) {
				float ideal_rotation = std::min( RAD2DEG( std::asinf( 30.f / std::max( speed, FLT_EPSILON ) ) ) * 0.5f, 45.f );

				float move_yaw = math::vector_angles( vec3_t( ), vec3_t( cmd->m_forwardmove, cmd->m_sidemove, 0.f ) ).y;
				float vel_yaw = math::vector_angles( vec3_t( ), velocity ).y;

				float velocity_delta = std::remainderf( g_csgo.m_engine( )->GetViewAngles( ).y - vel_yaw, 360.f );

				float move_delta = std::remainderf( move_yaw - velocity_delta, 360.f );
				float ideal_yaw = math::approach_angle( move_yaw, velocity_delta, ideal_rotation );

				float delta_yaw = std::remainderf( move_yaw - ideal_yaw, 360.f );

				if( std::abs( delta_yaw ) > ideal_rotation )
					ideal_rotation = 0.f;
				else if( ( cmd->m_cmd_nr % 2 ) )
					ideal_rotation *= -1;

				cmd->m_sidemove = ( cmd->m_cmd_nr % 2 ) ? 450.f : -450.f;
				cmd->m_forwardmove = 0;

				rotate_movement( cmd, std::remainderf( ideal_yaw + ideal_rotation, 360.f ) );
			}
			else {
				float move_yaw = math::vector_angles( vec3_t( ), vec3_t( cmd->m_forwardmove, cmd->m_sidemove, 0.f ) ).y;

				float rotation = cmd->m_mousedx < 0.f ? -90.f : 90.f;

				cmd->m_forwardmove = 450.f;
				cmd->m_sidemove = 0.f;

				rotate_movement( cmd, move_yaw + rotation );
			}
		}
	}
}

void c_movement::edge_jump( ) {
	if ( !g_settings.misc.edge_jump ) 
		return;

	if ( !g_input.is_key_pressed( ( VirtualKeys_t )g_settings.misc.edge_jump_key( ) ) )
		return;

	//needs key check here so its not always on
	//what??
	bool pre_onground = g_ctx.m_local->m_fFlags( ) & FL_ONGROUND;
	bool post_onground = g_cheat.m_prediction.get_predicted_flags( ) & FL_ONGROUND;

	if ( pre_onground && !post_onground ) {
		m_ucmd->m_buttons |= IN_JUMP;
	}
}

std::vector<float> edge_dist;
bool should_bug = false;
float old_vel_z = 0.f;

bool edge_bug_detect( const vec3_t& old_vel, const vec3_t& pred_vel, const vec3_t& post_vel ) {
  static cvar_t* sv_gravity = g_csgo.m_cvar( )->FindVar( xors( "sv_gravity" ) );
  float gravity = sv_gravity->get_float( );

  if ( old_vel.z < -6.f && pred_vel.z > old_vel.z && pred_vel.z < -6.f && old_vel.length2d( ) <= pred_vel.length2d( ) ) {
    const float gravity_vel_const = roundf( -gravity * TICK_INTERVAL( ) + pred_vel.z );
    if ( gravity_vel_const == roundf( post_vel.z ) )
      return true;
  }
  
  float ebz_vel = gravity * 0.5f * TICK_INTERVAL( );
  if ( pred_vel.length2d( ) <= post_vel.length2d( ) && -ebz_vel > pred_vel.z && round( post_vel.z ) == round( -ebz_vel ) )
      return true;

  return false;
}

// if this func succeeds aka the traces hit then FL_ONGROUND will be set on that tick
void trace_player_collision( const vec3_t& start, const vec3_t& end,
                             const vec3_t& min_src, const vec3_t& max_src, CGameTrace& pm ) {
  Ray_t ray;
  vec3_t mins, maxs;
  vec3_t endpos = pm.endpos;
  float fraction = pm.fraction;
  unsigned int mask = MASK_PLAYERSOLID;

  mins = min_src;
  maxs = { std::min( 0.f, max_src.x ), std::min( 0.f, max_src.y ), max_src.z };
  
  auto check_ray_quads = [ & ]( vec3_t mins, vec3_t maxs ) {
    ray.Init( start, end, mins, maxs );
    g_csgo.m_trace( )->TraceRay( ray, mask, 0, &pm );
    
    if ( pm.DidHit( ) && pm.plane.normal.z >= 0.7f )
      return true;
    else
      return false;
  };

  if ( check_ray_quads( mins, maxs ) ) return;

  mins = { std::max( 0.f, min_src.x ), std::max( 0.f, min_src.y ), min_src.z };
  maxs = max_src;
  if ( check_ray_quads( mins, maxs ) ) return;

  mins = { min_src.x, std::max( 0.f, min_src.y ), min_src.z };
  maxs = { std::min( 0.f, max_src.x ), max_src.y, max_src.z };
  if ( check_ray_quads( mins, maxs ) ) return;
  
  mins = { std::max( 0.f, min_src.x ), min_src.y, min_src.z };
  maxs = { max_src.x, std::min( 0.f, max_src.y ), max_src.z };
  if ( check_ray_quads( mins, maxs ) ) return;

  pm.fraction = fraction;
  pm.endpos = endpos;
}

void auto_strafe_to_edge( user_cmd_t* cmd, float wish_yaw, vec3_t& vel ) {
  float speed = vel.length2d( );
  if ( speed < 1.f ) return;

  float ideal = ( speed > 15.f ) ? RAD2DEG( atan2( 15.f, speed ) ) : 90.f;
  float delta = RAD2DEG( atan2( cmd->m_sidemove, cmd->m_forwardmove ) );
  float difference = fmod( wish_yaw - delta + 360.f, 360.f );

  if ( difference > 180.f )
    difference -= 360.f;

  float move_angle = ideal * difference / 90.f;

  cmd->m_sidemove = std::clamp( move_angle, -450.f, 450.f );
}

void c_movement::edge_bug( ) {
  if( !g_settings.misc.edge_bug )
    return;

  if ( !g_input.is_key_pressed( ( VirtualKeys_t )g_settings.misc.edge_bug_key( ) ) )
    return;

  if ( !g_ctx.m_local->is_alive( ) )
    return;

  if ( g_ctx.m_local->m_fFlags( ) & FL_ONGROUND )
    return;

  if ( g_ctx.m_local->m_nMoveType( ) == MOVETYPE_LADDER )
    return;
  
  vec3_t view_ang = g_ctx.m_local->m_angEyeAngles( );
  vec3_t pred_vel = g_ctx.m_local->m_vecVelocity( );
  vec3_t pred_pos = g_ctx.m_local->m_vecOrigin( );
  vec3_t post_pred_vel, post_pred_pos;
  float closest_distance = FLT_MAX;
  vec3_t old_vel = pred_vel;
  vec3_t best_pos, best_vel;
  view_ang.y += 90.f;
  should_bug = false;
  edge_dist.clear( );
  bug_path.clear( );

  for ( int i = 0; i < 25; ++i ) {
    pred_pos = g_cheat.m_prediction.aimware_extrapolate(
      g_ctx.m_local, pred_pos, pred_vel
    );

    CGameTrace trace;
    g_cheat.m_prediction.trace_player_bbox(
      g_ctx.m_local, pred_pos, pred_pos + pred_vel * TICK_INTERVAL( ), &trace
    );

    if ( trace.fraction != 1.0f )
      break;
    
    post_pred_vel = pred_vel;
    post_pred_pos = g_cheat.m_prediction.aimware_extrapolate(
      g_ctx.m_local, pred_pos, post_pred_vel
    );
    
    CGameTrace t_ground;
    vec3_t mins = g_ctx.m_local->m_vecMins( );
    vec3_t maxs = g_ctx.m_local->m_vecMaxs( );
    trace_player_collision(
      pred_pos, pred_pos - vec3_t( 0.f, 0.f, 32.f ),
      mins, maxs, t_ground
    );

    float dist_to_grnd = pred_pos.z - t_ground.endpos.z;
    edge_dist.push_back(
      dist_to_grnd
    );

    bug_path.push_back( {
      pred_pos, post_pred_pos
    });

    if ( edge_bug_detect( old_vel, pred_vel, post_pred_vel ) ) {
      hit_path = bug_path;
      best_pos = post_pred_pos;
      best_vel = post_pred_vel;
      should_bug = true;
      break;
    } else {
      float distance = ( post_pred_pos - pred_pos ).length( );
      if ( distance < closest_distance ) {
        closest_distance = distance;
        best_pos = post_pred_pos;
        best_vel = post_pred_vel;
        // break here ?
      }
    }
  }
  
  if ( should_bug ) {
    vec3_t dir = best_pos - g_ctx.m_local->m_vecOrigin( );
    float ideal_speed = best_vel.length2d( );
    dir.normalize_vector( );

    float speed_delta = ideal_speed - g_ctx.m_local->m_vecVelocity( ).length2d( );

    auto_strafe_to_edge( m_ucmd,
      RAD2DEG( atan2( dir.y, dir.x ) ),
      g_ctx.m_local->m_vecVelocity( )
    );

    for ( const auto& distance : edge_dist ) {
      if ( distance < 4.f ) {
        m_ucmd->m_buttons |= IN_DUCK;
        break;
      }
    }

    rotate_movement( m_ucmd, RAD2DEG( atan2( dir.x, dir.y ) ) );
    return;
  }

  old_vel_z = g_ctx.m_local->m_vecVelocity( ).z;
}

void c_movement::air_duck( ) {
	if( !g_settings.misc.air_duck )
		return;

	if( !( g_ctx.m_local->m_fFlags( ) & FL_ONGROUND ) )
		m_ucmd->m_buttons |= IN_DUCK;
}

void c_movement::jump_stats( ) {
	if( !g_settings.misc.show_jump_stats ) return;

	static auto sv_airaccelerate = g_csgo.m_cvar( )->FindVar( xors( "sv_airaccelerate" ) );
	static bool was_onground = g_ctx.m_local->m_fFlags( ) & FL_ONGROUND;
	static vec3_t last_origin{ };
	static float ground_vel{ };
	static float last_jump_max_speed{ };
	static float last_height{ };
	static float last_dist{ };

	const float lj_threshold = sv_airaccelerate->get_float( ) < 15.f ? 190.f : 240.f;

	bool on_ground = g_ctx.m_local->m_fFlags( ) & FL_ONGROUND;
	bool ducking = g_ctx.m_local->m_fFlags( ) & FL_DUCKING;

	char jump_string[ 250 ] = { };

	if( on_ground ) {
		int vertical = 0;

		if( !was_onground ) {
			vec3_t cur_origin = g_ctx.m_local->m_vecOrigin( );
			last_dist = cur_origin.dist_to( last_origin );

			if( std::abs( cur_origin.z - last_origin.z ) >= ( ducking ? 10.f : 5.f ) ) {
				vertical = cur_origin.z > last_origin.z ? 1 : -1;
			}

			if( ground_vel > 200.f && last_jump_max_speed > 260.f && std::abs( last_height ) > 20.f ) {
				if( vertical ) {
					strenc::w_sprintf_s( jump_string, 250, xors( "[\3JUMP STAT\1] pre: %0.2f | max vel: %0.2f | height: %0.2f | duck: %d | \2%s\n" ),
						ground_vel, last_jump_max_speed, last_height, ducking, vertical == 1 ? xors( "vertical" ) : xors( "dropjump" ) );
				}
				else {
					bool is_lj = last_dist > lj_threshold;
					strenc::w_sprintf_s( jump_string, 250, xors( "[\3JUMP STAT\1]: pre: %0.2f | max vel: %0.2f | height: %0.2f | duck: %d | dist: %c%0.2f\n" ),
						ground_vel, last_jump_max_speed, last_height, ducking, is_lj ? 4 : 1, last_dist );
				}

				g_csgo.m_clientmode( )->m_pChatElement->ChatPrintf( 0, 0, jump_string );
			}
		}
		last_origin = g_ctx.m_local->m_vecOrigin( );
		last_jump_max_speed = 0.f;
		last_height = 0.f;
		ground_vel = g_ctx.m_local->m_vecVelocity( ).length2d( );

		was_onground = true;
	}
	else {
		was_onground = false;
		float vel = g_ctx.m_local->m_vecVelocity( ).length2d( );
		if( vel > last_jump_max_speed ) {
			last_jump_max_speed = vel;
		}
		float delta = g_ctx.m_local->m_vecOrigin( ).z - last_origin.z;
		if( std::abs( delta ) > std::abs( last_height ) ) {
			last_height = delta;
		}
	}
}

//fuck hardcode
const vec3_t mins( -26.f, -26.f, 0 );
const vec3_t maxs( 26.f, 26.f, 44.f );

constexpr int TRACE_STEP_MAX = 45;
bool trace_ideal_step( float step, float speed, vec3_t velocity, vec3_t start_pos ) {
	vec3_t direction = math::vector_angles( vec3_t( ), velocity );
	float  wish_step = direction.y + step;

	vec3_t origin = start_pos;

	vec3_t start = origin;
	vec3_t trace_step = math::angle_vectors( vec3_t( 0, wish_step, 0 ) ) * velocity.length2d( ) * TICK_INTERVAL( );

	vec3_t pos = start + trace_step;

	CGameTrace tr;
	CTraceFilter filter;
	filter.pSkip = g_ctx.m_local;

	for( size_t i{ }; i <= 1 / TICK_INTERVAL( ); ++i ) {
		start = pos;
		trace_step = math::angle_vectors( vec3_t( 0, wish_step += step, 0 ) ) * velocity.length2d( ) * TICK_INTERVAL( );
		pos += trace_step;

		Ray_t ray;
		ray.Init( start, pos, mins, maxs );

		g_csgo.m_trace( )->TraceRay( ray, MASK_SOLID, &filter, &tr );

		if( !tr.DidHit( ) ) {
			if( i == TRACE_STEP_MAX ) {
				return true;
			}
			continue;
		}

		break;
	}

	return false;
}

float c_movement::get_best_strafe_angle( ) {
	vec3_t velocity = g_ctx.m_local->m_vecVelocity( );
	float ideal_step = get_ideal_strafe_step( velocity.length2d( ) ) * 0.8f;

	float step = ideal_step;
	vec3_t start = g_ctx.m_local->m_vecOrigin( );

	for( size_t i{ }; i < 9; ++i ) {
		if( trace_ideal_step( step, velocity.length2d( ), velocity, start ) ) {
			vec3_t direction = math::vector_angles( vec3_t( ), velocity );
			return direction.y + step;
		}

		step -= step * 0.2f;
	}

	step = ideal_step;
	for( size_t i{ }; i < velocity.length2d( ) * 0.015f; ++i ) {
		step += step * 0.2f;
		if( trace_ideal_step( step, velocity.length2d( ), velocity, start ) ) {
			vec3_t direction = math::vector_angles( vec3_t( ), velocity );
			return direction.y + step;
		}
	}

	return math::vector_angles( vec3_t( ), velocity ).y;
}

void c_movement::circle_strafe( ) {
	if( g_settings.misc.circle_strafe ) {
		if( g_ctx.m_local->m_nMoveType( ) == MOVETYPE_LADDER || g_ctx.m_local->m_nMoveType( ) == MOVETYPE_NOCLIP )
			return;

		static bool can_finish = true;
		auto cmd = g_ctx.get_last_cmd( );
		if( g_input.is_key_pressed( g_settings.misc.circle_strafe_key ) || !can_finish ) {
			m_ucmd->m_buttons |= IN_JUMP;
			cmd->m_forwardmove = 450.f;

			float speed = g_ctx.m_local->m_vecVelocity( ).length2d( );
			if( speed > 1.f ) {
				can_finish = false;
				float angle = get_best_strafe_angle( );

				float delta = std::remainderf( m_ucmd->m_viewangles.y - angle, 360.f );

				cmd->m_forwardmove = 5850.f / speed;
				cmd->m_sidemove = -450.f;
				rotate_movement( cmd, delta );

				vec3_t current_view;
				g_csgo.m_engine( )->GetViewAngles( current_view );
				float view_delta = std::remainderf( current_view.y - angle, 360.f );

				if( std::fabs( view_delta ) < 10.0f || speed < 250.f ) {
					can_finish = true;
				}
			}
			else {
				can_finish = true;
			}
		}
	}
}

void c_movement::fast_walk( ) {
	if( !g_settings.misc.fastwalk )
		return;

	if( !g_input.is_key_pressed( g_settings.misc.fastwalk_key ) )
		return;

	if( m_ucmd->m_buttons & IN_SPEED )
		m_ucmd->m_buttons &= ~IN_SPEED;

	vec3_t vel = g_ctx.m_local->m_vecVelocity( );
	float speed = vel.length2d( );
    
	if( speed < 126.f )
        return;

    static auto sv_accelerate = g_csgo.m_cvar( )->FindVar( xors( "sv_accelerate" ) );
    float accel = sv_accelerate->get_float( );
        
    float surf_friction = 1.f;
    float max_accelspeed = accel * g_csgo.m_globals->m_interval_per_tick * speed * surf_friction;
        
    float wishspeed;
       
    if( speed - max_accelspeed <= -1.f ) {
        wishspeed = max_accelspeed / ( speed / ( accel * g_csgo.m_globals->m_interval_per_tick ) );
    }
    else {
        wishspeed = max_accelspeed;
    }
        
    vec3_t ndir = math::vector_angles( vel * -1.f );
    ndir.y = g_csgo.m_engine( )->GetViewAngles( ).y - ndir.y;
    ndir = math::angle_vectors( ndir );
        
    m_ucmd->m_forwardmove = ndir.x * wishspeed;
    m_ucmd->m_sidemove = ndir.y * wishspeed;
}

END_REGION