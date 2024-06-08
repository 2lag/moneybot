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

/*
std::vector<float> edge_dist;

template <typename t>
void scale_vector( vec3_t& vec, t first_min, t first_max, t second_min, t second_max ) {
  #define map( i, a, b ) ( second_min + ( ( second_max - second_min ) / ( first_max - first_min ) ) * ( i - first_min ) )
  vec = {
    map( vec.x, one, two ),
    map( vec.y, one, two ),
    0.f
  };
}

void find_max_strafe_angle( vec3_t pred_vel, vec3_t pred_pos, float max_speed, float& max_ang ) {
  for ( int idx = 0; idx <= 180; idx += 1 ) {
    float rads_off( DEG2RAD( idx ) );
    vec3_t t_wishdir {
      cos( rads_off ),
      sin( rads_off ),
      0.f
    };
    t_wishdir.normalize_vector( );
    scale_vector< int >( t_wishdir, 0, 1, 0, max_speed );

    vec3_t t_vel = pred_vel;
    g_cheat.m_prediction.air_accelerate(
      g_ctx.m_local, pred_pos, t_vel, t_wishdir, max_speed
    );

    float ang_delta = atan2( t_vel.y, t_vel.x ) - atan2( pred_vel.y, pred_vel.x );
    float d_speed = ( t_vel - pred_vel ).length2d( );

    if ( fabs( ang_delta ) > max_ang ) {
      max_ang = fabs( ang_delta );
      max_ang *= 2; // 1 = barebones, 2 = perf, 3 = willingly slowdown
    }
  }
}

bool edge_bug_detect( c_base_player* ent, const vec3_t& _origin, const vec3_t& pred_vel ) {
  CGameTrace tr1{ };
  CGameTrace tr2{ };
  tr2.fraction = 1.f;

  vec3_t vel = pred_vel;
  vec3_t min = ent->m_vecMins( );
  vec3_t max = ent->m_vecMaxs( );
  vec3_t origin = _origin;
  vec3_t end = {
    origin.x,
    origin.y,
    origin.z - 2.f
  };
  g_cheat.m_prediction.try_touch_ground( ent, origin, end, min, max, &tr1 );
  
  if ( tr1.DidHit( ) )
    vel.z = 0;
  else
    return false;

  origin += ( vel * TICK_INTERVAL( ) );
  end += ( vel * TICK_INTERVAL( ) );
  g_cheat.m_prediction.try_touch_ground_in_quadrants( ent, origin, end, &tr2 );
  
  if ( tr2.endpos.z > tr1.endpos.z )
    return false;

  if ( !tr2.DidHit( ) )
    return true; // buggin

  return false; // ground
}

vec3_t extrapolate_edge( c_base_player* ent, vec3_t& origin, vec3_t& velocity, bool& stop_calc ) {
  static auto sv_jump_impulse = g_csgo.m_cvar( )->FindVar( xors( "sv_jump_impulse" ) );
  static auto sv_gravity = g_csgo.m_cvar( )->FindVar( xors( "sv_gravity" ) );

  auto& min = ent->m_vecMins( );
  auto& max = ent->m_vecMaxs( );

  velocity -= sv_gravity->get_float( ) * 0.5f * TICK_INTERVAL( );

  auto& start = origin;
  auto end = start + velocity * TICK_INTERVAL( );

  CTraceFilter f;
  CGameTrace tr;
  Ray_t ray;

  ray.Init( start, end, min, max );
  f.pSkip = ent;

  g_csgo.m_trace( )->TraceRay( ray, MASK_PLAYERSOLID, &f, &tr );

  if ( tr.fraction != 1.f ) {
    for ( int idx = 0; idx < 2; ++idx ) {
      velocity -= tr.plane.normal * velocity.dot( tr.plane.normal );
      auto dot = velocity.dot( tr.plane.normal );
      if ( dot < 0.f ) velocity -= dot * tr.plane.normal;

      end = tr.endpos + ( velocity * TICK_INTERVAL( ) * ( 1.f - tr.fraction ) );
      ray.Init( tr.endpos, end, min, max );
      g_csgo.m_trace( )->TraceRay( ray, MASK_PLAYERSOLID, &f, &tr );

      if ( tr.fraction == 1.f )
        break;
    }
  }
  
  vec3_t tr_end = tr.endpos;

  end = tr.endpos;
  end.z -= 2.f;

  ray.Init( tr.endpos, end, min, max );
  g_csgo.m_trace( )->TraceRay( ray, MASK_PLAYERSOLID, &f, &tr );

  if ( tr.fraction != 1.f && tr.plane.normal.z > 0.7f )
    stop_calc = true;
  else
    velocity.z -= sv_gravity->get_float( ) * 0.5f * TICK_INTERVAL( );

  return tr_end;
}

int eb_count;
// https://wiki.sourceruns.org/wiki/Edgebug
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
  vec3_t start_vel = g_ctx.m_local->m_vecVelocity( );
  vec3_t start_pos = g_ctx.m_local->m_vecOrigin( );
  vec3_t pred_vel = start_vel,
         pred_pos = start_pos,
         best_vel{ },
         best_pos{ };
  edge_dist.clear( );

  // swap off static values n come up with good dynamics
  const int max_ticks = util::is_low_fps( ) ? 32 : 64;
  const int num_paths = util::is_low_fps( ) ? 8 : 20;

  float max_ang = 0.f;
  const float max_speed = g_ctx.m_local->get_weapon( )->get_wpn_info( )->max_speed;

  find_max_strafe_angle(
    pred_vel, pred_pos,
    max_speed, max_ang
  );

  const float ang_step = max_ang / num_paths;
  for ( int idx = 0; idx < num_paths; ++idx ) {
    vec3_t path_pred_pos = pred_pos;
    vec3_t path_pred_vel = pred_vel;
    vec3_t prev_pos = path_pred_pos;
    bug_path.clear( );
    
    bool stop_calc = false;
    for ( int tick = 0; tick < max_ticks; ++tick ) {
      if ( stop_calc || !hit_path.empty( ) ) break;

      float rads_off = ( idx - ( num_paths / 2 ) ) * ang_step * ( tick + 1 );
      vec3_t wishdir = {
        pred_vel.x * cos( rads_off ) - pred_vel.y * sin( rads_off ),
        pred_vel.x * sin( rads_off ) - pred_vel.y * cos( rads_off ),
        0.f
      };
      wishdir.normalize_vector( );

      g_cheat.m_prediction.air_accelerate( g_ctx.m_local,
        path_pred_pos, path_pred_vel, wishdir, max_speed
      );

      path_pred_pos = extrapolate_edge(
        g_ctx.m_local, path_pred_pos, path_pred_vel, stop_calc
      );

      bug_path.push_back({ prev_pos, path_pred_pos });
      prev_pos = path_pred_pos;

      if ( !edge_bug_detect( g_ctx.m_local, path_pred_pos, path_pred_vel ) )
        continue;

      printf( "eb : %d\r", ++eb_count );

      hit_path = bug_path;

      // wrong, depend on trace
      edge_dist.push_back(
        ( g_ctx.m_local->m_vecOrigin( ) - path_pred_pos ).length2d( )
      );

      run_edge_bug = true;

      break;
    }

    if ( run_edge_bug || !hit_path.empty( ) )
      break;
  }
}

void strafe_to_edge( user_cmd_t* cmd, const vec3_t& target_dir ) {
  float yaw = g_csgo.m_engine( )->GetViewAngles( ).y;
  vec3_t vel = g_ctx.m_local->m_vecVelocity( );
  float speed = vel.length2d( );
  vec3_t move_dir = target_dir;
  move_dir.normalize_vector( );

  float target_yaw = math::vector_angles( vec3_t( ), move_dir ).y;
  float delta_yaw = remainderf( target_yaw - yaw, 360.f );

  if ( fabs( delta_yaw ) < 1.f )
    cmd->m_sidemove = 0;
  else
    cmd->m_sidemove = delta_yaw > 0 ? 450.f : -450.f;

  rotate_movement( cmd, delta_yaw );
}

void c_movement::perform_edge_bug( ) {
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

  if ( hit_path.empty( ) ) {
    run_edge_bug = false;
    return;
  }

  if ( !run_edge_bug )
    return;
  
  strafe_to_edge( m_ucmd, hit_path.front( ).second );
  hit_path.erase( hit_path.begin( ) );

  if ( !edge_dist.empty( ) && edge_dist.back( ) < 4.f && hit_path.size( ) < 2 ) // < 1 ?
    m_ucmd->m_buttons |= IN_DUCK;
}
*/

void c_movement::jump_bug( ) {
  static const float jb_dist = .1f;
  static bool was_jump = false;

  if ( !g_settings.misc.jump_bug_type )
    return;

  if ( !g_input.is_key_pressed( ( VirtualKeys_t )g_settings.misc.jump_bug_key( ) ) )
    return;
 
  if( !g_ctx.m_local->is_alive( ) )
    return;
 
  if ( g_ctx.m_local->m_nMoveType( ) == MOVETYPE_LADDER )
    return;

  m_ucmd->m_buttons &= ~IN_JUMP;
 

  if ( g_ctx.m_local->m_fFlags( ) & FL_ONGROUND ) {
    if( !was_jump )
      m_ucmd->m_buttons |= IN_JUMP;
    was_jump = m_ucmd->m_buttons & IN_JUMP;
    return;
  }

  was_jump = false;

  vec3_t prevstart = g_ctx.m_last_origin;
  vec3_t prevend = prevstart;
  prevend.z -= 3.9f;

  CGameTrace prev_tr{};

  g_cheat.m_prediction.try_touch_ground_in_quadrants(
    g_ctx.m_local, prevstart, prevend, &prev_tr
  );

  vec3_t origin = g_ctx.m_local->m_vecOrigin( );
  vec3_t vel = g_ctx.m_local->m_vecVelocity( );
  vec3_t min = g_ctx.m_local->m_vecMins( );
  vec3_t max = g_ctx.m_local->m_vecMaxs( );
  vec3_t end = origin;
  end.z -= jb_dist;
 
  CGameTrace tr;

  g_cheat.m_prediction.try_touch_ground(
    g_ctx.m_local, origin, end, min, max, &tr
  );
  bool trace_hit_ground = tr.startpos.z - tr.endpos.z < FLT_EPSILON;

  if ( trace_hit_ground && prev_tr.m_pEnt ) {
    m_ucmd->m_buttons &= ~IN_DUCK;
    m_ucmd->m_buttons |= IN_JUMP;
    return;
  }

 
  // if trace distance is _ > 4 duck otherwise jump
  if ( g_settings.misc.jump_bug_type == 2 ) {
    if ( !trace_hit_ground )
      m_ucmd->m_buttons |= IN_DUCK;
  } // if next tick's trace dist _ < 4
  else if ( g_settings.misc.jump_bug_type == 1 ) {
    end.z += jb_dist;
    origin = end;
    end += vel * TICK_INTERVAL( ) * 1.1f;
    end.z -= jb_dist;
 
    CGameTrace next_tr;
 
    g_cheat.m_prediction.try_touch_ground_in_quadrants(
      g_ctx.m_local, origin, end, &next_tr
    );
 
    if ( next_tr.m_pEnt )
      m_ucmd->m_buttons |= IN_DUCK;
  }
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