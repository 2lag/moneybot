#include "movement.hpp"
#include "interface.hpp"
#include "settings.hpp"
#include "context.hpp"

#include <algorithm>
#include "base_cheat.hpp"
#include "d3d.hpp"
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

const float CS_PLAYER_SPEED_RUN = 260.f;

// i had to reverse this
float gamemovement_maxspeed() {
  static cvar_t* sv_maxspeed = g_csgo.m_cvar()->FindVar( "sv_maxspeed" );
  
  float maxspeed = math::min( sv_maxspeed->get_float(), CS_PLAYER_SPEED_RUN );
  if( g_ctx.m_local->m_flStamina( ) > 0 ) {
    float speed_scale = std::clamp( 1.0f - g_ctx.m_local->m_flStamina( ) / 100.f, 0.f, 1.f );
    speed_scale *= speed_scale;
    maxspeed *= speed_scale;
  }
  return maxspeed;
}

// https://github.com/ValveSoftware/source-sdk-2013/blob/master/mp/src/game/shared/gamemovement.cpp#L1030-L1041
void clamp_move( vec2_t* move ) {
  float maxspeed = gamemovement_maxspeed();
  float speed_sqr = move->lengthsqr();

  if( !!speed_sqr && speed_sqr > maxspeed * maxspeed ) {
    float ratio = maxspeed / sqrt( speed_sqr );
    move->x *= ratio;
    move->y *= ratio;
  }
}

void store_eb_pos( c_movement::pred_data_t* d, vec3_t last, vec3_t cur ) {
  float original_z = cur.z;
  last.z = cur.z;

  vec3_t dir = cur - last;
  dir.normalize_vector( );

  bool pls_;
  CGameTrace t;
  g_cheat.m_prediction.trace_player_bbox( g_ctx.m_local, cur, cur - dir * (cur-dir).length(), &t );

  vec3_t endpos = t.endpos;
  endpos += t.plane.normal * -16.f;

  d->eb_hit = endpos;
  d->eb_hit.z = cur.z;
  d->eb_norm = t.plane.normal;
}

// https://github.com/ValveSoftware/source-sdk-2013/blob/master/mp/src/game/shared/gamemovement.cpp#L3774
void categorize_position( c_movement::pred_data_t* d ) {
  float zvel = d->velocity.z;

  vec3_t start = d->origin;
  vec3_t end = start;
  end.z -= 2.f;
  
  CGameTrace t;
  g_cheat.m_prediction.trace_player_bbox( g_ctx.m_local, start, end, &t );

  if( !t.m_pEnt )
    return;

  // surfing up ground
  if( zvel > 140.f ) {
    d->onground = false;
    // technically the behavior of surfing up a standable slope
    // is identical to edgebugging, but we dont want that.
    d->hit_wall = true;
    return;
  }

  if( t.plane.normal < 0.7f ) {
    g_cheat.m_prediction.try_touch_ground_in_quadrants( g_ctx.m_local, start, end, &t );
    if( !t.m_pEnt || t.plane.normal < 0.7f ) {
      return;
    }
  }

  d->eb_tick = false;
  d->onground = true;
}

// https://github.com/ValveSoftware/source-sdk-2013/blob/master/mp/src/game/shared/gamemovement.cpp#L2560
void try_player_move( c_movement::pred_data_t* d ) {
  CGameTrace t;
  vec3_t end;
  vec3_t travel;
  float timeleft = TICK_INTERVAL( );
  int numbumps = 4;

  vec3_t start = d->origin;

  for( int b = 0; b < numbumps; ++b ) {
    travel = d->velocity * timeleft;
    end = d->origin + travel;
    g_cheat.m_prediction.trace_player_bbox( g_ctx.m_local, d->origin, end, &t );

    if( t.fraction == 1.f ) {
      d->origin = t.endpos;
      return;
    }

    if( t.allsolid || t.fraction < 0.0001f )
      return;

    d->origin = t.endpos;
    timeleft -= timeleft * t.fraction;

    vec3_t new_velocity;
    g_cheat.m_prediction.clip_velocity( d->velocity, t.plane.normal, new_velocity, 1.f );
    d->velocity = new_velocity;

    if( t.plane.normal.z > 0.7 ) {
      d->eb_tick = true;
      d->eb_frac = t.fraction;
    }
    else {
      d->hit_wall = true;
      break;
    }
  }
}

// https://github.com/ValveSoftware/source-sdk-2013/blob/master/mp/src/game/shared/gamemovement.cpp#L1753
void eb_airmove( c_movement::pred_data_t* d ) {
  vec3_t forward, right, up;
  vec3_t wishvel;
  vec3_t wishdir;
  float wishspeed;
  float maxspeed = gamemovement_maxspeed();
  
  math::angle_vectors( d->angle, &forward, &right, &up );

  g_cheat.m_prediction.start_gravity( g_ctx.m_local, d->origin, d->velocity );
  
  vec2_t move = d->move;
  clamp_move( &move );
  float fmove = move.x;
  float smove = move.y;

  forward.z = right.z = 0;
  forward.normalize_vector();
  right.normalize_vector();

  for( int i = 0; i < 2; ++i ) {
    wishvel[i] = forward[i] * fmove + right[i] * smove;
  }
  wishvel[2] = 0;

  wishdir = wishvel;
  wishspeed = wishdir.length();
  wishdir.normalize_vector();

  if( wishspeed > 0 && wishspeed > maxspeed ) {
    wishvel *= ( maxspeed / wishspeed );
    wishspeed = maxspeed;
  }

  g_cheat.m_prediction.air_accelerate( g_ctx.m_local, d->origin, d->velocity, wishdir, wishspeed );
  try_player_move( d );
  categorize_position( d );
  g_cheat.m_prediction.finish_gravity( g_ctx.m_local, d->origin, d->velocity );
}


int c_movement::find_edge( pred_data_t* d, float strafe ) {
  const int EDGE_TICKS_MAX = 64; // also make a setting or base off fps

  eb_path p{};
  
  p.path.push_back( *d );
  for( int i = 0; i < EDGE_TICKS_MAX; ++i ) {
    bool eb = d->eb_tick;
    vec3_t origin = d->origin;
    eb_airmove( d );

    if( eb ) {
      store_eb_pos( d, origin, d->origin );

      p.path.push_back( *d );
      paths.push_back( p );
      return 1;
    }

    ++d->ticks;
    d->angle.y += strafe;
    d->angle.clamp( );
    p.path.push_back( *d );
    
    if( d->eb_tick && !eb ) {
      p.edge = d->origin;
      p.found = 1;
    }
    if( d->onground || d->hit_wall )
      break;
  }

  paths.push_back( p );
  return 0;
}

c_movement::eb_path c_movement::simulate_eb_path( float ang ) {
  pred_data_t d{};
  float strafe = ang;
  d.origin = g_cheat.m_prediction.old_origin;
  d.velocity = g_cheat.m_prediction.old_velocity;
  d.angle.y = m_ucmd->m_viewangles.y + strafe;
  d.angle.clamp( );
  if( strafe < 0.f )
    d.move.y = 450.f;
  if( strafe > 0.f )
    d.move.y = -450.f;

  find_edge( &d, strafe );
  auto path = paths.at( paths.size( ) - 1 );

  return path;
}

bool c_movement::eb_iterate_angles( eb_path* out_path, float* start, float* end, float* start_z, float* end_z ) {
  eb_path p;

  float last_ang = *start;
  float last_z = *start_z;
  float max_z_delta = 4.f;
  float old_z_delta = abs( *start_z - *end_z );
  float step = ( *end - *start ) * 0.2f;
  bool found = false;
  
  for( int i = 1; i <= 3; ++i ) {
    float ang = last_ang + step;

    p = simulate_eb_path( ang );
    if( p.found ) {
      *out_path = p;
      return true;
    }
    
    float z = p.path.at( p.path.size() - 1 ).origin.z;
    float start_diff = z - *start_z;
    if( abs( start_diff ) < 4.f ) {
      *start = ang;
      *start_z = z;
    }
    
    float end_diff = z - *end_z;
    if( abs( end_diff ) < 4.f ) {
      *end = ang;
      *end_z = ang;
    }

    last_z = z;
    last_ang = ang;
  }

  return false;
}


c_movement::eb_path c_movement::get_best_eb_angle() {
  float percent = 1.f; // make a setting from 0.5 - 2
  eb_path best_path;
  
  best_path = simulate_eb_path( 0.f );
  if( best_path.found ) {
    return best_path;
  }

  float total_ang = 3.f * percent;
  float last_ang = 0.f;

  float drop_start_ang = 0.f;
  float drop_end_ang = 0.f;

  float last_z_pos = FLT_MAX;
  float last_z_neg = FLT_MAX;
  float best_z_start = 0.f;
  float best_z_end = 0.f;
  float max_z_delta = 4.f;
  for( float strafe = 0.f; strafe <= total_ang; strafe += total_ang * 0.3f ) {
    best_path = simulate_eb_path( strafe );
    float pos_z = best_path.path.at( best_path.path.size( ) - 1 ).origin.z;
    bool pos_wall = best_path.path.at( best_path.path.size( ) - 1 ).hit_wall;

    if( best_path.found )
      return best_path;

    best_path = simulate_eb_path( -strafe );
    float neg_z = best_path.path.at( best_path.path.size( ) - 1 ).origin.z;
    bool neg_wall = best_path.path.at( best_path.path.size( ) - 1 ).hit_wall;

    if( best_path.found )
      return best_path;

    // z delta means there was a drop between the two
    // meaning there is an edge between them
    // 4.f is arbitrary
    if( !pos_wall && last_z_pos < FLT_MAX && last_z_pos - pos_z > max_z_delta ) {
      max_z_delta = abs( last_z_pos - pos_z );
      best_z_start = last_z_pos;
      best_z_end = pos_z;
      drop_start_ang = last_ang;
      drop_end_ang = strafe;
    }
    
    if( !neg_wall && last_z_neg < FLT_MAX && last_z_neg - neg_z > max_z_delta ) {
      max_z_delta = abs( last_z_neg - neg_z );
      best_z_start = neg_z;
      best_z_end = last_z_neg;
      drop_end_ang = -last_ang;
      drop_start_ang = -strafe;
    }

    last_z_neg = neg_z;
    last_z_pos = pos_z;
    last_ang = strafe;
  }

  if( max_z_delta > 4.f ) {
    float start = util::perf_counter( );
    float msec = 0.f;
    float delta = 0.f;
    bool found = false;
    float timeout = TICK_INTERVAL( ) * 1000.f * 0.45f;

    int iter = 0;
    do {
      found = eb_iterate_angles(
        &best_path,
        &drop_start_ang,
        &drop_end_ang,
        &best_z_start,
        &best_z_end
      );

      float msec = util::perf_counter( );
      ++iter;
      delta = msec - start;
    } while( !found && delta < timeout && iter < 64 );
  }

  return best_path;
}

void c_movement::strafe_to_path( eb_path* path ) {
  if( path->path.empty( ) )
    return;

  m_ucmd->m_forwardmove = path->path.at( 0 ).move.x;
  m_ucmd->m_sidemove = path->path.at( 0 ).move.y;

  m_ucmd->m_viewangles.y = path->path.at( 0 ).angle.y;
  g_csgo.m_engine( )->SetViewAngles( m_ucmd->m_viewangles );
  if( path->path.size( ) == 1 )
    m_eb_hit = true;
  
  path->path.erase( path->path.begin( ) );
}


// https://wiki.sourceruns.org/wiki/Edgebug
void c_movement::edge_bug( ) {
  auto cleanup = [&]( ) { m_eb_path = {}; };

  if( !g_settings.misc.edge_bug )
      return cleanup( );
  if ( !g_input.is_key_pressed( ( VirtualKeys_t )g_settings.misc.edge_bug_key( ) ) )
    return cleanup( );
  if ( !g_ctx.m_local->is_alive( ) )
    return cleanup( );
  if( g_ctx.m_local->m_fFlags( ) & FL_ONGROUND )
    return cleanup( );
  if ( g_ctx.m_local->m_nMoveType( ) == MOVETYPE_LADDER )
    return cleanup( );

  paths.clear( );

  auto& wanted_path = m_eb_path.path;

  if( wanted_path.size() > 0 ) {
    auto last_tick = wanted_path.at( wanted_path.size( ) - 1 );
    if( last_tick.eb_tick ) {
      vec3_t wanted_origin = m_eb_path.path.at( 0 ).origin;
      vec3_t origin = g_cheat.m_prediction.old_origin;

      // 0.001 is kinda lenient but whatever 
      // this optimally shouldnt happen anyway unless ur fps drops below tickrate
      if( origin.dist_to_sqr( wanted_origin ) < 0.001f ) {
        strafe_to_path( &m_eb_path );
        return;
      }
    }

    m_eb_path.path.clear( );
  }
  
  eb_path best_path = get_best_eb_angle();
  if( best_path.found ) {
    strafe_to_path( &best_path );
    m_eb_path = best_path;
  }
}

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