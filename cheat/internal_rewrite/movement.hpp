#pragma once
#include "vector.hpp"
#include "util.hpp"
#include <vector>

//forward declarations
class user_cmd_t;

NAMESPACE_REGION( features )

class c_movement {
public:
  struct move_data_t {
    vec3_t velocity;
    vec3_t origin;
  };
  
  struct pred_data_t : move_data_t {
    vec2_t move;
    vec3_t angle;

    bool onground;
    bool eb_tick;
    float eb_frac;
    float eb_dot;

    bool hit_wall;
    int  ticks;
  };

  struct eb_path {
    std::vector<pred_data_t> path;
    vec3_t edge;
    int found;
  };


protected:
  user_cmd_t* m_ucmd{ };
	bool		m_direction{ };

  eb_path m_eb_path;

	void bhop( );
	void auto_strafer( );

  void jump_bug( );

  void edge_bug( );
  eb_path get_best_eb_angle();
  eb_path simulate_eb_path( float angle );
  void strafe_to_path( eb_path* path );
  int find_edge( pred_data_t* d, float strafe );

  void pixel_surf( );

	void edge_jump( );
	void jump_stats( );

	bool get_best_direction( float ideal_step, float left, float right, float weight );
	float get_best_strafe_step( float speed, vec3_t direction );
	float get_best_strafe_angle( );
	void circle_strafe( );
    void fast_walk( );
    void air_duck( );

public:
  std::vector<eb_path> paths;
  vec3_t edge_pos;
  int eb_found;
  
	void operator()( user_cmd_t* ucmd ) {
		m_ucmd = ucmd;
    edge_bug( );
    //pixel_surf( );
		auto_strafer( );
		circle_strafe( );
		fast_walk( );
		bhop( );
    jump_bug( );
		edge_jump( );
		jump_stats( );
		air_duck( );
	}
};

END_REGION