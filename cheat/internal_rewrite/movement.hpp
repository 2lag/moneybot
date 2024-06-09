#pragma once
#include "util.hpp"
#include <vector>

//forward declarations
class user_cmd_t;

NAMESPACE_REGION( features )

class c_movement {
	user_cmd_t* m_ucmd{ };
	bool		m_direction{ };

	void bhop( );
	void auto_strafer( );

  void jump_bug( );

  void edge_bug( );
  void perform_edge_bug( );

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
  std::vector<std::pair<vec3_t, vec3_t>> bug_path;
  std::vector<std::pair<vec3_t, vec3_t>> hit_path;
  bool run_edge_bug = false;

	void operator()( user_cmd_t* ucmd ) {
		m_ucmd = ucmd;
    //edge_bug( );
    //perform_edge_bug( );
    pixel_surf( );
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