#pragma once
#include "util.hpp"
#include "IClientEntityList.hpp"
#include "ClientClass.hpp"
#include "color.hpp"

enum ClientFrameStage_t {
	FRAME_UNDEFINED = -1,
	FRAME_START,
	FRAME_NET_UPDATE_START,
	FRAME_NET_UPDATE_POSTDATAUPDATE_START,
	FRAME_NET_UPDATE_POSTDATAUPDATE_END,
	FRAME_NET_UPDATE_END,
	FRAME_RENDER_START,
	FRAME_RENDER_END
};

struct GlowObject_t {
private:
	int unk0001; // 0x0
public:
	IClientEntity* m_pEntity; // 0x4
	fclr_t color;
	char junk0[ 8 ];
	float m_flBloomAmount;
	char junk1[ 4 ];
	bool m_bRenderWhenOccluded;
	bool m_bRenderWhenUnoccluded;
	bool m_fullBloom;
	char unk[ 1 ];
	int  m_fullBloomStencil;
	int unk2;
	int  m_splitscreenSlot;
};

struct GlowObjectManager_t {
	GlowObject_t* DataPtr; //0000 
	unsigned int Max; //0004 
	unsigned int unk02; //0008 
	unsigned int Count; //000C 
	unsigned int DataPtrBack; //0010 
	int m_nFirstFreeSlot; //0014 
	unsigned int unk1; //0018 
	unsigned int unk2; //001C 
	unsigned int unk3; //0020 
	unsigned int unk4; //0024 
	unsigned int unk5; //0028 
};

class IBaseClientDLL
{
public:
	ClientClass* GetAllClasses( ) {
		using fn = ClientClass*( __thiscall* )( void* );
		return util::get_vfunc< fn >( this, 8 )( this );
	}

	bool DispatchUserMessage( int msg_type, ulong_t a3, ulong_t len, const void* msg_data ) {
		using fn = bool( __thiscall* )( void*, int, ulong_t, ulong_t, const void* );
		return util::get_vfunc< fn >( this, 38 )( this, msg_type, a3, len, msg_data );
	}

	/*
	DONTCARE
	void SetCrosshairAngle( Vector& angle )
	{
		typedef void( __thiscall* SetCrosshairAngleFn )( void*, Vector& );
		CallVFunction<SetCrosshairAngleFn>( this, 29 )( this, angle );
	}

	void HudProcessInput( bool enable )
	{
		using fn = void( __thiscall* )( void*, bool );
		CallVFunction<fn>( this, 10 )( this, enable );
	}

	void HudUpdate( bool enable )
	{
		using fn = void( __thiscall* )( void*, bool );
		CallVFunction<fn>( this, 11 )( this, enable );
	}

	void IN_ActivateMouse( )
	{
		using fn = void( __thiscall* )( void* );
		CallVFunction<fn>( this, 15 )( this );
	}

	void IN_DeactivateMouse( )
	{
		using fn = void( __thiscall* )( void* );
		CallVFunction<fn>( this, 16 )( this );
	}*/
};