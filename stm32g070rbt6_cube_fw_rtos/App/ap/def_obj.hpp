/*
 * def_obj.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef AP_DEF_OBJ_HPP_
#define AP_DEF_OBJ_HPP_



#define AP_DEF_OBJ_BASE_MOTOR                   0x0010
#define AP_DEF_OBJ_BASE_CYLINDER                0x0100
#define AP_DEF_OBJ_BASE_VACUUM                  0x0120

#define AP_DEF_OBJ_BASE_IO                      0x0140
#define AP_DEF_START_IN_ADDR                    1000
#define AP_DEF_START_OUT_ADDR                   3000


#define AP_DEF_OBJ_MOTOR_ID_BASE                1
#define M_SetMotorId(instance_no)               AP_DEF_OBJ_MOTOR_ID_BASE + instance_no
#define M_GetMotorInstanceId(node_id)           node_id - AP_DEF_OBJ_MOTOR_ID_BASE
#define M_GetMotorObjId(obj)                    AP_DEF_OBJ_BASE_MOTOR|obj

namespace AP_OBJ
{
	enum MOTOR
	{
		MOTOR_X,
		MOTOR_Y,
#ifdef APP_USE_MOTOR_R
		MOTOR_R,
#endif
		MOTOR_MAX,

	};

	enum CYL
	{
		CYL_TOOL_OPENCLOSE,

		CYL_MAX,

	};

	enum VAC
	{
		VAC_DETECT,

		VAC_MAX,
	};

}




#endif /* AP_DEF_OBJ_HPP_ */
