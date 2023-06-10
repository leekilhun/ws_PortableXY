/*
 * enMotor_moons.hpp
 *
 *  Created on: 2023. 6. 10.
 *      Author: gns2l
 */

#ifndef AP__INC_ENMOTOR_MOONS_HPP_
#define AP__INC_ENMOTOR_MOONS_HPP_


namespace MOTOR
{

	constexpr uint16_t MODBUS_MULTIPLE_PARAM_VEL = 4;//240;
	constexpr uint16_t MODBUS_MULTIPLE_PARAM_ACC = 6;
	constexpr uint8_t MODBUS_FAIL_RETRY_CNT_MAX = 3;
	class enMotor_moons
	{
		/****************************************************
		 *  data
		 ****************************************************/

		 /*
			 Hex Value BLu SV STAC6 ST STM
			 0x0001     Position Limit
			 0x0002     CCW Limit
			 0x0004     CW Limit
			 0x0008     Over Temp
			 0x0010     Excess Regen* Internal Voltage Excess Regen Internal Voltage Internal Voltage
			 0x0020     Over Voltage
			 0x0040     Under Voltage* Under Voltage Under Voltage Under Voltage Under Voltage
			 0x0080     Over Current
			 0x0100     Bad Hall Sensor Open Motor Winding
			 0x0200     Bad Encoder (not used)
			 0x0400     Comm Error
			 0x0800     Bad Flash
			 0x1000     Wizard Failed No Move
			 0x2000     Current Foldback Motor Resistance Out of Range (not used) (not used)
			 0x4000     Blank Q Segment
			 0x8000     No Move (not used)

			*/
		union MOONS_SS_ALARM_STATUS
		{
			uint16_t	al_status;
			struct
			{
				unsigned	Position_Error : 1;        // = 0x00000001;
				unsigned	CCW_Limit : 1;		         // = 0x00000002;
				unsigned	CW_Limit : 1;			         // = 0x00000004;
				unsigned	Over_Temp : 1;	           // = 0x00000008;
				unsigned	Internal_Voltage : 1;      // = 0x00000010;
				unsigned	Over_Voltage : 1;		       // = 0x00000020;
				unsigned	Under_Voltage : 1;	 	     // = 0x00000040;
				unsigned	Over_Current : 1;	         // = 0x00000080;
				unsigned	Open_Winding : 1;	         // = 0x00000100;
				unsigned	Bad_Encoder : 1;		       // = 0x00000200;
				unsigned	Comm_Error : 1;		         // = 0x00000400;
				unsigned	Bad_Flash : 1;		         // = 0x00000800;
				unsigned	No_Move : 1;               // = 0x00001000;
				unsigned	Current_Foldback : 1;	     // = 0x00002000;
				unsigned	Blank_Q_Segment : 1;	     // = 0x00004000;
				unsigned	NV_Memory_Double_err : 1;	 // = 0x00008000;
			};
		};
		/*

			Hex Value  Status Code bit definition
			0x0001     Motor Enabled (Motor Disabled if this bit = 0)
			0x0002     Sampling (for Quick Tuner)
			0x0004     Drive Fault (check Alarm Code)
			0x0008     In Position (motor is in position)
			0x0010     Moving (motor is moving)
			0x0020     Jogging (currently in jog mode)
			0x0040     Stopping (in the process of stopping from a stop command)
			0x0080     Waiting for an input (executing WI command)
			0x0100     Saving (parameter data is being saved)
			0x0200     Alarm present (check Alarm Code)
			0x0400     Homing (executing an SH command)
			0x0800     Wait Time (executing a WT command)
			0x1000     Q Program is running
			0x2000     Initializing (happens at power up)
			0x4000     not used
			0x8000     not used
			0x
		 */
		union MOONS_SS_DRIVE_STATUS
		{
			uint16_t	sc_status;
			struct
			{
				unsigned	Motor_Enabled : 1;			 // = 0x00000001;
				unsigned	Sampling : 1;			       // = 0x00000002;
				unsigned	Drive_Fault : 1;			   // = 0x00000004;
				unsigned	In_Position : 1;	       // = 0x00000008;
				unsigned	Moving : 1;	             // = 0x00000010;
				unsigned	Jogging : 1;		         // = 0x00000020;
				unsigned	Stopping : 1;			       // = 0x00000040;
				unsigned	Waiting_for_input : 1;	 // = 0x00000080;
				unsigned	Saving : 1;	             // = 0x00000100;
				unsigned	Alarm_present : 1;		   // = 0x00000200;
				unsigned	Homing : 1;		           // = 0x00000400;
				unsigned	WaitTime : 1;		         // = 0x00000800;
				unsigned	QProgram_is_running : 1; // = 0x00001000;
				unsigned	Initializing : 1;			   // = 0x00002000;
				unsigned	not_used0 : 1;		       // = 0x00004000;
				unsigned	not_used1 : 1;			     // = 0x00008000;
			};
		};




	public:

		struct moons_data_t {

			MOONS_SS_ALARM_STATUS al_code{};
			MOONS_SS_DRIVE_STATUS drv_status{};
			uint16_t immediate_expanded_input{};
			uint16_t driver_board_inputs{};
			uint32_t encoder_position{};
			int immediate_abs_position{};
			uint32_t abs_position_command{};
			uint16_t immediate_act_velocity{};
			uint16_t immediate_target_velocity{};

			moons_data_t() = default;

			// copy constructor
			moons_data_t(const moons_data_t& rhs) = default;
			// copy assignment operator
			moons_data_t& operator=(const moons_data_t& rhs) = default;
			// move constructor
			moons_data_t(moons_data_t&& rhs) = default;
			// move assignment operator
			moons_data_t& operator=(moons_data_t&& rhs) = default;

		};


		/*
		 * modbus ������� ������ �����ʹ� �Ʒ��� ���� ó���Ѵ�
		 * �����ӵ��� ��ǰ ������ (rps*s)�� 6���
		 * �ӵ��� ��ǰ ������(rps)�� 240���.
		 * rpm  4���
		 */
		struct motion_param_t {
			uint32_t jog_speed{};
			uint32_t jog_accel{};
			uint32_t jog_decel{};

			uint32_t move_speed{};
			uint32_t move_accel{};
			uint32_t move_decel{};

			motion_param_t() = default;

			void Init() {
				constexpr uint32_t default_rpm = 60;
				constexpr uint32_t default_rpss = 100;
				jog_speed = default_rpm; //(uint32_t)(default_rpm * MODBUS_MULTIPLE_PARAM_VEL);
				jog_accel = default_rpss; //(uint32_t)(default_rpss * MODBUS_MULTIPLE_PARAM_ACC);
				jog_decel = jog_accel;

				move_speed = jog_speed;
				move_accel = jog_accel;
				move_decel = jog_accel;
			}

			// copy constructor
			motion_param_t(const motion_param_t& rhs) = default;
			// copy assignment operator
			motion_param_t& operator=(const motion_param_t& rhs) = default;
			// move constructor
			motion_param_t(motion_param_t&& rhs) = default;
			// move assignment operator
			motion_param_t& operator=(motion_param_t&& rhs) = default;
		};

		struct origin_param_t {
			uint16_t accel{}; //rpss
			uint16_t decel{}; //rpss
			uint16_t speed{}; //rpm
			uint32_t offset{}; //pulse cnt
			char home_x_no{}; // '1' ~ '5'
			int find_home_dir{}; // 1 CW, -1 CCW
			char home_x_level{}; //'L' Low, 'H' High, 'F' fall edge signal, 'R' rising edge signal

			origin_param_t() {
				Init();
			}

			void Init() {
				accel = 100;
				decel = 100;
				speed = 50;
				offset = 10000;
				home_x_no = '5'; // home sens
				home_x_level = 'F';
				find_home_dir = -1;
			}

			// copy constructor
			origin_param_t(const origin_param_t& rhs) = default;
			// copy assignment operator
			origin_param_t& operator=(const origin_param_t& rhs) = default;
			// move constructor
			origin_param_t(origin_param_t&& rhs) = default;
			// move assignment operator
			origin_param_t& operator=(origin_param_t&& rhs) = default;

		};


		struct cfg_t {
			origin_param_t origin_param{};
			motion_param_t motor_param{};
			AP_OBJ::MOTOR  instance_no{};
			//axis_dat* p_apAxisDat{};
			MCU_REG::ap_reg* ptr_ap_reg{};
			//ap_dat* p_apCfgDat{};
			uart_moons* p_comm{};

			cfg_t() = default;

			// copy constructor
			cfg_t(const cfg_t& rhs) = default;
			// copy assignment operator
			cfg_t& operator=(const cfg_t& rhs) = default;
			// move constructor
			cfg_t(cfg_t&& rhs) = default;
			// move assignment operator
			cfg_t& operator=(cfg_t&& rhs) = default;
	

		};

	public:
		cfg_t m_cfg;


	private:
		uint8_t m_nodeId;
		uart_moons::rx_packet_t m_receiveData;
		moons_data_t m_motorData;
		uint16_t m_commErrCnt;
		int m_cmdDistPulse;
		uint16_t m_alarmCnt;
#ifdef _USE_HW_RTOS
		osMutexId m_mutex_id;
#endif

		/****************************************************
		 *  Constructor
		 ****************************************************/
	public:
		enMotor_moons(uint8_t id) : m_cfg{}, m_nodeId(id), m_receiveData()
			, m_motorData{}, m_commErrCnt{}, m_cmdDistPulse{}, m_alarmCnt{}
		{
#ifdef _USE_HW_RTOS
			osMutexDef(m_mutex_id);
			m_mutex_id = osMutexCreate(osMutex(m_mutex_id));
#endif

		}
		virtual ~enMotor_moons() {}

		/****************************************************
		 *  func
		 ****************************************************/

		inline void Init(cfg_t& cfg) {
			m_cfg = cfg;
			m_cfg.p_comm->AttCallbackFunc(m_cfg.instance_no, this, receiveDataFunc);
			logPrintf(">>enMotor_moons Init Success! instance[%d] \n", (uint8_t)m_cfg.instance_no);

		}

		inline uart_moons::rx_packet_t* GetPacketData() {
			return &m_receiveData;
		}

		inline void GetMotionParamData(motion_param_t& ret_data) const {
			ret_data = m_cfg.motor_param;
		}

		inline void GetOriginParamData(origin_param_t& ret_data) const {
			ret_data = m_cfg.origin_param;
		}

		inline void GetMotorData(moons_data_t& ret_data) {
			ret_data = m_motorData;
		}

		/*
		inline uint8_t GetInstanceNo() {
			return m_cfg.instance_no;
		}

		inline uint8_t GetErrCnt() const {
			return m_commErrCnt;
		}

		inline uint8_t AddErrCnt() {
			m_cfg.p_comm->ResetCommFlag();
			return ++m_commErrCnt;
		}*/

	private:

		void insertData(uart_moons::rx_packet_t& data) {
			m_receiveData = data;
			uint8_t* ptr_data = m_receiveData.data;
			enum func_e {
				read_HoldingReg = 0x03,
				read_InputReg = 0x04,
				write_SingleReg = 0x06,
				write_MultiReg = 0x10
			};

			switch (m_receiveData.func_type) {
			case read_HoldingReg:
			{
				if (m_receiveData.data_length == 24)
				{
					m_motorData.al_code.al_status = (uint16_t)(ptr_data[0] << 8) | (uint16_t)(ptr_data[1] << 0);
					m_motorData.drv_status.sc_status = (uint16_t)(ptr_data[2] << 8) | (uint16_t)(ptr_data[3] << 0);
					m_motorData.immediate_expanded_input = (uint16_t)(ptr_data[4] << 8) | (uint16_t)(ptr_data[5] << 0);
					m_motorData.driver_board_inputs = (uint16_t)(ptr_data[6] << 8) | (uint16_t)(ptr_data[7] << 0);
					m_motorData.encoder_position = (uint32_t)(ptr_data[8] << 24) | (uint32_t)(ptr_data[9] << 16) | (uint32_t)(ptr_data[10] << 8) | (uint32_t)(ptr_data[11] << 0);
					m_motorData.immediate_abs_position = int((uint32_t)(ptr_data[12] << 24) | (uint32_t)(ptr_data[13] << 16) | (uint32_t)(ptr_data[14] << 8) | (uint32_t)(ptr_data[15] << 0));
					m_motorData.abs_position_command = (uint32_t)(ptr_data[16] << 24) | (uint32_t)(ptr_data[17] << 16) | (uint32_t)(ptr_data[18] << 8) | (uint32_t)(ptr_data[19] << 0);
					m_motorData.immediate_act_velocity = (uint16_t)(ptr_data[20] << 8) | (uint16_t)(ptr_data[21] << 0);
					m_motorData.immediate_target_velocity = (uint16_t)(ptr_data[22] << 8) | (uint16_t)(ptr_data[23] << 0);
				}
			}
			break;
			case read_InputReg:
			{

			}
			break;
			case write_SingleReg:
			{

			}
			break;
			case write_MultiReg:
			{

			}
			break;
			default:
				break;
			}


		}

		// callback function
		static void receiveDataFunc(void* obj, void* w_parm, void* l_parm) {
			enMotor_moons* ptr_this = (enMotor_moons*)obj;
			if (w_parm == nullptr && obj == nullptr && l_parm == nullptr)
				return;
			//uint8_t instance_no = *((uint8_t*)w_parm);
			ptr_this->insertData(*((uart_moons::rx_packet_t*)l_parm));
		}

	public:
		inline bool IsMotorOn() {
			return m_motorData.drv_status.Motor_Enabled;
		}

		inline bool IsMotorRun() {
			return m_motorData.drv_status.Moving;
		}

		inline bool IsMotorHoming() {
			return m_motorData.drv_status.Homing;
		}

		inline bool IsMotorStop() {
			return !IsMotorRun()/*m_motorData.drv_status.Stopping*/;
		}

		inline bool IsNoErrStopAndZero() {
			volatile bool ret = true;
			ret &= m_motorData.encoder_position < 100;
			ret &= !IsAlarmState(); //not
			ret &= IsMotorStop();
			return ret;

		}

		inline errno_t MotorOnOff(bool on_off = true) {
			return m_cfg.p_comm->MotorOnOff(m_nodeId, on_off);
		}

		inline errno_t SetParamDataMove(uint32_t rpm, uint32_t acc, uint32_t dec) {
			m_cfg.motor_param.move_accel = acc;
			m_cfg.motor_param.move_decel = dec;
			m_cfg.motor_param.move_speed = rpm;
			uart_moons::speed_t paramsC {
				(uint16_t)(acc* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(dec* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(rpm* MODBUS_MULTIPLE_PARAM_VEL) };

			return m_cfg.p_comm->SetMoveParam(m_nodeId, paramsC);
		}


		inline errno_t SetMoveDistSpeed(uint32_t rpm, uint32_t acc, uint32_t dec, int dist_pulse) {
			m_cfg.motor_param.move_accel = acc;
			m_cfg.motor_param.move_decel = dec;
			m_cfg.motor_param.move_speed = rpm;
			uart_moons::speed_t paramsC {
				(uint16_t)(acc* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(dec* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(rpm* MODBUS_MULTIPLE_PARAM_VEL) };

			return m_cfg.p_comm->SetMove(m_nodeId, paramsC, dist_pulse);
		}

#if 0
		inline errno_t SetMoveDistSpeed(float rpm_f, uint32_t acc, uint32_t dec, int dist_pulse) {
			uart_moons::speed_t paramsC {
				(uint16_t)(acc* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(dec* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)((uint32_t)(rpm_f* (float)MODBUS_MULTIPLE_PARAM_VEL)) };

			return m_cfg.p_comm->SetMove(m_nodeId, paramsC, dist_pulse);
		}
#endif


		inline errno_t JogMove(uart_moons::speed_t& param, bool is_cw = true) {
			m_cfg.motor_param.jog_accel = param.accel;
			m_cfg.motor_param.jog_decel = param.decel;
			m_cfg.motor_param.jog_speed = param.speed;

			param.accel *= MODBUS_MULTIPLE_PARAM_ACC;
			param.decel *= MODBUS_MULTIPLE_PARAM_ACC;
			param.speed *= MODBUS_MULTIPLE_PARAM_VEL;
			return m_cfg.p_comm->JogMove(m_nodeId, param, is_cw);
		}



		inline errno_t JogMove(bool is_cw = true) {
			uart_moons::speed_t paramsC {
				(uint16_t)(m_cfg.motor_param.jog_accel* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(m_cfg.motor_param.jog_decel* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(m_cfg.motor_param.jog_speed* MODBUS_MULTIPLE_PARAM_VEL) };
			return m_cfg.p_comm->JogMove(m_nodeId, paramsC, is_cw);
		}

		inline errno_t JogStop() {
			return m_cfg.p_comm->JogStop(m_nodeId);
		}

		inline errno_t MoveStop() {
			return m_cfg.p_comm->MoveStop(m_nodeId);
		}

		inline errno_t SetOutput(uint16_t pin_no, bool is_on = true) {
			return m_cfg.p_comm->SetOutput(m_nodeId, pin_no, is_on);
		}

		inline errno_t MoveRelative(int dist_pulse, uint32_t rpm, uint32_t acc, uint32_t dec) {
			errno_t ret = ERROR_SUCCESS;
			m_cmdDistPulse = m_motorData.immediate_abs_position + dist_pulse;

			ret = SetMoveDistSpeed(rpm, acc, dec, dist_pulse);
			if (ret == ERROR_SUCCESS)
				return m_cfg.p_comm->MoveRelactive(m_nodeId);
			else
				return ret;
		}

		inline errno_t MoveRelative(int dist_pulse, float rpm_f, uint32_t acc, uint32_t dec) {
			errno_t ret = ERROR_SUCCESS;
			m_cmdDistPulse = m_motorData.immediate_abs_position + dist_pulse;
			ret = SetMoveDistSpeed(rpm_f, acc, dec, dist_pulse);
			if (ret == ERROR_SUCCESS)
				return m_cfg.p_comm->MoveRelactive(m_nodeId);
			else
				return ret;
		}

		inline errno_t MoveRelative(int dist_pulse) {
			errno_t ret = ERROR_SUCCESS;
			m_cmdDistPulse = m_motorData.immediate_abs_position + dist_pulse;
			ret = m_cfg.p_comm->DistancePoint(m_nodeId, dist_pulse);
			if (ret == ERROR_SUCCESS)
				return m_cfg.p_comm->MoveRelactive(m_nodeId);
			else
				return ret;
		}

		inline errno_t MoveAbsolutive(int dist_pulse, uint32_t rpm, uint32_t acc, uint32_t dec) {
			errno_t ret = ERROR_SUCCESS;
			m_cmdDistPulse = dist_pulse;

			ret = SetMoveDistSpeed(rpm, acc, dec, dist_pulse);
			if (ret == ERROR_SUCCESS)
				return m_cfg.p_comm->moveAbsolutive(m_nodeId);
			else
				return ret;
		}

		inline errno_t MoveAbsolutive(int dist_pulse) {
			errno_t ret = ERROR_SUCCESS;
			m_cmdDistPulse = dist_pulse;
			ret = m_cfg.p_comm->DistancePoint(m_nodeId, dist_pulse);
			if (ret == ERROR_SUCCESS)
				return m_cfg.p_comm->moveAbsolutive(m_nodeId);
			else
				return ret;
		}

		inline errno_t MoveToOffset() {
			errno_t ret = ERROR_SUCCESS;
			m_cmdDistPulse = m_cfg.origin_param.offset;
			ret = MoveAbsolutive(m_cmdDistPulse, m_cfg.motor_param.move_speed, m_cfg.motor_param.move_accel, m_cfg.motor_param.move_decel);

			if (ret == ERROR_SUCCESS)
				return m_cfg.p_comm->moveAbsolutive(m_nodeId);
			else
				return ret;
		}

		inline void Recovery() {
			m_commErrCnt = 0;
		}


		inline bool IsCommAlarm() const {
			return m_motorData.al_code.Comm_Error;
		}

		inline bool IsAlarmState() {
			if (m_motorData.drv_status.Alarm_present)
				this->m_alarmCnt++;
			else
				this->m_alarmCnt = 0;
			constexpr uint16_t limit_alarm_cnt = 1000;
			return (this->m_alarmCnt > limit_alarm_cnt ? true : false);
		}

		inline errno_t GetMotorData() {
			return m_cfg.p_comm->RequestMotorData(m_nodeId);
		}

		inline errno_t SetOriginParam(origin_param_t& param) {
			m_cfg.origin_param = param;
			uart_moons::speed_t paramsC {
				(uint16_t)(param.accel* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(param.decel* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(param.speed* MODBUS_MULTIPLE_PARAM_VEL)};

			return  m_cfg.p_comm->SetOriginParam(m_nodeId, paramsC, param.find_home_dir);
		}


		inline errno_t OriginMotor() {
			errno_t ret = ERROR_SUCCESS;
			uart_moons::speed_t paramsC {
				(uint16_t)(m_cfg.origin_param.accel* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(m_cfg.origin_param.decel* MODBUS_MULTIPLE_PARAM_ACC),
					(uint16_t)(m_cfg.origin_param.speed* MODBUS_MULTIPLE_PARAM_VEL)};
			ret = m_cfg.p_comm->SetOriginParam(m_nodeId, paramsC, m_cfg.origin_param.find_home_dir);
			//logPrintf("enMotor_moons OriginMotor  m_cfg.origin_param.find_home_dir(%d)   \n", m_cfg.origin_param.find_home_dir);
			if (ret == ERROR_SUCCESS)
				return  m_cfg.p_comm->OriginAxis(m_nodeId, m_cfg.origin_param.home_x_no, m_cfg.origin_param.home_x_level);
			return ret;
		}


		inline errno_t ClearState() {
			return m_cfg.p_comm->ClearState(m_nodeId);
		}

		inline errno_t ClearEncoder() {
			return m_cfg.p_comm->ClearEncoder(m_nodeId);
		}

		inline bool IsInPose() {
			constexpr int error_tolerance = 10;
			return (abs((int)(m_motorData.immediate_abs_position) - m_cmdDistPulse) < error_tolerance);
		}

		inline bool IsInPosition(int pos) {
			constexpr int error_tolerance = 10;
			return (abs((int)(m_motorData.immediate_abs_position) - pos) < error_tolerance);
		}

	}; // end of class



} // end of namespace



#endif /* AP__INC_ENMOTOR_MOONS_HPP_ */
