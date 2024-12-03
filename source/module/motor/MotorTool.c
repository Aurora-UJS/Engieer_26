#include "MotorTool.h"
#include "CANtool.h"

//yaw电机启动确认
uint8_t can5FIFO_flag = 1;
uint8_t Data_Enable[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };		//达妙电机使能命令
uint8_t Data_Failure[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD };		//电机失能命令
uint8_t Data_Save_zero[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE };	    //电机保存零点命令
// 电机数据
moto_info_t moto_data[16];

/**
 * @brief RM系列电机驱动
 *
 * @param hcan CAN端口
 * @param ID 电机ID
 * @param v1 电机1电流
 * @param v2 电机2电流
 * @param v3 电机3电流
 * @param v4 电机4电流
 */
void can_output(FDCAN_HandleTypeDef *hcan, uint16_t ID, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
    uint8_t TxDate[8] = {0}; // 放数值的数组
    TxDate[0] = v1 >> 8;
    TxDate[1] = v1;
    TxDate[2] = v2 >> 8;
    TxDate[3] = v2;
    TxDate[4] = v3 >> 8;
    TxDate[5] = v3;
    TxDate[6] = v4 >> 8;
    TxDate[7] = v4; // 1到4的电机的值
    fdcanx_send_data(hcan, ID, TxDate, FDCAN_DLC_BYTES_8);
}

/**
 * @brief  达妙电机位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
void PosSpeed_CtrlMotor(FDCAN_HandleTypeDef *hcan, uint16_t ID, float _pos, float _vel)
{
    uint8_t TxDate[8] = {0}; // 放数值的数组
    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;

    TxDate[0] = *pbuf;
    TxDate[1] = *(pbuf + 1);
    TxDate[2] = *(pbuf + 2);
    TxDate[3] = *(pbuf + 3);
    TxDate[4] = *vbuf;
    TxDate[5] = *(vbuf + 1);
    TxDate[6] = *(vbuf + 2);
    TxDate[7] = *(vbuf + 3);

    fdcanx_send_data(hcan, ID, TxDate, FDCAN_DLC_BYTES_8);
}

/**
 * @brief  速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */
void Speed_CtrlMotor(FDCAN_HandleTypeDef *hcan, uint16_t ID, float _vel)
{
  uint8_t TxDate[8] = { 0 };   							    //放数值的数组
  uint8_t *vbuf;
  vbuf = (uint8_t *)&_vel;

  TxDate[0] = *vbuf;
  TxDate[1] = *(vbuf + 1);
  TxDate[2] = *(vbuf + 2);
  TxDate[3] = *(vbuf + 3);

  //找到空的发送邮箱，把数据发送出去
    fdcanx_send_data(hcan, ID, TxDate, FDCAN_DLC_BYTES_4);
}

/**
 * @brief RM系列电机接收
 * 
 * @param ID 电机ID
 * @param rx_data 电机数据
 */
void can_input_RM( uint16_t ID, uint8_t *rx_data)
{
      uint8_t index = ID - FEEDBACK_ID_BASE;            // get motor index by can_id    通过can_id获取运动指标		
      moto_data[index].rotor_angle = ((rx_data[0] << 8) | rx_data[1]);
      moto_data[index].rotor_speed = ((rx_data[2] << 8) | rx_data[3]);
      moto_data[index].set_voltage = ((rx_data[4] << 8) | rx_data[5]);
      moto_data[index].temp = rx_data[6];
}

/**
 * @brief DM系列电机接收
 * 
 * @param ID 电机ID
 * @param rx_data 电机数据
 */
void can_input_dm( uint16_t ID, uint8_t *rx_data)
{
    moto_data[ID].can_id = rx_data[0];
    moto_data[ID].rotor_angle = (rx_data[1] << 8) | rx_data[2];
    moto_data[ID].rotor_speed = (rx_data[3] << 4) | (rx_data[4] >> 4);
    moto_data[ID].torque_current = (rx_data[4] << 4) | rx_data[5];
    moto_data[ID].temp = rx_data[6] > rx_data[7] ? rx_data[6] : rx_data[7];//取高温
}