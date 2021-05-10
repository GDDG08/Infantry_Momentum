/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : cha_referee_ctrl.c
 *  Description  : �¿���д��~
 *  LastEditors  : ����ؼ���ᶯ��
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 07:55:42
 */


#include "cha_referee_ctrl.h"
#include "referee_periph.h"

#if __FN_IF_ENABLE(__FN_CTRL_REFEREE)
/********** Drawing Constants **********/

// ����ͼ�㣺ͼ��0 ~ 9����ͼ���ڸǵ�ͼ��
// ���ھ������µķ�ͼ�㹦�ܣ�����ǰ��ͼ��ʹ��3������ͼ��ʹ��2
// ���������ڲ������ڵ�������½���ʹ��ͼ��2

const uint8_t AIM_LINE_LAYER        = 2;
const Draw_Color AIM_LINE_COLOR     = Draw_COLOR_GREEN;
const uint8_t AIM_LINE_LINE_NUM     = 3 + 1;
const uint16_t AIM_LINES[AIM_LINE_LINE_NUM][6] = {  // ID, Width, X1, Y1, X2, Y2
    {0x101, 2, 0, 0, 0, 0},     // Vertical Line
    {0x102, 4, 0, 0, 0, 0},     // Horizontal Line 1
    {0x103, 2, 0, 0, 0, 0},     // Horizontal Line 2
    {0x104, 2, 0, 0, 0, 0}      // Horizontal Line 3
};

const uint8_t CROSSHAIR_LAYER       = 2;
const Draw_Color CROSSHAIR_COLOR    = Draw_COLOR_GREEN;
const uint16_t CROSSHAIR[5]         = {0x201, 2, 0, 0, 0};  // ID, Width, X, Y, R

const uint8_t WIDTH_MARK_LAYER      = 2;
const Draw_Color WIDTH_MARK_COLOR   = Draw_COLOR_YELLOW;
const uint16_t WIDTH_MARK_NORMAL[2][6] = {
    {0x301, 2, 0, 0, 0, 0},     // Left Mark Line, Normal
    {0x302, 2, 0, 0, 0, 0},     // Right Mark Line, Normal
};
const uint16_t WIDTH_MARK_GYRO[2][6] = {
    {0x301, 2, 0, 0, 0, 0},     // Left Mark Line, Gyro Mode
    {0x302, 2, 0, 0, 0, 0},     // Right Mark Line, Gyro Mode
};

const uint8_t CAP_STATE_LAYER[2]    = {3, 2};   // Foreground, Background
const Draw_Color CAP_STATE_COLOR[5] = {
    Draw_COLOR_WHITE,           // Background
    Draw_COLOR_GREEN,           // Text
    Draw_COLOR_GREEN,           // Foreground, Full (50% ~ 100%)
    Draw_COLOR_YELLOW,          // Foreground, Insufficient (10% ~ 50%)
    Draw_COLOR_ORANGE           // Foreground, Empty (0% ~ 10%)
};
const uint16_t CAP_STATE[4]         = {2, 0, 0, 0};     // Width, X, Y, R
const uint16_t CAP_STATE_CIRCLE     = 0x401;            // Background Circle ID
const uint16_t CAP_STATE_ARC        = 0x402;            // Foreground Arc ID
const uint16_t CAP_STATE_TEXT[5]    = {0x403, 20, 2, 0, 0};     // ID, Font Size, Width, X, Y
const char *CAP_STATE_TEXT_STR      = "CAP";

const uint8_t PITCH_METER_LAYER     = 2;
const Draw_Color PITCH_METER_COLOR  = Draw_COLOR_GREEN;
const uint16_t PITCH_METER_TEXT[5]  = {0x501, 20, 2, 0, 0};     // ID, Font Size, Width, X, Y
const char *PITCH_METER_TEXT_STR    = "PITCH:";
const uint16_t PITCH_METER_VALUE[6] = {0x502, 20, 1, 2, 0, 0};  // ID, Font Size, Precision, Width, X, Y



/********** END OF Drawing Constants **********/


DrawCtrl_DrawCtrlDataTypeDef DrawCtrl_drawCtrlData;


/**
  * @brief      ���ó�����ģʽ
  * @param      mode: ������ģʽ��1ΪС���ݣ�0Ϊ��ͨ��
  * @retval     ��
  */
void DrawCtrl_SetWidthMode(uint8_t mode) {
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    drawctrl->width_mode = mode;
}


/**
  * @brief      ���õ��ݵ���
  * @param      state: ���ݵ�����0 ~ 100����λ�ٷֱȣ�
  * @retval     ��
  */
void DrawCtrl_SetCapState(uint8_t state) {
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    drawctrl->cap_state = state;
}


/**
  * @brief      ����Pitch���
  * @param      angle: Pitch���
  * @retval     ��
  */
void DrawCtrl_SetPitchAngle(float angle) {
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    drawctrl->pitch_angle = angle;
}


/**
  * @brief      ��׼�߻��ƣ���ʼ���׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupAimLine() {
    // draw_cnt: 4
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(AIM_LINES[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, AIM_LINES[i][1], AIM_LINES[i][2], AIM_LINES[i][3], AIM_LINES[i][4], AIM_LINES[i][5]);
    }
}


/**
  * @brief      ��׼�߻��ƣ����½׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_UpdateAimLine() {
    // nothing
}


/**
  * @brief      ׼�Ļ��ƣ���ʼ���׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupCrosshair() {
    // draw_cnt: 1
    Draw_AddCircle(CROSSHAIR[0], CROSSHAIR_LAYER, CROSSHAIR_COLOR, CROSSHAIR[1], CROSSHAIR[2], CROSSHAIR[3], CROSSHAIR[4]);
}


/**
  * @brief      ׼�Ļ��ƣ����½׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_UpdateCrosshair() {
    // nothing
}


/**
  * @brief      �����߻��ƣ���ʼ���׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupWidthMark() {
    // draw_cnt: 2
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    drawctrl->width_mode_last = drawctrl->width_mode;
    const uint16_t (*mark)[6] = (drawctrl->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_AddLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}


/**
  * @brief      �����߻��ƣ����½׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_UpdateWidthMark() {
    // draw_cnt: 2 when mode changed, 0 when mode not change
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    if (drawctrl->width_mode_last == drawctrl->width_mode) return;
    drawctrl->width_mode_last = drawctrl->width_mode;
    const uint16_t (*mark)[6] = (drawctrl->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_ModifyLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}


/**
  * @brief      ����״̬���ƣ���ʼ���׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupCapState() {
    // draw_cnt: 2
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    
    Draw_AddCircle(CAP_STATE_CIRCLE, CAP_STATE_LAYER[1], CAP_STATE_COLOR[0], CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3]);
    
    Draw_Color color;
    if (drawctrl->cap_state > 100)
        return;
    else if (drawctrl->cap_state >= 50) 
        color = CAP_STATE_COLOR[2];
    else if (drawctrl->cap_state >= 20)
        color = CAP_STATE_COLOR[3];
    else 
        color = CAP_STATE_COLOR[4];

    uint16_t start_angle = 0;
    uint16_t end_angle = 0;
    if (drawctrl->cap_state > 0 && drawctrl->cap_state <= 100)
        end_angle = (uint16_t) (360.0 * drawctrl->cap_state / 100.0);
    
    Draw_AddArc(CAP_STATE_ARC, CAP_STATE_LAYER[0], color, start_angle, end_angle, CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3], CAP_STATE[3]);
}


/**
  * @brief      ����״̬���ƣ����½׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_UpdateCapState() {
    // draw_cnt: 1
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;

    Draw_Color color;
    if (drawctrl->cap_state > 100)
        return;
    else if (drawctrl->cap_state >= 50) 
        color = CAP_STATE_COLOR[2];
    else if (drawctrl->cap_state >= 20)
        color = CAP_STATE_COLOR[3];
    else
        color = CAP_STATE_COLOR[4];
    
    uint16_t start_angle = 0;
    uint16_t end_angle = 0;
    if (drawctrl->cap_state > 0 && drawctrl->cap_state <= 100)
        end_angle = (uint16_t) (360.0 * drawctrl->cap_state / 100.0);
    
    Draw_ModifyArc(CAP_STATE_ARC, CAP_STATE_LAYER[0], color, start_angle, end_angle, CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3], CAP_STATE[3]);
}


/**
  * @brief      Pitch��Ǽƻ��ƣ���ʼ���׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupPitchMeter() {
    // draw_cnt: 1
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    float value = drawctrl->pitch_angle;
    Draw_AddFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
}


/**
  * @brief      Pitch��Ǽƻ��ƣ����½׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_UpdatePitchMeter() {
    // draw_cnt: 1
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    uint16_t value = (uint16_t) (drawctrl->pitch_angle);
    Draw_ModifyFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
}


/**
  * @brief      ģʽ��ʾ���ƣ���ʼ���׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupModeDisplay() {
    
}


/**
  * @brief      ģʽ��ʾ���ƣ����½׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_UpdateModeDisplay() {
    
}


/**
  * @brief      ������ʾ���ƣ���ʼ���׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupErrorDisplay() {
    
}


/**
  * @brief      ������ʾ���ƣ����½׶�
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_UpdateErrorDisplay() {
    
}


/**
  * @brief      �����ܳ�ʼ���׶����ֻ���
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_SetupAllString() {
    // cmd_cnt: 2
    //DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    
    Draw_AddString(CAP_STATE_TEXT[0], CAP_STATE_LAYER[1], CAP_STATE_COLOR[1], CAP_STATE_TEXT[1], CAP_STATE_TEXT[2], CAP_STATE_TEXT[3], CAP_STATE_TEXT[4], CAP_STATE_TEXT_STR);
    Draw_AddString(PITCH_METER_TEXT[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_TEXT[1], PITCH_METER_TEXT[2], PITCH_METER_TEXT[3], PITCH_METER_TEXT[4], PITCH_METER_TEXT_STR);
    
}


/**
  * @brief      ��ʼ�������ƹ���
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_Setup() {                 
    Draw_ClearAll();                    // cmd_cnt: 1, total_cmd_cnt: 1
    
    DrawCtrl_SetupAimLine();            // draw_cnt: 4
    DrawCtrl_SetupCrosshair();          // draw_cnt: 1
    DrawCtrl_SetupWidthMark();          // draw_cnt: 2, send(7), total_cmd_cnt: 2
    DrawCtrl_SetupCapState();           // draw_cnt: 2
    DrawCtrl_SetupPitchMeter();         // draw_cnt: 1
    DrawCtrl_SetupModeDisplay();        // draw_cnt: 0
    DrawCtrl_SetupErrorDisplay();       // draw_cnt: 0, send(3+2), total_cmd_cnt: 3
    
    DrawCtrl_SetupAllString();          // cmd_cnt: 2, total_cmd_cnt: 5
    
    Referee_DrawingBufferFlush();       // useless since string cmd sent previously
}


/**
  * @brief      ���¸����ƹ���
  * @param      ��
  * @retval     ��
  */
void DrawCtrl_Update() {                
    DrawCtrl_UpdateAimLine();           // draw_cnt: 0
    DrawCtrl_UpdateCrosshair();         // draw_cnt: 0
    DrawCtrl_UpdateWidthMark();         // draw_cnt: if gyro mode changed 2, else 0
    DrawCtrl_UpdateCapState();          // draw_cnt: 1
    DrawCtrl_UpdatePitchMeter();        // draw_cnt: 1
    DrawCtrl_UpdateModeDisplay();       // draw_cnt: 0
    DrawCtrl_UpdateErrorDisplay();      // draw_cnt: 0
    
    Referee_DrawingBufferFlush();       // if gyro mode changed, send(4+1), total_cmd_cnt: 1
                                        // else, send(2), total_cmd_cnt: 1
}

#endif
