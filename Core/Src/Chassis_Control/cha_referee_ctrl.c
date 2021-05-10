/*
 *  Project      : Infantry_Momentum
 * 
 *  file         : cha_referee_ctrl.c
 *  Description  : 陈康冰写的~
 *  LastEditors  : 动情丶卜灬动心
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2021-05-08 07:55:42
 */


#include "cha_referee_ctrl.h"
#include "referee_periph.h"

#if __FN_IF_ENABLE(__FN_CTRL_REFEREE)
/********** Drawing Constants **********/

// 关于图层：图层0 ~ 9，高图层遮盖低图层
// 对于经常更新的分图层功能，建议前景图层使用3，背景图层使用2
// 其他功能在不产生遮挡的情况下建议使用图层2

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
  * @brief      设置车宽线模式
  * @param      mode: 车宽线模式（1为小陀螺，0为普通）
  * @retval     无
  */
void DrawCtrl_SetWidthMode(uint8_t mode) {
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    drawctrl->width_mode = mode;
}


/**
  * @brief      设置电容电量
  * @param      state: 电容电量（0 ~ 100，单位百分比）
  * @retval     无
  */
void DrawCtrl_SetCapState(uint8_t state) {
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    drawctrl->cap_state = state;
}


/**
  * @brief      设置Pitch倾角
  * @param      angle: Pitch倾角
  * @retval     无
  */
void DrawCtrl_SetPitchAngle(float angle) {
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    drawctrl->pitch_angle = angle;
}


/**
  * @brief      瞄准线绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_SetupAimLine() {
    // draw_cnt: 4
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(AIM_LINES[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, AIM_LINES[i][1], AIM_LINES[i][2], AIM_LINES[i][3], AIM_LINES[i][4], AIM_LINES[i][5]);
    }
}


/**
  * @brief      瞄准线绘制：更新阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_UpdateAimLine() {
    // nothing
}


/**
  * @brief      准心绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_SetupCrosshair() {
    // draw_cnt: 1
    Draw_AddCircle(CROSSHAIR[0], CROSSHAIR_LAYER, CROSSHAIR_COLOR, CROSSHAIR[1], CROSSHAIR[2], CROSSHAIR[3], CROSSHAIR[4]);
}


/**
  * @brief      准心绘制：更新阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_UpdateCrosshair() {
    // nothing
}


/**
  * @brief      车宽线绘制：初始化阶段
  * @param      无
  * @retval     无
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
  * @brief      车宽线绘制：更新阶段
  * @param      无
  * @retval     无
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
  * @brief      电容状态绘制：初始化阶段
  * @param      无
  * @retval     无
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
  * @brief      电容状态绘制：更新阶段
  * @param      无
  * @retval     无
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
  * @brief      Pitch倾角计绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_SetupPitchMeter() {
    // draw_cnt: 1
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    float value = drawctrl->pitch_angle;
    Draw_AddFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
}


/**
  * @brief      Pitch倾角计绘制：更新阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_UpdatePitchMeter() {
    // draw_cnt: 1
    DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    uint16_t value = (uint16_t) (drawctrl->pitch_angle);
    Draw_ModifyFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
}


/**
  * @brief      模式显示绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_SetupModeDisplay() {
    
}


/**
  * @brief      模式显示绘制：更新阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_UpdateModeDisplay() {
    
}


/**
  * @brief      错误显示绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_SetupErrorDisplay() {
    
}


/**
  * @brief      错误显示绘制：更新阶段
  * @param      无
  * @retval     无
  */
void DrawCtrl_UpdateErrorDisplay() {
    
}


/**
  * @brief      各功能初始化阶段文字绘制
  * @param      无
  * @retval     无
  */
void DrawCtrl_SetupAllString() {
    // cmd_cnt: 2
    //DrawCtrl_DrawCtrlDataTypeDef *drawctrl = &DrawCtrl_drawCtrlData;
    
    Draw_AddString(CAP_STATE_TEXT[0], CAP_STATE_LAYER[1], CAP_STATE_COLOR[1], CAP_STATE_TEXT[1], CAP_STATE_TEXT[2], CAP_STATE_TEXT[3], CAP_STATE_TEXT[4], CAP_STATE_TEXT_STR);
    Draw_AddString(PITCH_METER_TEXT[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_TEXT[1], PITCH_METER_TEXT[2], PITCH_METER_TEXT[3], PITCH_METER_TEXT[4], PITCH_METER_TEXT_STR);
    
}


/**
  * @brief      初始化各绘制功能
  * @param      无
  * @retval     无
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
  * @brief      更新各绘制功能
  * @param      无
  * @retval     无
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
