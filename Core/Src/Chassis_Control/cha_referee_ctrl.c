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

#if __FN_IF_ENABLE(__FN_CTRL_CHASSIS)

#include "referee_periph.h"

/********** Drawing Constants **********/

// ����ͼ�㣺ͼ��0 ~ 9����ͼ���ڸǵ�ͼ��
// ���ھ������µķ�ͼ�㹦�ܣ�����ǰ��ͼ��ʹ��3������ͼ��ʹ��2
// ���������ڲ������ڵ�������½���ʹ��ͼ��2

// �������꣺���½�Ϊ (0, 0)��ˮƽ����Ϊ X����ֱ����Ϊ Y

const uint8_t AIM_LINE_LAYER = 2;
const Draw_Color AIM_LINE_COLOR = Draw_COLOR_GREEN;
const uint8_t AIM_LINE_LINE_MODE = 3;
const uint8_t AIM_LINE_LINE_NUM = 3 + 1;
const uint16_t AIM_LINES[AIM_LINE_LINE_MODE][AIM_LINE_LINE_NUM][6] = {  // ID, Width, X1, Y1, X2, Y2
    {
        // Mode 0: 15 m/s
        {0x101, 2, 0, 0, 0, 0},  // Vertical Line
        {0x102, 4, 0, 0, 0, 0},  // Horizontal Line 1
        {0x103, 2, 0, 0, 0, 0},  // Horizontal Line 2
        {0x104, 2, 0, 0, 0, 0}   // Horizontal Line 3
    },
    {
        // Mode 1: 18 m/s
        {0x101, 2, 0, 0, 0, 0},  // Vertical Line
        {0x102, 4, 0, 0, 0, 0},  // Horiz0ontal Line 1
        {0x103, 2, 0, 0, 0, 0},  // Horizontal Line 2
        {0x104, 2, 0, 0, 0, 0}   // Horizontal Line 3
    },
    {
        // Mode 2: 18 m/s
        {0x101, 2, 0, 0, 0, 0},  // Vertical Line
        {0x102, 4, 0, 0, 0, 0},  // Horizontal Line 1
        {0x103, 2, 0, 0, 0, 0},  // Horizontal Line 2
        {0x104, 2, 0, 0, 0, 0}   // Horizontal Line 3
    }};

const uint8_t CROSSHAIR_LAYER = 2;
const Draw_Color CROSSHAIR_COLOR = Draw_COLOR_GREEN;
const uint16_t CROSSHAIR[5] = {0x201, 2, 960, 415, 10};  // ID, Width, X, Y, R

const uint8_t WIDTH_MARK_LAYER = 2;
const Draw_Color WIDTH_MARK_COLOR = Draw_COLOR_BLACK;
const uint16_t WIDTH_MARK_GYRO[2][6] = {
    // ID, Width, X0, Y0, X1, Y1
    {0x301, 5, 660, 400, 660, 200},    // Left Mark Line, Normal
    {0x302, 5, 1260, 400, 1260, 200},  // Right Mark Line, Normal
};
const uint16_t WIDTH_MARK_NORMAL[2][6] = {
    {0x301, 5, 660, 400, 960, 200},   // Left Mark Line, Gyro Mode
    {0x302, 5, 1260, 400, 960, 200},  // Right Mark Line, Gyro Mode
};

const uint8_t CAP_STATE_LAYER[2] = {3, 2};  // Foreground, Background
const Draw_Color CAP_STATE_COLOR[5] = {
    Draw_COLOR_WHITE,   // Background
    Draw_COLOR_GREEN,   // Text
    Draw_COLOR_ORANGE,  // Foreground, Full (50% ~ 100%)
    Draw_COLOR_ORANGE,  // Foreground, Insufficient (10% ~ 50%)
    Draw_COLOR_ORANGE   // Foreground, Empty (0% ~ 10%)
};
const uint16_t CAP_STATE[4] = {6, 960, 240, 40};              // Width, X, Y, R
const uint16_t CAP_STATE_CIRCLE = 0x401;                      // Background Circle ID
const uint16_t CAP_STATE_ARC = 0x402;                         // Foreground Arc ID
const uint16_t CAP_STATE_TEXT[5] = {0x403, 20, 2, 900, 240};  // ID, Font Size, Width, X, Y
const char* CAP_STATE_TEXT_STR = "CAP";

const uint8_t PITCH_METER_LAYER = 2;
const Draw_Color PITCH_METER_COLOR = Draw_COLOR_GREEN;
const uint16_t PITCH_METER_TEXT[5] = {0x501, 20, 2, 1500, 540};  // ID, Font Size, Width, X, Y
const char* PITCH_METER_TEXT_STR = "PITCH:";
const uint16_t PITCH_METER_VALUE[6] = {0x502, 20, 3, 2, 1600, 540};  // ID, Font Size, Precision, Width, X, Y

/********** END OF Drawing Constants **********/

Referee_DrawDataTypeDef Referee_DrawData;

/**
 * @brief      ���ó�����ģʽ
 * @param      mode: ������ģʽ��1ΪС���ݣ�0Ϊ��ͨ��
 * @retval     ��
 */
void Referee_SetWidthMode(uint8_t mode) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->width_mode = mode;
}

/**
 * @brief      ������׼��ģʽ
 * @param      mode: ��׼��ģʽ��0 ~ 2��Ӧ���� 15,18,30 m/s��
 * @retval     ��
 */
void Referee_SetAimMode(uint8_t mode) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    if (mode > 2)
        return;
    draw->aim_mode = mode;
}

/**
 * @brief      ���õ��ݵ���
 * @param      state: ���ݵ�����0 ~ 100����λ�ٷֱȣ�
 * @retval     ��
 */
void Referee_SetCapState(uint8_t state) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->cap_state = state;
}

/**
 * @brief      ����Pitch���
 * @param      angle: Pitch���
 * @retval     ��
 */
void Referee_SetPitchAngle(float angle) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->pitch_angle = angle;
}

/**
 * @brief      ��׼�߻��ƣ���ʼ���׶�
 * @param      ��
 * @retval     ��
 */
void Referee_SetupAimLine() {
    // draw_cnt: 4
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->aim_mode_last = draw->aim_mode;
    const uint16_t(*aim_lines)[6] = AIM_LINES[draw->aim_mode];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
}

/**
 * @brief      ��׼�߻��ƣ����½׶�
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateAimLine() {
    // draw_cnt: 4 when mode changed, 0 when mode not change
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    if (draw->aim_mode_last == draw->aim_mode)
        return;
    draw->aim_mode_last = draw->aim_mode;
    const uint16_t(*aim_lines)[6] = AIM_LINES[draw->aim_mode];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
}

/**
 * @brief      ׼�Ļ��ƣ���ʼ���׶�
 * @param      ��
 * @retval     ��
 */
void Referee_SetupCrosshair() {
    // draw_cnt: 1
    Draw_AddCircle(CROSSHAIR[0], CROSSHAIR_LAYER, CROSSHAIR_COLOR, CROSSHAIR[1], CROSSHAIR[2], CROSSHAIR[3], CROSSHAIR[4]);
}

/**
 * @brief      ׼�Ļ��ƣ����½׶�
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateCrosshair() {
    // nothing
}

/**
 * @brief      �����߻��ƣ���ʼ���׶�
 * @param      ��
 * @retval     ��
 */
void Referee_SetupWidthMark() {
    // draw_cnt: 2
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->width_mode_last = draw->width_mode;
    const uint16_t(*mark)[6] = (draw->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_AddLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}

/**
 * @brief      �����߻��ƣ����½׶�
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateWidthMark() {
    // draw_cnt: 2 when mode changed, 0 when mode not change
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    if (draw->width_mode_last == draw->width_mode)
        return;
    draw->width_mode_last = draw->width_mode;
    const uint16_t(*mark)[6] = (draw->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_ModifyLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}

/**
 * @brief      ����״̬���ƣ���ʼ���׶�
 * @param      ��
 * @retval     ��
 */
void Referee_SetupCapState() {
    // draw_cnt: 2
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;

    Draw_AddCircle(CAP_STATE_CIRCLE, CAP_STATE_LAYER[1], CAP_STATE_COLOR[0], CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3]);

    int value = draw->cap_state;
    value = (int)(-draw->pitch_angle + 10) * 2;

    Draw_Color color;
    if (value > 100)
        return;
    else if (value >= 50)
        color = CAP_STATE_COLOR[2];
    else if (value >= 20)
        color = CAP_STATE_COLOR[3];
    else
        color = CAP_STATE_COLOR[4];

    uint16_t start_angle = 0;
    uint16_t end_angle = 0;
    if (value > 0 && value <= 100)
        end_angle = (uint16_t)(360.0 * value / 100.0);

    Draw_AddArc(CAP_STATE_ARC, CAP_STATE_LAYER[0], color, start_angle, end_angle, CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3], CAP_STATE[3]);
}

/**
 * @brief      ����״̬���ƣ����½׶�
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateCapState() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;

    int value = draw->cap_state;
    //    value = (int) (-draw->pitch_angle + 10) * 2;

    Draw_Color color;
    if (value > 100)
        return;
    else if (value >= 50)
        color = CAP_STATE_COLOR[2];
    else if (value >= 20)
        color = CAP_STATE_COLOR[3];
    else
        color = CAP_STATE_COLOR[4];

    uint16_t start_angle = 0;
    uint16_t end_angle = 1;
    if (value > 0 && value <= 100)
        end_angle = (uint16_t)(360.0f * value / 100.0f);

    Draw_ModifyArc(CAP_STATE_ARC, CAP_STATE_LAYER[0], color, start_angle, end_angle, CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3], CAP_STATE[3]);
}

/**
 * @brief      Pitch��Ǽƻ��ƣ���ʼ���׶�
 * @param      ��
 * @retval     ��
 */
void Referee_SetupPitchMeter() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    float value = -draw->pitch_angle;
    Draw_AddFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
}

/**
 * @brief      Pitch��Ǽƻ��ƣ����½׶�
 * @param      ��
 * @retval     ��
 */
void Referee_UpdatePitchMeter() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    float value = -draw->pitch_angle;
    Draw_ModifyFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
    // Draw_ModifyInt(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], (int32_t) (value * 1000));
}

/**
 * @brief      ģʽ��ʾ���ƣ���ʼ���׶�
 * @param      ��
 * @retval     ��
 */
void Referee_SetupModeDisplay() {
}

/**
 * @brief      ģʽ��ʾ���ƣ����½׶�
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateModeDisplay() {
}

/**
 * @brief      ������ʾ���ƣ���ʼ���׶�
 * @param      ��
 * @retval     ��
 */
void Referee_SetupErrorDisplay() {
}

/**
 * @brief      ������ʾ���ƣ����½׶�
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateErrorDisplay() {
}

/**
 * @brief      �����ܳ�ʼ���׶����ֻ���
 * @param      ��
 * @retval     ��
 */
void Referee_SetupAllString() {
    // cmd_cnt: 2
    // Referee_RefereeDataTypeDef *Referee = &Referee_DrawData;

    Draw_AddString(CAP_STATE_TEXT[0], CAP_STATE_LAYER[1], CAP_STATE_COLOR[1], CAP_STATE_TEXT[1], CAP_STATE_TEXT[2], CAP_STATE_TEXT[3], CAP_STATE_TEXT[4], CAP_STATE_TEXT_STR);
    Draw_AddString(PITCH_METER_TEXT[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_TEXT[1], PITCH_METER_TEXT[2], PITCH_METER_TEXT[3], PITCH_METER_TEXT[4], PITCH_METER_TEXT_STR);
}

/**
 * @brief      ��ʼ�������ƹ���
 * @param      ��
 * @retval     ��
 */
void Referee_Setup() {
    static int last_time = -1000;
    int now = HAL_GetTick();
    if (now - last_time < 1000)
        return;
    last_time = now;

    Draw_ClearAll();  // cmd_cnt: 1, total_cmd_cnt: 1

    Referee_SetupAimLine();       // draw_cnt: 4
    Referee_SetupCrosshair();     // draw_cnt: 1
    Referee_SetupWidthMark();     // draw_cnt: 2, send(7), total_cmd_cnt: 2
    Referee_SetupCapState();      // draw_cnt: 2
    Referee_SetupPitchMeter();    // draw_cnt: 1
    Referee_SetupModeDisplay();   // draw_cnt: 0
    Referee_SetupErrorDisplay();  // draw_cnt: 0, send(2)send(1), total_cmd_cnt: 4

    Referee_SetupAllString();  // cmd_cnt: 2, total_cmd_cnt: 6

    Referee_DrawingBufferFlush();  // useless since string cmd sent previously
}

/**
 * @brief      ���¸����ƹ���
 * @param      ��
 * @retval     ��
 */
void Referee_Update() {
    Referee_UpdateAimLine();       // draw_cnt: if bullet speed changed 4, else 0
    Referee_UpdateCrosshair();     // draw_cnt: 0
    Referee_UpdateWidthMark();     // draw_cnt: if gyro mode changed 2, else 0
    Referee_UpdateCapState();      // draw_cnt: 1
    Referee_UpdatePitchMeter();    // draw_cnt: 1
    Referee_UpdateModeDisplay();   // draw_cnt: 0
    Referee_UpdateErrorDisplay();  // draw_cnt: 0

    Referee_DrawingBufferFlush();  // max draw_cnt: 8, cmd_cnt:2
                                   // min draw_cnt: 2, cmd_cnt:1
}

#endif
