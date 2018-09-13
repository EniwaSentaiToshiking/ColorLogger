/**
 ******************************************************************************
 ** �t�@�C���� : app.cpp
 **
 ** �T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/HRP2�pC++�T���v���v���O����
 **
 ** ���L : sample_cpp (���C���g���[�X/�K�����[�^/�����g�Z���T/�����[�g�X�^�[�g)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "TouchSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "Logger.h"

using namespace ev3api;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
#define GYRO_OFFSET 0
#define LIGHT_WHITE 40 /* ���F�̌��Z���T�l */
#define LIGHT_BLACK 0  /* ���F�̌��Z���T�l */
#define TAIL_ANGLE 30  /* ���S��~���̊p�x[�x] */
#define P_GAIN 2.5F    /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX 60 /* ���S��~�p���[�^����PWM��΍ő�l */

/* LCD�t�H���g�T�C�Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6 /*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8 /*TODO: magic number*/)

#define SPEED_CALC_INTERVAL 200 // 速度調節の動作周期 [ms]
#define TASK_INTERVAL 4

static void tail_control(int32_t angle);
void lcddraw(int height, int value);
void motorCalib();

/* �I�u�W�F�N�g�ւ̃|�C���^��` */
TouchSensor *touchSensor;
ColorSensor *colorSensor;
Motor *tailMotor;
Clock *clock;
Logger *logger;

/* ���C���^�X�N */
void main_task(intptr_t unused)
{
    /* �e�I�u�W�F�N�g�𐶐��E���������� */
    touchSensor = new TouchSensor(PORT_4);
    colorSensor = new ColorSensor(PORT_2);
    tailMotor = new Motor(PORT_D);
    clock = new Clock();
    logger = new Logger("jack_color.log");

    /* LCD��ʕ\�� */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("color sensor logger", 0, CALIB_FONT_HEIGHT * 1);

    /* �K�����[�^�[�̃��Z�b�g */
    tailMotor->reset();

    ev3_led_set_color(LED_ORANGE); /* �����������ʒm */

    motorCalib();

    /* �X�^�[�g�ҋ@ */
    while (1)
    {

        tail_control(TAIL_ANGLE); /* ���S��~�p�p�x�ɐ��� */

        if (touchSensor->isPressed())
        {
            break; /* �^�b�`�Z���T�������ꂽ */
        }

        clock->sleep(10);
    }

    ev3_led_set_color(LED_GREEN); /* �X�^�[�g�ʒm */

    /**
    * Main loop for the self-balance control algorithm
    */

    clock->reset();

    while (1)
    {
        if (ev3_button_is_pressed(BACK_BUTTON))
            break;

        tail_control(TAIL_ANGLE);

        rgb_raw_t rgb;
        colorSensor->getRawColor(rgb);
        int brightness = (0.299 * rgb.r) + (0.587 * rgb.g) + (0.114 * rgb.b);

        int H = 0;
        int S = 0;
        int max = 0;
        int min = 0;
        colorid_t max_rgb = COLOR_NONE;

        //rgbの最小値計算
        if (rgb.r <= rgb.g && rgb.r <= rgb.b)
        {
            min = rgb.r;
        }

        if (rgb.g <= rgb.r && rgb.g <= rgb.b)
        {
            min = rgb.g;
        }

        if (rgb.b <= rgb.r && rgb.b <= rgb.g)
        {
            min = rgb.b;
        }

        //rgbの最大値計算
        if (rgb.r >= rgb.g && rgb.r >= rgb.b)
        {
            max = rgb.r;
            max_rgb = COLOR_RED;
        }

        if (rgb.g >= rgb.r && rgb.g >= rgb.b)
        {
            max = rgb.g;
            max_rgb = COLOR_GREEN;
        }

        if (rgb.b >= rgb.g && rgb.b >= rgb.r)
        {
            max = rgb.b;
            max_rgb = COLOR_BLUE;
        }

        if (rgb.r == rgb.g && rgb.g == rgb.b)
        {
            max_rgb = COLOR_NONE;
        }

        //色相を計算
        switch (max_rgb)
        {
        case COLOR_RED:
            H = ((int)(60 * ((rgb.g - rgb.b) / (double)(max - min))) + 360) % 360;
            break;
        case COLOR_GREEN:
            H = ((int)(60 * ((rgb.b - rgb.r) / (double)(max - min))) + 120) % 360;
            break;
        case COLOR_BLUE:
            H = ((int)(60 * ((rgb.r - rgb.g) / (double)(max - min))) + 240) % 360;
            break;
        case COLOR_NONE:
            H = 0;
        default:
            break;
        }

        // 彩度を計算
        S = max - min;

        int color = COLOR_NONE;

        //閾値で色判定
        if (S < 50)
        {
            color = COLOR_BLACK;
        }
        else if (H >= 0 && H < 30)
        {
            color = COLOR_RED;
        }
        else if (H >= 60 && H < 76)
        {
            color = COLOR_YELLOW;
        }
        else if (H >= 100 && H < 130)
        {
            color = COLOR_GREEN;
        }
        else if (H >= 155 && H < 210)
        {
            color = COLOR_BLUE;
        }


        clock->sleep(4);

        /*
         * lcd print
         */
        lcddraw(3, brightness);
        lcddraw(4, H);
        lcddraw(5, S);
        lcddraw(6, color);
        /*
         * カラーセンサロガー
         */
        //logger->logging(colorSensor->getBrightness());
    }

    ext_tsk();
}

//*****************************************************************************
// �֐��� : tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
static void tail_control(int32_t angle)
{
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* ��ᐧ�� */
    /* PWM�o�͖O�a���� */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);
}

void lcddraw(int height, int value)
{
    char buf[4];
    snprintf(buf, 4, "%d", value);
    ev3_lcd_draw_string("           ", 0, CALIB_FONT_HEIGHT * height);
    ev3_lcd_draw_string(buf, 0, CALIB_FONT_HEIGHT * height);
}

void motorCalib()
{
    int prevArmMotorCount = -10;
    while (tailMotor->getCount() - prevArmMotorCount != 0)
    {
        tailMotor->setPWM(-10);
        prevArmMotorCount = tailMotor->getCount();
        clock->sleep(100);
    }

    tailMotor->reset();
}
