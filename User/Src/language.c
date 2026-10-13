/**
 * @file language.c
 * @brief Multi-language String Implementation
 */

#include "language.h"

/*============================================================================
 * Private Variables
 *============================================================================*/

/* Current language */
static LanguageType current_lang = MULTILANG_ENGLISH;

/*============================================================================
 * String Tables
 *============================================================================*/

/* English strings */
static const char* strings_en[LANG_STRING_COUNT] = {
    /* Unused index 0 (STR_ID = 0) */
    "",
    /* ADC Channel Names */
    "TX Power 1",           /* 发射功率1 */
    "Reflected 1",         /* 反射功率1 */
    "TX Power 2",          /* 发射功率2 */
    "Reflected 2",         /* 反射功率2 */
    "RF Input",            /* RF输入功率 */
    "Main Current",        /* 主电流 */
    "Main Voltage",        /* 主电压 */
    "NTC Temp",            /* 核心NTC温度 */
    "Output Sample",       /* 输出检波 */

    /* Power states */
    "Idle",
    "Standby",
    "Transmit",
    "Ready",

    /* Measurements */
    "TX Power",
    "Reflected",
    "SWR",
    "Voltage",
    "Current",
    "Temperature",
    "Band",

    /* Alerts */
    "Alert",
    "SWR Too High",
    "Reflected High",
    "Current Too High",
    "Hardware Fault",
    "Overdrive",
    "Freq Mismatch",
    "Alarm",

    /* Buttons/Actions */
    "Dismiss",
    "OK",
    "Cancel",
    "Save",
    "Calibrate",

    /* Calibration */
    "Touch Calibration",
    "Cal Point",
    "Press Point",
    "Cal Success",
    "Cal Failed",
    "Retry",

    /* Menu items */
    "Settings",
    "Language",
    "Radio Type",
    "Radio Baud",

    /* Power */
    "Power On",
    "Power Off",
    "Power",

    /* Directional coupler */
    "Coupler Cal",
    "Connect Load",
    "Connect Antenna"
};

/* Chinese strings */
static const char* strings_zh[LANG_STRING_COUNT] = {
    /* Unused index 0 (STR_ID = 0) */
    "",
    /* ADC Channel Names */
    "发射功率1",
    "反射功率1",
    "发射功率2",
    "反射功率2",
    "RF输入功率",
    "主电流",
    "主电压",
    "核心NTC温度",
    "输出检波",

    /* Power states */
    "待机",
    "预备",
    "发射",
    "就绪",

    /* Measurements */
    "发射功率",
    "反射功率",
    "驻波比",
    "电压",
    "电流",
    "温度",
    "波段",

    /* Alerts */
    "警报",
    "驻波比过高",
    "反射功率过高",
    "电流过高",
    "硬件故障",
    "过驱动",
    "频率不匹配",
    "报警",

    /* Buttons/Actions */
    "解除",
    "确定",
    "取消",
    "保存",
    "校准",

    /* Calibration */
    "触屏校准",
    "校准点",
    "请按此点",
    "校准成功",
    "校准失败",
    "重试",

    /* Menu items */
    "设置",
    "语言",
    "电台类型",
    "电台波特率",

    /* Power */
    "开机",
    "关机",
    "电源",

    /* Directional coupler */
    "耦合器校准",
    "连接假负载",
    "连接天线"
};

/*============================================================================
 * Public Functions
 *============================================================================*/

/**
 * @brief Get string by ID in current language
 */
const char* lang_get_string(StringId id)
{
    if (id >= LANG_STRING_COUNT) {
        return "";
    }

    if (current_lang == MULTILANG_CHINESE) {
        return strings_zh[id];
    }

    return strings_en[id];
}

/**
 * @brief Get English string
 */
const char* lang_get_string_en(StringId id)
{
    if (id >= LANG_STRING_COUNT) {
        return "";
    }
    return strings_en[id];
}

/**
 * @brief Get Chinese string
 */
const char* lang_get_string_zh(StringId id)
{
    if (id >= LANG_STRING_COUNT) {
        return "";
    }
    return strings_zh[id];
}

/**
 * @brief Set language
 */
void lang_set_language(LanguageType lang)
{
    current_lang = lang;
}

/**
 * @brief Get current language
 */
LanguageType lang_get_language(void)
{
    return current_lang;
}