#include "app_config.h"

/*
 * 比赛预设集中表（UTF-8）
 * 保分方案只是普通预设。可直接增删条目，app_race_preset_count会自动更新。
 */
const app_race_preset_t app_race_presets[] =
{
    {"SAFE_ALL", {
        {1U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_CLASSIC},
        {1U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_IMAGE_ONLY},
        {1U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_BOMB_IMAGE}}},
    {"NORMAL_ALL", {
        {1U, APP_RACE_STRATEGY_NORMAL, 120U, APP_RACE_ALGO_CLASSIC},
        {1U, APP_RACE_STRATEGY_NORMAL, 120U, APP_RACE_ALGO_IMAGE_ONLY},
        {1U, APP_RACE_STRATEGY_NORMAL, 120U, APP_RACE_ALGO_BOMB_IMAGE}}},
    {"SPRINT_ALL", {
        {1U, APP_RACE_STRATEGY_SPRINT, 150U, APP_RACE_ALGO_CLASSIC},
        {1U, APP_RACE_STRATEGY_SPRINT, 150U, APP_RACE_ALGO_IMAGE_ONLY},
        {1U, APP_RACE_STRATEGY_SPRINT, 150U, APP_RACE_ALGO_BOMB_IMAGE}}},
    {"ROUND1_ONLY", {
        {1U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_CLASSIC},
        {0U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_IMAGE_ONLY},
        {0U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_BOMB_IMAGE}}},
    {"ROUND12", {
        {1U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_CLASSIC},
        {1U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_IMAGE_ONLY},
        {0U, APP_RACE_STRATEGY_SAFE,   120U, APP_RACE_ALGO_BOMB_IMAGE}}}
};

const uint8 app_race_preset_count =
    (uint8)(sizeof(app_race_presets) / sizeof(app_race_presets[0]));

typedef char app_race_preset_count_must_not_exceed_six[
    ((sizeof(app_race_presets) / sizeof(app_race_presets[0])) <=
      APP_RACE_PRESET_MAX) ? 1 : -1];

/* 三档速度的四方向制动参数。需要赛道微调时只改这里。 */
const app_point_brake_profile_t app_point_brake_profiles[] =
{
    {100U, 50U, 30U, 47U, 20U}, /* 已验证基础档 */
    {120U, 40U, 25U, 37U, 15U}, /* 当前正式比赛默认档 */
    {150U, 55U, 35U, 52U, 25U}  /* 冲刺档，保持偏保守 */
};

const uint8 app_point_brake_profile_count =
    (uint8)(sizeof(app_point_brake_profiles) /
            sizeof(app_point_brake_profiles[0]));
