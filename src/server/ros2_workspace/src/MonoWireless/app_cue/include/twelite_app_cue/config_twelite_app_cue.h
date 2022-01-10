/**
 * @file config_twelite_app_cue.h
 * @brief TWELITE® CUE converterのコンフィグ
 * @date 2022-01-10
 *
 * @copyright Copyright (c) 2022-.
 *               MaSiRo Project.
 *
 */
#ifndef _CONFIG_TWELITE_APP_CUE_H_
#define _CONFIG_TWELITE_APP_CUE_H_

#include <string>

namespace TWELITE
{
namespace app_cue
{
const std::string CONFIG_TWELITE_APP_CUE_TOPIC_NAME = "/MonoWireless/TWELITE/app_cue";
const int CONFIG_TWELITE_APP_CUE_QOS                = 255;
} // namespace app_cue
} // namespace TWELITE

#endif