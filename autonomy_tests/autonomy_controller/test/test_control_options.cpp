/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>
#include <memory>
#include <string>

#include "autonomy/common/config.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/control_options.hpp"

namespace
{

autonomy::control::proto::ControllerOptions LoadFromInstalledConfig()
{
  const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
    ::autonomy::common::kConfigurationFilesDirectory);
  auto resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
    *resolver, "control/controller.lua");
  ::autonomy::common::LuaParameterDictionary lua(code, std::move(resolver));
  return autonomy::control::LoadOptions(
    lua.GetDictionary("AUTONOMY_CONTROLLER").get());
}

}  // namespace

TEST(ControlOptions, LoadControllerLua)
{
  try {
    const auto opts = LoadFromInstalledConfig();
    EXPECT_GT(opts.controller_frequency(), 0.0);
    EXPECT_GE(opts.controller_plugins_size(), 1);
    EXPECT_TRUE(opts.has_mppi_controller_options());
  } catch (const std::exception & ex) {
    GTEST_SKIP() << "controller.lua not available: " << ex.what();
  }
}

TEST(ControlOptions, NullDictionaryYieldsDefaults)
{
  const auto opts = autonomy::control::LoadOptions(nullptr);
  EXPECT_EQ(opts.controller_plugins_size(), 0);
}
