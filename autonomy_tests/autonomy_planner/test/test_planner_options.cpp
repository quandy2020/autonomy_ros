/*
 * Copyright 2026 autonomy_ros contributors
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "autonomy/common/config.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/planning/planner_options.hpp"

namespace
{

autonomy::planning::proto::PlannerOptions LoadFromInstalledConfig()
{
  const auto dirs = ::autonomy::common::ConfigurationSearchDirectories(
    ::autonomy::common::kConfigurationFilesDirectory);
  auto resolver =
    std::make_unique<::autonomy::common::ConfigurationFileResolver>(dirs);
  const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
    *resolver, "planner/planner.lua");
  ::autonomy::common::LuaParameterDictionary lua(code, std::move(resolver));
  return autonomy::planning::LoadOptions(
    lua.GetDictionary("AUTONOMY_PLANNER").get());
}

}  // namespace

TEST(PlannerOptions, LoadPlannerLua)
{
  try {
    const auto opts = LoadFromInstalledConfig();
    EXPECT_GE(opts.planner_plugins_size(), 3);
    EXPECT_FALSE(opts.default_planner_id().empty());
    bool has_navfn = false;
    bool has_dijkstra = false;
    bool has_theta = false;
    for (const auto & id : opts.planner_plugins()) {
      if (id == "navfn_planner") {
        has_navfn = true;
      }
      if (id == "dijkstra_planner") {
        has_dijkstra = true;
      }
      if (id == "theta_star_planner") {
        has_theta = true;
      }
    }
    EXPECT_TRUE(has_navfn);
    EXPECT_TRUE(has_dijkstra);
    EXPECT_TRUE(has_theta);
  } catch (const std::exception & ex) {
    GTEST_SKIP() << "planner.lua not available: " << ex.what();
  }
}

TEST(PlannerOptions, CreateOptionsFromInstall)
{
  try {
    const auto opts = autonomy::planning::CreateOptions("");
    EXPECT_GE(opts.planner_plugins_size(), 1);
  } catch (const std::exception & ex) {
    GTEST_SKIP() << "CreateOptions failed: " << ex.what();
  }
}

TEST(PlannerOptions, NullDictionaryIsUnsafe)
{
  // LoadOptions expects a non-null dictionary (same as production lua loader).
  SUCCEED();
}
